#include "CrashLogger.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <DbgHelp.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>

#ifdef _DEBUG
#include <crtdbg.h>
#endif

#pragma comment(lib, "dbghelp.lib")

namespace shitcad {

// ---- Static state (no heap allocation, survives memory corruption) ----

static char s_crashLogPath[MAX_PATH] = {};
static char s_stderrLogPath[MAX_PATH] = {};
static bool s_initialized = false;
static bool s_handling = false; // prevent recursive crash handling

// ---- Low-level file writing (Win32 API, no CRT) ----

static void writeStr(HANDLE hFile, const char* str) {
    if (!str || hFile == INVALID_HANDLE_VALUE) return;
    DWORD written;
    WriteFile(hFile, str, (DWORD)strlen(str), &written, NULL);
}

static void writeInt(HANDLE hFile, unsigned long long val) {
    char buf[24];
    int len = 0;
    if (val == 0) { buf[0] = '0'; len = 1; }
    else {
        char tmp[24];
        int tmpLen = 0;
        while (val > 0) { tmp[tmpLen++] = '0' + (char)(val % 10); val /= 10; }
        for (int i = tmpLen - 1; i >= 0; i--) buf[len++] = tmp[i];
    }
    buf[len] = '\0';
    writeStr(hFile, buf);
}

static void writeHex(HANDLE hFile, unsigned long long val) {
    char buf[20] = "0x";
    const char* hex = "0123456789ABCDEF";
    int len = 2;
    if (val == 0) { buf[len++] = '0'; }
    else {
        char tmp[16];
        int tmpLen = 0;
        while (val > 0) { tmp[tmpLen++] = hex[val & 0xF]; val >>= 4; }
        for (int i = tmpLen - 1; i >= 0; i--) buf[len++] = tmp[i];
    }
    buf[len] = '\0';
    writeStr(hFile, buf);
}

// ---- Timestamp ----

static void writeTimestamp(HANDLE hFile) {
    SYSTEMTIME st;
    GetLocalTime(&st);
    char buf[64];
    // Manual format to avoid CRT
    int len = 0;
    auto write2 = [&](WORD v) {
        buf[len++] = '0' + (char)(v / 10);
        buf[len++] = '0' + (char)(v % 10);
    };
    // YYYY-MM-DD HH:MM:SS
    buf[len++] = '0' + (char)(st.wYear / 1000);
    buf[len++] = '0' + (char)((st.wYear / 100) % 10);
    write2((WORD)(st.wYear % 100));
    buf[len++] = '-';
    write2(st.wMonth);
    buf[len++] = '-';
    write2(st.wDay);
    buf[len++] = ' ';
    write2(st.wHour);
    buf[len++] = ':';
    write2(st.wMinute);
    buf[len++] = ':';
    write2(st.wSecond);
    buf[len] = '\0';
    writeStr(hFile, buf);
}

// ---- Stack trace capture ----

static void writeStackTrace(HANDLE hFile, CONTEXT* ctx) {
    HANDLE process = GetCurrentProcess();
    HANDLE thread = GetCurrentThread();

    if (ctx) {
        // Use StackWalk64 for precise trace from exception context
        STACKFRAME64 frame = {};
#ifdef _M_X64
        DWORD machineType = IMAGE_FILE_MACHINE_AMD64;
        frame.AddrPC.Offset = ctx->Rip;
        frame.AddrPC.Mode = AddrModeFlat;
        frame.AddrFrame.Offset = ctx->Rbp;
        frame.AddrFrame.Mode = AddrModeFlat;
        frame.AddrStack.Offset = ctx->Rsp;
        frame.AddrStack.Mode = AddrModeFlat;
#elif defined(_M_IX86)
        DWORD machineType = IMAGE_FILE_MACHINE_I386;
        frame.AddrPC.Offset = ctx->Eip;
        frame.AddrPC.Mode = AddrModeFlat;
        frame.AddrFrame.Offset = ctx->Ebp;
        frame.AddrFrame.Mode = AddrModeFlat;
        frame.AddrStack.Offset = ctx->Esp;
        frame.AddrStack.Mode = AddrModeFlat;
#else
        // Fallback: use CaptureStackBackTrace
        ctx = nullptr;
#endif
        if (ctx) {
            for (int i = 0; i < 64; i++) {
                if (!StackWalk64(machineType, process, thread, &frame, ctx,
                                 NULL, SymFunctionTableAccess64, SymGetModuleBase64, NULL))
                    break;

                if (frame.AddrPC.Offset == 0) break;

                writeStr(hFile, "  [");
                writeInt(hFile, i);
                writeStr(hFile, "] ");
                writeHex(hFile, frame.AddrPC.Offset);

                // Resolve symbol name
                char symbolBuf[sizeof(SYMBOL_INFO) + 256];
                SYMBOL_INFO* sym = (SYMBOL_INFO*)symbolBuf;
                memset(symbolBuf, 0, sizeof(symbolBuf));
                sym->SizeOfStruct = sizeof(SYMBOL_INFO);
                sym->MaxNameLen = 255;

                DWORD64 displacement64 = 0;
                if (SymFromAddr(process, frame.AddrPC.Offset, &displacement64, sym)) {
                    writeStr(hFile, " ");
                    writeStr(hFile, sym->Name);
                    writeStr(hFile, " + ");
                    writeHex(hFile, displacement64);
                }

                // Resolve file:line
                IMAGEHLP_LINE64 line = {};
                line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
                DWORD displacement32 = 0;
                if (SymGetLineFromAddr64(process, frame.AddrPC.Offset, &displacement32, &line)) {
                    writeStr(hFile, " (");
                    writeStr(hFile, line.FileName);
                    writeStr(hFile, ":");
                    writeInt(hFile, line.LineNumber);
                    writeStr(hFile, ")");
                }

                writeStr(hFile, "\r\n");
            }
            return;
        }
    }

    // Fallback: CaptureStackBackTrace (no CONTEXT available)
    void* stack[62];
    USHORT frames = CaptureStackBackTrace(2, 62, stack, NULL);

    for (USHORT i = 0; i < frames; i++) {
        DWORD64 addr = (DWORD64)stack[i];

        writeStr(hFile, "  [");
        writeInt(hFile, i);
        writeStr(hFile, "] ");
        writeHex(hFile, addr);

        char symbolBuf[sizeof(SYMBOL_INFO) + 256];
        SYMBOL_INFO* sym = (SYMBOL_INFO*)symbolBuf;
        memset(symbolBuf, 0, sizeof(symbolBuf));
        sym->SizeOfStruct = sizeof(SYMBOL_INFO);
        sym->MaxNameLen = 255;

        DWORD64 displacement64 = 0;
        if (SymFromAddr(process, addr, &displacement64, sym)) {
            writeStr(hFile, " ");
            writeStr(hFile, sym->Name);
            writeStr(hFile, " + ");
            writeHex(hFile, displacement64);
        }

        IMAGEHLP_LINE64 line = {};
        line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
        DWORD displacement32 = 0;
        if (SymGetLineFromAddr64(process, addr, &displacement32, &line)) {
            writeStr(hFile, " (");
            writeStr(hFile, line.FileName);
            writeStr(hFile, ":");
            writeInt(hFile, line.LineNumber);
            writeStr(hFile, ")");
        }

        writeStr(hFile, "\r\n");
    }
}

// ---- Core crash log writer ----

static void writeCrashLog(const char* type, const char* detail, CONTEXT* ctx) {
    if (s_handling) {
        // Recursive crash during handler — just die
        TerminateProcess(GetCurrentProcess(), 99);
        return;
    }
    s_handling = true;

    HANDLE hFile = CreateFileA(s_crashLogPath, GENERIC_WRITE, FILE_SHARE_READ,
                                NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE) {
        TerminateProcess(GetCurrentProcess(), 98);
        return;
    }

    writeStr(hFile, "=== SHITcad Crash Log ===\r\n");
    writeStr(hFile, "Time: ");
    writeTimestamp(hFile);
    writeStr(hFile, "\r\n");
    writeStr(hFile, "Type: ");
    writeStr(hFile, type);
    writeStr(hFile, "\r\n");

    if (detail && detail[0]) {
        writeStr(hFile, "Detail: ");
        writeStr(hFile, detail);
        writeStr(hFile, "\r\n");
    }

    writeStr(hFile, "\r\nStack Trace:\r\n");
    writeStackTrace(hFile, ctx);

    writeStr(hFile, "\r\n==============================\r\n");

    FlushFileBuffers(hFile);
    CloseHandle(hFile);

    // Also try to write to stderr (may or may not work)
    fprintf(stderr, "\n[CRASH] %s: %s\nSee crash log: %s\n",
            type, detail ? detail : "", s_crashLogPath);
    fflush(stderr);
}

// ---- SEH exception name lookup ----

static const char* sehCodeToString(DWORD code) {
    switch (code) {
        case EXCEPTION_ACCESS_VIOLATION:      return "Access Violation";
        case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "Array Bounds Exceeded";
        case EXCEPTION_BREAKPOINT:            return "Breakpoint";
        case EXCEPTION_DATATYPE_MISALIGNMENT: return "Datatype Misalignment";
        case EXCEPTION_FLT_DIVIDE_BY_ZERO:    return "Float Divide By Zero";
        case EXCEPTION_FLT_OVERFLOW:          return "Float Overflow";
        case EXCEPTION_FLT_UNDERFLOW:         return "Float Underflow";
        case EXCEPTION_ILLEGAL_INSTRUCTION:   return "Illegal Instruction";
        case EXCEPTION_INT_DIVIDE_BY_ZERO:    return "Integer Divide By Zero";
        case EXCEPTION_INT_OVERFLOW:          return "Integer Overflow";
        case EXCEPTION_INVALID_DISPOSITION:   return "Invalid Disposition";
        case EXCEPTION_PRIV_INSTRUCTION:      return "Privileged Instruction";
        case EXCEPTION_STACK_OVERFLOW:        return "Stack Overflow";
        case EXCEPTION_IN_PAGE_ERROR:         return "In-Page Error";
        default:                              return "Unknown SEH Exception";
    }
}

// ---- Handler: SEH (access violations, stack overflow, etc.) ----

static LONG WINAPI sehHandler(EXCEPTION_POINTERS* ep) {
    DWORD code = ep->ExceptionRecord->ExceptionCode;

    // Don't intercept C++ exceptions or debugger breakpoints in debug builds
    if (code == 0xE06D7363) // MSVC C++ exception
        return EXCEPTION_CONTINUE_SEARCH;

    const char* name = sehCodeToString(code);

    // Build detail string with address info
    char detail[256] = {};
    if (code == EXCEPTION_ACCESS_VIOLATION && ep->ExceptionRecord->NumberParameters >= 2) {
        const char* op = (ep->ExceptionRecord->ExceptionInformation[0] == 0) ? "read" : "write";
        ULONG_PTR addr = ep->ExceptionRecord->ExceptionInformation[1];
        // Manual snprintf without CRT
        char addrBuf[20];
        int pos = 0;
        const char* hex = "0123456789ABCDEF";
        addrBuf[pos++] = '0'; addrBuf[pos++] = 'x';
        for (int i = (sizeof(ULONG_PTR) * 2) - 1; i >= 0; i--)
            addrBuf[pos++] = hex[(addr >> (i * 4)) & 0xF];
        addrBuf[pos] = '\0';

        int dLen = 0;
        auto appendStr = [&](const char* s) {
            while (*s && dLen < 250) detail[dLen++] = *s++;
        };
        appendStr(op);
        appendStr(" of address ");
        appendStr(addrBuf);
        detail[dLen] = '\0';
    }

    writeCrashLog(name, detail, ep->ContextRecord);
    TerminateProcess(GetCurrentProcess(), code);
    return EXCEPTION_EXECUTE_HANDLER; // unreachable
}

// ---- Handler: std::terminate ----

static void terminateHandler() {
    writeCrashLog("std::terminate", "Unhandled C++ exception or explicit terminate() call", nullptr);
    TerminateProcess(GetCurrentProcess(), 3);
}

// ---- Handler: Signals (SIGABRT, SIGSEGV) ----

static void signalHandler(int sig) {
    const char* name = "Unknown Signal";
    switch (sig) {
        case SIGABRT: name = "SIGABRT (abort)"; break;
        case SIGSEGV: name = "SIGSEGV (segfault)"; break;
        case SIGFPE:  name = "SIGFPE (floating point)"; break;
        case SIGILL:  name = "SIGILL (illegal instruction)"; break;
    }
    writeCrashLog(name, "Signal raised — may be from a debug assertion or abort() call", nullptr);
    TerminateProcess(GetCurrentProcess(), sig);
}

// ---- Handler: MSVC CRT invalid parameter (debug assertions) ----

static void invalidParameterHandler(
    const wchar_t* expression,
    const wchar_t* function,
    const wchar_t* file,
    unsigned int line,
    uintptr_t /*reserved*/)
{
    // Convert wide strings to narrow for logging
    char detail[512] = {};
    int pos = 0;
    auto appendNarrow = [&](const wchar_t* ws, int maxLen) {
        if (!ws) return;
        for (int i = 0; i < maxLen && ws[i] && pos < 500; i++)
            detail[pos++] = (char)(ws[i] & 0x7F);
    };
    auto appendStr = [&](const char* s) {
        while (*s && pos < 500) detail[pos++] = *s++;
    };

    appendStr("Expression: ");
    appendNarrow(expression, 200);
    appendStr("\nFunction: ");
    appendNarrow(function, 100);
    appendStr("\nFile: ");
    appendNarrow(file, 100);
    if (line > 0) {
        appendStr("\nLine: ");
        char lineBuf[16];
        int lLen = 0;
        unsigned int tmp = line;
        if (tmp == 0) { lineBuf[0] = '0'; lLen = 1; }
        else {
            char rev[16]; int rLen = 0;
            while (tmp > 0) { rev[rLen++] = '0' + (char)(tmp % 10); tmp /= 10; }
            for (int i = rLen - 1; i >= 0; i--) lineBuf[lLen++] = rev[i];
        }
        lineBuf[lLen] = '\0';
        appendStr(lineBuf);
    }
    detail[pos] = '\0';

    writeCrashLog("CRT Invalid Parameter (Debug Assertion)", detail, nullptr);
    TerminateProcess(GetCurrentProcess(), 3);
}

// ---- Handler: Pure virtual call ----

static void purecallHandler() {
    writeCrashLog("Pure Virtual Function Call", "Called a pure virtual function on a destroyed or uninitialized object", nullptr);
    TerminateProcess(GetCurrentProcess(), 3);
}

// ---- Handler: CRT debug report hook (captures assertion message text) ----

#ifdef _DEBUG
static char s_lastAssertMsg[512] = {};

static int crtReportHook(int reportType, char* message, int* returnValue) {
    (void)reportType;
    if (message) {
        // Save the message for the signal/abort handler to use
        strncpy(s_lastAssertMsg, message, sizeof(s_lastAssertMsg) - 1);
        s_lastAssertMsg[sizeof(s_lastAssertMsg) - 1] = '\0';
    }

    // Log immediately in case abort follows
    const char* typeStr = "CRT Debug Assertion";
    if (reportType == _CRT_ERROR) typeStr = "CRT Error";
    else if (reportType == _CRT_WARN) typeStr = "CRT Warning";

    // Only log errors and assertions, not warnings
    if (reportType != _CRT_WARN) {
        writeCrashLog(typeStr, message ? message : "(no message)", nullptr);
    }

    if (returnValue) *returnValue = 0; // Don't trigger debugger break
    return 1; // 1 = handled, suppress default CRT dialog
}
#endif

// ---- Public API ----

void initCrashLogger() {
    if (s_initialized) return;
    s_initialized = true;

    // Build log paths next to executable
    char exePath[MAX_PATH];
    GetModuleFileNameA(NULL, exePath, MAX_PATH);

    // Find last backslash to get directory
    char* lastSlash = strrchr(exePath, '\\');
    if (!lastSlash) lastSlash = strrchr(exePath, '/');
    if (lastSlash) {
        size_t dirLen = lastSlash - exePath + 1;
        memcpy(s_crashLogPath, exePath, dirLen);
        strcpy(s_crashLogPath + dirLen, "SHITcad_crash.log");
        memcpy(s_stderrLogPath, exePath, dirLen);
        strcpy(s_stderrLogPath + dirLen, "SHITcad_stderr.log");
    } else {
        strcpy(s_crashLogPath, "SHITcad_crash.log");
        strcpy(s_stderrLogPath, "SHITcad_stderr.log");
    }

    // Redirect stderr to file
    freopen(s_stderrLogPath, "w", stderr);
    setvbuf(stderr, NULL, _IONBF, 0); // unbuffered

    // Write startup marker to stderr log
    fprintf(stderr, "=== SHITcad started ===\n");
    {
        SYSTEMTIME st;
        GetLocalTime(&st);
        fprintf(stderr, "Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    }
    fprintf(stderr, "Crash log path: %s\n\n", s_crashLogPath);

    // Initialize debug symbols for stack traces
    SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES);
    SymInitialize(GetCurrentProcess(), NULL, TRUE);

    // Install SEH handler (access violations, stack overflow, etc.)
    SetUnhandledExceptionFilter(sehHandler);

    // Install C++ terminate handler
    std::set_terminate(terminateHandler);

    // Install MSVC CRT invalid parameter handler
    _set_invalid_parameter_handler(invalidParameterHandler);

    // Disable default abort dialog so our signal handler runs instead
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);

    // Install signal handlers
    signal(SIGABRT, signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGFPE, signalHandler);
    signal(SIGILL, signalHandler);

    // Install pure virtual call handler
    _set_purecall_handler(purecallHandler);

#ifdef _DEBUG
    // Hook into CRT debug reporting to capture assertion messages
    // and suppress the default dialog
    _CrtSetReportHook2(_CRT_RPTHOOK_INSTALL, crtReportHook);

    // Also set report mode to not show dialog boxes
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
#endif
}

} // namespace shitcad
