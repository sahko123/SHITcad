#ifndef NOMINMAX
#define NOMINMAX // also set project-wide; guarded to avoid C4005
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "SimProcess.h"

#include <algorithm>

namespace shitcad {

static constexpr size_t kStderrCap = 16 * 1024;

ProcessRunner::~ProcessRunner() {
    closeAll();
}

std::string ProcessRunner::quoteArg(const std::string& arg) {
    if (!arg.empty() && arg.find_first_of(" \t\"") == std::string::npos) return arg;
    std::string out = "\"";
    size_t backslashes = 0;
    for (char c : arg) {
        if (c == '\\') {
            backslashes++;
        } else if (c == '"') {
            out.append(backslashes * 2 + 1, '\\');
            out.push_back('"');
            backslashes = 0;
        } else {
            out.append(backslashes, '\\');
            out.push_back(c);
            backslashes = 0;
        }
    }
    out.append(backslashes * 2, '\\'); // backslashes before the closing quote
    out.push_back('"');
    return out;
}

bool ProcessRunner::start(const std::vector<std::string>& argv, const std::string& cwd,
                          bool killWithApp, std::string& error) {
    closeAll();
    running_ = finished_ = false;
    exitCode_ = 0;
    outBuf_.clear();
    stderr_.clear();
    if (argv.empty()) { error = "empty command"; return false; }

    SECURITY_ATTRIBUTES sa = {sizeof(sa), nullptr, TRUE};
    HANDLE outR = nullptr, outW = nullptr, errR = nullptr, errW = nullptr;
    if (!CreatePipe(&outR, &outW, &sa, 0) || !CreatePipe(&errR, &errW, &sa, 0)) {
        error = "CreatePipe failed";
        if (outR) CloseHandle(outR);
        if (outW) CloseHandle(outW);
        return false;
    }
    // Only the write ends go to the child.
    SetHandleInformation(outR, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(errR, HANDLE_FLAG_INHERIT, 0);
    HANDLE nul = CreateFileA("NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa,
                             OPEN_EXISTING, 0, nullptr);

    std::string cmd;
    for (size_t i = 0; i < argv.size(); i++) {
        if (i) cmd.push_back(' ');
        cmd += quoteArg(argv[i]);
    }
    std::vector<char> cmdBuf(cmd.begin(), cmd.end());
    cmdBuf.push_back('\0');

    STARTUPINFOA si = {};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = outW;
    si.hStdError = errW;
    si.hStdInput = nul;
    PROCESS_INFORMATION pi = {};

    DWORD flags = CREATE_NO_WINDOW | (killWithApp ? CREATE_SUSPENDED : 0);
    BOOL ok = CreateProcessA(nullptr, cmdBuf.data(), nullptr, nullptr, TRUE, flags, nullptr,
                             cwd.empty() ? nullptr : cwd.c_str(), &si, &pi);
    CloseHandle(outW);
    CloseHandle(errW);
    if (nul != INVALID_HANDLE_VALUE) CloseHandle(nul);
    if (!ok) {
        DWORD e = GetLastError();
        CloseHandle(outR);
        CloseHandle(errR);
        error = "Could not start '" + argv[0] + "' (Windows error " + std::to_string(e) +
                (e == ERROR_FILE_NOT_FOUND ? ": not found - check the Python path" :
                 e == ERROR_DIRECTORY ? ": working directory does not exist" : "") + ")";
        return false;
    }

    if (killWithApp) {
        // Assigned before the child runs any code, so nothing it spawns escapes.
        HANDLE job = CreateJobObjectA(nullptr, nullptr);
        if (job) {
            JOBOBJECT_EXTENDED_LIMIT_INFORMATION info = {};
            info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            SetInformationJobObject(job, JobObjectExtendedLimitInformation, &info, sizeof(info));
            AssignProcessToJobObject(job, pi.hProcess);
            job_ = job;
        }
        ResumeThread(pi.hThread);
    }
    CloseHandle(pi.hThread);

    process_ = pi.hProcess;
    outRead_ = outR;
    errRead_ = errR;
    running_ = true;
    return true;
}

void ProcessRunner::drain(void* pipe, std::string& into, bool isStderr) {
    if (!pipe) return;
    char buf[8192];
    for (;;) {
        DWORD avail = 0;
        if (!PeekNamedPipe((HANDLE)pipe, nullptr, 0, nullptr, &avail, nullptr) || avail == 0) return;
        DWORD got = 0;
        if (!ReadFile((HANDLE)pipe, buf, (DWORD)std::min<DWORD>(avail, sizeof(buf)), &got, nullptr) || got == 0)
            return;
        into.append(buf, got);
        if (isStderr && into.size() > kStderrCap) into.erase(0, into.size() - kStderrCap);
    }
}

void ProcessRunner::poll(std::vector<std::string>& lines) {
    if (!process_) return;
    drain(outRead_, outBuf_, false);
    drain(errRead_, stderr_, true);

    bool exited = WaitForSingleObject((HANDLE)process_, 0) == WAIT_OBJECT_0;
    if (exited) {
        // Anything written just before exit is still in the pipes.
        drain(outRead_, outBuf_, false);
        drain(errRead_, stderr_, true);
    }

    size_t start = 0;
    for (size_t nl; (nl = outBuf_.find('\n', start)) != std::string::npos; start = nl + 1) {
        std::string line = outBuf_.substr(start, nl - start);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) lines.push_back(std::move(line));
    }
    outBuf_.erase(0, start);

    if (exited) {
        if (!outBuf_.empty()) { lines.push_back(outBuf_); outBuf_.clear(); }
        DWORD code = 0;
        GetExitCodeProcess((HANDLE)process_, &code);
        exitCode_ = (int)code;
        running_ = false;
        finished_ = true;
        closeAll();
    }
}

void ProcessRunner::cancel() {
    if (process_ && running_) {
        TerminateProcess((HANDLE)process_, 1);
        WaitForSingleObject((HANDLE)process_, 5000);
    }
    std::vector<std::string> ignored;
    poll(ignored);
}

void ProcessRunner::closeAll() {
    if (outRead_) { CloseHandle((HANDLE)outRead_); outRead_ = nullptr; }
    if (errRead_) { CloseHandle((HANDLE)errRead_); errRead_ = nullptr; }
    if (process_) { CloseHandle((HANDLE)process_); process_ = nullptr; }
    // Closing the job kills anything still in it - the point of killWithApp.
    if (job_) { CloseHandle((HANDLE)job_); job_ = nullptr; }
}

} // namespace shitcad
