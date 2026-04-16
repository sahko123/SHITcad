#pragma once

namespace shitcad {

// Initialize crash logging. Call as the very first thing in main().
// Installs handlers for: SEH exceptions (access violations, stack overflow),
// C++ std::terminate, MSVC CRT invalid parameters (debug assertions like
// "vector not seekable before begin"), SIGABRT, SIGSEGV, pure virtual calls.
// Redirects stderr to SHITcad_stderr.log next to the executable.
// Crash logs are written to SHITcad_crash.log next to the executable.
void initCrashLogger();

} // namespace shitcad
