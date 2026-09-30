#pragma once
#include <string>
#include <vector>

namespace shitcad {

// A child process whose stdout is read line by line without threads.
//
// The app is single-threaded, so instead of blocking reads this is polled once
// per frame: poll() drains whatever the child has written to both pipes and
// returns the complete stdout lines. Both pipes are always drained - a child
// that fills an unread stderr pipe (Python/VTK warnings) blocks forever.
//
// Windows only, like the rest of the file I/O layer.
class ProcessRunner {
public:
    ProcessRunner() = default;
    ~ProcessRunner();
    ProcessRunner(const ProcessRunner&) = delete;
    ProcessRunner& operator=(const ProcessRunner&) = delete;

    // `killWithApp`: put the child in a job object that is killed when this
    // process exits, so a long run does not outlive a closed or crashed app.
    // Leave false for launchers whose children must survive (a viewer).
    bool start(const std::vector<std::string>& argv, const std::string& cwd,
               bool killWithApp, std::string& error);

    // Append complete stdout lines produced since the last call. Updates running().
    void poll(std::vector<std::string>& lines);

    bool started() const { return process_ != nullptr || finished_; }
    bool running() const { return running_; }
    bool finished() const { return finished_; }
    int exitCode() const { return exitCode_; }
    const std::string& stderrTail() const { return stderr_; } // last ~16 KB
    void cancel();

    // Windows command-line quoting for one argument (CommandLineToArgvW rules).
    static std::string quoteArg(const std::string& arg);

private:
    void* process_ = nullptr;
    void* job_ = nullptr;
    void* outRead_ = nullptr;
    void* errRead_ = nullptr;
    bool killWithApp_ = false;
    bool running_ = false;
    bool finished_ = false;
    int exitCode_ = 0;
    std::string outBuf_;
    std::string stderr_;

    void drain(void* pipe, std::string& into, bool isStderr);
    void closeAll();
};

} // namespace shitcad
