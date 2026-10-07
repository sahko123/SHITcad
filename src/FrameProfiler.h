#pragma once
#include <chrono>
#include <cstdio>

namespace shitcad {

// Lightweight per-frame profiler. Records section timings and logs when
// total frame time exceeds the budget (default 33ms = 30fps).
// Supports nesting — each begin() allocates a slot and pushes onto a stack,
// end() pops and records the duration.
//
// Usage:
//   profiler.beginFrame();
//   profiler.begin("Outer");
//     profiler.begin("Inner");
//     profiler.end();  // closes "Inner"
//   profiler.end();    // closes "Outer"
//   profiler.endFrame();

class FrameProfiler {
public:
    static constexpr int kMaxSections = 32;
    static constexpr int kMaxDepth = 8;
    static constexpr float kDefaultBudgetMs = 33.33f; // 30fps

    void beginFrame() {
        sectionCount_ = 0;
        stackDepth_ = 0;
        frameStart_ = now();
    }

    void begin(const char* name) {
        if (sectionCount_ >= kMaxSections || stackDepth_ >= kMaxDepth) return;
        int idx = sectionCount_++;
        sections_[idx].name = name;
        sections_[idx].depth = stackDepth_;
        sections_[idx].startTime = now();
        stack_[stackDepth_++] = idx;
    }

    void end() {
        if (stackDepth_ <= 0) return;
        int idx = stack_[--stackDepth_];
        sections_[idx].durationMs = elapsedMs(sections_[idx].startTime, now());
    }

    void endFrame() {
        float totalMs = elapsedMs(frameStart_, now());
        if (totalMs < budgetMs_) return;

        // Over budget — log the breakdown
        fprintf(stderr, "[PERF] Frame %.1fms (budget %.0fms):", totalMs, budgetMs_);
        for (int i = 0; i < sectionCount_; i++) {
            if (sections_[i].durationMs >= 0.5f) { // only show sections >= 0.5ms
                fprintf(stderr, "  ");
                // Indent nested sections
                for (int d = 0; d < sections_[i].depth; d++) fprintf(stderr, ">");
                fprintf(stderr, "%s=%.1f", sections_[i].name, sections_[i].durationMs);
            }
        }
        fprintf(stderr, "\n");
    }

private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    static TimePoint now() { return Clock::now(); }
    static float elapsedMs(TimePoint a, TimePoint b) {
        return std::chrono::duration<float, std::milli>(b - a).count();
    }

    struct Section {
        const char* name = "";
        TimePoint startTime;
        float durationMs = 0;
        int depth = 0;
    };

    Section sections_[kMaxSections];
    int sectionCount_ = 0;
    int stack_[kMaxDepth] = {};
    int stackDepth_ = 0;
    TimePoint frameStart_;
    float budgetMs_ = kDefaultBudgetMs;
};

} // namespace shitcad
