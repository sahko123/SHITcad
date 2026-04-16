#pragma once
#include "SketchData.h"
#include <vector>

namespace shitcad {

class History {
public:
    static constexpr int kMaxSnapshots = 100;

    void pushState(const Sketch& sketch);
    bool undo(Sketch& sketch);
    bool redo(Sketch& sketch);
    bool canUndo() const { return current_ > 0; }
    bool canRedo() const { return current_ + 1 < (int)snapshots_.size(); }
    void clear();

private:
    std::vector<Sketch> snapshots_;
    int current_ = -1;
};

} // namespace shitcad
