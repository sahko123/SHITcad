#include "History.h"

namespace shitcad {

void History::pushState(const Sketch& sketch) {
    // Discard any redo states beyond current
    if (current_ + 1 < (int)snapshots_.size()) {
        snapshots_.erase(snapshots_.begin() + current_ + 1, snapshots_.end());
    }
    snapshots_.push_back(sketch);
    current_ = (int)snapshots_.size() - 1;
}

bool History::undo(Sketch& sketch) {
    if (!canUndo()) return false;
    current_--;
    sketch = snapshots_[current_];
    return true;
}

bool History::redo(Sketch& sketch) {
    if (!canRedo()) return false;
    current_++;
    sketch = snapshots_[current_];
    return true;
}

void History::clear() {
    snapshots_.clear();
    current_ = -1;
}

} // namespace shitcad
