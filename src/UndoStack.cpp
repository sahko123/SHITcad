#include "UndoStack.h"

namespace shitcad {

void UndoStack::push(UndoCommand cmd) {
    // Truncate any redo history beyond current position
    if (current_ + 1 < (int)commands_.size()) {
        commands_.erase(commands_.begin() + current_ + 1, commands_.end());
    }
    commands_.push_back(std::move(cmd));
    current_ = (int)commands_.size() - 1;
}

bool UndoStack::undoStep(UndoCommand& out) {
    if (!canUndo()) return false;
    out = commands_[current_--];
    return true;
}

bool UndoStack::redoStep(UndoCommand& out) {
    if (!canRedo()) return false;
    out = commands_[++current_];
    return true;
}

void UndoStack::clear() {
    commands_.clear();
    current_ = -1;
}

} // namespace shitcad
