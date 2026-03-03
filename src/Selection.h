#pragma once
#include "HitTest.h"

namespace shitcad {

struct SelectionState {
    HitType type = HitType::None;
    EntityID entityID = NullID;
    bool isDragging = false;
    bool dragStarted = false; // true after first drag movement (for undo)

    void select(HitType t, EntityID id) {
        type = t;
        entityID = id;
        isDragging = false;
        dragStarted = false;
    }

    void clear() {
        type = HitType::None;
        entityID = NullID;
        isDragging = false;
        dragStarted = false;
    }

    bool hasSelection() const { return type != HitType::None; }
    bool isPointSelected() const { return type == HitType::Point; }
    bool isLineSelected() const { return type == HitType::Line; }
    bool isCircleSelected() const { return type == HitType::Circle; }
};

} // namespace shitcad
