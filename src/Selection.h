#pragma once
#include "HitTest.h"
#include <vector>
#include <algorithm>

namespace shitcad {

struct SelectedEntity {
    HitType type = HitType::None;
    EntityID id = NullID;
};

enum class SelectionDragMode : uint8_t {
    None,
    PointDrag,
    DimDrag,
    BoxSelect,
    LassoSelect,
};

struct SelectionState {
    std::vector<SelectedEntity> selected;

    // Drag state
    SelectionDragMode dragMode = SelectionDragMode::None;
    Point2D dragAnchor = {};          // box select anchor (local coords)
    Point2D dragAnchorScreen = {};    // box select anchor (screen coords)
    EntityID dragPointID = NullID;    // point being dragged
    EntityID dragDimID = NullID;      // dimension being dragged
    bool dragStarted = false;         // true after first drag movement (for undo)
    std::vector<Point2D> lassoPoints; // lasso polygon vertices

    // --- Multi-selection methods ---

    void addToSelection(HitType t, EntityID id) {
        if (!isSelected(t, id))
            selected.push_back({t, id});
    }

    void removeFromSelection(HitType t, EntityID id) {
        selected.erase(
            std::remove_if(selected.begin(), selected.end(),
                [t, id](const SelectedEntity& e) { return e.type == t && e.id == id; }),
            selected.end());
    }

    void toggleSelection(HitType t, EntityID id) {
        if (isSelected(t, id))
            removeFromSelection(t, id);
        else
            addToSelection(t, id);
    }

    bool isSelected(HitType t, EntityID id) const {
        for (const auto& e : selected)
            if (e.type == t && e.id == id) return true;
        return false;
    }

    void select(HitType t, EntityID id) {
        selected.clear();
        selected.push_back({t, id});
        dragMode = SelectionDragMode::None;
        dragStarted = false;
    }

    void clear() {
        selected.clear();
        dragMode = SelectionDragMode::None;
        dragPointID = NullID;
        dragDimID = NullID;
        dragStarted = false;
        lassoPoints.clear();
    }

    bool hasSelection() const { return !selected.empty(); }

    // Backward-compat: returns first selected entity ID (or NullID)
    EntityID entityID() const {
        return selected.empty() ? NullID : selected[0].id;
    }

    // Backward-compat: returns first selected type (or None)
    HitType type() const {
        return selected.empty() ? HitType::None : selected[0].type;
    }
};

} // namespace shitcad
