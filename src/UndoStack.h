#pragma once
#include "FeatureHistory.h"
#include <vector>
#include <utility>

namespace shitcad {

enum class UndoActionType : uint8_t {
    AddFeature,
    DeleteFeature,
    SuppressFeature,
    UnsuppressFeature,
    RenameFeature,
    SetRollbackPos,
    ModifySketch,
    ModifyExtrude,
    ModifyRevolve,
    ModifyLoft,
    ModifyBoolean,
};

struct UndoCommand {
    UndoActionType type = UndoActionType::AddFeature;
    FeatureID featureID = NullFeatureID;

    // RenameFeature
    std::string oldName;
    std::string newName;

    // SetRollbackPos
    int oldRollbackPos = -1;
    int newRollbackPos = -1;

    // DeleteFeature: saved features with their original indices (sorted ascending)
    std::vector<std::pair<int, Feature>> deletedFeatures;

    // AddFeature: copy of the feature that was added
    Feature addedFeature;

    // ModifySketch
    Sketch oldSketch;
    Sketch newSketch;

    // ModifyExtrude
    ExtrudeFeatureData oldExtrude;
    ExtrudeFeatureData newExtrude;

    // ModifyRevolve
    RevolveFeatureData oldRevolve;
    RevolveFeatureData newRevolve;

    // ModifyLoft
    LoftFeatureData oldLoft;
    LoftFeatureData newLoft;

    // ModifyBoolean
    BooleanFeatureData oldBoolean;
    BooleanFeatureData newBoolean;
};

class UndoStack {
public:
    void push(UndoCommand cmd);
    bool canUndo() const { return current_ >= 0; }
    bool canRedo() const { return current_ + 1 < (int)commands_.size(); }
    // Returns true and fills `out` with the command. Returns false if nothing to undo/redo.
    bool undoStep(UndoCommand& out);
    bool redoStep(UndoCommand& out);
    void clear();

private:
    std::vector<UndoCommand> commands_;
    int current_ = -1;
};

} // namespace shitcad
