#pragma once
#include "SketchPlane.h"
#include "SketchData.h"
#include <TopoDS_Face.hxx>
#include "Tools.h"
#include "Snap.h"
#include "History.h"
#include "Solver.h"
#include "HitTest.h"
#include "Selection.h"
#include "Viewport3D.h"
#include "Scene3D.h"
#include "SketchRenderer.h"
#include "ExtrudeTool.h"
#include "Preferences.h"
#include "FeatureHistory.h"
#include "UndoStack.h"
#include "Serialization.h"
#include "FrameProfiler.h"

struct GLFWwindow;

namespace shitcad {

enum class InteractionMode : uint8_t {
    Navigate,
    Sketching,
};

class App {
public:
    bool init();
    void run();
    void shutdown();

    InteractionMode mode() const { return mode_; }
    Scene3D& scene() { return scene_; }
    Viewport3D& viewport3D() { return viewport3D_; }

    void enterSketchMode(int planeIndex);
    void finishSketch(bool recordFeature = true);

    void enterExtrudeMode();
    bool isExtrudeActive() const { return tool_.type == ToolType::Extrude; }

    void enterRevolveMode();
    bool isRevolveActive() const { return tool_.type == ToolType::Revolve; }

    void enterLoftMode();
    bool isLoftActive() const { return tool_.type == ToolType::Loft; }

    void enterBooleanMode(BooleanOperation op);
    bool isBooleanActive() const { return tool_.type == ToolType::BooleanUnion || tool_.type == ToolType::BooleanSubtract; }

    static constexpr int kXYPlane = 0;
    static constexpr int kXZPlane = 1;
    static constexpr int kYZPlane = 2;
    static constexpr int kRefPlaneCount = 3;

private:
    GLFWwindow* window_ = nullptr;
    InteractionMode mode_ = InteractionMode::Navigate;

    // 3D
    Viewport3D viewport3D_;
    Scene3D scene_;
    SketchRenderer sketchRenderer_;

    // Sketch planes (first 3 are reference planes)
    std::vector<SketchPlane> sketchPlanes_;
    PlaneID nextPlaneID_ = 1;
    int activeSketchPlane_ = -1;

    // Sketch editing state
    ToolState tool_;
    ArcToolState arcTool_;
    SnapEngine snapEngine_;
    History history_;
    Solver solver_;
    SnapResult currentSnap_;
    SelectionState selection_;
    Point2D cursorLocal_ = {};

    // Camera animation
    bool cameraAnimating_ = false;
    float cameraAnimT_ = 0.0f;
    OrbitCamera cameraFrom_;
    OrbitCamera cameraTo_;

    // Extrude tool
    ExtrudeToolState extrudeTool_;

    // Revolve tool
    RevolveToolState revolveTool_;

    // Loft tool
    LoftToolState loftTool_;

    // Boolean tool
    BooleanToolState booleanTool_;

    // Feature history
    FeatureHistory featureHistory_;
    UndoStack globalUndo_;
    bool timelineOpen_ = true;
    int dragStartRollbackPos_ = -1; // for playhead drag undo
    bool playheadReplayPending_ = false; // deferred replay during drag
    std::chrono::steady_clock::time_point playheadLastMoveTime_; // for idle detection
    FeatureID selectedFeatureID_ = NullFeatureID; // timeline selection

    // Save/load
    std::string currentFilePath_;
    bool unsavedChanges_ = false;

    // Object tree sidebar
    bool objectTreeOpen_ = true;

    // Offset plane creation dialog
    bool addPlaneDialogOpen_ = false;
    int addPlaneSourceIndex_ = 0;   // index into sketchPlanes_ or -1 for face pick
    float addPlaneOffset_ = 10.0f;
    char addPlaneOffsetBuf_[32] = "10.0";
    char addPlaneNameBuf_[64] = "";
    bool addPlaneFromFace_ = false;  // true = pick face mode
    bool addPlaneWaitingFace_ = false; // waiting for user to click a face
    SketchPlane addPlaneFaceSource_;   // holds face plane data when face is picked

    // Cylinder tangent plane dialog
    bool cylPlaneDialogOpen_ = false;
    float cylPlaneAngle_ = 0.0f;
    char cylPlaneAngleBuf_[32] = "0";
    char cylPlaneNameBuf_[64] = "";
    TopoDS_Face cylPlaneFace_;
    float cylPlaneHitWorld_[3] = {};
    int cylPlaneBodyIndex_ = -1;

    // Preferences
    Preferences prefs_;
    bool prefsOpen_ = false;
    float dpiScale_ = 1.0f;

    // Frame profiler
    FrameProfiler profiler_;

    // Dimension label bounding boxes (rebuilt each frame during renderDimensions)
    struct DimLabelRect {
        EntityID constraintID = NullID;
        int sketchPlaneIndex = -1;
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    };
    std::vector<DimLabelRect> dimLabelRects_;

    // Dimension tool state
    struct DimToolState {
        enum Phase { Selecting, Editing, EditingAndPlacing };
        Phase phase = Selecting;
        HitType selType = HitType::None;
        EntityID entityA = NullID;
        EntityID entityB = NullID;
        float measuredMm = 0;
        char inputBuf[64] = {};
        bool focusNeeded = false;
        bool driven = false;
        bool editingExisting = false;
        EntityID constraintID = NullID;
        bool placingFirstFrame = false; // skip click on the frame we enter EditingAndPlacing
        bool angleAutoSide = false;    // true = mouse side controls angle (acute vs reflex)
        float angleBase = 0;           // unsigned angle (0-180) for auto-side computation
        char warningMsg[128] = {};
        float warningTimer = 0;
        void reset() { *this = {}; }
    };
    DimToolState dimTool_;

    void renderFrame();
    void render3DScene(int w, int h);
    void drawToolbar();
    void drawObjectTree();
    void handleNavigateInput(float vpW, float vpH);
    void handleSketchInput(float vpW, float vpH);
    void handleToolAction(Sketch& sketch, Point2D localPos);
    void switchTool(ToolType newTool);
    void handleSelection(Sketch& sketch, bool ctrlHeld = false);
    void handleDrag(Sketch& sketch);
    void handleDeletion(Sketch& sketch);
    void drawDimensionPanel(Sketch& sketch);
    void handleDimToolClick(Sketch& sketch);
    void drawPreferencesWindow();
    void drawTimeline(float panelW);
    void replayAllFeatures();
    void globalUndo();
    void globalRedo();
    void handleExtrudeInput(float vpW, float vpH);
    void drawExtrudePanel();
    void updateExtrudePreview();
    void renderExtrudePreview(const float* view, const float* proj, const float* eyePos);
    void renderExtrudeHandle(const float* view, const float* proj, float vpW, float vpH);
    void renderDimensions(const float view[16], const float proj[16], float vpW, float vpH);
    void commitExtrude();
    void cancelExtrude();
    void editExtrudeFeature(FeatureID id);
    void handleRevolveInput(float vpW, float vpH);
    void drawRevolvePanel();
    void updateRevolvePreview();
    void renderRevolvePreview(const float* view, const float* proj, const float* eyePos);
    void commitRevolve();
    void cancelRevolve();
    void editRevolveFeature(FeatureID id);
    void handleLoftInput(float vpW, float vpH);
    void drawLoftPanel();
    void updateLoftPreview();
    void renderLoftPreview(const float* view, const float* proj, const float* eyePos);
    void commitLoft();
    void cancelLoft();
    void editLoftFeature(FeatureID id);
    void handleBooleanInput(float vpW, float vpH);
    void drawBooleanPanel();
    void updateBooleanPreview();
    void renderBooleanPreview(const float* view, const float* proj, const float* eyePos);
    void commitBoolean();
    void cancelBoolean();
    void saveProjectDialog();
    void openProjectDialog();
    void exportStlDialog();
    void importStlDialog();
    void exportStepDialog();
    void importStepDialog();
    void exportIgesDialog();
    void importIgesDialog();
    void exportObjDialog();
    void exportDxfDialog();
    void importModelDialog();
    void applyGeometricConstraint(Sketch& sketch, ConstraintType type);
    void updateWindowTitle();
    void markDirty();
    void updateCameraAnimation(float dt);
    void orientCameraToPlane(const SketchPlane& plane);

    void getViewProj(int w, int h, float view[16], float proj[16]);
    bool hasActiveSketch() const { return activeSketchPlane_ >= 0 && activeSketchPlane_ < (int)sketchPlanes_.size(); }
    Sketch& activeSketch() { return sketchPlanes_[activeSketchPlane_].sketch; }
    SketchPlane& activePlane() { return sketchPlanes_[activeSketchPlane_]; }

    // Extrude can reference a sketch plane independent of sketch mode
    bool hasExtrudeSketch() const { return extrudeTool_.sketchPlaneIndex >= 0 && extrudeTool_.sketchPlaneIndex < (int)sketchPlanes_.size(); }
    Sketch& extrudeSketch() { return sketchPlanes_[extrudeTool_.sketchPlaneIndex].sketch; }
    SketchPlane& extrudePlane() { return sketchPlanes_[extrudeTool_.sketchPlaneIndex]; }

    bool hasRevolveSketch() const { return revolveTool_.sketchPlaneIndex >= 0 && revolveTool_.sketchPlaneIndex < (int)sketchPlanes_.size(); }
    Sketch& revolveSketch() { return sketchPlanes_[revolveTool_.sketchPlaneIndex].sketch; }
    SketchPlane& revolvePlane() { return sketchPlanes_[revolveTool_.sketchPlaneIndex]; }
};

} // namespace shitcad
