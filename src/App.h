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
#include "FacePicker.h"
#include "MeshImport.h"
#include "SimProcess.h"
#include "SimResults.h"
#include "ViewportInput.h"
#include "Overlay2D.h"
#include "AppHost.h"
#include <functional>

namespace shitcad {

enum class InteractionMode : uint8_t {
    Navigate,
    Sketching,
};

// Top-level workspace, chosen from the toolbar. Model is the CAD app as it
// always was; Simulation sets up, runs and views a spray simulation on the
// same geometry. Only switchable from Navigate with no tool active.
enum class Workspace : uint8_t {
    Model,
    Simulation,
};

// ---- Front-end model -------------------------------------------------------
// What a panel shows and what its controls do, independent of the toolkit that
// draws it. The ImGui toolbar and the Qt toolbar both read ToolbarModel and
// call App::perform, so the two cannot drift apart. (Phase 5 of
// docs/qt-migration-plan.md: each panel gets this as it moves to Qt.)
enum class UiAction : uint8_t {
    // sketch toolbar
    FinishSketch, SelectTool /* arg: ToolType */, Extrude, Revolve, Loft, SnapView,
    ApplyConstraint /* arg: index into App::kToolbarConstraints */, ExportDxf,
    // model / simulation toolbar
    SetWorkspace /* arg: Workspace */, Save, Open, ImportStep, ImportIges, ImportStl,
    ExportStep, ExportIges, ExportStl, ExportObj, ToggleSection, Union, Subtract,
    // everywhere
    ToggleOrtho, TogglePrefs,
};

struct ToolbarModel {
    enum class Variant : uint8_t { Sketch, Model, Simulation };
    Variant variant = Variant::Model;
    ToolType tool = ToolType::None;
    Workspace workspace = Workspace::Model;
    bool canSwitchWorkspace = false;
    bool extrudeActive = false, revolveActive = false, loftActive = false;
    bool unionActive = false, subtractActive = false;
    bool ortho = false;
    bool sectionOn = false;
    size_t bodyCount = 0;
    std::string sketchPlaneName;
    bool constraintValid[9] = {};   // parallel to App::kToolbarConstraints

    bool operator==(const ToolbarModel& o) const;
    bool operator!=(const ToolbarModel& o) const { return !(*this == o); }
};

class App {
public:
    // Called by the host once a GL context is current and an ImGui context
    // (with fonts) exists, before the ImGui backends are initialised.
    bool init(AppHost* host, float dpiScale);
    // One frame: runs posted commands, reads input, builds the UI. The host
    // has begun the ImGui frame.
    void frame(float dt, int framebufferW, int framebufferH);
    // Draws the 3D scene into the current framebuffer, then hands this
    // frame's overlays to ImGui. The host calls ImGui::Render() after it.
    void paint();
    // Frees App's GL resources. The host tears down ImGui and the window after.
    void shutdown();

    // Queue an operation to run at the start of the next frame, when the GL
    // context is current and no frame is half-built. Panels that live outside
    // the ImGui frame (Qt, later) must change App state only through this.
    void post(std::function<void()> fn);
    bool hasPosted() const { return !posted_.empty(); }

    FrameProfiler& profiler() { return profiler_; }

    // Front-end model (see UiAction above).
    struct ToolbarConstraint { const char* label; ConstraintType type; };
    static constexpr int kToolbarConstraintCount = 9;
    static const ToolbarConstraint kToolbarConstraints[kToolbarConstraintCount];
    static constexpr int kSketchToolCount = 10;
    static const ToolType kSketchTools[kSketchToolCount];
    static const char* const kSketchToolLabels[kSketchToolCount];

    ToolbarModel toolbarModel() const;
    void perform(UiAction action, int arg = 0);

    // Preferences: the struct is the model. setPreferences applies the side
    // effects of a change (theme reset and grid rebuild for light mode, the
    // profile detector backend).
    const Preferences& preferences() const { return prefs_; }
    void setPreferences(const Preferences& p);
    bool preferencesOpen() const { return prefsOpen_; }
    void setPreferencesOpen(bool open) { prefsOpen_ = open; }

    // ---- Mesh import dialog: STL has no units, so the user confirms one.
    struct MeshImportModel {
        bool open = false;
        std::string path, error, name;
        size_t triangles = 0;
        int unitIndex = 1;               // into kUnits
        float extMm[3] = {};             // size in the chosen unit, in mm
        float maxExtMm = 0.0f;
        bool sizeSuspicious = false;     // outside what cip-sim will trace
    };
    MeshImportModel meshImportModel() const;
    void setMeshImportName(const std::string& name);
    void setMeshImportUnit(int unitIndex);
    void confirmMeshImport();            // adds the feature, then opens placement
    void cancelMeshImport();

    // ---- Mesh placement panel: edits apply live, Done records one undo step.
    struct MeshPlaceModel {
        bool active = false;
        std::string name, error;         // error: file unreadable / unit unknown
        int unitIndex = 1;
        double lo[3] = {}, hi[3] = {};   // placed bounds, mm
        double pos[3] = {};              // translation being edited
        float angleDeg = 45.0f;
        int angleAxis = 2;
    };
    MeshPlaceModel meshPlaceModel() const;
    void meshPlaceSetUnit(int unitIndex);
    void meshPlaceRotate(int axis, double degrees);   // about the mesh's centre
    void meshPlaceSetAngle(float degrees, int axis) { meshPlace_.angleDeg = degrees; meshPlace_.angleAxis = axis; }
    void meshPlaceSetPosition(const double pos[3]);
    void meshPlaceDropToGround();
    void meshPlaceCentreOnOrigin();
    void meshPlaceResetPlacement();
    void finishMeshPlace(bool keep);

    // ---- Add Reference Plane: offset from a plane, or from a picked face.
    struct AddPlaneModel {
        bool open = false;
        bool fromFace = false;
        bool waitingFace = false;      // told to click a face, none picked yet
        int sourceIndex = 0;           // into the planes below
        std::string offsetText, name;
        struct Source { int index; std::string name; };
        std::vector<Source> sources;   // reference planes to offset from
        bool canCreate = false;
    };
    AddPlaneModel addPlaneModel() const;
    void openAddPlaneDialog();
    void setAddPlaneSource(bool fromFace, int planeIndex);
    void setAddPlaneOffsetText(const std::string& text);
    void setAddPlaneName(const std::string& name);
    void createOffsetPlane();
    void cancelAddPlane();

    // ---- Tangent plane on a cylinder (opened by picking a cylindrical face).
    struct TangentPlaneModel {
        bool open = false;
        float angleDeg = 0.0f;
        std::string name;
    };
    TangentPlaneModel tangentPlaneModel() const;
    void setTangentPlaneAngle(float degrees);
    void setTangentPlaneName(const std::string& name);
    void createTangentPlane();         // creates it and starts a sketch on it
    void cancelTangentPlane();

    // ---- Object tree: reference planes, sketches and bodies, each with a
    // visibility check. Double-clicking a plane or sketch edits its sketch.
    struct ObjectTreeModel {
        bool open = false;
        struct Row {
            uint64_t key;              // stable while the row means the same thing
            int index;                 // plane index, or body index for bodies
            std::string label;
            bool visible;
        };
        std::vector<Row> planes;       // reference planes
        std::vector<Row> sketches;     // planes that have sketch geometry
        std::vector<Row> bodies;
    };
    ObjectTreeModel objectTreeModel() const;
    bool objectTreeOpen() const { return objectTreeOpen_; }
    void setObjectTreeOpen(bool open) { objectTreeOpen_ = open; }
    void setPlaneVisible(int planeIndex, bool visible);
    void setSketchVisible(int planeIndex, bool visible);
    void setBodyVisible(int bodyIndex, bool visible);
    void editPlaneSketch(int planeIndex); // enterSketchMode, ignoring a stale index

    // ---- Tool panels. Typed values are applied as the ImGui fields apply
    // them, on Enter or when the field loses focus; an invalid value puts
    // back the last good one.
    struct ExtrudePanelModel {
        bool open = false;
        int operation = 0;             // ExtrudeOperation
        bool cutAllowed = false;       // Cut needs a body to cut
        std::string distanceText, offsetText;
        int direction = 0;             // ExtrudeDirection
        int selectedProfiles = 0, totalProfiles = 0;
        bool canCommit = false;
    };
    ExtrudePanelModel extrudePanelModel() const;
    void setExtrudeOperation(int op);
    void setExtrudeDistanceText(const std::string& text);
    void setExtrudeDirection(int dir);
    void setExtrudeOffsetText(const std::string& text);
    void commitExtrude();
    void cancelExtrude();

    struct RevolvePanelModel {
        bool open = false;
        int operation = 0;             // ExtrudeOperation
        std::string angleText;
        uint32_t axisLine = 0;         // EntityID, 0 while none is picked
        bool selectingAxis = false;
        int selectedProfiles = 0, totalProfiles = 0;
        bool canCommit = false;
    };
    RevolvePanelModel revolvePanelModel() const;
    void setRevolveOperation(int op);
    void setRevolveAngleText(const std::string& text);
    void pickRevolveAxis(bool clearCurrent);   // next line clicked becomes the axis
    void pickRevolveProfiles();
    void commitRevolve();
    void cancelRevolve();

    struct LoftPanelModel {
        bool open = false;
        struct Section { int plane, profile, profileCount; };
        std::vector<Section> sections;
        struct Candidate { int plane; std::string label; };
        std::vector<Candidate> candidates;   // sketch planes with profiles, not yet used
        bool solid = true;
        bool canCommit = false;
    };
    LoftPanelModel loftPanelModel() const;
    void addLoftSection(int planeIndex);
    void removeLoftSection(int section);
    void setLoftSectionProfile(int section, int profile);
    void setLoftSolid(bool solid);
    void commitLoft();
    void cancelLoft();

    struct BooleanPanelModel {
        bool open = false;
        bool isUnion = true;
        int target = -1, tool = -1;    // body indices, -1 until picked
        bool previewValid = false;
        bool canCommit = false;
    };
    BooleanPanelModel booleanPanelModel() const;
    void clearBooleanTarget();         // clears the tool body too
    void clearBooleanTool();
    void commitBoolean();
    void cancelBoolean();

    // ---- Section view: one plane cutting the whole scene. A view setting,
    // not saved and not undone.
    struct SectionModel {
        bool windowOpen = false;       // the Model workspace's Section window
        bool enabled = false;
        int axis = 1;                  // 0 X, 1 Y, 2 Z
        bool flip = false, cap = true;
        float position = 0.0f;         // mm, clamped to lo..hi
        bool outOfRange = false;       // the plane is outside lo..hi (the scene shrank)
        float lo = 0.0f, hi = 0.0f;    // the scene's extent along the axis
        int openSurfaces = 0, closedSurfaces = 0;   // visible bodies, for the cap note
    };
    SectionModel sectionModel();       // not const: scene bounds are cached
    void setSectionWindowOpen(bool open) { sectionWindowOpen_ = open; }
    void setSectionEnabled(bool on);   // turning it on centres the plane
    void setSectionAxis(int axis);     // a new axis centres the plane
    void setSectionFlip(bool flip) { section_.flip = flip; }
    void setSectionPosition(float mm); // clamped to the scene
    void centreSection();
    void setSectionCap(bool cap) { section_.cap = cap; }

    // Panels the host draws itself (Qt); App then skips their ImGui versions.
    enum HostPanel : uint32_t {
        HostToolbar     = 1u << 0,
        HostPreferences = 1u << 1,
        HostMeshImport  = 1u << 2,
        HostMeshPlace   = 1u << 3,
        HostAddPlane    = 1u << 4,
        HostTangentPlane = 1u << 5,
        HostObjectTree  = 1u << 6,
        HostToolPanels  = 1u << 7,     // extrude, revolve, loft, boolean
        HostSection     = 1u << 8,     // the Model workspace's Section window
    };
    void setHostPanel(HostPanel p, bool on = true) { hostPanels_ = on ? (hostPanels_ | p) : (hostPanels_ & ~p); }
    bool hostHas(HostPanel p) const { return (hostPanels_ & p) != 0; }

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
    AppHost* host_ = nullptr;
    uint32_t hostPanels_ = 0;             // HostPanel bits
    float imguiToolbarHeight() const;     // 0 when the host provides the toolbar
    int fbW_ = 0, fbH_ = 0;               // framebuffer size for this frame
    std::vector<std::function<void()>> posted_;
    void framebufferSize(int& w, int& h) const { w = fbW_; h = fbH_; }
    double nowSeconds() const;            // monotonic, for durations
    // A file or folder from the user, UTF-8, empty if cancelled. `title`
    // overrides the dialog's title (folder pickers say what to pick).
    std::string chooseFile(FileDialog kind, const char* title = nullptr);
    InteractionMode mode_ = InteractionMode::Navigate;

    // This frame's input for the 3D view, filled once at the top of
    // renderFrame(). Viewport handlers read this, never the GUI toolkit.
    InputFrame in_;
    // Screen-space drawing over the 3D view for this frame, recorded by the
    // overlay code and drawn once after the scene (see Overlay2D.h).
    Overlay2D overlay_;

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
    FilletToolState filletTool_;
    int lastSketchDof_ = 0;
    SnapEngine snapEngine_;
    EntityID hvCrossEntityID_ = NullID; // line/circle/arc crossed by the H/V rail this frame
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

    // Mesh import dialog: STL has no units, so the user confirms one while
    // seeing the size it implies before the feature is created.
    struct MeshImportDialogState {
        bool open = false;
        std::string path;
        char nameBuf[128] = {};
        int unitIndex = 1; // index into kUnits; 1 = mm
        MeshFileInfo info;
        std::string error;
        void reset() { *this = {}; }
    };
    MeshImportDialogState meshImportDialog_;

    // Placement panel for an imported mesh (rotate / move it into place).
    // Edits apply live; Done records one undo step, Cancel restores.
    struct MeshPlaceState {
        FeatureID featureID = NullFeatureID;
        MeshImportFeatureData original;
        double posBuf[3] = {0, 0, 0};
        float angleDeg = 45.0f;
        int angleAxis = 2; // 0 X, 1 Y, 2 Z
        void reset() { *this = {}; }
        bool active() const { return featureID != NullFeatureID; }
    };
    MeshPlaceState meshPlace_;

    // Hover pick on imported meshes (navigate mode), shown as a readout
    MeshPickResult meshHover_;
    float meshHoverMouse_[2] = {-1, -1};

    // Simulation workspace
    Workspace workspace_ = Workspace::Model;
    SimulationSetup simulation_;
    // Last state recorded on the undo stack. Widgets edit simulation_ live;
    // commitSimulationEdit() pushes one undo step from this base, so a slider
    // drag is one step rather than one per frame.
    SimulationSetup simUndoBase_;
    struct SimUiState {
        bool placing = false;          // next click on a surface places a nozzle
        uint32_t selectedNozzle = 0;
        float standoffMm = 20.0f;      // how far inside the surface a placed nozzle sits
        std::string message;           // last export/validation message
        bool messageIsError = false;
        std::vector<std::string> warnings;
        float sceneExtentMm = 1000.0f; // cached for cone display length
        size_t sceneExtentKey = 0;
    };
    SimUiState simUi_;
    GLuint simLineVAO_ = 0;
    GLuint simLineVBO_ = 0;

    // Where the simulation engine lives. Per machine, not per project, so it
    // is kept in %APPDATA%\SHITcad\simulation.json rather than the project.
    struct SimEngineSettings {
        std::string cipSimPath;        // cip-sim repository root
        std::string python = "python";
        bool loaded = false;
    };
    SimEngineSettings simEngine_;

    // A run in progress or just finished. The runner is polled every frame,
    // whichever workspace is showing, so switching tabs does not stall a run.
    enum class SimPhase : uint8_t { Idle, Running, Done, Failed, Cancelled };
    SimPhase simPhase_ = SimPhase::Idle;
    ProcessRunner simRunner_;
    std::string simRunDir_;
    std::string simRunInputs_;          // inputs digest the run was started with
    std::vector<std::string> simRunLog_;
    std::string simRunError_;
    double simRunStart_ = 0.0;
    double simRunEnd_ = 0.0;
    RunSummary simSummary_;

    ProcessRunner paraviewLauncher_;
    std::string paraviewMessage_;

    // Results drawn on the geometry.
    struct SimResultView {
        bool loaded = false;
        bool show = true;
        ResultMesh mesh;
        int field = 0;
        GLuint vao = 0, vboGeom = 0, vboColour = 0;
        int vertexCount = 0;
        bool colourDirty = true;
        std::string inputs;             // everything the run depended on (see runInputs)
        std::vector<uint32_t> coveredFeatures; // meshes these results are drawn over
        bool closed = false;            // can the cut face be capped?
        RunSummary summary;
        std::string runDir;
    };
    SimResultView simView_;
    ShaderProgram resultShader_;

    // Section (cut) view through the whole scene, in either workspace.
    SectionPlane section_;
    bool sectionWindowOpen_ = false;
    size_t sceneBoundsKey_ = (size_t)-1;   // not 0: that is a real key for an empty scene
    float sceneLo_[3] = {0, 0, 0};
    float sceneHi_[3] = {0, 0, 0};
    GLuint capVAO_ = 0, capVBO_ = 0;

    // Preferences
    Preferences prefs_;
    bool prefsOpen_ = false;
    float dpiScale_ = 1.0f;

    // Frame profiler
    FrameProfiler profiler_;

    // Transient sketch status message (shown as overlay for sketchMsgTimer_ seconds)
    char sketchMsg_[128] = {};
    float sketchMsgTimer_ = 0.0f;

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
    void drawInlineDimInput(Sketch& sketch);   // value box while drawing a circle / fillet
    void drawSketchMessage();                  // transient warning at the bottom of the view
    void handleToolAction(Sketch& sketch, Point2D localPos);
    void switchTool(ToolType newTool);
    void handleSelection(Sketch& sketch, bool ctrlHeld = false);
    void handleDrag(Sketch& sketch);
    void handleDeletion(Sketch& sketch);
    void drawDimensionPanel(Sketch& sketch);
    void handleDimToolClick(Sketch& sketch);
    void drawPreferencesWindow();
    void drawAddPlaneDialog();
    void drawTangentPlaneDialog();
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
    void editExtrudeFeature(FeatureID id);
    void handleRevolveInput(float vpW, float vpH);
    void drawRevolvePanel();
    void updateRevolvePreview();
    void renderRevolvePreview(const float* view, const float* proj, const float* eyePos);
    void editRevolveFeature(FeatureID id);
    void handleLoftInput(float vpW, float vpH);
    void drawLoftPanel();
    void updateLoftPreview();
    void renderLoftPreview(const float* view, const float* proj, const float* eyePos);
    void editLoftFeature(FeatureID id);
    void handleBooleanInput(float vpW, float vpH);
    void drawBooleanPanel();
    void updateBooleanPreview();
    void renderBooleanPreview(const float* view, const float* proj, const float* eyePos);
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
    void beginMeshImport(const std::string& path);
    void drawMeshImportDialog();
    void editMeshImportFeature(FeatureID id);
    void drawMeshPlacePanel();
    void setMeshImportData(const MeshImportFeatureData& data); // live edit + replay
    // Bounds of the mesh being placed, from its drawn triangles when available.
    bool meshPlaceBounds(MeshImportFeatureData& data, double lo[3], double hi[3], std::string* error) const;
    void validateMeshPlace();             // drop the panel if its feature went away
    void updateMeshHover(float vpW, float vpH);
    // Simulation workspace (AppSimulation.cpp)
    bool canSwitchWorkspace() const;
    void setWorkspace(Workspace w);
    void drawSimulationPanel();
    void handleSimulationInput(float vpW, float vpH);
    void renderSimulationOverlay(const float* view, const float* proj);
    void commitSimulationEdit();
    void exportSimulationSpecDialog();
    float simulationSceneExtent();
    void loadEngineSettings();
    void saveEngineSettings();
    std::string engineProblem() const; // empty if the engine looks usable
    void startTier1Run();
    void pollSimulationRun();
    bool loadSimulationResults(const RunSummary& summary, const std::string& inputs,
                               const std::string& runDir, std::string& error);
    // Everything a run's numbers depend on, as one string: the spec, the run
    // settings (which are NOT in the spec), and a stamp of every STL on disk.
    // Comparing this is how "these results are stale" is decided.
    bool runInputs(std::string& inputs, std::string& error) const;
    void clearSimulationRun();
    void releaseSimulationResults();
    void renderSimulationResults(const float* view, const float* proj, const float* eyePos);
    void drawSimulationRunSection();
    void drawSimulationResultsSection();
    void openResultsInParaView();
    // Section view (AppSection.cpp)
    void sceneBounds(float lo[3], float hi[3]);
    void drawSectionControls();
    void renderSectionCap(const float* view, const float* proj, bool resultsShown);
    void drawMeshHoverReadout();
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
