#include "App.h"
#include "ProfileDetector.h"
#include "Extrude.h"
#include "AutoConstraint.h"
#include "FacePicker.h"
#include "ExtrudeTool.h"
#include "FeatureReplay.h"
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "UnitUtils.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <string>

namespace shitcad {

static float lerp(float a, float b, float t) { return a + (b - a) * t; }

bool App::init() {
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    window_ = glfwCreateWindow(1280, 720, "SHITcad", nullptr, nullptr);
    if (!window_) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    int version = gladLoadGL(glfwGetProcAddress);
    if (version == 0) {
        fprintf(stderr, "Failed to initialize OpenGL loader\n");
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // DPI scaling: query monitor content scale
    float xscale = 1.0f, yscale = 1.0f;
    glfwGetWindowContentScale(window_, &xscale, &yscale);
    dpiScale_ = xscale > yscale ? xscale : yscale;
    if (dpiScale_ < 1.0f) dpiScale_ = 1.0f;

    // Load a crisp TTF font at native DPI size (no blurry bitmap scaling)
    ImGuiIO& io = ImGui::GetIO();
    float fontSize = 15.0f * dpiScale_;
    const char* fontPaths[] = {
        "C:/Windows/Fonts/segoeui.ttf",   // Segoe UI (Windows 10/11)
        "C:/Windows/Fonts/calibri.ttf",    // Calibri fallback
        "C:/Windows/Fonts/arial.ttf",      // Arial fallback
    };
    bool fontLoaded = false;
    for (const char* path : fontPaths) {
        FILE* f = fopen(path, "rb");
        if (f) {
            fclose(f);
            io.Fonts->AddFontFromFileTTF(path, fontSize);
            fontLoaded = true;
            break;
        }
    }
    if (!fontLoaded) {
        // Fall back to default bitmap font with scaling
        io.FontGlobalScale = dpiScale_;
    }

    prefs_.applyTheme();

    // Scale ImGui style for high-DPI
    ImGui::GetStyle().ScaleAllSizes(dpiScale_);

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    if (!viewport3D_.init()) {
        fprintf(stderr, "Failed to init 3D viewport\n");
    }

    if (!sketchRenderer_.init()) {
        fprintf(stderr, "Failed to init sketch renderer\n");
    }

    // Create reference planes
    sketchPlanes_.resize(3);
    nextPlaneID_ = 1;

    // XY plane (blue) — normal +Z
    sketchPlanes_[0].planeID = nextPlaneID_++;
    sketchPlanes_[0].origin[0] = 0; sketchPlanes_[0].origin[1] = 0; sketchPlanes_[0].origin[2] = 0;
    sketchPlanes_[0].normal[0] = 0; sketchPlanes_[0].normal[1] = 0; sketchPlanes_[0].normal[2] = 1;
    sketchPlanes_[0].uAxis[0]  = 1; sketchPlanes_[0].uAxis[1]  = 0; sketchPlanes_[0].uAxis[2]  = 0;
    sketchPlanes_[0].vAxis[0]  = 0; sketchPlanes_[0].vAxis[1]  = 1; sketchPlanes_[0].vAxis[2]  = 0;
    sketchPlanes_[0].name = "XY Plane";
    sketchPlanes_[0].color[0] = 0.2f; sketchPlanes_[0].color[1] = 0.2f;
    sketchPlanes_[0].color[2] = 0.8f; sketchPlanes_[0].color[3] = 0.10f;
    sketchPlanes_[0].isReferencePlane = true;

    // XZ plane (green) — normal +Y
    sketchPlanes_[1].planeID = nextPlaneID_++;
    sketchPlanes_[1].origin[0] = 0; sketchPlanes_[1].origin[1] = 0; sketchPlanes_[1].origin[2] = 0;
    sketchPlanes_[1].normal[0] = 0; sketchPlanes_[1].normal[1] = 1; sketchPlanes_[1].normal[2] = 0;
    sketchPlanes_[1].uAxis[0]  = 1; sketchPlanes_[1].uAxis[1]  = 0; sketchPlanes_[1].uAxis[2]  = 0;
    sketchPlanes_[1].vAxis[0]  = 0; sketchPlanes_[1].vAxis[1]  = 0; sketchPlanes_[1].vAxis[2]  = 1;
    sketchPlanes_[1].name = "XZ Plane";
    sketchPlanes_[1].color[0] = 0.2f; sketchPlanes_[1].color[1] = 0.8f;
    sketchPlanes_[1].color[2] = 0.2f; sketchPlanes_[1].color[3] = 0.10f;
    sketchPlanes_[1].isReferencePlane = true;

    // YZ plane (red) — normal +X
    sketchPlanes_[2].planeID = nextPlaneID_++;
    sketchPlanes_[2].origin[0] = 0; sketchPlanes_[2].origin[1] = 0; sketchPlanes_[2].origin[2] = 0;
    sketchPlanes_[2].normal[0] = 1; sketchPlanes_[2].normal[1] = 0; sketchPlanes_[2].normal[2] = 0;
    sketchPlanes_[2].uAxis[0]  = 0; sketchPlanes_[2].uAxis[1]  = 1; sketchPlanes_[2].uAxis[2]  = 0;
    sketchPlanes_[2].vAxis[0]  = 0; sketchPlanes_[2].vAxis[1]  = 0; sketchPlanes_[2].vAxis[2]  = 1;
    sketchPlanes_[2].name = "YZ Plane";
    sketchPlanes_[2].color[0] = 0.8f; sketchPlanes_[2].color[1] = 0.2f;
    sketchPlanes_[2].color[2] = 0.2f; sketchPlanes_[2].color[3] = 0.10f;
    sketchPlanes_[2].isReferencePlane = true;

    tool_.type = ToolType::Line;

    return true;
}

void App::run() {
    double lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window_)) {
        profiler_.beginFrame();

        profiler_.begin("PollEvents");
        glfwPollEvents();
        profiler_.end();

        double now = glfwGetTime();
        float dt = (float)(now - lastTime);
        lastTime = now;

        updateCameraAnimation(dt);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        profiler_.begin("UI+Input");
        renderFrame();
        profiler_.end();

        ImGui::Render();

        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);

        profiler_.begin("Render3D");
        render3DScene(w, h);
        profiler_.end();

        profiler_.begin("ImGuiDraw");
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        profiler_.end();

        profiler_.begin("SwapBuffers");
        glfwSwapBuffers(window_);
        profiler_.end();

        profiler_.recordFrameEnd();
        profiler_.endFrame();
    }
}

void App::getViewProj(int w, int h, float view[16], float proj[16]) {
    viewport3D_.camera().getViewMatrix(view);
    float aspect = (h > 0) ? (float)w / (float)h : 1.0f;
    viewport3D_.camera().getProjection(proj, aspect);
}

void App::render3DScene(int w, int h) {
    glViewport(0, 0, w, h);
    glEnable(GL_DEPTH_TEST);

    // Sync user-adjustable colors into active theme
    auto& tm = activeThemeMut();
    tm.sketchLine[0] = prefs_.sketchLineColor[0];
    tm.sketchLine[1] = prefs_.sketchLineColor[1];
    tm.sketchLine[2] = prefs_.sketchLineColor[2];

    const auto& bg = activeTheme().bgColor;
    glClearColor(bg[0], bg[1], bg[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    float view[16], proj[16];
    getViewProj(w, h, view, proj);

    // Ground grid — hidden in sketch mode since the adaptive sketch grid takes over
    if (mode_ != InteractionMode::Sketching)
        viewport3D_.drawGroundGrid(view, proj);

    // Bodies — push faces back slightly so wireframe edges render cleanly on top
    float eye[3];
    viewport3D_.camera().getEyePosition(eye);
    if (prefs_.showWireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
    }
    profiler_.begin("Bodies");
    // Results are drawn on the imported meshes' own triangles, so the meshes
    // step aside while results are showing.
    const bool resultsShown = workspace_ == Workspace::Simulation && simView_.loaded && simView_.show;
    scene_.render(viewport3D_.meshShader(), view, proj, eye, resultsShown, &section_);
    renderSimulationResults(view, proj, eye);
    renderSectionCap(view, proj, resultsShown);
    profiler_.end();
    if (prefs_.showWireframe) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        glLineWidth(prefs_.edgeThickness);
        scene_.renderEdges(viewport3D_.gridShader(), view, proj, prefs_.edgeColor, &section_);
        glLineWidth(1.0f);
    }

    // Extrude preview (translucent) + drag handle
    if (tool_.type == ToolType::Extrude && hasExtrudeSketch()) {
        profiler_.begin("ExtPreview");
        if (extrudeTool_.previewDirty) updateExtrudePreview();
        renderExtrudePreview(view, proj, eye);
        renderExtrudeHandle(view, proj, (float)w, (float)h);
        profiler_.end();
    }
    // Revolve preview
    if (tool_.type == ToolType::Revolve && hasRevolveSketch()) {
        if (revolveTool_.previewDirty) updateRevolvePreview();
        renderRevolvePreview(view, proj, eye);
    }
    // Loft preview
    if (tool_.type == ToolType::Loft) {
        if (loftTool_.previewDirty) updateLoftPreview();
        renderLoftPreview(view, proj, eye);
    }
    // Boolean preview
    if (isBooleanActive()) {
        if (booleanTool_.previewDirty) updateBooleanPreview();
        renderBooleanPreview(view, proj, eye);
    }

    renderSimulationOverlay(view, proj);

    // Reference planes (translucent) — render all planes marked as reference
    sketchRenderer_.renderReferencePlanes(sketchPlanes_.data(), (int)sketchPlanes_.size(),
                                          activeSketchPlane_, view, proj);

    // Cylinder tangent plane preview
    if (cylPlaneDialogOpen_) {
        SketchPlane previewPlane;
        if (buildCylinderTangentPlane(cylPlaneFace_, cylPlaneAngle_, cylPlaneHitWorld_, previewPlane)) {
            float color[4] = {0.2f, 0.8f, 0.4f, 0.25f};
            sketchRenderer_.renderPlanePreview(previewPlane, view, proj, color);
        }
    }

    // Render all sketch geometry (independent of reference plane visibility)
    glLineWidth(prefs_.sketchLineThickness);
    for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
        const auto& sp = sketchPlanes_[i];
        if (!sp.sketchVisible && i != activeSketchPlane_) continue;
        if (sp.sketch.points.empty() && sp.sketch.lines.empty() && sp.sketch.circles.empty())
            continue;
        bool isActive = (i == activeSketchPlane_);
        // Active sketch renders on top of all geometry so it's visible through solids
        if (isActive) glDisable(GL_DEPTH_TEST);
        sketchRenderer_.renderSketch(sp, view, proj, isActive, isActive ? selection_ : SelectionState{}, isActive ? lastSketchDof_ : 0);
        if (isActive) glEnable(GL_DEPTH_TEST);
    }
    glLineWidth(1.0f);

    // Profile highlights during extrude mode (works in both sketch and navigate mode)
    if (tool_.type == ToolType::Extrude && hasExtrudeSketch() && !extrudeTool_.allProfiles.empty()) {
        profiler_.begin("Profiles");
        const auto& sp = extrudePlane();
        int hoveredIdx = -1;
        if (!extrudeTool_.isDragging) {
            hoveredIdx = hitTestProfile(extrudeSketch(), extrudeTool_.allProfiles, cursorLocal_,
                                        extrudeTool_.renderCache);
        }
        sketchRenderer_.renderProfileHighlights(
            sp, view, proj, extrudeSketch(),
            extrudeTool_.allProfiles, extrudeTool_.selectedProfileIndices,
            hoveredIdx, extrudeTool_.renderCache);
        profiler_.end();
    }

    // Active sketch overlays (render on top of all geometry)
    if (activeSketchPlane_ >= 0) {
        glDisable(GL_DEPTH_TEST);
        glLineWidth(prefs_.sketchLineThickness);
        const auto& sp = sketchPlanes_[activeSketchPlane_];

        // Grid on active plane
        float apparentScale = computeApparentScale(sp, view, proj, (float)w, (float)h);
        float safeScale = std::max(apparentScale, 1e-9f); // never divide by zero
        float gridStep = 1.0f;
        {
            float worldSpacing = 20.0f * dpiScale_ / safeScale;
            gridStep = std::pow(10.0f, std::floor(std::log10(worldSpacing)));
            if (gridStep * safeScale < 5.0f * dpiScale_) gridStep *= 10.0f;
        }

        // Compute visible local-space bounds for the grid.
        // Primary: viewport-diagonal / apparentScale centred on the plane hit of the screen centre.
        // Secondary: screen corners are projected and expand the bounds for oblique views.
        float gStartU, gEndU, gStartV, gEndV;
        {
            // Half-extent large enough to cover the full viewport diagonal at this zoom level.
            // Use safeScale so this works at any zoom, including extreme zoom-out.
            float halfExt = std::sqrt((float)(w * w + h * h)) / safeScale;

            // Centre the extent on where the viewport centre hits the sketch plane
            float cro[3], crd[3];
            screenToRay((float)w * 0.5f, (float)h * 0.5f, 0, 0, (float)w, (float)h, view, proj, cro, crd);
            float cLx = 0.0f, cLy = 0.0f, cT;
            if (sp.rayIntersect(cro, crd, cLx, cLy, cT) && cT > 0.0f && cT < 1e6f) {
                gStartU = cLx - halfExt; gEndU = cLx + halfExt;
                gStartV = cLy - halfExt; gEndV = cLy + halfExt;
            } else {
                gStartU = -halfExt; gEndU = halfExt;
                gStartV = -halfExt; gEndV = halfExt;
            }

            // Expand with screen corner projections (handles oblique viewing angles)
            float cx[4] = {0.0f, (float)w, (float)w, 0.0f};
            float cy[4] = {0.0f, 0.0f,     (float)h, (float)h};
            for (int ci = 0; ci < 4; ci++) {
                float ro[3], rd[3];
                screenToRay(cx[ci], cy[ci], 0, 0, (float)w, (float)h, view, proj, ro, rd);
                float lx, ly, t;
                if (sp.rayIntersect(ro, rd, lx, ly, t) && t > 0.0f && t < 1e5f) {
                    gStartU = std::min(gStartU, lx); gEndU = std::max(gEndU, lx);
                    gStartV = std::min(gStartV, ly); gEndV = std::max(gEndV, ly);
                }
            }

            // Small margin so lines don't pop at the exact viewport edge
            float mU = (gEndU - gStartU) * 0.05f;
            float mV = (gEndV - gStartV) * 0.05f;
            gStartU -= mU; gEndU += mU;
            gStartV -= mV; gEndV += mV;
        }

        sketchRenderer_.renderGrid(sp, view, proj, gridStep, gStartU, gEndU, gStartV, gEndV);

        // Tool preview (use snapped position for visual feedback)
        Point2D previewCursor = (currentSnap_.type != SnapType::None)
            ? currentSnap_.position : cursorLocal_;
        sketchRenderer_.renderToolPreview(sp, view, proj, tool_, arcTool_, previewCursor);

        // Snap indicator
        sketchRenderer_.renderSnapIndicator(sp, view, proj, currentSnap_);

        // Selection overlay
        if (selection_.dragMode == SelectionDragMode::BoxSelect) {
            // Draw box in screen space using ImGui
            ImDrawList* dl = ImGui::GetForegroundDrawList();
            ImVec2 a(f(selection_.dragAnchorScreen.x), f(selection_.dragAnchorScreen.y));
            ImVec2 b = ImGui::GetIO().MousePos;
            ImVec2 mn(std::min(a.x, b.x), std::min(a.y, b.y));
            ImVec2 mx(std::max(a.x, b.x), std::max(a.y, b.y));
            dl->AddRectFilled(mn, mx, IM_COL32(0, 230, 230, 38));
            dl->AddRect(mn, mx, IM_COL32(0, 230, 230, 200), 0.0f, 0, 1.5f);
        } else if (selection_.dragMode == SelectionDragMode::LassoSelect) {
            sketchRenderer_.renderSelectionOverlay(sp, view, proj, selection_, cursorLocal_);
        }
        glLineWidth(1.0f);
        glEnable(GL_DEPTH_TEST);
    }

    glDisable(GL_DEPTH_TEST);
}

void App::renderFrame() {
    ImGuiIO& io = ImGui::GetIO();
    float vpW = io.DisplaySize.x;
    float vpH = io.DisplaySize.y;

    // Sync user-adjustable dimension colors into active theme (before renderDimensions)
    {
        auto& tm = activeThemeMut();
        auto toU32 = [](const float c[4]) -> ImU32 {
            return IM_COL32((int)(c[0]*255), (int)(c[1]*255), (int)(c[2]*255), (int)(c[3]*255));
        };
        tm.dimLineColor = toU32(prefs_.dimLineCol);
        tm.dimTextColor = toU32(prefs_.dimTextCol);
        tm.dimBgColor = toU32(prefs_.dimBgCol);
    }

    // Toggle object tree with T key (only in navigate mode)
    if (!io.WantCaptureKeyboard && mode_ == InteractionMode::Navigate &&
        ImGui::IsKeyPressed(ImGuiKey_T)) {
        objectTreeOpen_ = !objectTreeOpen_;
    }

    // Toggle ortho/perspective with O key
    if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_O)) {
        viewport3D_.camera().orthographic = !viewport3D_.camera().orthographic;
    }

    // Toolbar — auto-fit height
    float toolbarH = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({vpW, toolbarH});
    ImGui::Begin("##toolbar", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing);
    drawToolbar();
    ImGui::End();

    // Object tree sidebar (positioned below toolbar)
    float panelW = 0.0f;
    if (objectTreeOpen_) {
        panelW = 200.0f;
        drawObjectTree();
    }

    // Handle input (below toolbar, right of object tree)
    float inputY = toolbarH;
    float inputH = vpH - inputY;

    viewport3D_.handleInput(panelW, inputY, vpW - panelW, inputH);

    // Route extrude/revolve input regardless of mode
    profiler_.begin("Input");
    if (tool_.type == ToolType::Extrude && hasExtrudeSketch()) {
        handleExtrudeInput(vpW, vpH);
    } else if (tool_.type == ToolType::Revolve && hasRevolveSketch()) {
        handleRevolveInput(vpW, vpH);
    } else if (tool_.type == ToolType::Loft) {
        handleLoftInput(vpW, vpH);
    } else if (isBooleanActive()) {
        handleBooleanInput(vpW, vpH);
    } else if (mode_ == InteractionMode::Navigate || !hasActiveSketch()) {
        if (workspace_ == Workspace::Simulation)
            handleSimulationInput(vpW, vpH);
        else
            handleNavigateInput(vpW, vpH);
    } else {
        handleSketchInput(vpW, vpH);
    }
    // Same condition that routes input to handleNavigateInput above. Not
    // `tool_.type == None`: init() leaves the sketch Line tool selected while
    // in Navigate mode, which would hide the readout for the whole session.
    bool navigating = mode_ == InteractionMode::Navigate &&
                      tool_.type != ToolType::Extrude && tool_.type != ToolType::Revolve &&
                      tool_.type != ToolType::Loft && !isBooleanActive();
    // In the Simulation workspace the hover pick only matters while placing.
    if (navigating && (workspace_ == Workspace::Model || simUi_.placing))
        updateMeshHover(vpW, vpH);
    else
        meshHover_ = {};
    profiler_.end();

    // Extrude panel
    if (tool_.type == ToolType::Extrude && hasExtrudeSketch()) drawExtrudePanel();
    if (tool_.type == ToolType::Revolve && hasRevolveSketch()) drawRevolvePanel();
    if (tool_.type == ToolType::Loft) drawLoftPanel();
    if (isBooleanActive()) drawBooleanPanel();

    // Dimension panel
    if (tool_.type == ToolType::Dimension && activeSketchPlane_ >= 0) drawDimensionPanel(activeSketch());

    // Preferences window
    if (prefsOpen_) drawPreferencesWindow();

    // Cylinder tangent plane dialog
    if (cylPlaneDialogOpen_) {
        ImGui::SetNextWindowSize({280, 0}, ImGuiCond_Always);
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, {0.5f, 0.5f});
        ImGui::Begin("Tangent Plane", &cylPlaneDialogOpen_,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Text("Create a tangent plane on cylinder");
        ImGui::Separator();

        ImGui::Text("Name:");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##cylname", cylPlaneNameBuf_, sizeof(cylPlaneNameBuf_));

        ImGui::Text("Angle (degrees from click point):");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##cylangle", cylPlaneAngleBuf_, sizeof(cylPlaneAngleBuf_),
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            cylPlaneAngle_ = (float)atof(cylPlaneAngleBuf_);
        }
        ImGui::SliderFloat("##cylangleslider", &cylPlaneAngle_, -180.0f, 180.0f, "%.1f deg");
        snprintf(cylPlaneAngleBuf_, sizeof(cylPlaneAngleBuf_), "%.1f", cylPlaneAngle_);

        ImGui::Separator();
        if (ImGui::Button("Create & Sketch", {-1, 0})) {
            SketchPlane newPlane;
            if (buildCylinderTangentPlane(cylPlaneFace_, cylPlaneAngle_, cylPlaneHitWorld_, newPlane)) {
                newPlane.sourceBodyIndex = cylPlaneBodyIndex_;
                newPlane.name = strlen(cylPlaneNameBuf_) > 0 ? cylPlaneNameBuf_ : "CylPlane";
                newPlane.isReferencePlane = true;
                newPlane.color[0] = 0.2f; newPlane.color[1] = 0.7f;
                newPlane.color[2] = 0.5f; newPlane.color[3] = 0.15f;
                projectFaceOntoSketch(cylPlaneFace_, newPlane, newPlane.sketch);
                newPlane.planeID = nextPlaneID_++;
                sketchPlanes_.push_back(std::move(newPlane));
                cylPlaneDialogOpen_ = false;
                enterSketchMode((int)sketchPlanes_.size() - 1);
            }
        }
        if (ImGui::Button("Cancel", {-1, 0})) {
            cylPlaneDialogOpen_ = false;
        }

        ImGui::End();
    }

    if (sectionWindowOpen_ && workspace_ == Workspace::Model) {
        ImGui::SetNextWindowSize({300, 0}, ImGuiCond_Appearing);
        ImGui::SetNextWindowPos({vpW - 320, 60}, ImGuiCond_Appearing);
        ImGui::Begin("Section view", &sectionWindowOpen_, ImGuiWindowFlags_AlwaysAutoResize);
        drawSectionControls();
        ImGui::End();
    }
    drawMeshImportDialog();
    pollSimulationRun(); // every frame, whichever workspace is showing
    drawSimulationPanel();
    drawMeshPlacePanel();
    drawMeshHoverReadout();

    // Timeline
    float timelineH = 0.0f;
    if (timelineOpen_ && !featureHistory_.empty()) {
        timelineH = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
        drawTimeline(panelW);
    }

    // FPS / frametime overlay
    {
        ImGuiIO& fpsIo = ImGui::GetIO();
        char fpsText[64];
        snprintf(fpsText, sizeof(fpsText), "%.1f FPS  (%.2f ms)", fpsIo.Framerate, 1000.0f / fpsIo.Framerate);
        ImVec2 textSize = ImGui::CalcTextSize(fpsText);
        ImVec2 pos(vpW - textSize.x - 8.0f, vpH - textSize.y - 8.0f - timelineH);
        ImDrawList* dl = ImGui::GetForegroundDrawList();
        dl->AddRectFilled(ImVec2(pos.x - 4, pos.y - 2), ImVec2(pos.x + textSize.x + 4, pos.y + textSize.y + 2),
                          IM_COL32(0, 0, 0, 140), 4.0f);
        dl->AddText(pos, IM_COL32(200, 200, 200, 255), fpsText);
    }

    // Dimension annotations (must be in ImGui frame, before ImGui::Render())
    {
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        float view[16], proj[16];
        getViewProj(w, h, view, proj);
        renderDimensions(view, proj, (float)w, (float)h);
    }

    // Scale ruler — bottom-left of viewport, only in sketch mode
    if (mode_ == InteractionMode::Sketching && hasActiveSketch()) {
        int fbW, fbH;
        glfwGetFramebufferSize(window_, &fbW, &fbH);
        float view[16], proj[16];
        getViewProj(fbW, fbH, view, proj);
        const auto& sp = activePlane();
        float apparentScale = computeApparentScale(sp, view, proj, (float)fbW, (float)fbH);
        float safeScale = std::max(apparentScale, 1e-9f);

        // Target ruler length ~100 logical px; round to nearest 1/2/5 × 10^n
        float targetPx  = 100.0f * dpiScale_;
        float rawMm     = targetPx / safeScale;
        float mag       = std::pow(10.0f, std::floor(std::log10(rawMm)));
        float norm      = rawMm / mag;
        float rulerMm;
        if      (norm < 1.5f) rulerMm = 1.0f * mag;
        else if (norm < 3.5f) rulerMm = 2.0f * mag;
        else if (norm < 7.5f) rulerMm = 5.0f * mag;
        else                  rulerMm = 10.0f * mag;

        // Back to logical screen pixels
        float rulerPx = rulerMm * safeScale / dpiScale_;

        // Format label
        char label[64];
        if      (rulerMm >= 1'000'000.0f) snprintf(label, sizeof(label), "%.4g km", rulerMm / 1'000'000.0f);
        else if (rulerMm >= 1000.0f)      snprintf(label, sizeof(label), "%.4g m",  rulerMm / 1000.0f);
        else                              snprintf(label, sizeof(label), "%.4g mm", rulerMm);

        // Position: bottom-left of viewport, above any timeline
        float margin  = 16.0f;
        float rulerX  = panelW + margin;
        float rulerY  = vpH - margin - 24.0f;

        ImDrawList* dl = ImGui::GetForegroundDrawList();
        ImU32 col = IM_COL32(160, 160, 160, 220);
        float capH = 5.0f;

        dl->AddLine(ImVec2(rulerX,           rulerY), ImVec2(rulerX + rulerPx, rulerY), col, 1.5f);
        dl->AddLine(ImVec2(rulerX,           rulerY - capH), ImVec2(rulerX,           rulerY + capH), col, 1.5f);
        dl->AddLine(ImVec2(rulerX + rulerPx, rulerY - capH), ImVec2(rulerX + rulerPx, rulerY + capH), col, 1.5f);

        ImVec2 ts = ImGui::CalcTextSize(label);
        dl->AddText(ImVec2(rulerX + rulerPx * 0.5f - ts.x * 0.5f, rulerY - ts.y - 3.0f), col, label);
    }
}


// UI functions (drawPreferencesWindow, drawTimeline, drawObjectTree, drawToolbar,
// replayAllFeatures, globalUndo, globalRedo, save/open/export, updateWindowTitle, markDirty)
// are in AppUI.cpp


void App::handleNavigateInput(float vpW, float vpH) {
    ImGuiIO& io = ImGui::GetIO();

    // Global undo/redo in navigate mode
    if (!io.WantCaptureKeyboard) {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z) && !io.KeyShift) globalUndo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) globalRedo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z) && io.KeyShift) globalRedo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S)) saveProjectDialog();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O)) openProjectDialog();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_E)) exportStlDialog();
    }

    // E key: enter extrude mode from navigate mode
    if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_E)) {
        enterExtrudeMode();
        return;
    }
    // V key: enter revolve mode from navigate mode
    if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_V)) {
        enterRevolveMode();
        return;
    }

    // Don't handle clicks on toolbar
    if (io.MousePos.y < 30.0f) return;

    // Face pick for "Add Reference Plane" dialog
    if (addPlaneWaitingFace_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        float view[16], proj[16];
        getViewProj(w, h, view, proj);

        float rayOrig[3], rayDir[3];
        screenToRay(io.MousePos.x, io.MousePos.y, 0, 0, vpW, vpH, view, proj, rayOrig, rayDir);

        FacePickResult faceHit = pickFace(scene_, rayOrig, rayDir);
        if (faceHit.hit) {
            SketchPlane facePlane;
            if (extractPlaneFromFace(faceHit.face, facePlane, faceHit.hitWorld)) {
                facePlane.sourceBodyIndex = faceHit.bodyIndex;
                addPlaneFaceSource_ = std::move(facePlane);
                addPlaneWaitingFace_ = false;
            }
        }
        return; // consume the click
    }

    // Double-click: start sketch on a reference plane or body face
    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        float view[16], proj[16];
        getViewProj(w, h, view, proj);

        float rayOrig[3], rayDir[3];
        screenToRay(io.MousePos.x, io.MousePos.y, 0, 0, vpW, vpH, view, proj, rayOrig, rayDir);

        // Check reference planes first (built-in + user-created)
        float bestT = 1e30f;
        int bestPlane = -1;
        float planeExtent = 10.0f;

        for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
            if (!sketchPlanes_[i].isReferencePlane) continue;
            if (!sketchPlanes_[i].visible) continue;
            float lx, ly, t;
            if (sketchPlanes_[i].rayIntersect(rayOrig, rayDir, lx, ly, t)) {
                if (t > 0 && t < bestT &&
                    std::fabs(lx) <= planeExtent && std::fabs(ly) <= planeExtent) {
                    bestT = t;
                    bestPlane = i;
                }
            }
        }

        // Check body faces
        FacePickResult faceHit = pickFace(scene_, rayOrig, rayDir);
        if (faceHit.hit && faceHit.t < bestT) {
            if (isCylindricalFace(faceHit.face)) {
                // Open tangent plane dialog for cylinders
                cylPlaneDialogOpen_ = true;
                cylPlaneAngle_ = 0.0f;
                snprintf(cylPlaneAngleBuf_, sizeof(cylPlaneAngleBuf_), "0");
                snprintf(cylPlaneNameBuf_, sizeof(cylPlaneNameBuf_), "CylPlane");
                cylPlaneFace_ = faceHit.face;
                cylPlaneHitWorld_[0] = faceHit.hitWorld[0];
                cylPlaneHitWorld_[1] = faceHit.hitWorld[1];
                cylPlaneHitWorld_[2] = faceHit.hitWorld[2];
                cylPlaneBodyIndex_ = faceHit.bodyIndex;
                return;
            }
            // Create or find sketch plane for this face
            SketchPlane newPlane;
            if (extractPlaneFromFace(faceHit.face, newPlane, faceHit.hitWorld)) {
                newPlane.sourceBodyIndex = faceHit.bodyIndex;
                newPlane.name = "Face";
                newPlane.color[0] = 0.7f; newPlane.color[1] = 0.5f;
                newPlane.color[2] = 0.2f; newPlane.color[3] = 0.15f;
                projectFaceOntoSketch(faceHit.face, newPlane, newPlane.sketch);
                newPlane.planeID = nextPlaneID_++;
                sketchPlanes_.push_back(std::move(newPlane));
                enterSketchMode((int)sketchPlanes_.size() - 1);
                return;
            }
        }

        if (bestPlane >= 0) {
            enterSketchMode(bestPlane);
        }
    }
}

// handleSketchInput, handleToolAction, switchTool, handleSelection, handleDrag,
// handleDeletion, applyGeometricConstraint are in AppSketch.cpp

// handleDimToolClick, drawDimensionPanel, renderDimensions are in AppDimension.cpp

void App::enterSketchMode(int planeIndex) {
    activeSketchPlane_ = planeIndex;
    mode_ = InteractionMode::Sketching;
    tool_.type = ToolType::None;
    tool_.reset();
    arcTool_.reset();
    selection_.clear();

    // Restore feature snapshot if editing an existing sketch feature
    FeatureID skFeat = featureHistory_.findSketchFeatureForPlane(planeIndex);
    if (skFeat != NullFeatureID) {
        auto* feat = featureHistory_.findFeature(skFeat);
        if (feat) {
            sketchPlanes_[planeIndex].sketch = std::get<SketchFeatureData>(feat->data).sketchSnapshot;
        }
    }

    // Ensure a projected anchor exists at the sketch origin so users can snap to it
    // and geometry placed there won't drift when constraints are later applied.
    {
        Sketch& sk = sketchPlanes_[planeIndex].sketch;
        bool hasOrigin = false;
        for (const auto& pt : sk.points) {
            if (pt.projected && std::fabs(pt.x) < 1e-9 && std::fabs(pt.y) < 1e-9) {
                hasOrigin = true;
                break;
            }
        }
        if (!hasOrigin) {
            EntityID oid = sk.addPoint(0.0, 0.0);
            PointEntity* op = sk.findPoint(oid);
            if (op) op->projected = true;
        }

        // Add projected X/Y axis lines if none exist yet
        bool hasProjectedLines = false;
        for (const auto& l : sk.lines) {
            if (l.projected) { hasProjectedLines = true; break; }
        }
        if (!hasProjectedLines) {
            constexpr double kAxisHalfLen = 10000.0;
            EntityID xNeg = sk.addPoint(-kAxisHalfLen, 0.0);
            EntityID xPos = sk.addPoint( kAxisHalfLen, 0.0);
            EntityID xAxis = sk.addLine(xNeg, xPos);
            if (auto* p = sk.findPoint(xNeg)) p->projected = true;
            if (auto* p = sk.findPoint(xPos)) p->projected = true;
            if (auto* l = sk.findLine(xAxis)) l->projected = true;

            EntityID yNeg = sk.addPoint(0.0, -kAxisHalfLen);
            EntityID yPos = sk.addPoint(0.0,  kAxisHalfLen);
            EntityID yAxis = sk.addLine(yNeg, yPos);
            if (auto* p = sk.findPoint(yNeg)) p->projected = true;
            if (auto* p = sk.findPoint(yPos)) p->projected = true;
            if (auto* l = sk.findLine(yAxis)) l->projected = true;
        }
    }

    history_.clear();
    history_.pushState(sketchPlanes_[planeIndex].sketch);
    lastSketchDof_ = solver_.solve(sketchPlanes_[planeIndex].sketch).dof;
    // Hide all reference planes (they stay hidden unless user unhides via object tree)
    for (int i = 0; i < kRefPlaneCount && i < (int)sketchPlanes_.size(); i++) {
        sketchPlanes_[i].visible = false;
    }
    orientCameraToPlane(sketchPlanes_[planeIndex]);
}

void App::finishSketch(bool recordFeature) {
    lastSketchDof_ = 0;
    if (tool_.type == ToolType::Extrude) cancelExtrude();

    // Record/update sketch feature and trigger replay if editing existing
    if (recordFeature && activeSketchPlane_ >= 0) {
        const Sketch& sk = activeSketch();
        auto hasUserGeom = [](const auto& vec) {
            for (const auto& e : vec) if (!e.projected) return true;
            return false;
        };
        bool sketchHasGeometry = hasUserGeom(sk.lines) || hasUserGeom(sk.circles) || hasUserGeom(sk.arcs);
        FeatureID existing = featureHistory_.findSketchFeatureForPlane(activeSketchPlane_);

        if (existing != NullFeatureID) {
            // Editing an existing sketch feature — check if it changed
            auto* feat = featureHistory_.findFeature(existing);
            const Sketch& oldSnap = std::get<SketchFeatureData>(feat->data).sketchSnapshot;

            bool changed = (oldSnap.points.size() != sk.points.size()
                || oldSnap.lines.size() != sk.lines.size()
                || oldSnap.circles.size() != sk.circles.size()
                || oldSnap.arcs.size() != sk.arcs.size()
                || oldSnap.constraints.size() != sk.constraints.size()
                || oldSnap.nextID != sk.nextID);

            if (!changed) {
                for (size_t pi = 0; pi < oldSnap.points.size() && !changed; pi++) {
                    if (oldSnap.points[pi].x != sk.points[pi].x ||
                        oldSnap.points[pi].y != sk.points[pi].y)
                        changed = true;
                }
            }
            if (!changed) {
                for (size_t ci = 0; ci < oldSnap.circles.size() && !changed; ci++) {
                    if (oldSnap.circles[ci].radius != sk.circles[ci].radius)
                        changed = true;
                }
            }
            if (!changed) {
                for (size_t ci = 0; ci < oldSnap.constraints.size() && !changed; ci++) {
                    if (oldSnap.constraints[ci].value != sk.constraints[ci].value ||
                        oldSnap.constraints[ci].driven != sk.constraints[ci].driven ||
                        oldSnap.constraints[ci].entityA != sk.constraints[ci].entityA ||
                        oldSnap.constraints[ci].entityB != sk.constraints[ci].entityB)
                        changed = true;
                }
            }

            if (changed) {
                Sketch oldSnapCopy = oldSnap;
                featureHistory_.updateSketchSnapshot(existing, sk);

                UndoCommand cmd;
                cmd.type = UndoActionType::ModifySketch;
                cmd.featureID = existing;
                cmd.oldSketch = std::move(oldSnapCopy);
                cmd.newSketch = sk;
                globalUndo_.push(std::move(cmd)); markDirty();
            }

            // Replay if dependents exist
            auto deps = featureHistory_.getDependents(existing);
            if (!deps.empty()) {
                replayAllFeatures();
            }
        } else if (sketchHasGeometry) {
            // New sketch with geometry — create a feature
            FeatureID newID = featureHistory_.addSketchFeature(activeSketchPlane_, sk, activePlane().planeID);

            UndoCommand cmd;
            cmd.type = UndoActionType::AddFeature;
            cmd.addedFeature = *featureHistory_.findFeature(newID);
            globalUndo_.push(std::move(cmd)); markDirty();
        }
    }

    activeSketchPlane_ = -1;
    mode_ = InteractionMode::Navigate;
    tool_.reset();
    arcTool_.reset();
    selection_.clear();
    dimTool_.reset();
}

void App::orientCameraToPlane(const SketchPlane& plane) {
    float nx = plane.normal[0], ny = plane.normal[1], nz = plane.normal[2];

    float targetPitch = std::asin(std::clamp(ny, -1.0f, 1.0f)) * 180.0f / kPi;

    // For axis-aligned planes, snap to clean 90° values; for arbitrary planes, use exact angles
    bool isAxisAligned = (std::fabs(std::fabs(nx) - 1.0f) < 0.01f && std::fabs(ny) < 0.01f && std::fabs(nz) < 0.01f)
                      || (std::fabs(nx) < 0.01f && std::fabs(std::fabs(ny) - 1.0f) < 0.01f && std::fabs(nz) < 0.01f)
                      || (std::fabs(nx) < 0.01f && std::fabs(ny) < 0.01f && std::fabs(std::fabs(nz) - 1.0f) < 0.01f);

    if (isAxisAligned) {
        targetPitch = std::round(targetPitch / 90.0f) * 90.0f;
    }

    float targetYaw;
    float horizLen = std::sqrt(nx * nx + nz * nz);
    if (horizLen < 1e-4f) {
        // Near-vertical normal — keep current yaw (snap for axis-aligned)
        targetYaw = isAxisAligned
            ? std::round(viewport3D_.camera().yaw / 90.0f) * 90.0f
            : viewport3D_.camera().yaw;
    } else {
        targetYaw = std::atan2(nx, nz) * 180.0f / kPi;
        if (isAxisAligned) targetYaw = std::round(targetYaw / 90.0f) * 90.0f;
    }

    // Take shortest rotation path for yaw (avoid 350° → 10° going the long way)
    float currentYaw = viewport3D_.camera().yaw;
    float yawDiff = targetYaw - currentYaw;
    while (yawDiff > 180.0f) yawDiff -= 360.0f;
    while (yawDiff < -180.0f) yawDiff += 360.0f;
    targetYaw = currentYaw + yawDiff;

    cameraFrom_ = viewport3D_.camera();
    cameraTo_ = viewport3D_.camera();
    cameraTo_.yaw = targetYaw;
    cameraTo_.pitch = targetPitch;
    cameraTo_.targetX = plane.origin[0];
    cameraTo_.targetY = plane.origin[1];
    cameraTo_.targetZ = plane.origin[2];

    cameraAnimating_ = true;
    cameraAnimT_ = 0.0f;
}

void App::updateCameraAnimation(float dt) {
    if (!cameraAnimating_) return;

    cameraAnimT_ += dt * 3.0f;
    if (cameraAnimT_ >= 1.0f) {
        cameraAnimT_ = 1.0f;
        cameraAnimating_ = false;
    }

    float t = cameraAnimT_;
    float s = t * t * (3.0f - 2.0f * t); // smoothstep

    OrbitCamera& cam = viewport3D_.camera();
    cam.yaw = lerp(cameraFrom_.yaw, cameraTo_.yaw, s);
    cam.pitch = lerp(cameraFrom_.pitch, cameraTo_.pitch, s);
    cam.distance = lerp(cameraFrom_.distance, cameraTo_.distance, s);
    cam.targetX = lerp(cameraFrom_.targetX, cameraTo_.targetX, s);
    cam.targetY = lerp(cameraFrom_.targetY, cameraTo_.targetY, s);
    cam.targetZ = lerp(cameraFrom_.targetZ, cameraTo_.targetZ, s);
}

// enterExtrudeMode, handleExtrudeInput, drawExtrudePanel, updateExtrudePreview,
// renderExtrudePreview, renderExtrudeHandle, editExtrudeFeature, commitExtrude,
// cancelExtrude are in AppExtrude.cpp

// enterRevolveMode, handleRevolveInput, drawRevolvePanel, updateRevolvePreview,
// renderRevolvePreview, commitRevolve, cancelRevolve, editRevolveFeature are in AppRevolve.cpp

// enterLoftMode, handleLoftInput, drawLoftPanel, updateLoftPreview,
// renderLoftPreview, commitLoft, cancelLoft, editLoftFeature are in AppLoft.cpp

void App::shutdown() {
    scene_.clear();
    sketchRenderer_.shutdown();
    viewport3D_.shutdown();
    if (simLineVAO_) { glDeleteVertexArrays(1, &simLineVAO_); simLineVAO_ = 0; }
    if (simLineVBO_) { glDeleteBuffers(1, &simLineVBO_); simLineVBO_ = 0; }
    if (capVAO_) { glDeleteVertexArrays(1, &capVAO_); capVAO_ = 0; }
    if (capVBO_) { glDeleteBuffers(1, &capVBO_); capVBO_ = 0; }
    if (simRunner_.running()) simRunner_.cancel();
    releaseSimulationResults();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

} // namespace shitcad
