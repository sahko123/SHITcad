# SHITcad

Parametric CAD modeler. Sketch 2D profiles with constraints, extrude/revolve/loft them into 3D solids, full feature history with undo/redo and timeline playback.

## Build

```bash
# Requires: CMake 3.20+, MSVC (C++17), vcpkg (installs OpenCASCADE and a trimmed qtbase)
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

Tests are opt-in (`-DSHITCAD_BUILD_TESTS=ON`), then run `build/Release/MeshImportTest.exe`, `SimulationTest.exe` and `ReplayTest.exe`. They build on the core sources only (no `App*.cpp`, no UI). `ReplayTest` builds one history per feature type in code and checks volumes, bounds, every constraint type and save/load round trips; replay needs no GL context. `SimulationTest` compiles the shaders and picks through a section view in an offscreen Qt GL context. Manual checks: `docs/smoke-checklist.md`.
Run the build from PowerShell or cmd: Git Bash rewrites MSBuild's `/m` switch into a path.

The UI is Qt 6 Widgets (`src/qt/`, the only place Qt is used; history in `docs/qt-migration-plan.md`). The first configure builds OpenCASCADE and qtbase through vcpkg and takes a long time. The build copies Qt's `platforms/qwindows.dll` next to the executable; without it nothing can open a window.

Dependencies fetched via FetchContent: glad (the GL loader, generated at build time; needs Python with Jinja2) and nlohmann/json 3.11.3. OpenCASCADE and Qt come from vcpkg.

## Architecture

Everything is in the `shitcad` namespace. Single-window app, no threads.

### Two modes
- **Navigate** (`InteractionMode::Navigate`) - orbit 3D view, pick features, scrub timeline
- **Sketching** (`InteractionMode::Sketching`) - 2D constrained sketch editing on a plane

### App class split across files
The `App` class is large and split across multiple .cpp files by responsibility:

| File | Methods |
|------|---------|
| `App.cpp` | `init()`, `frame()`, `paint()`, `post()`, `shutdown()`, `renderFrame()`, camera, navigation input |
| `AppSketch.cpp` | `handleSketchInput()`, `handleToolAction()`, `handleSelection()`, `handleDrag()`, constraint application, the inline value box's model |
| `AppUI.cpp` | Panel models and operations: toolbar (`toolbarModel()`/`perform()`), preferences, mesh import and placement, plane dialogs, object tree, timeline; save/open/export, global undo |
| `AppDimension.cpp` | Dimension panel model, `handleDimToolClick()`, `renderDimensions()`, dimension label layout |
| `AppExtrude.cpp` | Extrude tool: input, panel model, preview, commit, edit |
| `AppRevolve.cpp` | Revolve tool: input, panel model, preview, commit, edit |
| `AppLoft.cpp` | Loft tool: input, panel model, preview, commit, edit |
| `AppBoolean.cpp` | Boolean tool: input, panel model, preview, commit |

### UI (`src/qt/`)
- `main.cpp` builds the `QMainWindow`: the viewport as central widget, the toolbar, the object tree and Simulation docks, dialogs, and the widgets floating over the view (tool panel, timeline, Dimension panel, inline value box). It also sets the Fusion palette from the Light Mode preference and saves the window layout (`QSettings`).
- `ViewportWidget` is the `QOpenGLWidget` host (implements `AppHost`): each repaint it fills an `InputFrame` (`QtInput.cpp`), calls `App::frame` then `App::paint`, paints `App::overlay()` with QPainter (`QtOverlay.cpp`, GL state saved and restored around it), and emits `frameBuilt`.
- **Panels are views of App**: on `frameBuilt` each reads a model struct (`toolbarModel()`, `timelineModel()`, `simSetupModel()`, ...) and changes App only by `App::post()`-ing a named operation, which runs at the start of the next frame. A refresh never writes into a widget that has focus or is being dragged; structural changes (rows added or removed) rebuild only what changed. Live edits commit their undo step when the edit finishes.
- **Keyboard shortcuts stay in the viewport handlers**: `KeyRouting.cpp` forwards keys from the docks, the floating panels and the tool windows to the viewport unless a text field has focus (Escape and Enter stay with a dialog). Qt dialogs tell App they closed by overriding `reject()`.

### Data flow for 3D features (extrude/revolve/loft)
1. User enters sketch mode on a `SketchPlane` -> edits `Sketch` entities
2. `finishSketch()` records a `SketchFeatureData` snapshot in `FeatureHistory`
3. User enters extrude/revolve/loft mode
4. `ProfileDetector::detectClosedProfiles()` finds closed regions in the sketch
5. User selects profiles, adjusts parameters
6. `commit*()` calls OCCT to build the `TopoDS_Shape`, adds `Body3D` to `Scene3D`
7. Feature data (with `ProfileSignature`s) is stored in `FeatureHistory`
8. `FeatureReplay` can rebuild the entire model from history

### ID system
- `EntityID` (`uint32_t`) - identifies sketch entities (points, lines, circles, arcs, constraints). `NullID = 0`.
- `FeatureID` (`uint32_t`) - identifies features in the history tree. `NullFeatureID = 0`.
- `PlaneID` (`uint32_t`) - stable plane identifier. `NullPlaneID = 0`.
- IDs are monotonically increasing, generated by `genID()` / `nextID_`.

### Sketch data model (`SketchData.h`)
Entities are stored in flat vectors with O(1) lookup indices (`std::unordered_map<EntityID, size_t>`):
- `points`, `lines`, `circles`, `arcs`, `ellipses`, `ellipseArcs`, `splines`, `constraints`
- After bulk modifications, call `rebuildIndices()`
- Points are shared: a line references two point IDs, an arc references center + start + end points
- Removing an entity cascades: removes referencing constraints and orphaned points

### Constraint solver (`Solver.cpp`)
Newton-Raphson, up to 40 iterations, convergence tolerance 1e-6. Call `solver.solve(sketch)` or `solver.solve(sketch, draggedPointID)` for interactive dragging. Returns `SolveResult` with DOF, convergence status, total error.

### Constraint types
`Coincident`, `Horizontal`, `Vertical`, `Distance`, `Radius`, `Diameter`, `PointDistance`, `PointOnLine`, `PointLineDistance`, `EqualLength`, `Perpendicular`, `Parallel`, `Tangent`, `Angle`, `Symmetric`, `Concentric`, `Midpoint`

### Units and axes
The 3D viewport is **Y-up**: the ground plane is XZ at y = 0 and the orbit camera's up is +Y. The grid shows only while the view looks straight along an axis (`OrbitCamera::viewAxis()`, e.g. after numpad 1/3/7), in the plane facing the camera, as a backdrop that writes no depth. (The "XY Plane" reference plane is vertical.) STL from most CAD packages, Onshape included, is Z-up and lands on its side until rotated. Simulation exports declare `"up": [0, 1, 0]` rather than converting.

All internal values are in **millimeters**. `UnitUtils.h` handles parsing/formatting with 11 unit types. `Constraint::value` is always in mm; `Constraint::inputUnit` and `inputValue` preserve the user's original input.

### Feature history and replay
- `FeatureHistory` stores a linear list of `Feature` objects (sketch, extrude, revolve, loft, boolean)
- Features reference each other by `FeatureID` (e.g., extrude references its source sketch feature)
- `ProfileSignature` matches profiles across sketch edits using entity IDs + centroid
- `FeatureReplay::replayAll()` rebuilds the entire 3D model from the feature list
- Timeline has a rollback position (playhead) for "what-if" exploration

### Undo system (`UndoStack.h`)
Global undo stack with typed commands: `AddFeature`, `DeleteFeature`, `SuppressFeature`, `ModifySketch`, `ModifyExtrude`, etc. Each command stores old and new state for reversal.

### Rendering pipeline
- `Viewport3D` - orbit camera (yaw/pitch/distance), orthographic/perspective, ground grid, and the sky/ground background (`drawBackground`, first after the clear; theme colours `skyZenith`/`skyHorizon`/`groundHorizon`/`groundNadir`)
- `Scene3D` - stores `Body3D` objects (OCCT shape + tessellated mesh and edges on the CPU; VAO/VBO are a cache uploaded lazily by `syncGpu()` when rendering). Adding, replacing and removing bodies never calls GL, so replay/undo/commit need no GL context (the tests run without one); freed buffers are queued and deleted at the next render. `vertexCount` is set when the body is built.
- Host boundary: App never calls Qt. The host drives `App::frame(dt, fbW, fbH, input)` then `App::paint()` with the context current, then draws `App::overlay()`; `AppHost` gives App the window title, redraw requests and file dialogs. Everything outside the frame changes App state only through `App::post()`.
- Viewport input is `App::in_` (`ViewportInput.h`), passed in once per frame with the semantics the handlers were written against: held-key repeat, trickled events (a quick click is seen down, then up), furthest-drag distance. Screen-space drawing over the 3D view is recorded into `App::overlay_` (`Overlay2D.h`) during the frame, including from inside the GL pass, and drawn once after it; its text is measured by the host's function so labels and hit rectangles match what is drawn.
- `SketchRenderer` - draws sketch geometry, tool previews, selection highlights, dimension labels
- Shaders are compiled at init via `ShaderProgram`

### Profile detection (`ProfileDetector.h/cpp`)
`detectClosedProfiles()` walks sketch geometry adjacency to find closed regions. Returns `ClosedProfile` with outer boundary + holes. Handles line/arc/circle/ellipse/spline boundaries. Tessellation via `tessellateProfile()`.

### Mesh imports (`MeshImport.h/cpp`)
- STL import is a `FeatureType::MeshImport` feature that stores the **file path and its unit**, not the triangles. Replay re-reads the file (cached by path + size + mtime), so a re-exported file is picked up automatically and a missing one is an error on that feature.
- STL has no units, so the import dialog makes the user choose one while showing the resulting size. Coordinates are scaled to mm on load.
- Placement (`MeshTransform`: row-major rotation matrix + translation in mm, `p' = R p + t`) is applied after unit scaling on every replay. The Place panel (opens after import; timeline double-click or right-click "Rotate / Move...") rotates about the mesh's bounding-box centre via `rotateAbout`, applies edits live, and records one `ModifyMeshImport` undo step on Done. Quarter turns from `axisRotation` are exact. An identity transform is not written to the project file.
- The body it creates is **mesh-only**: `Body3D::shape` is null, `isMeshOnly()` is true, and `sourceFeature` holds the MeshImport's `FeatureID`. `pickFace` cannot see it; use `pickMesh` (`FacePicker.h`).
- Projects containing a MeshImport are saved as `version: 2`; projects without one stay `version: 1`.

### Simulation workspace (`Simulation.h/cpp`, `AppSimulation.cpp`)
- Toolbar tabs switch `workspace_` between Model and Simulation (only from Navigate with no 3D tool active). Simulation replaces `handleNavigateInput` with `handleSimulationInput` and shows the Simulation panel.
- `SimulationSetup` (surface roles, nozzles, run settings) is set-up data, not geometry: saved as a `"simulation"` block in the project (forces `version: 2`), undone as a whole via `ModifySimulation`. Live edits change `simulation_` directly; `commitSimulationEdit()` pushes one undo step from `simUndoBase_` (a no-op if nothing changed) - call it when an edit finishes (slider released, field finished). The operations for discrete actions (flip, delete, role) commit themselves.
- Nozzles store position/axis in their **host mesh's frame** (`hostFeature`), so moving or rotating an import carries its nozzles. Always go through `nozzleWorld` / `setNozzleWorld`.
- A placed nozzle sprays along **-normal** of the picked triangle (into the cavity for an outward-wound fluid-cavity STL), offset inward by the standoff.
- `buildTier1Spec` writes a cip-sim spec (`cip-sim/spec/README.md`): mm, `"up": [0,1,0]`, one surface per active MeshImport with its unit and placement as `transform`. The frame is declared, not converted.
- `SimulationTest.exe <dir>` also writes a spec + expected bounds bundle that cip-sim's `tests/test_shitcad_export.py` verifies from Python. `SimulationTest.exe <dir> <cip-sim dir> [python]` adds the process-runner tests and an end-to-end Tier 1 run.
- **Running** (`AppSimulationRun.cpp`): Run writes the spec to `<project>_sim/tier1_<time>/` (or `%TEMP%\SHITcad_sim`) and starts `python -u -m cipsim.cli tier1` via `ProcessRunner` (`SimProcess.h`) in the cip-sim folder. No threads: `pollSimulationRun()` runs every frame and reads JSON-lines events. Runs are in a kill-on-close job; the ParaView launcher is not, so the viewer outlives the app.
- **Engine location** (cip-sim folder, Python) is per machine: `%APPDATA%\SHITcad\simulation.json`, edited in the Run section.
- **Results** (`SimResults.h`): cip-sim's `tier1.json` + `tier1.bin` (flat float32, metres) load into a `ResultMesh` in mm, drawn with `kResultVertSrc/kResultFragSrc` (per-vertex colour, optional cut plane). Imported meshes are skipped by `Scene3D::render(..., skipMeshOnly)` while results show, since results are drawn on the same triangles. The panel marks results stale when the spec that would run now differs from the one that produced them.
- `windows.h` defines `near`/`far` as empty macros: include it after `App.h` and `#undef` them.

### Section views (`Section.h`, `AppSection.cpp`)
- One `SectionPlane` (`section_`) cuts the whole scene: imported meshes, solid bodies, their wireframe edges and simulation results. Spray cones and the ground grid are deliberately left whole.
- Shaders declare `uClipOn` / `uClipNormal` / `uClipOffset`; **always call `applyClip(shader, plane_or_nullptr)` before drawing**, since uniforms persist on a program between draws and a stale one cuts the wrong thing. Tool previews and the ground grid pass `nullptr`.
- `renderSectionCap` fills the opening by stencil: draw the clipped closed meshes with depth testing off, front faces incrementing and back faces decrementing, then draw a quad on the plane where the count is non-zero. Needs the stencil buffer cleared each frame.
- Controls appear in the Simulation panel under "View" and, in the Model workspace, from the toolbar's Section button. The plane is a view setting: not saved in the project.

### Fixes from the 2026-09-16 review (read before touching these)
- **Paths are stored as UTF-8.** The file dialogs hand back UTF-8 (the old Win32 dialogs returned the ANSI code page, and an accented character threw out of `dump()` and terminated the app). JSON dumps also use `error_handler_t::replace`.
- **Open files through `Utf8Path.h`, never with a raw `std::string`.** `std::ofstream(str)`, `fs::path(str)`, `path::string()`, `CreateProcessA` and `ShellExecuteA` all read ANSI on Windows, so a UTF-8 path fails to open anything outside ASCII (and `path::string()` throws). Use `fsPath(str)` to open and `utf8(path)` to get a string back; start processes with the W APIs. OCCT's `const char*` file functions take UTF-8, so pass them the string directly. `testNonAsciiPaths` covers import, save/load, results and the engine launch.
- **State that can go stale is the main hazard here**, not the maths. `App::runInputs()` is the single definition of what a run depended on - spec + rays/bounces (which are argv, not in the spec) + a size/mtime stamp of every STL - and the Results panel compares it. `clearSimulationRun()` is called when a project is opened; `simSummary_` is cleared on start and on cancel, so a previous run's numbers can never be attributed to a later one.
- **A mesh's unit change rescales its nozzles** (`setMeshImportData`), since they are stored in the host's scaled frame.
- **The mesh cache keys on path + size + mtime + a sampled content hash.** Size and mtime alone collide: a re-exported STL with the same triangle count is the same size, Windows file times are ~4 ms granular, and timestamp-preserving copies collide exactly.
- **Picking takes the section plane** (`pickMesh`/`pickFace`): a hit on the cut-away side is skipped, or a click lands on geometry that is not on screen. `pickFace` also skips hidden bodies.
- **Culling stays off** for the body pass (both shaders are two-sided and `pickMesh` is two-sided); `SketchRenderer` enables it without restoring it, so `render3DScene` disables it explicitly and `renderSectionCap` restores what it found.
- **The cap only caps closed surfaces** (`trianglesAreClosed`, cached per mesh as `MeshFileInfo::closed` and `Body3D::closed`). An open surface's face counts never cancel, so the stencil covers its whole projected area and the quad paints over the model.
- **Bounds caches fold vertex positions into the key**, because a moved mesh has an identical vertex count.
- **Result colours distinguish "no measurement" from "zero"**: NaN and zero are grey, a sprayed-but-unsampled face is amber, and unscored surfaces are muted since the legend's percentages count scored wall only. Categorical colours keep their slot when one fails to parse.

### File I/O (`Serialization.h/cpp`)
- Project format: JSON (nlohmann/json), stores full feature history + plane definitions
- Export: STL, STEP, IGES, OBJ, DXF (via OCCT)
- Import: STL (as a MeshImport feature, above), STEP, IGES
- File dialogs go through `AppHost::chooseFile` (`FileDialogs.h` lists them): `QFileDialog`, native on Windows, with Unicode paths.

## Conventions

- **C++17**, `#pragma once` for headers
- **Naming**: `camelCase` for methods/variables, `PascalCase` for types/classes, `kPascalCase` for constants, trailing underscore for private members (`mode_`, `window_`)
- **Enums**: `enum class` with `uint8_t` underlying type
- **State structs**: tool states (e.g., `ExtrudeToolState`) have a `reset()` method that does `*this = {}`
- **No exceptions** - error handling via return values (`bool`, `SolveResult`, etc.)
- **OCCT types**: `TopoDS_Shape`, `TopoDS_Face`, `gp_Pnt`, etc. are used at the boundary between sketch/profile data and 3D geometry
- **UI**: Qt Widgets, only in `src/qt/`; panels follow App's models and act through `App::post()` (see "UI" above). No UI code in `App*.cpp` beyond models and operations.
- **OpenGL 3.3 core** - no fixed pipeline, manual VAO/VBO management

## Adding a new sketch entity type

1. Define the entity struct in `SketchData.h` (follow `ArcEntity` pattern)
2. Add vector + index map in `Sketch`, implement `find*()`, `add*()`, `remove*()`, update `rebuildIndices()`
3. Add to `HitTest.cpp` - distance function + case in `hitTest()`
4. Add to `Snap.cpp` if it has snap points (endpoints, center, etc.)
5. Add to `SketchRenderer.cpp` for drawing
6. Add to `ProfileDetector.cpp` if it can be a profile boundary
7. Add to `Solver.cpp` if constraints can reference it
8. Add to `Serialization.cpp` for save/load
9. Add `ToolType` entry and handler in `Tools.h/cpp`
10. Wire up in `AppSketch.cpp` (`handleToolAction`) and the toolbar (`toolbarModel()`/`perform()` in `AppUI.cpp`, drawn by `src/qt/Toolbar.cpp`)

## Adding a new constraint type

1. Add to `ConstraintType` enum in `SketchData.h`
2. Implement residual + Jacobian in `Solver.cpp` (in the main solve loop)
3. Add to `SketchRenderer.cpp` if it needs a visual indicator
4. Add to `AppDimension.cpp` if it's a dimensional constraint (distance/angle)
5. Add to `AppSketch.cpp` `applyGeometricConstraint()` if it's geometric
6. Add to `Serialization.cpp` for save/load
7. Add a toolbar button (`toolbarModel()`/`perform()` in `AppUI.cpp`, drawn by `src/qt/Toolbar.cpp`)

## Adding a new 3D feature type

1. Define `*FeatureData` struct in `FeatureHistory.h`
2. Add to `FeatureType` enum and `Feature::data` variant
3. Add `add*Feature()`, `update*Data()` to `FeatureHistory`
4. Create `App*Tool.cpp` with input, panel model and operations, preview, commit, cancel, edit methods, and a page for it in `src/qt/ToolPanel.cpp`
5. Add `*ToolState` struct in `ExtrudeTool.h` (or new header)
6. Add tool state member + methods to `App.h`
7. Add case in `FeatureReplay.cpp` `replayAll()` for rebuilding from history
8. Add to `Serialization.cpp` for save/load
9. Add `UndoActionType` + handling in `UndoStack.h/cpp`
10. Wire up the toolbar entry (`toolbarModel()`/`perform()` in `AppUI.cpp`)

## Key constants (`Constants.h`)

| Constant | Value | Purpose |
|----------|-------|---------|
| `kSolverMaxIterations` | 40 | Newton-Raphson iteration limit |
| `kSolverConvergenceTol` | 1e-6 | Per-constraint convergence threshold |
| `kCircleTessSteps` | 72 | Circle tessellation subdivisions |
| `kArcTessDegreesPerStep` | 5.0 | Arc tessellation resolution |
| `kPreviewThrottleMs` | 50 | Min ms between 3D preview rebuilds |
| `kDegenerateLen` | 1e-6 | Length below which geometry is degenerate |

## Common pitfalls

- Forgetting to call `sketch.rebuildIndices()` after bulk entity modifications
- Not updating `ProfileSignature` matching when changing how profiles are detected
- OCCT boolean operations can fail silently - always check the resulting shape is valid
- `Constraint::value` is always in mm regardless of what unit the user typed
- The `App` class `.cpp` files all share the same `App::` method scope - a method declared in `App.h` can be defined in any of the `App*.cpp` files
- When adding new serialization fields, maintain backward compatibility with existing project files (check for key existence before reading)
- Any loop that runs OCCT operations over scene bodies must skip `body.isMeshOnly()` bodies - they have no B-rep. OCCT rejects a null shape rather than crashing, but the failure is logged to the diagnostics file on every replay.
- Anything added straight to `Scene3D` without a feature is wiped by the next `replayFeatures()` and never saved. STEP and IGES import still do this.
