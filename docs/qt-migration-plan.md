# Qt Widgets migration plan

Goal: replace GLFW + Dear ImGui with Qt 6 Widgets, and leave the CAD core and the
3D viewport as layers that do not know which GUI toolkit hosts them. Cross-platform
is a goal; Linux is kept compiling but is not run or debugged yet.

## Rules for every phase

1. **The app works at the end of every phase**, and ideally at every commit. ImGui
   and Qt coexist until the last phase; nothing is removed before its replacement
   has passed the smoke checklist.
2. **Moving and changing are separate commits.** A commit that moves input or
   drawing onto a new interface does not also change behaviour.
3. **The core does not change**: solver, sketch data, feature history, replay, undo,
   OCCT code, serialization and the project file format. If a phase appears to
   need a core change, stop and discuss it first.
4. **The safety net comes first** (Phase 0). The phases after it are only as safe as
   the checks that run after each step.

## What exists today (measured 2026-09-22)

- About 820 `ImGui::` calls in 13 compiled files. `AppUI.cpp` has 364, the
  simulation files about 170, and each tool file 30-50. `Canvas.cpp/h` is not in
  `CMakeLists.txt` and is dead code.
- ImGui does three jobs:
  - **Widgets**: toolbar, object tree, timeline, preferences, tool panels,
    simulation panel, dialogs.
  - **Input**: every 3D handler polls ImGui: 54 `IsKeyPressed`, 33 `io.MousePos`,
    `IsMouseClicked/Released/DoubleClicked/Dragging`, `MouseDelta`, `MouseWheel`,
    `WantCaptureMouse/Keyboard`. This includes `Viewport3D::handleInput`,
    `handleNavigateInput`, `handleSketchInput`, the Extrude/Revolve/Loft/Boolean
    and Simulation handlers, and `updateMeshHover`.
  - **Viewport overlays**: `GetForegroundDrawList()` in `App.cpp` (box select, FPS,
    scale ruler), `AppDimension.cpp` (dimension labels and their hit rects),
    `AppExtrude.cpp`, `AppSketch.cpp`, `AppSimulation.cpp` and `AppUI.cpp` (mesh
    hover readout).
- ImGui widgets inside the viewport: the inline dimension input (`##InlineDim`,
  `AppSketch.cpp`), the sketch message, and the dimension value box. `AppSketch.cpp`
  reaches into ImGui internals (`GImGui->ActiveId = 0`) to reset an InputText.
- The viewport is the **whole window**; panels float over it. Handlers receive
  `vpW/vpH = io.DisplaySize`. Three handlers hard-code the toolbar height
  (`io.MousePos.y < 30.0f` in `App.cpp` `handleNavigateInput`, `AppSimulation.cpp`
  and `AppUI.cpp`).
- **Shortcuts depend on the mode.** `Ctrl+Z/Y` undo the *sketch* history in a sketch
  (`AppSketch.cpp`) and the *global* history in Navigate and Simulation. `E` enters
  extrude, but `Ctrl+E` exports STL. `O` toggles the projection, but `Ctrl+O` opens a
  project.
- `Theme` in `Preferences.h` stores dimension colours as `ImU32`, and
  `Preferences::applyTheme()` styles ImGui.
- The tests (`MeshImportTest`, `SimulationTest`) exclude `App*.cpp`, so nothing
  automated covers the UI or input layer. `MeshImportTest` creates a hidden GLFW
  window only because replay uploads meshes to OpenGL: `Scene3D::addBody` uploads
  VBOs immediately, so replay, undo and commit all need a current GL context.
- Some overlay drawing happens **inside the GL pass**: `renderExtrudeHandle` is
  called from `render3DScene` and draws to ImGui's foreground layer.
- `handleSketchInput` mixes input with ImGui widgets (the `##InlineDim` and
  `##SketchMsg` windows), and the dimension tool reads `io.InputQueueCharacters`
  to detect that the user has started typing.
- Every `IsKeyPressed` call uses ImGui's default auto-repeat.

## Phase 0: safety net and cleanup (no user-visible change)

1. Delete `Canvas.cpp/h`. Remove the unused `BRepBuilderAPI_Sewing`,
   `BRepBuilderAPI_MakeFace` and `RWStl` includes from `Serialization.cpp`.
2. **Fixture histories, built in code** (as `MeshImportTest` already does), not
   saved from the app by hand, so they stay valid when the format changes: a sketch
   with every entity and constraint type; extrude (join and cut); revolve; loft;
   boolean; a suppressed feature and a rolled-back playhead; an STL import with
   placement; a simulation setup with nozzles. Save them to a temporary folder as
   part of the test to exercise the load path as well.
3. **`ReplayTest`**: for each fixture, replay, then check body count, volume and
   bounding box (`GProp`), solver convergence for every sketch, and that
   save -> load -> save gives identical JSON (the serializer writes no timestamps,
   and nlohmann's default object type sorts keys). This covers everything under
   the UI. Until Phase 3 makes GPU upload lazy, it uses a hidden GLFW window, like
   `MeshImportTest`.
4. **Smoke checklist** ([smoke-checklist.md](smoke-checklist.md)): a manual script
   that covers every interaction path:
   - orbit, pan, zoom, ortho toggle
   - enter a sketch; draw each entity; snaps; box and lasso select; drag; delete
   - the inline dimension input and the dimension tool (new and editing existing)
   - geometric constraints
   - extrude via drag handle and via typed value; revolve; loft; boolean
   - timeline: scrub, rollback, rename, suppress, delete, double-click to edit
   - undo/redo across all of the above
   - add a reference plane from offset and from a face; cylinder tangent plane
   - STL import, unit choice, placement
   - simulation: place a nozzle, edit, run, results, stale marking, section view
   - save, open, and every export
5. Optional: a GitHub Actions Windows build that runs the tests. It needs vcpkg
   binary caching (e.g. `VCPKG_BINARY_SOURCES` with the GitHub Actions cache),
   or every run rebuilds OCCT, and Qt later.

**Done when:** `ReplayTest` passes and the checklist has been walked once on the
current build to record the baseline.

## Phase 1: input abstraction (still GLFW + ImGui)

Add `ViewportInput.h`:

```cpp
enum class Key : uint8_t { A, C, D, E, F, L, N, O, P, R, S, T, V, Y, Z,
                           Num0, Num9, Keypad0, Keypad9, KeypadDecimal, KeypadEnter,
                           Period, Enter, Escape, Delete, Backspace, Count };
enum class MouseButton : uint8_t { Left, Right, Middle, Count };

struct InputFrame {
    // Viewport-local, in framebuffer pixels (the same space as picking and projection).
    float mouseX = 0, mouseY = 0, mouseDX = 0, mouseDY = 0, wheel = 0;
    float viewportW = 0, viewportH = 0, dt = 0;
    bool shift = false, ctrl = false, alt = false;
    bool uiWantsMouse = false, uiWantsKeyboard = false;
    std::bitset<(size_t)MouseButton::Count> down, clicked, released, doubleClicked;
    std::bitset<(size_t)Key::Count> pressed;   // includes auto-repeat, like ImGui::IsKeyPressed
    std::u32string typed;                      // exactly what ImGui would queue in InputQueueCharacters
    bool dragging(MouseButton b, float threshold) const;
    bool hovered() const;                      // mouse inside the viewport
};
```

1. Add `fillInputFromImGui(InputFrame&)`, the only place that reads ImGui input.
   It must match ImGui's semantics exactly: key repeat, the double-click interval,
   the drag threshold, and the typed characters. The dimension tool's angle
   auto-side toggle switches off on the first typed character, so a mismatch in
   `typed` changes behaviour.
2. Pass `const InputFrame&` to every handler, replacing the ImGui calls mechanically.
   Do one handler per commit and run the relevant checklist items after each.
3. Replace the hard-coded `mouseY < 30` checks with `hovered()`. The viewport rect
   comes from the host, not a guessed toolbar height.
4. Keep `IsItemDeactivatedAfterEdit` and the other widget-state calls as they are:
   those belong to panels, not input.
5. Move the ImGui windows out of `handleSketchInput`: the inline dimension input
   and the sketch message become `drawInlineDimInput()` and `drawSketchMessage()`,
   called from `renderFrame` after input handling, in the same order as today. The
   handler keeps the state and decisions; the draw functions only show them. Split
   any other handler that opens ImGui windows the same way.

**Done when:** no input handler reads input from ImGui or draws ImGui widgets, and
the checklist passes.

**Status (done).** `InputFrame` is in `ViewportInput.h`; `App::in_` is filled once by
`fillInputFromImGui` at the top of `renderFrame` (a member rather than a parameter,
so the handler signatures did not change; `Viewport3D::handleInput` takes it as a
parameter). The snapshot is exact: ImGui updates capture flags and mouse/key state in
`NewFrame`, clears typed characters in `EndFrame`, and every query the handlers made
uses the "any owner" test, which no widget earlier in the frame can lock. The
toolbar checks compare `mouseY < viewY` (the toolbar height) instead of `< 30`, which
is the same set of pixels once ImGui's own capture over the toolbar is counted.
`drawInlineDimInput` and `drawSketchMessage` are separate functions called from the
same point in `handleSketchInput`. A scripted input scenario run against the old and
new builds gave pixel-identical screenshots. Still in the handlers, by design:
`ImDrawList` overlays (Phase 2) and the `GImGui->ActiveId = 0` resets of the
dimension InputText (Phase 5.10).

## Phase 2: overlay abstraction (still ImGui)

Add `Overlay2D.h`, an interface for screen-space drawing: `line`, `rect`,
`filledRect(rounding)`, `circle`, `polyline`, `text`, `measureText`, with colours as
a plain `Rgba`. Everything in the viewport that is currently drawn with
`GetForegroundDrawList()` is drawn through it: box select, FPS, scale ruler,
dimension labels, extrude handle label, sketch overlays, simulation labels, and the
mesh hover readout.

1. **Recorded, not immediate.** `Overlay2D` appends commands to a list during tick
   and paint, and the host draws the list once, after the GL pass. This is
   necessary because some overlays are produced inside the GL pass
   (`renderExtrudeHandle`), and QPainter cannot be mixed into the middle of GL
   drawing. `measureText` answers straight away, since layout needs it. For now,
   `ImGuiOverlay` replays the list into `ImDrawList` before `ImGui::Render()`.
2. Change `Theme`'s `ImU32` members to `Rgba`, so `Preferences.h` no longer depends
   on ImGui.
3. `dimLabelRects_` keeps using the same `measureText`. Hit rects and drawn labels
   then stay consistent when the font changes in Phase 6.

**Done when:** no viewport drawing includes `imgui.h`, and the checklist passes.

**Status (done).** `Overlay2D.h` holds the recorded command list and `Color32`
(IM_COL32's byte layout, so theme values are unchanged). `App::overlay_` is cleared
before `renderFrame` and flushed by `flushOverlayToImGui` after `render3DScene` and
before `ImGui::Render()`. That also fixes a latent ordering hazard: the extrude
handle, box select and simulation overlays used to append to ImGui's foreground
list after `ImGui::Render()` had already built the draw data. `measureWithImGui`
keeps the two measuring calls the old code used (`CalcTextSize`, and
`CalcTextSizeA` for the 0.85-scale constraint icons), because they round
differently. `InputFrame` gained `screenW/screenH` for projection in overlays.
Old and new builds were pixel-identical across a scenario covering constraint
icons, dimension and diameter labels, the H snap label, a box select mid-drag,
the ruler and the extrude handle. Left for later: the FPS readout still reads
`ImGui::GetIO().Framerate` (host code, replaced in Phase 4), and the results
legend and timeline draw into their own panels (`GetWindowDrawList`, Phase 5).

## Phase 3: panel boundaries (still ImGui)

Each `draw*Panel` / `draw*Dialog` should contain only widget code plus calls to
named App operations. The Qt version then calls the same operations and nothing
else.

1. Pull inline logic out of the ImGui code into named methods, e.g.
   `renameFeature`, `suppressFeature`, `deleteFeature`, `setRollback`,
   `createOffsetPlane`, `createTangentPlane`, `setExtrudeDistance`. Many already
   exist (`commitExtrude`, `cancelExtrude`, `editExtrudeFeature`, ...).
2. **Change counters** on App: `documentRev_` (bumped by `markDirty`, replay, undo,
   redo, load), `selectionRev_`, `toolRev_` and `simRev_`. Retained Qt panels
   compare these each tick. This keeps the "rebuilt from state" property ImGui
   gives today, so undo, replay and project load cannot leave a panel showing
   stale values. Panels do not cache model state between revisions. Two kinds of
   update, because rebuilding on every change would break live editing:
   - **Structural rebuild** only when the set of things changes (features, planes,
     nozzles, profiles added or removed, or the tool switched). Each counter
     carries a separate structural part so panels can tell the difference.
   - **Value refresh** otherwise: update existing widgets in place, and **never
     write into the widget that has focus or is being dragged**. Otherwise a live
     simulation edit, which bumps `simRev_` every frame, would clobber the field
     being typed in.
   - Trees and lists (object tree, nozzle list) update items in place, keyed by
     `FeatureID` / `PlaneID` / nozzle id, so selection, expanded state and scroll
     position survive. No clear-and-refill.
3. **Lazy GPU upload in `Scene3D`.** `addBody` / `replaceBody` / `addMeshBody` store
   the CPU mesh and edges and mark the body dirty; `render()` / `renderEdges()`
   upload dirty bodies. GL resources of removed bodies go on a list that is freed at
   the next render. Replay, undo and commit then never touch GL, which:
   - lets `ReplayTest` and `MeshImportTest` run without a GL context, so removing
     GLFW in Phase 6 does not break the tests;
   - removes the "no current context" crash from Qt slots at its source in
     Phase 4.
   `Scene3D` is the render layer, not the core, so this is allowed by rule 3. Check
   everything that reads a body's GL handles directly (e.g. the simulation result
   VAO and the section cap) still works with an upload that happens later.
4. Separate App from the host: `App::initGL()`, `App::tick(const InputFrame&)`
   (input handling and UI), `App::paint(int w, int h)` (`render3DScene`) and
   `App::shutdownGL()`. `App::init/run` become `GlfwHost`, in `main_glfw.cpp`.
5. **Command queue**: `App::post(std::function<void()>)`. `tick` runs the queue at
   its start. `post()` also asks the host for a redraw (a no-op while rendering is
   continuous), so switching to on-demand rendering later does not leave commands
   waiting for an unrelated event. The ImGui panels can keep calling App directly;
   the queue is for the Qt panels.

**Done when:** `App` has no GLFW calls, the tests run without a GL window, and
the checklist passes.

**Status (done, with two items moved).** Lazy upload is in `Scene3D` (`syncGpu`,
`releaseGpu`, CPU `edges`); `ReplayTest` and `MeshImportTest` now run with no GL
context and check that replay uploaded nothing. `App` has no GLFW calls:
`main.cpp` holds `GlfwHost`, `AppHost.h` is the interface, and App exposes
`init(host, dpi)`, `frame(dt, w, h)`, `paint()`, `post()`, `shutdown()`. The frame
loop keeps its old order. All three input scenarios were pixel-identical to the
pre-migration build.
**Moved to Phase 5:** the change counters (item 2) and extracting named operations
from each panel (item 1). Both are added with the first Qt panel that uses them, so
each is checked by a real consumer rather than written speculatively.

## Phase 4: Qt host, with the existing ImGui UI running inside it

1. Add Qt 6 `qtbase` (features `widgets`, `opengl`) to `vcpkg.json`. In CMake:
   `find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)`, `CMAKE_AUTOMOC`, and
   a new `SHITcadQt` target beside the existing `SHITcad`. Both build until Phase 6.
2. `main_qt.cpp`: `QApplication`, with `QSurfaceFormat` set as the default before it
   (3.3 core, depth 24, **stencil 8**, which the section cap needs). A `QMainWindow`
   whose central widget is `ViewportWidget : QOpenGLWidget`.
3. `ViewportWidget`: GLAD loads through `QOpenGLContext::getProcAddress`. Qt
   mouse, wheel and key events accumulate into the next `InputFrame`.
   `paintGL()` calls `App::tick` then `App::paint`. `frameSwapped -> update()`
   keeps rendering continuous, as today.
4. **ImGui inside the QOpenGLWidget**: write a small ImGui platform backend
   (about 200 lines) that feeds Qt events to `ImGuiIO`, and keep
   `imgui_impl_opengl3` as the renderer. The whole existing UI then runs unchanged
   inside the Qt window. This step is what makes Phase 5 incremental.
5. Pitfalls to handle here:
   - **The default framebuffer is not 0.** QOpenGLWidget renders into an FBO. Use
     `defaultFramebufferObject()` wherever framebuffer 0 is assumed. No code binds
     it today, but future offscreen passes must not.
   - **Qt widgets never call App operations directly.** They `post()` a command,
     and `App::tick` runs the queue inside `paintGL`. After Phase 3's lazy upload,
     a direct call no longer crashes for lack of a GL context, but the queue still
     keeps the order of operations within a frame the same as today and keeps
     App state from changing halfway through a tick.
   - **Re-entrancy, from the first Qt build.** The Win32 `GetOpenFileName` /
     `GetSaveFileName` dialogs that are still in use run their own message loop.
     That loop keeps dispatching Qt events to the Qt window, so `paintGL`, and
     with it `tick`, can run again while a save or open is in progress.
     `QFileDialog::exec` and `QMessageBox` later do the same. Add an `inTick_`
     guard (a nested `paintGL` renders the frame but skips tick and the queue) in
     this phase, not in Phase 5. Better still, open modal dialogs from a queued
     command after the tick finishes.
   - **Logical vs device pixels.** Qt event positions are logical. Multiply by
     `devicePixelRatioF()` when filling `InputFrame`, so picking, projection and
     overlays share one pixel space. Test at 150% Windows scaling.
   - **Focus.** Give the viewport `Qt::StrongFocus`. While ImGui draws inside the
     viewport, `uiWantsMouse` and `uiWantsKeyboard` come from ImGui's own capture
     flags. `uiWantsKeyboard` is also true when a Qt text field has focus. The
     keyboard rules for docks are in Phase 5.1.
   - **DPI changes (known, temporary).** ImGui's fonts are sized once at startup.
     Moving the window to a monitor with different scaling blurs ImGui text until
     Phase 6. Viewport rendering and picking follow `devicePixelRatioF()` and stay
     correct. Accept this rather than rebuild ImGui's font atlas.

**Done when:** `SHITcadQt` passes the full checklist and matches the GLFW build.
From then on the Qt build is the default.

**Status (done, except the manual checklist and a 150% DPI check).** `-DSHITCAD_QT=ON`
builds `SHITcadQt` from `src/qt/` (`main_qt.cpp`, `ViewportWidget`, `ImGuiQt`) with the
same App sources; vcpkg installs a trimmed qtbase 6.11 (the manifest's `qt` feature,
about 26 minutes the first time, cached after that). ImGui is fed in device pixels,
so it sees the coordinates GLFW gave it; the three input scenarios matched the
pre-migration build to within a few anti-aliasing pixels (+-1 colour steps from
blending into the widget's FBO). Checked by hand: modal file dialog mid-frame
(the `inFrame_` guard), window restore/resize, clean exit. Fixes this phase needed:
the native dialogs had no owner window, so under Qt they opened *behind* the main
window; they now take `GetActiveWindow()` as owner (also right under GLFW). Qt
compiles consumers with `/permissive-`, which rejected a `goto` that jumped over an
initialisation in `AppSketch.cpp`. vcpkg copies the Qt DLLs but not plugins, so a
post-build step copies the Windows platform and style plugins. The option stays
**off by default** for now so the plain build does not need Qt; Phase 5 needs it on.
Not yet checked: 150% display scaling, and a full walk of the smoke checklist.

## Phase 5: move panels to Qt, one per commit

Each commit adds the Qt panel, deletes its ImGui version, and runs the checklist
items that cover it. Panels read App state when the change counters move, and they
write only through the command queue. Undo grouping carries over:
`IsItemDeactivatedAfterEdit` becomes `editingFinished` / `sliderReleased` calling
`commitSimulationEdit()`.

Order, from low risk to high:

1. **Toolbar, menus and keyboard routing.** `QToolBar` and `QMenuBar` with
   `QAction`s. Today every shortcut works whenever no text field is active. In Qt,
   clicking a dock (tree, timeline) takes keyboard focus away from the viewport,
   and the shortcuts would stop working unless this commit handles it:
   - **App-wide shortcuts** (Ctrl+Z, Ctrl+Y, Ctrl+Shift+Z, Ctrl+S, Ctrl+O, Ctrl+E)
     become `QAction`s with `Qt::ApplicationShortcut` that `post()` the existing
     operation. **Remove them from the viewport handlers in the same commit**, so
     nothing fires twice. Keep today's modes: if a shortcut is ignored in a mode now,
     the action's handler checks the same condition. Undo and redo **dispatch on the
     mode**: sketch history in a sketch, global history otherwise. They are also
     ignored while the inline dimension input is active, as today.
   - **Mode-dependent keys** (E, T, O, V, Escape, Delete, digits for inline
     dimensions, ...) stay in the viewport handlers. An event filter on the main
     window forwards key presses to the viewport unless the focus widget is a text
     input (`QLineEdit`, `QAbstractSpinBox`, `QTextEdit`) or the key is used by
     the focused widget itself (arrow keys in the tree).
   - Test the whole keyboard section of the checklist twice: once after clicking in
     the viewport, once after clicking in each dock.
   - **Layout persistence**: save and restore window geometry and dock layout
     (`QMainWindow::saveState`/`restoreState`) through `QSettings`. Preferences keep
     their current file.
2. **Preferences**: a `QDialog` editing `prefs_`.
3. **File dialogs**: `QFileDialog`. `QString::toUtf8()` gives UTF-8 directly, so
   `ansiToUtf8` can go. Rerun `testNonAsciiPaths`.
4. **Small dialogs**: Add Reference Plane (including the "click a face" wait
   state), Tangent Plane, Import Mesh (unit choice with its size readout), and
   Mesh Place.
5. **Object tree**: `QTreeWidget` in a dock, updated in place on `documentRev_`
   (keyed items; structural changes add and remove rows, not refill the tree), with
   selection kept in sync in both directions. Guard against loops: a selection
   change that came from App must not post a selection command back.
6. **Tool panels**: Extrude, Revolve, Loft, Boolean and the dimension panel in one
   "tool options" dock, which switches content with `tool_.type`.
7. **Section controls**.
8. **Timeline**: a custom `QWidget` with `paintEvent`: playhead drag with
   the deferred replay kept intact, double-click to edit, context menu with rename,
   suppress and delete, error tooltips.
9. **Simulation panel**: setup, run and results sections. This is the largest.
   Keep `pollSimulationRun()` in `tick`. Here the value-refresh rule matters most:
   live edits bump `simRev_` every frame, so the focused or dragged widget is never
   written back to.
10. **Widgets inside the viewport**: the inline dimension input and the dimension
    value box become a frameless `QLineEdit` child of `ViewportWidget`, placed at
    the label. Child widgets composite correctly over a QOpenGLWidget; they would
    not over a native `QWindow` container, which is why the host uses QOpenGLWidget.
    This removes the `GImGui->ActiveId` hack.

Note: once the panels dock, the viewport becomes a smaller rectangle and no
longer covers the whole window. `InputFrame.viewportW/H` handles that. Check aspect
ratio and picking near the edges.

**Done when:** no `ImGui::` widget calls remain.

**Progress.**
- 5.1 toolbar: done. `ToolbarModel` + `UiAction` / `App::perform` (in `AppUI.cpp`) hold what
  the toolbar shows and does; the ImGui toolbar was rewritten onto them and stayed
  pixel-identical to the pre-migration build (a scenario clicking buttons in all three
  variants), and `src/qt/Toolbar.cpp` is the Qt front end (rebuilt when the variant
  changes, states re-applied every frame, clicks posted). The Qt host calls
  `App::setHostToolbar(true)`, so the ImGui toolbar is skipped there and the input rect
  starts at the top of the viewport. Toolbar buttons are `Qt::NoFocus`, so the viewport
  keeps the keyboard and the shortcut work in 5.1 is not needed yet: it moves to the
  first dock that can take focus (the object tree, 5.5). The ImGui tool panels are
  still placed 30 px down, a small gap in the Qt build until they move (5.6).
- 5.2 preferences: done. The `Preferences` struct is the model; `App::setPreferences`
  applies the side effects the ImGui window used to apply inline (the ImGui window now
  edits a copy and hands it over, pixel-identical to before). `src/qt/PreferencesDialog`
  is a non-modal dialog shown while `preferencesOpen()`; it pushes edits through
  `post` and pulls App values back when they differ (light mode resets the colours).
  Host panels are now a bit set (`App::setHostPanel`).
- 5.3 file dialogs: done. `FileDialogs.h` lists every chooser portably; App asks the
  host (`AppHost::chooseFile`) and falls back to the Win32 dialogs, so the GLFW build
  is unchanged. The Qt host uses `QFileDialog` (native on Windows, Unicode paths, no
  ANSI step): saving to and opening from `...\Größe\` worked. Found while testing: keys
  held when a modal dialog opens never see their release, and ImGui auto-repeated them
  once it closed (Ctrl+O reopened Open). `ImGuiQt::releaseAll` now runs before a dialog
  and whenever the window deactivates, as GLFW does on focus loss. Pre-existing and
  left alone: `O` toggles the projection even with Ctrl held, so Ctrl+O also flips it.
- 5.4 mesh dialogs: done (the two plane dialogs are still to come). `MeshImportModel`
  and `MeshPlaceModel` plus named operations (rotate about the centre, drop to ground,
  centre on origin, set unit/position) hold what those panels do; the ImGui panels were
  rewritten onto them and stayed pixel-identical through an import, a rotation, a
  drop to ground, Done and an undo. `src/qt/MeshDialogs.cpp` is the Qt front end.
  `App::validateMeshPlace` now runs every frame: dropping the panel when its feature is
  deleted or undone used to happen while the ImGui panel drew itself.

## Phase 6: remove ImGui and GLFW

1. `QtOverlay` draws the recorded `Overlay2D` list with `QPainter` on the viewport
   after the GL pass, and implements `measureText` with `QFontMetricsF`. Because
   the list is recorded (Phase 2), nothing that produces overlays changes.
2. Delete the ImGui backend, `ImGuiOverlay`, `fillInputFromImGui`, `GlfwHost` and
   the `SHITcad` GLFW target. Remove ImGui and GLFW from `cmake/Dependencies.cmake`.
   The tests need no GL context after Phase 3, so remove the hidden GLFW window
   from `MeshImportTest` as well. If a future test does need GL, use
   `QOffscreenSurface`.
3. Themes: `Preferences::applyTheme()` sets a Qt palette (Fusion style, dark and
   light).
4. Update `CLAUDE.md` (build, architecture, the ImGui-specific notes) and the README.

## Phase 7: platform layer and cross-platform build (can start after Phase 4)

| Today (Windows-only) | Replacement |
|---|---|
| `SimProcess.cpp`: `CreateProcessW`, pipes, kill-on-close job object | Keep the interface. Windows keeps the job object; POSIX gets a process-group version (untested). Or use `QProcess` plus the job object on Windows |
| `AppSimulationRun.cpp`: `%APPDATA%`, `%TEMP%`, `ShellExecute` (ParaView), `python` | `QStandardPaths`, `QProcess::startDetached`; default interpreter `python3` off Windows |
| `CrashLogger.cpp`: dbghelp minidumps | `#ifdef _WIN32`; stub elsewhere |
| CMake: `opengl32`, `dbghelp` | `find_package(OpenGL)`; dbghelp only on `WIN32` |
| System font paths in `App::init` | Qt's default font |

Add a Linux GitHub Actions job that **only compiles**, so drift shows up without
anyone running the app on Linux. Like the Windows job, it needs vcpkg binary
caching, or every run spends an hour or more building OCCT and Qt.

## Risks to watch

- **Input semantics drifting** in Phase 1: key repeat, double-click timing, drag
  threshold. Test these directly with the checklist, not just "it still works".
- **Stale panels** in Phase 5: every panel must follow the change counters and
  keep no model state of its own. This is the main new hazard a retained UI brings.
  The opposite failure, a refresh overwriting what the user is editing, is ruled
  out by the value-refresh rule in Phase 3.
- **Shortcuts dying when a dock has focus**: covered by Phase 5.1. Test with focus
  in each dock, not only in the viewport.
- **GL context and re-entrancy** in Phase 4: lazy upload (Phase 3) removes the
  context crash, and the `inTick_` guard plus the command queue cover
  re-entrancy. A direct `App` call from a Qt slot is still a bug.

## Rough size

Phases 0-3 are about a quarter of the work and change nothing a user can see.
Phase 4 is small: the host, a roughly 200-line ImGui backend, and the pitfalls
above. Phase 5 is most of the work; the timeline and the simulation panel are the
largest items. Phases 6 and 7 are small once Phase 5 is done.
- **Build time**: the first vcpkg build of `qtbase` takes roughly 30-60 minutes;
  cached after that.
- **Licensing**: LGPLv3, linked dynamically. Ship the Qt DLLs (`windeployqt`). Use
  only `qtbase` modules.
