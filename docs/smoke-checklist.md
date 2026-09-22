# Smoke checklist

A manual pass over every way of using the app, for the Qt migration
(`docs/qt-migration-plan.md`). The automated tests (`ReplayTest`, `MeshImportTest`,
`SimulationTest`) cover what lies under the UI; this list covers what they cannot:
input, panels, and what shows in the viewport.

**How to use it:** run the whole list once on the build before a phase starts, to
record the baseline, and again when the phase is done. After each commit inside a
phase, run the sections that commit touches. Anything that behaves differently from
the baseline is a failure, even if the new behaviour looks better. Record it and
decide on it separately.

Where a key is listed, test it twice once Qt docks exist: after clicking in the
viewport, and after clicking in a dock (object tree, timeline).

## 1. Camera and view

- [ ] Middle-drag orbits; Shift + middle-drag pans; the wheel zooms toward the view.
- [ ] `O` toggles orthographic/perspective (in any mode, when no text field is active).
- [ ] `T` toggles the object tree (Navigate only).
- [ ] The camera animates when entering a sketch, and `N` in a sketch re-orients to the plane.
- [ ] FPS readout at bottom right, above the timeline when that is showing.
- [ ] Resizing the window keeps the aspect correct and picking accurate at all four edges.
- [ ] At 150% Windows display scaling: text sharp, clicks land where the cursor is.

## 2. Sketching

- [ ] Double-click a reference plane: enters a sketch, and the camera turns to face it.
- [ ] Double-click a body face: enters a sketch on that face with its edges projected.
- [ ] Tools by key: `P` point, `L` line, `C` circle, `R` rectangle, `Shift+R` centre
      rectangle, `A` 3-point arc, `Shift+A` centre arc, `D` dimension, `F` fillet.
      Also from the toolbar.
- [ ] Line chain: consecutive clicks chain; `Escape` ends the chain but stays in the tool;
      a second `Escape` drops to no tool; a third leaves the sketch.
- [ ] Snaps: endpoint, midpoint, centre, on-curve, H/V rails; the snap indicator shows.
- [ ] Automatic constraints appear while drawing (H/V, coincident).
- [ ] Inline dimension while drawing: type digits (including the numpad and `.`) and
      the value box appears at the cursor; `Enter` applies it; `Backspace` edits it;
      `Escape` cancels it.
- [ ] Click select, Ctrl-click to add; box select left-to-right; lasso select.
- [ ] Drag a point: it follows the cursor within the constraints; the DOF count updates.
- [ ] `Delete` / `Backspace` removes the selection and its dependent constraints.
- [ ] Sketch undo/redo: `Ctrl+Z`, `Ctrl+Y`, `Ctrl+Shift+Z` act on the **sketch**
      history while in a sketch, not the global one.
- [ ] Scale ruler at bottom left, rounded to 1/2/5 x 10^n, updating with zoom.
- [ ] Transient sketch messages (e.g. a rejected constraint) show, then fade.
- [ ] Leaving the sketch records one sketch feature on the timeline.

## 3. Constraints and dimensions

- [ ] Each geometric constraint from the toolbar on a valid selection: coincident,
      horizontal, vertical, parallel, perpendicular, collinear, tangent, equal, symmetric,
      concentric, midpoint, point on line, point on circle.
- [ ] A conflicting constraint is refused with a message, and the geometry is unchanged.
- [ ] Dimension tool (`D`): line length, point-point distance, point-line distance, radius,
      diameter, angle. The value box opens with the measured value; typing a unit works
      (`1in`, `2 cm`); `Enter` applies; `Escape` cancels.
- [ ] Angle dimension: moving the mouse flips between acute and reflex until you start typing.
- [ ] Driven (reference) dimensions show in their own colour and are not enforced.
- [ ] Dimension labels: click one to edit its value; drag one to move it.
- [ ] A dimension's unit and typed value survive save/load.

## 4. 3D features

- [ ] `E` enters extrude (from Navigate or a sketch); click profiles to select them.
- [ ] Extrude: drag the arrow handle (the length label follows); type a distance; offset;
      direction (one side, other side, both, symmetric); new body vs cut;
      `Enter` / `Keypad Enter` commits; `Escape` cancels. The preview updates live.
- [ ] `V` enters revolve: pick profile and axis line, set the angle, commit and cancel.
- [ ] Loft: pick sections on two or more planes, solid or shell, commit and cancel.
- [ ] Boolean union and subtract: pick target and tool, preview, `Enter` commits,
      `Escape` cancels.
- [ ] A failing feature shows its error on the timeline (tooltip) and does not crash.

## 5. Timeline, history and undo

- [ ] Drag the playhead: the model rolls back live; releasing records one undo step.
- [ ] Double-click a feature to edit it (sketch, extrude, revolve, loft, mesh placement).
- [ ] Right-click a feature: rename, suppress/unsuppress, delete (dependents go with it),
      Rotate / Move for mesh imports.
- [ ] `Delete` on a selected timeline feature deletes it.
- [ ] Global undo/redo in Navigate (`Ctrl+Z`, `Ctrl+Y`, `Ctrl+Shift+Z`) steps through
      feature adds, deletes, suppresses, edits, renames, mesh placement and simulation edits.
- [ ] Object tree: planes, sketches and bodies listed; visibility toggles work; selecting
      there matches the viewport.

## 6. Planes

- [ ] Add Reference Plane at an offset from an existing plane: name, offset (typed with
      `Enter`); a preview shows.
- [ ] Add Reference Plane from a face: the dialog waits, you click a face, the plane appears.
- [ ] Tangent plane on a cylinder: the angle slider and the typed angle both move the
      preview; Create & Sketch enters a sketch on it.

## 7. Import, export and files

- [ ] `Ctrl+S` saves (a dialog the first time); `Ctrl+O` opens; the window title shows the
      file name and an unsaved-changes mark.
- [ ] A path with non-ASCII characters (e.g. `C:\Temp\Größe\test.shitcad`) saves and opens.
- [ ] Import STL: the unit dialog shows the resulting size for each unit, and the
      name field works.
- [ ] Mesh placement panel: rotate by axis and angle, quarter turns, move, drop to
      ground. Done records one undo step; Cancel restores.
- [ ] Hovering an imported mesh shows the pick readout; it does not show while the cursor
      is over the toolbar or a panel.
- [ ] Import STEP and IGES.
- [ ] Export STL (`Ctrl+E` in Navigate), STEP, IGES, OBJ, DXF (from a sketch).

## 8. Simulation workspace

- [ ] The Model/Simulation tabs switch only from Navigate with no tool active.
- [ ] Surface roles set per imported mesh.
- [ ] Place a nozzle: click Place, click inside the vessel; the spray cone shows.
      `Escape` cancels placing; `Delete` removes the selected nozzle.
- [ ] Edit nozzle fields: a slider drag or a typed edit is **one** undo step.
- [ ] Global shortcuts work in this workspace too (`Ctrl+Z/Y/S/O`).
- [ ] Engine settings: the cip-sim folder and python path persist across restarts.
- [ ] Run: the log streams, Cancel stops it, and a finished run loads results on the geometry.
- [ ] Results: field selection, legend, colours (grey = no data, amber = sprayed but
      unsampled); stale marking after moving a mesh or editing a nozzle.
- [ ] Open in ParaView launches it; closing SHITcad leaves ParaView open.
- [ ] Opening another project clears the previous run's results and numbers.

## 9. Section view

- [ ] Model workspace: the toolbar's Section button opens the controls; Simulation: under View.
- [ ] Cutting through solids and meshes: closed ones show a filled cap; an open surface
      does not flood the view with the cap colour.
- [ ] Picking and hover ignore the cut-away side.
- [ ] The ground grid and spray cones stay whole.

## 10. Preferences

- [ ] Light and dark themes switch the whole UI and the viewport background.
- [ ] Sketch line colour/thickness, edge colour/thickness, dimension colours and the
      wireframe toggle all take effect immediately.
