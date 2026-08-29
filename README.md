# SHITcad

A parametric CAD modeler. Sketch 2D profiles with geometric and dimensional
constraints, extrude / revolve / loft them into 3D solids, and edit anything
afterwards through a full feature history with undo/redo and timeline playback.

Built with C++17, OpenGL 3.3, Dear ImGui, and OpenCASCADE.

## Features

- **2D sketching** — lines, rectangles, circles, arcs, ellipses, splines, with
  snapping and automatic constraint inference
- **Constraint solver** — Newton-Raphson solver supporting coincident,
  horizontal, vertical, distance, radius, diameter, point-on-line, equal length,
  perpendicular, parallel, tangent, angle, symmetric, concentric and midpoint
  constraints, with live DOF reporting
- **3D features** — extrude, revolve, loft and boolean operations built on
  OpenCASCADE
- **Feature history** — linear timeline with rollback ("what-if") playhead,
  editable parameters, and full model replay from history
- **Units** — all internal values are millimeters; input accepts 11 unit types
- **File I/O** — JSON project format; export to STL, STEP, IGES, OBJ and DXF;
  import from STL, STEP and IGES

## Requirements

- **Windows** (uses Win32 file dialogs and `dbghelp` for crash logging)
- **Visual Studio 2019 or 2022** with the C++ desktop workload (MSVC, C++17)
- **CMake 3.20+**
- **[vcpkg](https://github.com/microsoft/vcpkg)** — supplies OpenCASCADE
- **Python 3** with **Jinja2** (`pip install jinja2`) — glad generates the
  OpenGL loader at build time with a Python script. The configure step checks
  for this and tells you if it's missing.
- A GPU/driver supporting **OpenGL 3.3 core**

GLFW 3.4, glad, Dear ImGui 1.91.9 and nlohmann/json 3.11.3 are fetched
automatically by CMake at configure time — no manual setup needed.

## Building

### 1. Get vcpkg (skip if you already have it)

```bash
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg && ./bootstrap-vcpkg.bat
```

Also make sure Jinja2 is available to the Python that CMake will find:

```bash
pip install jinja2
```

### 2. Configure and build

```bash
git clone https://github.com/sahko123/SHITcad.git
cd SHITcad
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

`vcpkg.json` declares OpenCASCADE as a manifest dependency, so the configure
step installs it automatically. **The first configure builds OpenCASCADE from
source and can take upwards of an hour**; subsequent builds reuse vcpkg's binary
cache and are fast.

### 3. Run

```bash
./build/Release/SHITcad.exe
```

vcpkg copies the OpenCASCADE DLLs next to the executable as part of the build,
so it runs in place with no extra setup.

## Repository layout

| Path | Contents |
|------|----------|
| `src/` | All application source. The `App` class is split across `App*.cpp` files by responsibility (sketching, UI, dimensions, extrude, revolve, loft, boolean). |
| `cmake/Dependencies.cmake` | FetchContent declarations and the OpenCASCADE target |
| `vcpkg.json` | vcpkg manifest pinning the OpenCASCADE dependency |
| `CLAUDE.md` | Detailed architecture notes and conventions |

See [CLAUDE.md](CLAUDE.md) for the architecture walkthrough — data flow, the ID
system, the sketch data model, the solver, and how to add new entity types,
constraints or 3D features.

## Troubleshooting

**`Could not find a package configuration file provided by "OpenCASCADE"`**
The toolchain file wasn't passed, or the path to it is wrong. Re-run the
configure step with the correct `-DCMAKE_TOOLCHAIN_FILE=` path, deleting the
`build/` directory first.

**`The glad OpenGL loader generator requires the Python module 'jinja2'`**
Run `pip install jinja2` for the Python interpreter named in the error message,
then configure again.

**The window opens black or the app exits immediately**
The GPU or driver doesn't expose OpenGL 3.3 core. Check
`build/<config>/SHITcad_stderr.log` for the GL initialization error.

**Crash on startup**
A crash dump and log are written next to the executable by `CrashLogger`.
