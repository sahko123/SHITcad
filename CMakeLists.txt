cmake_minimum_required(VERSION 3.20)
project(SHITcad VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/Dependencies.cmake)

add_executable(SHITcad
    src/main.cpp
    src/App.cpp
    src/AppSketch.cpp
    src/AppUI.cpp
    src/AppDimension.cpp
    src/AppExtrude.cpp
    src/AppRevolve.cpp
    src/AppLoft.cpp
    src/AppBoolean.cpp
    src/SketchData.cpp
    src/Tools.cpp
    src/Snap.cpp
    src/History.cpp
    src/AutoConstraint.cpp
    src/Solver.cpp
    src/HitTest.cpp
    src/ShaderProgram.cpp
    src/Viewport3D.cpp
    src/Scene3D.cpp
    src/ProfileDetector.cpp
    src/Extrude.cpp
    src/SketchPlane.cpp
    src/SketchRenderer.cpp
    src/FacePicker.cpp
    src/MeshImport.cpp
    src/Simulation.cpp
    src/SimProcess.cpp
    src/SimResults.cpp
    src/AppSection.cpp
    src/AppSimulation.cpp
    src/AppSimulationRun.cpp
    src/ExtrudeTool.cpp
    src/Preferences.cpp
    src/Intersect.cpp
    src/FeatureHistory.cpp
    src/FeatureReplay.cpp
    src/UndoStack.cpp
    src/Serialization.cpp
    src/CrashLogger.cpp
)

target_include_directories(SHITcad PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(SHITcad PRIVATE imgui opengl32 occt_libs nlohmann_json::nlohmann_json dbghelp)

if(MSVC)
    target_compile_options(SHITcad PRIVATE /W4)
endif()

# Opt-in tests: cmake -B build -DSHITCAD_BUILD_TESTS=ON
option(SHITCAD_BUILD_TESTS "Build test executables" OFF)
if(SHITCAD_BUILD_TESTS)
    # Everything except the App UI layer and main().
    get_target_property(_shitcad_sources SHITcad SOURCES)
    list(FILTER _shitcad_sources EXCLUDE REGEX "src/(main|App[A-Za-z]*)\\.cpp$")

    foreach(_test MeshImportTest SimulationTest ReplayTest)
        add_executable(${_test} tests/${_test}.cpp ${_shitcad_sources})
        target_include_directories(${_test} PRIVATE ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(${_test} PRIVATE imgui opengl32 occt_libs nlohmann_json::nlohmann_json dbghelp)
    endforeach()
endif()
