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
