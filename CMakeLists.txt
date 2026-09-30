cmake_minimum_required(VERSION 3.20)

project(SHITcad VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/Dependencies.cmake)
find_package(Qt6 REQUIRED COMPONENTS Gui Widgets OpenGLWidgets)

# Everything below the UI: sketching, solver, features, replay, geometry,
# rendering, simulation set-up and I/O. The tests build on this too.
set(SHITCAD_CORE_SOURCES
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
    src/CadImport.cpp
    src/Simulation.cpp
    src/SimProcess.cpp
    src/SimResults.cpp
    src/ExtrudeTool.cpp
    src/Preferences.cpp
    src/Intersect.cpp
    src/FeatureHistory.cpp
    src/FeatureReplay.cpp
    src/UndoStack.cpp
    src/Serialization.cpp
    src/CrashLogger.cpp
)

add_executable(SHITcad
    ${SHITCAD_CORE_SOURCES}
    # App: the viewport's input handling, tools and panel models
    src/App.cpp
    src/AppSketch.cpp
    src/AppUI.cpp
    src/AppDimension.cpp
    src/AppExtrude.cpp
    src/AppRevolve.cpp
    src/AppLoft.cpp
    src/AppBoolean.cpp
    src/AppSection.cpp
    src/AppSimulation.cpp
    src/AppSimulationRun.cpp
    # Qt: the window, the viewport widget and the panels
    src/qt/main.cpp
    src/qt/ViewportWidget.cpp
    src/qt/QtInput.cpp
    src/qt/QtOverlay.cpp
    src/qt/KeyRouting.cpp
    src/qt/Toolbar.cpp
    src/qt/PreferencesDialog.cpp
    src/qt/MeshDialogs.cpp
    src/qt/PlaneDialogs.cpp
    src/qt/ObjectTree.cpp
    src/qt/ToolPanel.cpp
    src/qt/SectionControls.cpp
    src/qt/Timeline.cpp
    src/qt/SimulationPanel.cpp
    src/qt/InViewport.cpp
)
set_target_properties(SHITcad PROPERTIES AUTOMOC ON)
target_include_directories(SHITcad PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/qt)
target_link_libraries(SHITcad PRIVATE glad_gl opengl32 occt_libs nlohmann_json::nlohmann_json dbghelp
                      Qt6::Widgets Qt6::OpenGLWidgets)
if(MSVC)
    target_compile_options(SHITcad PRIVATE /W4)
endif()

# vcpkg copies the Qt DLLs next to the exe but not the plugins Qt loads at run
# time; without the platform plugin nothing can open a window (or, for the
# tests, an offscreen GL context).
if(WIN32 AND DEFINED VCPKG_INSTALLED_DIR)
    set(_qt_plugins "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/$<$<CONFIG:Debug>:debug/>Qt6/plugins")
    set(_qt_d "$<$<CONFIG:Debug>:d>")
    add_custom_command(TARGET SHITcad POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:SHITcad>/platforms"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_qt_plugins}/platforms/qwindows${_qt_d}.dll" "$<TARGET_FILE_DIR:SHITcad>/platforms/"
        VERBATIM)
endif()

# Opt-in tests: cmake -B build -DSHITCAD_BUILD_TESTS=ON
option(SHITCAD_BUILD_TESTS "Build test executables" OFF)
if(SHITCAD_BUILD_TESTS)
    foreach(_test MeshImportTest SimulationTest ReplayTest)
        add_executable(${_test} tests/${_test}.cpp ${SHITCAD_CORE_SOURCES})
        target_include_directories(${_test} PRIVATE ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(${_test} PRIVATE glad_gl opengl32 occt_libs nlohmann_json::nlohmann_json dbghelp)
    endforeach()
    # SimulationTest compiles the shaders and picks through a section, which
    # needs a GL context: an offscreen one from Qt.
    target_link_libraries(SimulationTest PRIVATE Qt6::Gui)
endif()
