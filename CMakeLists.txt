cmake_minimum_required(VERSION 3.20)

# The Qt host is opt-in while the migration is in progress
# (docs/qt-migration-plan.md). It must be decided before project(), which is
# when vcpkg installs the manifest's dependencies.
option(SHITCAD_QT "Also build the Qt host, SHITcadQt (installs qtbase through vcpkg)" OFF)
if(SHITCAD_QT)
    list(APPEND VCPKG_MANIFEST_FEATURES "qt")
endif()

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

if(SHITCAD_QT)
    find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)

    # Same App sources as SHITcad, with the Qt host instead of the GLFW one.
    get_target_property(_qt_app_sources SHITcad SOURCES)
    list(REMOVE_ITEM _qt_app_sources src/main.cpp)
    add_executable(SHITcadQt
        ${_qt_app_sources}
        src/qt/main_qt.cpp
        src/qt/ViewportWidget.cpp
        src/qt/ImGuiQt.cpp
        src/qt/Toolbar.cpp
        src/qt/PreferencesDialog.cpp
        src/qt/MeshDialogs.cpp
        src/qt/PlaneDialogs.cpp
        src/qt/ObjectTree.cpp
        src/qt/KeyRouting.cpp
        src/qt/ToolPanel.cpp
        src/qt/SectionControls.cpp
        src/qt/Timeline.cpp
        src/qt/SimulationPanel.cpp
        src/qt/InViewport.cpp
    )
    set_target_properties(SHITcadQt PROPERTIES AUTOMOC ON)
    target_include_directories(SHITcadQt PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/qt)
    target_link_libraries(SHITcadQt PRIVATE imgui opengl32 occt_libs nlohmann_json::nlohmann_json dbghelp
                          Qt6::Widgets Qt6::OpenGLWidgets)
    if(MSVC)
        target_compile_options(SHITcadQt PRIVATE /W4)
    endif()

    # vcpkg copies the Qt DLLs next to the exe but not the plugins Qt loads at
    # run time; without the platform plugin the app cannot open a window.
    if(WIN32 AND DEFINED VCPKG_INSTALLED_DIR)
        set(_qt_plugins "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/$<$<CONFIG:Debug>:debug/>Qt6/plugins")
        set(_qt_d "$<$<CONFIG:Debug>:d>")
        add_custom_command(TARGET SHITcadQt POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:SHITcadQt>/platforms" "$<TARGET_FILE_DIR:SHITcadQt>/styles"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_qt_plugins}/platforms/qwindows${_qt_d}.dll" "$<TARGET_FILE_DIR:SHITcadQt>/platforms/"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_qt_plugins}/styles/qmodernwindowsstyle${_qt_d}.dll" "$<TARGET_FILE_DIR:SHITcadQt>/styles/"
            VERBATIM)
    endif()
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
