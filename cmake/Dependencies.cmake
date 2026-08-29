include(FetchContent)

# --- GLFW ---
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)

# --- glad (OpenGL loader) ---
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG        v2.0.6
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  cmake
)

# --- Dear ImGui ---
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.9
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(glfw glad)

# glad generates its loader at build time with a Python script that needs
# Jinja2. Check for it now so a missing module fails here with a clear message
# instead of as an opaque custom-build error partway through the build.
find_package(Python COMPONENTS Interpreter REQUIRED)
execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import jinja2"
    RESULT_VARIABLE _jinja2_missing
    OUTPUT_QUIET ERROR_QUIET
)
if(_jinja2_missing)
    message(FATAL_ERROR
        "The glad OpenGL loader generator requires the Python module 'jinja2', "
        "which was not found in ${Python_EXECUTABLE}.
"
        "Install it with:
"
        "    ${Python_EXECUTABLE} -m pip install jinja2
"
        "then re-run the configure step.")
endif()

# glad: generate the loader library
glad_add_library(glad_gl REPRODUCIBLE LOADER API gl:core=3.3)

# ImGui: populate manually (no CMakeLists.txt in upstream repo)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui PUBLIC glfw glad_gl)

# --- nlohmann/json (header-only) ---
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

# --- OpenCASCADE (via vcpkg) ---
find_package(OpenCASCADE CONFIG REQUIRED
    COMPONENTS FoundationClasses ModelingData ModelingAlgorithms)
message(STATUS "OpenCASCADE found: ${OpenCASCADE_INCLUDE_DIR}")

add_library(occt_libs INTERFACE)
target_include_directories(occt_libs INTERFACE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(occt_libs INTERFACE
    TKernel TKMath TKG3d TKG2d TKGeomBase TKGeomAlgo
    TKBRep TKPrim TKTopAlgo TKShHealing TKMesh TKBO TKBool TKOffset
)

# Data exchange libraries (not part of core components, link directly)
foreach(_lib TKDESTL TKXSBase TKDEStep TKDEIges)
    find_library(${_lib}_LIB NAMES ${_lib} PATHS "${OpenCASCADE_LIBRARY_DIR}" NO_DEFAULT_PATH)
    find_library(${_lib}_LIB_DEBUG NAMES ${_lib} PATHS "${OpenCASCADE_LIBRARY_DIR}/../debug/lib" NO_DEFAULT_PATH)
    if(${_lib}_LIB)
        target_link_libraries(occt_libs INTERFACE
            $<$<CONFIG:Debug>:${${_lib}_LIB_DEBUG}>
            $<$<NOT:$<CONFIG:Debug>>:${${_lib}_LIB}>)
    else()
        message(WARNING "${_lib} not found - some import/export formats will be unavailable")
    endif()
endforeach()
