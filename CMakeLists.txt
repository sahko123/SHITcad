cmake_minimum_required(VERSION 3.20)
project(SHITcad VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/Dependencies.cmake)

add_executable(SHITcad
    src/main.cpp
    src/App.cpp
    src/Canvas.cpp
    src/SketchData.cpp
    src/Tools.cpp
    src/Snap.cpp
    src/History.cpp
    src/AutoConstraint.cpp
    src/Solver.cpp
)

target_include_directories(SHITcad PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(SHITcad PRIVATE imgui opengl32 libslvs)

if(MSVC)
    target_compile_options(SHITcad PRIVATE /W4)
endif()
