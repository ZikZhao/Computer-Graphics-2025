# Invoking the CMake build from the command line is a two step process, first, *generate* a build by running the following:
#
#   cmake -Bbuild -H. -DCMAKE_BUILD_TYPE=Release
# 
# Where `Release` can be replaced with `Debug`, which contains gdb and address sanitiser definitions already written for you.
# Once a build is created, proceed with *compilation*:
# 
#   cmake --build build --target RedNoise --config Release # optionally, for parallel build, append -j $(nproc)
#
# This creates the executable in the build directory. You only need to *generate* a build if you modify the CMakeList.txt file.
# For any other changes to the source code, simply recompile.


cmake_minimum_required(VERSION 3.12)
project(CG-CW)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)

# Note, we do this for glm because it's a header only library and because we shipped it with the project
# normally you would use find_package(<package_name>) for libraries with actual objects
set(GLM_INCLUDE_DIRS libs/glm-0.9.7.2)

find_package(SDL2 REQUIRED)
find_package(Threads REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} libs)
# include_directories(libs/sdw)

add_executable(CG-CW
        src/main.cpp
        src/window.cpp
        src/world.cpp
        src/raytracer.cpp
        src/rasterizer.cpp
        src/renderer.cpp
        src/photon_map.cpp
        src/video_recorder.cpp)

if (MSVC)
    target_compile_options(CG-CW
            PUBLIC
            /W3
            /Zc:wchar_t
            )
    set(DEBUG_OPTIONS /MTd)
    set(RELEASE_OPTIONS /MT /GF /Gy /O2 /fp:fast)
    if (NOT DEFINED SDL2_LIBRARIES)
        set(SDL2_LIBRARIES SDL2::SDL2 SDL2::SDL2main)
    endif()
else ()
    target_compile_options(CG-CW
        PUBLIC
        -Wall
        -Wextra
        -Wcast-align
        -Wfatal-errors
        -Werror=return-type
        -Wno-unused-parameter
        -Wno-unused-variable
        -Wno-ignored-attributes)

    set(DEBUG_OPTIONS -fno-omit-frame-pointer -g)
    set(RELEASE_OPTIONS -O3 -march=native -mtune=native)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -static-libgcc")
    target_link_libraries(CG-CW PRIVATE Threads::Threads)
    target_link_libraries(CG-CW PUBLIC $<$<CONFIG:Debug>:-Wl,-lasan>)

endif()


target_compile_options(CG-CW PUBLIC "$<$<CONFIG:RelWithDebInfo>:${RELEASE_OPTIONS}>")
target_compile_options(CG-CW PUBLIC "$<$<CONFIG:Release>:${RELEASE_OPTIONS}>")
target_compile_options(CG-CW PUBLIC "$<$<CONFIG:Debug>:${DEBUG_OPTIONS}>")
 
target_link_libraries(CG-CW PRIVATE ${SDL2_LIBRARIES})

# Create a symlink to compile_commands.json in the build root for tools like clang-tidy and clangd
if (CMAKE_EXPORT_COMPILE_COMMANDS)
    set(BUILD_ROOT "${CMAKE_SOURCE_DIR}/build")
    file(MAKE_DIRECTORY "${BUILD_ROOT}")
    get_filename_component(REAL_BINARY_DIR "${CMAKE_BINARY_DIR}" ABSOLUTE)
    get_filename_component(REAL_BUILD_ROOT "${BUILD_ROOT}" ABSOLUTE)
    
    if (NOT "${REAL_BINARY_DIR}" STREQUAL "${REAL_BUILD_ROOT}")
        file(CREATE_LINK 
            "${CMAKE_BINARY_DIR}/compile_commands.json" 
            "${BUILD_ROOT}/compile_commands.json" 
            SYMBOLIC
        )
        message(STATUS "Symlinked compile_commands.json to ${BUILD_ROOT}")
    endif()
endif()
