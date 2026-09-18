cmake_minimum_required(VERSION 3.10)

project(supertooth C)

# Default to an optimized build when the user doesn't specify one explicitly
# (e.g. plain `cmake ..` previously meant unoptimized -O0). Single-config
# generators only; multi-config generators (Visual Studio, Xcode) and explicit
# -DCMAKE_BUILD_TYPE=... are left untouched.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD_TESTS "Build the unit and integration test suite" OFF)
option(BUILD_GUI "Build the Qt graphical interface" OFF)

enable_testing()

# Main source directory
add_subdirectory(src)

# Unit / integration tests, opt-in via -DBUILD_TESTS=ON. Run with
# `ctest` once built, or `make tests` (or `ninja tests`).
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()