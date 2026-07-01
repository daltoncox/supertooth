cmake_minimum_required(VERSION 3.10)

project(supertooth C)

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