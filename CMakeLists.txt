cmake_minimum_required(VERSION 3.16)

project(supertooth C CXX)

# Default to an optimized build when the user doesn't specify one explicitly
# (e.g. plain `cmake ..` previously meant unoptimized -O0). Single-config
# generators only; multi-config generators (Visual Studio, Xcode) and explicit
# -DCMAKE_BUILD_TYPE=... are left untouched.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD_TESTS "Build the unit and integration test suite" OFF)
option(BUILD_GUI "Build the Qt graphical interface" OFF)
option(ENABLE_HACKRF "Enable HackRF radio backend" ON)
option(ENABLE_BLADERF "Enable bladeRF radio backend" OFF)
option(SUPERTOOTH_PORTABLE "Portable binary: vendored liquid-dsp without AVX512/AMX codegen (keeps AVX2 and below). CI release enables this." OFF)
option(SUPERTOOTH_USE_SYSTEM_LIQUID "Link system liquid-dsp instead of the FetchContent-vendored static build" OFF)

enable_testing()

# Non-portable (default/dev) builds tune for the build machine. Checked per
# language; MSVC and older compilers simply fail the check and skip it.
if(NOT SUPERTOOTH_PORTABLE)
    include(CheckCCompilerFlag)
    include(CheckCXXCompilerFlag)
    check_c_compiler_flag(-march=native SUPERTOOTH_HAVE_MARCH_NATIVE_C)
    check_cxx_compiler_flag(-march=native SUPERTOOTH_HAVE_MARCH_NATIVE_CXX)
    mark_as_advanced(SUPERTOOTH_HAVE_MARCH_NATIVE_C SUPERTOOTH_HAVE_MARCH_NATIVE_CXX)
    if(SUPERTOOTH_HAVE_MARCH_NATIVE_C)
        add_compile_options($<$<COMPILE_LANGUAGE:C>:-march=native>)
    endif()
    if(SUPERTOOTH_HAVE_MARCH_NATIVE_CXX)
        add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-march=native>)
    endif()
endif()

# ---------------------------------------------------------------------------
# liquid-dsp (statically vendored via FetchContent)
# ---------------------------------------------------------------------------
# Mirrors the flags previously used in packaging/build-liquid.sh (since
# removed): static-only build, no examples/autotests/benchmarks/sandbox/doc,
# SIMD runtime dispatch enabled, hot-path logging compiled out at
# LOGGING_LEVEL=error. With SUPERTOOTH_PORTABLE=ON (x86 only), liquid's SIMD
# probe is skipped and the pre-AVX512 baseline (SSE..AVX2) is declared
# present, so the binary keeps AVX2 but never contains AVX512/AMX.
if(NOT SUPERTOOTH_USE_SYSTEM_LIQUID)
    include(FetchContent)
    FetchContent_Declare(
        liquid
        GIT_REPOSITORY https://github.com/jgaeddert/liquid-dsp.git
        GIT_TAG v1.8.3
        GIT_SHALLOW TRUE
    )
    set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "liquid-dsp: static only" FORCE)
    set(BUILD_STATIC_LIBS ON CACHE INTERNAL "liquid-dsp: static only" FORCE)
    set(BUILD_EXAMPLES OFF CACHE INTERNAL "Disable building liquid examples" FORCE)
    set(BUILD_AUTOTESTS OFF CACHE INTERNAL "Disable building liquid tests" FORCE)
    set(BUILD_BENCHMARKS OFF CACHE INTERNAL "Disable building liquid benchmarks" FORCE)
    set(BUILD_SANDBOX OFF CACHE INTERNAL "Disable building liquid sandbox" FORCE)
    set(BUILD_DOC OFF CACHE INTERNAL "Disable building liquid docs" FORCE)
    set(ENABLE_SIMD ON CACHE INTERNAL "liquid-dsp: SIMD runtime dispatch" FORCE)
    set(FIND_SIMD ON CACHE INTERNAL "liquid-dsp: probe builder SIMD" FORCE)
    set(FIND_FFTW OFF CACHE INTERNAL "liquid-dsp: internal FFT, no fftw dependency" FORCE)
    set(LOGGING_LEVEL "error" CACHE STRING "liquid-dsp: compile out hot-path logging" FORCE)
    # Portable x86: FIND_SIMD=OFF suppresses liquid's CHECK_SIMD re-probing
    # (a cached FALSE would just be re-detected on AVX512 builders), and the
    # pre-AVX512 baseline is declared present. AVX512/AMX stay undefined, so
    # BUILD_AVX512/BUILD_AMX = 0 while AVX2 and below are kept. Non-x86
    # targets (ARM NEON etc.) keep the normal probe path.
    if(SUPERTOOTH_PORTABLE
            AND (CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86|X86|i[3-6]86|AMD64|amd64)"
                OR CMAKE_OSX_ARCHITECTURES MATCHES "x86_64"))
        set(FIND_SIMD OFF CACHE INTERNAL "liquid-dsp: skip SIMD probe (portable preset)" FORCE)
        foreach(_supertooth_isa SSE SSE2 SSE3 SSSE3 SSE41 SSE42 AVX AVX2)
            foreach(_supertooth_lang C CXX)
                set(${_supertooth_lang}_${_supertooth_isa}_FOUND TRUE CACHE BOOL "liquid-dsp: portable pre-AVX512 baseline" FORCE)
            endforeach()
        endforeach()
        unset(_supertooth_isa)
        unset(_supertooth_lang)
    endif()
    # liquid forces CMAKE_CXX_STANDARD=14 in cache (Qt6 needs 17+ via
    # qt_standard_project_setup); preserve ours across its configuration.
    if(DEFINED CMAKE_CXX_STANDARD)
        set(_supertooth_saved_cxx_standard ${CMAKE_CXX_STANDARD})
    endif()
    # liquid-dsp's historical builds (autotools and packaging/build-liquid)
    # all used -ffast-math; without it the channelizer hot path measures
    # ~40% slower (Mac Studio hybrid replay). Scoped to liquid via
    # save/restore so supertooth's own FP semantics are untouched.
    if(DEFINED CMAKE_C_FLAGS)
        set(_supertooth_saved_c_flags "${CMAKE_C_FLAGS}")
    endif()
    if(DEFINED CMAKE_CXX_FLAGS)
        set(_supertooth_saved_cxx_flags "${CMAKE_CXX_FLAGS}")
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math")
    FetchContent_MakeAvailable(liquid)
    if(DEFINED _supertooth_saved_c_flags)
        set(CMAKE_C_FLAGS "${_supertooth_saved_c_flags}")
    else()
        unset(CMAKE_C_FLAGS)
    endif()
    unset(_supertooth_saved_c_flags)
    if(DEFINED _supertooth_saved_cxx_flags)
        set(CMAKE_CXX_FLAGS "${_supertooth_saved_cxx_flags}")
    else()
        unset(CMAKE_CXX_FLAGS)
    endif()
    unset(_supertooth_saved_cxx_flags)
    if(DEFINED _supertooth_saved_cxx_standard)
        set(CMAKE_CXX_STANDARD ${_supertooth_saved_cxx_standard} CACHE STRING "C++ standard" FORCE)
    else()
        unset(CMAKE_CXX_STANDARD CACHE)
    endif()
    unset(_supertooth_saved_cxx_standard)
    # Restore shared default for our own targets (liquid needs static-only).
    set(BUILD_SHARED_LIBS ON CACHE INTERNAL "Restore default" FORCE)
    set(BUILD_STATIC_LIBS OFF CACHE INTERNAL "Restore default" FORCE)
endif()

# Main source directory
add_subdirectory(src)

# Unit / integration tests, opt-in via -DBUILD_TESTS=ON. Run with
# `ctest` once built, or `make tests` (or `ninja tests`).
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()