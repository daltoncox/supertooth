find_package(PkgConfig)

if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local")
    set(CMAKE_FIND_FRAMEWORK NEVER)
endif()

if(PkgConfig_FOUND)
    if(ENABLE_HACKRF)
        pkg_check_modules(HACKRF libhackrf)
    endif()
    if(ENABLE_BLADERF)
        # Upstream .pc file uses capital RF (libbladeRF.pc); some distro
        # packages ship a lowercase alias. Take the first match so old and
        # new prefixes both work.
        pkg_search_module(BLADERF libbladeRF libbladerf)
    endif()
    pkg_check_modules(LIQUID liquid-dsp)
endif()

if(ENABLE_HACKRF AND HACKRF_FOUND)
    find_library(HACKRF_LIBRARY_PATH NAMES hackrf libhackrf
        PATHS ${HACKRF_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(HACKRF_LIBRARY_PATH)
        set(HACKRF_LIBRARIES ${HACKRF_LIBRARY_PATH})
    endif()
endif()

if(ENABLE_BLADERF AND BLADERF_FOUND)
    find_library(BLADERF_LIBRARY_PATH NAMES bladeRF libbladeRF bladerf libbladerf
        PATHS ${BLADERF_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(BLADERF_LIBRARY_PATH)
        set(BLADERF_LIBRARIES ${BLADERF_LIBRARY_PATH})
    endif()
endif()

if(NOT LIQUID_FOUND)
    find_library(LIQUID_LIBRARIES NAMES liquid
        PATHS /opt/liquid-dsp/lib /opt/liquid-dsp/lib64 /opt/homebrew/lib /usr/local/lib)
    find_path(LIQUID_INCLUDE_DIRS liquid/liquid.h
        PATHS /opt/liquid-dsp/include /opt/homebrew/include /usr/local/include)

    if(LIQUID_LIBRARIES AND LIQUID_INCLUDE_DIRS)
        set(LIQUID_FOUND TRUE)
        message(STATUS "Found Liquid DSP manually: ${LIQUID_LIBRARIES}")
    endif()
endif()

if(ENABLE_HACKRF AND NOT HACKRF_FOUND)
    find_library(HACKRF_LIBRARIES NAMES hackrf libhackrf
        PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(HACKRF_INCLUDE_DIRS libhackrf/hackrf.h
        PATHS /opt/homebrew/include /usr/local/include)

    if(HACKRF_LIBRARIES AND HACKRF_INCLUDE_DIRS)
        set(HACKRF_FOUND TRUE)
        message(STATUS "Found HackRF manually: ${HACKRF_LIBRARIES}")
    endif()
endif()

if(ENABLE_BLADERF AND NOT BLADERF_FOUND)
    # NOTE: upstream soname uses capital RF (libbladeRF).
    find_library(BLADERF_LIBRARIES NAMES bladeRF libbladeRF bladerf libbladerf
        PATHS /opt/bladerf/lib /opt/bladerf/lib64 /opt/homebrew/lib /usr/local/lib)
    find_path(BLADERF_INCLUDE_DIRS libbladeRF.h
        PATHS /opt/bladerf/include /opt/homebrew/include /usr/local/include)

    if(BLADERF_LIBRARIES AND BLADERF_INCLUDE_DIRS)
        set(BLADERF_FOUND TRUE)
        message(STATUS "Found bladeRF manually: ${BLADERF_LIBRARIES}")
    endif()
endif()

if(ENABLE_HACKRF AND NOT HACKRF_FOUND)
    message(FATAL_ERROR "HackRF library not found. Please install with: brew install hackrf (or configure with -DENABLE_HACKRF=OFF)")
endif()

if(ENABLE_BLADERF AND NOT BLADERF_FOUND)
    message(FATAL_ERROR "bladeRF library not found. Please install with: brew install libbladerf (or configure with -DENABLE_BLADERF=OFF)")
endif()

# 8-bit SC8_Q7 + oversample support (for sample rates above 61.44 Msps)
# needs libbladeRF >= 2.5.0. Fail fast at configure time when pkg-config
# reports an older version (bladerf.c carries a compile-time backstop for
# hand-rolled include paths that bypass pkg-config).
if(ENABLE_BLADERF AND BLADERF_FOUND AND BLADERF_VERSION AND BLADERF_VERSION VERSION_LESS "2.5.0")
    message(FATAL_ERROR "libbladeRF >= 2.5.0 required (found ${BLADERF_VERSION}). Newer Ubuntu releases ship it; otherwise build from source: bash packaging/build-bladerf-linux.sh (or configure with -DENABLE_BLADERF=OFF)")
endif()

if(NOT ENABLE_HACKRF AND NOT ENABLE_BLADERF)
    message(STATUS "No live radio backends enabled; only file replay will be available")
endif()

if(NOT LIQUID_FOUND)
    message(FATAL_ERROR "Liquid DSP library not found. Build from source: bash packaging/build-liquid-linux.sh (Linux) or bash packaging/build-liquid-macos.sh (macOS)")
endif()

find_package(Threads REQUIRED)

set(CORE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/dsp
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/radio
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/ble
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/bredr
    ${CMAKE_CURRENT_SOURCE_DIR}/service
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------
# Version string
# ---------------------------------------------------------------------------
# Set via -DSUPERTOOTH_VERSION=vX.X.X for CI tagged releases; otherwise
# derived from git describe --tags --always --dirty or falls back to
# "unknown".
set(SUPERTOOTH_VERSION "unknown" CACHE STRING
    "Supertooth version string (set via -D for CI tagged releases)")

if(SUPERTOOTH_VERSION STREQUAL "unknown")
    find_package(Git QUIET)
    if(Git_FOUND)
        execute_process(COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE SUPERTOOTH_VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET)
        if(SUPERTOOTH_VERSION STREQUAL "")
            set(SUPERTOOTH_VERSION "unknown")
        endif()
    endif()
endif()

configure_file(version.c.in version.c @ONLY)

add_library(supertooth_core STATIC
    dsp/rssi_measurements.c
    dsp/rssi_measurements.h
    dsp/channelizer_bank.c
    dsp/channelizer_bank.h
    dsp/ddc_stage.c
    dsp/ddc_stage.h
    models/receive_event_models.h
    models/phy.h
    models/rssi_tracker.c
    models/rssi_tracker.h
    protocol/ble/ble_display.c protocol/ble/ble_display.h
    protocol/ble/ble_codec.c protocol/ble/ble_codec.h
    protocol/ble/ble_bitstream_decoder.c protocol/ble/ble_bitstream_decoder.h
    protocol/ble/ble_registry.c protocol/ble/ble_registry.h
    protocol/ble/bt_assigned_numbers.c protocol/ble/bt_assigned_numbers.h
    protocol/bredr/bredr_display.c protocol/bredr/bredr_display.h
    protocol/bredr/bredr_codec.c protocol/bredr/bredr_codec.h
    protocol/bredr/bredr_bitstream_decoder.c protocol/bredr/bredr_bitstream_decoder.h
    protocol/bredr/bredr_link.c protocol/bredr/bredr_link.h
    protocol/bredr/bredr_registry.c protocol/bredr/bredr_registry.h
    protocol/bredr/bredr_clock_recovery.c protocol/bredr/bredr_clock_recovery.h
    radio/radio_common.c radio/radio_common.h
    radio/sample_dispatcher.c radio/sample_dispatcher.h
    radio/wav.c radio/wav.h
    radio/file.c radio/file.h
    service/ble_channel_processor.c service/ble_channel_processor.h
    service/bredr_channel_processor.c service/bredr_channel_processor.h
    service/channelizer_service.c service/channelizer_service.h
    service/collector.c service/collector.h
    service/session.c service/session.h
    ${CMAKE_CURRENT_BINARY_DIR}/version.c
)

target_link_libraries(supertooth_core
    Threads::Threads
)

if(ENABLE_HACKRF)
    target_sources(supertooth_core PRIVATE
        radio/hackrf.c radio/hackrf.h
    )
    target_compile_definitions(supertooth_core PUBLIC HAVE_HACKRF=1)
    target_link_libraries(supertooth_core ${HACKRF_LIBRARIES})
    target_include_directories(supertooth_core PUBLIC ${HACKRF_INCLUDE_DIRS})
    if(HACKRF_CFLAGS_OTHER)
        target_compile_options(supertooth_core PRIVATE ${HACKRF_CFLAGS_OTHER})
    endif()
else()
    target_compile_definitions(supertooth_core PUBLIC HAVE_HACKRF=0)
endif()

if(ENABLE_BLADERF)
    target_sources(supertooth_core PRIVATE
        radio/bladerf.c radio/bladerf.h
    )
    target_compile_definitions(supertooth_core PUBLIC HAVE_BLADERF=1)
    target_link_libraries(supertooth_core ${BLADERF_LIBRARIES})
    target_include_directories(supertooth_core PUBLIC ${BLADERF_INCLUDE_DIRS})
    if(BLADERF_CFLAGS_OTHER)
        target_compile_options(supertooth_core PRIVATE ${BLADERF_CFLAGS_OTHER})
    endif()
else()
    target_compile_definitions(supertooth_core PUBLIC HAVE_BLADERF=0)
endif()

target_link_libraries(supertooth_core
    ${LIQUID_LIBRARIES}
)

target_include_directories(supertooth_core PUBLIC
    ${LIQUID_INCLUDE_DIRS}
    ${CORE_INCLUDE_DIRS}
)

if(LIQUID_CFLAGS_OTHER)
    target_compile_options(supertooth_core PRIVATE ${LIQUID_CFLAGS_OTHER})
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(supertooth_core m)
endif()
