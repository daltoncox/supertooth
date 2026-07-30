find_package(PkgConfig)

if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local")
    set(CMAKE_FIND_FRAMEWORK NEVER)
endif()

if(PkgConfig_FOUND)
    pkg_check_modules(HACKRF libhackrf)
    pkg_check_modules(LIQUID liquid-dsp)
    pkg_check_modules(BTBB libbtbb)
endif()

if(HACKRF_FOUND)
    find_library(HACKRF_LIBRARY_PATH NAMES hackrf libhackrf
        PATHS ${HACKRF_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(HACKRF_LIBRARY_PATH)
        set(HACKRF_LIBRARIES ${HACKRF_LIBRARY_PATH})
    endif()
endif()

if(BTBB_FOUND)
    find_library(BTBB_LIBRARY_PATH NAMES btbb libbtbb
        PATHS ${BTBB_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(BTBB_LIBRARY_PATH)
        set(BTBB_LIBRARIES ${BTBB_LIBRARY_PATH})
    endif()
endif()

if(NOT LIQUID_FOUND)
    find_library(LIQUID_LIBRARIES NAMES liquid
        PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(LIQUID_INCLUDE_DIRS liquid/liquid.h
        PATHS /opt/homebrew/include /usr/local/include)

    if(LIQUID_LIBRARIES AND LIQUID_INCLUDE_DIRS)
        set(LIQUID_FOUND TRUE)
        message(STATUS "Found Liquid DSP manually: ${LIQUID_LIBRARIES}")
    endif()
endif()

if(NOT HACKRF_FOUND)
    find_library(HACKRF_LIBRARIES NAMES hackrf libhackrf
        PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(HACKRF_INCLUDE_DIRS libhackrf/hackrf.h
        PATHS /opt/homebrew/include /usr/local/include)

    if(HACKRF_LIBRARIES AND HACKRF_INCLUDE_DIRS)
        set(HACKRF_FOUND TRUE)
        message(STATUS "Found HackRF manually: ${HACKRF_LIBRARIES}")
    endif()
endif()

if(NOT BTBB_FOUND)
    find_library(BTBB_LIBRARIES NAMES btbb libbtbb
        PATHS
            /opt/homebrew/lib
            /usr/local/lib
            /usr/lib
            /opt/local/lib)
    find_path(BTBB_INCLUDE_DIRS btbb.h
        PATHS
            /opt/homebrew/include
            /usr/local/include
            /usr/include
            /opt/local/include)

    if(BTBB_LIBRARIES AND BTBB_INCLUDE_DIRS)
        set(BTBB_FOUND TRUE)
        message(STATUS "Found BTBB manually: ${BTBB_LIBRARIES}")
    else()
        find_path(BTBB_INCLUDE_DIRS btbb.h
            PATHS
                /opt/homebrew/include/libbtbb
                /usr/local/include/libbtbb
                /usr/include/libbtbb
                /opt/local/include/libbtbb)

        if(BTBB_INCLUDE_DIRS)
            set(BTBB_FOUND TRUE)
            message(STATUS "Found BTBB headers in subdirectory: ${BTBB_INCLUDE_DIRS}")
        endif()
    endif()
endif()

if(NOT HACKRF_FOUND)
    message(FATAL_ERROR "HackRF library not found. Please install with: brew install hackrf")
endif()

if(NOT LIQUID_FOUND)
    message(FATAL_ERROR "Liquid DSP library not found. Please install with: brew install liquid-dsp")
endif()

if(NOT BTBB_FOUND)
    message(FATAL_ERROR "BTBB library is required for this project")
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
    models/receive_event_models.h
    models/phy.h
    protocol/ble/ble_display.c protocol/ble/ble_display.h
    protocol/ble/ble_codec.c protocol/ble/ble_codec.h
    protocol/ble/ble_bitstream_decoder.c protocol/ble/ble_bitstream_decoder.h
    protocol/ble/ble_piconet.c protocol/ble/ble_piconet.h
    protocol/ble/bt_assigned_numbers.c protocol/ble/bt_assigned_numbers.h
    protocol/bredr/bredr_display.c protocol/bredr/bredr_display.h
    protocol/bredr/bredr_codec.c protocol/bredr/bredr_codec.h
    protocol/bredr/bredr_bitstream_decoder.c protocol/bredr/bredr_bitstream_decoder.h
    protocol/bredr/bredr_piconet.c protocol/bredr/bredr_piconet.h
    protocol/bredr/bredr_piconet_store.c protocol/bredr/bredr_piconet_store.h
    protocol/bredr/bredr_recovery.c protocol/bredr/bredr_recovery.h
    protocol/bredr/bredr_recovery_backend.h
    protocol/bredr/bredr_recovery_btbb.c protocol/bredr/bredr_recovery_btbb.h
    radio/radio_common.c radio/radio_common.h
    radio/sample_dispatcher.c radio/sample_dispatcher.h
    radio/hackrf.c radio/hackrf.h
    service/ble_channel_processor.c service/ble_channel_processor.h
    service/ble_channel_processor_new.c service/ble_channel_processor_new.h
    service/bredr_channel_processor_new.c service/bredr_channel_processor_new.h
    service/session.c service/session.h
    service/bredr_channel_processor.c service/bredr_channel_processor.h
    service/receiver_session.c service/receiver_session.h
    service/receiver_session_ble.c
    service/receiver_session_bredr.c
    service/receiver_session_hybrid.c
    ${CMAKE_CURRENT_BINARY_DIR}/version.c
)

target_link_libraries(supertooth_core
    ${HACKRF_LIBRARIES}
    ${LIQUID_LIBRARIES}
    ${BTBB_LIBRARIES}
    Threads::Threads
)

target_include_directories(supertooth_core PUBLIC
    ${HACKRF_INCLUDE_DIRS}
    ${LIQUID_INCLUDE_DIRS}
    ${BTBB_INCLUDE_DIRS}
    ${CORE_INCLUDE_DIRS}
)

if(HACKRF_CFLAGS_OTHER)
    target_compile_options(supertooth_core PRIVATE ${HACKRF_CFLAGS_OTHER})
endif()

if(LIQUID_CFLAGS_OTHER)
    target_compile_options(supertooth_core PRIVATE ${LIQUID_CFLAGS_OTHER})
endif()

if(BTBB_CFLAGS_OTHER)
    target_compile_options(supertooth_core PRIVATE ${BTBB_CFLAGS_OTHER})
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(supertooth_core m)
endif()
