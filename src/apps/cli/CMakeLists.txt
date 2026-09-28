set(CLI_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/core/dsp
    ${CMAKE_SOURCE_DIR}/src/core/models
    ${CMAKE_SOURCE_DIR}/src/core/radio
    ${CMAKE_SOURCE_DIR}/src/core/protocol/ble
    ${CMAKE_SOURCE_DIR}/src/core/protocol/bredr
    ${CMAKE_SOURCE_DIR}/src/core/service
)

add_library(cli_common STATIC
    app_common.c
    app_common.h
    app_device_view.c
    app_device_view.h
    app_record.c
    app_record.h
    app_summary_view.c
    app_summary_view.h
    cli_modes.h
    run_ble.c
    run_bredr.c
    run_hybrid.c
    run_record.c
)

target_link_libraries(cli_common
    supertooth_core
)

target_include_directories(cli_common PUBLIC
    ${CLI_INCLUDE_DIRS}
)

# Multiplexed CLI: `supertooth <hybrid|ble|bredr|record> [options]`.
add_executable(supertooth
    supertooth.c
)

target_link_libraries(supertooth
    cli_common
    supertooth_core
)

include(GNUInstallDirs)

install(TARGETS supertooth
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
