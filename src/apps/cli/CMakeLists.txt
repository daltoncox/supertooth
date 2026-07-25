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
)

target_link_libraries(cli_common
    supertooth_core
)

target_include_directories(cli_common PUBLIC
    ${CLI_INCLUDE_DIRS}
)

add_executable(supertooth-bredr
    supertooth-bredr.c
)

target_link_libraries(supertooth-bredr
    cli_common
    supertooth_core
)

add_executable(supertooth-ble
    supertooth-ble.c
)

target_link_libraries(supertooth-ble
    cli_common
    supertooth_core
)

add_executable(supertooth-hybrid
    supertooth-hybrid.c
)

target_link_libraries(supertooth-hybrid
    cli_common
    supertooth_core
)

include(GNUInstallDirs)

install(TARGETS supertooth-bredr supertooth-ble supertooth-hybrid
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
