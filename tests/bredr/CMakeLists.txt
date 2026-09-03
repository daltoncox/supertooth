# BR/EDR tests.
#
# Each test is a small executable that returns 0 on success and non-zero
# on failure. They are registered with ctest via add_test() so the whole
# suite can be run with `ctest` (or `ctest --output-on-failure`).

# ---------------------------------------------------------------------------
# Test-only static build of the vendored libbtbb reference implementation.
#
# libbtbb is GPL; it is linked EXCLUSIVELY into test executables as a
# differential oracle and is NEVER linked into supertooth_core or any
# shipped product binary.
#
# The libbtbb checkout is optional and gitignored. When it is absent, the
# oracle parity test is skipped so the rest of the suite still configures.
# ---------------------------------------------------------------------------
set(BTBB_SRC_DIR ${CMAKE_SOURCE_DIR}/libbtbb/lib/src)

if(EXISTS "${BTBB_SRC_DIR}/bluetooth_packet.c"
   AND EXISTS "${BTBB_SRC_DIR}/bluetooth_piconet.c")
    set(BTBB_ORACLE_AVAILABLE TRUE)
else()
    set(BTBB_ORACLE_AVAILABLE FALSE)
    message(STATUS "libbtbb not found at ${BTBB_SRC_DIR}; skipping btbb oracle tests")
endif()

if(BTBB_ORACLE_AVAILABLE)
    add_library(btbb_oracle STATIC
        ${BTBB_SRC_DIR}/bluetooth_packet.c
        ${BTBB_SRC_DIR}/bluetooth_piconet.c
    )

    target_include_directories(btbb_oracle PUBLIC
        ${BTBB_SRC_DIR}
    )

    set_target_properties(btbb_oracle PROPERTIES
        C_STANDARD 99
    )
endif()

add_executable(test_fec_2_3
    test_fec_2_3.c
)

target_link_libraries(test_fec_2_3
    supertooth_core
)

add_test(NAME test_fec_2_3 COMMAND test_fec_2_3)

add_executable(test_crc
    test_crc.c
)

target_link_libraries(test_crc
    supertooth_core
)

add_test(NAME test_crc COMMAND test_crc)

add_executable(test_recovery
    test_recovery.c
)

target_link_libraries(test_recovery
    supertooth_core
)

add_test(NAME test_recovery COMMAND test_recovery)

add_executable(test_capture_151FC475
    test_capture_151FC475.c
)

target_link_libraries(test_capture_151FC475
    supertooth_core
)

add_test(NAME test_capture_151FC475 COMMAND test_capture_151FC475)

add_executable(test_codec_oracle
    test_codec_oracle.c
    libbtbb_oracle.c
)

target_link_libraries(test_codec_oracle
    supertooth_core
)

add_test(NAME test_codec_oracle COMMAND test_codec_oracle)

if(BTBB_ORACLE_AVAILABLE)
    add_executable(test_recovery_btbb_parity
        test_recovery_btbb_parity.c
        btbb_test_adapter.c
    )

    target_link_libraries(test_recovery_btbb_parity
        supertooth_core
        btbb_oracle
    )

    add_test(NAME test_recovery_btbb_parity COMMAND test_recovery_btbb_parity)
endif()

add_executable(test_replay_frame_dump test_replay_frame_dump.c)
target_link_libraries(test_replay_frame_dump PRIVATE supertooth_core)
add_test(NAME test_replay_frame_dump COMMAND test_replay_frame_dump)

add_executable(test_drift_confirm
    test_drift_confirm.c
)

target_link_libraries(test_drift_confirm
    supertooth_core
)

add_test(NAME test_drift_confirm COMMAND test_drift_confirm)
