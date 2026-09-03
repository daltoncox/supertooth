# Tests for supertooth.
#
# Protocol-family-specific test suites live in their own subdirectories
# (e.g. `bredr/`, `ble/`). Each subdirectory has its own CMakeLists.txt
# that defines its test executables and registers them with ctest.

add_subdirectory(bredr)
add_subdirectory(ble)
add_subdirectory(dsp)
add_subdirectory(radio)
add_subdirectory(service)

set(_supertooth_test_targets
    test_fec_2_3
    test_crc
    test_recovery
    test_capture_151FC475
    test_codec_oracle
    test_replay_frame_dump
    test_drift_confirm
    test_ble_crc_reverse
    test_ble_data_codec
    test_ble_bitstream_decoder
    test_radio_list_devices
    test_channel_layout
    test_rssi_window
    test_rssi_tracker
)

# Oracle parity test exists only when the optional libbtbb checkout is present
# (see tests/bredr/CMakeLists.txt).
if(TARGET test_recovery_btbb_parity)
    list(APPEND _supertooth_test_targets test_recovery_btbb_parity)
endif()

add_custom_target(tests)
add_dependencies(tests ${_supertooth_test_targets})
