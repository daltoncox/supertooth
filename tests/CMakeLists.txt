# Tests for supertooth.
#
# Protocol-family-specific test suites live in their own subdirectories
# (e.g. `bredr/`, and `ble/` for future BLE tests). Each subdirectory
# has its own CMakeLists.txt that defines its test executables and
# registers them with ctest.

add_subdirectory(bredr)
add_subdirectory(radio)

set(_supertooth_test_targets
    test_fec_2_3
    test_crc
    test_radio_list_devices
)

add_custom_target(tests)
add_dependencies(tests ${_supertooth_test_targets})
