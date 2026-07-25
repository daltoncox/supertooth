# Radio backend tests.
#
# These exercise the radio abstraction layer (device enumeration, open by
# identifier, etc.). Tests that require physical hardware are written to
# skip gracefully (return 0) when no devices are present, so they remain
# safe to run in CI.

add_executable(test_radio_list_devices
    test_radio_list_devices.c
)

target_link_libraries(test_radio_list_devices
    supertooth_core
)

add_test(NAME test_radio_list_devices COMMAND test_radio_list_devices)