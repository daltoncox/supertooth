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

add_executable(test_wav_io
    test_wav_io.c
)

target_link_libraries(test_wav_io
    supertooth_core
)

add_test(NAME test_wav_io COMMAND test_wav_io)

add_executable(test_file_device
    test_file_device.c
)

target_link_libraries(test_file_device
    supertooth_core
)

add_test(NAME test_file_device COMMAND test_file_device)

add_executable(test_dispatcher_backpressure
    test_dispatcher_backpressure.c
)

target_link_libraries(test_dispatcher_backpressure
    supertooth_core
)

add_test(NAME test_dispatcher_backpressure COMMAND test_dispatcher_backpressure)