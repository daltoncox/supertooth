# BLE tests.
#
# Each test is a small executable that returns 0 on success and non-zero
# on failure. They are registered with ctest via add_test() so the whole
# suite can be run with `ctest` (or `ctest --output-on-failure`).

add_executable(test_ble_crc_reverse
    test_ble_crc_reverse.c
)

target_link_libraries(test_ble_crc_reverse
    supertooth_core
)

add_test(NAME test_ble_crc_reverse COMMAND test_ble_crc_reverse)

add_executable(test_ble_data_codec
    test_ble_data_codec.c
)

target_link_libraries(test_ble_data_codec
    supertooth_core
)

add_test(NAME test_ble_data_codec COMMAND test_ble_data_codec)

add_executable(test_ble_bitstream_decoder
    test_ble_bitstream_decoder.c
)

target_link_libraries(test_ble_bitstream_decoder
    supertooth_core
)

add_test(NAME test_ble_bitstream_decoder COMMAND test_ble_bitstream_decoder)
