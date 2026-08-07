# DSP-layer tests.
#
# Each test is a small executable that returns 0 on success and non-zero
# on failure. They are registered with ctest via add_test() so the whole
# suite can be run with `ctest` (or `ctest --output-on-failure`).

add_executable(test_rssi_window
    test_rssi_window.c
)

target_link_libraries(test_rssi_window
    supertooth_core
)

add_test(NAME test_rssi_window COMMAND test_rssi_window)
