# Service-layer tests.
#
# Each test is a small executable that returns 0 on success and non-zero
# on failure. They are registered with ctest via add_test() so the whole
# suite can be run with `ctest` (or `ctest --output-on-failure`).

add_executable(test_channel_layout
    test_channel_layout.c
)

target_link_libraries(test_channel_layout
    supertooth_core
)

add_test(NAME test_channel_layout COMMAND test_channel_layout)
