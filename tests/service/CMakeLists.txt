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

add_executable(test_collector_blocking
    test_collector_blocking.c
)

target_link_libraries(test_collector_blocking
    supertooth_core
)

add_test(NAME test_collector_blocking COMMAND test_collector_blocking)

add_executable(test_exhaustive_replay
    test_exhaustive_replay.c
)

target_link_libraries(test_exhaustive_replay
    supertooth_core
)

add_test(NAME test_exhaustive_replay COMMAND test_exhaustive_replay)

add_executable(test_channelizer_service_lanes
    test_channelizer_service_lanes.c
)

target_link_libraries(test_channelizer_service_lanes
    supertooth_core
)

add_test(NAME test_channelizer_service_lanes COMMAND test_channelizer_service_lanes)
