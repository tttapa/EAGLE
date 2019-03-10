add_executable(drone_test test-C-Controllers.cpp)
target_link_libraries(drone_test gtest_main drone fileloader CControllers)

include(GoogleTest)
gtest_discover_tests(drone_test)