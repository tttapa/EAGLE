add_executable(quaternion_test test-Quaternion.cpp)
target_link_libraries(quaternion_test gtest_main Quaternion::quaternion)

include(GoogleTest)
gtest_discover_tests(quaternion_test)