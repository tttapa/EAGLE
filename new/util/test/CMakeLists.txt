add_executable(util_test test-MeanSquareError.cpp)
target_link_libraries(util_test gtest_main Util::util)

include(GoogleTest)
gtest_discover_tests(util_test)