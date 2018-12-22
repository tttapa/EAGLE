file(GLOB TEST_SRCS *.cpp)
add_executable(matrix_test ${TEST_SRCS})
target_link_libraries(matrix_test gtest_main Matrix::matrix)

include(GoogleTest)
gtest_discover_tests(matrix_test)