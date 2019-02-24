# Add an executable with tests, and specify the source files to compile
add_executable(py-matrix-test test-PyMatrix.cpp)

# Link the test executable with the Google Test main entry point and the library
# under test
target_link_libraries(py-matrix-test 
    gtest_main
    PyMatrix::py-matrix
)

# Add the tests to Google Test
include(GoogleTest)
gtest_discover_tests(py-matrix-test)