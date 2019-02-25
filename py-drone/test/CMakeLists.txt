# Add an executable with tests, and specify the source files to compile
add_executable(py-drone-test test-PyDrone.cpp)

# Link the test executable with the Google Test main entry point and the library
# under test
target_link_libraries(py-drone-test 
    gtest
    PyDrone::py-drone
    pybind11::embed
)

# Add the tests to Google Test
include(GoogleTest)
gtest_discover_tests(py-drone-test)