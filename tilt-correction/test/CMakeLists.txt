# Add an executable with tests, and specify the source files to compile
add_executable(tilt_correction_test test-TiltCorrection.cpp)
# Link the test executable with the Google Test main entry point and the library
# under test
target_link_libraries(tilt_correction_test 
    gtest_main
    TiltCorrection::tilt-correction
    util
    CControllers::CControllers # Siel's tilt correction
)

# Add the tests to Google Test
include(GoogleTest)
gtest_discover_tests(tilt_correction_test)