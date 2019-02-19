# Add an executable with tests, and specify the source files to compile
add_executable(chromosome_test test-Chromosome.cpp)
# Link the test executable with the Google Test main entry point and the library
# under test
target_link_libraries(chromosome_test gtest_main GeneticTuner::genetic-tuner)

# Add the tests to Google Test
include(GoogleTest)
gtest_discover_tests(chromosome_test)