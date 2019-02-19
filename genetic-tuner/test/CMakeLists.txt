add_executable(chromosome_test test-Chromosome.cpp)
target_link_libraries(chromosome_test gtest_main Chromosome::Chromosome)

include(GoogleTest)
gtest_discover_tests(chromosome_test)