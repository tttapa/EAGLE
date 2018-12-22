add_executable(simulation_test test-System.cpp)
target_link_libraries(simulation_test gtest_main Simulation::simulation)

include(GoogleTest)
gtest_discover_tests(simulation_test)