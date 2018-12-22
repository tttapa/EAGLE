add_executable(ode_test test-DoPri.cpp test-ODEEval.cpp)
target_link_libraries(ode_test gtest_main ODE::ode)

include(GoogleTest)
gtest_discover_tests(ode_test)