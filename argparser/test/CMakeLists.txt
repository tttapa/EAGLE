add_executable(argparser_test test-ArgParser.cpp)
target_link_libraries(argparser_test gtest_main ArgParser::argparser)

include(GoogleTest)
gtest_discover_tests(argparser_test)