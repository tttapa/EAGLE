add_executable(fileloader_test test-FileLoader.cpp)
target_link_libraries(fileloader_test gtest_main FileLoader::fileloader)

include(GoogleTest)
gtest_discover_tests(fileloader_test)