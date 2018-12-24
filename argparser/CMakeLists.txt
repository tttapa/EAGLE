##############################################
# Create target and set properties

add_library(argparser
    src/ArgParser.cpp
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(ArgParser::argparser ALIAS argparser)

#Set target properties
target_include_directories(argparser
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(argparser PRIVATE cxx_std_17)
# target_compile_options(argparser PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)

target_link_libraries(argparser 
    PRIVATE 
        Util::util 
)

add_subdirectory(test)