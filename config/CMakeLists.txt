##############################################
# Create target and set properties

add_library(config
    src/Config.cpp
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Config::config ALIAS config)

#Set target properties
target_include_directories(config
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(config PUBLIC cxx_std_17)
# target_compile_options(config PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)
target_link_libraries(config
    PUBLIC 
        stdc++fs
    PRIVATE 
        Quaternion::quaternion
        ODE::ode
)

# add_subdirectory(test)