##############################################
# Create target and set properties

add_library(drone-log-loader
    src/DroneLogLoader.cpp
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(DroneLogLoader::drone-log-loader ALIAS drone-log-loader)

#Set target properties
target_include_directories(drone-log-loader
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(drone-log-loader PRIVATE cxx_std_17)
# target_compile_options(drone-log-loader PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)
target_link_libraries(drone-log-loader 
    PUBLIC 
        Drone::drone 
        stdc++fs
)

# add_subdirectory(test) # TODO