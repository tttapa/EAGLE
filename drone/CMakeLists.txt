##############################################
# Create target and set properties

add_library(drone
    src/Drone.cpp
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Drone::drone ALIAS drone)

#Set target properties
target_include_directories(drone
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(argparser PRIVATE cxx_std_17)
# target_compile_options(argparser PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)

target_link_libraries(drone
    PUBLIC
        Matrix::matrix 
        Quaternion::quaternion
        Simulation::simulation
    PRIVATE
        FileLoader::fileloader
        CControllers::CControllers
)

# add_subdirectory(test) # TODO