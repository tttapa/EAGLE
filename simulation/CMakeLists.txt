# Create target and set properties
add_library(simulation INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Simulation::simulation ALIAS simulation)

#Set target properties
target_include_directories(simulation
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # PRIVATE
    #     ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(simulation PRIVATE cxx_std_17)

target_link_libraries(simulation
    INTERFACE
        ODE::ode
        Utilities::utilities
        Matrix::matrix
)

# add_subdirectory(test) TODO