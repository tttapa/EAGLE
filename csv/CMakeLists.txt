# Create target and set properties
add_library(csv INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(CSV::csv ALIAS csv)

#Set target properties
target_include_directories(csv
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # PRIVATE
    #     ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(csv PRIVATE cxx_std_17)

target_link_libraries(csv
    INTERFACE
        ODE::ode
        Utilities::utilities
        stdc++fs
)

# add_subdirectory(test) TODO