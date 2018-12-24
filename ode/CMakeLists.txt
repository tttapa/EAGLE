# Create target and set properties
add_library(ode INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(ODE::ode ALIAS ode)

#Set target properties
target_include_directories(ode
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(ode INTERFACE cxx_std_17)

target_link_libraries(ode
    INTERFACE
        Matrix::matrix
        Util::util
)

add_subdirectory(test)