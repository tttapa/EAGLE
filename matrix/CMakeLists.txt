# Declare dependencies
find_package(LAPACK REQUIRED)

# Create target and set properties
add_library(matrix INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Matrix::matrix ALIAS matrix)

#Set target properties
target_include_directories(matrix
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# target_compile_features(matrix PRIVATE cxx_std_17)
target_compile_options(matrix INTERFACE ${LAPACK_LINKER_FLAGS})

target_link_libraries(matrix
    INTERFACE
        lapacke
        ${LAPACK_LIBRARIES}
)

add_subdirectory(test)