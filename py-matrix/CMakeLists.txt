# Create library target and specify the implementation files to compile.
add_library(py-matrix
    src/PyMatrix.cpp
)

# Add an alias so that library can be used inside the build tree, e.g. when 
# testing
add_library(PyMatrix::py-matrix ALIAS py-matrix)

find_package(pybind11 REQUIRED)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   Any header files in the `src` directory are private, they are only required 
#   to compile the implementation files of the library, users of the library
#   should not include them.
target_include_directories(py-matrix
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Tell CMake to compile this library with C++17 features enabled
target_compile_features(py-matrix PUBLIC cxx_std_17)

# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   Libraries that are used in the public header files should be public,
#   libraries that are only used in the implementation files should be private.
target_link_libraries(py-matrix
    PUBLIC 
        matrix
        pybind11::embed
)

# Run CMake again in the `test` folder, to discover the CMakeLists.txt file that
# adds the tests.
add_subdirectory(test) 