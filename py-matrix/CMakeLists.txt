# Create header-only library target.
add_library(py-matrix INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when 
# testing
add_library(PyMatrix::py-matrix ALIAS py-matrix)

find_package(pybind11 REQUIRED)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   This particular library is header-only, so there are only INTERFACE
#   include directories. (Having PRIVATE properties would make no sense, because
#   the header-only library has nothing to compile.)
target_include_directories(py-matrix
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Tell CMake to compile this library with C++17 features enabled
target_compile_features(py-matrix INTERFACE cxx_std_17)

# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   This particular library is header-only, so there are only INTERFACE 
#   link options.   
target_link_libraries(py-matrix
    INTERFACE 
        matrix
        pybind11::embed
)

# Run CMake again in the `test` folder, to discover the CMakeLists.txt file that
# adds the tests.
add_subdirectory(test)