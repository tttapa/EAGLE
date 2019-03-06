# Create header-only library target.
add_library(py-drone INTERFACE)

# Tell CMake to compile this library with C++17 features enabled
target_compile_features(py-drone INTERFACE cxx_std_17)

# Add an alias so that library can be used inside the build tree, e.g. when 
# testing
add_library(PyDrone::py-drone ALIAS py-drone)

find_package(pybind11 REQUIRED)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
#   This particular library is header-only, so there are only INTERFACE
#   include directories. (Having PRIVATE properties would make no sense, because
#   the header-only library has nothing to compile.)
target_include_directories(py-drone
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
#   This particular library is header-only, so there are only INTERFACE 
#   link options.   
target_link_libraries(py-drone
    INTERFACE 
        py-matrix
        drone
        plot
)

# Run CMake again in the `test` folder, to discover the CMakeLists.txt file that
# adds the tests.
add_subdirectory(test)

################################################################################
################################ PYTHON MODULE #################################
################################################################################

find_package(pybind11 REQUIRED)

# Create a Python module
pybind11_add_module(py_drone_module src/PyDrone.module.cpp)

# Set target include directories 
#   These are the folders where the compiler will search for included header 
#   files.
#   The include files in the `include` folder are part of the public API of the
#   library, users of the library need to be able to include them in their code. 
target_include_directories(py_drone_module
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# Link the library with its dependencies.
#   This manages both the include directories of the dependencies, as well as  
#   the actual linking of the other libraries.
target_link_libraries(py_drone_module
    PUBLIC 
        pybind11::module
        py-matrix
        drone-plottable
)