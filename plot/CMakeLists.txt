##############################################
# Create target and set properties

add_library(plot
    src/Plot.cpp
    src/PlotHelpers.cpp
    src/PlotStepResponse.cpp
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Plot::plot ALIAS plot)

#Set target properties
target_include_directories(plot
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Bas: this works "cmake -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 ."
#set(PYBIND11_PYTHON_VERSION 3.6)   # doesn't make a difference
find_package(pybind11 REQUIRED)

# find_package(Threads REQUIRED) # TODO

# target_compile_features(plot PRIVATE cxx_std_17)
# target_compile_options(plot PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)
target_link_libraries(plot
    PUBLIC 
        Drone::drone
        StepResponse::step-response
        DronePlottable::drone-plottable
        DroneLogLoader::drone-log-loader
        pybind11::embed
        # Threads::Threads # TODO
        # dl # TODO
        # util # TODO
    PRIVATE
        Utilities::utilities
        stdc++fs
)

# add_subdirectory(test) # TODO