# Create target and set properties
add_library(drone-plottable INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(DronePlottable::drone-plottable ALIAS drone-plottable)

#Set target properties
target_include_directories(drone-plottable
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone-plottable
    INTERFACE
        drone 
        drone-log-loader
)