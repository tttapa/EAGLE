# Create target and set properties
add_library(utilities INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Utilities::utilities ALIAS utilities)

#Set target properties
target_include_directories(utilities
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # PRIVATE
    #     ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(utilities PRIVATE cxx_std_17)

# target_link_libraries(utilities
#     PRIVATE
#         Matrix::matrix
# )

add_subdirectory(test)