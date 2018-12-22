# Create target and set properties
add_library(util INTERFACE)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Util::util ALIAS util)

#Set target properties
target_include_directories(util
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # PRIVATE
    #     ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(util PRIVATE cxx_std_17)

# target_link_libraries(util
#     PRIVATE
#         Matrix::matrix
# )

add_subdirectory(test)