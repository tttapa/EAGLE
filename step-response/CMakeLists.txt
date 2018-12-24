##############################################
# Create target and set properties

add_library(step-response
    INTERFACE
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(StepResponse::step-response ALIAS step-response)

#Set target properties
target_include_directories(step-response
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(step-response PRIVATE cxx_std_17)
# target_compile_options(step-response PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)
target_link_libraries(step-response
    INTERFACE
        Matrix::matrix
        Plot::plot
)

# add_subdirectory(test) # TODO