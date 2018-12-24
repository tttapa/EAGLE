##############################################
# Create target and set properties

add_library(quaternion
    src/Quaternion.cpp
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(Quaternion::quaternion ALIAS quaternion)

#Set target properties
target_include_directories(quaternion
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# target_compile_features(quaternion PRIVATE cxx_std_17)
# target_compile_options(quaternion PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)

target_link_libraries(quaternion 
    PUBLIC 
        Matrix::matrix 
)

add_subdirectory(test) 