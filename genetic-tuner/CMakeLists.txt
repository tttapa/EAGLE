##############################################
# Create target and set properties


#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(GeneticTuner::genetic-tuner ALIAS genetic-tuner)

#Set target properties
target_include_directories(genetic-tuner
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(genetic-tuner PUBLIC cxx_std_17)
# target_compile_options(fileloader PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)

target_link_libraries(genetic-tuner 
    PUBLIC 
        Matrix::matrix 
        stdc++fs
)

add_subdirectory(test) 