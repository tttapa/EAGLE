##############################################
# Create target and set properties

add_library(fileloader
    src/FileLoader.cpp
)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(FileLoader::fileloader ALIAS fileloader)

#Set target properties
target_include_directories(fileloader
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(fileloader PUBLIC cxx_std_17)
# target_compile_options(fileloader PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Werror -pedantic>)

target_link_libraries(fileloader 
    PUBLIC 
        Matrix::matrix 
        stdc++fs
)

add_subdirectory(test) 