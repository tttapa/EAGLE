cmake_minimum_required(VERSION 3.10)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_C_STANDARD 90) # ANSI

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# <copy>
# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest/googletest-build
                 EXCLUDE_FROM_ALL)
# </copy>

# SET (CMAKE_C_COMPILER "clang")
# SET (CMAKE_CXX_COMPILER "clang++")
SET (CMAKE_C_COMPILER "gcc-8")
SET (CMAKE_CXX_COMPILER "g++-8")

# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) # LTO

include(GoogleTest)
enable_testing()

# Compiler flags for C++
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} \
-O2 \
-Wall -Wextra \
-Wsuggest-override \
-Wno-register \
-Wno-missing-braces -Wno-unknown-pragmas \
-fdiagnostics-show-option \
-fextended-identifiers -finput-charset=UTF-8")

# Compiler flags for C
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} \
-O2 \
-Wall -Wextra")

#include Google Test
include_directories(${CMAKE_BINARY_DIR}/googletest/googletest-src/googletest/include)

#include Google Mock
include_directories(${CMAKE_BINARY_DIR}/googletest/googletest-src/googlemock/include)

# include Matplotlib-cpp & Python 
find_package(PythonLibs 2.7 REQUIRED)
include_directories($ENV{HOME}/lib/matplotlib-cpp)

# Source files
include_directories("../src")
file(GLOB_RECURSE SOURCES "../src/*.cpp")
add_library(source STATIC ${SOURCES})

# Main source files
include_directories("../main")
file(GLOB_RECURSE MAIN "../main/*.cpp" "../main/*.c")
add_executable(main ${MAIN})
target_include_directories(main PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(main 
                      source
                      ${PYTHON_LIBRARIES}
                      lapacke
                      stdc++fs)

add_definitions(-DPLOT_CONTROLLER)
add_definitions(-DPLOT_ACTUAL_VS_ESTIMATED_STATES)
add_definitions(-DPLOT_OBSERVER)

# Tuner
file(GLOB_RECURSE TUNER "../tuner/*.cpp" "../tuner/*.c")
add_executable(tuner ${TUNER})
find_package(OpenMP REQUIRED)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
target_include_directories(tuner PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(tuner source ${PYTHON_LIBRARIES} lapacke)

# Test executable compilation and linking
file(GLOB_RECURSE TESTS "../test/*.cpp")
add_executable(tests ${TESTS})
target_link_libraries(tests
                      gmock
                      source
                      lapacke
                      stdc++fs)

# Add tests
gtest_discover_tests(tests)

# Add Doxygen target
add_custom_target(documentation rm -r html && doxygen
                  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/../doc/")