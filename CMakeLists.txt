cmake_minimum_required(VERSION 3.10)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# TODO
# set (CMAKE_CXX_STANDARD 17)
# set (CMAKE_C_STANDARD 90) # ANSI

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

################################################################################
##############################   GOOGLE TEST   #################################
################################################################################

# <copy>
# Download and unpack googletest at configure time
configure_file(${CMAKE_CURRENT_LIST_DIR}/cmake/GoogleTest-CMakeLists.txt.in 
               googletest/CMakeLists.txt)
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

include(GoogleTest)
enable_testing()

################################################################################
#############################   COMPILER OPTIONS   #############################
################################################################################

# SET (CMAKE_C_COMPILER "clang")
# SET (CMAKE_CXX_COMPILER "clang++")
SET (CMAKE_C_COMPILER "gcc-8")
SET (CMAKE_CXX_COMPILER "g++-8")

# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) # LTO

# Compiler flags for C++
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} \
-O3 -g3 \
-Wall -Wextra -Werror \
-Wsuggest-override \
-Wno-register \
-Wno-missing-braces -Wno-unknown-pragmas \
-fdiagnostics-show-option \
-fextended-identifiers -finput-charset=UTF-8")

# Compiler flags for C
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} \
-O3 -g3 \
-Wall -Wextra -Werror -pedantic")

################################################################################
#################################   LIBRARIES   ################################
################################################################################

find_package(CControllers)

add_subdirectory("matrix")
add_subdirectory("util")
add_subdirectory("argparser")
add_subdirectory("quaternion")
add_subdirectory("fileloader")
add_subdirectory("ode")
add_subdirectory("simulation")
add_subdirectory("drone")
add_subdirectory("step-response")
add_subdirectory("drone-log-loader")
add_subdirectory("plot")
add_subdirectory("csv")
add_subdirectory("config")
add_subdirectory("genetic-tuner")

################################################################################
###############################   APPLICATIONS   ###############################
################################################################################

### Plot Simulation

file(GLOB_RECURSE SRCS_plot_simulation "applications/plot-simulation/*.cpp")
add_executable(plot-simulation ${SRCS_plot_simulation})
target_link_libraries(plot-simulation PRIVATE argparser 
                                              plot
                                              config)

### Tuner

find_package(OpenMP REQUIRED)

file(GLOB_RECURSE SRCS_tuner "applications/tuner/*.cpp")
add_executable(tuner ${SRCS_tuner})
target_include_directories(tuner PRIVATE "applications/tuner/")
target_link_libraries(tuner PRIVATE argparser 
                                    plot
                                    config
                                    OpenMP::OpenMP_CXX
                                    -static-libgcc 
                                    -static-libstdc++)

################################################################################
##############################   CUSTOM TARGETS   ##############################
################################################################################

# Add Doxygen target
add_custom_target(documentation rm -rf html && doxygen
                  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/../doc/")