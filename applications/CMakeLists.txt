### Plot Simulation

file(GLOB_RECURSE SRCS_plot_simulation "plot-simulation/*.cpp")
add_executable(plot-simulation ${SRCS_plot_simulation})
target_link_libraries(plot-simulation PRIVATE argparser 
                                              plot
                                              config)


file(GLOB_RECURSE SRCS_plot_log "plot-log/*.cpp")
add_executable(plot-log ${SRCS_plot_log})
target_link_libraries(plot-log PRIVATE argparser 
                                              plot
                                              config
                                              drone-log-loader)

### Tuner

find_package(OpenMP REQUIRED)

file(GLOB_RECURSE SRCS_tuner "tuner/*.cpp")
add_executable(tuner ${SRCS_tuner})
target_include_directories(tuner PRIVATE "tuner/")
target_link_libraries(tuner PRIVATE argparser 
                                    plot
                                    config
                                    OpenMP::OpenMP_CXX
                                    -static-libgcc 
                                    -static-libstdc++)


### Genetics

file(GLOB_RECURSE SRCS_genetics "genetics/*.cpp")
add_executable(genetics ${SRCS_genetics})
target_link_libraries(genetics PRIVATE genetic-tuner)

### Python Plot Simulation

add_subdirectory("py-plot-simulation")