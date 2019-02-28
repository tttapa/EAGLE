add_executable(py-plot-simulation
    main.cpp
)

target_link_libraries(py-plot-simulation
    PUBLIC 
        py-drone
        plot
)