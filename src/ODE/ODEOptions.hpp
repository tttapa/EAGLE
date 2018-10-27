#pragma once

struct AdaptiveODEOptions {
    double t_start = 0;     // initial value for independent variable
    double t_end   = 1;     // final value for independent variable
    double epsilon = 1e-6;  // tolerance
    double h_start = 1e-2;  // initial step size
    double h_min   = 1e-6;  // minimum step size
    size_t maxiter = 1e6;   // maximum number of iterations
};