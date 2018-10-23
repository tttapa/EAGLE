#include <cmath>    // pow
#include <cstddef>  // size_t
#include <limits>   // epsilon
#include <utility>  // pair
#include <vector>   // vector

#include <iostream>  // TODO

#pragma once

namespace DormandPrinceConstants {

constexpr double a21 = 1.0 / 5.0;
constexpr double a31 = 3.0 / 40.0;
constexpr double a32 = 9.0 / 40.0;
constexpr double a41 = 44.0 / 45.0;
constexpr double a42 = -56.0 / 15.0;
constexpr double a43 = 32.0 / 9.0;
constexpr double a51 = 19372.0 / 6561.0;
constexpr double a52 = -25360.0 / 2187.0;
constexpr double a53 = 64448.0 / 6561.0;
constexpr double a54 = -212.0 / 729.0;
constexpr double a61 = 9017.0 / 3168.0;
constexpr double a62 = -355.0 / 33.0;
constexpr double a63 = 46732.0 / 5247.0;
constexpr double a64 = 49.0 / 176.0;
constexpr double a65 = -5103.0 / 18656.0;
constexpr double a71 = 35.0 / 384.0;
constexpr double a72 = 0.0;
constexpr double a73 = 500.0 / 1113.0;
constexpr double a74 = 125.0 / 192.0;
constexpr double a75 = -2187.0 / 6784.0;
constexpr double a76 = 11.0 / 84.0;

constexpr double c2  = 1.0 / 5.0;
constexpr double c3  = 3.0 / 10.0;
constexpr double c4  = 4.0 / 5.0;
constexpr double c5  = 8.0 / 9.0;
constexpr double c6  = 1.0;
constexpr double c7  = 1.0;
constexpr double b1  = 35.0 / 384.0;
constexpr double b2  = 0.0;
constexpr double b3  = 500.0 / 1113.0;
constexpr double b4  = 125.0 / 192.0;
constexpr double b5  = -2187.0 / 6784.0;
constexpr double b6  = 11.0 / 84.0;
constexpr double b7  = 0.0;
constexpr double b1p = 5179.0 / 57600.0;
constexpr double b2p = 0.0;
constexpr double b3p = 7571.0 / 16695.0;
constexpr double b4p = 393.0 / 640.0;
constexpr double b5p = -92097.0 / 339200.0;
constexpr double b6p = 187.0 / 2100.0;
constexpr double b7p = 1.0 / 40.0;

}  // namespace DormandPrinceConstants

double norm(double x) { return fabs(x); }

template <class F, class T>
std::pair<std::vector<double>, std::vector<T>>
dormandPrince(F f,             // function f(double t, T x)
              double t_start,  // initial value for independent variable
              double t_end,    // final value for independent variable
              double epsilon,  // tolerance
              double h_start,  // initial step size
              T x_start,       // initial value
              size_t maxiter   // maximum number of iterations
) {
    using namespace DormandPrinceConstants;

    double t = t_start;
    T x      = x_start;
    double h = h_start;

    std::vector<double> t_v = {t_start};
    std::vector<T> x_v      = {x_start};

    for (size_t i = 0; i < maxiter; ++i) {
        // Calculate all seven slopes
        T K1 = f(t, x);
        T K2 = f(t + c2 * h, x + h * (a21 * K1));
        T K3 = f(t + c3 * h, x + h * (a31 * K1 + a32 * K2));
        T K4 = f(t + c4 * h, x + h * (a41 * K1 + a42 * K2 + a43 * K3));
        T K5 =
            f(t + c5 * h, x + h * (a51 * K1 + a52 * K2 + a53 * K3 + a54 * K4));
        T K6 = f(t + h, x + h * (a61 * K1 + a62 * K2 + a63 * K3 + a64 * K4 +
                                 a65 * K5));
        T K7 = f(t + h, x + h * (a71 * K1 + a72 * K2 + a73 * K3 + a74 * K4 +
                                 a75 * K5 + a76 * K6));

        double error =
            norm((b1 - b1p) * K1 + (b3 - b3p) * K3 + (b4 - b4p) * K4 +
                 (b5 - b5p) * K5 + (b6 - b6p) * K6 + (b7 - b7p) * K7);

        double s = pow(h * epsilon / 2.0 / error, 1.0 / 5.0);

        double h_new = h;
        double t_new = t;
        T x_new      = x;

        // adaptive step size, but conservative
        if (s < 1.0 || s >= 2)
            h_new = s * h;

        double h_min = 1e-6; // TODO TODO

        if (h_new < h_min) {
            h_new = h_min;
            t_new += h_min;
        }

        if (error < epsilon) {
            t_new += h;
            x_new += h * (b1 * K1 + b3 * K3 + b4 * K4 + b5 * K5 + b6 * K6);
            t_v.push_back(t_new);
            x_v.push_back(x_new);
        }

        if (t_new >= t_end)
            // finished
            return {t_v, x_v};
        else if (t_new + h_new > t_end)
            // finishes on next iteration, don't jump over t_end
            h_new = t_end - t_new;

        h = h_new;
        t = t_new;
        x = x_new;
    }
    return {t_v, x_v};  // TODO
}