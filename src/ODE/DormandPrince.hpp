#pragma once

#include <cmath>    // pow
#include <cstddef>  // size_t
#include <exception>
#include <limits>  // epsilon

#include "DormandPrinceConstants.hpp"
#include "ODEOptions.hpp"
#include "ODEResult.hpp"

inline double norm(double x) { return fabs(x); }

template <class IteratorTimeBegin, class IteratorXBegin, class F, class T>
ODEResultCode dormandPrince(IteratorTimeBegin timeresult,
                            IteratorXBegin xresult,
                            F f,        // function f(double t, T x)
                            T x_start,  // initial value
                            const AdaptiveODEOptions &opt  // options
) {
    using namespace DormandPrinceConstants;
    using std::isfinite;
    double t = opt.t_start;
    T x      = x_start;
    double h = opt.h_start;

    *timeresult++ = {t};
    *xresult++    = {x};

    ODEResultCode resultCode = ODEResultCodes::SUCCESS;

    for (size_t i = 0; i < opt.maxiter; ++i) {
        if (!isfinite(x))
            throw std::runtime_error{"Error: x is not finite"};
        if (!isfinite(t))
            throw std::runtime_error{"Error: t is not finite"};
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

        double s = pow(h * opt.epsilon / 2.0 / error, 1.0 / 5.0);

        double h_new = h;
        double t_new = t;
        T x_new      = x;

        // adaptive step size, but conservative
        if (s < 1.0)
            h_new = s * h;
        if (s >= 2)
            h_new = 2.0 * h;

        // when discontinuity happens, skip over it
        if (h_new < opt.h_min) {
            h_new = opt.h_min;
            t_new += opt.h_min;
            resultCode |= ODEResultCodes::MINIMUM_STEP_SIZE_REACHED;
        }

        if (error < opt.epsilon) {
            t_new += h;
            x_new += h * (b1 * K1 + b3 * K3 + b4 * K4 + b5 * K5 + b6 * K6);
            *timeresult++ = {fmin(t_new, opt.t_end)};
            *xresult++    = {x_new};
        }

        if (t_new >= opt.t_end) {
            // finished
            return resultCode;
        } else if (t_new + h_new > opt.t_end)
            // finishes on next iteration, don't jump over t_end
            h_new = opt.t_end - t_new;

        h = h_new;
        t = t_new;
        x = x_new;
    }
    resultCode |= ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED;
    return resultCode;
}

template <class F, class T>
ODEResultX<T> dormandPrince(F f,        // function f(double t, T x)
                            T x_start,  // initial value
                            const AdaptiveODEOptions &opt  // options
) {
    std::vector<double> t_v;
    std::vector<T> x_v;
    ODEResultCode resultCode = dormandPrince(
        std::back_inserter(t_v), std::back_inserter(x_v), f, x_start, opt);
    return {t_v, x_v, resultCode};
}