#pragma once

#include "ODEResult.hpp"
#include <Matrix/Array.hpp>
#include <algorithm>

template <class X>
inline X interpolate(double t1, double t2, const X &x1, const X &x2, double t) {
    return (t - t1) / (t2 - t1) * (x2 - x1) + x1;
}

template <class X>
std::vector<X> sampleODEResult(const ODEResultX<X> &result, double t_start,
                               double Ts, double t_end) {
    size_t N = floor((t_end - t_start) / Ts) + 1;
    std::vector<X> sampled;
    sampled.reserve(N);
    const auto tbegin_it = result.time.begin();
    auto tcurr_it        = tbegin_it;
    const auto tend_it   = result.time.end();
    for (size_t i = 0; i < N; ++i) {
        double t   = t_start + Ts * i;
        auto t2_it = std::lower_bound(tcurr_it, tend_it, t);  // *t2_it >= t
        if (t2_it == tbegin_it)
            ++t2_it;
        if (t2_it == tend_it)
            break;
        auto t1_it = t2_it - 1;  // *t1_it < t
        auto idx_1 = t1_it - tbegin_it;
        auto x1    = result.solution[idx_1];
        auto x2    = result.solution[idx_1 + 1];
        sampled.push_back(interpolate(*t1_it, *t2_it, x1, x2, t));
        tcurr_it = t1_it;  // it doesn't make sense to start searching
        ;                  // from the start each time
    }
    return sampled;
}