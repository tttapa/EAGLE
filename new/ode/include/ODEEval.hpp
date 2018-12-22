#pragma once

#include "ODEResult.hpp"
#include <Array.hpp>
#include <Time.hpp>
#include <algorithm>
#include <cassert>

/** 
 * @brief   Interpolates linearly for two given points `(t1, x1)`, `(t2, x2)`,
 *          for a given time `t`.
 */
template <class X>
inline X interpolate(double t1, double t2, const X &x1, const X &x2, double t) {
    return (t - t1) / (t2 - t1) * (x2 - x1) + x1;
}

/** 
 * @brief   Sample a given ODEResult with a specific sample time, over a 
 *          specific time frame.
 * 
 * @param   result
 *          A struct containing 
 */
template <class X>
std::vector<X> sampleODEResult(const std::vector<double> &tv,
                               const std::vector<X> &xv, double t_start,
                               double Ts, double t_end) {
    assert(tv.size() == xv.size());
    size_t N = numberOfSamplesInTimeRange(t_start, Ts, t_end);
    std::vector<X> sampled;
    sampled.reserve(N);
    const auto tbegin_it = tv.begin();
    auto tcurr_it        = tbegin_it;
    const auto tend_it   = tv.end();
    for (size_t i = 0; i < N; ++i) {
        double t = t_start + Ts * i;
        // Find t1 and t2 such that: t1 < t <= t2
        auto t2_it = std::lower_bound(tcurr_it, tend_it, t);  // *t2_it >= t
        if (t2_it == tbegin_it)  // t = tv[0]
            ++t2_it;
        if (t2_it == tend_it)    // t > tv.back()
            break;               // no more data points to interpolate from
        auto t1_it = t2_it - 1;  // *t1_it < t
        auto idx_1 = t1_it - tbegin_it;
        auto x1    = xv[idx_1];
        auto x2    = xv[idx_1 + 1];
        auto x = interpolate(*t1_it, *t2_it, x1, x2, t);
        sampled.push_back(x);
        tcurr_it = t1_it;  // it doesn't make sense to start searching
        ;                  // from the start each time
    }
    return sampled;
}

template <class X>
std::vector<X> sampleODEResult(const ODEResultX<X> &result, double t_start,
                               double Ts, double t_end) {
    return sampleODEResult(result.time, result.solution, t_start, Ts, t_end);
}