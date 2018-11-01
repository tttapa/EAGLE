#pragma once

#include <cmath>

constexpr inline size_t numberOfSamplesInTimeRange(double t_start, double Ts,
                                                   double t_end) {
    return floor((t_end - t_start) / Ts) + 1;
}