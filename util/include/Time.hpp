#pragma once

#include <cstddef>

constexpr inline size_t numberOfSamplesInTimeRange(double t_start, double Ts,
                                                   double t_end) {
    return size_t((t_end - t_start) / Ts) + 1;
}