#pragma once

#include <cmath>  // M_PI

// Convert degrees to radians
constexpr long double operator""_deg(long double deg) {
    return deg * M_PI / 180.0;
}
// Convert degrees to radians
constexpr long double operator""_deg(unsigned long long deg) {
    return static_cast<long double>(deg) * M_PI / 180.0;
}