#pragma once

#include <Array.hpp>
#include <cmath>  // fabs

inline bool isAlmostEqual(double valuelhs, double valuerhs, double epsilon) {
    return fabs(valuelhs - valuerhs) < epsilon;
}

template <class T, size_t N, class U>
bool isAlmostEqual(const Array<T, N> &arraylhs, const Array<T, N> &arrayrhs,
                   U epsilon) {
    for (size_t i = 0; i < N; ++i)
        if (!isAlmostEqual(arraylhs[i], arrayrhs[i], epsilon))
            return false;
    return true;
}