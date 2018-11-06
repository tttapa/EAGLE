#pragma once

#include "Quaternion.hpp"

template <size_t N>
ColVector<N> quaternionStatesAdd(const ColVector<N> &a, const ColVector<N> &b) {
    ColVector<N> result;
    assignBlock<4, N, 0, 1>(result) =
        getBlock<4, N, 0, 1>(a) + getBlock<4, N, 0, 1>(b);
    assignBlock<0, 4, 0, 1>(result) =
        quatmultiply(getBlock<0, 4, 0, 1>(a), getBlock<0, 4, 0, 1>(b));
    return result;
}

template <size_t N>
ColVector<N> quaternionStatesSub(const ColVector<N> &a, const ColVector<N> &b) {
    ColVector<N> result;
    assignBlock<4, N, 0, 1>(result) =
        getBlock<4, N, 0, 1>(a) - getBlock<4, N, 0, 1>(b);
    assignBlock<0, 4, 0, 1>(result) =
        quatDifference(getBlock<0, 4, 0, 1>(a), getBlock<0, 4, 0, 1>(b));
    return result;
}