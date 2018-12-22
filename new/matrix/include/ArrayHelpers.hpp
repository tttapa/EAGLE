#pragma once

#include "Array.hpp"
#include <algorithm>

template <class T>
class Incrementor {
  public:
    constexpr Incrementor(T start = 0, T increment = 1)
        : value(start), increment(increment) {}
    constexpr T operator()() {
        T temp = value;
        value += increment;
        return temp;
    }

  private:
    T value;
    const T increment;
};

/**
 * @brief   Generate an array where the first value is given, and the subsequent
 *          values are calculated as the previous value incremented with a given
 *          value:  
 *          @f$ x[0] = start @f$  
 *          @f$ x[k+1] = x[k] + increment @f$.
 * 
 * For example:  
 * ```
 * auto x = generateArray<unsigned int, 4>(2, 3);
 * ```
 * is equivalent to  
 * ```
 * Array<unsigned int, 4> x = {2, 5, 8, 11};
 * ```
 * 
 * @tparam  T 
 *          The type of the elements in the array.
 * @tparam  N 
 *          The number of elements in the array.
 * 
 * @param   start
 *          The first value in the array.
 * @param   increment
 *          The value to add to each subsequent element of the array.
 * 
 * @return  The generated array.
 */
template <class T, size_t N, class U>
constexpr Array<T, N> generatedArray(U start = 0, U increment = 1) {
    Array<T, N> array = {};
    Incrementor<U> g(start, increment);
    std::generate(array.begin(), array.end(), g);
    return array;
}

template <class T, size_t N>
constexpr Array<T, N> filledArray(const T &value) {
    Array<T, N> array = {};
    std::fill(array.begin(), array.end(), value);
    return array;
}