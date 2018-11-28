#pragma once

#include <algorithm>  // copy
#include <cmath>      // isfinite
#include <cstddef>
#include <numeric>  // accumulate
#include <type_traits>

template <class T, size_t N>
struct Array;

template <class T>
struct getArrayTypeBase {
    using type = void;
};
template <class T, size_t N>
struct getArrayTypeBase<Array<T, N>> {
    using type = T;
};
template <typename T>
struct getArrayType : getArrayTypeBase<T> {};

template <class T>
struct getArrayLengthBase {
    static constexpr size_t value = 0;
};
template <class T, size_t N>
struct getArrayLengthBase<Array<T, N>> {
    static constexpr size_t value = N;
};
template <typename T>
struct getArrayLength : getArrayLengthBase<T> {};

template <class T, size_t N>
struct Array {
    T data[N];

    constexpr T &operator[](size_t index) { return data[index]; }
    constexpr const T &operator[](size_t index) const { return data[index]; }
    constexpr T *begin() { return &data[0]; }
    constexpr const T *begin() const { return &data[0]; }
    constexpr T *end() { return &data[N]; }
    constexpr const T *end() const { return &data[N]; }

    constexpr bool operator==(const Array<T, N> &rhs) const {
        if (this == &rhs)
            return true;
        for (size_t i = 0; i < N; i++)
            if ((*this)[i] != rhs[i])
                return false;
        return true;
    }

    constexpr Array<T, N> operator*(double rhs) const {
        Array<T, N> result = *this;
        result *= rhs;
        return result;
    }

    constexpr Array<T, N> &operator*=(double rhs) {
        for (auto &el : *this)
            el *= rhs;
        return *this;
    }

    constexpr Array<T, N> operator/(double rhs) const {
        Array<T, N> result = *this;
        result /= rhs;
        return result;
    }

    constexpr Array<T, N> &operator/=(double rhs) {
        for (auto &el : *this)
            el /= rhs;
        return *this;
    }

    constexpr Array<T, N> operator+(const Array<T, N> &rhs) const {
        Array<T, N> result = *this;
        result += rhs;
        return result;
    }

    constexpr Array<T, N> &operator+=(const Array<T, N> &rhs) {
        for (size_t i = 0; i < N; ++i)
            (*this)[i] += rhs[i];
        return *this;
    }

    constexpr Array<T, N> operator-(const Array<T, N> &rhs) const {
        Array<T, N> result = *this;
        result -= rhs;
        return result;
    }

    constexpr Array<T, N> &operator-=(const Array<T, N> &rhs) {
        for (size_t i = 0; i < N; ++i)
            (*this)[i] -= rhs[i];
        return *this;
    }

    constexpr Array<T, N> operator-() const {
        Array<T, N> result = *this;
        for (size_t i = 0; i < N; ++i)
            (result)[i] = -(result)[i];
        return result;
    }

    constexpr bool operator!=(const Array<T, N> &rhs) const {
        return !(*this == rhs);
    }

    /**
     * @brief   Assignment operator for Array<T, 1>.
     */
    template <bool EnableBool = true>
    constexpr typename std::enable_if<N == 1 && EnableBool, Array<T, 1> &>::type
    operator=(const T &t) {
        data[0] = t;
        return *this;
    }

    /**
     * @brief   Implicit conversion from Array<T, 1> to T&.
     */
    template <bool EnableBool = true>
    constexpr operator typename std::add_lvalue_reference<
        typename std::enable_if<N == 1 && EnableBool, T>::type>::type() {
        return data[0];
    }

    /**
     * @brief   Implicit conversion from const Array<T, 1> to const T&.
     */
    template <bool EnableBool = true>
    constexpr
    operator typename std::add_lvalue_reference<typename std::add_const<
        typename std::enable_if<N == 1 && EnableBool, T>::type>::type>::type()
        const {
        return data[0];
    }

    /**
     * @brief   Implicit conversion from Array<Array<T, 1>, 1> to T&.
     */
    template <bool EnableBool = true>
    operator typename std::add_lvalue_reference<typename std::enable_if<
        N == 1 && getArrayLength<T>::value == 1 && EnableBool,
        typename getArrayType<T>::type>::type>::type() {
        return data[0][0];
    }

    /**
     * @brief   Implicit conversion from const Array<Array<T, 1>, 1> to const T&.
     */
    template <bool EnableBool = true>
    operator typename std::add_lvalue_reference<
        typename std::add_const<typename std::enable_if<
            N == 1 && getArrayLength<T>::value == 1 && EnableBool,
            typename getArrayType<T>::type>::type>::type>::type() const {
        return data[0][0];
    }

    /** 
     * @brief   Comparison of arrays: less than
     */
    constexpr Array<decltype(data[0] < data[0]), N>
    operator<(const Array<T, N> &rhs) const {
        Array<decltype(data[0] < data[0]), N> result = {};
        for (size_t i = 0; i < N; ++i)
            result[i] = this->data[i] < rhs.data[i];
        return result;
    }

    /** 
     * @brief   Comparison of arrays: less than or equal
     */
    constexpr Array<decltype(data[0] <= data[0]), N>
    operator<=(const Array<T, N> &rhs) const {
        Array<decltype(data[0] <= data[0]), N> result = {};
        for (size_t i = 0; i < N; ++i)
            result[i] = this->data[i] <= rhs.data[i];
        return result;
    }

    /** 
     * @brief   Comparison of arrays: greater than
     */
    constexpr Array<decltype(data[0] > data[0]), N>
    operator>(const Array<T, N> &rhs) const {
        Array<decltype(data[0] > data[0]), N> result = {};
        for (size_t i = 0; i < N; ++i)
            result[i] = this->data[i] > rhs.data[i];
        return result;
    }

    /** 
     * @brief   Comparison of arrays: greater than or equal
     */
    constexpr Array<decltype(data[0] >= data[0]), N>
    operator>=(const Array<T, N> &rhs) const {
        Array<decltype(data[0] >= data[0]), N> result = {};
        for (size_t i = 0; i < N; ++i)
            result[i] = this->data[i] >= rhs.data[i];
        return result;
    }

    constexpr static Array<T, N> fromCppArray(const T (&a)[N]) {
        Array<T, N> result = {};
        std::copy(a, a + N, result.begin());
        return result;
    }

    constexpr static Array<T, N> fromCArray(const T *a) {
        Array<T, N> result = {};
        std::copy(a, a + N, result.begin());
        return result;
    }

    static constexpr size_t length = N;
    using type                     = T;
};

template <class T, size_t N>
constexpr Array<T, N> abs(const Array<T, N> &a) {
    using namespace std;
    Array<T, N> result = a;
    for (auto &e : result)
        e = abs(e);
    return result;
}

template <class T, size_t N>
bool isfinite(const Array<T, N> &a) {
    using std::isfinite;
    return std::all_of(a.begin(), a.end(),
                       [](const T &e) { return isfinite(e); });
}

template <class T, size_t N>
constexpr Array<T, N> operator*(double lhs, const Array<T, N> &rhs) {
    Array<T, N> result = rhs;
    result *= lhs;
    return result;
}

template <class T>
constexpr Array<T, 1> operator*(Array<T, 1> lhs, Array<T, 1> rhs) {
    return {T{lhs} * T{rhs}};
}

template <class T>
constexpr Array<T, 1> operator/(Array<T, 1> lhs, Array<T, 1> rhs) {
    return {T{lhs} / T{rhs}};
}

template <class T, class U,
          typename = std::enable_if<getArrayLength<U>::value == 0>>
constexpr auto operator+(const Array<T, 1> &a, const U &u)
    -> decltype(static_cast<T>(a) + u) {
    return static_cast<T>(a) + u;
}

template <class T, class U,
          typename = std::enable_if<getArrayLength<U>::value == 0>>
constexpr auto operator+(const U &u, const Array<T, 1> &a)
    -> decltype(u + static_cast<T>(a)) {
    return u + static_cast<T>(a);
}

template <class T, class U,
          typename = std::enable_if<getArrayLength<U>::value == 0>>
constexpr auto operator-(const Array<T, 1> &a, const U &u)
    -> decltype(static_cast<T>(a) - u) {
    return static_cast<T>(a) - u;
}

template <class T, class U,
          typename = std::enable_if<getArrayLength<U>::value == 0>>
constexpr auto operator-(const U &u, const Array<T, 1> &a)
    -> decltype(u - static_cast<T>(a)) {
    return u - static_cast<T>(a);
}

template <class T, class U,
          typename = std::enable_if<getArrayLength<U>::value == 0>>
constexpr auto operator/(const U &u, const Array<T, 1> &a)
    -> decltype(u / static_cast<T>(a)) {
    return u / static_cast<T>(a);
}

template <class T, size_t N>
constexpr T sum(const Array<T, N> &a) {
    return std::accumulate(a.begin(), a.end(), T{});
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator<(const U &u, const Array<T, 1> &a) {
    return u < static_cast<T>(a);
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator<(const Array<T, 1> &a, const U &u) {
    return static_cast<T>(a) < u;
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator<=(const U &u, const Array<T, 1> &a) {
    return u <= static_cast<T>(a);
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator<=(const Array<T, 1> &a, const U &u) {
    return static_cast<T>(a) <= u;
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator>(const U &u, const Array<T, 1> &a) {
    return u > static_cast<T>(a);
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator>(const Array<T, 1> &a, const U &u) {
    return static_cast<T>(a) > u;
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator>=(const U &u, const Array<T, 1> &a) {
    return u >= static_cast<T>(a);
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator>=(const Array<T, 1> &a, const U &u) {
    return static_cast<T>(a) >= u;
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator==(const U &u, const Array<T, 1> &a) {
    return u == static_cast<T>(a);
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator==(const Array<T, 1> &a, const U &u) {
    return static_cast<T>(a) == u;
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator!=(const U &u, const Array<T, 1> &a) {
    return u != static_cast<T>(a);
}

template <class T, class U>
constexpr typename std::enable_if<getArrayLength<U>::value == 0, bool>::type
operator!=(const Array<T, 1> &a, const U &u) {
    return static_cast<T>(a) != u;
}

//

template <class T, class U>
constexpr U &operator+=(U &u, const Array<T, 1> &a) {
    return u += static_cast<T>(a);
}

template <class T, class U>
constexpr Array<T, 1> &operator+=(Array<T, 1> &a, const U &u) {
    return a += {u};
}

//

template <class T, class F>
constexpr auto map(const T &t, F &&f) {
    return f(t);
}

template <class T, size_t N, class F>
constexpr auto map(const Array<T, N> &a, F &&f) {
    Array<decltype(map(a[0], f)), N> result = {};
    for (size_t i = 0; i < N; ++i)
        result[i] = map(a[i], f);
    return result;
}