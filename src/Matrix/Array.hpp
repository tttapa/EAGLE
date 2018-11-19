#pragma once

#include <cstddef>
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
    static constexpr size_t length = 0;
};
template <class T, size_t N>
struct getArrayLengthBase<Array<T, N>> {
    static constexpr size_t length = N;
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
     * @brief   Implicit conversion from Array<T, 1> to T&.
     */
    template <bool EnableBool = true>
    operator typename std::add_lvalue_reference<
        typename std::enable_if<N == 1 && EnableBool, T>::type>::type() {
        return data[0];
    }

    /**
     * @brief   Implicit conversion from const Array<T, 1> to const T&.
     */
    template <bool EnableBool = true>
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
        N == 1 && getArrayLength<T>::length == 1 && EnableBool,
        typename getArrayType<T>::type>::type>::type() {
        return data[0][0];
    }

    /**
     * @brief   Implicit conversion from const Array<Array<T, 1>, 1> to const T&.
     */
    template <bool EnableBool = true>
    operator typename std::add_lvalue_reference<
        typename std::add_const<typename std::enable_if<
            N == 1 && getArrayLength<T>::length == 1 && EnableBool,
            typename getArrayType<T>::type>::type>::type>::type() const {
        return data[0][0];
    }

    static constexpr size_t length = N;
    using type                     = T;
};

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