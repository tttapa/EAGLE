#pragma once

#include <cstddef>

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

    constexpr bool operator!=(const Array<T, N> &rhs) const {
        return !(*this == rhs);
    }

    static constexpr size_t length = N;
};
