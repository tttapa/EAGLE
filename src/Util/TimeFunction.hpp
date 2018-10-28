#pragma once

#include <functional>

template <class T>
struct TimeFunctionT {
  public:
    virtual T operator()(double t) = 0;
};

template <class T>
struct FunctionalTimeFunctionT : public TimeFunctionT<T> {
    FunctionalTimeFunctionT(std::function<T(double)> f) : f(f) {}
    std::function<T(double)> f;
    T operator()(double t) override { return f(t); }
};

template <class T>
struct FunctionPointerTimeFunctionT : public TimeFunctionT<T> {
    using FunctionPtr_t = T (*)(double);
    FunctionPointerTimeFunctionT(FunctionPtr_t f) : f(f) {}
    FunctionPtr_t f;
    T operator()(double t) override { return f(t); }
};

template <class T>
struct ConstantTimeFunctionT : public TimeFunctionT<T> {
    ConstantTimeFunctionT(const T &f) : f(f) {}
    const T f;
    T operator()(double /* t */) override { return f; }
};