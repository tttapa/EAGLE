#pragma once

#include "Params.hpp"
#include <Matrix.hpp>

template <class T, size_t Nx, size_t Nu>
class Model {
  public:
    using VecX_t = ColVector<T, Nx>;
    using VecU_t = ColVector<T, Nu>;

    virtual VecX_t operator()(const VecX_t &x, const VecU_t &u) = 0;
};