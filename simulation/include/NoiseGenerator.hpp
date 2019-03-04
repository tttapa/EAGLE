#pragma once

#include <Matrix.hpp>
#include <random>

template <size_t N>
struct NoiseGenerator {
    virtual ~NoiseGenerator()                                        = default;
    virtual ColVector<N> operator()(double t, const ColVector<N> &v) = 0;
};

template <size_t N>
class GaussianNoiseGenerator : public NoiseGenerator<N> {
  public:
    GaussianNoiseGenerator(const Array<double, N> &variances, uint32_t seed = 1)
        : distributions(varianceToDistributions(variances)) {
        rgen.seed(seed);
    }
    GaussianNoiseGenerator(const RowVector<N> &variances, uint32_t seed = 1)
        : GaussianNoiseGenerator(variances[0], seed) {}

    ColVector<N> operator()(double /* t */, const ColVector<N> &v) override {
        return v + randomVector();
    }
    static Array<std::normal_distribution<double>, N>
    varianceToDistributions(const Array<double, N> &variances) {
        Array<std::normal_distribution<double>, N> distributions;
        std::transform(
            variances.begin(), variances.end(), distributions.begin(),
            [](double variance) {
                return std::normal_distribution<double>{0, sqrt(variance)};
            });
        return distributions;
    }
    ColVector<N> randomVector() {
        ColVector<N> result = {};
        std::transform(distributions.begin(), distributions.end(),
                       result.begin(),
                       [this](std::normal_distribution<double> &dist) {
                           return dist(rgen);
                       });
        return result;
    }

  private:
    std::default_random_engine rgen;
    Array<std::normal_distribution<double>, N> distributions;
};

template <size_t N>
struct NoNoiseGenerator : public NoiseGenerator<N> {
    ColVector<N> operator()(double /* t */, const ColVector<N> &v) override {
        return v;
    }
};