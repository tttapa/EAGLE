#pragma once

#include <Def.hpp>
#include <Randn.hpp>
#include <Config.hpp>
#include <TunerConfig.hpp>

constexpr static size_t Nq = Nx_att - 1;
constexpr static size_t Nr = Nu_att;

struct Weights {
    ColVector<Nq> Q_diag;
    ColVector<Nr> R_diag;
    auto Q() const { return diag(Q_diag); }
    auto R() const { return diag(R_diag); }
    double cost;
    bool operator<(const Weights &rhs) const { return this->cost < rhs.cost; }

    void mutate() {
        auto dQ = randn(Config::Tuner::varQ);
        Q_diag += dQ;
        clamp(Q_diag, Config::Tuner::Qmin, Config::Tuner::Qmax);
        auto dR = randn(Config::Tuner::varR);
        R_diag += dR;
        clamp(R_diag, Config::Tuner::Rmin, Config::Tuner::Rmax);

        // TODO: symmetry
        Q_diag[1] = Q_diag[0];
        Q_diag[4] = Q_diag[3];
        Q_diag[7] = Q_diag[6];

        R_diag[1] = R_diag[0];
    }

    void crossOver(const Weights &parent1, const Weights &parent2) {
        static std::default_random_engine generator;
        static std::uniform_int_distribution<size_t> q_distr(0, Nq);
        static std::uniform_int_distribution<size_t> r_distr(0, Nr);
        size_t q_idx = q_distr(generator);
        size_t r_idx = r_distr(generator);

        for (size_t i = 0; i < q_idx; ++i)
            this->Q_diag[i] = parent1.Q_diag[i];
        for (size_t i = q_idx; i < Nq; ++i)
            this->Q_diag[i] = parent2.Q_diag[i];
        for (size_t i = 0; i < r_idx; ++i)
            this->R_diag[i] = parent1.R_diag[i];
        for (size_t i = r_idx; i < Nr; ++i)
            this->R_diag[i] = parent2.R_diag[i];
    }

    void renormalize() {
        const double factor = 1.0 / R_diag[0];
        for (double &d : Q_diag)
            d *= factor;
        for (double &d : R_diag)
            d *= factor;
    }
};