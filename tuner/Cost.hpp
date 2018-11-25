#pragma once

#include <Config.hpp>
#include <Drone/Drone.hpp>

#define DEBUG
// #define PLOT_ALL_QUATERNION_STATES

class RealTimeCostCalculator {
  public:
    RealTimeCostCalculator(ContinuousModel<Nx_att, Nu_att, Ny_att> &model,
                           const Quaternion &q_ref, double factor,
                           const Quaternion &q_0);

    bool operator()(size_t k, const ColVector<Nx_att> &x,
                    const ColVector<Nu_att> &u);
    double getCost() const;

#ifdef DEBUG
    void plot() const;
#endif

  private:
    ContinuousModel<Nx_att, Nu_att, Ny_att> &model;
    const Quaternion q_ref;
    const Quaternion q_thr;  // threshold
    Quaternion q_prev;

    const ColVector<4> dir;

    TColVector<bool, 4> rising = {0, 1, 1, 1};  // TODO!!!
    ColVector<4> maxerr;
    TColVector<bool, 4> crossed = {};

    ColVector<4> settled   = -ones<4, 1>();
    ColVector<4> riseTime  = -ones<4, 1>();
    ColVector<4> overshoot = zeros<4, 1>();

#ifdef DEBUG
    std::vector<Quaternion> q_v = {};
    std::vector<double> k_v     = {};
#endif
    static constexpr ColVector<4> getDirection(const Quaternion &q_ref,
                                               const Quaternion &q_0) {
        ColVector<4> result = {};
        for (size_t i = 0; i < 4; ++i) {
            result[i] = q_ref[i] >= q_0[i] ? 1.0 : -1.0;
        }
        return result;
    }
};

double getRiseTimeCost(Drone::FixedClampAttitudeController &ctrl,
                       ContinuousModel<Nx_att, Nu_att, Ny_att> &model,
                       Quaternion q_ref, double q_refNormSqThres,
                       ColVector<Nx_att> x0 = {1});

double getCost(Drone::FixedClampAttitudeController &ctrl,
               ContinuousModel<Nx_att, Nu_att, Ny_att> &model);