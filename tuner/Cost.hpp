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
    const Quaternion q_thr;  // threshold = factor * abs(q_ref - q_0)
    Quaternion q_prev;

    // Contains +1.0 if the reference is greater than the initial value, 
    // and -1.0 if the reference is less than the initial value
    const ColVector<4> dir;

    // The time point where the curve last crossed the settling interval
    ColVector<4> lastthrescross = -ones<4, 1>();
    // Whether the next crossing of the settling interval will be rising (true)
    // or falling (false)
    TColVector<bool, 4> nextthrescrossrising;
    // Whether the curve is rising (true) or falling (false)
    TColVector<bool, 4> rising; 
    // The last relative maximum error (extrema of overshoot ringing)
    ColVector<4> maxerr;
    // Whether the curve has crossed the reference or not
    TColVector<bool, 4> crossed = {};

    // The time point where the curve settled to within the settling interval
    ColVector<4> settled   = -ones<4, 1>();
    // The time point where the curve first entered the settling interval
    ColVector<4> riseTime  = -ones<4, 1>();
    // The extremum of the error after crossing the reference
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