#include "Cost.hpp"
#include <Plot.hpp>

constexpr double infinity = std::numeric_limits<double>::infinity();

RealTimeCostCalculator::RealTimeCostCalculator(
    ContinuousModel<Nx_att, Nu_att, Ny_att> &model, const Quaternion &q_ref,
    double factor, const Quaternion &q_0)
    : model{model}, q_ref{q_ref}, factor{factor}, q_prev{q_0}, dir{getDirection(
                                                                   q_ref,
                                                                   q_0)} {}

bool RealTimeCostCalculator::operator()(size_t k, const ColVector<Nx_att> &x,
                                        const ColVector<Nu_att> &u) {
    using namespace std;
    DroneAttitudeOutput yy = model.getOutput(x, u);
    auto q                 = yy.getOrientation();

    bool continueSimulation = true;

    auto q_err = q_ref - q;
    auto q_dif = q - q_prev;

#ifdef DEBUG
    cerr << endl                                                   //
         << "k = " << k << endl                                    //
         << "q_ref       = " << transpose(q_ref)                   //
         << "q_thres_err = " << transpose(factor * q_ref)          //
         << "q_err       = " << transpose(q_err)                   //
         << "q_thres_dif = " << transpose(factor * q_ref / 238.0)  //
         << "q_dif       = " << transpose(q_dif)                   //
         << "riseTime    = " << transpose(riseTime)                //
         << "settled     = " << transpose(settled)                 //
         << "dir         = " << transpose(dir);

    q_v.push_back(q);
    k_v.push_back(k);
#endif

    for (size_t i = 0; i < 4; ++i) {
#ifdef DEBUG
        cerr << i << endl;
        cerr << endl
             << "dir[i] * q_err[i]               = " << dir[i] * q_err[i]
             << endl
             << "dir[i] * abs(q_ref[i]) * factor = "
             << dir[i] * abs(q_ref[i] * factor) << endl;
#endif

        if (riseTime[i] == -1.0) {
            if (dir[i] * q_err[i] <= abs(q_ref[i]) * factor) {
                riseTime[i] = k;
#ifdef DEBUG
                cerr << "risen " << i << endl;
#endif
            }
        }
        if (riseTime[i] != -1.0) {
            if (abs(q_err[i]) > abs(q_ref[i]) * factor && q_ref[i] != 0.0) {
                overshoot[i] += 5e3 * (q_err[i] * q_err[i]);
            } else if ((abs(q_dif[i]) <= abs(q_ref[i]) * factor / 238.0 ||
                        q_ref[i] == 0.0) &&
                       settled[i] == -1.0) {
                settled[i] = k;
#ifdef DEBUG
                cerr << "settled " << i << endl;
#else
                continueSimulation = false;
                for (size_t i = 0; i < 4; ++i)
                    if (settled[i] == -1)
                        continueSimulation = true;
#endif
            }
        }
    }
    q_prev = q;
    return continueSimulation;
}

double RealTimeCostCalculator::getCost() const {
    double cost = sum(riseTime);
    for (size_t i = 0; i < 4; ++i) {
        if (settled[i] == -1.0)
            cost += 1e10 * abs(q_prev[i] - q_ref[i]);
        else
            cost += overshoot[i];
    }
    return cost;
}

#ifdef DEBUG
void RealTimeCostCalculator::plot() const {
    using namespace std;
    plotVectors(k_v, q_v, {0, 4}, {"q0", "q1", "q2", "q3"},
                {"c.-", "r.-", "g.-", "b.-"});
    plt::plot(vector<double>{k_v[0], k_v.back()},
              vector<double>{q_ref[0], q_ref[0]}, "c--");
    plt::plot(vector<double>{k_v[0], k_v.back()},
              vector<double>{q_ref[1], q_ref[1]}, "r--");
    plt::plot(vector<double>{k_v[0], k_v.back()},
              vector<double>{q_ref[2], q_ref[2]}, "g--");
    plt::plot(vector<double>{k_v[0], k_v.back()},
              vector<double>{q_ref[3], q_ref[3]}, "b--");
    plt::axvline(riseTime[0], "--", "c");
    plt::axvline(riseTime[1], "--", "r");
    plt::axvline(riseTime[2], "--", "g");
    plt::axvline(riseTime[3], "--", "b");
    plt::axvline(settled[0], "--", "c");
    plt::axvline(settled[1], "--", "r");
    plt::axvline(settled[2], "--", "g");
    plt::axvline(settled[3], "--", "b");

    cerr << ANSIColors::whiteb << "Settled   = " << transpose(settled)  //
         << "Rise time = " << transpose(riseTime)                       //
         << "Overshoot = " << transpose(overshoot)                      //
         << ANSIColors::reset << endl;
}
#endif

double getRiseTimeCost(Drone::FixedClampAttitudeController &ctrl,
                       ContinuousModel<Nx_att, Nu_att, Ny_att> &model,
                       Quaternion q_ref, double factor, ColVector<Nx_att> x0) {
    DroneAttitudeOutput y_ref;
    y_ref.setOrientation(q_ref);
    AdaptiveODEOptions opt                           = Config::Tuner::odeopt;
    ConstantTimeFunctionT<ColVector<Ny_att>> y_ref_f = {y_ref};

    RealTimeCostCalculator costcalc = {model, q_ref, factor, {1} /* TODO */};

    ODEResultCode resultCode =
        model.simulateRealTime(ctrl, y_ref_f, x0, opt, costcalc);
    if (resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED)
        return infinity;

#ifdef DEBUG
    costcalc.plot();
    plt::show();
#endif

    return costcalc.getCost();
}

double getCost(Drone::FixedClampAttitudeController &ctrl,
               ContinuousModel<Nx_att, Nu_att, Ny_att> &model) {
    constexpr double factor = 0.01;  // 1% criterium
    double totalCost        = 0;
    {
        static Quaternion ref = eul2quat({M_PI / 16, M_PI / 16, M_PI / 16});
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = eul2quat({0, M_PI / 16, M_PI / 16});
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = eul2quat({M_PI / 16, 0, 0});
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = eul2quat({0, M_PI / 16, 0});
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = eul2quat({0, 0, M_PI / 16});
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    return totalCost;
}
