#include "Cost.hpp"
#include <Plot.hpp>

constexpr double infinity = std::numeric_limits<double>::infinity();

RealTimeCostCalculator::RealTimeCostCalculator(
    ContinuousModel<Nx_att, Nu_att, Ny_att> &model, const Quaternion &q_ref,
    double factor, const Quaternion &q_0)
    : model{model}, q_ref{q_ref}, q_dif{abs(q_ref - q_0)}, q_thr{factor *
                                                                 q_dif},
      q_prev{q_0}, dir{getDirection(q_ref, q_0)}, maxerr{infinity, infinity,
                                                         infinity, infinity} {}

bool RealTimeCostCalculator::operator()(size_t k, const ColVector<Nx_att> &x,
                                        const ColVector<Nu_att> &u) {
    using namespace std;

    if (!isfinite(x) || !isfinite(u)) {
        std::fill(overshoot.begin(), overshoot.end(), infinity);
        return false;
    }

    DroneAttitudeOutput yy = model.getOutput(x, u);
    auto q                 = yy.getOrientation();

    // Should the simulation continue after this time step?
    bool continueSimulation = true;

    // The error is the difference between the reference and the current value
    auto q_err = q_ref - q;

#ifdef DEBUG
    q_v.push_back(q);
    k_v.push_back(k);
#endif

    for (size_t i = 0; i < 4; ++i) {

        // if not risen yet
        if (riseTime[i] == -1.0) {
            // if we are crossing the threshold
            // (sign of threshold depends on direction)
            if (dir[i] * q_err[i] <= q_thr[i]) {
                riseTime[i] = k;
                // first crossing of the settling interval
                lastthrescross[i] = k;
                // if this was a rising crossing, the next crossing will be
                // falling, and vice versa
                bool rising             = dir[i] > 0;
                nextthrescrossrising[i] = !rising;
            }
        }

        // if we have not actually crossed the reference yet,
        // and we are currently crossing it
        // (i.e. error crosses 0 in the right direction)
        if (!crossed[i] && dir[i] * q_err[i] <= 0) {
            crossed[i] = true;
            // remember whether we're rising or falling
            rising[i] = dir[i] > 0;
        }

        // if we haven't crossed the reference yet after 2 times the rise time
        // it probably means it has settled without overshoot
        //   TODO: is this a reasonable assumption?
        if (!crossed[i] && k == 2 * riseTime[i] && settled[i] == -1.0) {
            settled[i] = riseTime[i];
#ifndef DEBUG
            continueSimulation = false;
            for (size_t i = 0; i < 4; ++i)
                if (settled[i] == -1.0)
                    continueSimulation = true;
#endif
        }

        // if we have crossed the reference, but haven't settled yet (overshoot)
        if (crossed[i] && settled[i] == -1.0) {
            // find relative extrema of error
            double newmaxerr = maxerr[i];
            if (rising[i]) {             // curve was previously rising
                if (q[i] < q_prev[i]) {  // was rising, is now falling again
                    rising[i] = false;
                    // previous point was relative maximum
                    newmaxerr = q_prev[i] - q_ref[i];
                }
            } else {                     // curve was previously falling
                if (q[i] > q_prev[i]) {  // was falling, is now rising again
                    rising[i] = true;
                    // previous point was relative minimum
                    newmaxerr = q_ref[i] - q_prev[i];
                }
            }

            double newlastthrescross = lastthrescross[i];
            // find crossings of settling interval
            if (rising[i]) {  // curve is currently rising
                // if we are looking for a rising crossing of the settling
                // interval, and we are currently crossing the lower bound
                // of the interval
                if (nextthrescrossrising[i] == true && q_err[i] <= q_thr[i]) {
                    // remember this crossing
                    newlastthrescross = k;
                    // next crossing will be falling
                    nextthrescrossrising[i] = false;
                }
            } else {  // curve is currently falling
                // if we are looking for a falling crossing of the settling
                // interval, and we are currently crossing the upper bound
                // of the interval
                if (nextthrescrossrising[i] == false && -q_err[i] <= q_thr[i]) {
                    // remember this crossing
                    newlastthrescross = k;
                    // next crossing will be rising
                    nextthrescrossrising[i] = true;
                }
            }

            // if extremum is greater than previous extremum,
            // the oscillations are increasing in amplitude,
            // so the system is unstable
            if (newmaxerr > maxerr[i]) {  // unstable
                overshoot[i] = infinity;
                settled[i]   = infinity;
#ifndef DEBUG
                continueSimulation = false;
#endif
            } else {  // stable
                if (maxerr[i] == infinity)
                    overshoot[i] = dir[i] * newmaxerr;
                maxerr[i] = newmaxerr;
            }

            // if the new extremum lies within the settling interval,
            // the system has settled, and the settling time was the time
            // of the previous crossing of the settling interval
            if (maxerr[i] <= q_thr[i] || q_thr[i] == 0.0) {
                settled[i] = lastthrescross[i];
                if (q_thr[i] == 0.0)
                    overshoot[i] = 0.0;
#ifndef DEBUG
                continueSimulation = false;
                for (size_t i = 0; i < 4; ++i)
                    if (settled[i] == -1.0)
                        continueSimulation = true;
#endif
            } else {
                // only a real crossing if the extremum is outside of the
                // settling interval, otherwise, it doesn't really cross the
                // bounds of the interval, it just stays inside of it
                lastthrescross[i] = newlastthrescross;
            }
        }
    }

    q_prev = q;

    return continueSimulation;
}

double RealTimeCostCalculator::getCost(double notRisenCost,
                                       double notSettledCost,
                                       double overshootCost,
                                       double settleTimeCost) const {
    auto settled = this->settled;
    double cost  = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (q_dif[i] == 0.0)
            continue;
        if (!crossed[i])
            settled[i] = riseTime[i];
        if (riseTime[i] == -1.0)
            cost += notRisenCost * abs(q_ref[i] - q_prev[i]);
        else if (settled[i] == -1.0)
            cost += notSettledCost *
                    (abs(overshoot[i]) +  // TODO: maybe overshoot is still zero
                     abs(q_ref[i] - q_prev[i]));
        else
            cost += riseTime[i]                                     //
                    + overshootCost * abs(overshoot[i] / q_dif[i])  //
                    + settleTimeCost * (settled[i] - riseTime[i]);
        assert(settled[i] >= riseTime[i] || settled[i] == -1.0);
    }
    return cost;
}

#ifdef DEBUG
void RealTimeCostCalculator::plot() const {
    using namespace std;

    cerr << ANSIColors::whiteb << "Settled   = " << transpose(settled)  //
         << "Rise time = " << transpose(riseTime)                       //
         << "Overshoot = " << transpose(overshoot)                      //
         << "q         = " << transpose(q_prev)                         //
         << ANSIColors::reset << endl;

    if (k_v.empty() || q_v.empty())
        return;

    plt::figure_size(1280, 720);
#ifdef PLOT_ALL_QUATERNION_STATES
    plotVectors(k_v, q_v, {0, 4}, {"q0", "q1", "q2", "q3"},
                {"c.-", "r.-", "g.-", "b.-"});
#else
    plotVectors(k_v, q_v, {1, 4}, {"q1", "q2", "q3"},  //
                {"r.-", "g.-", "b.-"});
#endif
#ifdef PLOT_ALL_QUATERNION_STATES
    if (q_ref[0] != 0) {
        plt::axhline(q_ref[0], "--", "c");
        plt::axhline(q_ref[0] - q_thr[0], "-.", "c");
        plt::axhline(q_ref[0] + q_thr[0], "-.", "c");
        plt::axhline(q_ref[0] + overshoot[0], ":", "c");
    }
#endif
    if (q_ref[1] != 0) {
        plt::axhline(q_ref[1], "--", "r");
        plt::axhline(q_ref[1] - q_thr[1], "-.", "r");
        plt::axhline(q_ref[1] + q_thr[1], "-.", "r");
        plt::axhline(q_ref[1] + overshoot[1], ":", "r");
    }
    if (q_ref[2] != 0) {
        plt::axhline(q_ref[2], "--", "g");
        plt::axhline(q_ref[2] - q_thr[2], "-.", "g");
        plt::axhline(q_ref[2] + q_thr[2], "-.", "g");
        plt::axhline(q_ref[2] + overshoot[2], ":", "g");
    }
    if (q_ref[3] != 0) {
        plt::axhline(q_ref[3], "--", "b");
        plt::axhline(q_ref[3] - q_thr[3], "-.", "b");
        plt::axhline(q_ref[3] + q_thr[3], "-.", "b");
        plt::axhline(q_ref[3] + overshoot[3], ":", "b");
    }
#ifdef PLOT_ALL_QUATERNION_STATES
    if (riseTime[0] > 0)
        plt::axvline(riseTime[0], "--", "c");
#endif
    if (riseTime[1] > 0)
        plt::axvline(riseTime[1], "--", "r");
    if (riseTime[2] > 0)
        plt::axvline(riseTime[2], "--", "g");
    if (riseTime[3] > 0)
        plt::axvline(riseTime[3], "--", "b");
#ifdef PLOT_ALL_QUATERNION_STATES
    if (settled[0] > 0)
        plt::axvline(settled[0], "-.", "c");
#endif
    if (settled[1] > 0)
        plt::axvline(settled[1], "-.", "r");
    if (settled[2] > 0)
        plt::axvline(settled[2], "-.", "g");
    if (settled[3] > 0)
        plt::axvline(settled[3], "-.", "b");

    plt::xlim(0.0, k_v.back());
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

#ifdef DEBUG
    costcalc.plot();
    plt::tight_layout();
    plt::show();
#endif

    if (resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED)
        return infinity;

    return costcalc.getCost();
}

double getCost(Drone::FixedClampAttitudeController &ctrl,
               ContinuousModel<Nx_att, Nu_att, Ny_att> &model) {
    constexpr double factor = 0.01;  // 1% criterium
    constexpr Quaternion qx = eul2quat({0, 0, M_PI / 8});
    constexpr Quaternion qy = eul2quat({0, M_PI / 8, 0});
    constexpr Quaternion qz = eul2quat({M_PI / 8, 0, 0});
    double totalCost        = 0;
    {
        constexpr Quaternion ref = quatmultiply(qx, quatmultiply(qy, qz));
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = quatmultiply(qx, qy);
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = qx;
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = qy;
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    {
        static Quaternion ref = qz;
        totalCost += getRiseTimeCost(ctrl, model, ref, factor);
    }
    return totalCost;
}
