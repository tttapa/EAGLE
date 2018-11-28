#include "Cost.hpp"

constexpr double infinity = std::numeric_limits<double>::infinity();

template <size_t N>
double getTimeStepCost(const typename StepResponseAnalyzer<N>::Result &result,
                       double notRisenCost, double notSettledCost,
                       double overshootCost, double settleTimeCost) {
    const auto &finalerror = result.finalerror;
    const auto &settletime = result.settletime;
    const auto &absdelta   = result.absdelta;
    const auto &overshoot  = result.overshoot;
    const auto &risetime   = result.risetime;
    double cost            = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (overshoot[i] == infinity)
            return infinity;
        if (absdelta[i] == 0)
            continue;
        if (risetime[i] == -1.0)
            cost += notRisenCost * abs(finalerror[i]);
        else if (settletime[i] == -1.0)
            cost += notSettledCost *
                    (abs(overshoot[i]) +  // TODO: maybe overshoot is still zero
                     abs(finalerror[i]));
        else
            cost += risetime[i]                                        //
                    + overshootCost * abs(overshoot[i] / absdelta[i])  //
                    + settleTimeCost * (settletime[i] - risetime[i]);
        assert(settletime[i] >= risetime[i] || settletime[i] == -1.0);
    }
    return cost;
}

double getRiseTimeCost(Drone::FixedClampAttitudeController &attctrl,
                       Drone::AttitudeModel &attmodel, Quaternion q_ref,
                       double errorfactor, const DroneAttitudeState &attx0,
                       const AdaptiveODEOptions &opt, const CostWeights &cost) {
    DroneAttitudeOutput y_ref;
    y_ref.setOrientation(q_ref);
    const DroneAttitudeOutput atty0 = attmodel.getOutput(attx0, {0});
    const Quaternion q0             = atty0.getOrientation();
    ConstantTimeFunctionT<ColVector<Ny_att>> y_ref_f = {y_ref};

#ifdef DEBUG
    StepResponseAnalyzerPlotter<4> analyzer = {q_ref, errorfactor, q0, false};
#else
    StepResponseAnalyzer<4> analyzer = {q_ref, errorfactor, q0};
#endif

    auto f = [&](double t, const ColVector<Nx_att> &x,
                 const ColVector<Nu_att> &u) {
        DroneAttitudeOutput y = attmodel.getOutput(x, u);
        return analyzer(t, y.getOrientation());
    };

    ODEResultCode resultCode =
        attmodel.simulateRealTime(attctrl, y_ref_f, attx0, opt, f);
    resultCode.verbose();

#ifdef DEBUG
    analyzer.plot();
    plt::show();
#endif

    if (resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED)
        return infinity;

    auto result = analyzer.getResult();
#ifdef DEBUG
    using std::cerr;
    using std::endl;

    cerr << ANSIColors::whiteb << endl
         << "Rise time:   \t" << asrowvector(result.risetime) << endl
         << "Overshoot:   \t" << asrowvector(result.overshoot) << endl
         << "Settle time: \t" << asrowvector(result.settletime) << endl
         << ANSIColors::reset;
#endif
    return getTimeStepCost<4>(result, cost.notRisen, cost.notSettled,
                              cost.overshoot, cost.settleTime);
}

double getCost(Drone::FixedClampAttitudeController &ctrl,
               Drone::AttitudeModel &model, double errorfactor,
               const DroneAttitudeState &attx0, const AdaptiveODEOptions &opt,
               const CostWeights &cost) {

    double totalCost = 0;
    for (const Quaternion &ref : CostReferences::references)
        totalCost +=
            getRiseTimeCost(ctrl, model, ref, errorfactor, attx0, opt, cost);
    return totalCost;
}
