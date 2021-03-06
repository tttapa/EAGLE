#include <PlotStepResponse.hpp>

#include <Drone.hpp>
#include <StepResponseAnalyzerPlotter.hpp>
#include <iostream>

void plotStepResponseAttitude(const Drone &drone,
                              const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                              const Matrix<Nu_att, Nu_att> &R,
                              double steperrorfactor, const Quaternion &q_ref,
                              const AdaptiveODEOptions &opt,
                              pybind11::object axes, const std::string &title,
                              const std::string &legendSuffix, int colorset) {

    using std::cout;
    using std::endl;

    auto attctrl  = drone.getAttitudeController(Q, R);
    auto attmodel = drone.getAttitudeModel();

    DroneAttitudeOutput y_ref;
    y_ref.setOrientation(q_ref);
    ConstantTimeFunctionT<ColVector<Ny_att>> y_ref_f = {y_ref};

    const DroneState x0             = drone.getStableState();
    const DroneAttitudeState attx0  = x0.getAttitude();
    const DroneAttitudeOutput atty0 = attmodel.getOutput(attx0, {0});
    const Quaternion q0             = atty0.getOrientation();

    StepResponseAnalyzerPlotter<4> stepAnalyzerPlt = {q_ref, steperrorfactor,
                                                      q0, false};
    auto f = [&](double t, const ColVector<Nx_att> &x,
                 const ColVector<Nu_att> &u) {
        DroneAttitudeOutput y = attmodel.getOutput(x, u);
        return stepAnalyzerPlt(t, y.getOrientation());
    };

    ODEResultCode resultCode =
        attmodel.simulateRealTime(attctrl, y_ref_f, attx0, opt, f);
    resultCode.verbose();

    Quaternion risetimes   = stepAnalyzerPlt.getResult().risetime;
    Quaternion overshoots  = stepAnalyzerPlt.getResult().overshoot;
    Quaternion settletimes = stepAnalyzerPlt.getResult().settletime;

    cout << ANSIColors::whiteb << endl
         << "Rise time:   \t" << asrowvector(risetimes) << endl
         << "Overshoot:   \t" << asrowvector(overshoots) << endl
         << "Settle time: \t" << asrowvector(settletimes) << endl
         << ANSIColors::reset;

    stepAnalyzerPlt.plot(axes, {1, 4}, title,
                         {"$q_1$" + legendSuffix, "$q_2$" + legendSuffix,
                          "$q_3$" + legendSuffix},
                         colorset);
}