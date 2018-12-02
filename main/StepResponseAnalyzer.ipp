#include "StepResponseAnalyzer.hpp"
#include <Plot.hpp>

template <size_t N>
bool StepResponseAnalyzer<N>::calculate(double t, const ColVector<N> &x) {
    using namespace std;

    if (!isfinite(x) || !isfinite(t)) {
        fill(overshoot.begin(), overshoot.end(), infinity);
        return false;
    }

    // Should the simulation continue after this time step?
    bool continueSimulation = true;

    // The error is the difference between the reference and the current value
    auto x_err = x_ref - x;

    for (size_t i = 0; i < N; ++i) {
        if (settletime[i] >= 0)
            continue;  // settled already, ignore

        // find extrema and update direction
        if (dir[i] * x[i] < dir[i] * x_prev[i]) {
            dir[i]             = -dir[i];
            double newextremum = dir[i] * (x_ref[i] - x_prev[i]);
            if (newextremum > lastextremum[i] && extremumcount[i] > 1UL) {
                settletime[i] = infinity;  // unstable
#ifndef DEBUG
                continueSimulation = false;
                for (size_t i = 0; i < N; ++i)
                    if (settletime[i] < 0.0)
                        continueSimulation = true;
#else
                cerr << "unstable " << i;
#endif
            }
            if (overshoot[i] == 0.0 && risetime[i] > 0.0)
                overshoot[i] = newextremum;
            lastextremum[i] = newextremum;
            ++extremumcount[i];
        }
        bool previnside = inside[i];
        // signal enters error band
        if (!inside[i] && x_err[i] <= x_thr[i] && -x_err[i] <= x_thr[i]) {
            lastenter[i] = t;
            inside[i]    = true;
            if (risetime[i] < 0.0)
                risetime[i] = t;
        }
        // signal exits error band
        if (inside[i] && !(x_err[i] <= x_thr[i] && -x_err[i] <= x_thr[i])) {
            lastexit[i] = t;
            inside[i]   = false;
        }
        // passes through without sample in error band
        if (!previnside && !inside[i] &&
            (x_prev[i] < x_ref[i]) != (x[i] < x_ref[i])) {
            lastenter[i] = lastexit[i] = t;
        }

        // if we have been within the settling band for at least 5 times the
        // rise time,  it probably means it has settled
        //   TODO: is this a reasonable assumption?
        if (t - lastenter[i] >= 5 * risetime[i] && inside[i]) {
            settletime[i] = risetime[i];
            cerr << "settled (×5): i = " << i << ", t = " << t
                 << ", lastenter = " << lastenter[i] << endl;
#ifndef DEBUG
            continueSimulation = false;
            for (size_t i = 0; i < N; ++i)
                if (settletime[i] < 0.0)
                    continueSimulation = true;
#endif
        }

        if ((lastextremum[i] <= x_thr[i] && extremumcount[i] > 1UL) ||
            x_adif[i] == 0) {
            settletime[i] = lastenter[i];
            cerr << "settled: i = " << i << ", t = " << t
                 << ", lastenter = " << lastenter[i] << endl;
#ifndef DEBUG
            continueSimulation = false;
            for (size_t i = 0; i < N; ++i)
                if (settletime[i] < 0.0)
                    continueSimulation = true;
#endif
        }
    }

    x_prev = x;

    return continueSimulation;
}

/*

double RealTimeCostCalculator<N>::getCost(double notRisenCost,
                                       double notSettledCost,
                                       double overshootCost,
                                       double settleTimeCost) const {
    auto settled = this->settled;
    double cost  = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (overshoot[i] == infinity)
            return infinity;
        if (x_dif[i] == 0.0)
            continue;
        if (!crossed[i])
            settled[i] = risetime[i];
        if (risetime[i]< 0.0)
            cost += notRisenCost * abs(x_ref[i] - x_prev[i]);
        else if (settled[i]< 0.0)
            cost += notSettledCost *
                    (abs(overshoot[i]) +  // TODO: maybe overshoot is still zero
                     abs(x_ref[i] - x_prev[i]));
        else
            cost += risetime[i]                                     //
                    + overshootCost * abs(overshoot[i] / x_dif[i])  //
                    + settleTimeCost * (settled[i] - risetime[i]);
        assert(settled[i] >= risetime[i] || settled[i]< 0.0);
    }
    return cost;
}
*/

template <size_t N>
void StepResponseAnalyzerPlotter<N>::plot(
    IndexRange idx, const std::vector<std::string> &legends,
    const std::vector<std::string> &colors, const std::string &title) const {

    if (t_v.empty() || x_v.empty())
        return;

    const auto result      = this->getResult();
    const auto &overshoot  = result.overshoot;
    const auto &risetime   = result.risetime;
    const auto &settletime = result.settletime;
    const auto &x_ref      = this->x_ref;
    const auto &x_thr      = this->x_thr;

    std::vector<std::string> formats;
    std::transform(colors.begin(), colors.end(), std::back_inserter(formats),
                   [](const std::string &c) { return c + ".-"; });
    plotVectors(t_v, x_v, idx, legends, formats, title);

    for (size_t k = 0; k < (idx.end - idx.start); ++k) {
        size_t i          = k + idx.start;
        std::string color = k < colors.size() ? colors[k] : "";
        if (x_ref[i] != 0.0) {
            plt::axhline(x_ref[i], "--", color);
            plt::axhline(x_ref[i] - x_thr[i], "-.", color);
            plt::axhline(x_ref[i] + x_thr[i], "-.", color);
            plt::axhline(x_ref[i] + overshoot[i], ":", color);
        }
        if (risetime[i] > 0.0)
            plt::axvline(risetime[i], "--", color);
        if (settletime[i] > 0.0)
            plt::axvline(settletime[i], "-.", color);
    }
    plt::xlim(t_v[0], t_v.back());
}