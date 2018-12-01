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

        // if not risen yet
        if (risetime[i] < 0.0) {
            // if we are crossing the threshold
            // (sign of threshold depends on direction)
            if (dir[i] * x_err[i] <= x_thr[i]) {
                risetime[i] = t;
                // first crossing of the settling interval
                lastthrescross[i] = t;
                // if this was a rising crossing, the next crossing will be
                // falling, and vice versa
                bool rising             = dir[i] > 0;
                nextthrescrossrising[i] = !rising;
            }
        }

        // if we have not actually crossed the reference yet,
        // and we are currently crossing it
        // (i.e. error crosses 0 in the right direction)
        if (!crossed[i] && dir[i] * x_err[i] <= 0) {
            crossed[i] = true;
            // remember whether we're rising or falling
            rising[i] = dir[i] > 0;
        }

        // if we haven't crossed the reference yet after 4 times the rise time
        // it probably means it has settled without overshoot
        //   TODO: is this a reasonable assumption?
        if (!crossed[i] && t >= 4 * risetime[i] && settletime[i] < 0.0) {
            settletime[i] = risetime[i];
#ifndef DEBUG
            continueSimulation = false;
            for (size_t i = 0; i < 4; ++i)
                if (settletime[i] < 0.0)
                    continueSimulation = true;
#endif
        }

        // if we have crossed the reference, but haven't settled yet (overshoot)
        if (crossed[i] && settletime[i] < 0.0) {
            // find relative extrema of error
            double newmaxerr = maxerr[i];
            if (rising[i]) {             // curve was previously rising
                if (x[i] < x_prev[i]) {  // was rising, is now falling again
                    rising[i] = false;
                    // previous point was relative maximum
                    newmaxerr = x_prev[i] - x_ref[i];
                }
            } else {                     // curve was previously falling
                if (x[i] > x_prev[i]) {  // was falling, is now rising again
                    rising[i] = true;
                    // previous point was relative minimum
                    newmaxerr = x_ref[i] - x_prev[i];
                }
            }

            double newlastthrescross = lastthrescross[i];
            // find crossings of settling interval
            if (rising[i]) {  // curve is currently rising
                // if we are looking for a rising crossing of the settling
                // interval, and we are currently crossing the lower bound
                // of the interval
                if (nextthrescrossrising[i] == true && x_err[i] <= x_thr[i]) {
                    // remember this crossing
                    newlastthrescross = t;
                    // next crossing will be falling
                    nextthrescrossrising[i] = false;
                }
            } else {  // curve is currently falling
                // if we are looking for a falling crossing of the settling
                // interval, and we are currently crossing the upper bound
                // of the interval
                if (nextthrescrossrising[i] == false && -x_err[i] <= x_thr[i]) {
                    // remember this crossing
                    newlastthrescross = t;
                    // next crossing will be rising
                    nextthrescrossrising[i] = true;
                }
            }

            bool validnewextremum = false;

            // if extremum is greater than previous extremum,
            // the oscillations are increasing in amplitude,
            // so the system is unstable
            if (newmaxerr > maxerr[i] && !firstextremum[i]) {  // unstable
                overshoot[i]  = infinity;
                settletime[i] = infinity;
#ifndef DEBUG
                continueSimulation = false;
#else
                cerr << "Unstable " << i << " t = " << t << endl;
#endif
            } else if (newmaxerr == maxerr[i]) {  // no new extremum
                ;
            } else {                          // stable, new extremum
                if (maxerr[i] == infinity) {  // this extremum is the first
                    overshoot[i]     = dir[i] * newmaxerr;
                    firstextremum[i] = true;
                } else {  // first extremum doesn't count for settling
                    firstextremum[i] = false;
                    validnewextremum = true;
                }
                maxerr[i] = newmaxerr;
            }

            // if the new extremum lies within the settling interval,
            // the system has settled, and the settling time was the time
            // of the previous crossing of the settling interval
            if (maxerr[i] <= x_thr[i] || x_thr[i] == 0.0) {
                if (validnewextremum || x_thr[i] == 0.0) {
                    settletime[i] = lastthrescross[i];
                    if (x_thr[i] == 0.0)
                        overshoot[i] = 0.0;
#ifndef DEBUG
                    continueSimulation = false;
                    for (size_t i = 0; i < 4; ++i)
                        if (settletime[i] < 0.0)
                            continueSimulation = true;
#endif
                } else {  // first extremum, but still no new extremum
                    // probably stable
                    if (t - lastthrescross[i] >= 4 * risetime[i]) {
                        settletime[i] = lastthrescross[i];
#ifndef DEBUG
                        continueSimulation = false;
                        for (size_t i = 0; i < 4; ++i)
                            if (settletime[i] < 0.0)
                                continueSimulation = true;
#endif
                    }
                }
            } else {
                // only a real crossing if the extremum is outside of the
                // settling interval, otherwise, it doesn't really cross the
                // bounds of the interval, it just stays inside of it
                lastthrescross[i] = newlastthrescross;
                // TODO: ignores first extremum
            }
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