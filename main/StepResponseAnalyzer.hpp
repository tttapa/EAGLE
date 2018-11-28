#pragma once

#include <Matrix/Matrix.hpp>
#include <Matrix/MatrixHelpers.hpp>

template <size_t N>
class StepResponseAnalyzer {
    constexpr static double infinity = std::numeric_limits<double>::infinity();

  public:
    StepResponseAnalyzer() = delete;
    constexpr StepResponseAnalyzer(const ColVector<N> &x_ref, double factor,
                                   const ColVector<N> &x_0)
        : x_ref{x_ref},                   //
          x_adif{abs(x_ref - x_0)},       //
          x_thr{factor * x_adif},         //
          dir{getDirection(x_ref, x_0)},  //
          x_prev{x_0} {}

    virtual bool operator()(double t, const ColVector<N> &x) {
        return calculate(t, x);
    }

    struct Result {
        /**
         * The last value that was fed to the analyzer
         */
        ColVector<N> finalvalue;
        /** 
         * The last error value
         */
        ColVector<N> finalerror;
        /** 
         * The size of the step: `abs(x_ref - x_0)`
         */
        ColVector<N> absdelta;
        /** 
         * The time point where the curve first entered the settling interval
         */
        ColVector<N> risetime;
        /** 
         * The extremum of the error after crossing the reference
         */
        ColVector<N> overshoot;
        /** 
         * The time point where the curve settled within the settling interval
         */
        ColVector<N> settletime;
    };

    Result getResult() const {
        Result result = {
            .finalvalue = x_prev,
            .finalerror = x_ref - x_prev,
            .absdelta   = x_adif,
            .risetime   = risetime,
            .overshoot  = overshoot,
            .settletime = settletime,
        };
        for (size_t i = 0; i < N; ++i)
            if (!crossed[i])
                result.settletime[i] = result.risetime[i];
        return result;
    }

  protected:
    const ColVector<N> x_ref;
    const ColVector<N> x_adif;  // dif = abs(x_ref - x_0)
    const ColVector<N> x_thr;   // threshold = factor * x_adif

    // Contains +1.0 if the reference is greater than or equal to the initial
    // value, and -1.0 if the reference is less than the initial value
    const ColVector<4> dir;

    ColVector<N> x_prev;
    ColVector<N> risetime   = -ones<N, 1>();
    ColVector<N> overshoot  = zeros<N, 1>();
    ColVector<N> settletime = -ones<N, 1>();

    // The time point where the curve last crossed the settling interval
    ColVector<4> lastthrescross = -ones<4, 1>();
    // Whether the next crossing of the settling interval will be rising (true)
    // or falling (false)
    TColVector<bool, 4> nextthrescrossrising = {};
    // Whether the curve is rising (true) or falling (false)
    TColVector<bool, 4> rising = {};
    // The last relative maximum error (extrema of overshoot ringing)
    ColVector<4> maxerr = filledMatrix<N, 1>(infinity);
    // Whether the curve has crossed the reference or not
    TColVector<bool, 4> crossed = {};

    static constexpr ColVector<N> getDirection(const ColVector<N> &x_ref,
                                               const ColVector<N> &x_0) {
        ColVector<N> x_dif = x_ref - x_0;
        return map(x_dif, [](double d) { return d >= 0 ? 1.0 : -1.0; });
    }

    bool calculate(double t, const ColVector<N> &x);
};

#include <Plot.hpp>
#include <string>
#include <vector>

template <size_t N>
class StepResponseAnalyzerPlotter : public StepResponseAnalyzer<N> {
  public:
    constexpr StepResponseAnalyzerPlotter(const ColVector<N> &x_ref,
                                          double factor,
                                          const ColVector<N> &x_0,
                                          bool continueToEnd = true)
        : StepResponseAnalyzer<N>{x_ref, factor, x_0},  //
          continueToEnd{continueToEnd} {}

    void plot(IndexRange idx                          = {0, N},
              const std::vector<std::string> &legends = {},
              const std::vector<std::string> &colors = {"r", "g", "b", "m", "c",
                                                        "y"},
              const std::string &title               = "") const;

    bool operator()(double t, const ColVector<N> &x) override {
        t_v.push_back(t);
        x_v.push_back(x);
        return this->calculate(t, x) | continueToEnd;
    }

  private:
    bool continueToEnd;
    std::vector<double> t_v       = {};
    std::vector<ColVector<N>> x_v = {};
};

#include "StepResponseAnalyzer.ipp"