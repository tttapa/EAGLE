#include "Model.hpp"
#include "Params.hpp"
#include <DormandPrince.hpp>
#include <Matrix.hpp>
#include <iostream>

using namespace std;

class NonLinearFullModel : public Model<double, 10, 4> {
  public:
    NonLinearFullModel(Params p) : p(p) {}
    VecX_t operator()(const VecX_t &x, const VecU_t &u) override { return {}; }
    Params p;
};

int main(int argc, char const *argv[]) {
    Params p = {};

    return 0;
}
