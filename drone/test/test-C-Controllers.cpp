#include <C-code-wrappers/CLQRController.hpp>
#include <Degrees.hpp>
#include <Drone.hpp>
#include <FileLoader.hpp>
#include <gtest/gtest.h>

using namespace std;

class CControllersTest : public ::testing::Test {
  protected:
    const std::filesystem::path paramsAndMatricesPath =
        PARAMS_AND_MATRICES_PATH;

    void SetUp() override {
        K_att = loadMatrix<3, 9>(paramsAndMatricesPath / "attitude/lqr/K");
        K_alt = loadMatrix<1, 4>(paramsAndMatricesPath / "altitude/lqi/K");
        L_att = loadMatrix<9, 6>(paramsAndMatricesPath / "attitude/kal/L");
        L_alt = loadMatrix<3, 1>(paramsAndMatricesPath / "altitude/kal/L");
        maxIntegral =
            loadDouble(paramsAndMatricesPath / "altitude/lqi/max_integral");
        drone = {paramsAndMatricesPath};
    }

    Matrix<3, 9> K_att;
    Matrix<1, 4> K_alt;
    Matrix<9, 6> L_att;
    Matrix<3, 1> L_alt;
    double maxIntegral;
    Drone drone;

    int config = 1;
};

#pragma region Controllers......................................................

TEST_F(CControllersTest, attitudeController) {
    Attitude::LQRController controller   = drone.getAttitudeController(K_att);
    Attitude::CLQRController ccontroller = {drone.p.Ts_att, config};

    DroneAttitudeState x;
    x.setAngularVelocity({-2, -1.5, -3});
    x.setMotorSpeed({0.2, 0.21, 0.22});
    x.setOrientation(eul2quat({2_deg, 3_deg, 4_deg}));

    DroneAttitudeOutput r;
    r.setOrientation(eul2quat({50_deg, 40_deg, 30_deg}));

    auto resultCpp = controller(x, r);
    auto resultC   = ccontroller(x, r);

    cout << "  - C++  :\t" << asrowvector(resultCpp, " ", 16) << endl;
    cout << "  - C    :\t" << asrowvector(resultC, " ", 16) << endl;
    cout << "  - diff :\t" << asrowvector(resultCpp - resultC, " ", 16) << endl;

    ASSERT_TRUE(isAlmostEqual(resultCpp, resultC, 1e-11));
}

TEST_F(CControllersTest, altitudeController) {
    Altitude::LQRController controller =
        drone.getAltitudeController(K_alt, maxIntegral);
    Altitude::CLQRController ccontroller = {drone.p.Ts_alt, config, true};

    ColVector<3> x = {{0.5, 2, 1}};
    ColVector<1> r = {{4}};

    auto resultCpp = controller(x, r);
    auto resultC   = ccontroller(x, r);

    cout << "Control:" << endl;
    cout << "  - C++  :\t" << std::setprecision(16) << double(resultCpp)
         << endl;
    cout << "  - C    :\t" << std::setprecision(16) << double(resultC) << endl;
    cout << "  - diff :\t" << std::setprecision(16)
         << double(resultCpp - resultC) << endl;
    cout << "Integral:" << endl;
    cout << "  - C++  :\t" << std::setprecision(16)
         << double(controller.getIntegral()) << endl;
    cout << "  - C    :\t" << std::setprecision(16)
         << double(ccontroller.getIntegral()) << endl;
    cout << "  - diff :\t" << std::setprecision(16)
         << double(controller.getIntegral() - ccontroller.getIntegral())
         << endl;

    ASSERT_TRUE(isAlmostEqual(resultCpp, resultC, 1e-11));
    ASSERT_TRUE(isAlmostEqual(controller.getIntegral(),
                              ccontroller.getIntegral(), 1e-11));
}

TEST_F(CControllersTest, altitudeControllerMaxIntegralPositive) {
    Altitude::LQRController controller =
        drone.getAltitudeController(K_alt, maxIntegral);
    Altitude::CLQRController ccontroller = {drone.p.Ts_alt, config, true};

    ColVector<3> x = {{0.5, 2, 1}};
    ColVector<1> r = {{13 / drone.p.Ts_alt}};

    auto resultCpp = controller(x, r);
    auto resultC   = ccontroller(x, r);

    cout << "Control:" << endl;
    cout << "  - C++  :\t" << std::setprecision(16) << double(resultCpp)
         << endl;
    cout << "  - C    :\t" << std::setprecision(16) << double(resultC) << endl;
    cout << "  - diff :\t" << std::setprecision(16)
         << double(resultCpp - resultC) << endl;
    cout << "Integral:" << endl;
    cout << "  - C++  :\t" << std::setprecision(16)
         << double(controller.getIntegral()) << endl;
    cout << "  - C    :\t" << std::setprecision(16)
         << double(ccontroller.getIntegral()) << endl;
    cout << "  - diff :\t" << std::setprecision(16)
         << double(controller.getIntegral() - ccontroller.getIntegral())
         << endl;

    ASSERT_TRUE(isAlmostEqual(resultCpp, resultC, 1e-11));
    ASSERT_TRUE(isAlmostEqual(controller.getIntegral(),
                              ccontroller.getIntegral(), 1e-11));
}

TEST_F(CControllersTest, altitudeControllerMaxIntegralNegative) {
    Altitude::LQRController controller =
        drone.getAltitudeController(K_alt, maxIntegral);
    Altitude::CLQRController ccontroller = {drone.p.Ts_alt, config, true};

    ColVector<3> x = {{0.5, 2, 1}};
    ColVector<1> r = {{-13 / drone.p.Ts_alt}};

    auto resultCpp = controller(x, r);
    auto resultC   = ccontroller(x, r);

    cout << "Control:" << endl;
    cout << "  - C++  :\t" << std::setprecision(16) << double(resultCpp)
         << endl;
    cout << "  - C    :\t" << std::setprecision(16) << double(resultC) << endl;
    cout << "  - diff :\t" << std::setprecision(16)
         << double(resultCpp - resultC) << endl;
    cout << "Integral:" << endl;
    cout << "  - C++  :\t" << std::setprecision(16)
         << double(controller.getIntegral()) << endl;
    cout << "  - C    :\t" << std::setprecision(16)
         << double(ccontroller.getIntegral()) << endl;
    cout << "  - diff :\t" << std::setprecision(16)
         << double(controller.getIntegral() - ccontroller.getIntegral())
         << endl;

    ASSERT_TRUE(isAlmostEqual(resultCpp, resultC, 1e-11));
    ASSERT_TRUE(isAlmostEqual(controller.getIntegral(),
                              ccontroller.getIntegral(), 1e-11));
}

#pragma endregion

#pragma region Observers........................................................

TEST_F(CControllersTest, attitudeObserver) {
    Attitude::KalmanObserver observer   = drone.getAttitudeObserver(L_att);
    Attitude::CKalmanObserver cobserver = {drone.p.Ts_att};

    DroneAttitudeState x;
    x.setAngularVelocity({-2, -1.5, -3});
    x.setMotorSpeed({0.2, 0.21, 0.22});
    x.setOrientation(eul2quat({2_deg, 3_deg, 4_deg}));

    DroneAttitudeOutput y;
    y.setAngularVelocity({-2.4, -2.0, -2.8});
    y.setOrientation(eul2quat({2.5_deg, 2.6_deg, 4.1_deg}));

    DroneAttitudeControl u;
    u.setAttitudeControl({0.5, -0.5, 0.7});

    auto resultCpp = observer.getStateChange(x, y, u);
    auto resultC   = cobserver.getStateChange(x, y, u);

    cout << "  - C++  :\t" << asrowvector(resultCpp, " ", 16) << endl;
    cout << "  - C    :\t" << asrowvector(resultC, " ", 16) << endl;
    cout << "  - diff :\t" << asrowvector(resultCpp - resultC, " ", 16) << endl;

    ASSERT_TRUE(isAlmostEqual(resultCpp, resultC, 1e-11));
}

TEST_F(CControllersTest, altitudeObserver) {
    Altitude::KalmanObserver observer   = drone.getAltitudeObserver(L_alt);
    Altitude::CKalmanObserver cobserver = {drone.p.Ts_att};

    ColVector<3> x = {{-2, -1.5, -3}};

    ColVector<1> y = {-1.3};

    ColVector<1> u = {0.5};

    auto resultCpp = observer.getStateChange(x, y, u);
    auto resultC   = cobserver.getStateChange(x, y, u);

    cout << "  - C++  :\t" << asrowvector(resultCpp, " ", 16) << endl;
    cout << "  - C    :\t" << asrowvector(resultC, " ", 16) << endl;
    cout << "  - diff :\t" << asrowvector(resultCpp - resultC, " ", 16) << endl;

    ASSERT_TRUE(isAlmostEqual(resultCpp, resultC, 1e-11));
}

#pragma endregion