#include <AlmostEqual.hpp>
#include <HouseholderQR.hpp>
#include <gtest/gtest.h>

using Matrices::T;
using std::cout;
using std::endl;

double operator""_hexf(unsigned long long v) {
    static_assert(sizeof(v) == sizeof(double), "Error");
    return reinterpret_cast<double &>(v);
}

TEST(QR, QR) {
    Matrix<double, 3, 2> A            = {{
        {11, 12},
        {21, 22},
        {31, 32},
    }};
    QR<double, 3, 2> qtr              = householderQR(A);
    Matrix<double, 3, 2> QTA          = qtr.applyTranspose(A);
    Matrix<double, 3, 2> U_expected   = {{
        {1.132195224629404, 0},
        {0.475278341931121, -1.074832100035896},
        {0.701601361898321, -0.919095183717348},
    }};
    Matrix<double, 3, 2> R_expected   = {{
        {-39.025632602175733, -40.639956209488311},
        {0, 0.627661764705549},
        {0, 0},
    }};
    Matrix<double, 3, 2> QTA_expected = {{
        {0xc0438347edda072d_hexf, 0xc04451ea15c74feb_hexf},
        {0x0000000000000000_hexf, 0x3fe415ce200b84d0_hexf},
        {0xbce0000000000000_hexf, 0xbce0000000000000_hexf},
    }};

    ASSERT_TRUE(isAlmostEqual(qtr.R, R_expected, 1e-12));
    ASSERT_TRUE(isAlmostEqual(qtr.U, U_expected, 1e-12));
    ASSERT_TRUE(isAlmostEqual(QTA, QTA_expected, 1e-12));
}