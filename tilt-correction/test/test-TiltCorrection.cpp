#include <Degrees.hpp>
#include <AlmostEqual.hpp>
#include <TiltCorrection.hpp>
#include <gtest/gtest.h>

// See eagle-control-slides.pdf: 182: Example (i)
TEST(TiltCorrection, simple) {
    double z_             = 1.101718;
    double theta          = 15_deg;
    double phi            = 20_deg;
    double psi            = 30_deg;
    Quaternion q          = eul2quat({psi, theta, phi});
    ColVector<2> position = {{42, 45}};
    ColVector<3> result   = getCorrectedPosition(position, z_, q);
    ColVector<3> expected = {{
        42 - 0.420455702855121,
        45 + 0.192352205961815,
        1.000000091161111,
    }};
    EXPECT_TRUE(isAlmostEqual(result, expected, 1e-15));
}