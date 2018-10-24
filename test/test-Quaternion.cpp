#include <AlmostEqual.hpp>
#include <Quaternion.hpp>
#include <gtest/gtest.h>

TEST(Quaternion, quatmultiply) {
    Quaternion q        = {{
        {1},
        {2},
        {5},
        {7},
    }};
    Quaternion r        = {{
        {11},
        {13},
        {17},
        {19},
    }};
    Quaternion result   = quatmultiply(q, r);
    Quaternion expected = {{
        {-233},
        {11},
        {125},
        {65},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Quaternion, conjugate) {
    Quaternion q        = {{
        {1},
        {2},
        {5},
        {7},
    }};
    Quaternion result   = quatconjugate(q);
    Quaternion expected = {{
        {1},
        {-2},
        {-5},
        {-7},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Quaternion, eul2quat) {
    EulerAngles e       = {1, 2, 3};
    Quaternion result   = eul2quat(e);
    Quaternion expected = {{
        {0.435952844073566},
        {0.444435113443001},
        {0.310622451065704},
        {-0.718287018243412},
    }};
    ASSERT_TRUE(isAlmostEqual(result, expected, 1e-15));
}

TEST(Quaternion, quat2eul) {
    EulerAngles e        = {0.3, 0.7, 0.9};
    Quaternion q         = eul2quat(e);
    EulerAngles result   = quat2eul(q);
    EulerAngles expected = e;
    ASSERT_TRUE(isAlmostEqual(result, expected, 1e-15));
}

// -------------------------------------------------------------------------- //

TEST(ReducedQuaternion, convertAndBack) {
    EulerAngles e       = {0.3, 0.7, 0.9};
    Quaternion q        = eul2quat(e);
    ReducedQuaternion r = quat2red(q);
    Quaternion result   = red2quat(r);
    Quaternion expected = q;
    ASSERT_TRUE(isAlmostEqual(result, expected, 1e-15));
}