#include <Quaternions/ReducedQuaternion.hpp>
#include <Util/AlmostEqual.hpp>
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

TEST(Quaternion, quatrotate) {
    Quaternion q          = {{
        {1},
        {2},
        {5},
        {7},
    }};
    q                     = q / norm(q);
    Matrix<3, 2> v        = {{
        {1, 2},
        {3, 4},
        {5, 6},
    }};
    Matrix<3, 2> result   = quatrotate(q, v);
    Matrix<3, 2> expected = {{
        {1.556962025316456, 1.341772151898735},
        {3.734177215189874, 4.405063291139241},
        {4.316455696202532, 5.898734177215190},
    }};
    std::cout << result << std::endl;
    ASSERT_TRUE(isAlmostEqual(result, expected, 1e-12));
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
    EulerAngles e         = {0.3, -0.7, 0.9};
    Quaternion q          = eul2quat(e);
    EulerAngles result    = quat2eul(q);
    EulerAngles expected  = e;
    Quaternion q_expected = {
        0.814068885161671,
        0.450147392727527,
        -0.244234643193019,
        0.273877005417802,
    };
    ASSERT_TRUE(isAlmostEqual(q, q_expected, 1e-15));
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