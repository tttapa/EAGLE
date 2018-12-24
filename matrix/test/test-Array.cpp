#include <gtest/gtest.h>

#include <ArrayHelpers.hpp>

TEST(Array, initializeAndRetrieve) {
    Array<int, 6> arr = {0, 1, 2, 3, 4, 5};
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(arr[i], i);
}

TEST(Array, initializeAndRetrieveConst) {
    const Array<int, 6> arr = {0, 1, 2, 3, 4, 5};
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(arr[i], i);
}

TEST(Array, write) {
    Array<int, 6> arr = {42, 1, 2, 3, 4, 5};
    arr[0]            = 0;
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(arr[i], i);
}

TEST(Array, rangeFor) {
    Array<int, 6> arr = {0, 1, 2, 3, 4, 5};
    int i             = 0;
    for (int &el : arr)
        EXPECT_EQ(el, i++);
}

TEST(Array, rangeForConst) {
    const Array<int, 6> arr = {0, 1, 2, 3, 4, 5};
    int i                   = 0;
    for (const int &el : arr)
        EXPECT_EQ(el, i++);
}

TEST(Array, equality) {
    Array<int, 5> a = {1, 2, 3, 4, 5};
    Array<int, 5> b = {1, 2, 3, 4, 5};
    Array<int, 5> c = {1, 2, 3, 4, 6};
    EXPECT_EQ(a, b);
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);
    EXPECT_NE(a, c);
    EXPECT_FALSE(a == c);
    EXPECT_TRUE(a != c);
}

TEST(Array, equalityOne) {
    Array<int, 1> a = {42};
    EXPECT_TRUE(a == 42);
    EXPECT_FALSE(a == 43);
}

TEST(Array, operatorT) {
    Array<int, 1> a = {42};
    int i           = a;
    EXPECT_EQ(i, 42);
}

TEST(Array, compareLt) {
    Array<int, 3> a         = {1, 2, 3};
    Array<int, 3> b         = {1, 3, 2};
    Array<bool, 3> result   = a < b;
    Array<bool, 3> expected = {false, true, false};
    EXPECT_EQ(result, expected);
}

TEST(Array, compareLe) {
    Array<int, 3> a         = {1, 2, 3};
    Array<int, 3> b         = {1, 3, 2};
    Array<bool, 3> result   = a <= b;
    Array<bool, 3> expected = {true, true, false};
    EXPECT_EQ(result, expected);
}

TEST(Array, compareGt) {
    Array<int, 3> a         = {1, 2, 3};
    Array<int, 3> b         = {1, 3, 2};
    Array<bool, 3> result   = a > b;
    Array<bool, 3> expected = {false, false, true};
    EXPECT_EQ(result, expected);
}

TEST(Array, compareGe) {
    Array<int, 3> a         = {1, 2, 3};
    Array<int, 3> b         = {1, 3, 2};
    Array<bool, 3> result   = a >= b;
    Array<bool, 3> expected = {true, false, true};
    EXPECT_EQ(result, expected);
}

TEST(Array, assignSingleElement) {
    Array<int, 1> a;
    a               = 42;
    Array<int, 1> b = {42};
    EXPECT_EQ(a, b);
}

TEST(Array, fromCArray) {
    const int array[3]     = {1, 2, 3};
    Array<int, 3> result   = Array<int, 3>::fromCArray(array);
    Array<int, 3> expected = {1, 2, 3};
    EXPECT_EQ(result, expected);
}

TEST(Array, fromCppArray) {
    const int array[3]     = {1, 2, 3};
    Array<int, 3> result   = Array<int, 3>::fromCppArray(array);
    Array<int, 3> expected = {1, 2, 3};
    EXPECT_EQ(result, expected);
}

TEST(Array, abs) {
    Array<double, 3> a        = {0.0, -1.1, 2.2};
    Array<double, 3> expected = {0.0, 1.1, 2.2};
    EXPECT_EQ(abs(a), expected);
}

TEST(Array, isfinite) {
    Array<double, 3> m = {0.0, -1.1, 2.2};
    ASSERT_TRUE(isfinite(m));
    m[2] = std::numeric_limits<double>::infinity();
    ASSERT_FALSE(isfinite(m));
    m[2] = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(isfinite(m));
    m[2] = std::numeric_limits<double>::signaling_NaN();
    ASSERT_FALSE(isfinite(m));
    m[2] = 1.0 / 0.0;
    ASSERT_FALSE(isfinite(m));
}

TEST(Array, map) {
    Array<double, 4> a      = {-1, 2.2, 3, 4};
    Array<double, 4> result = map(a, [](double d) { return d + 5; });
    Array<double, 4> expected = {4, 7.2, 8, 9};
    ASSERT_EQ(result, expected);
}

// -------------------------------------------------------------------------- //

TEST(generateArray, simple) {
    auto x                   = generatedArray<unsigned int, 4>(2, 3);
    Array<unsigned int, 4> y = {2, 5, 8, 11};
    EXPECT_EQ(x, y);
}

TEST(fillArray, simple) {
    auto x                   = filledArray<unsigned int, 4>(42);
    Array<unsigned int, 4> y = {42, 42, 42, 42};
    EXPECT_EQ(x, y);
}