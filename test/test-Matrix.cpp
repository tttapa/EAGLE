#include <Matrix/Matrix.hpp>
#include <gtest/gtest.h>

using Matrices::T;
using std::cout;
using std::endl;

TEST(Matrix, equals) {
    TMatrix<int, 3, 2> a = {{
        {11, 12},
        {21, 22},
        {31, 32},
    }};
    TMatrix<int, 3, 2> b = {{
        {11, 12},
        {21, 22},
        {31, 32},
    }};

    ASSERT_EQ(a, b);
    ASSERT_EQ(b, a);
    ASSERT_TRUE(a == b);
    ASSERT_TRUE(b == a);
    ASSERT_FALSE(a != b);
    ASSERT_FALSE(b != a);
}

TEST(Matrix, notEquals) {
    TMatrix<int, 3, 2> a = {{
        {11, 12},
        {21, 22},
        {31, 32},
    }};
    TMatrix<int, 3, 2> b = {{
        {11, 12},
        {21, 22},
        {31, 33},
    }};

    ASSERT_NE(a, b);
    ASSERT_NE(b, a);
    ASSERT_TRUE(a != b);
    ASSERT_TRUE(b != a);
    ASSERT_FALSE(a == b);
    ASSERT_FALSE(b == a);
}

TEST(Matrix, operatorTMatrix) {
    TMatrix<int, 1, 1> a = {42};
    int i = a;
    ASSERT_EQ(i, 42);
}

TEST(Matrix, operatorTMatrixConst) {
    const TMatrix<int, 1, 1> a = {42};
    int i = a;  
    ASSERT_EQ(i, 42);
}

TEST(Matrix, multiply) {
    TMatrix<int, 3, 2> a = {{
        {11, 12},
        {21, 22},
        {31, 32},
    }};
    TMatrix<int, 2, 4> b = {{
        {11, 12, 13, 14},
        {21, 22, 23, 24},
    }};

    TMatrix<int, 3, 4> result   = a * b;
    TMatrix<int, 3, 4> expected = {{
        {373, 396, 419, 442},
        {693, 736, 779, 822},
        {1013, 1076, 1139, 1202},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, scalarMultiply) {
    TMatrix<int, 3, 2> m = {{
        {11, 12},
        {21, 22},
        {31, 32},
    }};
    int s               = 2;

    TMatrix<int, 3, 2> result1  = s * m;
    TMatrix<int, 3, 2> result2  = m * s;
    TMatrix<int, 3, 2> expected = {{
        {22, 24},
        {42, 44},
        {62, 64},
    }};
    ASSERT_EQ(result1, expected);
    ASSERT_EQ(result2, expected);
}

TEST(Matrix, Tdiag) {
    TMatrix<int, 3, 3> result   = Tdiag<int, 3>({{{11, 22, 33}}});
    TMatrix<int, 3, 3> expected = {{
        {11, 0, 0},
        {0, 22, 0},
        {0, 0, 33},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, diag) {
    Matrix<3, 3> result   = diag<3>({{{11, 22, 33}}});
    Matrix<3, 3> expected = {{
        {11, 0, 0},
        {0, 22, 0},
        {0, 0, 33},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, add) {
    TMatrix<int, 2, 3> a = {{
        {11, 12, 13},
        {21, 22, 23},
    }};
    TMatrix<int, 2, 3> b = {{
        {101, 102, 103},
        {201, 202, 203},
    }};

    TMatrix<int, 2, 3> result   = a + b;
    TMatrix<int, 2, 3> expected = {{
        {112, 114, 116},
        {222, 224, 226},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, subtract) {
    TMatrix<int, 2, 3> a = {{
        {111, 122, 133},
        {211, 222, 233},
    }};
    TMatrix<int, 2, 3> b = {{
        {11, 12, 13},
        {21, 22, 23},
    }};

    TMatrix<int, 2, 3> result   = a - b;
    TMatrix<int, 2, 3> expected = {{
        {100, 110, 120},
        {190, 200, 210},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, transpose) {
    TMatrix<int, 2, 3> a = {{
        {11, 12, 13},
        {21, 22, 23},
    }};

    TMatrix<int, 3, 2> result1  = transpose(a);
    TMatrix<int, 3, 2> result2  = a ^ T;
    TMatrix<int, 3, 2> expected = {{
        {11, 21},
        {12, 22},
        {13, 23},
    }};
    ASSERT_EQ(result1, expected);
    ASSERT_EQ(result2, expected);
}

TEST(Matrix, assignBlock) {
    TMatrix<int, 4, 5> m        = {{
        {11, 12, 13, 14, 15},
        {21, 22, 23, 24, 25},
        {31, 32, 33, 34, 35},
        {41, 42, 43, 44, 45},
    }};
    assignBlock<1, 3, 2, 4>(m) = {{
        {101, 102},
        {103, 104},
    }};
    TMatrix<int, 4, 5> expected = {{
        {11, 12, 13, 14, 15},
        {21, 22, 101, 102, 25},
        {31, 32, 103, 104, 35},
        {41, 42, 43, 44, 45},
    }};
    ASSERT_EQ(m, expected);
}

TEST(Matrix, getBlock) {
    TMatrix<int, 4, 5> m        = {{
        {11, 12, 13, 14, 15},
        {21, 22, 23, 24, 25},
        {31, 32, 33, 34, 35},
        {41, 42, 43, 44, 45},
    }};
    TMatrix<int, 2, 2> result   = getBlock<1, 3, 2, 4>(m);
    TMatrix<int, 2, 2> expected = {{
        {23, 24},
        {33, 34},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, hcat) {
    TMatrix<int, 3, 2> l        = {{
        {11, 12},
        {21, 22},
        {31, 32},
    }};
    TMatrix<int, 3, 2> r        = {{
        {13, 14},
        {23, 24},
        {33, 34},
    }};
    TMatrix<int, 3, 4> result   = hcat(l, r);
    TMatrix<int, 3, 4> expected = {{
        {11, 12, 13, 14},
        {21, 22, 23, 24},
        {31, 32, 33, 34},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, hcat3) {
    TMatrix<int, 2, 2> l        = {{
        {11, 12},
        {21, 22},
    }};
    TMatrix<int, 2, 2> m        = {{
        {13, 14},
        {23, 24},
    }};
    TMatrix<int, 2, 3> r        = {{
        {15, 16, 17},
        {25, 26, 27},
    }};
    TMatrix<int, 2, 7> result   = hcat(l, m, r);
    TMatrix<int, 2, 7> expected = {{
        {11, 12, 13, 14, 15, 16, 17},
        {21, 22, 23, 24, 25, 26, 27},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, vcat3) {
    TMatrix<int, 1, 2> t        = {{
        {11, 12},
    }};
    TMatrix<int, 2, 2> m        = {{
        {21, 22},
        {31, 32},
    }};
    TMatrix<int, 3, 2> b        = {{
        {41, 42},
        {51, 52},
        {61, 62},
    }};
    TMatrix<int, 6, 2> result   = vcat(t, m, b);
    TMatrix<int, 6, 2> expected = {{
        {11, 12},
        {21, 22},
        {31, 32},
        {41, 42},
        {51, 52},
        {61, 62},
    }};
    ASSERT_EQ(result, expected);
}

TEST(Matrix, vcathcat) {
    TMatrix<int, 2, 3> lt       = {{
        {1, 2, 3},
        {6, 7, 8},
    }};
    TMatrix<int, 2, 2> rt       = {{
        {4, 5},
        {9, 10},
    }};
    TMatrix<int, 3, 3> lb       = {{
        {11, 12, 13},
        {16, 17, 18},
        {21, 22, 23},
    }};
    TMatrix<int, 3, 2> rb       = {{
        {14, 15},
        {19, 20},
        {24, 25},
    }};
    TMatrix<int, 5, 5> result   = vcat(hcat(lt, rt), hcat(lb, rb));
    TMatrix<int, 5, 5> expected = {{
        {1, 2, 3, 4, 5},
        {6, 7, 8, 9, 10},
        {11, 12, 13, 14, 15},
        {16, 17, 18, 19, 20},
        {21, 22, 23, 24, 25},
    }};
    ASSERT_EQ(result, expected);
}