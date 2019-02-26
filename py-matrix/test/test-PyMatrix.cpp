#include <gtest/gtest.h>
#include <pybind11/embed.h>

#include <PyMatrix.hpp>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    pybind11::scoped_interpreter interpreter = {};
    return RUN_ALL_TESTS();
}

using namespace pybind11::literals;

TEST(PyMatrix, Cpp2Python) {
    Matrix<5, 3> m = {{
        {11, 12, 13},
        {21, 22, 23},
        {31, 32, 33},
        {41, 42, 43},
        {51, 52, 53},
    }};
    auto locals    = pybind11::dict{"m"_a = m};
    pybind11::exec(R"(
        import numpy as np
        expected = np.array((
            (11, 12, 13),
            (21, 22, 23),
            (31, 32, 33),
            (41, 42, 43),
            (51, 52, 53)
        ))
        success = np.array_equal(m, expected) and m.dtype == np.double
        )",
                   pybind11::globals(), locals);
    ASSERT_TRUE(locals["success"].cast<bool>());
}

TEST(PyMatrix, Cpp2PythonInt) {
    TMatrix<int, 5, 3> m = {{
        {11, 12, 13},
        {21, 22, 23},
        {31, 32, 33},
        {41, 42, 43},
        {51, 52, 53},
    }};
    auto locals          = pybind11::dict{"m"_a = m};
    pybind11::exec(R"(
        import numpy as np
        expected = np.array((
            (11, 12, 13),
            (21, 22, 23),
            (31, 32, 33),
            (41, 42, 43),
            (51, 52, 53)
        ))
        success = np.array_equal(m, expected) and m.dtype == np.intc
        )",
                   pybind11::globals(), locals);
    ASSERT_TRUE(locals["success"].cast<bool>());
}

TEST(PyMatrix, Python2Cpp) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        matrix = np.array((
            (11, 12, 13),
            (21, 22, 23),
            (31, 32, 33),
            (41, 42, 43),
            (51, 52, 53)
        ))
        )",
                   pybind11::globals(), locals);
    Matrix<5, 3> result   = locals["matrix"].cast<Matrix<5, 3>>();
    Matrix<5, 3> expected = {{
        {11, 12, 13},
        {21, 22, 23},
        {31, 32, 33},
        {41, 42, 43},
        {51, 52, 53},
    }};
    ASSERT_EQ(result, expected);
}

TEST(PyMatrix, Python2CppInt) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        matrix = np.array((
            (11, 12, 13),
            (21, 22, 23),
            (31, 32, 33),
            (41, 42, 43),
            (51, 52, 53)
        ))
        )",
                   pybind11::globals(), locals);
    TMatrix<int, 5, 3> result   = locals["matrix"].cast<TMatrix<int, 5, 3>>();
    TMatrix<int, 5, 3> expected = {{
        {11, 12, 13},
        {21, 22, 23},
        {31, 32, 33},
        {41, 42, 43},
        {51, 52, 53},
    }};
    ASSERT_EQ(result, expected);
}

TEST(PyMatrix, Python2CppWrongDimensionsRow) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        matrix = np.array((
            (11, 12, 13),
            (21, 22, 23),
            (31, 32, 33),
            (41, 42, 43),
        ))
        )",
                   pybind11::globals(), locals);
    try {
        locals["matrix"].cast<Matrix<5, 3>>();
        FAIL();
    } catch (std::runtime_error &e) {
        SUCCEED();
    }
}

TEST(PyMatrix, Python2CppWrongDimensionsCol) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        matrix = np.array((
            (11, 12, 13, 14),
            (21, 22, 23, 24),
            (31, 32, 33, 34),
            (41, 42, 43, 44),
            (51, 52, 53, 54)
        ))
        )",
                   pybind11::globals(), locals);
    try {
        locals["matrix"].cast<Matrix<5, 3>>();
        FAIL();
    } catch (std::runtime_error &e) {
        SUCCEED();
    }
}

TEST(PyMatrix, Python2CppWrongType) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        matrix = np.array((
            (11, 12, 13, 14),
            (21, 22, 23, 24),
            (31, 32, 33, 34),
            (41, 42, 43, 44),
            (51, 52, 53, 54)
        ))
        )",
                   pybind11::globals(), locals);
    try {
        struct Foo {};
        locals["matrix"].cast<TMatrix<Foo, 5, 4>>();
        FAIL();
    } catch (std::runtime_error &e) {
        SUCCEED();
    }
}

TEST(PyMatrix, Cpp2PythonColVector) {
    ColVector<3> v = {{1, 2, 3}};
    auto locals    = pybind11::dict{"v"_a = v};
    pybind11::exec(R"(
        import numpy as np
        expected = np.array(((1,), (2,), (3,)))
        success = np.array_equal(v, expected) and v.dtype == np.double
        )",
                   pybind11::globals(), locals);
    ASSERT_TRUE(locals["success"].cast<bool>());
}

TEST(PyMatrix, Python2CppColVector) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        v = np.array((1, 2, 3))
        )",
                   pybind11::globals(), locals);

    ColVector<3> result   = locals["v"].cast<ColVector<3>>();
    ColVector<3> expected = {{1, 2, 3}};

    ASSERT_EQ(result, expected);
}

TEST(PyMatrix, Python2CppColVector2D) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        v = np.array(((1,), (2,), (3,)))
        )",
                   pybind11::globals(), locals);

    ColVector<3> result   = locals["v"].cast<ColVector<3>>();
    ColVector<3> expected = {{1, 2, 3}};

    ASSERT_EQ(result, expected);
}

TEST(PyMatrix, Cpp2PythonRowVector) {
    RowVector<3> v = {{1, 2, 3}};
    auto locals    = pybind11::dict{"v"_a = v};
    pybind11::exec(R"(
        import numpy as np
        expected = np.array(((1, 2, 3),))
        success = np.array_equal(v, expected) and v.dtype == np.double
        )",
                   pybind11::globals(), locals);
    ASSERT_TRUE(locals["success"].cast<bool>());
}

TEST(PyMatrix, Python2CppRowVector) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        v = np.array((1, 2, 3))
        )",
                   pybind11::globals(), locals);

    RowVector<3> result   = locals["v"].cast<RowVector<3>>();
    RowVector<3> expected = {{1, 2, 3}};

    ASSERT_EQ(result, expected);
}

TEST(PyMatrix, Python2CppRowVector2D) {
    auto locals = pybind11::dict{};
    pybind11::exec(R"(
        import numpy as np
        v = expected = np.array(((1, 2, 3),))
        )",
                   pybind11::globals(), locals);

    RowVector<3> result   = locals["v"].cast<RowVector<3>>();
    RowVector<3> expected = {{1, 2, 3}};

    ASSERT_EQ(result, expected);
}