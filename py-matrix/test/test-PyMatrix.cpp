#include <gtest/gtest.h>
#include <pybind11/embed.h>

#include <PyMatrix.hpp>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  pybind11::scoped_interpreter interpreter = {};
  return RUN_ALL_TESTS();
}

TEST(PyMatrix, Cpp2Python) {
    Matrix<5, 3> m = {{
        {11, 12, 13},
        {21, 22, 23},
        {31, 32, 33},
        {41, 42, 43},
        {51, 52, 53},
    }};

    using namespace pybind11::literals;

    auto locals = pybind11::dict{"m"_a = m};
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

    using namespace pybind11::literals;

    auto locals = pybind11::dict{"m"_a = m};
    pybind11::exec(R"(
        import numpy as np
        expected = np.array((
            (11, 12, 13),
            (21, 22, 23),
            (31, 32, 33),
            (41, 42, 43),
            (51, 52, 53)
        ))
        success = np.array_equal(m, expected) and m.dtype == np.int32
        )",
                   pybind11::globals(), locals);

    ASSERT_TRUE(locals["success"].cast<bool>());
}

TEST(PyMatrix, Python2Cpp) {
    using namespace pybind11::literals;

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
    using namespace pybind11::literals;

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
    using namespace pybind11::literals;

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
    using namespace pybind11::literals;

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
    using namespace pybind11::literals;

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