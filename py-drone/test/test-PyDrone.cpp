#include <gtest/gtest.h>

#include <PyDrone.hpp>

namespace py = pybind11;

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    pybind11::scoped_interpreter interpreter = {};
    return RUN_ALL_TESTS();
}

TEST(PyDrone, Python2Cpp) {
    using namespace pybind11::literals;
    auto module = py::module::import("PyDrone");
    auto locals = pybind11::dict{"pydrone"_a = module};
    pybind11::exec(R"(
        import numpy as np
        x = pydrone.DroneParamsAndMatrices()
        x.uh = 3.14
        x.Id = np.array((
            (1, 2, 3),
            (4, 5, 6),
            (7, 8, 9)
        ))
    )",
                   pybind11::globals(), locals);
    auto px                  = locals["x"];
    DroneParamsAndMatrices x = px.cast<DroneParamsAndMatrices>();
    Matrix<3, 3> expectedId  = {{
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9},
    }};
    EXPECT_EQ(x.uh, 3.14);
    EXPECT_EQ(x.Id, expectedId);
}

TEST(PyDrone, Cpp2Python) {
    using namespace pybind11::literals;
    auto module = py::module::import("PyDrone");
    DroneParamsAndMatrices x;
    x.uh                  = 3.14;
    x.Id                  = {{
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9},
    }};
    pybind11::object px   = pybind11::cast(x);
    pybind11::dict locals = pybind11::dict{"x"_a = px};
    pybind11::exec(R"(
        import numpy as np
        expectedId = np.array((
            (1, 2, 3),
            (4, 5, 6),
            (7, 8, 9)
        ))
        success = x.uh == 3.14 and np.array_equal(x.Id, expectedId)
    )",
                   pybind11::globals(), locals);
    ASSERT_TRUE(locals["success"].cast<bool>());
}

TEST(PyDrone, loadPython) {
    using namespace pybind11::literals;
    std::filesystem::path f = __FILE__;
    auto p                  = f.parent_path() / "ParamsAndMatrices";
    auto s                  = p.string();
    auto module             = py::module::import("PyDrone");
    pybind11::dict locals = pybind11::dict{"path"_a = s, "pydrone"_a = module};
    pybind11::exec(R"(
        x = pydrone.DroneParamsAndMatrices()
        x.load(path)
    )",
                   pybind11::globals(), locals);
    auto px                       = locals["x"];
    DroneParamsAndMatrices x      = px.cast<DroneParamsAndMatrices>();
    Matrix<10, 10> expectedAa_att = {{
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 3.2955056672349254, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 3.1113450564188558, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, -1.9360346773861661},
        {0, 0, 0, 0, 0, 0, 0, -28.571428571428569, -0, -0},
        {0, 0, 0, 0, 0, 0, 0, -0, -28.571428571428569, -0},
        {0, 0, 0, 0, 0, 0, 0, -0, -0, -28.571428571428569},
    }};
    ASSERT_EQ(x.Aa_att, expectedAa_att);
}