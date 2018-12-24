#include <gtest/gtest.h>

#include <FileLoader.hpp>
#include <filesystem>
#include <libgen.h>

using std::filesystem::path;

TEST(FileLoader, LoadMatrix) {
    path f                = __FILE__;
    auto p                = f.parent_path();
    Matrix<2, 3> result   = loadMatrix<2, 3>(p / "test");
    Matrix<2, 3> expected = {{
        {1, 2, 3},
        {4, 5, 6},
    }};
    ASSERT_EQ(result, expected);
}

TEST(FileLoader, LoadMatrixWrongRows) {
    path f = __FILE__;
    auto p = f.parent_path();
    try {
        loadMatrix<3, 3>(p / "test");
        FAIL();
    } catch (std::runtime_error &e) {
        ASSERT_EQ(
            e.what(),
            std::string("Error: number of rows in file doesn't match expected "
                        "number of rows. ") +
                "(" + (p / "test.matrix").string() +
                ") "
                "Expected: 3, File: 2");
    }
}

TEST(FileLoader, LoadMatrixWrongCols) {
    path f = __FILE__;
    auto p = f.parent_path();
    try {
        loadMatrix<2, 4>(p / "test");
        FAIL();
    } catch (std::runtime_error &e) {
        ASSERT_EQ(e.what(),
                  std::string(
                      "Error: number of columns in file doesn't match expected "
                      "number of columns. ") +
                      "(" + (p / "test.matrix").string() +
                      ") "
                      "Expected: 4, File: 3");
    }
}

TEST(FileLoader, LoadDouble) {
    path f          = __FILE__;
    auto p          = f.parent_path();
    double result   = loadDouble(p / "test");
    double expected = M_PI;
    ASSERT_EQ(result, expected);
}