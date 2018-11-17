#include <gtest/gtest.h>

#include <libgen.h>
#include <Util/FileLoader.hpp>

#include <filesystem>
namespace fs = std::filesystem;

TEST(FileLoader, LoadMatrix) {
    fs::path f = __FILE__;
    f = f.parent_path() / "test";
    Matrix<2, 3> result = loadMatrix<2, 3>(f.string());
    Matrix<2, 3> expected = {{
        {1, 2, 3},
        {4, 5, 6},
    }};
    ASSERT_EQ(result, expected);
}