#include "RegexReplace.hpp"
#include <Matrix/Matrix.hpp>
#include <map>
#include <string>
#include <vector>

class DynamicMatrix {
  public:
    DynamicMatrix() : data{} {}
    template <size_t R, size_t C>
    DynamicMatrix(const Matrix<R, C> &matrix) {
        data.resize(R);
        for (size_t r = 0; r < R; ++r) {
            data[r].resize(C);
            std::copy(matrix[r].begin(), matrix[r].end(), data[r].begin());
        }
    }
    std::vector<double> &at(size_t i) { return data.at(i); }
    const std::vector<double> &at(size_t i) const { return data.at(i); }
    std::vector<std::vector<double>> data;
};

std::string doubleToString(double dbl, int precision = 6);

void replaceTagsInFile(const std::string &infilename,
                       const std::string &outfilename,
                       const std::map<std::string, DynamicMatrix> &matrixdict);