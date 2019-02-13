#include <iostream>

#include <Matrix.hpp>

using namespace std;

int main(int argc, const char *argv[]) {
    for (int i = 0; i < argc; ++i)
        cout << argv[i] << ',';
    cout << endl;
    Matrix<2, 3> m = {{
        {1, 2, 3},
        {4, 5, 6},
    }};
    cout << m << endl;
    return EXIT_SUCCESS;
}