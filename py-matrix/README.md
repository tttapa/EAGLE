# PyMatrix

This header-only library handles bidirectional conversion between 2D Python
NumPy arrays and C++ `TMatrix<T, R, C>` instances.

## C++ → Python
Casting from C++ to Python should happen automatically, if not, use:
```cpp
TMatrix<int, 3, 4> matrix = {};
pybind11::object pyMatrix = pybind11::cast(matrix);
```
## Python → C++
Casting from C++ to Python should happen automatically, if not, use:
```cpp
pybind11::object pyMatrix = (...);
TMatrix<int, 3, 4> matrix = pybind11::cast<TMatrix<int, 3, 4>>(pyMatrix);
```
Conversion will fail if the type is incompatible, or if the dimensions do not 
match.  
This can only be checked at runtime, because NumPy arrays are dynamic by 
definition.  
When casting from a Python object to a `TMatrix<T, R, C>` fails, an exception is
thrown.

Don't forget to initialize the Python Interpreter before running any Python or 
pybind11 code.