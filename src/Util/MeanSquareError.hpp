#include <numeric>

template <class InputIt>
auto squareError(InputIt first1, InputIt last1, InputIt first2) {
    auto square     = [](auto x) { return x * x; };
    auto diffsquare = [&square](auto x, auto y) { return square(x - y); };
    auto plus       = [](auto x, auto y) { return x + y; };
    using sum_t =
        std::remove_reference_t<decltype(diffsquare(*first1, *first2))>;
    sum_t initial = {};
    return std::inner_product(first1, last1, first2, initial, plus, diffsquare);
}

template <class InputIt>
auto meanSquareError(InputIt first1, InputIt last1, InputIt first2) {
    return squareError(first1, last1, first2) / (last1 - first1);
}

template <class InputIt>
auto rootMeanSquareError(InputIt first1, InputIt last1, InputIt first2) {
    return sqrt(meanSquareError(first1, last1, first2));
}

template <class InputIt>
auto maxSquareError(InputIt first1, InputIt last1, InputIt first2) {
    auto square     = [](auto x) { return x * x; };
    auto maxSquareError = decltype(square(*first1 - *first2)){};
    while (first1 != last1) {
        auto errorSq = square(*first1++ - *first2++);
        if (errorSq > maxSquareError)
            maxSquareError = errorSq;
    }
    return maxSquareError;
}