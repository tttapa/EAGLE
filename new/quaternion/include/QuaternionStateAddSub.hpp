#pragma once

#include "Quaternion.hpp"

/**
 * @brief   Add two state vectors where the first 4 elements are to be 
 *          interpreted as quaternions.
 * 
 * @tparam  N
 *          The number of elements in the state vectors.
 * @param   a 
 *          The left operand of the addition.
 * @param   b
 *          The right operand of the addition. 
 * @return
 *          @f$ \begin{pmatrix}
 *          \begin{pmatrix} a_0 \\ a_1 \\ a_2 \\ a_3 \end{pmatrix} \otimes
 *          \begin{pmatrix} b_0 \\ b_1 \\ b_2 \\ b_3 \end{pmatrix} \\
 *          a_4 + b_4 \\
 *          \vdots \\
 *          a_{N-1} + b_{N-1}
 *          \end{pmatrix} @f$  
 *          Where @f$ \otimes @f$ is the Hamiltonian product of two quaternions.
 */
template <size_t N>
ColVector<N> quaternionStatesAdd(const ColVector<N> &a, const ColVector<N> &b) {
    ColVector<N> result;
    assignBlock<4, N, 0, 1>(result) =
        getBlock<4, N, 0, 1>(a) + getBlock<4, N, 0, 1>(b);
    assignBlock<0, 4, 0, 1>(result) =
        quatmultiply(getBlock<0, 4, 0, 1>(a), getBlock<0, 4, 0, 1>(b));
    return result;
}

/**
 * @brief   Subtract two state vectors where the first 4 elements are to be 
 *          interpreted as quaternions.
 * 
 * @tparam  N
 *          The number of elements in the state vectors.
 * @param   a 
 *          The left operand of the subtraction.
 * @param   b
 *          The right operand of the subtraction. 
 * @return
 *          @f$ \begin{pmatrix}
 *          \begin{pmatrix} a_0 \\ a_1 \\ a_2 \\ a_3 \end{pmatrix} \otimes
 *          \begin{pmatrix} b_0 \\ -b_1 \\ -b_2 \\ -b_3 \end{pmatrix} \\
 *          a_4 - b_4 \\
 *          \vdots \\
 *          a_{N-1} - b_{N-1}
 *          \end{pmatrix} @f$  
 *          Where @f$ \otimes @f$ is the Hamiltonian product of two quaternions.
 */
template <size_t N>
ColVector<N> quaternionStatesSub(const ColVector<N> &a, const ColVector<N> &b) {
    ColVector<N> result;
    assignBlock<4, N, 0, 1>(result) =
        getBlock<4, N, 0, 1>(a) - getBlock<4, N, 0, 1>(b);
    assignBlock<0, 4, 0, 1>(result) =
        quatDifference(getBlock<0, 4, 0, 1>(a), getBlock<0, 4, 0, 1>(b));
    return result;
}