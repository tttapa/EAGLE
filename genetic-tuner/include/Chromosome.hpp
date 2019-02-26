#pragma once

#include <Matrix.hpp>
#include <random>
#include <sstream>


template <size_t N>
/**
 * Chromosome is a ColVector of size N that can be used in GA operations like
 * mutation and cross-over.
 */
using Chromosome = ColVector<N>;

/**
 * Generator used to choose random indices. This generator is also used to
 * perform mutations.
 */
static std::random_device rd;
static std::mt19937 gen(rd());  // Mersenne_twister_engine seeded with rd()


template <size_t N>
/**
 * Perform crossing-over between two parent chromosomes to create two child
 * chromosomes.
 */
void crossOver(const Chromosome<N> &parent1, const Chromosome<N> &parent2,
               Chromosome<N> &child1, Chromosome<N> &child2) {

    // Choose a random index in [0,N].
    std::uniform_int_distribution<size_t> distr(0, N);
    size_t idx = distr(gen);

    // Flip the genes starting at that random index.
    for (size_t i = 0; i < idx; i++) {
        child1[i] = parent1[i];
        child2[i] = parent2[i];
    }
    for (size_t i = idx; i < N; i++) {
        child1[i] = parent2[i];
        child2[i] = parent1[i];
    }
}


template <size_t N>
/**
 * Mutate the given chromosome by adding an extra dChrom to the chromosome,
 * where `dChrom[i] = factor*randn*chrom[i]`.
 */
void mutate(Chromosome<N> &chrom, double factor) {

    // Standard normal distribution
    std::normal_distribution<double> distribution(0.0, 1.0);

    for (size_t i = 0; i < N; i++) {
        double random = distribution(gen);
        chrom[i] += factor * random * chrom[i];
    }
}


template <size_t N>
/**
 * Create string representation of the given chromosome.
 */
std::string toString(Chromosome<N> &chrom) {
    std::ostringstream result;
    result << '[';
    for (size_t i = 0; i < N; i++) {
        if (i > 0)
            result << ',';
        result << chrom[i];
    }
    result << ']';
    return result.str();
}