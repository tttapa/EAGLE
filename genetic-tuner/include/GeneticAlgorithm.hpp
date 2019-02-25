#pragma once
#include <Chromosome.hpp>
#include <Member.hpp>
#include <cmath>

template <size_t N>
/**
 * Class providing the structure for the GA.
 */
class GeneticAlgorithm {

  public:

    //!!! MOVE THIS TO POPULATION CLASS!!!
    // Population should know how to 
    //   - initialize itself across a given domain
    //   - calculate (and remember) its members' scores
    //   - return a sub-population with the top x members
    /**
     * Initialize the population with a given size and spread the population
     * uniformly across the domain, either using a normal scale or a logarithmic
     * scale.
     */
    void initializePopulation(ColVector<N> minValues, ColVector<N> maxValues,
                              size_t size, bool useLogarithm) {
        population.resize(size);

        if (useLogarithmicDistribution) {
            // Uniformly distribute members across logarithmic scale.
            double logValue;
            for (size_t member = 0; member < size; member++) {
                for (size_t gene = 0; gene < N; gene++) {
                    logValue = (double) member / (double) size *
                               (log(maxValues[gene]) - log(minValues[gene]));
                    population[member][gene] = pow(10, logValue);
                }
            }
        } else {
            // Uniformly distribute members across normal scale.
            for (size_t member = 0; member < size; member++) {
                for (size_t gene = 0; gene < N; gene++) {
                    population[member][gene] =
                        (double) member / (double) size *
                        (maxValues[gene] - minValues[gene]);
                }
            }
        }
    }

    /**
     * Generate 

  protected:
    vector<Member> population;
    vector<Member> allChildren;
}