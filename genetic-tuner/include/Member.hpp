#pragma once
#include <Chromosome.hpp>

template <size_t N>
/**
 * A member is an abstract class containing genetic data and some function to
 * evaluate the fitness of that chromosome. The fitness function (simulation)
 * must be specified by the subclass.
 */
class Member {

  public:
    /** Fitness function provided by subclass's simulation. */
    virtual double getFitness();

    /** Get this member's genetic data. */
    Chromosome<N> getChromosome() { return chromosome; }

    /** Mutate this member's chromosome by a given factor. */
    void mutateChromosome(double factor) { mutate(chromosome, factor); }

  protected:
    /** This member's genetic data. */
    Chromosome<N> chromosome;
};

/**
 * Member of the "attitude" population used to determine the LQR cost matrices
 * for the attitude controller. Chromosome contains 6 values for the cost matrix
 * Q and 2 values for the cost matrix R. These values are q12, q3, w12, w3, n12,
 * n3, u12, u3. We assume symmetry in the x- and y-direction, so Q = diag([q12,
 * q12,q3,w12,w12,w3,n12,n12,n3]) and R = diag([u12,u12,u3]).
 */
class AttitudeMember : Member<8> {

  public:

    /**
     * Return the fitness of this AttitudeMember based on simulation results.
     * TODO: `return 0.0;`
     */
    double getFitness() { return 0.0; }

    /** Return the diagonal of this AttitudeMember's Q cost matrix. */
    ColVector<9> getQDiag() {
        return ColVector<9>{
            chromosome[0], chromosome[0], chromosome[1],  // q1, q2, q3
            chromosome[2], chromosome[2], chromosome[3],  // w1, w2, w3
            chromosome[4], chromosome[4], chromosome[5],  // n1, n2, n3
        };
    }

    /** Return the diagonal of this AttitudeMember's R cost matrix. */
    ColVector<3> getRDiag() {  // ux, uy, uz
        return ColVector<3>{chromosome[6], chromosome[6], chromosome[7]};
    }

    /** Return this AttitudeMember's Q cost matrix (9x9). */
    auto getQ() { return diag(getQDiag()); }

    /** Return this AttitudeMember's R cost matrix (3x3). */
    auto getR() { return diag(getRDiag()); }
};

/**
 * Member of the "altitude" population used to determine the LQR cost matrices
 * for the altitude controller. Chromosome contains 3 values for the cost matrix
 * Q and 1 value for the cost matrix R. These values are nt, vz, z, ut. Thus Q =
 * diag([nt,vz,z]) and R = ut.
 */
class AltitudeMember : Member<4> {

  public:
    /**
     * Return the fitness of this AltitudeMember based on simulation results.
     * TODO: `return 0.0;`
     */
    double getFitness() { return 0.0; }

    /** Return the diagonal of this AltitudeMember's Q cost matrix. */
    ColVector<3> getQDiag() {  // nt, vz, z
        return ColVector<3>{chromosome[0], chromosome[1], chromosome[2]};
    }

    /** Return the diagonal of this AltitudeMember's R cost matrix. */
    ColVector<1> getRDiag() { return ColVector<1>{chromosome[3]}; }  // ut

    /** Return this AltitudeMember's Q cost matrix (3x3). */
    auto getQ() { return diag(getQDiag()); }

    /** Return this AltitudeMember's R cost matrix (1x1). */
    auto getR() { return diag(getRDiag()); }
};

/**
 * Member of the "navigation" population used to determine the LQR cost matrices
 * for the navigation controller. Chromosome contains 3 values for the cost
 * matrix Q and 1 value for the cost matrix R. These values are q12, xy, vxy,
 * q12ref. We assume symmetry in the x- and y-direction, so Q = diag([q12,q12,
 * xy,xy,vxy,vxy]) and R = diag([q12ref,q12ref]).
 */
class NavigationMember : Member<4> {

  public:
    /**
     * Return the fitness of this NavigationMember based on simulation results.
     * TODO: `return 0.0;`
     */
    double getFitness() { return 0.0; }

    /** Return the diagonal of this NavigationMember's Q cost matrix. */
    ColVector<6> getQDiag() {
        return ColVector<6>{
            chromosome[0], chromosome[0],  // q1, q2
            chromosome[1], chromosome[1],  // x, y
            chromosome[2], chromosome[2],  // vx, vy
        };
    }

    /** Return the diagonal of this NavigationMember's R cost matrix. */
    ColVector<2> getRDiag() {
        return ColVector<2>{chromosome[3], chromosome[3]};  // qref1, qref2
    }

    /** Return this NavigationMember's Q cost matrix (6x6). */
    auto getQ() { return diag(getQDiag()); }

    /** Return this NavigationMember's R cost matrix (2x2). */
    auto getR() { return diag(getRDiag()); }
};