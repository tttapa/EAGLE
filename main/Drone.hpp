#pragma once

#include "Params.hpp"

struct Drone {
    Drone() { compute(); }

    constexpr static size_t Nx = 10;
    constexpr static size_t Nu = 3;
    constexpr static size_t Ny = 7;

    void compute() {
        p.compute();
        A = vcat(                                                //
            zeros<1, 10>(),                                      //
            hcat(zeros<3, 4>(), 0.5 * eye<3>(), zeros<3, 3>()),  //
            hcat(zeros<3, 7>(), p.gamma_n),                      //
            hcat(zeros<3, 7>(), -p.k2 * eye<3>())                //
        );
        B = vcat(                   //
            zeros<4, 3>(),          //
            p.gamma_u,              //
            p.k2 * p.k1 * eye<3>()  //
        );
        C = hcat(eye<Ny>(), zeros<Ny, Nu>());
        D = zeros<Ny, Nu>();
    }

    Params p;

    /** ```
     *  A =  [  ·   ·   ·   ·   ·   ·   ·   ·   ·   ·  ]
     *       [  ·   ·   ·   ·  ┌─────────┐  ·   ·   ·  ]
     *       [  ·   ·   ·   ·  │  0.5 I3 │  ·   ·   ·  ]
     *       [  ·   ·   ·   ·  └─────────┘  ·   ·   ·  ]
     *       [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  │   Γ_n   │ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  │ -k2 I3  │ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ] 
     * ``` */
    Matrix<Nx, Nx> A;

    /** ```
     *  B =  [  ·   ·   ·  ]
     *       [  ·   ·   ·  ]
     *       [  ·   ·   ·  ]
     *       [  ·   ·   ·  ]
     *       [ ┌─────────┐ ]
     *       [ │   Γ_u   │ ]
     *       [ └─────────┘ ]
     *       [ ┌─────────┐ ]
     *       [ │ k1k2 I3 │ ]
     *       [ └─────────┘ ] 
     * ``` */
    Matrix<Nx, Nu> B;

    /** ```
     *  C =  [ 1  ·  ·  ·  ·  ·  ·  ·  ·  · ]
     *       [ ·  1  ·  ·  ·  ·  ·  ·  ·  · ]
     *       [ ·  ·  1  ·  ·  ·  ·  ·  ·  · ]
     *       [ ·  ·  ·  1  ·  ·  ·  ·  ·  · ]
     *       [ ·  ·  ·  ·  1  ·  ·  ·  ·  · ]
     *       [ ·  ·  ·  ·  ·  1  ·  ·  ·  · ]
     *       [ ·  ·  ·  ·  ·  ·  1  ·  ·  · ] 
     * ``` */
    Matrix<Ny, Nx> C;

    /** ```
     *  D =  [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ] 
     * ``` */
    Matrix<Ny, Nu> D;
};
