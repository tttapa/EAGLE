#pragma once

#include <cstdlib> // size_t

/**
 * Number of states of attitude model.
 *  - Orientation quaternion (4) 
 *  - Angular velocity (3)
 *  - Motor speeds (3)
 */
constexpr size_t Nx_att = 10;
/** 
 * Number of inputs of attitude model. 
 *  - Motor torque control (3)
 */
constexpr size_t Nu_att = 3;
/**
 * Number of outputs of attitude model. 
 *  - Orientation quaternion (4)
 *  - Angular velocity (3)
 */
constexpr size_t Ny_att = 7;

/** 
 * Number of states of the navigation model.
 *  - Velocity (3)
 *  - Position (3)
 *  - Motor speed (1)
 */
constexpr size_t Nx_nav = 7;
/** 
 * Number of inputs of the navigation model.
 *  - Motor thrust control (1)
 */
constexpr size_t Nu_nav = 1;
/** 
 * Number of outputs of the navigation model.
 *  - Position (3)
 */
constexpr size_t Ny_nav = 3;

/** Number of states. */
constexpr size_t Nx = Nx_att + Nx_nav;
/** Number of inputs. */
constexpr size_t Nu = Nu_att + Nu_nav;
/** Number of outputs. */
constexpr size_t Ny = Ny_att + Ny_nav;