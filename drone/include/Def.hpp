#pragma once

#include <cstdlib>  // size_t

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
 * Number of states of the altitude model.
 *  - Velocity (1)
 *  - Position (1)
 *  - Motor speed (1)
 */
constexpr size_t Nx_alt = 3;
/** 
 * Number of inputs of the altitude model.
 *  - Motor thrust control (1)
 */
constexpr size_t Nu_alt = 1;
/**
 * Number of outputs of the altitude model.
 *  - Position (1)
 */
constexpr size_t Ny_alt = 1;

/** 
 * Number of states of the navigation model.
 *  - Velocity (2)
 *  - Position (2)
 */
constexpr size_t Nx_nav = 4;
/** 
 * Number of inputs of the navigation model.
 *  - None (0)
 */
constexpr size_t Nu_nav = 0;
/** 
 * Number of outputs of the navigation model.
 *  - Position (2)
 */
constexpr size_t Ny_nav = 2;

/** Number of states. */
constexpr size_t Nx = Nx_att + Nx_alt + Nx_nav;
/** Number of inputs. */
constexpr size_t Nu = Nu_att + Nu_alt + Nu_nav;
/** Number of outputs. */
constexpr size_t Ny = Ny_att + Ny_alt + Ny_nav;

namespace Attitude {

constexpr size_t Nx = Nx_att;
constexpr size_t Nu = Nu_att;
constexpr size_t Ny = Ny_att;

}  // namespace Attitude

namespace Altitude {

constexpr size_t Nx = Nx_alt;
constexpr size_t Nu = Nu_alt;
constexpr size_t Ny = Ny_alt;

}  // namespace Altitude
