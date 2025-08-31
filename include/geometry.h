#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "kinematics.h"
#include "VectorOperations.h"
#include "constants.h"

using namespace Eigen;
using namespace std;

/**
 * @brief Computes the coordinates of a NACA 4-digit series airfoil at a given chord position.
 *
 * This function calculates the upper, lower, and camber line coordinates for a NACA 4-digit airfoil
 * based on the non-dimensional chord position and airfoil parameters. Supports both symmetric and
 * cambered airfoils, with open or closed trailing edges.
 *
 * @param x_c Non-dimensional chord position (x/c), must be in [0, 1].
 * @param c Chord length (meters).
 * @param q Non-dimensional position of maximum camber (fraction of chord, 0 < q < 1).
 * @param p Maximum camber (fraction of chord).
 * @param trailing_edge_type Trailing edge type: 1 for open, 0 for closed.
 * @param t_m Thickness-to-chord ratio (e.g., 0.12 for 12% thickness).
 * @return MatrixXd A 3x2 matrix containing [x_upper, y_upper; x_lower, y_lower; x_camber, y_camber].
 * @throws std::invalid_argument If x_c is outside [0, 1] or other invalid parameters.
 */
MatrixXd geometry(double x_c, double c, double q, double p, int trailing_edge_type, double t_m);

/**
 * @brief Discretizes the airfoil into nodes using cosine clustering.
 *
 * Generates initial nodal coordinates for a NACA 4-digit airfoil using cosine clustering to
 * concentrate nodes near the leading and trailing edges. Stores results in x0 and y0 vectors
 * and writes them to "output_files/_time=0.dat".
 *
 * @param n Number of nodes (even or odd).
 * @param c Chord length (meters).
 * @param q Non-dimensional position of maximum camber (fraction of chord, 0 < q < 1).
 * @param p Maximum camber (fraction of chord).
 * @param trailing_edge_type Trailing edge type: 1 for open, 0 for closed.
 * @param t_m Thickness-to-chord ratio.
 * @param x0 Output vector for x-coordinates (size n).
 * @param y0 Output vector for y-coordinates (size n).
 * @throws std::runtime_error If file output fails.
 * @throws std::invalid_argument If n < 2 or other invalid parameters.
 */
void nodal_coordinates_initial(int n, double c, double q, double p, int trailing_edge_type, double t_m, VectorXd &x0, VectorXd &y0);

/**
 * @brief Transforms airfoil nodal coordinates to the instantaneous inertial frame.
 *
 * Converts body-fixed coordinates to inertial coordinates accounting for heaving and pitching
 * motions. Writes transformed coordinates to "output_files/panel_points_instantaneous.dat".
 *
 * @param n Number of nodes.
 * @param h0 Heaving amplitude (meters).
 * @param h1 Heaving offset (meters).
 * @param phi_h Heaving phase angle (radians).
 * @param x_pitch Pitch axis x-coordinate (meters).
 * @param y_pitch Pitch axis y-coordinate (meters).
 * @param alpha Pitch angle (radians).
 * @param t Current time (seconds).
 * @param omega Angular frequency of pitching (radians/second).
 * @param x0 Input vector of body-fixed x-coordinates (size n).
 * @param y0 Input vector of body-fixed y-coordinates (size n).
 * @param x_pp Output vector for inertial x-coordinates (size n).
 * @param y_pp Output vector for inertial y-coordinates (size n).
 * @throws std::runtime_error If file output fails.
 * @throws std::invalid_argument If vectors are incorrectly sized.
 * @see body_fixed_frame_to_inertial_frame
 */
void nodal_coordinates_instantaneous(int n, double h0, double h1, double phi_h, double x_pitch, double y_pitch, double alpha, double t, double omega, VectorXd &x0, VectorXd &y0, VectorXd &x_pp, VectorXd &y_pp);

/**
 * @brief Computes control points as midpoints of airfoil panels.
 *
 * Calculates the midpoints of panels formed by consecutive nodes. Writes results to
 * "output_files/control_points_instantaneous.dat".
 *
 * @param n Number of nodes.
 * @param x_pp Input vector of panel x-coordinates (size n).
 * @param y_pp Input vector of panel y-coordinates (size n).
 * @param x_cp Output vector for control point x-coordinates (size n-1).
 * @param y_cp Output vector for control point y-coordinates (size n-1).
 * @throws std::runtime_error If file output fails.
 * @throws std::invalid_argument If vectors are incorrectly sized.
 */
void controlpoints(int n, VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp);

/**
 * @brief Computes panel lengths and their x, y components.
 *
 * Calculates the x and y differences and Euclidean length of each panel formed by consecutive nodes.
 *
 * @param n Number of nodes.
 * @param l_x Output vector for x-component of panel vectors (size n-1).
 * @param l_y Output vector for y-component of panel vectors (size n-1).
 * @param l Output vector for panel lengths (size n-1).
 * @param x_pp Input vector of panel x-coordinates (size n).
 * @param y_pp Input vector of panel y-coordinates (size n).
 * @throws std::invalid_argument If vectors are incorrectly sized or n < 2.
 */
void panel(int n, VectorXd &l_x, VectorXd &l_y, VectorXd &l, VectorXd &x_pp, VectorXd &y_pp);

/**
 * @brief Computes unit normal vectors for airfoil panels.
 *
 * Calculates the unit normal vectors (rotated 90° counterclockwise) for each panel.
 *
 * @param n Number of nodes.
 * @param unit_normal Output matrix of unit normal vectors (size (n-1) x 2).
 * @param l_x Input vector of panel x-components (size n-1).
 * @param l_y Input vector of panel y-components (size n-1).
 * @throws std::invalid_argument If vectors or matrix are incorrectly sized or n < 2.
 * @throws std::runtime_error If panel length is zero (causing normalization failure).
 * @see normalize_2d
 */
void normal_function_for_panels(int n, MatrixXd &unit_normal, VectorXd &l_x, VectorXd &l_y);

/**
 * @brief Computes unit tangent vectors for airfoil panels.
 *
 * Calculates the unit tangent vectors for each panel.
 *
 * @param n Number of nodes.
 * @param unit_tangent Output matrix of unit tangent vectors (size (n-1) x 2).
 * @param l_x Input vector of panel x-components (size n-1).
 * @param l_y Input vector of panel y-components (size n-1).
 * @throws std::invalid_argument If vectors or matrix are incorrectly sized or n < 2.
 * @throws std::runtime_error If panel length is zero (causing normalization failure).
 * @see normalize_2d
 */
void tangent_function_for_panels(int n, MatrixXd &unit_tangent, VectorXd &l_x, VectorXd &l_y);

#endif // GEOMETRY_H