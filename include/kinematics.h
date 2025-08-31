
/**
 * @file kinematics.h
 * @brief Contains functions for transforming coordinates and computing velocity in inertial and body-fixed frames.
 *
 * These functions are essential for converting between the body-fixed frame (BFF) and the inertial (earth) frame,
 * as well as for calculating velocity at any point on the body due to its kinematics.
 * 
 * @author [Rohit Chowdhury]
 */

 #ifndef KINEMATICS_H
 #define KINEMATICS_H
 
 #include <Eigen/Dense>
 #include "MotionParameters.h"
 #include "VectorOperations.h"
 
 using namespace Eigen;

 
/**
 * @brief Transforms coordinates from the body-fixed frame to the inertial frame.
 *
 * Converts a point’s coordinates from the body-fixed frame (BFF) to the inertial frame, accounting
 * for heaving and pitching motions about a specified rotation point. The transformation includes
 * rotation (via pitch angle) and translation (via heaving motion).
 *
 * @param h0 Heaving offset (meters).
 * @param h1 Heaving amplitude (meters).
 * @param phi_h Heaving phase angle (radians).
 * @param x_pitch x-coordinate of the pitch axis in the body-fixed frame (meters).
 * @param y_pitch y-coordinate of the pitch axis in the body-fixed frame (meters).
 * @param alpha Pitch angle (radians).
 * @param t Current time (seconds).
 * @param omega Angular frequency (radians/second).
 * @param bff_x_coord x-coordinate of the point in the body-fixed frame (meters).
 * @param bff_y_coord y-coordinate of the point in the body-fixed frame (meters).
 * @return VectorXd A 2D vector [x, y] representing the point’s coordinates in the inertial frame.
 * @see h_instantaneous
 */
VectorXd body_fixed_frame_to_inertial_frame(double h0, double h1, double phi_h, double x_pitch, double y_pitch, double alpha, double t, double omega, double bff_x_coord, double bff_y_coord);

/**
 * @brief Computes the total velocity at a point on the body’s surface in the inertial frame.
 *
 * Calculates the velocity at a specified point on the body’s surface, considering freestream flow,
 * plunging motion, and pitching motion about a rotation point. The freestream flow is assumed to be
 * parallel to the x-axis of the inertial frame.
 *
 * @param Qinf Freestream flow speed (meters/second).
 * @param x_pitch x-coordinate of the pitch axis in the body-fixed frame (meters).
 * @param y_pitch y-coordinate of the pitch axis in the body-fixed frame (meters).
 * @param h0 Heaving offset (meters).
 * @param h1 Heaving amplitude (meters).
 * @param phi_h Heaving phase angle (radians).
 * @param alpha0 Pitch angle offset (radians).
 * @param alpha1 Pitch angle amplitude (radians).
 * @param phi_alpha Pitch phase angle (radians).
 * @param t Current time (seconds).
 * @param omega Angular frequency (radians/second).
 * @param point_x_coord x-coordinate of the point in the inertial frame (meters).
 * @param point_y_coord y-coordinate of the point in the inertial frame (meters).
 * @return VectorXd A 2D vector [u, v] representing the total velocity in the inertial frame.
 * @see h_dot_instantaneous, alpha_dot_instantaneous, cross
 */
VectorXd velocity_at_surface_of_the_body_inertial_frame(double Qinf, double x_pitch, double y_pitch, double h0, double h1, double phi_h, double alpha0, double alpha1, double phi_alpha, double t, double omega, double point_x_coord, double point_y_coord);

#endif // KINEMATICS_H