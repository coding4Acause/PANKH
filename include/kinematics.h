
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
 using namespace Eigen;
 
 /**
  * @brief Transforms coordinates from the body-fixed frame (BFF) to the inertial frame.
  *
  * The function - [body_fixed_frame_to_inertial_frame] applies a rotation and translation to convert a point from the
  * body-fixed coordinate system to the global inertial frame.
  *
  * @param bff_x_coord X-coordinate of the point in BFF.
  * @param bff_y_coord Y-coordinate of the point in BFF.
  * @param alpha Rotation angle (radians).
  * @param t Time instant.
  * @return VectorXd (2D) containing the transformed coordinates in the inertial frame.
  */
 VectorXd body_fixed_frame_to_inertial_frame(double bff_x_coord, double bff_y_coord, double alpha, double t);
 
 /**
  * @brief Computes velocity at a surface point of the body in the inertial frame.
  *
  * This function calculates the total velocity at any given point on the body
  * due to its motion, including contributions from freestream velocity, plunging,
  * and pitching motions.
  *
  * @param t Time instant.
  * @param point_x_coord X-coordinate of the surface point in the inertial frame.
  * @param point_y_coord Y-coordinate of the surface point in the inertial frame.
  * @return VectorXd (2D) representing the velocity components at the given surface point.
  */
 VectorXd velocity_at_surface_of_the_body_inertial_frame(double t, double point_x_coord, double point_y_coord);
 
 #endif // KINEMATICS_H