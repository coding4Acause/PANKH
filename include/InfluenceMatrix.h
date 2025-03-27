/**
 * @file InfluenceMatrix.h
 * @brief Provides a function to compute the influence of a linearly varying vortex panel.
 *
 * This file contains function that compute the influence of a vortex panel of linearly varying strength
 * at an arbitrary point in the flowfield.
 *
 * @author [Rohit Chowdhury]
 * @date [Date]
 */

 #ifndef INFLUENCEMATRIX_H
 #define INFLUENCEMATRIX_H
 
 #include <Eigen/Dense>
 using namespace Eigen;
 
 /**
  * @brief Computes the influence matrix for a linearly varying vortex panel.
  *
  * Given the endpoints of a vortex panel and a desired point in the flowfield,
  * this function calculates the influence matrix, which is useful in solving the
  * panel method for aerodynamic simulations.
  *
  * @param point1_x X-coordinate of the first endpoint of the panel.
  * @param point1_y Y-coordinate of the first endpoint of the panel.
  * @param point2_x X-coordinate of the second endpoint of the panel.
  * @param point2_y Y-coordinate of the second endpoint of the panel.
  * @param desired_point_x X-coordinate of the target point in the flowfield.(point where the influence needs to be conmputed)
  * @param desired_point_y Y-coordinate of the target point in the flowfield.(point where the influence needs to be conmputed)
  * @return MatrixXd (2x2) representing the influence matrix corresponding to a particular panel.
  */
 MatrixXd influence_matrix(double point1_x, double point1_y, double point2_x, double point2_y, double desired_point_x, double desired_point_y);
 
 #endif // INFLUENCEMATRIX_H
 