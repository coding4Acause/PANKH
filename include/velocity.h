#ifndef VELOCITY_H
#define VELOCITY_H

#include <Eigen/Dense>
#include <cmath>
#include "InfluenceMatrix.h"
#include "constants.h"

using namespace Eigen;
using namespace std;


/**
 * @brief Computes the velocity induced at a point by bound vortices on an airfoil.
 *
 * @details This function calculates the velocity at a given point P(x, y) location 
 * due to the influence of bound vortices along the airfoil surface.
 * It uses the influence matrix to determine the contribution of each vortex panel.
 *
 * @param x_pp VectorXd containing x-coordinates of the panel endpoints.
 * @param y_pp VectorXd containing y-coordinates of the panel endpoints.
 * @param x Double value representing the x-coordinate of the evaluation point.
 * @param y Double value representing the y-coordinate of the evaluation point.
 * @param G_bound VectorXd containing the circulation strengths of the bound vortices.
 *
 * @return VectorXd A 2D velocity vector [Vx, Vy] induced at the given (x, y) location.
 */
VectorXd velocity_bound_vortices(int n, VectorXd &x_pp, VectorXd &y_pp, double x, double y, VectorXd G_bound);

/**
 * @brief Computes the velocity induced at a point by a single discrete vortex.
 *
 * @details This function calculates the velocity induced at a given (des_point_x, des_point_y) 
 * location due to a discrete vortex of strength `gamma` located at (vor_point_x, vor_point_y).
 * It follows the Biot-Savart law to determine the velocity components. This function will be required to compute the velocity
 * induced due to shed vortices in the wake.
 *
 * @param gamma Double value representing the circulation strength of the vortex.
 * @param vor_point_x Double value representing the x-coordinate of the vortex.
 * @param vor_point_y Double value representing the y-coordinate of the vortex.
 * @param des_point_x Double value representing the x-coordinate of the evaluation point.
 * @param des_point_y Double value representing the y-coordinate of the evaluation point.
 *
 * @return VectorXd A 2D velocity vector [Vx, Vy] induced at the given (des_point_x, des_point_y) location.
 */
VectorXd velocity_induced_due_to_discrete_vortex(double gamma, double vor_point_x, double vor_point_y, double des_point_x, double des_point_y);

#endif // VELOCITY_H

