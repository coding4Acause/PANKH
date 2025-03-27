#ifndef GEOMETRY_H //include guard
#define GEOMETRY_H 

#include <Eigen/Dense>
#include <cmath> 

using Eigen::VectorXd;
using Eigen::MatrixXd; 

// Function declarations

/**
 * @brief Function geometry computes the upper and lower surface coordinates of a NACA 4-digit series airfoil.
 * This function calculates the airfoil shape based on the given x/c position and returns the corresponding dimensionalized coordinates.
 *
 * @param x_c Non-dimensional chordwise location (x/c) ==> [0:1]
 * @return MatrixXd A 3x2 matrix containing the upper, lower, and mean camber coordinates.
 */
MatrixXd geometry(double x_c);

/**
 * @brief Generates the initial nodal coordinates using cosine clustering.
 *
 * @details This function discretizes the chord in a nonlinear manner to improve panel distribution.
 * near the leading and trailing edges. The term initial refers to the configuration of airfoil at time t=0 in inertial frame of reference which is also the origin of the
 * Therefore this function needs to be updated only once in the main function that too outside the time loop.
 *
 * @param x0 Reference to a VectorXd to store the x-coordinates of the nodes.
 * @param y0 Reference to a VectorXd to store the y-coordinates of the nodes.
 */
void nodal_coordinates_initial(VectorXd &x0, VectorXd &y0);

/**
 * @brief Computes the instantaneous nodal coordinates in the inertial frame.
 *
 * @details This function transforms the initially generated nodal coordinates to the inertial
 * frame based on the angle of attack and time. This needs to be called at every time instant to update the nodal coordinates of the panels on airfoil surface.
 *
 * @param x0 Reference to the x-coordinates of the nodes in the body-fixed frame.
 * @param y0 Reference to the y-coordinates of the nodes in the body-fixed frame.
 * @param x_pp Reference to a VectorXd to store the transformed x-coordinates.(Inertial frame)
 * @param y_pp Reference to a VectorXd to store the transformed y-coordinates.(Inertial frame)
 * @param alpha Angle of attack in radians.
 * @param t Time instant.
 */
void nodal_coordinates_instantaneous(VectorXd &x0, VectorXd &y0, VectorXd &x_pp, VectorXd &y_pp, double alpha, double t);

/**
 * @brief Computes the control points of the panels where the no penetration boundary condition is satisfied at every time instant.
 *
 * This function determines the midpoints of each panel, which serve as control points. It also needs to be updated at every time instant.
 * To calculate control points 
 *
 * @param x_pp Reference to the x-coordinates of the panel nodes.(which is ofcourse instantaneous for unsteady problem solved in the inertial frame of reference)
 * @param y_pp Reference to the y-coordinates of the panel nodes.(same as above)
 * @param x_cp Reference to a VectorXd to store the x-coordinates of the control points.
 * @param y_cp Reference to a VectorXd to store the y-coordinates of the control points.
 */
void controlpoints(VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp); // these control points are always created on the instantaneous panel coordinates.

/**
 * @brief Computes panel lengths and their x and y components.
 *
 * This function calculates the panel lengths and their respective x and y components
 * based on nodal coordinates.
 *
 * @param l_x Reference to a VectorXd to store the x-components of panel lengths.
 * @param l_y Reference to a VectorXd to store the y-components of panel lengths.
 * @param l Reference to a VectorXd to store the total panel lengths.
 * @param x_pp Reference to the x-coordinates of the panel nodes.
 * @param y_pp Reference to the y-coordinates of the panel nodes.
 */
void panel(VectorXd &l_x, VectorXd &l_y, VectorXd &l, VectorXd &x_pp, VectorXd &y_pp);

/**
 * @brief Computes the unit normal vectors for each panel.
 *
 * This function calculates the normal direction of each panel, which is crucial for
 * boundary condition enforcement in aerodynamic analysis.
 *
 * @param unit_normal Reference to a MatrixXd to store the unit normal vectors.
 * @param l_x Reference to the x-components of panel lengths.
 * @param l_y Reference to the y-components of panel lengths.
 */
void normal_function_for_panels(MatrixXd &unit_normal, VectorXd &l_x, VectorXd &l_y); /*here we are finding the unit normal of n-1 panels*/

/**
 * @brief Computes the unit tangent vectors for each panel.
 *
 * This function calculates the tangent direction of each panel, which is useful for
 * evaluating flow tangency conditions.
 *
 * @param unit_tangent Reference to a MatrixXd to store the unit tangent vectors.
 * @param l_x Reference to the x-components of panel lengths.
 * @param l_y Reference to the y-components of panel lengths.
 */
void tangent_function_for_panels(MatrixXd &unit_tangent, VectorXd &l_x, VectorXd &l_y); /*here we are finding the unit normal of n-1 panels*/

#endif //GEOMETRY_H
