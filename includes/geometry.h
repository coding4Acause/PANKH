#ifndef GEOMETRY_H //include guard
#define GEOMETRY_H 

#include <Eigen/Dense>
#include <cmath> 

using Eigen::VectorXd;
using Eigen::MatrixXd; 

// Function declarations
MatrixXd geometry(double x_c);
void nodal_coordinates_initial(VectorXd &x0, VectorXd &y0);
void nodal_coordinates_instantaneous(VectorXd &x0, VectorXd &y0, VectorXd &x_pp, VectorXd &y_pp, double alpha, double t);
void controlpoints(VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp); //these control points are always created on the instantaneous panel coordinates.
void panel(VectorXd &l_x, VectorXd &l_y, VectorXd &l, VectorXd &x_pp, VectorXd &y_pp);
void normal_function_for_panels(MatrixXd &unit_normal, VectorXd &l_x, VectorXd &l_y); /*here we are finding the unit normal of n-1 panels*/
void tangent_function_for_panels(MatrixXd &unit_tangent, VectorXd &l_x, VectorXd &l_y); /*here we are finding the unit normal of n-1 panels*/

#endif //GEOMETRY_H
