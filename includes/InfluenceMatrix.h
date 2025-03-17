#ifndef INFLUENCEMATRIX_H
#define INFLUENCEMATRIX_H

#include <Eigen/Dense>  // Eigen library for MatrixXd and VectorXd
#include <cmath>        // for sqrt, atan2, log
#include <iostream>    

using namespace Eigen;
using namespace std;

// Function declaration
MatrixXd influence_matrix(double point1_x, double point1_y, double point2_x, double point2_y, double desired_point_x, double desired_point_y);

#endif  // INFLUENCEMATRIX_H
