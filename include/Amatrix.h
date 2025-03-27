#ifndef AMATRIX_H
#define AMATRIX_H

#include <Eigen/Dense>
#include <fstream>  // For file writing
#include <iostream>
#include "InfluenceMatrix.h"

using namespace Eigen;
using namespace std;

/**
 * @brief Computes the A matrix for the vortex panel method.
 * 
 * @details This function calculates the entire coefficient matrix with the help of panel coefficient or panel influence matrix based on the 
 * control points and panel points of the airfoil. It also writes the computed matrix to a file.
 * 
 * @param A Reference to the matrix where computed values will be stored.
 * @param x_cp Vector of x-coordinates of control points.
 * @param y_cp Vector of y-coordinates of control points.
 * @param x_pp Vector of x-coordinates of panel points.
 * @param y_pp Vector of y-coordinates of panel points.
 * 
 */
// Function declaration
void Amatrix(MatrixXd &A, VectorXd &x_cp, VectorXd &y_cp, VectorXd &x_pp, VectorXd &y_pp);

#endif  // AMATRIX_H
