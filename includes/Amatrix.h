#ifndef AMATRIX_H
#define AMATRIX_H

#include <Eigen/Dense>
#include <fstream>  // For file writing
#include <iostream>
#include "InfluenceMatrix.h"

using namespace Eigen;
using namespace std;

// Function declaration
void Amatrix(MatrixXd &A, VectorXd &x_cp, VectorXd &y_cp, VectorXd &x_pp, VectorXd &y_pp);

#endif  // AMATRIX_H
