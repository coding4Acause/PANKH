#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include <fstream>
#include <cmath>

using namespace Eigen;
using namespace std;

// Function declarations
VectorXd body_fixed_frame_to_inertial_frame(double bff_x_coord, double bff_y_coord, double alpha, double t);
VectorXd velocity_at_surface_of_the_body_inertial_frame(double t, double point_x_coord, double point_y_coord);

#endif // KINEMATICS_H
