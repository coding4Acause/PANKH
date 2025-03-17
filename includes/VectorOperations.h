#ifndef VECTOR_OPERATIONS_H //include guard
#define VECTOR_OPERATIONS_H 

#include <Eigen/Dense>

using Eigen::VectorXd; // ✅ Safer than using the full namespace(i.e using namespace Eigen)

// Function declarations
VectorXd normalize_2d(VectorXd v);
double magnitude(VectorXd a);
double dot(VectorXd a, VectorXd b);
VectorXd cross(VectorXd a, VectorXd b);

#endif //VECTOR_OPERATIONS_H
