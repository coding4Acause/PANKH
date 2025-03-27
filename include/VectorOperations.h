#ifndef VECTOR_OPERATIONS_H // Include guard
#define VECTOR_OPERATIONS_H 

#include <Eigen/Dense>

using Eigen::VectorXd;

/**
 * @brief Normalizes a 2D vector.
 * 
 * @param v The input 2D vector.
 * @return Normalized 2D vector.
 */
VectorXd normalize_2d(VectorXd v);

/**
 * @brief Computes the magnitude of a vector.
 * 
 * @param a The input vector.
 * @return Magnitude (length) of the vector.
 */
double magnitude(VectorXd a);

/**
 * @brief Computes the dot product of two vectors.
 * 
 * @param a First vector.
 * @param b Second vector.
 * @return Dot product of a and b.
 */
double dot(VectorXd a, VectorXd b);

/**
 * @brief Computes the cross product of two 3D vectors.
 * 
 * @param a First vector.
 * @param b Second vector.
 * @return Cross product of a and b.
 */
VectorXd cross(VectorXd a, VectorXd b);

#endif // VECTOR_OPERATIONS_H
