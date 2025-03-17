#include "VectorOperations.h"
#include <cmath> //for sqrt command

VectorXd normalize_2d(VectorXd v)
{
    double mag = sqrt(v(0) * v(0) + v(1) * v(1));
    return v / mag;
}

double magnitude(VectorXd a)
{
    return sqrt(a(0) * a(0) + a(1) * a(1));
}

double dot(VectorXd a, VectorXd b)
{
    return a(0) * b(0) + a(1) * b(1);
}

VectorXd cross(VectorXd a, VectorXd b)
{
    VectorXd cross_product(3);
    cross_product(0) = a(1) * b(2) - a(2) * b(1);
    cross_product(1) = -(a(0) * b(2) - a(2) * b(0));
    cross_product(2) = a(0) * b(1) - a(1) * b(0);
    return cross_product;
}
