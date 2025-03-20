#include "MotionParameters.h"


VectorXd h_instantaneous(double t, double omega)
{
    VectorXd plunge(3);
    double h = h1 * sin(omega * t + phi_h);
    plunge(0) = 0.0;
    plunge(1) = -h;
    plunge(2) = 0.0;
    return (plunge);
}
VectorXd h_dot_instantaneous(double t, double omega)
{
    VectorXd plunge_vel(3);
    double h_dot = h1 * omega * cos(omega * t + phi_h);
    plunge_vel(0) = 0.0;
    plunge_vel(1) = -h_dot;
    plunge_vel(2) = 0.0;
    return (plunge_vel);
}

double alpha_instantaneous(double t)
{
    double alpha;
    alpha = alpha0 + alpha1 * sin(omega * t + phi_alpha);
    return alpha;
}

double alpha_dot_instantaneous(double t)
{
    double alpha_dot;
    alpha_dot = alpha1 * omega * cos(omega * t + phi_alpha);
    return alpha_dot;
}
