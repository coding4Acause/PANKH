#include "velocity.h"


// THIS FUNCTION CALCULATES THE VELOCITY INDUCED BY THE BOUND VORTICES(AIRFOIL VORTEX PANELS AT ANY RANDOM POINT IN THE FLOWFIELD)
VectorXd velocity_bound_vortices(int n,VectorXd &x_pp, VectorXd &y_pp, double x, double y, VectorXd G_bound)
{
    VectorXd VEL(2);
    VEL.setZero();

    MatrixXd P(2, 2);
    VectorXd vortex_strength(2);

    for (int i = 0; i < n - 1; i++)
    {
        P = influence_matrix(x_pp(i), y_pp(i), x_pp(i + 1), y_pp(i + 1), x, y);
        vortex_strength << G_bound(i), G_bound(i + 1);
        VEL += (P * vortex_strength);
    }

    return VEL;
}
// THIS FUNCTION CALCULATES THE VELOCITY INDUCED AT (des_point_x,des_point_y) BY A  POINT VORTEX OF STRENGTH GAMMA LOCATED AT (vor_point_x,vor_point_y)
VectorXd velocity_induced_due_to_discrete_vortex(double gamma, double vor_point_x, double vor_point_y, double des_point_x, double des_point_y)
{
    MatrixXd I(2, 2);
    VectorXd pos(2);
    VectorXd V(2);
    double r, delta_x, delta_y;

    delta_x = des_point_x - vor_point_x;
    delta_y = des_point_y - vor_point_y;

    r = sqrt(delta_x * delta_x + delta_y * delta_y);

    I << 0.0, 1.0,
        -1.0, 0.0;

    pos << delta_x, delta_y;

    V = (gamma / (2 * pi * r * r)) * I * pos;

    return V;
}
