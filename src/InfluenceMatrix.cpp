
#include "InfluenceMatrix.h"
#include "constants.h"
// THIS FUNCTION RETURNS THE INFLUENCE OF A LINEARLY STRENGTH VORTEX PANEL AT A RANDOM POINT IN THE FLOWFIELD.
MatrixXd influence_matrix(double point1_x, double point1_y, double point2_x, double point2_y, double desired_point_x, double desired_point_y)
{
    MatrixXd P1(2, 2);
    MatrixXd P2(2, 2);
    MatrixXd P(2, 2);
    VectorXd vec(2);
    VectorXd LPC(2);

    double dx, dy, li;
    double phi, psi;
    double eta, geta;

    dx = (point2_x - point1_x);
    dy = (point2_y - point1_y);

    li = sqrt(dx * dx + dy * dy);

    P1(0, 0) = dx;
    P1(0, 1) = dy;
    P1(1, 0) = -dy;
    P1(1, 1) = dx;

    vec(0) = desired_point_x - point1_x;
    vec(1) = desired_point_y - point1_y;

    LPC = (P1 * vec) / li;
    geta = LPC(0);
    eta = LPC(1);
    phi = atan2((eta * li), ((eta * eta) + (geta * geta) - (geta * (li))));
    psi = 0.5 * log(((geta * geta) + (eta * eta)) / (((geta - li) * (geta - li)) + (eta * eta)));

    P1(0, 0) = dx;
    P1(0, 1) = -dy;
    P1(1, 0) = dy;
    P1(1, 1) = dx;

    P2(0, 0) = (li - geta) * phi + (eta * psi);
    P2(0, 1) = (geta * phi) - (eta * psi);
    P2(1, 0) = (eta * phi - (li - geta) * psi - li);
    P2(1, 1) = ((-eta * phi) - (geta * psi) + li);

    P = (P1 * P2) / (2.0 * pi * li * li);

    return P;
}