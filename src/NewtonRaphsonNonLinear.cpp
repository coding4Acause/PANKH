#include "constants.h"
#include "VectorOperations.h"
#include "InfluenceMatrix.h"
#include "velocity.h"
#include "NewtonRaphsonNonLinear.h"


/*this function returns the residuals */
VectorXd newton_raphson(double dt, double t, double lwp, double theta_wp, VectorXd &vtotal_wp_cp, VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp, VectorXd &l, VectorXd &B_unsteady, VectorXd &gamma_unsteady, double gamma_old, VectorXd &gamma_bound, vector<double> &gamma_wake_strength, vector<double> &gamma_wake_x_location, vector<double> &gamma_wake_y_location, VectorXd &wake_panel_cp, VectorXd &wake_panel_normal, MatrixXd &A_unsteady, MatrixXd &unit_normal, MatrixXd &wake_panel_coordinates)
{
    VectorXd wake_influence(n - 1);
    Vector2d unit_gamma_wake(1, 1);
    MatrixXd panel_coeff_matrix_wake(2, 2);
    VectorXd normal_vector_panel_cp(2);
    VectorXd shed_vel(2), velocity_bound(2); // velocities due to [prev.shed,motion,bound vortices,total velocity]

    /*WAKE PANEL COORDINATES*/
    wake_panel_coordinates(0, 0) = x_pp(n - 1);
    wake_panel_coordinates(0, 1) = y_pp(n - 1);
    wake_panel_coordinates(1, 0) = x_pp(n - 1) + lwp * cos(theta_wp);
    wake_panel_coordinates(1, 1) = y_pp(n - 1) + lwp * sin(theta_wp);

    /* Once the position is guessed then calculate the control point coordinate of that wake panel */
    wake_panel_cp(0) = (wake_panel_coordinates(0, 0) + wake_panel_coordinates(1, 0)) / 2.0;
    wake_panel_cp(1) = (wake_panel_coordinates(0, 1) + wake_panel_coordinates(1, 1)) / 2.0;

    /* calculate the unit normal vector of the wake panel */
    wake_panel_normal(0) = -sin(theta_wp);
    wake_panel_normal(1) = cos(theta_wp);

    /*now we need to solve for AX=B, so first construct the Aunsteady matrix*/
    /* first calculate the influence of this wake panel on all the control points of the airfoil */
    for (int i = 0; i < n - 1; i++)
    {
        panel_coeff_matrix_wake = influence_matrix(wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), x_cp(i), y_cp(i));
        normal_vector_panel_cp(0) = unit_normal(i, 0);
        normal_vector_panel_cp(1) = unit_normal(i, 1);

        wake_influence(i) = dot((panel_coeff_matrix_wake * unit_gamma_wake), normal_vector_panel_cp);
        /* now we can COMPLETE the construction of the A_unsteady matrix by filling the influence of the wake panel on the control points of the airfoil's panels.*/
    }
    for (int i = 0; i < n - 1; i++) // filling the last column.....
    {
        A_unsteady(i, n) = wake_influence(i);
    }

    A_unsteady(n, n) = lwp; // from kelvins circulation theorem

    B_unsteady(n) = gamma_old; // from kelvins circulation theorem
    gamma_unsteady = A_unsteady.fullPivHouseholderQr().solve(B_unsteady);
    // cout << "B unsteady vector =" << endl;
    // cout << B_unsteady << endl;
    /* now finding the local flow velocities at the control points of the wake panel */

    for (int i = 0; i < n; i++)
    {
        gamma_bound(i) = gamma_unsteady(i);
    }

    /*finding the total velocity induced at the control point of the wake panel*/
    velocity_bound = velocity_bound_vortices(x_pp, y_pp, wake_panel_cp(0), wake_panel_cp(1), gamma_bound);
    if (t == 0)
    {
        shed_vel(0) = 0.0;
        shed_vel(1) = 0.0;
    }
    else
    {
        shed_vel(0) = 0.0;
        shed_vel(1) = 0.0;
        for (size_t j = 0; j < gamma_wake_strength.size(); j++) /* due to the previously shed vortices */
        {
            shed_vel = shed_vel + velocity_induced_due_to_discrete_vortex(gamma_wake_strength[j], gamma_wake_x_location[j], gamma_wake_y_location[j], wake_panel_cp(0), wake_panel_cp(1));
        }
    }

    vtotal_wp_cp = velocity_bound + shed_vel + freestream;
    VectorXd residuals(2);
    residuals(0) = lwp - magnitude(vtotal_wp_cp) * dt;
    residuals(1) = theta_wp - atan2(vtotal_wp_cp(1), vtotal_wp_cp(0));
    return residuals;
}
