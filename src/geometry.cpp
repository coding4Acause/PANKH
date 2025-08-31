#include "geometry.h"
#include <fstream>

/* this function returns the upper and lower coordinates (dimensionalised) of a NACA 4 digit series airfoil for any x/c. */
MatrixXd geometry(double x_c, double c, double q, double p, int trailing_edge_type, double t_m)
{

    MatrixXd airfoil_points(3, 2);

    double yc, der_yc, t;

    if (x_c <= q)
    {
        yc = (c * p * (2.0 * (x_c / q) - (x_c / q) * (x_c / q)));
        der_yc = (c * (2.0 * p / q) * (1.0 - (x_c / q)));
    }
    else
    {
        yc = (p * c * (2.0 * (1 - x_c) / (1 - q) - pow(((1 - x_c) / (1 - q)), 2.0)));
        der_yc = c * ((2.0 * p / (1.0 - q)) * (((1.0 - x_c) / (1.0 - q)) - 1.0));
    }
    if (trailing_edge_type == 1) // 1 indicates open trailing edge
    {
        t = c * (t_m * (2.969 * sqrt(x_c) - 1.260 * (x_c)-3.516 * pow((x_c), 2) + 2.843 * pow((x_c), 3.0) - 1.015 * (pow((x_c), 4.0))));
    }
    else
    {
        t = c * (t_m * (2.980 * sqrt(x_c) - 1.320 * (x_c)-3.286 * pow((x_c), 2) + 2.441 * pow((x_c), 3.0) - 0.815 * (pow((x_c), 4.0))));
    }
    if (p == 0.0) //[SYMMETRIC AIRFOIL]
    {
        airfoil_points(0, 0) = x_c * c;  // x_upper
        airfoil_points(0, 1) = t / 2.0;  // y_upper
        airfoil_points(1, 0) = x_c * c;  // x_lower
        airfoil_points(1, 1) = -t / 2.0; // y_lower
        airfoil_points(2, 0) = x_c * c;
        airfoil_points(2, 1) = 0.0;
    }
    else
    {
        airfoil_points(0, 0) = x_c * c - t * der_yc / (2 * (sqrt(1 + (der_yc * der_yc)))); // x_upper
        airfoil_points(0, 1) = yc + t / (2 * (sqrt(1 + (der_yc * der_yc))));               // y_upper
        airfoil_points(1, 0) = x_c * c + t * der_yc / (2 * (sqrt(1 + (der_yc * der_yc)))); // x_lower
        airfoil_points(1, 1) = yc - t / (2 * (sqrt(1 + (der_yc * der_yc))));               // y_lower
        airfoil_points(2, 0) = x_c * c;
        airfoil_points(2, 1) = yc;
    }
    return airfoil_points;
}

void nodal_coordinates_initial(int n,double c, double q, double p, int trailing_edge_type, double t_m, VectorXd &x0, VectorXd &y0) // this function basically discretizes the chord in a nonlinear way [COSINE CLUSTERING]
{
    double theta;
    MatrixXd panel_points(3, 2);
    double x_nd, delta_theta;
    delta_theta = (2 * pi) / (n - 1);
    ofstream myfile;
    myfile.open("output_files/_time=0.dat");

    if (n % 2 == 0)
    { // even number of nodes
        delta_theta = (2.0 * pi) / (n - 1);

        for (int i = (n / 2); i >= 1; i--)
        {                                    // storing lower coordinates first
            theta = (i - 0.5) * delta_theta; // cosine clustering
            x_nd = 0.5 * (1 - cos(theta));
            panel_points = geometry(x_nd, c, q, p, trailing_edge_type, t_m);
            x0(n / 2 - i) = panel_points(1, 0); // x_lower
            y0(n / 2 - i) = panel_points(1, 1); // y_lower
        }
        for (int i = 1; i <= (n / 2); i++)
        {                                    // storing upper coordinates
            theta = (i - 0.5) * delta_theta; // cosine clustering
            x_nd = 0.5 * (1 - cos(theta));
            panel_points = geometry(x_nd, c, q, p, trailing_edge_type, t_m);
            x0(n / 2 + i - 1) = panel_points(0, 0); // x_upper
            y0(n / 2 + i - 1) = panel_points(0, 1); // y_upper
        }
    }
    else
    {
        // odd number of nodes
        int np2 = n / 2;
        delta_theta = pi / static_cast<double>(np2);

        for (int i = 0; i < np2; i++)
        {                                  // lower coordinates
            theta = (i + 1) * delta_theta; // cosine clustering
            x_nd = 0.5 * (1 - cos(theta));
            panel_points = geometry(x_nd, c, q, p, trailing_edge_type, t_m);
            x0(np2 - 1 - i) = panel_points(1, 0); // x_lower
            y0(np2 - 1 - i) = panel_points(1, 1); // y_lower
        }
        // middle point
        x_nd = 0.5 * (1 - cos(0.0));
        panel_points = geometry(x_nd, c, q, p, trailing_edge_type, t_m);
        x0(np2) = panel_points(0, 0); // x_middle
        y0(np2) = panel_points(0, 1); // y_middle

        for (int i = 0; i < np2; i++)
        {                                  // upper coordinates
            theta = (i + 1) * delta_theta; // cosine clustering
            x_nd = 0.5 * (1 - cos(theta));
            panel_points = geometry(x_nd, c, q, p, trailing_edge_type, t_m);
            x0(np2 + 1 + i) = panel_points(0, 0); // x_upper
            y0(np2 + 1 + i) = panel_points(0, 1); // y_upper
        }
    }

    for (int i = 0; i < n; i++)
    {
        myfile << x0(i) << "\t" << y0(i) << endl;
    }
}

void nodal_coordinates_instantaneous(int n,double h0,double h1,double phi_h,double x_pitch, double y_pitch,double alpha, double t,double omega,VectorXd &x0, VectorXd &y0, VectorXd &x_pp, VectorXd &y_pp)
{
    ofstream myfile2;
    myfile2.open("output_files/panel_points_instantaneous.dat");
    double theta;
    MatrixXd panel_points(3, 2);
    double x_nd, delta_theta;
    delta_theta = (2.0 * pi) / (n - 1);
    VectorXd inert_temp(2);

    for (int i = 0; i < n; i++)
    {    
        inert_temp = body_fixed_frame_to_inertial_frame(h0,h1,phi_h,x_pitch,y_pitch,alpha,t,omega,x0(i), y0(i));
        x_pp(i) = inert_temp(0);
        y_pp(i) = inert_temp(1);
        myfile2 << x_pp(i) << "\t" << y_pp(i) << endl;
    }
}

/*calculate the control points */
void controlpoints(int n, VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp)
{
    ofstream myfile3;
    myfile3.open("output_files/control_points_instantaneous.dat");
    for (int j = 0; j < (n - 1); j++)
    {
        x_cp(j) = x_pp(j) - (x_pp(j) - x_pp(j + 1)) / 2;
        y_cp(j) = y_pp(j) - (y_pp(j) - y_pp(j + 1)) / 2;
        myfile3 << x_cp(j) << "\t" << y_cp(j) << endl;
    }
}

void panel(int n, VectorXd &l_x, VectorXd &l_y, VectorXd &l, VectorXd &x_pp, VectorXd &y_pp)
{
    for (int i = 0; i < n - 1; i++)
    {
        l_x(i) = x_pp(i + 1) - x_pp(i);
        l_y(i) = y_pp(i + 1) - y_pp(i);
        l(i) = sqrt(l_x(i) * l_x(i) + l_y(i) * l_y(i));
    }
}

void normal_function_for_panels(int n, MatrixXd &unit_normal, VectorXd &l_x, VectorXd &l_y) /*here we are finding the unit normal of n-1 panels*/
{
    VectorXd panel_length(2);
    VectorXd unit_normal_vector(2);
    for (int i = 0; i < n - 1; i++)
    {
        panel_length(0) = -l_y(i);
        panel_length(1) = l_x(i);
        unit_normal_vector = normalize_2d(panel_length);
        unit_normal(i, 0) = unit_normal_vector(0);
        unit_normal(i, 1) = unit_normal_vector(1);
    }
}
void tangent_function_for_panels(int n, MatrixXd &unit_tangent, VectorXd &l_x, VectorXd &l_y) /*here we are finding the unit normal of n-1 panels*/
{
    VectorXd panel_length(2);
    VectorXd unit_tangent_vector(2);
    for (int i = 0; i < n - 1; i++)
    {
        panel_length(0) = l_x(i);
        panel_length(1) = l_y(i);
        unit_tangent_vector = normalize_2d(panel_length);
        unit_tangent(i, 0) = unit_tangent_vector(0);
        unit_tangent(i, 1) = unit_tangent_vector(1);
    }
}