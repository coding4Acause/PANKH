#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Eigen>
#include <fstream>
#include <cstdio>

using namespace Eigen;
using namespace std;

#define pi acos(-1)
#define n 101 // the number of nodes[# panels=n-1]

/*NACA 4 digit series airfoil parameters*/
const int ymc = 0;
const int xmc = 0;
const int tmax = 12;
const int trailing_edge_type = 2; //[1-closed,else-open]

double c = 0.2;     // chord length
double Qinf = 10.0; // magnitude of the freestream velocity in m/sec
double dt = (16.0 * c) / (n * Qinf);
double tou_max = 50.0; // non-dimensional time
double timemax = tou_max * c * 0.5 / Qinf;
const int nsteps = timemax / dt; // max number of iterations or time steps.............

int wakechoice = 0; //[0-free wake, 1-prescribed wake]

/*In this problem we are rotating the airfoil by instantaneous angle of attack keeping the freestream direction parallel to X dirn.[INERTIAL FRAME]*/
double Uinf = Qinf;
double Vinf = 0.0;
Vector2d freestream(Uinf, Vinf);

// Motion parameters [needed for pitch/plunge cases]
double k = 0.2;                                             // Reduced frequency
double omega = (2.0 * k * Qinf) / c;                        // Angular Frequency of oscillation [rad/sec]
double T = 2.0 * pi / omega;                                // Time Period of oscillation
double h1 = 0.25 * c;                                       // Plunge amplitude
double alpha1 = 15.0 * pi / 180.0 - atan2(2.0 * k * h1, c); // Pitch amplitude
double h0 = 0.0;                                            // Mean plunge
double alpha0 = 0.0 * pi / 180.0;                           // Mean pitch
double phi_h = 0.0;                                         // Plunge phase
double phi_alpha = (90.0 + phi_h) * pi / 180.0;             // Pitch phase

// For newtonRaphson
double tolerance = 1.e-5; // convergence criteria
double epsilon = 1.e-8;   // increment

VectorXd normalize_2d(VectorXd v)
{
    double mag;
    mag = sqrt(v(0) * v(0) + v(1) * v(1));
    v = v / mag;
    return v;
}
double magnitude(VectorXd a)
{
    double mag;
    mag = sqrt(a(0) * a(0) + a(1) * a(1));
    return mag;
}
double dot(VectorXd a, VectorXd b)
{
    double dot_product;
    dot_product = a(0) * b(0) + a(1) * b(1);
    return dot_product;
}
VectorXd cross(VectorXd a, VectorXd b)
{
    VectorXd cross_product(3);
    cross_product(0) = a(1) * b(2) - a(2) * b(1);
    cross_product(1) = -(a(0) * b(2) - a(2) * b(0));
    cross_product(2) = a(0) * b(1) - a(1) * b(0);
    return cross_product;
}

/*this function returns the upper and lower coordinates (dimensionalised) of a NACA 4 digit series airfoil for any x/c. */
MatrixXd geometry(double x_c)
{

    MatrixXd airfoil_points(3, 2);
    double p, q, tm, yc, der_yc, t;
    p = (ymc / 100.0); // p non dimensionalised ymc [p=ymc/c]
    q = (xmc * 0.1);   // q non dimensionalised xmc [q=xmc/c]
    tm = (tmax / 100.0);

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
    if (trailing_edge_type == 1.0) // 1 indicates open trailing edge
    {
        t = c * (tm * (2.969 * sqrt(x_c) - 1.260 * (x_c)-3.516 * pow((x_c), 2) + 2.843 * pow((x_c), 3.0) - 1.015 * (pow((x_c), 4.0))));
    }
    else
    {
        t = c * (tm * (2.980 * sqrt(x_c) - 1.320 * (x_c)-3.286 * pow((x_c), 2) + 2.441 * pow((x_c), 3.0) - 0.815 * (pow((x_c), 4.0))));
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
/* this function basically discretizes the chord in a nonlinear way [COSINE CLUSTERING]*/

void nodal_coordinates_initial(VectorXd &x0, VectorXd &y0)
{
    double theta;
    MatrixXd panel_points(3, 2);
    double x_nd, delta_theta;
    delta_theta = (2 * pi) / (n - 1);
    ofstream myfile;
    myfile.open("naca_geometry.dat");

    if (n % 2 == 0)
    { // even number of nodes
        delta_theta = (2.0 * pi) / (n - 1);

        for (int i = (n / 2); i >= 1; i--)
        {                                    // storing lower coordinates first
            theta = (i - 0.5) * delta_theta; // cosine clustering
            x_nd = 0.5 * (1 - cos(theta));
            panel_points = geometry(x_nd);
            x0(n / 2 - i) = panel_points(1, 0); // x_lower
            y0(n / 2 - i) = panel_points(1, 1); // y_lower
        }
        for (int i = 1; i <= (n / 2); i++)
        {                                    // storing upper coordinates
            theta = (i - 0.5) * delta_theta; // cosine clustering
            x_nd = 0.5 * (1 - cos(theta));
            panel_points = geometry(x_nd);
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
            panel_points = geometry(x_nd);
            x0(np2 - 1 - i) = panel_points(1, 0); // x_lower
            y0(np2 - 1 - i) = panel_points(1, 1); // y_lower
        }
        // middle point
        x_nd = 0.5 * (1 - cos(0.0));
        panel_points = geometry(x_nd);
        x0(np2) = panel_points(0, 0); // x_middle
        y0(np2) = panel_points(0, 1); // y_middle

        for (int i = 0; i < np2; i++)
        {                                  // upper coordinates
            theta = (i + 1) * delta_theta; // cosine clustering
            x_nd = 0.5 * (1 - cos(theta));
            panel_points = geometry(x_nd);
            x0(np2 + 1 + i) = panel_points(0, 0); // x_upper
            y0(np2 + 1 + i) = panel_points(0, 1); // y_upper
        }
    }

    for (int i = 0; i < n; i++)
    {
        myfile << x0(i) << "\t" << y0(i) << endl;
    }
}
/*******some generalised functions which gives motion of the body.These functions will be required to convert from bff to inertial frame or vice-versa*/

/* Plunging motion function gives the coordinate of the geometry at any time instance because of the plunging motion */
/* Plunging motion function gives the coordinate of the geometry at any time instance because of the plunging motion */
VectorXd h_instantaneous(double t, double omega)
{
    VectorXd plunge(3);
    double h = h1 * sin(omega * t + phi_h);
    plunge(0) = 0.0;
    // plunge(1) = -h;
    plunge(1) = 0.0;
    plunge(2) = 0.0;
    return (plunge);
}

VectorXd h_dot_instantaneous(double t, double omega)
{
    VectorXd plunge_vel(3);
    double h_dot = h1 * omega * cos(omega * t + phi_h);
    plunge_vel(0) = 0.0;
    // plunge_vel(1) = -h_dot;
    plunge_vel(1) = 0.0;
    plunge_vel(2) = 0.0;
    return (plunge_vel);
}
/* function which returns the alpha at a corresponding time */
double alpha_instantaneous(double t)
{
    double alpha;
    // alpha = alpha0 + alpha1 * sin(omega * t + phi_alpha);
    alpha = 5.0 * pi / 180.0;
    return alpha;
}
/* function which returns the alpha at a corresponding time */
double alpha_dot_instantaneous(double t)
{
    double alpha_dot;
    alpha_dot = alpha1 * omega * cos(omega * t + phi_alpha);
    return 0.0;
}

VectorXd body_fixed_frame_to_inertial_frame(double bff_x_coord, double bff_y_coord, double alpha, double t)
{
    /*Generalsied case*/

    VectorXd bff(3);
    VectorXd rot_point_bff(3);

    /*the coordinates of the point[in bff] about which rotation is taking place */
    double xrot = c / 3.0;
    double yrot = 0.0; /* This is zero because the point of rotation lies in the x axis of bff*/
    double zrot = 0.0;

    rot_point_bff(0) = xrot;
    rot_point_bff(1) = yrot;
    rot_point_bff(2) = zrot;

    bff(0) = bff_x_coord - xrot;
    bff(1) = bff_y_coord - yrot;
    bff(2) = 0.0;

    VectorXd ef(3); // vector in fixed frame or earth frame

    MatrixXd R(3, 3);

    R(0, 0) = cos(alpha);
    R(0, 1) = sin(alpha);
    R(0, 2) = 0.0;
    R(1, 0) = -sin(alpha);
    R(1, 1) = cos(alpha);
    R(1, 2) = 0.0;
    R(2, 0) = 0.0;
    R(2, 1) = 0.0;
    R(2, 2) = 1.0;

    VectorXd translation(3);

    VectorXd fm(3);
    VectorXd pm(3);

    pm = h_instantaneous(t, omega);

    translation = pm;

    ef = (R * bff) + translation + rot_point_bff;

    VectorXd earth_frame(2); // vector in fixed frame or earth frame

    earth_frame(0) = ef(0);
    earth_frame(1) = ef(1);

    return (earth_frame);
}
VectorXd velocity_at_surface_of_the_body_inertial_frame(double t, double point_x_coord, double point_y_coord) /*x_coord and ycoord are the point on the surface of the body in inertial frame*/
{
    VectorXd freestream(3);
    /*ASSUMING THAT THERE IS A ONCOMING FLOW PARALLEL TO THE X AXIS OF THE INERTIAL FRAME */
    double Uinf = Qinf, Vinf = 0.0, Zinf = 0.0;
    freestream(0) = Uinf;
    freestream(1) = Vinf;
    freestream(2) = Zinf;

    VectorXd forward_velocity(3);
    /* Assuming that the plate is STATIONARY AND PERFORMING PLUNGING AND PITCHING MOTION */
    double U0 = 0.0, V0 = 0.0, Z0 = 0.0;
    forward_velocity(0) = U0;
    forward_velocity(1) = V0;
    forward_velocity(2) = Z0;

    /* Obtain the plunging velocity corresponding to the given time */
    VectorXd plunge_vel(3);
    plunge_vel = h_dot_instantaneous(t, omega);

    /* Obtain the angular velocity corresponding to the given time at the desired point on the surface of the body */
    VectorXd rpos(3);
    VectorXd xc(3);
    VectorXd xf(3);
    VectorXd rot_point_bff(3);
    VectorXd cap_omega(3);

    xc(0) = point_x_coord;
    xc(1) = point_y_coord;
    xc(2) = 0.0;

    VectorXd fm(3);
    VectorXd pm(3);

    pm = h_instantaneous(t, omega);

    rot_point_bff(0) = c / 3.0;
    rot_point_bff(1) = 0.0;
    rot_point_bff(2) = 0.0;

    xf = pm + rot_point_bff;

    rpos = xc - xf;

    double alpha_dot;
    alpha_dot = alpha_dot_instantaneous(t);

    cap_omega(0) = 0.0;
    cap_omega(1) = 0.0;
    cap_omega(2) = alpha_dot;

    VectorXd omega_cross_r(3);
    omega_cross_r = cross(cap_omega, rpos);

    /***** NOW CALCULARTE THE RELATIVE FLOW VELOCITY ******/
    VectorXd umi(3); // i indicates at ith cp.

    umi = freestream - forward_velocity - plunge_vel + omega_cross_r;
    VectorXd um(2);
    um(0) = umi(0);
    um(1) = umi(1);

    return (um);
}

void nodal_coordinates_instantaneous(VectorXd &x0, VectorXd &y0, VectorXd &x_pp, VectorXd &y_pp, double alpha, double t)
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
        inert_temp = body_fixed_frame_to_inertial_frame(x0(i), y0(i), alpha, t);
        x_pp(i) = inert_temp(0);
        y_pp(i) = inert_temp(1);
        myfile2 << x_pp(i) << "\t" << y_pp(i) << endl;
    }
}

/*calculate the control points */
void controlpoints(VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp)
{
    ofstream myfile3;
    myfile3.open("control_points_instantaneous.dat");
    for (int j = 0; j < (n - 1); j++)
    {
        x_cp(j) = x_pp(j) - (x_pp(j) - x_pp(j + 1)) / 2;
        y_cp(j) = y_pp(j) - (y_pp(j) - y_pp(j + 1)) / 2;
        myfile3 << x_cp(j) << "\t" << y_cp(j) << endl;
    }
}

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

void Amatrix(MatrixXd &A, VectorXd &x_cp, VectorXd &y_cp, VectorXd &x_pp, VectorXd &y_pp)
{
    ofstream myfile3;
    myfile3.open("A_matrix_file.dat");

    MatrixXd pcm(2, 2); // panel coefficient matrix
    double x1, x2, y1, y2, dx, dy, li;

    for (int j = 0; j < n; j++) // j index is row rows
    {
        for (int i = 0; i < n; i++) // i index is for columns
        {
            A(j, i) = 0.0;
        }
    }
    for (int j = 0; j < n - 1; j++)
    {

        x1 = x_pp(j);
        x2 = x_pp(j + 1);
        y1 = y_pp(j);
        y2 = y_pp(j + 1);
        dx = x2 - x1;
        dy = y2 - y1;

        li = sqrt((dx * dx) + (dy * dy));

        for (int i = 0; i < n - 1; i++)
        {
            pcm = influence_matrix(x_pp(i), y_pp(i), x_pp(i + 1), y_pp(i + 1), x_cp(j), y_cp(j));
            A(j, i) = A(j, i) + dx / li * pcm(1, 0) - dy / li * pcm(0, 0);
            A(j, i + 1) = A(j, i + 1) + dx / li * pcm(1, 1) - dy / li * pcm(0, 1);
        }
    }
    A(n - 1, 0) = 1.0;
    A(n - 1, n - 1) = 1.0;
    myfile3 << A << endl;
}

void panel(VectorXd &l_x, VectorXd &l_y, VectorXd &l, VectorXd &x_pp, VectorXd &y_pp)
{
    for (int i = 0; i < n - 1; i++)
    {
        l_x(i) = x_pp(i + 1) - x_pp(i);
        l_y(i) = y_pp(i + 1) - y_pp(i);
        l(i) = sqrt(l_x(i) * l_x(i) + l_y(i) * l_y(i));
    }
}

void normal_function_for_panels(MatrixXd &unit_normal, VectorXd &l_x, VectorXd &l_y) /*here we are finding the unit normal of n-1 panels*/
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
void tangent_function_for_panels(MatrixXd &unit_tangent, VectorXd &l_x, VectorXd &l_y) /*here we are finding the unit normal of n-1 panels*/
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
VectorXd velocity_bound_vortices(VectorXd &x_pp, VectorXd &y_pp, double x, double y, VectorXd G_bound)
{
    VectorXd VEL(2);
    VEL(0) = 0.0;
    VEL(1) = 0.0;

    MatrixXd P(2, 2);

    VectorXd vortex_strength(2);
    // VectorXd freestream(2);

    double resultant_velocity, C_p;

    for (int i = 0; i < n - 1; i++)
    {
        P = influence_matrix(x_pp(i), y_pp(i), x_pp(i + 1), y_pp(i + 1), x, y);
        vortex_strength(0) = G_bound(i);
        vortex_strength(1) = G_bound(i + 1);
        VEL = VEL + (P * vortex_strength);
    }

    return (VEL);
}
VectorXd velocity_induced_due_to_discrete_vortex(double gamma, double vor_point_x, double vor_point_y, double des_point_x, double des_point_y)
{
    MatrixXd I(2, 2);
    VectorXd pos(2);
    VectorXd V(2);
    double r, delta_x, delta_y;
    double unit_vortex_strength = 1.0;

    delta_x = (des_point_x - vor_point_x);
    delta_y = (des_point_y - vor_point_y);

    r = sqrt((delta_x * delta_x) + (delta_y * delta_y));

    I(0, 0) = 0.0;
    I(0, 1) = 1.0;
    I(1, 0) = -1.0;
    I(1, 1) = 0.0;

    pos(0) = des_point_x - vor_point_x;
    pos(1) = des_point_y - vor_point_y;

    V = (gamma / (2 * pi * r * r)) * I * pos;

    return (V);
}
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

void plot_wake(FILE *gnuplotPipe, const vector<double> &gamma_wake_x_location, const vector<double> &gamma_wake_y_location, double wake_x1, double wake_y1, double wake_x2, double wake_y2, const VectorXd &x_pp, const VectorXd &y_pp)
{
    fprintf(gnuplotPipe, "set terminal x11\n");  // Ensure X11 is used
    fprintf(gnuplotPipe, "set size ratio -1\n"); // Equal axis scaling

    fprintf(gnuplotPipe, "plot '-' with points pt 7 ps 1.0 lc rgb 'red', '-' with lines lw 3 lc rgb 'blue'\n");

    // Plot wake vortices (Red points)
    fprintf(gnuplotPipe, "%lf %lf\n", wake_x1, wake_y1);
    fprintf(gnuplotPipe, "%lf %lf\n", wake_x2, wake_y2);
    for (size_t k = 0; k < gamma_wake_x_location.size(); k++)
    {
        fprintf(gnuplotPipe, "%lf %lf\n", gamma_wake_x_location[k], gamma_wake_y_location[k]);
    }
    fprintf(gnuplotPipe, "e\n"); // End of first dataset (wake vortices)

    // Plot airfoil (Blue line)
    for (int i = 0; i < x_pp.size(); i++)
    {
        fprintf(gnuplotPipe, "%lf %lf\n", x_pp(i), y_pp(i));
    }
    fprintf(gnuplotPipe, "e\n"); // End of second dataset (airfoil)
    fflush(gnuplotPipe);         // Update plot immediately
}

void plot_ClvsTime(FILE *gnuplotPipe1, const vector<double> &xdata, const vector<double> &ydata)
{
    fprintf(gnuplotPipe1, "set grid\n");
    fprintf(gnuplotPipe1, "set title 'Evolution of lift coefficient with time'\n");
    fprintf(gnuplotPipe1, "set xlabel'2*Qinf*t/c'\n");
    fprintf(gnuplotPipe1, "set ylabel'Cl(t)/Cl_{steady}'\n");

    fprintf(gnuplotPipe1, "set yrange [0:1]\n");
    fprintf(gnuplotPipe1, "set xrange [0:%lf]\n", tou_max);

    fprintf(gnuplotPipe1, "set terminal x11\n"); // Ensure X11 is used
    fprintf(gnuplotPipe1, "plot '-' with lines lw 3 lc rgb 'green'\n");

    for (int i = 0; i < xdata.size(); i++)
    {
        fprintf(gnuplotPipe1, "%lf %lf\n", xdata[i], ydata[i]);
    }
    fprintf(gnuplotPipe1, "e\n"); // End of second dataset (airfoil)
    fflush(gnuplotPipe1);         // Update plot immediately
}

int main()
{
    double t;
    double alpha_ins;
    // cout << freestream << endl;
    vector<double> xdata;
    vector<double> ydata;
    VectorXd x0(n), y0(n), x_pp(n), y_pp(n), x_cp(n - 1), y_cp(n - 1);
    VectorXd l(n - 1), l_x(n - 1), l_y(n - 1), normal_vector_panel_cp(2);
    MatrixXd unit_normal(n - 1, 2), unit_tangent(n - 1, 2);
    MatrixXd A(n, n);
    VectorXd B_steady(n), gamma_steady(n);

    double offset = 1.e-4;
    /* FIRST OBTAIN THE STEADY STATE SOLUTION */
    nodal_coordinates_initial(x0, y0); // initial coordinates of airfoil
    alpha_ins = alpha_instantaneous(0);
    nodal_coordinates_instantaneous(x0, y0, x_pp, y_pp, alpha_ins, 0); // airfoil got rotated by alpha
    controlpoints(x_pp, y_pp, x_cp, y_cp);
    Amatrix(A, x_cp, y_cp, x_pp, y_pp);
    panel(l_x, l_y, l, x_pp, y_pp);
    normal_function_for_panels(unit_normal, l_x, l_y);
    tangent_function_for_panels(unit_tangent, l_x, l_y);

    for (int i = 0; i < n - 1; i++)
    {
        normal_vector_panel_cp(0) = unit_normal(i, 0);
        normal_vector_panel_cp(1) = unit_normal(i, 1);

        B_steady(i) = -dot(freestream, normal_vector_panel_cp);
    }

    B_steady(n - 1) = 0.0; // kutta condition
    gamma_steady = A.fullPivHouseholderQr().solve(B_steady);

    // cout << B_steady << endl;
    // cout << gamma_steady << endl;

    Vector2d vel;
    VectorXd cp_steady(n - 1);

    double V_STEADY;

    ofstream cp_steady_file;
    cp_steady_file.open("output_files/steady state pressure distribution");

    for (int i = 0; i < n - 1; i++)
    {
        vel = velocity_bound_vortices(x_pp, y_pp, x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset, gamma_steady);
        V_STEADY = magnitude(vel + freestream);
        cp_steady(i) = 1.0 - (V_STEADY * V_STEADY) / (Qinf * Qinf);
        cp_steady_file << x_cp(i) << "\t" << cp_steady(i) << endl;
    }
    // cout << cp_steady << endl;
    //  cout << "*****************" << endl;

    /* calculation of lift and drag */
    double cl = 0.0;
    double cn_tilda_steady, ca_tilda_steady; // CL AND CDP RESP. SINCE THE FREESTREAM IS NOT ROTATED,AIRFOIL IS ROTATED AND DIRN. OF LIFT AND DRAG IS WRT TO FREESTREAM

    VectorXd n_capi(2);
    VectorXd i_cap(2);
    VectorXd j_cap(2);

    i_cap(0) = 1.0;
    i_cap(1) = 0.0;

    j_cap(0) = 0.0;
    j_cap(1) = 1.0;

    cn_tilda_steady = 0.0;
    ca_tilda_steady = 0.0;

    double cl_tilda_kj = 0.0;

    for (int i = 0; i < n - 1; i++) // scanning the control points...........
    {
        n_capi(0) = unit_normal(i, 0);
        n_capi(1) = unit_normal(i, 1);
        cn_tilda_steady = cn_tilda_steady - (1.0 / c) * cp_steady(i) * l(i) * dot(n_capi, j_cap);
        ca_tilda_steady = ca_tilda_steady - (1.0 / c) * cp_steady(i) * l(i) * dot(n_capi, i_cap);
        cl_tilda_kj = cl_tilda_kj + (l(i) * (gamma_steady(i) + gamma_steady(i + 1))) / (c * Qinf);
        // cout << cp_steady(i) << endl;
    }

    cout << "steady state lift= " << "\t" << "pressure integration=" << "\t" << cn_tilda_steady << "\t" << "kutta-Joukowsky=" << "\t" << cl_tilda_kj << endl;

    /******************************************************************************************************************************************************** */
    //                                             STEADY STATE CALCULATIONS ENDS HERE. NOW WE WILL MOVE TO UNSTEADY                                          //
    /******************************************************************************************************************************************************** */

    MatrixXd A_unsteady(n + 1, n + 1);
    VectorXd B_unsteady(n + 1);
    VectorXd gamma_unsteady(n + 1);
    VectorXd gamma_bound(n);

    MatrixXd wake_panel_coordinates(2, 2), panel_coeff_matrix_wake(2, 2);
    VectorXd wake_panel_cp(2), wake_panel_normal(2);
    VectorXd wake_influence(n - 1);
    Vector2d unit_gamma_wake(1, 1);
    vector<double> gamma_wake_strength;   // create a vector which will store the strengths of the shed vortices
    vector<double> gamma_wake_x_location; // create vectors which will store the x and y coordinates of the shed vortices[remember that these vectors needs to be updated at the end of each time step]
    vector<double> gamma_wake_y_location;
    VectorXd phi_old(n - 1), phi_new(n - 1), cp(n - 1); // [phi old and phi new required for dphi/dt and cp vector stores pressure coefficient at all the control points.]

    VectorXd shed_vel(2), flow_vel(2);
    double lwp, theta_wp;
    double lwp_new, theta_wp_new;
    double gamma_wp = 0.0;
    Vector2d rhs_vector, length_and_angle;
    Matrix2d jacobian;
    double delta_lwp, delta_theta_wp;

    ofstream myfile_load_cal, wake_last_time_step, wake_panel, wakefile, motionfile, pressurefile, gammafile, potentialfile, amatrixfile, bvectorfile, airfoilnormalfile;
    myfile_load_cal.open("cl_cd_sudden_acc_n=101.dat");
    wake_last_time_step.open("wake at last time step.dat");
    wake_panel.open("output_files/wake panel at last time step.dat");

    /*initial guesses for lwp and theta_wp*/
    lwp = Qinf * dt;
    theta_wp = 0.0;
    double gamma_t_minus_dt;
    double gamma_old = 0.0;
    VectorXd vtotal_wp_cp(2);

    double cn_tilda, ca_tilda;

    FILE *gnuplotPipe = popen("gnuplot -persist", "w");
    if (!gnuplotPipe)
    {
        cerr << "Error: Could not open GNUplot.\n";
        return 1;
    }
    fprintf(gnuplotPipe, "set grid\n");
    fprintf(gnuplotPipe, "set title 'In-Situ Wake Vortex Visualization'\n");

    FILE *gnuplotPipe1 = popen("gnuplot -persist", "w");
    if (!gnuplotPipe)
    {
        cerr << "Error: Could not open GNUplot.\n";
        return 1;
    }

    double prcntgtme;

    for (int iter = 0; iter <= nsteps; iter++)
    {
        prcntgtme = iter / (double)(nsteps) * 100.0;
        cout << "percentage time completed =" << "\t" << prcntgtme << endl;
        t = iter * dt;

        string name = "output_files/vortex_shedding/wake_";
        name += to_string(iter);
        name += ".dat";
        wakefile.open(name.c_str());

        string name1 = "output_files/vortex_shedding/motion_";
        name1 += to_string(iter);
        name1 += ".dat";
        motionfile.open(name1.c_str());
        for (int i = 0; i < n; i++)
        {
            motionfile << x_pp(i) << "\t" << y_pp(i) << endl;
        }
        // string name2 = "output_files/pressure_file/t_";
        // name2 += to_string(iter);
        // name2 += ".dat";
        // pressurefile.open(name2.c_str());

        // string name3 = "output_files/gamma_vector_file/t_";
        // name3 += to_string(iter);
        // name3 += ".dat";
        // gammafile.open(name3.c_str());

        // string name4 = "output_files/potential_file/t_";
        // name4 += to_string(iter);
        // name4 += ".dat";
        // potentialfile.open(name4.c_str());

        // string name5 = "output_files/a_matrix_file/t_";
        // name5 += to_string(iter);
        // name5 += ".dat";
        // amatrixfile.open(name5.c_str());

        // string name6 = "output_files/b_vector_file/t_";
        // name6 += to_string(iter);
        // name6 += ".dat";
        // bvectorfile.open(name6.c_str());

        // string name7 = "output_files/airfoil_normal_file/t_";
        // name7 += to_string(iter);
        // name7 += ".dat";
        // airfoilnormalfile.open(name7.c_str());

        for (int i = 0; i < n - 1; i++)
        {
            airfoilnormalfile << unit_normal(i, 0) << "\t" << unit_normal(i, 1) << endl;
        }
        cout << "time period= " << T << endl;
        cout << "iteration /time instance =" << iter << endl;
        /*self induced portion and kutta conditon...*/
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                A_unsteady(i, j) = A(i, j);
            }
        }
        A_unsteady(n - 1, n) = 1.0; // kutta condition
        /* Kelvins Circulation [DGAMMA/DT=0.0] */
        A_unsteady(n, 0) = l(0) * 0.5;
        for (int i = 1; i < n - 1; i++)
        {
            A_unsteady(n, i) = (l(i - 1) + l(i - 1 + 1)) * 0.5;
        }
        A_unsteady(n, n - 1) = l(n - 2) * 0.5;

        /* the last column of the A_unsteady matrix will be filled inside the newtonraphson function after calculating the influence of the wake control point..*/

        /*construct the rhs or the B vector */
        // cout << "flowvelocity " << endl;
        for (int i = 0; i < n - 1; i++)
        {
            normal_vector_panel_cp(0) = unit_normal(i, 0);
            normal_vector_panel_cp(1) = unit_normal(i, 1);
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
                    shed_vel = shed_vel + velocity_induced_due_to_discrete_vortex(gamma_wake_strength[j], gamma_wake_x_location[j], gamma_wake_y_location[j], x_cp(i), y_cp(i));
                }
            }
            flow_vel = velocity_at_surface_of_the_body_inertial_frame(t, x_cp(i), y_cp(i));
            // cout << magnitude(flow_vel) << endl;

            B_unsteady(i) = -dot(shed_vel + flow_vel, normal_vector_panel_cp);
        }
        B_unsteady(n - 1) = 0.0; /* [kutta condition] */

        VectorXd residuals(2);
        VectorXd residuals_plus(2);
        int conv_iter = 0;
        double convergence;

        cout << "initial guess for the present time step = " << "length = " << lwp << "\t" << "angle = " << theta_wp << endl;

        do
        {
            cout << "convergence iteration= " << conv_iter << endl;
            /*first step is to fill the first column of the Jacobian matrix...*/
            residuals = newton_raphson(dt, t, lwp, theta_wp, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);
            jacobian(0, 0) = 0.0;
            jacobian(0, 1) = 0.0;
            jacobian(1, 0) = 0.0;
            jacobian(1, 1) = 0.0;
            residuals_plus = newton_raphson(dt, t, lwp + epsilon, theta_wp, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);

            jacobian(0, 0) = (residuals_plus(0) - residuals(0)) / epsilon;
            jacobian(1, 0) = (residuals_plus(1) - residuals(1)) / epsilon;

            residuals_plus = newton_raphson(dt, t, lwp, theta_wp + epsilon, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);

            jacobian(0, 1) = (residuals_plus(0) - residuals(0)) / epsilon;
            jacobian(1, 1) = (residuals_plus(1) - residuals(1)) / epsilon;
            cout << "JACOBIAN" << "\t" << endl
                 << jacobian << endl;
            /* fill the coefficient matrix or the jacobian matrix */
            rhs_vector(0) = -residuals(0);
            rhs_vector(1) = -residuals(1);
            length_and_angle = jacobian.fullPivHouseholderQr().solve(rhs_vector);

            delta_lwp = length_and_angle(0);
            delta_theta_wp = length_and_angle(1);
            convergence = magnitude(length_and_angle);
            cout << "convergence=" << convergence << endl;

            lwp_new = lwp + delta_lwp;
            theta_wp_new = theta_wp + delta_theta_wp;

            lwp = lwp_new;
            theta_wp = theta_wp_new;

            conv_iter++;
        } while ((convergence) > tolerance);
        residuals = newton_raphson(dt, t, lwp, theta_wp, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);
        gamma_wp = gamma_unsteady(n);
        cout << "CONVERGED VALUES =" << "\t" << "uwp= " << vtotal_wp_cp(0) << "\t" << "vwp=" << vtotal_wp_cp(1) << "\t" << "gamma_wp=" << gamma_wp << "\t" << "lwp=" << lwp << "\t" << "theta_wp=" << theta_wp << endl;
        cout << "--------------------------------------------------------------------------------------------------------------------- " << endl;
        gammafile << gamma_unsteady << endl;
        amatrixfile << A_unsteady << endl;
        bvectorfile << B_unsteady << endl;

        double gamma_t_minus_dt = 0.0;
        for (int i = 0; i < n - 1; i++)
        {
            gamma_t_minus_dt += (gamma_bound(i) + gamma_bound(i + 1)) * l(i) * 0.5;
        }
        gamma_old = gamma_t_minus_dt;

        /* Once the Iterative Procedure to calculate the length and orientation of the wake panel has converged,we can now calculate the aerodynamic loads ......*/

        /* For that first compute the pressure distribution on the surface of the airfoil and then integrate that pressure to obtain the lift and drag forces ...*/
        int z = 200;                               // number of panels..
        VectorXd x_forward_stag_streamline(z + 1); // z+1 is the number of nodes in forward stagnation streamline.
        VectorXd y_forward_stag_streamline(z + 1);
        VectorXd xcp_forward_stag_streamline(z);
        VectorXd ycp_forward_stag_streamline(z);

        /* now divide this stagnation line into z number of points by sine clustering such that clustering is towards the leading edge */
        double lz = (10.0 * c);

        for (int i = 0; i < z + 1; i++)
        {
            x_forward_stag_streamline(i) = (1.0 - sin(i * 0.5 * pi / z)) * (-lz) + x_pp(n / 2 - 1);
        }
        // cout << x_pp << endl;
        //   cout <<x_forward_stag_streamline << endl;

        for (int i = 0; i < z + 1; i++)
        {
            y_forward_stag_streamline(i) = (1.0 - sin(i * 0.5 * pi / z)) * (0) + y_pp(n / 2 - 1);
        }
        /*cal. the control points */

        /*cal. phi_le _at the current time step..*/
        ofstream fsl;
        fsl.open("check_streamline_usptream.dat");
        for (int i = 0; i < z; i++)
        {
            xcp_forward_stag_streamline(i) = (x_forward_stag_streamline(i) + x_forward_stag_streamline(i + 1)) / 2.0;
            ycp_forward_stag_streamline(i) = (y_forward_stag_streamline(i) + y_forward_stag_streamline(i + 1)) / 2.0;
        }

        for (int i = 0; i < z + 1; i++)
        {
            fsl << x_forward_stag_streamline(i) << "\t" << y_forward_stag_streamline(i) << endl;
        }

        /* calculate phi at LE [phi_le(t_k)]*/

        VectorXd wake_panel_strength(2);
        wake_panel_strength(0) = gamma_wp;
        wake_panel_strength(1) = gamma_wp;

        VectorXd vifsl_rw(2); // vifsl_rw stands for velocity induced at forward stagnation streamline due to recently shed wake panel
        VectorXd vifsl_b(2);  // vifsl_b  stands for velocity induced at forward stagnation streamline due to bound vortices
        VectorXd vifsl_pw(2); // vifsl_pw stands for velocity induced at forward stagnation streamline due to prev. shed wake vortices

        double tang_vel;
        double phi_le;

        phi_le = 0.0;
        VectorXd unit_tangent_vector(2);

        for (int i = 0; i < z; i++) // accessing all the control points of the forward stagnation streamline[APPROXIMATED]
        {
            panel_coeff_matrix_wake = influence_matrix(wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), xcp_forward_stag_streamline(i), ycp_forward_stag_streamline(i));
            vifsl_rw = panel_coeff_matrix_wake * wake_panel_strength;
            vifsl_b = velocity_bound_vortices(x_pp, y_pp, xcp_forward_stag_streamline(i), ycp_forward_stag_streamline(i), gamma_bound);
            if (t == 0)
            {
                vifsl_pw(0) = 0.0;
                vifsl_pw(1) = 0.0;
            }
            else
            {
                vifsl_pw(0) = 0.0;
                vifsl_pw(1) = 0.0;
                for (size_t j = 0; j < gamma_wake_strength.size(); j++) /* due to the previously shed vortices */
                {
                    vifsl_pw = vifsl_pw + velocity_induced_due_to_discrete_vortex(gamma_wake_strength[j], gamma_wake_x_location[j], gamma_wake_y_location[j], xcp_forward_stag_streamline(i), ycp_forward_stag_streamline(i));
                }
            }

            tang_vel = vifsl_rw(0) + vifsl_b(0) + vifsl_pw(0);
            // cout <<"tangential velcoity"<< endl;
            // cout << fabs(x_forward_stag_streamline((i + 1)) - x_forward_stag_streamline((i))) << endl;
            phi_le = phi_le + tang_vel * fabs(x_forward_stag_streamline((i + 1)) - x_forward_stag_streamline((i)));
            // phi_le =0.0;
        }
        VectorXd phi_airfoil_nodes(n);
        VectorXd viacp_rw(2); // viacp stands for velocity induced at airfoil control point.
        VectorXd viacp_b(2);
        VectorXd viacp_pw(2);
        // cout << endl <<  A_unsteady << endl;

        /*** now calculate the values of phi for the current time step at all the control points on the AIRFOIL surface ***/

        for (int j = 0; j < n; j++) // scanning all the nodes.
        {
            if (j >= 0 && j < (n + 1) / 2 - 1) // lower surface
            {
                double addition = 0.0;
                for (int i = j; i < (n + 1) / 2 - 1; i++) // scanning the control points..
                {
                    panel_coeff_matrix_wake = influence_matrix(wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset);
                    viacp_rw = panel_coeff_matrix_wake * wake_panel_strength;
                    viacp_b = velocity_bound_vortices(x_pp, y_pp, x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset, gamma_bound);
                    if (t == 0)
                    {
                        viacp_pw(0) = 0.0;
                        viacp_pw(1) = 0.0;
                    }
                    else
                    {
                        viacp_pw(0) = 0.0;
                        viacp_pw(1) = 0.0;
                        for (size_t k = 0; k < gamma_wake_strength.size(); k++) /* due to the previously shed vortices */
                        {
                            viacp_pw = viacp_pw + velocity_induced_due_to_discrete_vortex(gamma_wake_strength[k], gamma_wake_x_location[k], gamma_wake_y_location[k], x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset);
                        }
                    }
                    unit_tangent_vector(0) = unit_tangent(i, 0); // tangent vector at ith control point
                    unit_tangent_vector(1) = unit_tangent(i, 1);
                    tang_vel = dot(unit_tangent_vector, (viacp_rw + viacp_b + viacp_pw));

                    // tang_vel = magnitude(viacp_rw + viacp_b + viacp_pw);
                    addition = addition + (tang_vel * l(i));
                }
                phi_airfoil_nodes(j) = phi_le - addition;
            }

            if ((j > (n + 1) / 2 - 1) && j < (n)) // upper panels
            {
                double addition = 0.0;
                for (int i = (n + 1) / 2 - 1; i <= j - 1; i++) // scanning the control points
                {
                    panel_coeff_matrix_wake = influence_matrix(wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset);
                    viacp_rw = panel_coeff_matrix_wake * wake_panel_strength;
                    viacp_b = velocity_bound_vortices(x_pp, y_pp, x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset, gamma_bound);
                    if (t == 0)
                    {
                        viacp_pw(0) = 0.0;
                        viacp_pw(1) = 0.0;
                    }
                    else
                    {
                        viacp_pw(0) = 0.0;
                        viacp_pw(1) = 0.0;
                        for (size_t k = 0; k < gamma_wake_strength.size(); k++) /* due to the previously shed vortices */
                        {
                            viacp_pw = viacp_pw + velocity_induced_due_to_discrete_vortex(gamma_wake_strength[k], gamma_wake_x_location[k], gamma_wake_y_location[k], x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset);
                        }
                    }
                    unit_tangent_vector(0) = unit_tangent(i, 0); // tangent vector at ith control point
                    unit_tangent_vector(1) = unit_tangent(i, 1);
                    tang_vel = dot(unit_tangent_vector, (viacp_rw + viacp_b + viacp_pw));
                    addition = addition + (tang_vel * l(i));
                }
                phi_airfoil_nodes(j) = phi_le + addition;
            }
            if (j == (n + 1) / 2 - 1)
            {
                phi_airfoil_nodes(j) = phi_le;
            }
        }

        VectorXd phi_airfoil_cps(n - 1);
        VectorXd dphi_dt(n - 1);
        for (int i = 0; i < n - 1; i++) // accessing the control points.
        {
            phi_airfoil_cps(i) = (phi_airfoil_nodes(i + 1) + phi_airfoil_nodes(i)) / 2.0;
        }
        if (iter == 0)
        {
            phi_old = phi_airfoil_cps;
        }
        else
        {
            phi_new = phi_airfoil_cps;
        }
        for (int i = 0; i < n - 1; i++)
        {
            potentialfile << x_cp(i) << "\t" << phi_airfoil_cps(i) << endl;
        }

        /* calculation of the pressure coefficients at all the control points.. */
        VectorXd vi(2);

        double V;
        for (int i = 0; i < n - 1; i++)
        {
            panel_coeff_matrix_wake = influence_matrix(wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset);
            viacp_rw = panel_coeff_matrix_wake * wake_panel_strength;
            viacp_b = velocity_bound_vortices(x_pp, y_pp, x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset, gamma_bound);
            if (iter == 0)
            {
                viacp_pw(0) = 0.0;
                viacp_pw(1) = 0.0;
                dphi_dt(i) = 0.0;
            }
            else
            {
                viacp_pw(0) = 0.0;
                viacp_pw(1) = 0.0;
                for (size_t k = 0; k < gamma_wake_strength.size(); k++) /*........... due to the previously shed vortices.......*/
                {
                    viacp_pw = viacp_pw + velocity_induced_due_to_discrete_vortex(gamma_wake_strength[k], gamma_wake_x_location[k], gamma_wake_y_location[k], x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset);
                }
                dphi_dt(i) = ((phi_new(i) - phi_old(i))) / dt;
            }

            flow_vel = velocity_at_surface_of_the_body_inertial_frame(t, x_cp(i), y_cp(i));
            vi = viacp_rw + viacp_b + viacp_pw + flow_vel;
            V = magnitude(vi);
            cp(i) = 1.0 - (V * V) / (Qinf * Qinf) - (2.0 / (Qinf * Qinf)) * (dphi_dt(i));
            // cp(i) = 1.0 - (V * V) / (Qinf * Qinf); [steady]
        }
        for (int i = 0; i < n - 1; i++)
        {
            pressurefile << x_cp(i) << "\t" << cp(i) << endl;
        }

        if (iter > 0)
        {
            phi_old = phi_new;
        }
        /* calculation of lift and drag */

        cn_tilda = 0.0;
        ca_tilda = 0.0;

        for (int i = 0; i < n - 1; i++) // scanning the control points...........
        {
            n_capi(0) = unit_normal(i, 0);
            n_capi(1) = unit_normal(i, 1);
            cn_tilda = cn_tilda - (1.0 / c) * cp(i) * l(i) * dot(n_capi, j_cap);
            ca_tilda = ca_tilda - (1.0 / c) * cp(i) * l(i) * dot(n_capi, i_cap);
        }

        myfile_load_cal << 2.0 * t * Qinf / c << "\t" << (cn_tilda / cl_tilda_kj) << "\t" << ca_tilda << endl; // uncomment this for sudden acceleration case.

        xdata.push_back(2.0 * t * Qinf / c);
        ydata.push_back((cn_tilda / cl_tilda_kj));
        // myfile_load_cal << t/T << "\t" << cn_tilda << "\t" << ca_tilda << endl;

        /* now we need to convect the panel as a discrete vortex for the next time step*/
        /* so we need to find the local flow velocity or the velocity induced at the control point of the panels.....*/

        for (int i = 0; i < 2; i++)
        {
            wakefile << wake_panel_coordinates(i, 0) << "\t" << wake_panel_coordinates(i, 1) << endl;
        }
        for (size_t k = 0; k < gamma_wake_strength.size(); k++) /* due to the previously shed vortices */
        {
            wakefile << gamma_wake_x_location[k] << "\t" << gamma_wake_y_location[k] << endl;
        }
        plot_wake(gnuplotPipe, gamma_wake_x_location, gamma_wake_y_location, wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), x_pp, y_pp);
        plot_ClvsTime(gnuplotPipe1, xdata, ydata);

        /***  next task is to propagate the wake point vortices ***/

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  WAKE ROLLUP                                                                                                                                                                                                    //
        //  The vortices which were in the wake in this time step will move to a some different location in the next time step.They will travel with the local velocity.So before
        //  going  to the next time step,we should update their positions for next time step. Because calculations involved in the next time step should be from their updated positions.                                  //
        //  IN THIS PROBLEM WE ASSUMED THE CASE OF FREE WAKE MODELLING[where the wake vortices move with the local flow velocity(vel. induced at a  wake point due to other shed vortices,bound vortices and freestream.)] //
        //                                                                                                                                                                                                                 //
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        int size = gamma_wake_x_location.size();
        VectorXd gamma_wake_x_new_location(size);
        VectorXd gamma_wake_y_new_location(size);
        Vector2d velocity;
        Vector2d vel_wake_point;

        if (iter > 0)
        {
            for (int j = 0; j < size; j++)
            {
                shed_vel(0) = 0.0;
                shed_vel(1) = 0.0;
                for (int i = 0; i < size; i++) /* effect of other wake vortices on jth wake point... */
                {
                    if (i != j)
                    {
                        shed_vel = shed_vel + velocity_induced_due_to_discrete_vortex(gamma_wake_strength[i], gamma_wake_x_location[i], gamma_wake_y_location[i], gamma_wake_x_location[j], gamma_wake_y_location[j]);
                    }
                }
                // cout << shed_vel << endl;
                /*velocity induced at jth wake point due to bound vortices*/
                velocity = velocity_bound_vortices(x_pp, y_pp, gamma_wake_x_location[j], gamma_wake_y_location[j], gamma_bound);
                /*********** velocity induced at jth wake point due to wake panel ********/
                panel_coeff_matrix_wake = influence_matrix(wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), gamma_wake_x_location[j], gamma_wake_y_location[j]);
                vel_wake_point = panel_coeff_matrix_wake * wake_panel_strength;
                if (wakechoice == 0)
                {
                    /******** free wake ********/
                    gamma_wake_x_new_location[j] = gamma_wake_x_location[j] + (freestream(0) + shed_vel(0) + velocity(0) + vel_wake_point(0)) * dt;
                    gamma_wake_y_new_location[j] = gamma_wake_y_location[j] + (freestream(1) + shed_vel(1) + velocity(1) + vel_wake_point(1)) * dt;
                }
                else if (wakechoice == 1)
                {
                    /******** prescribed wake ********/
                    gamma_wake_x_new_location[j] = gamma_wake_x_location[j] + (freestream(0))*dt;
                    gamma_wake_y_new_location[j] = gamma_wake_y_location[j] + (freestream(1))*dt;
                }
            }
            for (int j = 0; j < size; j++)
            {
                //  cout << setw(10) << fixed << setprecision(5) << j << setw(10) << fixed << setprecision(5) << gamma_wake_x_location[j] << setw(10) << fixed << setprecision(5) << gamma_wake_y_location[j] << setw(10) << fixed << setprecision(5) << gamma_wake_x_new_location[j] << setw(10) << fixed << setprecision(5) << gamma_wake_y_new_location[j] << endl;
                gamma_wake_x_location[j] = gamma_wake_x_new_location[j];
                gamma_wake_y_location[j] = gamma_wake_y_new_location[j];
                // cout << gamma_wake_x_location[j] << "\t" << gamma_wake_y_location[j] << endl;
            }
        }
        gamma_wake_strength.push_back(gamma_wp * lwp);
        gamma_wake_x_location.push_back(wake_panel_cp(0) + vtotal_wp_cp(0) * dt); /* basically in gamma_wake_x_location and gamma_wake_y_location, we have updated that where the panel shed in the current time step will lie[as a discrete vortex] in the next time step */
        gamma_wake_y_location.push_back(wake_panel_cp(1) + vtotal_wp_cp(1) * dt);
        size = gamma_wake_x_new_location.size();

        wakefile.close();
        motionfile.close();
        pressurefile.close();
        gammafile.close();
        potentialfile.close();
        amatrixfile.close();
        bvectorfile.close();
        airfoilnormalfile.close();
    }
    pclose(gnuplotPipe);

    /*plotting the flowfield at the last time step.*/
    for (size_t j = 0; j < gamma_wake_strength.size(); j++)
    {
        wake_last_time_step << gamma_wake_x_location[j] << "\t" << gamma_wake_y_location[j] << endl;
    }
    return 0;
}
