
#include "kinematics.h"
#include <cmath>

/*******some generalised functions which gives motion of the body.These functions will be required to convert from bff to inertial frame or vice-versa*/

VectorXd body_fixed_frame_to_inertial_frame(double h0,double h1,double phi_h,double x_pitch, double y_pitch,double alpha, double t,double omega,double bff_x_coord, double bff_y_coord)
{
    /*Generalsied case*/

    VectorXd bff(3);
    VectorXd rot_point_bff(3);

    /*the coordinates of the point[in bff] about which rotation is taking place */
    double xrot = x_pitch;
    double yrot = y_pitch; /* This is zero because the point of rotation lies in the x axis of bff*/
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

    pm = h_instantaneous(h0, h1, phi_h, t, omega);

    translation = pm;

    ef = (R * bff) + translation + rot_point_bff;

    VectorXd earth_frame(2); // vector in fixed frame or earth frame

    earth_frame(0) = ef(0);
    earth_frame(1) = ef(1);

    return (earth_frame);
}
// THIS FUNCTION GIVES THE TOTAL VELOCITY AT ANY POINT ON THE BODY OF THE GEOMETRY DUE TO ITS KINEMATICS
VectorXd velocity_at_surface_of_the_body_inertial_frame(double Qinf, double x_pitch, double y_pitch, double h0, double h1, double phi_h, double alpha0, double alpha1, double phi_alpha, double t, double omega, double point_x_coord, double point_y_coord) /*x_coord and ycoord are the point on the surface of the body in inertial frame*/
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
    plunge_vel = h_dot_instantaneous(h1, phi_h, t, omega);

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

    pm = h_instantaneous(h0, h1, phi_h, t, omega);

    rot_point_bff(0) = x_pitch;
    rot_point_bff(1) = y_pitch;
    rot_point_bff(2) = 0.0;

    xf = pm + rot_point_bff;

    rpos = xc - xf;

    double alpha_dot;
    alpha_dot = alpha_dot_instantaneous(alpha1, phi_alpha, t, omega);

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
