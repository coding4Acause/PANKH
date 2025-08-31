#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include "json.hpp"
#include "VectorOperations.h"
#include "geometry.h"
#include "MotionParameters.h"
#include "kinematics.h"
#include "InfluenceMatrix.h"
#include "Amatrix.h"
#include "NewtonRaphsonNonLinear.h"
#include "velocity.h"
#include "gnuplot.h"
#include "constants.h"

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <input_file.json>" << endl;
        return 1;
    }

    string filename = argv[1];
    ifstream inputFile(filename);
    if (!inputFile.is_open())
    {
        cerr << "Error: Cannot open " << filename << endl;
        return 1;
    }

    json input;
    inputFile >> input;

    // Extract geometry
    int n = input["geometry"]["n"];
    double c = input["geometry"]["c"];
    double ymc = input["geometry"]["ymc"];
    double xmc = input["geometry"]["xmc"];
    double tmax = input["geometry"]["tmax"];
    int trailing_edge_type = input["geometry"]["trailing_edge_type"];

    // Derived parameters
    double p = ymc / 100.0;
    double q = xmc / 10.0;
    double t_m = tmax / 100.0;

    // Extract flow
    double rho = input["flow"]["rho"];
    double mu = input["flow"]["mu"];
    double Re = input["flow"]["Re"];
    double Qinf = input["flow"]["Qinf"].is_null() ? (Re * mu / (rho * c)) : input["flow"]["Qinf"].get<double>();
    double Vinf = input["flow"]["Vinf"];
    VectorXd freestream(2); // size must be specified
    freestream(0) = Qinf;
    freestream(1) = Vinf;

    // Extract motion
    double k = input["motion"]["k"];
    double h1 = input["motion"]["h1"].is_null() ? 0.25 * c : input["motion"]["h1"].get<double>();
    double h0 = input["motion"]["h0"];
    double alpha0 = input["motion"]["alpha0"].get<double>() * DEG2RAD;
    double phi_h = input["motion"]["phi_h"].get<double>() * DEG2RAD;

    // alpha1: either use JSON input (if provided) or derive it
    double alpha1 = input["motion"]["alpha1"].is_null()
                        ? (15.0 * DEG2RAD - atan2(2.0 * k * h1, c)) // derived
                        : input["motion"]["alpha1"].get<double>() * DEG2RAD;      // provided

    // Pitch axis: default to mid-chord if not specified
    double x_pitch = input["motion"]["x_pitch"].is_null() ? c / 3.0 : input["motion"]["x_pitch"].get<double>();
    double y_pitch = input["motion"]["y_pitch"].is_null() ? 0.0 : input["motion"]["y_pitch"].get<double>();


    // Extract simulation
    int wake = input["simulation"]["wake"];
    double tolerance = input["simulation"]["tolerance"];
    double epsilon = input["simulation"]["epsilon"];
    int ncycles = input["simulation"]["ncycles"];
    int nsteps = input["simulation"]["nsteps"];
    int z = input["simulation"]["z"];
    
    double phi_alpha=(90.0*phi_h)*DEG2RAD;
    double omega =(2.0*k*Qinf)/c;
    double T = 2.0*pi/omega;
    double dt = T / nsteps;    // time increment
    double time_max = ncycles * T; // maximum time
    double t;
    double alpha_ins;

    // cout << freestream << endl;

    VectorXd x0(n), y0(n), x_pp(n), y_pp(n), x_cp(n - 1), y_cp(n - 1);
    VectorXd l(n - 1), l_x(n - 1), l_y(n - 1), normal_vector_panel_cp(2);
    MatrixXd unit_normal(n - 1, 2), unit_tangent(n - 1, 2);
    MatrixXd A(n, n);

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
    VectorXd phi_old(n - 1), phi_new(n - 1), cp(n - 1); //[phi old and phi new required for dphi/dt and cp vector stores pressure coefficient at all the control points.]

    VectorXd shed_vel(2), flow_vel(2);
    double lwp, theta_wp;
    double lwp_new, theta_wp_new;
    double gamma_wp = 0.0;
    double offset = 1.e-4;

    nodal_coordinates_initial(n, c, q, p, trailing_edge_type, t_m, x0, y0);

    Vector2d rhs_vector, length_and_angle;
    Matrix2d jacobian;
    double delta_lwp, delta_theta_wp;

    double cl = 0.0;
    double cn_tilda, ca_tilda;
    double Ct = 0.0;

    VectorXd n_capi(2);
    VectorXd i_cap(2);
    VectorXd j_cap(2);

    i_cap(0) = 1.0;
    i_cap(1) = 0.0;

    j_cap(0) = 0.0;
    j_cap(1) = 1.0;

    ofstream myfile_load_cal, wake_last_time_step, wake_panel, wakefile, motionfile, pressurefile, gammafile, potentialfile, amatrixfile, bvectorfile, airfoilnormalfile;
    myfile_load_cal.open("output_files/cl_cd_pitch_plunge_k=1.2_n=101.dat");
    wake_last_time_step.open("output_files/wake at last time step.dat");
    wake_panel.open("output_files/wake panel at last time step.dat");

    /*initial guesses for lwp and theta_wp*/
    lwp = Qinf * dt;
    theta_wp = 0.0;
    double gamma_t_minus_dt;
    double gamma_old = 0.0;
    VectorXd vtotal_wp_cp(2);
    double iterMax = nsteps * ncycles;

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
    vector<double> xdata; // required for real time plotting cl vs t/T
    vector<double> ydata;

    double prcntgtme;

    for (int iter = 0; iter <= iterMax; iter++)
    {
        prcntgtme = iter / (double)(iterMax) * 100.0;
        cout << "percentage time completed =" << "\t" << prcntgtme << endl;
        t = iter * dt;
        alpha_ins = alpha_instantaneous(alpha0, alpha1, phi_alpha, t, omega);
        nodal_coordinates_instantaneous(n,h0,h1,phi_h,x_pitch,y_pitch,alpha_ins,t,omega,x0,y0,x_pp,y_pp);
        controlpoints(n, x_pp, y_pp, x_cp, y_cp);
        Amatrix(n, A, x_cp, y_cp, x_pp, y_pp);
        panel(n, l_x, l_y, l, x_pp, y_pp);
        normal_function_for_panels(n, unit_normal, l_x, l_y);
        tangent_function_for_panels(n, unit_tangent, l_x, l_y);

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
        string name2 = "output_files/pressure_file/t_";
        name2 += to_string(iter);
        name2 += ".dat";
        pressurefile.open(name2.c_str());

        string name3 = "output_files/gamma_vector_file/t_";
        name3 += to_string(iter);
        name3 += ".dat";
        gammafile.open(name3.c_str());

        string name4 = "output_files/potential_file/t_";
        name4 += to_string(iter);
        name4 += ".dat";
        potentialfile.open(name4.c_str());

        string name5 = "output_files/a_matrix_file/t_";
        name5 += to_string(iter);
        name5 += ".dat";
        amatrixfile.open(name5.c_str());

        string name6 = "output_files/b_vector_file/t_";
        name6 += to_string(iter);
        name6 += ".dat";
        bvectorfile.open(name6.c_str());

        string name7 = "output_files/airfoil_normal_file/t_";
        name7 += to_string(iter);
        name7 += ".dat";
        airfoilnormalfile.open(name7.c_str());

        for (int i = 0; i < n - 1; i++)
        {
            airfoilnormalfile << unit_normal(i, 0) << "\t" << unit_normal(i, 1) << endl;
        }

        cout << "percentage completed =" << iter / iterMax * 100.0 << endl;
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
            flow_vel = velocity_at_surface_of_the_body_inertial_frame(Qinf, x_pitch, y_pitch, h0, h1, phi_h, alpha0, alpha1, phi_alpha, t, omega, x_cp(i), y_cp(i));
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
            residuals = newton_raphson(n,dt, t, lwp, theta_wp, freestream, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);
            jacobian(0, 0) = 0.0;
            jacobian(0, 1) = 0.0;
            jacobian(1, 0) = 0.0;
            jacobian(1, 1) = 0.0;
            residuals_plus = newton_raphson(n,dt, t, lwp + epsilon, theta_wp, freestream, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);

            jacobian(0, 0) = (residuals_plus(0) - residuals(0)) / epsilon;
            jacobian(1, 0) = (residuals_plus(1) - residuals(1)) / epsilon;

            residuals_plus = newton_raphson(n,dt, t, lwp, theta_wp + epsilon, freestream, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);

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
        residuals = newton_raphson(n,dt, t, lwp, theta_wp, freestream, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);
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
        fsl.open("output_files/check_streamline_usptream.dat");
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
            vifsl_b = velocity_bound_vortices(n, x_pp, y_pp, xcp_forward_stag_streamline(i), ycp_forward_stag_streamline(i), gamma_bound);
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
                    viacp_b = velocity_bound_vortices(n,x_pp, y_pp, x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset, gamma_bound);
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
                    viacp_b = velocity_bound_vortices(n,x_pp, y_pp, x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset, gamma_bound);
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
            viacp_b = velocity_bound_vortices(n,x_pp, y_pp, x_cp(i) + unit_normal(i, 0) * offset, y_cp(i) + unit_normal(i, 1) * offset, gamma_bound);
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

            flow_vel = velocity_at_surface_of_the_body_inertial_frame(Qinf, x_pitch, y_pitch, h0, h1, phi_h, alpha0, alpha1, phi_alpha, t, omega, x_cp(i), y_cp(i));
            vi = viacp_rw + viacp_b + viacp_pw + flow_vel;
            V = magnitude(vi);
            cp(i) = 1.0 - (V * V) / (Qinf * Qinf) - (2.0 / (Qinf * Qinf)) * (dphi_dt(i));
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

        // myfile_load_cal << 2.0*t*Qinf/c  << "\t" << cn_tilda / cl_tilda_steady << "\t" << ca_tilda << endl; //uncomment this for sudden acceleration case.
        myfile_load_cal << t / T << "\t" << cn_tilda << "\t" << ca_tilda << endl;

        xdata.push_back(t / T);
        ydata.push_back(cn_tilda);

        /*now we need to convect the panel as a discrete vortex for the next time step*/
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
        plot_ClvsTime(gnuplotPipe1, xdata, ydata, ncycles);

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
                velocity = velocity_bound_vortices(n,x_pp, y_pp, gamma_wake_x_location[j], gamma_wake_y_location[j], gamma_bound);
                /*********** velocity induced at jth wake point due to wake panel ********/
                panel_coeff_matrix_wake = influence_matrix(wake_panel_coordinates(0, 0), wake_panel_coordinates(0, 1), wake_panel_coordinates(1, 0), wake_panel_coordinates(1, 1), gamma_wake_x_location[j], gamma_wake_y_location[j]);
                vel_wake_point = panel_coeff_matrix_wake * wake_panel_strength;
                /******** free wake ********/
                if (wake == 0)
                {
                    gamma_wake_x_new_location[j] = gamma_wake_x_location[j] + (freestream(0) + shed_vel(0) + velocity(0) + vel_wake_point(0)) * dt;
                    gamma_wake_y_new_location[j] = gamma_wake_y_location[j] + (freestream(1) + shed_vel(1) + velocity(1) + vel_wake_point(1)) * dt;
                }
                else if (wake == 1)
                {
                    /******** prescribed wake ********/
                    gamma_wake_x_new_location[j] = gamma_wake_x_location[j] + (freestream(0)) * dt;
                    gamma_wake_y_new_location[j] = gamma_wake_y_location[j] + (freestream(1)) * dt;
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
    pclose(gnuplotPipe1);

    /*plotting the flowfield at the last time step.*/
    for (size_t j = 0; j < gamma_wake_strength.size(); j++)
    {
        wake_last_time_step << gamma_wake_x_location[j] << "\t" << gamma_wake_y_location[j] << endl;
    }
}
