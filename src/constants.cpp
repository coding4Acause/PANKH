#include "constants.h"

// NACA 4 digit series Airfoil properties
int n = 101;              // number of nodes [So, number of panels = n-1] // this can be even or odd
double ymc = 0;           // 1st digit of NACA 4 digit series airfoil
double xmc = 0;           // 2nd digit of NACA 4 digit series airfoil
double tmax = 12;         // last two digits of NACA 4 digit series airfoil
double p = (ymc / 100.0); // p non dimensionalised ymc [p=ymc/c]
double q = (xmc * 0.1);   // q non dimensionalised xmc [q=xmc/c]
double tm = (tmax / 100.0);
double trailing_edge_type = 2; // if (trailing_edge_type ==1 ==> open ; else ==> closed)

// Test conditions in water tunnel 
double c = 0.1;                            // chord length
double rho = 998.2;                        // Density of water
double mu = 1.0016e-3;                     // Viscosity of water
double Re = 40000;                         // Reynolds number
double Qinf = Re * mu / (rho * c);         // Flow speed or magnitude of the freestream velocity
double dynpress = 0.5 * rho * Qinf * Qinf; // Dynamic pressure

double b = c / 2;        // Calculate half-chord.
double xf = c / 3;       // Position of pitch axis.
double a = (xf - b) / b; // Non-dimensional pitch axis position.

// In this problem we are rotating the airfoil by instantaneous angle of attack keeping the freestream direction parallel to X dirn.[INERTIAL FRAME]
double Uinf = Qinf;
double Vinf = 0.0;
Vector2d freestream(Uinf, Vinf);

// Motion parameters
double k = 1.2;                                             // Reduced frequency
double omega = (2.0 * k * Qinf) / c;                        // [rad/sec] Frequency of oscillation
double T = 2.0 * pi / omega;                                // Time Period of oscillation
double h1 = 0.25 * c;                                       // Plunge amplitude
double alpha1 = 15.0 * pi / 180.0 - atan2(2.0 * k * h1, c); // Pitch amplitude
double h0 = 0.0;                                            // Mean plunge
double alpha0 = 0.0 * pi / 180.0;                           // Mean pitch
double phi_h = 0.0;                                         // Plunge phase
double phi_alpha = (90.0 + phi_h) * pi / 180.0;             // Pitch phase

// choice of free or prescribed wake
int wake = 0; // [ 0-FREE WAKE, 1 FOR PRESCRIBED WAKE]

// convergence parameters
double tolerance = 1.e-6; // do while loop conv. criterion
double epsilon = 1.e-8;   // increment