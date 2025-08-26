#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace Eigen;
using namespace std;

#define pi acos(-1)

// NACA 4-digit airfoil properties
extern int n;  // Number of nodes [So, number of panels = n-1]
extern double ymc;
extern double xmc;
extern double tmax;
extern double p;  // Non-dimensionalized ymc [p=ymc/c]
extern double q;  // Non-dimensionalized xmc [q=xmc/c]
extern double t_m;
extern double trailing_edge_type;  // 1 = open, else closed

// Test conditions in water tunnel
extern double c;       // Chord length
extern double rho;     // Density of water
extern double mu;      // Viscosity of water
extern double Re;      // Reynolds number
extern double Qinf;    // Flow speed or magnitude of the freestream velocity
extern double dynpress; // Dynamic pressure

extern double b;  // Half-chord
extern double xf; // Position of pitch axis
extern double a;  // Non-dimensional pitch axis position

extern double Uinf;
extern double Vinf;
extern Eigen::Vector2d freestream;

// Motion parameters
extern double k;       // Reduced frequency
extern double omega;   // Frequency of oscillation [rad/sec]
extern double T;       // Time period of oscillation
extern double h1;      // Plunge amplitude
extern double alpha1;  // Pitch amplitude
extern double h0;      // Mean plunge
extern double alpha0;  // Mean pitch
extern double phi_h;   // Plunge phase
extern double phi_alpha; // Pitch phase

// Wake type
extern int wake; // [0 = FREE WAKE, 1 = PRESCRIBED WAKE]

// Convergence parameters
extern double tolerance; // Convergence criterion for loops
extern double epsilon;   // Increment

#endif // CONSTANTS_H
