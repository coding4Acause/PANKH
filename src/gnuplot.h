#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <vector>
#include <Eigen/Dense>
#include <cstdio>

using namespace std;
using namespace Eigen;

void plot_wake(FILE *gnuplotPipe, const vector<double> &gamma_wake_x_location, 
               const vector<double> &gamma_wake_y_location, double wake_x1, 
               double wake_y1, double wake_x2, double wake_y2, 
               const VectorXd &x_pp, const VectorXd &y_pp);

void plot_ClvsTime(FILE *gnuplotPipe1, const vector<double> &xdata, 
                   const vector<double> &ydata, int ncycles);

#endif // GNUPLOT_H
