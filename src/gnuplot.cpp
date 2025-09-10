#include "gnuplot.h"

void plot_wake(FILE *gnuplotPipe, const vector<double> &gamma_wake_x_location, const vector<double> &gamma_wake_y_location, double wake_x1, double wake_y1, double wake_x2, double wake_y2, const VectorXd &x_pp, const VectorXd &y_pp,const string &terminal_type)
{
    fprintf(gnuplotPipe, "set terminal %s\n", terminal_type.c_str());
    fprintf(gnuplotPipe, "set size ratio -1\n"); // Equal axis scaling
    fprintf(gnuplotPipe, "set key outside right\n"); 

    fprintf(gnuplotPipe, "plot '-' title 'Wake Vortices' with points pt 7 ps 1.0 lc rgb 'red', '-' title 'Airfoil Surface' with lines lw 3 lc rgb 'blue'\n");

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

void plot_ClvsTime(FILE *gnuplotPipe1, const vector<double> &xdata, const vector<double> &ydata, int ncycles,const string &terminal_type)
{
    fprintf(gnuplotPipe1, "set terminal %s\n", terminal_type.c_str());
    fprintf(gnuplotPipe1, "set grid\n");
    fprintf(gnuplotPipe1, "set title 'Evolution of lift coefficient with time'\n");
    fprintf(gnuplotPipe1, "set xlabel 't/T'\n");
    fprintf(gnuplotPipe1, "set ylabel 'Cl(t)'\n");

    fprintf(gnuplotPipe1, "set xrange [0:%d]\n",ncycles);

    fprintf(gnuplotPipe1, "set terminal %s\n", terminal_type.c_str());
    fprintf(gnuplotPipe1, "plot '-' with lines lw 3 lc rgb 'green'\n");

    for (size_t i = 0; i < xdata.size(); i++)
    {
        fprintf(gnuplotPipe1, "%lf %lf\n", xdata[i], ydata[i]);
    }
    fprintf(gnuplotPipe1, "e\n"); // End of dataset
    fflush(gnuplotPipe1);         // Update plot immediately
}
