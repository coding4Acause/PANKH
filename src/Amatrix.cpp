#include "Amatrix.h"
#include <fstream>  // For file writing

void Amatrix(int n, MatrixXd &A, VectorXd &x_cp, VectorXd &y_cp, VectorXd &x_pp, VectorXd &y_pp)
{
   // ofstream myfile3("A_matrix_file.dat");

    MatrixXd pcm(2, 2);
    double x1, x2, y1, y2, dx, dy, li;

    A.setZero();  // Initialize matrix with zeros

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
            A(j, i) += dx / li * pcm(1, 0) - dy / li * pcm(0, 0);
            A(j, i + 1) += dx / li * pcm(1, 1) - dy / li * pcm(0, 1);
        }
    }
    A(n - 1, 0) = 1.0;
    A(n - 1, n - 1) = 1.0;

   // myfile3 << A << endl;
}
