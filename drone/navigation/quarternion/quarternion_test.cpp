#include <iostream>
#include <vector>
#include <assert.h>
#include <math.h>
#include <cstdlib>
#include <cmath>
#include "kalman/kalman.hpp"
#include "kalman/kalman.cpp"

int main()
{
    std::vector<std::vector<double> > a;
    a = {{1,0,1},{0,3.0,6.0},{0,1,2}};
    Matrix<double> n({Matrix<double>({{0,0,1},{0,3.0,6.0},{0,1,2}})});
    n.show();
    Matrix<double> m{Matrix<double>(a)};
    std::vector<std::vector<double> > b;
    b = {{1.0,1.0,1.0},{2.0,2.0,2.0},{3.0,3.0,3.0}};
    m = n;
    (m*n).show();
    (m*m.inverse()).show();
    m = (m*m.inverse());

    Kalman kalman(0.01);
    Matrix<double> omega{std::vector<std::vector<double> >(3,std::vector<double>(1,1))};
    Matrix<double> y{std::vector<std::vector<double> >(6,std::vector<double>(1,1))};
    for (int i = 0; i < 50; i++)
    {
        omega(0) += omega(0) + (double)i;
        omega(1) += omega(1) + (double)i;
        omega(2) += omega(2) + (double)i;

        std::cout << i <<"回目" << "\n";
        kalman.calculate_quarternion(omega,y);
        kalman.show_kalman_gain();
    }

}