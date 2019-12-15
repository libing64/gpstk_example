#ifndef __DECORR_H
#define __DECORR_H
#include <iostream>
#include <Eigen/Eigen>


using namespace std;
using namespace Eigen;

void ldl_decomp(MatrixXd Q, MatrixXd &L, VectorXd &D);
void decorr(MatrixXd &L, VectorXd &D, MatrixXd &Z);
#endif