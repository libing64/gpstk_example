#include "decorr.h"
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

void ldl_decomp(MatrixXd Q, MatrixXd &L, VectorXd &D)
{
    int n = Q.rows();
    L = MatrixXd::Zero(n, n);
    D = VectorXd::Zero(n);
    for (int i = n - 1; i >= 0; i--)
    {
        D(i) = Q(i, i);
        for (int k = 0; k <= i; k++)
            L(i, k) = Q(i, k) / sqrt(Q(i, i));
        for (int j = 0; j <= (i - 1); j++)
        {
            for (int k = 0; k <= j; k++)
                Q(j, k) -= L(i, k) * L(i, j);
        }
        for (int k = 0; k <= i; k++)
            L(i, k) /= L(i, i);
    }
    //cout << "ldlt decomp: " << endl;
    //cout << "L: " << L << endl;
    //cout << "D: " << D.transpose() << endl;
}

/*gauss tranfomr for col k*/
void gauss(MatrixXd &L, MatrixXd &Z, int c)
{
    //cout << "gauss: " << c << endl;
    //cout << "Z1: " << Z << endl;
    //cout << "L1: " << L << endl;
    int n = L.rows();
    int i = c;
    for (int j = i + 1; j < n; j++) //row i + 1 -> n -1
    {
        int mu = round(L(j, i));
        if (mu != 0)
        {
            for (int k = j; k < n; k++)
                L(k, i) -= mu * L(k, j);
            for (int k = 0; k < n; k++)
                Z(k, j) += mu * Z(k, i);
        }
    }
    //cout << "Z2: " << Z << endl;
    //cout << "L2: " << L << endl;
}

void sswap(double &a, double &b)
{
    double t(a);
    a = b;
    b = t;
}

void permute(MatrixXd &L, VectorXd &D, int i, double delta, MatrixXd &Z)
{
    int n = L.rows();
    double lambda3 = D(i + 1) * L(i + 1, i) / delta;
    double eta = D(i) / delta;
    D(i) = eta * D(i + 1);
    D(i + 1) = delta;
    for (int j = 0; j <= i - 1; j++)
    {
        double lambda1 = L(i, j);
        double lambda2 = L(i + 1, j);
        L(i, j) = -L(i + 1, i) * lambda1 + lambda2;
        L(i + 1, j) = eta * lambda1 + lambda3 * lambda2;
    }
    L(i + 1, i) = lambda3;
    /*swap of cols L(i+2:n, i) and L(i+2:n, i+1)*/
    for (int k = i + 2; k < n; k++)
        sswap(L(k, i), L(k, i + 1));
    /*swap of cols Z(1:n, i) and Z(1:n, i+1)*/
    for (int k = 0; k < n; k++)
        sswap(Z(k, i), Z(k, i + 1));
}
void decorr(MatrixXd &L, VectorXd &D, MatrixXd &Z)
{
    int n = L.rows();
    int ii = n - 2;
    bool sw = true;
    MatrixXd iZt = MatrixXd::Identity(n, n);
    while (sw)
    {
        int i = n - 1; //loop for colomn from n - 1 to 0
        sw = false;
        while ((~sw) && (i > 0))
        {
            i--; //the ith column
            //cout << "i: " << i << "  ii: " << ii << endl;
            if (i <= ii)
            {
                gauss(L, iZt, i);
            }
            double delta = D(i) + L(i + 1, i) * L(i + 1, i) * D(i + 1);
            //cout << "delta: " << delta << endl;
            if (delta < D(i + 1))
            {
                permute(L, D, i, delta, iZt);
                ii = i;
                sw = true;
            }
        }
    }

    //cout << "iZt: " << iZt << endl;
    Z = iZt.inverse().transpose();
    for (int i = 0; i < Z.rows(); i++)
        for (int j = 0; j < Z.cols(); j++)
            Z(i, j) = round(Z(i, j));
}
// Shows how to utilize MLAMBDA-Eigen
int decorr_example()
{
    int n = 6;
    MatrixXd Q = MatrixXd::Zero(n, n);
    Q << 0.0977961, 0.0161137, 0.0468261, 0.0320695, 0.080857, 0.0376408,
        0.0161137, 0.0208976, 0.0185378, 0.00290225, 0.0111409, 0.0247762,
        0.0468261, 0.0185378, 0.0435412, 0.0227732, 0.0383208, 0.0382978,
        0.0320695, 0.00290225, 0.0227732, 0.0161712, 0.0273471, 0.0154774,
        0.080857, 0.0111409, 0.0383208, 0.0273471, 0.0672121, 0.0294637,
        0.0376408, 0.0247762, 0.0382978, 0.0154774, 0.0294637, 0.0392536;

    // int n = 3;
    // MatrixXd Q = MatrixXd::Zero(n, n);
    // Q << 6.288, 2.340, 0.544,
    //     2.340, 6.292, 5.978,
    //     0.544, 5.978, 6.290;

    // int n = 2;
    // MatrixXd Q = MatrixXd::Zero(n, n);
    // Q << 53.4, 38.4,
    //      38.4, 28.0;

    MatrixXd L, Z;
    VectorXd D;
    ldl_decomp(Q, L, D);

    MatrixXd DD = MatrixXd::Zero(n, n);
    for (int i = 0; i < n; i++)
        DD(i, i) = D(i);
    MatrixXd Q_ldlt = L.transpose() * DD * L;

    decorr(L, D, Z);
    MatrixXd QQ = Z.transpose() * Q * Z;
    cout << "Z: " << Z << endl;
    cout << "QQ : " << QQ << endl;
    return 0;
}
