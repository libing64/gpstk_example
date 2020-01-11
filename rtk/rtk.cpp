#include "rtk.h"
#include "pvt.h"
#include "decorr.h"
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <cfloat>
#include <Eigen/Eigen>
#include "GPSEphemerisStore.hpp"

using namespace std;
using namespace Eigen;
using namespace gpstk;

// typedef struct
// {
//     double P1;
//     double P2;
//     double C1;
//     double C2;
//     float pos[3];
//     float pos_station[3];
//     Vector3d sat_pos;
//     //Xvt sat_xvt;
//     //SatID prn;
// } rtk_obs_t;

//vector<rtk_obs_t> &rtk_obs, Vector3d station_pos
void generate_data(vector<rtk_obs_t> &rtk_obs, VectorXd &x_rcv, VectorXd& x_station, VectorXd& N_rcv, VectorXd& N_station)
{
    x_rcv = VectorXd::Random(4) * 100000;//x and dt
    x_station = x_rcv + VectorXd::Random(4) * 200; //x and dt
    N_rcv = VectorXd::Zero(rtk_obs.size());
    N_station = VectorXd::Zero(rtk_obs.size());
    cout << "x_rcv: " << x_rcv.transpose() << endl;
    cout << "x_station: " << x_station.transpose() << endl;
    for (int i = 0; i < rtk_obs.size(); i++)
    {
        rtk_obs[i].sat_pos = VectorXd::Random(3) * 1000000;
        rtk_obs[i].P1 = distance(rtk_obs[i].sat_pos, x_rcv.segment(0, 3)) + x_rcv(3);
        rtk_obs[i].P2 = distance(rtk_obs[i].sat_pos, x_station.segment(0, 3)) + x_station(3);
        double n_rcv = rtk_obs[i].P1 / L1_WAVELENGTH_GPS; //ambiguty resolution
        double n_station = rtk_obs[i].P2 / L1_WAVELENGTH_GPS;
        rtk_obs[i].C1 = n_rcv - floor(n_rcv);
        rtk_obs[i].C2 = n_station - floor(n_station);
        N_rcv(i) = floor(n_rcv);
        N_station(i) = floor(n_station);
        cout << "sat: " << i << "  " << rtk_obs[i].sat_pos.transpose()
             << "  P1:" << rtk_obs[i].P1 << "  C1:" << rtk_obs[i].C1 
             << "  P2:" << rtk_obs[i].P2 << "  C2:" << rtk_obs[i].C2 << endl;
    }
}

MatrixXd matrix_round(const MatrixXd m)
{
    MatrixXd mm = m;
    for (int i = 0; i < mm.rows(); i++)
    {
        for (int j = 0; j < mm.cols(); j++)
        {
            mm(i, j) = round(mm(i, j));
        }
    }
    return mm;
}

void decorrelation(const MatrixXd Qn, MatrixXd &Q_decorr, MatrixXd &Zt, MatrixXd &iZt)
{
    MatrixXd Q_raw = Qn;
    MatrixXd Q = Qn;
    int n = Qn.rows();

    MatrixXd L = MatrixXd::Zero(n, n);
    VectorXd D = VectorXd::Zero(n);
    ldl_decomp(Q, L, D);

    decorr(L, D, Zt, iZt);
    Q_decorr = Zt * Q * Zt.transpose();
    //cout << "Zt: " << Zt << endl;
    //cout << "Q_decorr : " << Q_decorr << endl;
}

void lambda_search(MatrixXd Qz, VectorXd z, VectorXd &z_fixed)
{
    int n = Qz.rows();
    int range = 3;
    VectorXd offset = VectorXd::Zero(n);
    for (int i = 0; i < n; i++)
        z_fixed(i) = round(z(i));
    for (int i = 0; i < n; i++)
    {
        VectorXd zz = z_fixed;
        double min_val = DBL_MAX;
        for (int j = -range; j <= range; j++)
        {
            zz(i) = z_fixed(i) + j;
            VectorXd dz = (zz - z);
            double val = dz.transpose() * Qz * dz;
            if (val < min_val)
            {
                min_val = val;
                offset(i) = j;
            }
        }
    }
    cout << "offset: " << offset.transpose() << endl;
    z_fixed = z_fixed + offset;
}

//整周模糊度求解
void lambda_solver(MatrixXd Qa, VectorXd a, VectorXd &a_fixed)
{
    MatrixXd Zt, iZt, Qz;
    int n = Qa.rows();
    decorrelation(Qa, Qz, Zt, iZt);
    VectorXd z = Zt * a;
    cout << "a: " << a.transpose() << endl;
    cout << "z: " << z.transpose() << endl;

    VectorXd z_fixed = VectorXd::Zero(n);

    lambda_search(Qz, z, z_fixed);
    a_fixed = iZt * z_fixed;
    cout << "z_fixed: " << z_fixed.transpose() << endl;
    cout << "a_fix: " << a_fixed.transpose() << endl;
}

/* 
prepare covariance matrix for double difference
please note that that the measurements of difference satellite are uncorrelated, but dd will make them correlated
n : n satellates
*/
void covariance_matrix(int n, MatrixXd &H_dd, MatrixXd &Q_dd, MatrixXd &W_dd)
{
    const double Pcov = 1.0;
    const double Ccov = 1.0;
    Eigen::MatrixXd Q = MatrixXd::Zero(2 * n, 2 * n); //covariance matrix of raw gps measurement
    for (int i = 0; i < n; i++)
    {
        Q(2 * i, 2 * i) = Ccov;         //covariance of carrier phase measurement
        Q(2 * i + 1, 2 * i + 1) = Pcov; //covariance of Pseudorange measurement
    }
    //cout << "line: " << __LINE__ << endl;
    //compute cov of difference
    H_dd = MatrixXd::Zero(2 * n - 2, 2 * n);
    for (int i = 0; i < (n - 1); i++)
    {
        H_dd(2 * i, 0) = 1;
        H_dd(2 * i, 2 * i + 2) = -1;

        H_dd(2 * i + 1, 1) = 1;
        H_dd(2 * i + 1, 2 * i + 3) = -1;
    }
    //cout << "line: " << __LINE__ << endl;
    Q_dd = H_dd * Q * H_dd.transpose();
    cout << "Q: " << Q << endl;
    cout << "H_dd: " << H_dd << endl;
    cout << "Q_dd: " << Q_dd << endl;
    //cout << "line: " << __LINE__ << endl;
    W_dd = Q_dd.inverse();
    //cout << "W_dd " << W_dd << endl;
}


//covariance matrix of sigle difference
void covariance_matrix_sd(int n, MatrixXd &Q, MatrixXd &W)
{
    const double Pcov = 1.0;
    const double Ccov = 1.0;
    Q = MatrixXd::Zero(2 * n, 2 * n); //covariance matrix of raw gps measurement
    for (int i = 0; i < n; i++)
    {
        Q(2 * i, 2 * i) = Ccov;         //covariance of carrier phase measurement
        Q(2 * i + 1, 2 * i + 1) = Pcov; //covariance of Pseudorange measurement
    }
    W = Q.inverse();
    cout << "W " << W << endl;
}

void correct_satpos(Triple &sat_pos, Vector3d rcv_pos)
{
    Vector3d pos;
    pos(0) = sat_pos[0];
    pos(1) = sat_pos[1];
    pos(2) = sat_pos[2];
    double tx = distance(pos, rcv_pos) / C_MPS;
    double wt = angVelocity() * tx; // radians
    sat_pos[0] = cos(wt) * sat_pos[0] + sin(wt) * sat_pos[1];
    sat_pos[1] = -sin(wt) * sat_pos[0] + cos(wt) * sat_pos[1];
}
//measurement [c1, p1, c2, p2, ...cn, pn] -> double difference
void rtk_solver(vector<rtk_obs_t> &rtk_obs, Vector3d station_pos)
{
    VectorXd x_rcv, x_station;
    VectorXd N_rcv, N_station;
    generate_data(rtk_obs, x_rcv, x_station, N_rcv, N_station);
    cout << "x_rcv: " << x_rcv.transpose() << endl;
    cout << "x_station: " << x_station.transpose() << endl;
    station_pos = x_station.segment(0, 3);
    int n = rtk_obs.size();
    cout << "rtk solver: " << n << endl;
    int rows = 2 * n - 2; //all the measurements converted to double difference
    int cols = n - 1 + 3; //all the variables to be solved, dx, dy, dz and (n-1) ambiguity resolution
    MatrixXd H = MatrixXd::Zero(rows, cols);
    VectorXd y = VectorXd::Zero(rows);

    MatrixXd H_dd, W_dd, Q_dd;
    covariance_matrix(n, H_dd, Q_dd, W_dd);


    if (rtk_obs.size() <= 6)
    {
        cerr << "no enough observation for rtk solver" << endl;
    }

    rtk_obs_t obs1 = rtk_obs[0];
    Vector3d sat_pos1 = obs1.sat_pos;
    Vector3d I1 = sat_pos1- station_pos;
    I1.normalize();

    for (int i = 0; i < (n - 1); i++)
    {
        rtk_obs_t obs2 = rtk_obs[i + 1];
        double dd_c = (obs1.C1 - obs1.C2) - (obs2.C1 - obs2.C2); //double difference
        double dd_p = (obs1.P1 - obs1.P2) - (obs2.P1 - obs2.P2); //double difference

        y(2 * i) = dd_c;
        y(2 * i + 1) = dd_p;

        Vector3d I2;
        Vector3d sat_pos2 = obs2.sat_pos;
        I2 = sat_pos2 - station_pos;
        I2.normalize();

        H.block(2 * i, 0, 1, 3) = (I1 - I2).transpose();
        H(2 * i, 3 + i) = L1_WAVELENGTH_GPS;
        H.block(2 * i + 1, 0, 1, 3) = (I1 - I2).transpose();
    }
    cout << "H: " << H << endl;
    cout << "y: " << y.transpose() << endl;
    //least square solver without weight
    // VectorXd x1 = H.bdcSvd(ComputeThinU | ComputeThinV).solve(y);
    // cout << "x1: " << x1.transpose() << endl;
    MatrixXd Ht = H.transpose();
    //VectorXd x = (Ht * W_dd * H).inverse() * Ht * W_dd * y;
    VectorXd x = (Ht * W_dd * H).ldlt().solve(Ht * W_dd * y);
    VectorXd res = y - H * x;
    cout << "x: " << x.transpose() << endl;
    cout << "res " << res.transpose() << endl;
    cout << "x_real: " << (x_rcv - x_station).transpose() << endl;
    VectorXd N_ab = VectorXd::Zero(n-1);
    VectorXd dN = (N_rcv - N_station);
    for (int i = 0; i < (n - 1); i++)
    {
        N_ab(i) = dN(i + 1) - dN(0);
    }
    cout << "N_ab: " << N_ab.transpose() << endl;

    if (res.norm() / (2 * n) > 10.0)
    {
        cout << "lambda solve failed!!!!!  " << endl;
    }

    //compute Qxn, Qx, QnComputeThinV
    MatrixXd hh = (Ht * W_dd * H).inverse() * Ht * W_dd;
    MatrixXd Q_xn = hh * Q_dd * hh.transpose();

    // cout << "W_dd: " << W_dd << endl;
    // cout << "hh: " << hh << endl;
    // cout << "Q_dd: " << Q_dd << endl;
    // cout << "Q_xn: " << Q_xn << endl;
    //cout << "Q_xn size: " << Q_xn.rows() << "  " << Q_xn.cols() << endl;
    //cout << "n: " << n << endl;
    MatrixXd Qx = Q_xn.block(0, 0, 3, 3);
    //cout << "Qx: " << Qx << endl;
    MatrixXd Qn = Q_xn.block(3, 3, n - 1, n - 1);
    //cout << "Qn: " << Qn << endl;
    VectorXd N = x.segment(3, n - 1);
    VectorXd N_fixed;
    lambda_solver(Qn, N, N_fixed);

    //solve (dx, dy, dz) with ambigity resolution being fixed
    int m = x.rows(); //length of x
    //cout << "n: " << n << endl;
    //cout << "m: " << m << endl;
    VectorXd xx = VectorXd::Zero(m);
    xx.segment(3, n - 1) = N_fixed;
    MatrixXd HH = H.block(0, 0, 2 * n - 2, 3);
    VectorXd yy = y - H * xx; //after ambiguity being removed

    MatrixXd HHt = HH.transpose();
    VectorXd x2 = (HHt * W_dd * HH).inverse() * HHt * W_dd * yy;
    cout << "yy: " << yy.transpose() << endl;
    cout << "x2: " << x2.transpose() << endl;
}


//单差方式求解
void single_diff_solver(vector<rtk_obs_t> &rtk_obs, Vector3d station_pos)
{
    VectorXd x_rcv, x_station;
    VectorXd N_rcv, N_station;
    generate_data(rtk_obs, x_rcv, x_station, N_rcv, N_station);
    cout << "x_rcv: " << x_rcv.transpose() << endl;
    cout << "x_station: " << x_station.transpose() << endl;
    station_pos = x_station.segment(0, 3);
    int n = rtk_obs.size();
    cout << "rtk solver: " << n << endl;
    int rows = 2 * n; //all the measurements converted to double difference
    int cols = n + 4; //all the variables to be solved, dx, dy, dz dt, and n ambiguity resolution
    MatrixXd H = MatrixXd::Zero(rows, cols);
    VectorXd y = VectorXd::Zero(rows);

    MatrixXd W, Q;
    covariance_matrix_sd(n, Q, W);

    if (rtk_obs.size() <= 4)
    {
        cerr << "no enough observation for rtk solver" << endl;
    }

    for (int i = 0; i < n; i++)
    {
        rtk_obs_t obs = rtk_obs[i];
        double sd_c = (obs.P1 - obs.P2);//(obs.C1 - obs.C2); //single difference
        double sd_p = (obs.P1 - obs.P2); //single difference

        y(2 * i) = sd_c;
        y(2 * i + 1) = sd_p;

        Vector3d I;
        Vector3d sat_pos = obs.sat_pos;
        I = sat_pos - station_pos;//!!!important
        I.normalize();

        // H.block(2 * i, 0, 1, 3) = I.transpose();
        // H(2 * i, 4 + i) = L1_WAVELENGTH_GPS;

        H.block(2 * i, 0, 1, 3) = -I.transpose();
        H(2 * i, 3) = 1;
        //pseudorange
        H.block(2 * i + 1, 0, 1, 3) = -I.transpose();
        H(2 * i + 1, 3) = 1;
    }
    cout << "H: " << H << endl;
    cout << "y: " << y.transpose() << endl;
    //least square solver without weight
    // VectorXd x1 = H.bdcSvd(ComputeThinU | ComputeThinV).solve(y);
    // cout << "x1: " << x1.transpose() << endl;
    MatrixXd Ht = H.transpose();
    //VectorXd x = (Ht * W_dd * H).inverse() * Ht * W_dd * y;
    VectorXd x = (Ht * W * H).ldlt().solve(Ht * W * y);
    VectorXd res = y - H * x;
    cout << "x: " << x.transpose() << endl;
    cout << "res " << res.transpose() << endl;
    cout << "x_real: " << (x_rcv - x_station).transpose() << endl;
    VectorXd dN = (N_rcv - N_station);
    cout <<  "dN:  " << dN.transpose() << endl;
}