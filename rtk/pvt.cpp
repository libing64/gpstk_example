#include "pvt.h"
#include "GPSEphemerisStore.hpp"



double distance(Vector3d a, Vector3d b)
{
    return (a - b).norm();
}

void genetate_data(vector<pvt_obs_t>& pvt_obs, VectorXd& x)
{
    x = VectorXd::Random(4) * 100000;
    cout << "xx: " << x.transpose() << endl;

    for (int i = 0; i < pvt_obs.size(); i++)
    {
        pvt_obs[i].sat_pos = VectorXd::Random(3) * 1000000;
        pvt_obs[i].P = distance(pvt_obs[i].sat_pos, x.segment(0, 3)) + x(3);
        cout << "sat: " << i << "  P:" << pvt_obs[i].P << "  " << pvt_obs[i].sat_pos.transpose() << endl;
    }

}

/// defined in ICD-GPS-200C, 20.3.3.4.3.3 and Table 20-IV
/// @return angular velocity of Earth in radians/sec.
double angVelocity() 
{
    return 7.2921151467e-5;
}
//solve receiver position
void pvt_solver(vector<pvt_obs_t> pvt_obs, Vector4d& solution, VectorXd& residual)
{
    VectorXd x = Vector4d::Zero(4);
    const double C_MPS = 2.99792458e8;
    //VectorXd x = Vector4d(-3978242.4348, 3382841.1715, 3649902.7667, 0);
    //genetate_data(pvt_obs, x);
    //cout << "xx2: " << x.transpose() << endl;
    x = Vector4d::Zero(4);
    int n = pvt_obs.size();

    double err = 1e8;
    int cnt = 0;
    VectorXd y = VectorXd::Zero(n);
    MatrixXd H = MatrixXd::Zero(n, 4);
    MatrixXd W = MatrixXd::Zero(n, n);
    while (err > 0.01 && cnt < 20)
    {
        for (int i = 0; i < n; i++)
        {
            pvt_obs_t obs = pvt_obs[i];
            Vector3d sat_pos = obs.sat_pos;
            double P = obs.P;
            double rho = distance(sat_pos, x.segment(0, 3));
            double tx = rho / C_MPS;
            y(i) = P - rho - x(3);

            // correct for earth rotation
            double wt = angVelocity() * tx; // radians
            sat_pos(0) = cos(wt) * sat_pos(0) + sin(wt) * sat_pos(1);
            sat_pos(1) = -sin(wt) * sat_pos(0) + cos(wt) * sat_pos(1);
            sat_pos(2) = sat_pos(2);

            rho = distance(sat_pos, x.segment(0, 3));

            Vector3d I = x.segment(0, 3) - sat_pos;
            I.normalize();
            H.block(i, 0, 1, 3) = I.transpose();
            H(i, 3) = 1.0;
            //cout << "P: " << P << "  rho : " << rho  << "  y: " << y(i)  << "  weight: " << W(i, i) << " elev angle: " << elev_angle << endl;
        }
        MatrixXd HH = H.transpose() * H;
        VectorXd dx = HH.ldlt().solve(H.transpose() * y);
        //cout << "H: " << H << endl;
        // cout << "y: " << y.transpose() << endl;
        // cout << "dx: " << dx.transpose() << endl;
        // cout << "H * dx: " << (H*dx).transpose() << endl;
        x += dx;
        err = dx.norm();
        cnt++;
        // cout << "x: " << x.transpose() << endl;
        // cout << "err: " << err << endl;
        // cout << "===============================" << endl;
    }
    solution = x;
    residual = y;
}

