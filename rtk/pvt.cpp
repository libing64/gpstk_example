#include "pvt.h"


double distance(Vector3d a, Vector3d b)
{
    return (a - b).norm();
}

void genetate_data(vector<pvt_obs_t>& pvt_obs)
{
    VectorXd x = VectorXd::Random(4);
    cout << "xx: " << x.transpose() << endl;

    for (int i = 0; i < pvt_obs.size(); i++)
    {
        pvt_obs[i].sat_pos = VectorXd::Random(3);
        pvt_obs[i].P = distance(pvt_obs[i].sat_pos, x.segment(0, 3)) + x(3);
        cout << "sat: " << i << "  P:" << pvt_obs[i].P << "  " << pvt_obs[i].sat_pos.transpose() << endl;
    }

}

void residual(VectorXd x, vector<pvt_obs_t> pvt_obs, VectorXd& res, MatrixXd& H)
{
    int n = pvt_obs.size();
    res = VectorXd::Zero(n);
    H = MatrixXd::Zero(n, 4);
    for (int i = 0; i < n; i++)
    {
        pvt_obs_t obs = pvt_obs[i];
        Vector3d sat_pos = obs.sat_pos;
        double P = obs.P;
        double rho = distance(sat_pos, x.segment(0, 3)) + x(3);
        res(i) = P - rho;
        cout << "P: " << P << "  rho : " << rho << "  y: " << res(i) << endl;
        H(i, 0) = (x(0) - sat_pos(0)) / rho;
        H(i, 1) = (x(1) - sat_pos(1)) / rho;
        H(i, 2) = (x(2) - sat_pos(2)) / rho;
        H(i, 3) = 1.0;
    }
}
//solve receiver position
void pvt_solver(vector<pvt_obs_t> pvt_obs)
{

    //genetate_data(pvt_obs);
    //cout << "xx2: " << x.transpose() << endl;
    int n = pvt_obs.size();

    //VectorXd x = Vector4d(-3978242.4348, 3382841.1715, 3649902.7667, 0);
    VectorXd x = VectorXd::Zero(4); //x, y, z, tau
    double err = 1e8;
    int cnt = 0;
    VectorXd y = VectorXd::Zero(n);
    MatrixXd H = MatrixXd::Zero(n, 4);
    for (int i = 0; i < n; i++)
    {
        cout << "sat: " << i << "  P:" << pvt_obs[i].P  << "  " << pvt_obs[i].sat_pos.transpose() << endl;
    }
    double mu = 1.0;
    while (err > 0.1 && cnt < 20)
    {
        cout << "cnt : <<<<<<<<<<<<<<<<<<<<<<<<<<" << cnt << endl;
        for (int i = 0; i < n; i++)
        {
            pvt_obs_t obs = pvt_obs[i];
            Vector3d sat_pos = obs.sat_pos;
            double P = obs.P;
            double rho = distance(sat_pos, x.segment(0, 3)) + x(3);
            y(i) = P - rho;
            cout << "P: " << P << "  rho : " << rho  << "  y: " << y(i) << endl;
            H(i, 0) = (x(0) - sat_pos(0)) / rho;
            H(i, 1) = (x(1) - sat_pos(1)) / rho;
            H(i, 2) = (x(2) - sat_pos(2)) / rho;
            H(i, 3) = 1.0;
        }
        VectorXd dx = H.bdcSvd(ComputeThinU | ComputeThinV).solve(y);
        cout << "H: " << H << endl;
        cout << "y: " << y.transpose() << endl;
        cout << "dx: " << dx.transpose() << endl;
        x += dx;
        err = y.norm();
        cout << "x: " << x.transpose() << endl;
        cout << "err: " << err << endl;
        cnt++;
        cout << "===============================" << endl;
    }

}

