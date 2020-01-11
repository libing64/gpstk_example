#include "ppp.h"
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace gpstk;


ppp_solver::ppp_solver(int n_sat)
{
    int n = 5 + n_sat;
    A = MatrixXd::Zero(n_sat, n);
    P = MatrixXd::Identity(n, n);
}

ppp_solver::~ppp_solver()
{

}

void solve(vector<ppp_obs_t> ppp_obs)
{

    for (int i = 0; i < ppp_obs.size(); i++)
    {
        ppp_obs_t obs = ppp_obs[i];
        Vector3d sat_pos = obs.sat_pos;
        double P = obs.P;
        double C = obs.C;

        double rho = distance(sat_pos, x.segment(0, 3));
        double tx = rho / C_MPS;
        y(i) = P - rho - x(3);

        Vector3d I = x.segment(0, 3) - sat_pos;
        I.normalize();
        H.block(i, 0, 1, 3) = I.transpose();
        H(i, 3) = 1;
        H(i, 4) = M;//zenith path delay
    }

}