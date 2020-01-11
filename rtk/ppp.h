#ifndef __PPP_H
#define __PPP_H
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace gpstk;



typedef struct
{
    double P;
    double C;
    SatID prn;
    Vector3d sat_pos;
} ppp_obs_t;


class ppp_solver
{
private:
    /* data */
public:
    VectorXd x;//x,y,z, dt, zpd, N1...Nn
    MatrixXd A;
    MatrixXd P;
    map<SatID, int> sat_index;
    ppp_solver();
    ~ppp_solver();
    void solve(vector<ppp_obs_t> ppt_obs);
};






#endif