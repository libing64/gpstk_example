#ifndef __RTK_H
#define __RTK_H

#include <iostream>
#include <Eigen/Eigen>
#include "GPSEphemerisStore.hpp"

using namespace std;
using namespace Eigen;
using namespace gpstk;

typedef struct
{
    double P1;
    double P2;
    double C1;
    double C2;
    float pos[3];
    float pos_station[3];
    Vector3d sat_pos;
    //Xvt sat_xvt;
    //SatID prn;
} rtk_obs_t;

void rtk_solver(vector<rtk_obs_t> &rtk_obs, Vector3d station_pos);
void single_diff_solver(vector<rtk_obs_t> &rtk_obs, Vector3d station_pos);
#endif
