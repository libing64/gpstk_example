#ifndef __PVT_H
#define __PVT_H
#include<iostream>
#include<Eigen/Eigen>
#include "GPSEphemerisStore.hpp"

using namespace std;
using namespace Eigen;
using namespace gpstk;

typedef struct
{
    double P;
    Vector3d sat_pos;
} pvt_obs_t;

void pvt_solver(vector<pvt_obs_t> pvt_obs, Vector4d &solution, VectorXd &residual);

double angVelocity();
double distance(Vector3d a, Vector3d b);

#endif