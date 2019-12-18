#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <cfloat>

// Classes for handling observations RINEX files (data)
#include "Rinex3ObsHeader.hpp"
#include "Rinex3ObsData.hpp"
#include "Rinex3ObsStream.hpp"

// Classes for handling satellite navigation parameters RINEX
// files (ephemerides)
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"
#include "Rinex3NavStream.hpp"

// Classes for handling RINEX files with meteorological parameters
#include "RinexMetBase.hpp"
#include "RinexMetData.hpp"
#include "RinexMetHeader.hpp"
#include "RinexMetStream.hpp"

// Class for handling tropospheric model
#include "GGTropModel.hpp"

// Class for storing "broadcast-type" ephemerides
#include "GPSEphemerisStore.hpp"

// Class for handling RAIM
#include "PRSolution2.hpp"

// Class defining GPS system constants
#include "GNSSconstants.hpp"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "decorr.h"
#include "pvt.h"

using namespace std;
using namespace gpstk;
using namespace Eigen;


typedef struct 
{
    double P1;
    double P2;
    double C1;
    double C2;
    float  pos[3];
    float  pos_station[3];
    Xvt sat_xvt;
    SatID prn;
} rtk_obs_t;


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

void decorrelation(const MatrixXd Qn, MatrixXd& Q_decorr, MatrixXd& Zt, MatrixXd& iZt)
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

void lambda_search(MatrixXd Qz, VectorXd z, VectorXd& z_fixed)
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
void lambda_solver(MatrixXd Qa, VectorXd a, VectorXd& a_fixed)
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
    const double Pcov = 2.0;
    const double Ccov = 0.01;
    Eigen::MatrixXd Q = MatrixXd::Zero(2 * n, 2 * n); //covariance matrix of raw gps measurement
    for (int i = 0; i < n; i++)
    {
        Q(2 * i, 2 * i) = Ccov;         //covariance of Pseudorange measurement
        Q(2 * i + 1, 2 * i + 1) = Pcov; //covariance of carrier phase measurement
    }
    //cout << "line: " << __LINE__ << endl;
    //compute cov of difference
    H_dd = MatrixXd::Zero(2 * n - 2, 2 * n);
    for (int i = 0; i < (n - 1); i++)
    {
        H_dd(2 * i, 2 * i) = 1;
        H_dd(2 * i, 2 * i + 2) = -1;

        H_dd(2 * i + 1, 2 * i + 1) = 1;
        H_dd(2 * i + 1, 2 * i + 3) = -1;
    }
    //cout << "line: " << __LINE__ << endl;
    Q_dd = H_dd * Q * H_dd.transpose();
    //cout << "Q: " << Q << endl;
    //cout << "Q_dd: " << Q_dd << endl;
    //cout << "line: " << __LINE__ << endl;
    W_dd = Q_dd.inverse();
    //cout << "W_dd " << W_dd << endl;
}

//(-3976219.5082, 3382372.5671, 3652512.9849).
//(-3978242.4348, 3382841.1715, 3649902.7667)
//measurement [c1, p1, c2, p2, ...cn, pn] -> double difference
void rtk_solver(vector<rtk_obs_t> &rtk_obs)
{
    int n = rtk_obs.size();
    cout << "rtk solver: " << n << endl;
    int rows = 2 * n - 2;//all the measurements converted to double difference
    int cols = n - 1 + 3;//all the variables to be solved, dx, dy, dz and (n-1) ambiguity resolution
    MatrixXd H = MatrixXd::Zero(rows, cols);
    VectorXd y = VectorXd::Zero(rows);

    MatrixXd H_dd, W_dd, Q_dd;
    covariance_matrix(n, H_dd, Q_dd, W_dd);
    //position of two receivers
    Vector3d pos1 = Vector3d(-3976219.5082, 3382372.5671, 3652512.9849);//station
    Vector3d pos2 = Vector3d(-3978242.4348, 3382841.1715, 3649902.7667);//receiver

  
    //computer Qx and Qn
    for (int i = 0; i < (n - 1); i++)
    {
        rtk_obs_t obs1 = rtk_obs[i];
        rtk_obs_t obs2 = rtk_obs[i + 1];
        double dd_c = (obs1.C1 - obs1.C2) - (obs2.C1 - obs2.C2);//double difference
        double dd_p = (obs1.P1 - obs1.P2) - (obs2.P1 - obs2.P2); //double difference

        y(2 * i) = dd_c;
        y(2 * i + 1) = dd_p;

        Vector3d I1, I2;
        Triple sat_pos1 = obs1.sat_xvt.getPos();
        Triple sat_pos2 = obs2.sat_xvt.getPos();

        for (int i = 0; i < 3; i++)
        {
            I1(i) = sat_pos1[i] - pos1(i);
            I2(i) = sat_pos2[i] - pos1(i);
        }
        I1.normalize();
        I2.normalize();
        
        H.block(2 * i,     0, 1, 3) = (I1 - I2).transpose();
        H(2 * i, 3 + i) = L1_WAVELENGTH_GPS;
        H.block(2 * i + 1, 0, 1, 3) = (I1 - I2).transpose();
    }
    //cout << "H: " << H << endl;
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

    if (res.norm() / (2 * n) > 10.0)
    {
        cout << "lambda solve failed!!!!!  " << endl; 
    }
    Vector3d baseline = pos1 - pos2;
    cout << "baseline: " << baseline.transpose() << endl;

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
    MatrixXd Qn = Q_xn.block(3, 3,  n - 1, n - 1);
    //cout << "Qn: " << Qn << endl;
    VectorXd N = x.segment(3, n - 1);
    VectorXd N_fixed;
    lambda_solver(Qn, N, N_fixed);


    //solve (dx, dy, dz) with ambigity resolution being fixed
    int m = x.rows();//length of x
    //cout << "n: " << n << endl;
    //cout << "m: " << m << endl;
    VectorXd xx = VectorXd::Zero(m);
    xx.segment(3, n-1) = N_fixed;
    MatrixXd HH = H.block(0, 0, 2 * n - 2, 3);
    VectorXd yy = y - H * xx;//after ambiguity being removed

    MatrixXd HHt = HH.transpose();
    VectorXd x2 = (HHt * W_dd * HH).inverse() * HHt * W_dd * yy;
    cout << "yy: " << yy.transpose() << endl;
    cout << "x2: " << x2.transpose() << endl;
}

int main(int argc, char *argv[])
{

    // Declaration of objects for storing ephemerides and handling RAIM
    GPSEphemerisStore bcestore;

    // This verifies the ammount of command-line parameters given and
    // prints a help message, if necessary
    if (argc != 4)
    {
        cerr << "Usage:" << endl;
        cout << "   " << argv[0]
             << " <RINEX Obs file>  <RINEX Nav file>  <RINEX Obs file(station)>"
             << endl;

        exit(-1);
    }

    // Read nav file and store unique list of ephemerides
    Rinex3NavStream rnffs(argv[2]); // Open ephemerides data file
    Rinex3NavData rne;
    Rinex3NavHeader hdr;

    // Let's read the header (may be skipped)
    rnffs >> hdr;

    // Storing the ephemeris in "bcstore"
    while (rnffs >> rne)
        bcestore.addEphemeris(rne);

    // Setting the criteria for looking up ephemeris
    bcestore.SearchNear();


    // Open and read the observation file one epoch at a time.
    // For each epoch, compute and print a position solution
    Rinex3ObsStream roffs(argv[1]); // Open observations data file
    Rinex3ObsHeader roh;
    Rinex3ObsData rod;

    Rinex3ObsStream roffs_station(argv[3]); // Open observations data file
    Rinex3ObsHeader roh_station;
    Rinex3ObsData rod_station;

    // Let's read the header
    cout << "Rinex Header" << endl;
    roffs >> roh;
    roh.dump(cout);

    cout << "Rinex Header for station" << endl;
    roffs_station >> roh_station;
    roh_station.dump(cout);

    // The following lines fetch the corresponding indexes for some
    // observation types we are interested in. Given that old-style
    // observation types are used, GPS is assumed.
    int indexP1 = roh.getObsIndex("C1C");
    int indexC1 = roh.getObsIndex("L1C");

    int indexP1_station = roh_station.getObsIndex("C1C");
    int indexC1_station = roh_station.getObsIndex("L1C");

    // cout << "indexP1: " << indexP1 << endl;
    // cout << "indexC1: " << indexC1 << endl;


    // Let's process all lines of observation data, one by one
    while (roffs >> rod  && roffs_station >> rod_station) 
    {
        cout << "receiver and station time: " << rod.time << "  " << rod_station.time << endl; 
        if (rod.epochFlag == 0 || rod.epochFlag == 1) // Begin usable data
        {
            //int cnt;
            // cin >> cnt;
            // cout << "cnt: " << cnt << endl;
            vector<SatID> prnVec;
            vector<double> rangeVec;
            vector<rtk_obs_t> rtk_obs_q;
            vector<pvt_obs_t> pvt_obs_q;

            // Define the "it" iterator to visit the observations PRN map.
            // Rinex3ObsData::DataMap is a map from RinexSatID to
            // vector<RinexDatum>:
            //      std::map<RinexSatID, vector<RinexDatum> >
            Rinex3ObsData::DataMap::const_iterator it;

            // This part gets the PRN numbers and ionosphere-corrected
            // pseudoranges for the current epoch. They are correspondly fed
            // into "prnVec" and "rangeVec"; "obs" is a public attribute of
            // Rinex3ObsData to get the map of observations
            double P1(0.0), C1(0.0);
            double P1_station(0.0), C1_station(0.0);
            RinexSatID prn;

            rtk_obs_t rtk_obs;
            pvt_obs_t pvt_obs;
            for (it = rod.obs.begin(); it != rod.obs.end(); it++)
            {
                // The RINEX file may have P1 observations, but the current
                // satellite may not have them.
                try
                {
                    prn = it->first;
                    P1 = rod.getObs(it->first, indexP1).data;
                    C1 = rod.getObs(it->first, indexC1).data;

                    Rinex3ObsData::DataMap::const_iterator it_station;
                    it_station = rod_station.obs.find(it->first);
                    if (it_station == rod_station.obs.end())
                    {
                        cout << "sat " << it->first << "  is not observed in station" << endl;
                        continue;
                    } else 
                    {
                        P1_station = rod_station.getObs(it->first, indexP1_station).data;
                        C1_station = rod_station.getObs(it->first, indexC1_station).data;

                        //compute sat pos
                        Xvt sat_xvt = bcestore.getXvt(prn, rod.time);
                        // cout << "satellite pos:" << sat_xvt.x << endl;
                        // cout << "satellite vel:" << sat_xvt.v << endl;
                        // cout << "satellite clock bias: " << sat_xvt.clkbias << endl;
                        // cout << "satellite clock drift: " << sat_xvt.clkdrift << endl;

                        rtk_obs.P1 = P1;
                        rtk_obs.P2 = P1_station;
                        rtk_obs.C1 = C1;
                        rtk_obs.C2 = C1_station;
                        rtk_obs.sat_xvt = sat_xvt;
                        rtk_obs.prn = prn;
                        rtk_obs_q.push_back(rtk_obs);

                        pvt_obs.P = P1;
                        Triple sat_pos = sat_xvt.getPos();
                        pvt_obs.sat_pos(0) = sat_pos[0];
                        pvt_obs.sat_pos(1) = sat_pos[1];
                        pvt_obs.sat_pos(2) = sat_pos[2];
                        pvt_obs_q.push_back(pvt_obs);
                    }
                }
                catch (...)
                {
                    continue;
                }
            }
            //rtk_solver(rtk_obs_q);
            pvt_solver(pvt_obs_q);
        }

        
    }

    return 0;
}
