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

#include "rtk.h"
#include "decorr.h"
#include "pvt.h"

using namespace std;
using namespace gpstk;
using namespace Eigen;





int main(int argc, char *argv[])
{

    const double C_MPS = 2.99792458e8;
    // Declaration of objects for storing ephemerides and handling RAIM
    GPSEphemerisStore bcestore;
    const double gamma = (L1_FREQ_GPS / L2_FREQ_GPS) * (L1_FREQ_GPS / L2_FREQ_GPS);
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
    int indexP2 = roh.getObsIndex("C2W");
    int indexC1 = roh.getObsIndex("L1C");


    int indexP1_station = roh_station.getObsIndex("C1C");
    int indexC1_station = roh_station.getObsIndex("L1C");

    cout << "indexP1: " << indexP1 << endl;
    cout << "indexP2: " << indexP2 << endl;
    cout << "indexC1: " << indexC1 << endl;


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
            double P1(0.0), P2(0.0), C1(0.0);
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
                    P2 = rod.getObs(it->first, indexP2).data;
                    C1 = rod.getObs(it->first, indexC1).data;
                    double ionocorr = 1.0 / (1.0 - gamma) * (P1 - P2);

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

                        Triple sat_pos = sat_xvt.getPos();


                        rtk_obs.P1 = P1 - ionocorr + sat_xvt.clkbias * C_MPS;
                        rtk_obs.P2 = P1_station - ionocorr + sat_xvt.clkbias * C_MPS;
                        rtk_obs.C1 = C1;
                        rtk_obs.C2 = C1_station;
                        rtk_obs.sat_pos(0) = sat_pos[0];
                        rtk_obs.sat_pos(1) = sat_pos[1];
                        rtk_obs.sat_pos(2) = sat_pos[2];
                        //rtk_obs.prn = prn;
                        rtk_obs_q.push_back(rtk_obs);

                        pvt_obs.P = P1_station - ionocorr + sat_xvt.clkbias * C_MPS;
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

            //solve station pos
            Vector4d station_solution;
            VectorXd residual;
            pvt_solver(pvt_obs_q, station_solution, residual);
            cout << "=================" << endl;
            cout << "station_solution: " << station_solution.transpose() << endl;
            cout << "residual: " << residual.transpose() << endl;
            cout << "=================" << endl;

            //solve relative pos
            Vector3d station_pos = station_solution.segment(0, 3);
            //rtk_solver(rtk_obs_q, station_pos);
            single_diff_solver(rtk_obs_q, station_pos);
        }

        
    }

    return 0;
}
