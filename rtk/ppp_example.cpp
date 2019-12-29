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

// Class to store satellite precise navigation data
#include "SP3EphemerisStore.hpp"

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

int main(int argc, char *argv[])
{

    const double C_MPS = 2.99792458e8;
    const double gamma = (L1_FREQ_GPS / L2_FREQ_GPS) * (L1_FREQ_GPS / L2_FREQ_GPS);

    // Read nav file and store unique list of ephemerides
    Rinex3NavData rne;
    Rinex3NavHeader hdr;

    //load all the sp3 file
    // Declare a "SP3EphemerisStore" object to handle precise ephemeris
    SP3EphemerisStore SP3EphList;
    // Set flags to reject satellites with bad or absent positional
    // values or clocks
    SP3EphList.rejectBadPositions(true);
    SP3EphList.rejectBadClocks(true);
    SP3EphList.loadFile("../igs13354.sp3");
    SP3EphList.loadFile("../igs13355.sp3");
    SP3EphList.loadFile("../igs13356.sp3");

    // Open and read the observation file one epoch at a time.
    // For each epoch, compute and print a position solution
    Rinex3ObsStream roffs("../onsa2240.05o"); // Open observations data file
    Rinex3ObsHeader roh;
    Rinex3ObsData rod;

    // Let's read the header
    cout << "Rinex Header" << endl;
    roffs >> roh;
    roh.dump(cout);

    // The following lines fetch the corresponding indexes for some
    // observation types we are interested in. Given that old-style
    // observation types are used, GPS is assumed.
    int indexP1 = roh.getObsIndex("C1C");
    cout << "indexP1: " << indexP1 << endl;
    int indexP2 = roh.getObsIndex("C2W");

    cout << "indexP2: " << indexP2 << endl;
    // Let's process all lines of observation data, one by one
    while (roffs >> rod)
    {
        if (rod.epochFlag == 0 || rod.epochFlag == 1) // Begin usable data
        {
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
            double P1(0.0), P2(0.0);
            RinexSatID prn;
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
                    double ionocorr = 1.0 / (1.0 - gamma) * (P1 - P2);

                    //compute sat pos, takc care of clock bias
                    Xvt sat_xvt = SP3EphList.getXvt(prn, rod.time);
                    cout << "satellite pos:" << sat_xvt.x << endl;
                    cout << "satellite vel:" << sat_xvt.v << endl;
                    cout << "satellite clock bias: " << sat_xvt.clkbias * C_MPS << endl;
                    cout << "satellite clock drift: " << sat_xvt.clkdrift << endl;

                    pvt_obs.P = P1 - ionocorr + sat_xvt.clkbias * C_MPS;
                    Triple sat_pos = sat_xvt.getPos();
                    pvt_obs.sat_pos(0) = sat_pos[0];
                    pvt_obs.sat_pos(1) = sat_pos[1];
                    pvt_obs.sat_pos(2) = sat_pos[2];
                    pvt_obs_q.push_back(pvt_obs);
                }
                catch (...)
                {
                    continue;
                }
            }
            Vector4d solution;
            VectorXd residual;
            pvt_solver(pvt_obs_q, solution, residual);
            cout << "=================" << endl;
            cout << "solution: " << solution.transpose() << endl;
            cout << "residual: " << residual.transpose() << endl;
            cout << "=================" << endl;
        }
    }

    return 0;
}
