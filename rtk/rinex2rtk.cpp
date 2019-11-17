#include <string>
#include <vector>

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

using namespace std;
using namespace gpstk;
using namespace Eigen;


typedef struct 
{
    double P1;
    double P2;
    double C1;
    double C2;
    Xvt sat_xvt;
    SatID prn;
} rtk_obs_t;


void rtk_solver(vector<rtk_obs_t>& rtk_obs)
{
    int n = rtk_obs.size();
    MatrixXd H = MatrixXd(2*n - 2, n - 1 + 3);
    VectorXd y = VectorXd(2 * n - 2);
    for (int i = 0; i < (n - 1); i++)
    {
        rtk_obs_t obs1 = rtk_obs[i];
        rtk_obs_t obs2 = rtk_obs[i + 1];
        double dd_c = (obs1.C1 - obs1.C2) - (obs2.C1 - obs2.C2);//double difference
        double dd_p = (obs1.P1 - obs1.P2) - (obs2.P1 - obs2.P2); //double difference

        y(2 * i) = dd_c;
        y(2 * i + 1) = dd_p;

        //H(2 * i, ) =
        //H(2 * i + 1, ) = 
    }

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

    // Let's compute an useful constant (also found in "GNSSconstants.hpp")
    const double gamma = (L1_FREQ_GPS / L2_FREQ_GPS) * (L1_FREQ_GPS / L2_FREQ_GPS);

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

    cout << "indexP1: " << indexP1 << endl;
    cout << "indexC1: " << indexC1 << endl;


    // Let's process all lines of observation data, one by one
    while (roffs >> rod  && roffs_station >> rod_station) 
    {
        if (rod.epochFlag == 0 || rod.epochFlag == 1) // Begin usable data
        {
            int cnt;
            cin >> cnt;
            cout << "cnt: " << cnt << endl;
            vector<SatID> prnVec;
            vector<double> rangeVec;
            vector<rtk_obs_t> rtk_obs_q;

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
            for (it = rod.obs.begin(); it != rod.obs.end(); it++)
            {
                // The RINEX file may have P1 observations, but the current
                // satellite may not have them.
                try
                {
                    prn = it->first;
                    P1 = rod.getObs(it->first, indexP1).data;
                    C1 = rod.getObs(it->first, indexC1).data;
                    P1_station = rod_station.getObs(it->first, indexP1_station).data;
                    C1_station = rod_station.getObs(it->first, indexC1_station).data;

                    //compute sat pos
                    Xvt sat_xvt = bcestore.getXvt(prn, rod.time);
                    cout << "satellite pos:" << sat_xvt.x << endl;
                    cout << "satellite vel:" << sat_xvt.v << endl;
                    cout << "satellite clock bias: " << sat_xvt.clkbias << endl;
                    cout << "satellite clock drift: " << sat_xvt.clkdrift << endl;


                    rtk_obs.P1 = P1;
                    rtk_obs.P2 = P1_station;
                    rtk_obs.C1 = C1;
                    rtk_obs.C2 = C1_station;
                    rtk_obs.sat_xvt = sat_xvt;
                    rtk_obs.prn = prn;
                    rtk_obs_q.push_back(rtk_obs);
                }
                catch (...)
                {
                    continue;
                }

                cout << "satId:  " << prn << "   P1: " << P1 << "   C1:" << C1 << endl;
                cout << "station:" << prn << "   P1: " << P1_station << "   C1:" << C1_station << endl;
                double ionocorr(0.0);

                // Now, we include the current PRN number in the first part
                // of "it" iterator into the vector holding the satellites.
                // All satellites in view at this epoch that have P1 or P1+P2
                // observations will be included.
                prnVec.push_back(it->first);

                // The same is done for the vector of doubles holding the
                // corrected ranges
                rangeVec.push_back(P1 - ionocorr);

                // WARNING: Please note that so far no further correction
                // is done on data: Relativistic effects, tropospheric
                // correction, instrumental delays, etc.
            }
        }
    }

    return 0;
}
