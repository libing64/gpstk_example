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

using namespace std;
using namespace gpstk;

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
    Rinex3ObsStream roffs(argv[3]); // Open observations data file
    Rinex3ObsHeader roh;
    Rinex3ObsData rod;

    // Let's read the header
    roffs >> roh;
    roh.dump(cout);

    // The following lines fetch the corresponding indexes for some
    // observation types we are interested in. Given that old-style
    // observation types are used, GPS is assumed.
    int indexP1 = roh.getObsIndex("C1C");
    int indexC1 = roh.getObsIndex("L1C");

    cout << "indexP1: " << indexP1 << endl;
    cout << "indexC1: " << indexC1 << endl;


    // Let's process all lines of observation data, one by one
    while (roffs >> rod)
    {

        if (rod.epochFlag == 0 || rod.epochFlag == 1) // Begin usable data
        {
            int cnt;
            cin >> cnt;
            cout << "cnt: " << cnt << endl;
            vector<SatID> prnVec;
            vector<double> rangeVec;

            // Define the "it" iterator to visit the observations PRN map.
            // Rinex3ObsData::DataMap is a map from RinexSatID to
            // vector<RinexDatum>:
            //      std::map<RinexSatID, vector<RinexDatum> >
            Rinex3ObsData::DataMap::const_iterator it;

            // This part gets the PRN numbers and ionosphere-corrected
            // pseudoranges for the current epoch. They are correspondly fed
            // into "prnVec" and "rangeVec"; "obs" is a public attribute of
            // Rinex3ObsData to get the map of observations
            for (it = rod.obs.begin(); it != rod.obs.end(); it++)
            {
                // The RINEX file may have P1 observations, but the current
                // satellite may not have them.
                double P1(0.0);
                try
                {
                    P1 = rod.getObs(it->first, indexP1).data;
                }
                catch (...)
                {
                    continue;
                }

                double C1(0.0);
                try
                {
                    C1 = rod.getObs(it->first, indexC1).data;
                }
                catch (...)
                {
                    continue;
                }

                cout << "P1: " << P1 << "   C1:" << C1 << endl;
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
