#include "Rinex3ObsBase.hpp"
#include "Rinex3ObsData.hpp"
#include "Rinex3ObsHeader.hpp"
#include "Rinex3ObsStream.hpp"
#include "CivilTime.hpp"
#include "GNSSconstants.hpp"
#include <iostream>

using namespace std;
using namespace gpstk;

int main(int argc, char *argv[])
{
    int myprn;
    if (argc < 2) //madr1480.08o prn = 6
    {
        cout << "Required argument is a RINEX obs file." << endl;
        exit(-1);
    }

    double gamma = (L1_FREQ_GPS / L2_FREQ_GPS) * (L1_FREQ_GPS / L2_FREQ_GPS);

    cout << "Reading " << argv[1] << "." << endl;
    Rinex3ObsStream roffs(argv[1]);
    Rinex3ObsHeader roh;
    Rinex3ObsData roe;
    RinexDatum dataobj;

    roffs >> roh;
    roh.dump(cout);

    cout << "Name your PRN of interest (by number: 1 through 32): ";
    cin >> myprn;

    int indexP1(roh.getObsIndex("C1W"));
    int indexP2(roh.getObsIndex("C2W"));
    int indexL1(roh.getObsIndex("L1C"));

    while (roffs >> roe)
    {

        // Let's use the CivilTime class to print time
        CivilTime civtime(roe.time);
        SatID prn(myprn, SatID::systemGPS);

        Rinex3ObsData::DataMap::iterator pointer = roe.obs.find(prn);
        if (pointer == roe.obs.end())
        {
            cout << "PRN " << myprn << " not in view " << endl;
        }
        else
        {
            // Get P1, P2 and L1 observations
            dataobj = roe.getObs(prn, indexP1);
            double P1 = dataobj.data;

            dataobj = roe.getObs(prn, indexP2);
            double P2 = dataobj.data;

            dataobj = roe.getObs(prn, indexL1);
            double L1 = dataobj.data;

            // Compute multipath
            double mu = P1 - L1 * (C_MPS / L1_FREQ_GPS) - 2 * (P1 - P2) / (1 - gamma);

            cout << " PRN " << myprn << " biased multipath " << mu << endl;
        }
    }

    cout << "Read " << roffs.recordNumber << " epochs.  Cheers." << endl;
    return 0;
}
