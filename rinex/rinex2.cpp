#include "RinexObsBase.hpp"
#include "RinexObsData.hpp"
#include "RinexObsHeader.hpp"
#include "RinexObsStream.hpp"
#include "CivilTime.hpp"
#include "GNSSconstants.hpp"
#include <iostream>

using namespace std;
using namespace gpstk;

int main(int argc, char *argv[])
{
    RinexObsStream roffs(argv[1]);//for example bahr1620.o4o
    RinexObsHeader roh;
    RinexObsData roe;
    RinexDatum dataobj;

    double gamma = (L1_FREQ_GPS / L2_FREQ_GPS) * (L1_FREQ_GPS / L2_FREQ_GPS);

    int m_prn = 14;
    SatID prn(m_prn, SatID::systemGPS);

    // Read the RINEX header (don't skip this step)
    roffs >> roh;

    // Print RINEX header to terminal screen
    roh.dump(cout);

    while (roffs >> roe)
    {

        CivilTime civtime(roe.time);

        RinexObsData::RinexSatMap::iterator pointer = roe.obs.find(prn);
        if (pointer != roe.obs.end())
        {
            // Get P1, P2 and L1 observations
            dataobj = (*pointer).second[RinexObsHeader::P1];
            double P1 = dataobj.data;
            double P2 = (*pointer).second[RinexObsHeader::P2].data;
            cout << "P1: " << P1 << "   P2:  " << P2 << endl;

            double L1 = pointer->second[RinexObsHeader::L1].data;
            // Compute multipath
            double mu = P1 - L1 * (C_MPS / L1_FREQ_GPS) - 2.0 * (P1 - P2) / (1 - gamma);

            cout << " PRN " << m_prn << " biased multipath " << mu << endl;
        }
    }
    cout << "Read " << roffs.recordNumber << " epochs.  Cheers." << endl;
    return 0;
}
