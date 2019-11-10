
#include <iostream>
#include "ExtractData.hpp"
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"
#include "Rinex3NavStream.hpp"

// Class to store satellite broadcast navigation data
#include "GPSEphemerisStore.hpp"
#include "Rinex3EphemerisStore.hpp"

// Time-class year-day-second
#include "YDSTime.hpp"

using namespace std;
using namespace gpstk;


int main(int argc, char *argv[])
{
    Rinex3NavStream rnffs(argv[1]);   // Object to read Rinex navigation data files
    Rinex3NavData rne;     // Object to store Rinex navigation data
    Rinex3NavHeader rnh; // Object to read the header of Rinex

    rnffs >> rnh;

    GPSEphemerisStore ephStore;

    SatID prn(5, SatID::systemGPS);
    // rnh.dump();
    while(rnffs >> rne)
    {
        CivilTime civtime(rne.time);
        cout << "civtime: " << civtime << endl;
        ephStore.addEphemeris(rne);
        ephStore.SearchUser();
        //计算卫星的位置，速度和时钟偏差
        Xvt sat_xvt = ephStore.getXvt(prn, rne.time);
        cout << "satellite pos:" << sat_xvt.x << endl;
        cout << "satellite vel:" << sat_xvt.v << endl;
        cout << "satellite clock bias: " << sat_xvt.clkbias << endl;
        cout << "satellite clock drift: " << sat_xvt.clkdrift << endl;
    }
    cout << "nav msg numbers: " << rnffs.recordNumber << endl;
    return 0;
}
