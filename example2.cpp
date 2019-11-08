//==============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 3.0 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//
//  Copyright 2004-2019, The University of Texas at Austin
//
//==============================================================================

//==============================================================================
//
//  This software developed by Applied Research Laboratories at the University of
//  Texas at Austin, under contract to an agency or agencies within the U.S.
//  Department of Defense. The U.S. Government retains all rights to use,
//  duplicate, distribute, disclose, or release this software.
//
//  Pursuant to DoD Directive 523024
//
//  DISTRIBUTION STATEMENT A: This software has been approved for public
//                            release, distribution is unlimited.
//
//==============================================================================

#include <iostream>
#include <iomanip>

#include "Rinex3ObsBase.hpp"
#include "Rinex3ObsHeader.hpp"
#include "Rinex3ObsData.hpp"
#include "Rinex3ObsStream.hpp"

using namespace std;
using namespace gpstk;

int main(void)
{

    // Create the input file stream
    Rinex3ObsStream rin("bahr1620.04o");

    // Create the output file stream
    Rinex3ObsStream rout("bahr1620.04o.new", ios::out | ios::trunc);

    // Read the RINEX header
    Rinex3ObsHeader head; //RINEX header object
    rin >> head;
    rout.header = rin.header;

    //add some print
    cout << endl
         << head.hsRunBy << "  :PGM / RUN BY / DATE";
    cout << endl
         << head.hsVersion << "  :RINEX VERSION / TYPE";
    cout << endl
         << head.hsComment << "   :COMMENT";
    cout << endl
         << head.hsMarkerName << "  :MARKER NAME";
    cout << endl
         << head.hsMarkerNumber << "  :MARKER NUMBER";
    cout << endl
         << head.hsMarkerType << "   :MARKER TYPE";
    cout << endl
         << head.hsObserver << "       :OBSERVER / AGENCY";
    cout << endl
         << head.hsReceiver << "       :REC # / TYPE / VERS";
    cout << endl
         << head.hsAntennaType << "     :ANT # / TYPE";
    cout << endl
         << head.hsAntennaPosition << "     :APPROX POSITION XYZ";
    cout << endl
         << head.hsAntennaDeltaHEN << "     :ANTENNA: DELTA H/E/N";
    cout << endl
         << head.hsAntennaDeltaXYZ << "     :ANTENNA: DELTA X/Y/Z";
    cout << endl
         << head.hsAntennaPhaseCtr << "     :ANTENNA: PHASECENTER";
    cout << endl
         << head.hsAntennaBsightXYZ << "     :ANTENNA: B.SIGHT XYZ";
    cout << endl
         << head.hsAntennaZeroDirAzi << "     :ANTENNA: ZERODIR AZI";
    cout << endl
         << head.hsAntennaZeroDirXYZ << "     :ANTENNA: ZERODIR XYZ";
    cout << endl
         << head.hsCenterOfMass << "     :CENTER OF MASS: XYZ";
    cout << endl
         << head.hsNumObs << "     :# / TYPES OF OBSERV";
    cout << endl
         << head.hsSystemNumObs << "     :SYS / # / OBS TYPES";
    cout << endl
         << head.hsWaveFact << "     :WAVELENGTH FACT L1/2";
    cout << endl
         << head.hsSigStrengthUnit << "     :SIGNAL STRENGTH UNIT";
    cout << endl
         << head.hsInterval << "     :INTERVAL";
    cout << endl
         << head.hsFirstTime << "     :TIME OF FIRST OBS";
    cout << endl
         << head.hsLastTime << "     :TIME OF LAST OBS";
    cout << endl
         << head.hsReceiverOffset << "     :RCV CLOCK OFFS APPL";
    cout << endl
         << head.hsSystemDCBSapplied << "     :SYS / DCBS APPLIED";
    cout << endl
         << head.hsSystemPCVSapplied << "     :SYS / PCVS APPLIED";
    cout << endl
         << head.hsSystemScaleFac << "     :SYS / SCALE FACTOR";
    cout << endl
         << head.hsSystemPhaseShift << "     :SYS / PHASE SHIFT";
    cout << endl
         << head.hsGlonassSlotFreqNo << "     :GLONASS SLOT / FRQ #";
    cout << endl
         << head.hsGlonassCodPhsBias << "     :GLONASS COD/PHS/BIS";
    cout << endl
         << head.hsLeapSeconds << "     :LEAP SECONDS";
    cout << endl
         << head.hsNumSats << "     :# OF SATELLITES";
    cout << endl
         << head.hsPrnObs << "     :PRN / # OF OBS";
    cout << endl
         << head.hsEoH << "     :END OF HEADER";

    rout << rout.header;

    // Loop over all data epochs
    Rinex3ObsData data; //RINEX data object
    while (rin >> data)
    {
        cout << "time:  " <<  data.time << endl;
        rout << data;
    }

    exit(0);
}
