#include <iostream>

// Classes for handling observations RINEX files (data)
#include "Rinex3ObsData.hpp"
#include "Rinex3ObsStream.hpp"

// Class to easily extract data from Rinex3ObsData objects
#include "ExtractData.hpp"

// Classes for handling satellite navigation parameters RINEX files
// (Broadcast ephemerides)
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"
#include "Rinex3NavStream.hpp"

// Class to store satellite broadcast navigation data
#include "GPSEphemerisStore.hpp"

// Class to model GPS data for a mobile receiver
#include "ModeledPR.hpp"

// Class to model the tropospheric delays
#include "TropModel.hpp"

// Classes to model ans store ionospheric delays
#include "IonoModel.hpp"
#include "IonoModelStore.hpp"

// Class to solve the equations system using a Weighted Least Mean Square method
#include "SolverWMS.hpp"

// Class to compute the weights to be used for each satellite
#include "MOPSWeight.hpp"

// Basic framework for programs in the GPSTk. The 'process()' method MUST
// be implemented
#include "BasicFramework.hpp"

#include "GNSSconstants.hpp" // DEG_TO_RAD

// Time-class year-day-second
#include "YDSTime.hpp"

using namespace std;
using namespace gpstk;

// A new class is declared that will handle program behaviour
// This class inherits from BasicFramework
class positioning_algo : public BasicFramework
{
public:
    // Constructor declaration
    positioning_algo(char *arg0);

protected:
    // Method that will take care of processing
    virtual void process();

    // Method that hold code to be run BEFORE processing
    virtual void spinUp();

private:
    // These field represent options at command line interface (CLI)
    CommandOptionWithArg dataFile;
    CommandOptionWithArg navFile;

    // If you want to share objects and variables among methods, you'd
    // better declare them here
    Rinex3ObsStream rObsFile;   // Object to read Rinex observation data files
    Rinex3ObsData rData;        // Object to store Rinex observation data
    Rinex3NavStream rNavFile;   // Object to read Rinex navigation data files
    Rinex3NavData rNavData;     // Object to store Rinex navigation data
    Rinex3NavHeader rNavHeader; // Object to read the header of Rinex
                                // navigation data files
    IonoModelStore ionoStore;   // Object to store ionospheric models
    GPSEphemerisStore bceStore; // Object to store ephemeris
    ModeledPR modelPR;          // Declare a ModeledReferencePR object
    MOPSTropModel mopsTM;       // Declare a MOPSTropModel object
    ExtractData obsC1;          // Declare an ExtractData object
    int indexC1;                // Index to "C1" observation
    bool useFormerPos;          // Flag indicating if we have an a priori
                                // position
    Position formerPosition;    // Object to store the former position
    IonoModel ioModel;          // Declare a Ionospheric Model object
    SolverWMS solver;           // Declare an object to apply WMS method
    MOPSWeight mopsWeights;     // Object to compute satellites' weights
};

// Let's implement constructor details
positioning_algo::positioning_algo(char *arg0)
    : BasicFramework(arg0, ""),
      // Option initialization. "true" means a mandatory option
      dataFile(CommandOption::stdType, 'i', "datainput",
               " [-i|--datainput]      Name of RINEX observations file.", true),
      navFile(CommandOption::stdType, 'n', "navinput",
              " [-n|--navinput]      Name of RINEX broadcast navigation file.", true)
{
    // These options may appear just once at CLI
    dataFile.setMaxCount(1);
    navFile.setMaxCount(1);
} // End of constructor details

// Method that will be executed AFTER initialization but BEFORE processing
void positioning_algo::spinUp()
{

    rObsFile.open(dataFile.getValue()[0], std::ios::in);

    cout << "line: " << __LINE__ << endl;
    // We need to read the header of the observations file
    Rinex3ObsHeader roh;
    rObsFile >> roh;
    cout << "line: " << __LINE__ << endl;
    // We need the index pointing to C1-type observations
    try
    {
        indexC1 = roh.getObsIndex("C1");
    }
    catch (...)
    {
        cerr << "The observation file doesn't have C1 pseudoranges." << endl;
        exit(1);
    }
    cout << "line: " << __LINE__ << endl;

    rNavFile.open(navFile.getValue()[0].c_str(), std::ios::in);
    rNavFile >> rNavHeader;

    // Let's feed the ionospheric model (Klobuchar type) from data in the
    // navigation (ephemeris) file header. First, we must check if there are
    // valid ionospheric correction parameters in the header
    if (rNavHeader.valid & Rinex3NavHeader::validIonoCorrGPS)
    {
        // Extract the Alpha and Beta parameters from the header
        double *ionAlpha = rNavHeader.mapIonoCorr["GPSA"].param;
        double *ionBeta = rNavHeader.mapIonoCorr["GPSB"].param;

        // Feed the ionospheric model with the parameters
        ioModel.setModel(ionAlpha, ionBeta);
    }
    else
    {
        cerr << "WARNING: Navigation file " << navFile.getValue()[0].c_str()
             << " doesn't have valid ionospheric correction parameters." << endl;
    }

    // WARNING-WARNING-WARNING: In this case, the same model will be used
    // for the full data span
    ionoStore.addIonoModel(CommonTime::BEGINNING_OF_TIME, ioModel);

    // Storing the ephemeris in "bceStore"
    while (rNavFile >> rNavData)
        bceStore.addEphemeris(rNavData);

    // Setting the criteria for looking up ephemeris
    bceStore.SearchUser(); // This is the default

    // This is set to true if the former computed positon will be used as
    // a priori position
    useFormerPos = false; // At first, we don't have an a priori position

    // Prepare for printing later on
    cout << fixed << setprecision(8);
}

void positioning_algo::process()
{

    // Let's read the observations RINEX, epoch by epoch
    while (rObsFile >> rData)
    {
        // Begin usable data with enough number of satellites
        if ((rData.epochFlag == 0 || rData.epochFlag == 1) &&
            (rData.numSVs > 3))
        {

            // Number of satellites with valid data in this epoch
            int validSats = 0;
            int prepareResult;
            double rxAltitude; // Receiver altitude for tropospheric model
            double rxLatitude; // Receiver latitude for tropospheric model

            // We need to extract C1 data from this epoch. Skip epoch if not
            // enough data (4 SV at least) is available
            if (obsC1.getData(rData, indexC1) < 4)
            {
                // The former position will not be valid next time
                useFormerPos = false;
                continue;
            }

            // If possible, use former position as a priori
            if (useFormerPos)
            {

                prepareResult = modelPR.Prepare(formerPosition);

                // We need to seed this kind of tropospheric model with
                // receiver altitude
                rxAltitude = formerPosition.getAltitude();
                rxLatitude = formerPosition.getGeodeticLatitude();
            }
            else
            {
                // Use Bancroft method is no a priori position is available
                cerr << "Bancroft method was used at epoch "
                     << static_cast<YDSTime>(rData.time).sod << endl;

                prepareResult = modelPR.Prepare(rData.time,
                                                obsC1.availableSV,
                                                obsC1.obsData,
                                                bceStore);

                // We need to seed this kind of tropospheric model with
                // receiver altitude
                rxAltitude = modelPR.rxPos.getAltitude();
                rxLatitude = modelPR.rxPos.getGeodeticLatitude();
            }

            // If there were problems with Prepare(), skip this epoch
            if (prepareResult)
            {
                // The former position will not be valid next time
                useFormerPos = false;
                continue;
            }

            // If there were no problems, let's feed the tropospheric model
            mopsTM.setReceiverHeight(rxAltitude);
            mopsTM.setReceiverLatitude(rxLatitude);
            mopsTM.setDayOfYear(static_cast<YDSTime>(rData.time).doy);

            // Now, let's compute the GPS model for our observable (C1)
            validSats = modelPR.Compute(rData.time,
                                        obsC1.availableSV,
                                        obsC1.obsData,
                                        bceStore,
                                        &mopsTM,
                                        &ionoStore);

            // Only get into further computations if there are enough
            // satellites
            if (validSats >= 4)
            {
                // Now let's solve the navigation equations using the WMS method
                try
                {
                    // First, compute the satellites' weights
                    int goodSv = mopsWeights.getWeights(rData.time,
                                                        modelPR.availableSV,
                                                        bceStore,
                                                        modelPR.ionoCorrections,
                                                        modelPR.elevationSV,
                                                        modelPR.azimuthSV,
                                                        modelPR.rxPos);

                    // Some minimum checking is in order
                    if (goodSv != (int)modelPR.prefitResiduals.size())
                        continue;

                    // Then, solve the system
                    solver.Compute(modelPR.prefitResiduals,
                                   modelPR.geoMatrix,
                                   mopsWeights.weightsVector);
                }
                catch (InvalidSolver &e)
                {
                    cerr << "Couldn't solve equations system at epoch "
                         << static_cast<YDSTime>(rData.time).sod << endl;
                    cerr << e << endl;

                    // The former position will not be valid next time
                    useFormerPos = false;
                    continue;
                }

                // With "solver", we got the difference vector between the
                // a priori position and the computed, 'real' position. Then,
                // let's convert the solution to a Position object
                Position solPos((modelPR.rxPos.X() + solver.solution[0]),
                                (modelPR.rxPos.Y() + solver.solution[1]),
                                (modelPR.rxPos.Z() + solver.solution[2]));

                // Print results
                CivilTime civtime(rData.time);
                cout << civtime    << "   ";                              // Output field #1
                cout << solPos.X() << "   ";                // Output field #2
                cout << solPos.Y() << "   ";                // Output field #3
                cout << solPos.Z() << "   ";                // Output field #4
                cout << solPos.longitude() << "   ";        // Output field #5
                cout << solPos.geodeticLatitude() << "   "; // Output field #6
                cout << solPos.height() << "   ";           // Output field #7
                cout << endl;

                formerPosition = solPos;

                // Next time, former position will be used as a priori
                useFormerPos = true;

            } // End of 'if( validSats >= 4 )'
            else
            {
                // The former position will not be valid next time
                useFormerPos = false;
            }

        } // End of 'if( (rData.epochFlag == 0 || rData.epochFlag == 1) &&...'
        else
        {
            // The former position will not be valid next time
            useFormerPos = false;
        }
    }

    return;
}

int main(int argc, char *argv[])
{
    positioning_algo program(argv[0]);
    if (!program.initialize(argc, argv))
        return 0;

    if (!program.run())
        return 1;

    return 0;
}
