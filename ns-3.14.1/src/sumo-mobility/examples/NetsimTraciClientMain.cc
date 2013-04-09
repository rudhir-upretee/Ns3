/****************************************************************************/
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#if 0
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include "config.h"
#endif
#endif

#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>

#if 0
#include "NetsimTraciClient.h"
#include "MSVehicleStateTable.h"
#endif

#include "ns3/MSVehicleStateTable.h"
#include "ns3/NetsimTraciClient.h"

// ===========================================================================
// used namespaces
// ===========================================================================
//using namespace netsimtraciclient;


// ===========================================================================
// method definitions
// ===========================================================================
int main(int argc, char* argv[]) {
    std::string outFileName = "testclient_result.out";
    int port = -1;
    std::string host = "localhost";
    std::string strBeginTime = "", strEndTime = "";

    if ((argc == 1) || (argc % 2 == 0)) {
        std::cout << "Usage: NetsimTraciClient -p <remote port>"
                  << "[-h <remote host>] [-o <outputfile name>]"
                  << "[-b <begin time>] [-e <end time>]"
                  << std::endl;
        return 0;
    }

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg.compare("-o") == 0) {
            outFileName = argv[i + 1];
            i++;
        } else if (arg.compare("-p") == 0) {
            port = atoi(argv[i + 1]);
            i++;
        } else if (arg.compare("-h") == 0) {
            host = argv[i + 1];
            i++;
        } else if (arg.compare("-b") == 0) {
            strBeginTime = argv[i + 1];
            i++;
        } else if (arg.compare("-e") == 0) {
            strEndTime = argv[i + 1];
            i++;
        } else {
            std::cout << "unknown parameter: " << argv[i] << std::endl;
            return 1;
        }
    }

    if (port == -1) {
        std::cout << "Missing port" << std::endl;
        return 1;
    }

    ////////////////////////////////////////////
    // Start Ns3 from the given path
    ///////////////////////////////////////////

    ///////////////////////////////////////////
    // Start sumo with given remote port
    ///////////////////////////////////////////

    ///////////////////////////////////////////
    // Start TraCI Sumo Client
    ///////////////////////////////////////////
    MSVehicleStateTable* ptrVehStateTable = new MSVehicleStateTable();
    NetsimTraciClient client(ptrVehStateTable,
                            string2time(strBeginTime)/1000,
                            string2time(strEndTime)/1000,
                            outFileName);
    return !client.run(port, host);

}
