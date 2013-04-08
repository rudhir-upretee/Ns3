#include <iostream>
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/NetsimTraciClient.h"
#include "ns3/sumo-mobility-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VanetScript");

///////////////////////////////////////////////////////////////////////////////
// Global Constants
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Forward Declarations
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Main Script
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
    {
    //
    // Default initialization values
    int nWifi = 2;
    int traciPort = 50000;
    string traciHost = "localhost";
    int simulatorStartTime = 0;
    int simulatorStopTime = 0;

    CommandLine cmd;
    cmd.AddValue("nWifi", "Number of wifi STA devices", nWifi);
    cmd.AddValue("traciPort", "TRACI Server Port", traciPort);
    cmd.AddValue("traciHost", "TRACI Server Host", traciHost);
    cmd.AddValue("simulatorStopTime", "Simulator stop time", simulatorStopTime);
    cmd.Parse(argc, argv);

    ///////////////////////////////////////////////////////////////////////////
    //
    // Configure NS3 Nodes
    //
    // Create Nodes. Configure Wifi PHY and attach it with Wifi channel.
    // Create Wifi MAC and configure it to work in Ad-hoc mode. Create and
    // install the the configured PHY and MAC into a NetDevice. Attach the
    // NetDevice to a Node.
    //
    ///////////////////////////////////////////////////////////////////////////
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nWifi);

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiHelper wifi = WifiHelper::Default();
    NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("OfdmRate54Mbps"));
    wifiMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer staDevices;
    staDevices = wifi.Install(wifiPhy, wifiMac, wifiStaNodes);

    //////////////////////////////////////////////////////////////////////////
    //
    // Configure IP Stack
    //
    //////////////////////////////////////////////////////////////////////////
    InternetStackHelper stack;
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.3.0", "255.255.255.0");

    Ipv4InterfaceContainer wifiInterfaces;
    wifiInterfaces = address.Assign(staDevices);

    //////////////////////////////////////////////////////////////////////////
    //
    // Configure Wifi mobility
    //
    //////////////////////////////////////////////////////////////////////////

    // Create vehicle state table
    MSVehicleStateTable* ptrVehStateTable = new MSVehicleStateTable();
    ptrVehStateTable->testFillVSTable();

    SumoMobilityHelper sumoMobility = SumoMobilityHelper(traciPort,
                                                         traciHost,
                                                         ptrVehStateTable,
                                                         54000+simulatorStartTime,
                                                         54000+simulatorStopTime);

    //////////////////////////////////////////////////////////////////////////
    //
    // Configure Applications to be installed on the nodes
    //
    //////////////////////////////////////////////////////////////////////////
    ApplicationContainer appContainerVanetApp;

    //
    // Configure vanetApp application. Set remote address as broadcast address.
    // Set port number as 1025
    //
    VanetMonitorHelper vanetApp("ns3::UdpSocketFactory",
                                InetSocketAddress("255.255.255.255", 1025));

    UniformVariable randVarTime(0, 1);
    for (int i = 0; i < nWifi; i++)
        {
        appContainerVanetApp = vanetApp.Install(wifiStaNodes.Get(i));
        appContainerVanetApp.Start(Seconds(randVarTime.GetValue()));
        appContainerVanetApp.Stop(Seconds(simulatorStopTime));
        }

    // Trace sink should be attached after the source has been initialized.
    // Trace source are in VanetMonitorApplication. So this should be
    // done after the application initialization.
    sumoMobility.HookAppCallbacks();

    //
    // Start simulation
    //
    NS_LOG_DEBUG ("Simulation Started......................................|");
    Simulator::Stop(Seconds(simulatorStopTime));
    Simulator::Run();
    Simulator::Destroy();

    NS_LOG_DEBUG ("Simulation Completed....................................|");

    return 0;
    }

