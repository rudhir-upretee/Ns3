#include <iostream>
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VanetApp");

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
    uint32_t nWifi = 2;
    double simulatorStopTime = 101.0;
    double vanetAppStartTime = 0.0;
    double vanetAppStopTime = 100.0;

    CommandLine cmd;
    cmd.AddValue("nWifi", "Number of wifi STA devices", nWifi);
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
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
            StringValue("OfdmRate54Mbps"));
    NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
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
    Ns2MobilityHelper ns2 = Ns2MobilityHelper("TraceFile");
    ns2.Install();

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
    for (uint32_t i = 0; i < nWifi; i++)
        {
        appContainerVanetApp = vanetApp.Install(wifiStaNodes.Get(i));
        appContainerVanetApp.Start(Seconds(vanetAppStartTime));
        appContainerVanetApp.Stop(Seconds(vanetAppStopTime));
        }

    // Trace sink should be attached after the source has been initialized.
    // Trace source are in VanetMonitorApplication. So this should be
    // done after the application initialization.
    ns2.HookAppCallbacks();

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

