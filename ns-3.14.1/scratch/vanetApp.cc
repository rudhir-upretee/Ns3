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

NS_LOG_COMPONENT_DEFINE ("VanetApp");

#define NS2_MOBILITY_TRACE


///////////////////////////////////////////////////////////////////////////////
// Global Constants
///////////////////////////////////////////////////////////////////////////////
const int NODE_ID         = 0;           // Node Id index
const int IP_ADDR         = 1;           // IP address index
const int TOT_TX_PKT_CNT  = 2;           // Total packets transmitted cnt index
const int NUM_TUPLES      = 3;
const double rndTimeStartRange = 0.0;
const double rndTimeStopRange  = 1.0;
double lastElapsedSecond       = 0.0;
double appOnOffStartTime       = 0.0;


///////////////////////////////////////////////////////////////////////////////
// Forward Declarations
///////////////////////////////////////////////////////////////////////////////
typedef struct {
	double   startTime;
	double   stopTime;
} nodeTime_t;

nodeTime_t *pNodeTime;

std::map <const uint32_t, uint32_t> IpList;

void printIpAddr          (std::ostream &os, uint32_t address);
void printElapsedTime     ();

///////////////////////////////////////////////////////////////////////////////
// Main Script
///////////////////////////////////////////////////////////////////////////////
int main (int argc, char *argv[])
{
	//
	// Default initialization values
  	uint32_t nWifi = 2;
  	double simulatorStopTime       = 101.0;

  	double appOnOffStopTime        = 100.0;
  	double appPacketSinkStartTime  = 0.0;
  	double appPacketSinkStopTime   = 100.0;

  	CommandLine cmd;
  	cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  	cmd.AddValue ("simulatorStopTime", "Simulator stop time", simulatorStopTime);
  	cmd.Parse (argc,argv);

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
  	wifiStaNodes.Create (nWifi);

  	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  	YansWifiPhyHelper wifiPhy         = YansWifiPhyHelper::Default ();
  	wifiPhy.SetChannel (wifiChannel.Create ());

  	WifiHelper wifi       = WifiHelper::Default ();
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue ("OfdmRate54Mbps"));
  	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  	wifiMac.SetType ("ns3::AdhocWifiMac");

  	NetDeviceContainer staDevices;
  	staDevices = wifi.Install (wifiPhy, wifiMac, wifiStaNodes);

	//////////////////////////////////////////////////////////////////////////
	//
	// Configure IP Stack
	//
	//////////////////////////////////////////////////////////////////////////
  	InternetStackHelper stack;
  	stack.Install (wifiStaNodes);

  	Ipv4AddressHelper address;
  	address.SetBase ("10.1.3.0", "255.255.255.0");

  	Ipv4InterfaceContainer wifiInterfaces;
  	wifiInterfaces = address.Assign (staDevices);

	//////////////////////////////////////////////////////////////////////////
	//
	// Configure Wifi mobility
	//
	//////////////////////////////////////////////////////////////////////////
  	Ns2MobilityHelper ns2 = Ns2MobilityHelper ("TraceFile");
  	ns2.Install ();

    //////////////////////////////////////////////////////////////////////////
    //
    // Configure Applications to be installed on the nodes
    //
    //////////////////////////////////////////////////////////////////////////
    ApplicationContainer appOnoff;
    ApplicationContainer appSink;

    //
    // Configure onoff application. Set remote address as broadcast address.
    // Set port number as 1025
    //
    OnOffHelper onoff ("ns3::UdpSocketFactory",
                       InetSocketAddress ("255.255.255.255", 1025));
    onoff.SetAttribute ("OnTime", StringValue ("Constant:1.0"));
    onoff.SetAttribute ("OffTime", StringValue ("Constant:0.0"));


    //
    // Configure sink application. Set host address as this address.
    // Set port number as 1025
    //
    PacketSinkHelper sink ("ns3::UdpSocketFactory",
  			               InetSocketAddress ("0.0.0.0", 1025));


	UniformVariable randVarTime(0,1);
    appOnOffStopTime        = simulatorStopTime - 1;
    appPacketSinkStopTime   = simulatorStopTime - 1;

    for(uint32_t i = 0; i < nWifi; i++) {
    	appOnoff = onoff.Install (wifiStaNodes.Get(i));
    	appSink = sink.Install (wifiStaNodes.Get(i));

		appOnoff.Start (Seconds (randVarTime.GetValue()));
		appOnoff.Stop (Seconds (appOnOffStopTime));

    	appSink.Start (Seconds (appPacketSinkStartTime));
		appSink.Stop (Seconds (appPacketSinkStopTime));

    }

  	//
  	// Start simulation
  	//
    NS_LOG_DEBUG ("Simulation Started......................................|");
	Simulator::Stop (Seconds (simulatorStopTime));
	Simulator::Run ();
  	Simulator::Destroy ();

	//////////////////////////////////////////////////////////////////////////
	//
	// Analyze result
	//
	//////////////////////////////////////////////////////////////////////////
  	NS_LOG_DEBUG ("Simulation Completed....................................|");

  	//////////////////////////////////////////////////////////////////////////
  	//
  	// Clean ups
  	//
  	//////////////////////////////////////////////////////////////////////////
  	return 0;

}

///////////////////////////////////////////////////////////////////////////////
//
// Print IP address in xxx.xxx.xxx.xxx format.
//
///////////////////////////////////////////////////////////////////////////////
void printIpAddr (std::ostream &os, uint32_t address)
{
	os << ((address >> 24) & 0xff) << "."
       << ((address >> 16) & 0xff) << "."
       << ((address >> 8) & 0xff) << "."
       << ((address >> 0) & 0xff);
}

///////////////////////////////////////////////////////////////////////////////
//
// Print Elapsed Time Every Second.
//
// This function is called roughly every second because it depends on the
// frequency of notifyPhyStateChange() trace call-back. Not very accurate.
//
//////////////////////////////////////////////////////////////////////////////
void printElapsedTime ()
{
	if((((Simulator::Now ()).GetSeconds()) - lastElapsedSecond) >= 1) {
		lastElapsedSecond = (Simulator::Now ()).GetSeconds();
		NS_LOG_DEBUG ("Seconds elapsed : " << lastElapsedSecond);
	}
}

