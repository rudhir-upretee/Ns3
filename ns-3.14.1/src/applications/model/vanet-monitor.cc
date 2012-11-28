/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
//
// Copyright (c) 2006 Georgia Tech Research Corporation
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Author: George F. Riley<riley@ece.gatech.edu>
//
// ns3 - On/Off Data Source Application class
// George F. Riley, Georgia Tech, Spring 2007
// Adapted from ApplicationOnOff in GTNetS.
#include "ns3/log.h"
#include "ns3/address.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/random-variable.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "vanet-monitor.h"

NS_LOG_COMPONENT_DEFINE("VanetMonitorApplication");

using namespace std;

namespace ns3
    {

    NS_OBJECT_ENSURE_REGISTERED(VanetMonitorApplication);

    TypeId VanetMonitorApplication::GetTypeId(void)
        {
        static TypeId tid = TypeId("ns3::VanetMonitorApplication")
            .SetParent<Application>()
            .AddConstructor<VanetMonitorApplication>()
            .AddAttribute("Remote", "The address of the destination",
                          AddressValue(),
                          MakeAddressAccessor(&VanetMonitorApplication::m_peer),
                          MakeAddressChecker())
            .AddAttribute("Protocol", "The type of protocol to use.",
                          TypeIdValue(UdpSocketFactory::GetTypeId()),
                          MakeTypeIdAccessor(&VanetMonitorApplication::m_tid),
                          MakeTypeIdChecker())
            .AddTraceSource("SumoCmd", "Send command to SUMO/Traci",
                            MakeTraceSourceAccessor(
                                &VanetMonitorApplication::m_sumoCmdTrace));
        return tid;
        }

    VanetMonitorApplication::VanetMonitorApplication()
        {
        NS_LOG_FUNCTION_NOARGS ();
        m_socket = 0;
        m_connected = false;
        }

    VanetMonitorApplication::~VanetMonitorApplication()
        {
        NS_LOG_FUNCTION_NOARGS ();
        }

    void VanetMonitorApplication::DoDispose(void)
        {
        NS_LOG_FUNCTION_NOARGS ();

        m_socket = 0;
        // chain up
        Application::DoDispose();
        }

    // Application Methods
    void VanetMonitorApplication::StartApplication()
        {
        NS_LOG_FUNCTION_NOARGS ();

        // Create the socket if not already
        if (!m_socket)
            {
            m_socket = Socket::CreateSocket(GetNode(), m_tid);
            m_socket->Bind();
            m_socket->Connect(m_peer);
            m_socket->SetAllowBroadcast(true);
            m_socket->ShutdownRecv();
            }
        // Insure no pending event
        //CancelEvents ();
        // If we are not yet connected, there is nothing to do here
        // The ConnectionComplete upcall will start timers at that time
        //if (!m_connected) return;
        ScheduleStartEvent();
        }

    void VanetMonitorApplication::ScheduleStartEvent()
        {  // Schedules the event to start sending data (switch to the "On" state)
        NS_LOG_FUNCTION_NOARGS ();

        //Time offInterval = Seconds (m_offTime.GetValue ());
        //NS_LOG_LOGIC ("start at " << offInterval);
        //m_startStopEvent = Simulator::Schedule (offInterval, &VanetMonitorApplication::StartSending, this);
        m_startStopEvent = Simulator::Schedule(Seconds(1.0),
                &VanetMonitorApplication::StartReadingStatus, this);
        }

    void VanetMonitorApplication::StartReadingStatus()
        {
        NS_LOG_FUNCTION_NOARGS ();

        double x = 0.0, y = 0.0, speed = 0.0;
        int id = GetNode()->GetId();
        //m_sumoCmdTrace(id);
        m_sumoCmdTrace(id, &x, &y, &speed);
        //Ptr<Ns2MobilityHelper> obj = GetObject<Ns2MobilityHelper>();
        //obj->GetVehicleStatus(id, &x, &y, &speed);

        NS_LOG_DEBUG ("Node : " << id <<
                " Pos x: " << x <<
                " Pos y: " << y <<
                " Speed: " << speed);
        m_startStopEvent = Simulator::Schedule(Seconds(1.0),
                &VanetMonitorApplication::StartReadingStatus, this);
        }

    void VanetMonitorApplication::StopApplication()
        {
        NS_LOG_FUNCTION_NOARGS ();

        CancelEvents();
        if (m_socket != 0)
            {
            m_socket->Close();
            }
        else
            {
            NS_LOG_WARN ("VanetMonitorApplication found null socket " <<
                         "to close in StopApplication");
            }
        }

    void VanetMonitorApplication::CancelEvents()
        {
        NS_LOG_FUNCTION_NOARGS ();

        Simulator::Cancel(m_startStopEvent);
        }

    }  // Namespace ns3
