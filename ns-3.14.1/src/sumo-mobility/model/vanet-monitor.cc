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

#include <cstdlib>
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

NS_LOG_COMPONENT_DEFINE("VanetMonitorApp");

using namespace std;

namespace ns3
    {
    typedef struct
        {
        int nodeId;
        double xPos;
        double yPos;
        double speed;
        double posOnLane;
        }PktBuf_t;
    static PktBuf_t packetBuf;

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
            .AddTraceSource("SumoCmdGet", "Send get command to SUMO/Traci",
                            MakeTraceSourceAccessor(
                            &VanetMonitorApplication::m_sumoCmdGetTrace))
            .AddTraceSource("SumoCmdSet", "Send set command to SUMO/Traci",
                            MakeTraceSourceAccessor(
                            &VanetMonitorApplication::m_sumoCmdSetTrace));
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

            int port = InetSocketAddress::ConvertFrom(m_peer).GetPort();
            InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(),
                                                        port);
            m_socket->Bind(local);
            m_socket->Connect(m_peer);
            m_socket->SetAllowBroadcast(true);

            // Hook receive call-back
            m_socket->SetRecvCallback(MakeCallback(&VanetMonitorApplication::RecvPacket,
                                                   this));
            }

        // Insure no pending event
        CancelEvents ();

        ScheduleStartEvent();
        }

    void VanetMonitorApplication::ScheduleStartEvent()
        {
        NS_LOG_FUNCTION_NOARGS ();

        m_startStopEvent = Simulator::Schedule(Seconds(0.01),
                &VanetMonitorApplication::StartMonitorLoop, this);
        }

    void VanetMonitorApplication::StartMonitorLoop()
        {
        NS_LOG_FUNCTION_NOARGS ();

        double xPos = 0.0, yPos = 0.0, speed = -1.0, posOnLane = 0.0;
        int id = GetNode()->GetId();

        /////////////////////////////////////////////////////////
        // Get the nodes state (position + speed)
        /////////////////////////////////////////////////////////
        m_sumoCmdGetTrace(id, &xPos, &yPos, &speed, &posOnLane);
#if 0
        Ptr<ConstantVelocityMobilityModel> model = 0;
        model = GetMobilityModel();
        if (model == 0)
            {
            NS_LOG_DEBUG("Mobility Model not found for ID "<< GetNode()->GetId());
            }
        else
            {
            Vector vel = model->GetVelocity();
            double xVel = vel.x;
            double yVel = vel.y;
            speed = sqrt(pow(xVel,2) + pow(yVel,2));

            Vector pos = model->GetPosition();
            xPos = pos.x;
            yPos = pos.y;
            }
#endif
        /////////////////////////////////////////////////////////
        // Broadcast state only if the state is valid
        /////////////////////////////////////////////////////////
        if(speed > -1.0)
            {
            packetBuf.nodeId = id;
            packetBuf.xPos = xPos;
            packetBuf.yPos = yPos;
            packetBuf.speed = speed;
            packetBuf.posOnLane = posOnLane;
            SendPacket((uint8_t*)&packetBuf, sizeof(packetBuf));
            }

        /////////////////////////////////////////////////////////
        // Schedule next cycle
        /////////////////////////////////////////////////////////
        m_startStopEvent = Simulator::Schedule(Seconds(1.0),
                &VanetMonitorApplication::StartMonitorLoop, this);
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

    void VanetMonitorApplication::SendPacket (uint8_t* buf, int size)
        {
        NS_LOG_FUNCTION_NOARGS ();
        if(buf == 0)
            {
            return;
            }

        NS_LOG_DEBUG ("Sender Node:" << GetNode()->GetId()
                << " Packet("
                << " id=" << ((PktBuf_t*)buf)->nodeId
                << " x=" << ((PktBuf_t*)buf)->xPos
                << " y=" << ((PktBuf_t*)buf)->yPos
                << " spd=" << ((PktBuf_t*)buf)->speed
                << " posOnLane=" << ((PktBuf_t*)buf)->posOnLane
                << " )");
        Ptr<Packet> packet = Create<Packet> (buf, size);

        /////////////////////////////////////////////////////////
        // Send packet
        /////////////////////////////////////////////////////////
        m_socket->Send (packet);

        // Check
        if (InetSocketAddress::IsMatchingType(m_peer))
            {
            NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                    << "s VanetMonitorApplication application sent "
                    << packet->GetSize () << " bytes to "
                    << InetSocketAddress::ConvertFrom(m_peer).GetIpv4 ()
                    << " port "
                    << InetSocketAddress::ConvertFrom (m_peer).GetPort ());
            }
        else if (Inet6SocketAddress::IsMatchingType(m_peer))
            {
            NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                    << "s VanetMonitorApplication application sent "
                    << packet->GetSize () << " bytes to "
                    << Inet6SocketAddress::ConvertFrom(m_peer).GetIpv6 ()
                    << " port "
                    << Inet6SocketAddress::ConvertFrom (m_peer).GetPort ());
            }

        }

    void VanetMonitorApplication::RecvPacket(Ptr<Socket> socket)
        {
        NS_LOG_FUNCTION_NOARGS ();

        Ptr<Packet> packet;
        PktBuf_t packetBuf;
        double xPos = 0.0, yPos = 0.0, speed = -1.0, posOnLane = 0.0;
        int nodeId = -1, senderId = -1;

        /////////////////////////////////////////////////////////
        // Receive packet
        /////////////////////////////////////////////////////////
        packet = socket->Recv();
        int len = packet->CopyData((uint8_t*) &packetBuf, sizeof(packetBuf));
        if (len > 0)
            {
            nodeId = GetNode()->GetId();
            senderId = packetBuf.nodeId;
            xPos = packetBuf.xPos;
            yPos = packetBuf.yPos;
            speed = packetBuf.speed;
            posOnLane = packetBuf.posOnLane;

            NS_LOG_DEBUG ("Receiver Node:" << nodeId
                    << " Packet("
                    << " id=" << senderId
                    << " x=" << xPos
                    << " y=" << yPos
                    << " spd=" << speed
                    << " posOnLane=" << posOnLane
                    << " )");
            }

        // Check
        if (InetSocketAddress::IsMatchingType(m_peer))
            {
            NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                    << "s packet sink received "
                    << packet->GetSize () << " bytes from "
                    << InetSocketAddress::ConvertFrom(m_peer).GetIpv4 ()
                    << " port "
                    << InetSocketAddress::ConvertFrom (m_peer).GetPort ());
            }
        else if (Inet6SocketAddress::IsMatchingType(m_peer))
            {
            NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                    << "s packet sink received "
                    << packet->GetSize () << " bytes from "
                    << Inet6SocketAddress::ConvertFrom(m_peer).GetIpv6 ()
                    << " port "
                    << Inet6SocketAddress::ConvertFrom (m_peer).GetPort ());
            }

        /////////////////////////////////////////////////////////
        // Save all the neighboring nodes state (position + speed)
        /////////////////////////////////////////////////////////
        if(speed > -1.0)
            {
            m_sumoCmdSetTrace(nodeId, senderId, xPos, yPos, speed, posOnLane);
            }
        }

    void VanetMonitorApplication::ConnectionSucceeded (Ptr<Socket>)
        {
        NS_LOG_FUNCTION_NOARGS ();
        NS_LOG_DEBUG ("VanetMonitorApplication, Connection Succeeded");
        m_connected = true;
        ScheduleStartEvent ();
        }

    void VanetMonitorApplication::ConnectionFailed (Ptr<Socket>)
        {
        NS_LOG_FUNCTION_NOARGS ();
        NS_LOG_DEBUG ("VanetMonitorApplication, Connection Failed");
        }

    Ptr<ConstantVelocityMobilityModel>
    VanetMonitorApplication::GetMobilityModel()
        {
        Ptr<Object> object = GetNode();

        Ptr<ConstantVelocityMobilityModel> model = object
                ->GetObject<ConstantVelocityMobilityModel>();
        return model;
        }

    }  // Namespace ns3
