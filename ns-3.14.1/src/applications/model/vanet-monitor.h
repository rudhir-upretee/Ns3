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
#ifndef VANET_MONITOR_APPLICATION_H
#define VANET_MONITOR_APPLICATION_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/data-rate.h"
#include "ns3/random-variable.h"
#include "ns3/traced-callback.h"

namespace ns3
    {
    class Address;
    class RandomVariable;
    class Socket;

    /**
     * VANET Monitor Application
     */
    class VanetMonitorApplication: public Application
        {
        public:
        static TypeId GetTypeId(void);

        VanetMonitorApplication();

        virtual ~VanetMonitorApplication();
        TracedCallback<int, double*, double*, double*> m_sumoCmdTrace;

        protected:
        virtual void DoDispose(void);
        private:
        // inherited from Application base class.
        virtual void StartApplication(void);  // Called at Start time
        virtual void StopApplication(void);  // Called at Stop time

        //helpers
        void CancelEvents();

        // Event handlers
        void StartReadingStatus();

        Ptr<Socket> m_socket;  // Associated socket
        Address m_peer;  // Peer address
        bool m_connected;  // True if connected
        EventId m_startStopEvent;  // Event id for next start or stop event
        TypeId m_tid;
        //TracedCallback<int, double*, double*, double*> m_sumoCmdTrace;

        void ScheduleStartEvent();
        void Ignore(Ptr<Socket>);
        };

    }  // namespace ns3

#endif /* VANET_MONITOR_APPLICATION_H */
