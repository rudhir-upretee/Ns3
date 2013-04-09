/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
 *               2009,2010 Contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Contributors: Thomas Waldecker <twaldecker@rocketmail.com>
 *               Martín Giachino <martin.giachino@gmail.com>
 */
#ifndef SUMO_MOBILITY_HELPER_H
#define SUMO_MOBILITY_HELPER_H

#include <string>
#include <stdint.h>
#include <map>
#include <vector>
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/application-container.h"
//#include "ns3/applications-module.h"
#include "ns3/vanet-monitor-helper.h"
#include "ns3/MSVehicleStateTable.h"
#include "ns3/NetsimTraciClient.h"

namespace ns3
    {
#define MAX_NODE_CNT 1024

    class ConstantVelocityMobilityModel;

    class SumoMobilityHelper
        {
        public:
        /**
         * \param
         */
        SumoMobilityHelper(int traciPort,
                            std::string traciHost,
                            MSVehicleStateTable* ptrVehStateTable,
                            int simulatorStartTime,
                            int simulatorStopTime,
                            ApplicationContainer* appCont,
                            VanetMonitorHelper* app);

        ~SumoMobilityHelper();

        /**
         * Read the ns2 trace file and configure the movement
         * patterns of all nodes contained in the global ns3::NodeList
         * whose nodeId is matches the nodeId of the nodes in the trace
         * file.
         */
        void Install();
        void HookAppCallbacksForAll();
        void HookAppCallbacksFor(int nodeId);

        private:

        struct DestinationPoint
            {
            public:
            // Start position of last movement
            Vector m_startPosition;
            // Final destination to be reached before next schedule.
            // Replaced with actually reached if needed.
            Vector m_finalPosition;
            // Speed of the last movement (needed to derive reached destination
            // at next schedule = start + velocity * actuallyTravelled)
            Vector m_speed;
            EventId m_stopEvent;
            double m_targetArrivalTime;
            double m_travelStartTime;

            DestinationPoint() :
                    m_startPosition(Vector(0, 0, 0)),
                    m_finalPosition(Vector(0, 0, 0)),
                    m_speed(Vector(0, 0, 0)),
                    m_targetArrivalTime(0),
                    m_travelStartTime(0)
                {
                }
            ;
            };
        std::map<int, DestinationPoint> m_lastMotionUpdate;

        typedef std::map<std::string, int > VehicleNodeMap;
        VehicleNodeMap m_vehicleNodeMap;

        EventId m_event;
        NodeList::Iterator m_nodelist_begin;
        NodeList::Iterator m_nodelist_end;
        NetsimTraciClient* m_traci_client;
        ApplicationContainer* m_app_container;
        VanetMonitorHelper* m_app;

        int lastNodeIdSeen;
        int simulatorStartTime;
        int simulatorStopTime;

        void ConfigNodesMovements();
        Ptr<ConstantVelocityMobilityModel> GetMobilityModel(int id);
        void SetMobilityModelForAll();
        void SetMobilityModelFor(int nodeId);
        Ptr<Object> GetObject(int id);
        Ptr<Node> GetNode(int id);
        void StartMotionUpdate();
        DestinationPoint SetMovement(Ptr<ConstantVelocityMobilityModel> model,
                Vector lastPos, double xFinalPosition, double yFinalPosition,
                double speed);
        Vector SetInitialPosition(Ptr<ConstantVelocityMobilityModel> model,
                double xcoord, double ycoord);
        bool isVehicleSeenFirstTime(std::string);
        void attachNodeAppAndMobilityFor(int nodeId);

        };

    }  // namespace ns3

#endif /* SUMO_MOBILITY_HELPER_H */
