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
 *
 *
 */

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>
#include "ns3/log.h"
#include "ns3/unused.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "sumo-mobility-helper.h"


NS_LOG_COMPONENT_DEFINE("SumoMobilityHelper");

using namespace std;

namespace ns3
    {
    // Declare static member variables
    static std::map<int, Vector> currPos;
    static std::map<int, double> currSpeed;
    static MSVehicleStateTable vehStateTbl;

    static void GetVehicleStatus(string path, int nId, double* x, double* y,
            double* spd);
    static void SetVehicleStatus(string path, int nodeId, int senderId,
            double x, double y, double spd);

    SumoMobilityHelper::SumoMobilityHelper(int traciPort,
                                            std::string traciHost,
                                            //MSVehicleStateTable* ptrVSTable,
                                            int simStartTime,
                                            int simStopTime,
                                            ApplicationContainer* appCont,
                                            VanetMonitorHelper* app)
        {
        //Initialize
        lastNodeIdSeen = 0;
#if 0
        m_ptrVehStateTbl = ptrVSTable;
#endif
        simulatorStartTime = simStartTime;
        simulatorStopTime = simStopTime;
        m_nodelist_begin = NodeList::Begin();
        m_nodelist_end = NodeList::End();
        m_app_container = appCont;
        m_app = app;

        // First, attach mobility model for all nodes in ns3.
        SetMobilityModelForAll();

        // Prepare the Traci Client
        m_traci_client = new NetsimTraciClient(//m_ptrVehStateTbl,
                                               simulatorStartTime,
                                               simulatorStopTime,
                                               "traciClientDebug.out");
        m_traci_client->start(traciPort, traciHost);

        // Start Mobility helper
        m_event = Simulator::Schedule(Seconds(0.01),
                                      &SumoMobilityHelper::StartMotionUpdate,
                                      this);
        }

    SumoMobilityHelper::~SumoMobilityHelper()
        {
        m_traci_client->close();
        }

    void SumoMobilityHelper::HookAppCallbacksForAll()
        {
        Config::Connect("/NodeList/*/ApplicationList/0/$ns3::VanetMonitorApplication/SumoCmdGet",
                        MakeCallback(&GetVehicleStatus));
        Config::Connect("/NodeList/*/ApplicationList/0/$ns3::VanetMonitorApplication/SumoCmdSet",
                        MakeCallback(&SetVehicleStatus));
        }

    void SumoMobilityHelper::HookAppCallbacksFor(int nodeId)
          {
          std::ostringstream outGet, outSet;

          outGet <<"/NodeList/"<< nodeId <<
                  "/ApplicationList/0/$ns3::VanetMonitorApplication/SumoCmdGet";
          Config::Connect(outGet.str(), MakeCallback(&GetVehicleStatus));

          outSet <<"/NodeList/"<< nodeId <<
                  "/ApplicationList/0/$ns3::VanetMonitorApplication/SumoCmdSet";
          Config::Connect(outSet.str(), MakeCallback(&SetVehicleStatus));
          }

    void SumoMobilityHelper::StartMotionUpdate()
        {
        // Send the last vehicle state table update to SUMO.
        // This should be done before advancing SUMO simulation. Because
        // SUMO should have last state update before calculating next state.
        //m_ptrVehStateTbl->testFillVSTable();
        //m_traci_client->sendVSTable();
        //m_ptrVehStateTbl->clearVehicleStateTable();
        vehStateTbl.testFillVSTable();
        m_traci_client->sendVSTable(&vehStateTbl);
        vehStateTbl.clearVehicleStateTable();

        // Move simulation in SUMO.
        m_traci_client->advanceSumoStep();

        // Move nodes in Ns3
        for (int i = 0; i < m_traci_client->getVehicleStateListCnt(); i++)
            {
            bool firstSeen = false;
            MSVehicleStateTable::VehicleState vState =
                    m_traci_client->getVehicleStateListAt(i);

            if (vState.Id == "null")
                {
                NS_LOG_DEBUG ("Vehicle Id out of bounds");
                break;
                }

            if(isVehicleSeenFirstTime(vState.Id))
                {
                m_vehicleNodeMap.insert(std::make_pair(vState.Id, lastNodeIdSeen++));

                firstSeen = true;
                }

            int nodeId = m_vehicleNodeMap[vState.Id];
            Ptr<ConstantVelocityMobilityModel> model = 0;

            if (firstSeen == true)
                {
                // Attach application and mobility model
                attachNodeAppAndMobilityFor(nodeId);

                // Get mobility model
                model = GetMobilityModel(nodeId);
                if (model == 0)
                    {
                    break;
                    }

                // Initialize position
                SumoMobilityHelper::DestinationPoint point;
                point.m_startPosition = SetInitialPosition(model,
                                                           vState.pos_x,
                                                           vState.pos_y);
                point.m_finalPosition = point.m_startPosition;
                point.m_speed = Vector(0,0,0);
                point.m_travelStartTime = Simulator::Now().GetSeconds();
                point.m_targetArrivalTime = Simulator::Now().GetSeconds();

                // Update the m_lastMotionUpdate and position reached vector
                m_lastMotionUpdate[nodeId] = point;
                currPos[nodeId] = point.m_startPosition;
                currSpeed[nodeId] = vState.speed;

                NS_LOG_DEBUG (" Node:" << nodeId <<
                                " vehicle:" << vState.Id <<
                                " Init position = " << m_lastMotionUpdate[nodeId].m_startPosition);

                }
            else
                {
                // Node seen second time onwards
                double now = Simulator::Now().GetSeconds();
                if (m_lastMotionUpdate[nodeId].m_targetArrivalTime > now)
                    {
                    double actuallytraveled = now
                            - m_lastMotionUpdate[nodeId].m_travelStartTime;

                    Vector reached = Vector(m_lastMotionUpdate[nodeId].m_startPosition.x
                                            + m_lastMotionUpdate[nodeId].m_speed.x
                                            * actuallytraveled,
                                            m_lastMotionUpdate[nodeId].m_startPosition.y
                                            + m_lastMotionUpdate[nodeId].m_speed.y
                                            * actuallytraveled,
                                            0
                                            );

                    m_lastMotionUpdate[nodeId].m_stopEvent.Cancel();
                    m_lastMotionUpdate[nodeId].m_finalPosition = reached;

                    // Update the position reached vector
                    currPos[nodeId] = reached;

                    NS_LOG_DEBUG (" Node:" << nodeId <<
                                    " vehicle:" << vState.Id <<
                                    " reached = " << reached <<
                                    " old dest = " << m_lastMotionUpdate[nodeId].m_finalPosition <<
                                    " new dest = " << vState.pos_x << ":" <<vState.pos_y);

                    }
                else
                    {
                    // Update the position reached vector
                    currPos[nodeId].x = m_lastMotionUpdate[nodeId].m_finalPosition.x;
                    currPos[nodeId].y = m_lastMotionUpdate[nodeId].m_finalPosition.y;
                    currPos[nodeId].z = m_lastMotionUpdate[nodeId].m_finalPosition.z;

                    NS_LOG_DEBUG (" Node:" << nodeId <<
                                    " vehicle:" << vState.Id <<
                                    " reached = " << m_lastMotionUpdate[nodeId].m_finalPosition <<
                                    " new dest = " << vState.pos_x << ":" <<vState.pos_y);
                    }

                // Update current speed
                currSpeed[nodeId] = vState.speed;

                // Get mobility model
                model = GetMobilityModel(nodeId);
                if (model == 0)
                    {
                    break;
                    }

                // Move the node
                m_lastMotionUpdate[nodeId] = SetMovement(model,
                                                         m_lastMotionUpdate[nodeId].m_finalPosition,
                                                         vState.pos_x,
                                                         vState.pos_y,
                                                         vState.speed);
                }
            }

        // Schedule for next round
        m_event = Simulator::Schedule(Seconds(1.0),
                                      &SumoMobilityHelper::StartMotionUpdate,
                                      this);
        }

    void SumoMobilityHelper::SetMobilityModelForAll()
        {
        int nodeId = 0;
        Ptr<Object> object = GetObject(nodeId);

        while (object != 0)
            {
            NS_LOG_DEBUG("Set mobility for node "<< nodeId);

            Ptr<ConstantVelocityMobilityModel> model =
                    CreateObject<ConstantVelocityMobilityModel>();
            object->AggregateObject(model);

            object = GetObject(++nodeId);
            }
        }

    void SumoMobilityHelper::SetMobilityModelFor(int nodeId)
        {
        Ptr<Object> object = GetObject(nodeId);

        if (object != 0)
            {
            NS_LOG_DEBUG("Set mobility for node "<< nodeId);

            Ptr<ConstantVelocityMobilityModel> model =
                    CreateObject<ConstantVelocityMobilityModel>();
            object->AggregateObject(model);
            }
        else
            {
            NS_LOG_DEBUG("This node does not exist "<< nodeId);
            }
        }

    Ptr<Object> SumoMobilityHelper::GetObject(int id)
        {
        NodeList::Iterator iterator = m_nodelist_begin;
        iterator += id;
        if (iterator >= m_nodelist_end)
            {
            return 0;
            }

        return *iterator;
        }

    Ptr<Node> SumoMobilityHelper::GetNode(int id)
        {
        NodeList::Iterator iterator = m_nodelist_begin;
        iterator += id;
        if (iterator >= m_nodelist_end)
            {
            return 0;
            }

        return *iterator;
        }

    Ptr<ConstantVelocityMobilityModel> SumoMobilityHelper::GetMobilityModel(
            int id)
        {
        Ptr<Object> object = GetObject(id);
        if (object == 0)
            {
            NS_LOG_ERROR("No Object for ID "<< id);
            return 0;
            }

        Ptr<ConstantVelocityMobilityModel> model = object
                ->GetObject<ConstantVelocityMobilityModel>();
        if (model == 0)
            {
            NS_LOG_DEBUG("Mobility Model not found for ID "<< id);

            // Has been taken care of.
            //model = CreateObject<ConstantVelocityMobilityModel>();
            //object->AggregateObject(model);
            }
        return model;
        }

    SumoMobilityHelper::DestinationPoint SumoMobilityHelper::SetMovement(
            Ptr<ConstantVelocityMobilityModel> model, Vector last_pos,
            double xFinalPosition, double yFinalPosition, double speed)
        {
        SumoMobilityHelper::DestinationPoint retval;
        retval.m_startPosition = last_pos;
        retval.m_finalPosition = last_pos;
        retval.m_travelStartTime = Simulator::Now().GetSeconds();
        retval.m_targetArrivalTime = Simulator::Now().GetSeconds();

        if (speed == 0)
            {
            //We have to maintain last position, and stop the movement
            model->SetVelocity(Vector(0, 0, 0));
            return retval;
            }
        if (speed > 0)
            {
            // first calculate the time; time = distance / speed
            double time = sqrt(pow(xFinalPosition - retval.m_finalPosition.x, 2)
                    + pow(yFinalPosition - retval.m_finalPosition.y, 2))
                    / speed;

            if (time == 0)
                {
                return retval;
                }
            // now calculate the speed = distance / time
            double xSpeed = (xFinalPosition - retval.m_finalPosition.x) / time;
            double ySpeed = (yFinalPosition - retval.m_finalPosition.y) / time;
            retval.m_speed = Vector(xSpeed, ySpeed, 0);

            // quick and dirty set zSpeed = 0
            double zSpeed = 0;

            NS_LOG_DEBUG ("Travel time = " << time <<
                            " Calculated Speed: X=" << xSpeed <<
                            " Y=" << ySpeed << " Z=" << zSpeed);

            // Stop motion after reaching the destination point
            retval.m_stopEvent =
                    Simulator::Schedule(Seconds(time),
                                        &ConstantVelocityMobilityModel::SetVelocity,
                                        model,
                                        Vector(0, 0, 0));

            model->SetVelocity(Vector(xSpeed, ySpeed, zSpeed));
            retval.m_targetArrivalTime += time;

            // These values will be same as xFinalPosition and yFinalPosition
            retval.m_finalPosition.x += xSpeed * time;
            retval.m_finalPosition.y += ySpeed * time;
            }
        return retval;
        }

    Vector SumoMobilityHelper::SetInitialPosition(
            Ptr<ConstantVelocityMobilityModel> model, double xcoord,
            double ycoord)
        {
        Vector position;
        position.x = xcoord;
        position.y = ycoord;
        position.z = 0.0;
        model->SetPosition(position);

        return position;
        }

    bool SumoMobilityHelper::isVehicleSeenFirstTime(string vehicleId)
        {
        VehicleNodeMap::iterator vIter = m_vehicleNodeMap.find(vehicleId);
        if (vIter == m_vehicleNodeMap.end())
            {
            return true;
            }
        return false;
        }

    void SumoMobilityHelper::attachNodeAppAndMobilityFor(int nodeId)
         {
         Ptr<Node> ptrNode = GetNode(nodeId);

         if(ptrNode != 0)
             {
             UniformVariable randVarTime(0, 1);
             *m_app_container = m_app->Install(ptrNode);
             (*m_app_container).Start(Seconds(randVarTime.GetValue()));
             (*m_app_container).Stop(Seconds(simulatorStopTime));

             // Trace sink should be attached after the source has been initialized.
             // Trace source are in VanetMonitorApplication. So this should be
             // done after the application initialization.
             HookAppCallbacksFor(nodeId);
             }
         else
             {
             NS_LOG_ERROR("This nodeId is invalid "<< nodeId);
             }
        }

    void GetVehicleStatus(string path, int nodeId, double* xPos, double* yPos,
            double* spd)

        {
        if ((xPos == NULL) || (yPos == NULL) || (spd == NULL))
            {
            NS_LOG_ERROR("Null arguments passed");
            *spd = -1.0;
            return;
            }

        if(nodeId >= (int)currPos.size())
            {
            NS_LOG_ERROR("No matching node Id");
            *spd = -1.0;
            return;
            }

        *xPos = currPos[nodeId].x;
        *yPos = currPos[nodeId].y;
        *spd = currSpeed[nodeId];

        }

    void SetVehicleStatus(string path, int nodeId, int senderId,
            double xPos, double yPos, double spd)
        {
        }

    }  // namespace ns3
