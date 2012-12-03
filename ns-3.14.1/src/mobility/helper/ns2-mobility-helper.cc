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

#include <fstream>
#include <sstream>
#include <map>
#include "ns3/log.h"
#include "ns3/unused.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns2-mobility-helper.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <vector>
#include "TraciClient.h"

NS_LOG_COMPONENT_DEFINE("Ns2MobilityHelper");

using namespace std;

namespace ns3
    {
    // Declare static member variables
    static std::map<int, Vector> currPos;
    static std::map<int, double> currSpeed;
    static int prevStatCnt = 0;
    static int nodeId[MAX_NODE_CNT];
    static double xcoord[MAX_NODE_CNT];
    static double ycoord[MAX_NODE_CNT];
    static double speed[MAX_NODE_CNT];
    static void GetVehicleStatus(string path, int nId, double* x, double* y,
            double* spd);

    Ns2MobilityHelper::Ns2MobilityHelper(std::string filename)
        {
        //Initialize node seen array
        for (int i = 0; i < MAX_NODE_CNT; i++)
            {
            nodeIdSeen[i] = -99;
            }
        nodeSeenCnt = 0;
        }

    void Ns2MobilityHelper::HookAppCallbacks()
        {
        Config::Connect("/NodeList/*/ApplicationList/0/$ns3::VanetMonitorApplication/SumoCmd",
                        MakeCallback(&GetVehicleStatus));
        }

    void Ns2MobilityHelper::Install()
        {
        //Install (NodeList::Begin (), NodeList::End ());
        m_nodelist_begin = NodeList::Begin();
        m_nodelist_end = NodeList::End();
        ConfigNodesMovements();
        }

    void Ns2MobilityHelper::ConfigNodesMovements()
        {
        // First, attach mobility model for all nodes in ns3.
        SetMobilityModelForAll();

        // Prepare the Traci Client
        m_traci_client = new TraciClient();
        m_traci_client->start();

        //StartMotionUpdate();
        m_event = Simulator::Schedule(Seconds(1.0),
                                      &Ns2MobilityHelper::StartMotionUpdate,
                                      this);
        }

    void Ns2MobilityHelper::StartMotionUpdate()
        {
        int n = -1;
        char cmdBuf[256];
        char statBuf[1024];

        // Request for vehicle status to Traci
        bzero(cmdBuf, sizeof(cmdBuf));
        strcpy(cmdBuf, "nil,stat");
        n = m_traci_client->sendData(cmdBuf, strlen(cmdBuf));
        if (n < 0)
            {
            NS_LOG_ERROR ("ERROR sending to socket");
            return;
            }

        // Receive vehicle status from Traci
        bzero(statBuf, sizeof(statBuf));
        n = m_traci_client->recvData(statBuf, sizeof(statBuf));
        if (n < 0)
            {
            NS_LOG_ERROR ("ERROR receiving from socket");
            return;
            }
        //NS_LOG_DEBUG ("Echo from server : " << statBuf);

        if (ParseStatus(statBuf) == -1)
            {
            return;
            }

        // Move the nodes whose status is obtained from the SUMO traci client
        // server
        for (int i = 0; i < prevStatCnt; i++)
            {
            NS_LOG_DEBUG ("Vehicle status from SUMO : " << "ID=" << nodeId[i]
                    << " x=" << xcoord[i]
                    << " y=" << ycoord[i]
                    << " speed=" << speed[i]);

            // Get the node Id
            int nId = nodeId[i];
            if (nId < 0)
                {
                NS_LOG_DEBUG ("Node number is invalid");
                continue;
                }

            Ptr<ConstantVelocityMobilityModel> model = GetMobilityModel(nId);
            if (model == 0)
                {
                NS_LOG_DEBUG ("Model couldn't be obtained for node : " << nId);
                continue;
                }

            if (IsNodeSeenFirstTime(nId) == true)
                {
                // Initialization
                Ns2MobilityHelper::DestinationPoint point;
                point.m_startPosition = SetInitialPosition(model,
                                                           xcoord[i],
                                                           ycoord[i]);
                point.m_finalPosition = point.m_startPosition;
                point.m_speed = Vector(0,0,0);
                point.m_travelStartTime = Simulator::Now().GetSeconds();
                point.m_targetArrivalTime = Simulator::Now().GetSeconds();

                // Update the m_lastMotionUpdate and position reached vector
                m_lastMotionUpdate[nId] = point;
                currPos[nId] = point.m_startPosition;
                currSpeed[nId] = 0.0;

                NS_LOG_DEBUG ("Initial position for node " << nId <<
                        " position = " << m_lastMotionUpdate[nId].m_startPosition);

                // Save the Node Ids that are seen so far
                SetNodeSeen(nId);
                }
            else
                {
                //
                // Node seen second time onwards

                NS_LOG_DEBUG ("Last Destination for node " << " " << nId
                        << " = " << m_lastMotionUpdate[nId].m_finalPosition);

                double now = Simulator::Now().GetSeconds();
                if (m_lastMotionUpdate[nId].m_targetArrivalTime > now)
                    {
                    double actuallytraveled = now
                            - m_lastMotionUpdate[nId].m_travelStartTime;
                    Vector reached = Vector(m_lastMotionUpdate[nId].m_startPosition.x
                                                    + m_lastMotionUpdate[nId].m_speed.x
                                                            * actuallytraveled,
                                            m_lastMotionUpdate[nId].m_startPosition.y
                                                    + m_lastMotionUpdate[nId].m_speed.y
                                                            * actuallytraveled,
                                            0

                                            );

                    NS_LOG_DEBUG ("Did not reach a destination! stoptime = " <<
                            m_lastMotionUpdate[nId].m_targetArrivalTime <<
                            ", now = " << now);

                    NS_LOG_DEBUG ("Reached position for node " << " " << nId
                            << " = " << reached);

                    m_lastMotionUpdate[nId].m_stopEvent.Cancel();
                    m_lastMotionUpdate[nId].m_finalPosition = reached;

                    // Update the position reached vector
                    currPos[nId] = reached;
                    }
                else
                    {
                    // Update the position reached vector
                    currPos[nId].x = m_lastMotionUpdate[nId].m_finalPosition.x;
                    currPos[nId].y = m_lastMotionUpdate[nId].m_finalPosition.y;
                    currPos[nId].z = m_lastMotionUpdate[nId].m_finalPosition.z;

                    NS_LOG_DEBUG ("Reach a destination! stoptime = " <<
                            m_lastMotionUpdate[nId].m_targetArrivalTime <<
                            ", now = " << now);

                    NS_LOG_DEBUG ("Reached position for node " << " " << nId
                            << " position =" << currPos[nId]);
                    }

                // Move the node
                m_lastMotionUpdate[nId] = SetMovement(model,
                                            m_lastMotionUpdate[nId].m_finalPosition,
                                            xcoord[i],
                                            ycoord[i],
                                            speed[i]);

                // Update current speed
                currSpeed[nId] = speed[i];
                }
            }

        m_event = Simulator::Schedule(Seconds(1.0),
                                      &Ns2MobilityHelper::StartMotionUpdate,
                                      this);
        }

    void Ns2MobilityHelper::SetMobilityModelForAll()
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

    Ptr<Object> Ns2MobilityHelper::GetObject(int id)
        {
        NodeList::Iterator iterator = m_nodelist_begin;
        iterator += id;
        if (iterator >= m_nodelist_end)
            {
            return 0;
            }

        return *iterator;
        }

    Ptr<ConstantVelocityMobilityModel> Ns2MobilityHelper::GetMobilityModel(
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
            model = CreateObject<ConstantVelocityMobilityModel>();
            object->AggregateObject(model);
            }
        return model;
        }

    Ns2MobilityHelper::DestinationPoint Ns2MobilityHelper::SetMovement(
            Ptr<ConstantVelocityMobilityModel> model, Vector last_pos,
            double xFinalPosition, double yFinalPosition, double speed)
        {
        Ns2MobilityHelper::DestinationPoint retval;
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
            //NS_LOG_DEBUG ("at=" << at << " time=" << time);
            NS_LOG_DEBUG (" time=" << time);
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

            NS_LOG_DEBUG ("Calculated Speed: X=" << xSpeed << " Y=" << ySpeed
                    << " Z=" << zSpeed);

#if 0
            retval.m_stopEvent =
                    Simulator::Schedule(Seconds(time),
                                        &ConstantVelocityMobilityModel::SetVelocity,
                                        model,
                                        Vector(0, 0, 0));
#endif

            model->SetVelocity(Vector(xSpeed, ySpeed, zSpeed));
            retval.m_targetArrivalTime += time;

            // These values will be same as xFinalPosition and yFinalPosition
            retval.m_finalPosition.x += xSpeed * time;
            retval.m_finalPosition.y += ySpeed * time;
            }
        return retval;
        }

    Vector Ns2MobilityHelper::SetInitialPosition(
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

    // Returned vehicle status strings can have multiple vehicle status each
    // separated by semicolon.
    //
    // Like:
    // veh1status;veh2status;veh3status
    //
    // Each vehicle status has multiple parameters separated by commas.
    //
    // Like:
    // vehId,xcord, ycord,speed
    //
    //static void ParseStatus(char* buf, VehicleInfo* vInfo)
    int Ns2MobilityHelper::ParseStatus(char* buf)
        {
        vector<string> allStatus;

        // Invalidate the status array count. For safety.
        prevStatCnt = 0;

        // Get vehicles status
        int statCnt = 0;
        char* pStatus = strtok(buf, ";");
        while (pStatus != NULL)
            {
            allStatus.push_back(string(pStatus));
            pStatus = strtok(NULL, ";");
            statCnt++;
            }
        if (statCnt >= MAX_NODE_CNT)
            {
            NS_LOG_ERROR("Exceed maximum node count in status count");
            return -1;
            }

        // Get vehicle parameters
        int cnt = 0;
        vector<string>::iterator it;
        for (it = allStatus.begin(); it < allStatus.end(); it++)
            {
            int paramCnt = 0;
            char* pParams = strtok((char*) (*it).c_str(), ",");
            while (pParams != NULL)
                {
                //cout << pParams << " ";
                if (paramCnt == 0)
                    {
                    nodeId[cnt] = GetNodeId(pParams);
                    }
                else if (paramCnt == 1)
                    {
                    //vInfo->x.push_back(atof(pParams));
                    xcoord[cnt] = atof(pParams);
                    }
                else if (paramCnt == 2)
                    {
                    //vInfo->y.push_back(atof(pParams));
                    ycoord[cnt] = atof(pParams);
                    }
                else if (paramCnt == 3)
                    {
                    //vInfo->speed.push_back(atof(pParams));
                    speed[cnt] = atof(pParams);
                    }
                paramCnt++;
                pParams = strtok(NULL, ",");
                }
            cnt++;
            }

        // Put the count once the array has been updated
        prevStatCnt = statCnt;
        return 0;
        }

    // This is how conversion from vehicle Id to node Id is done.
    // Vehicle id is veh0 or veh1 etc. Node Id needs just the integer
    // number associated with each veh0 or veh1 etc. Extract the id
    // string then convert to integer.
    int Ns2MobilityHelper::GetNodeId(char* str)
        {
        if (str == NULL)
            return -1;

        char strId[10];
        bzero(strId, sizeof(strId));

        // Skip the "veh" part
        strcpy(strId, &str[3]);
        return atoi(strId);
        }

    void Ns2MobilityHelper::SetNodeSeen(int id)
        {
        nodeIdSeen[nodeSeenCnt] = id;
        nodeSeenCnt++;
        }

    bool Ns2MobilityHelper::IsNodeSeenFirstTime(int id)
        {
#if 0
        for (vector<int>::iterator it = nodeIdEnteredSim.begin();
                it != nodeIdEnteredSim.end(); it++)
            {
            if (*it == id)
                {
                return true;
                }
            }
#endif
        for (int i = 0; i < prevStatCnt; i++)
            {
            if (nodeIdSeen[i] == id)
                {
                return false;
                }
            }
        return true;
        }

    void GetVehicleStatus(string path, int nId, double* xPos, double* yPos,
            double* spd)

        {
        if ((xPos == NULL) || (yPos == NULL) || (spd == NULL))
            {
            NS_LOG_ERROR("Null arguments passed");
            return;
            }

        // Return status only if the nodeId matches the node Id
        // present in previous status update array nodeId[].
        for (int i = 0; i < prevStatCnt; i++)
            {
            if (nodeId[i] == nId)
                {
                NS_LOG_LOGIC ("Matched node Id :" << nId);
                *xPos = currPos[nId].x;
                *yPos = currPos[nId].y;
                *spd = currSpeed[nId];
#if 0
                *xPos = xcoord[i];
                *yPos = ycoord[i];
                *spd = speed[i];
#endif
                return;
                }
            }

        // Node Id does not match. Notify the caller by setting
        // negative values.
        NS_LOG_LOGIC ("No matching node Id");
        *xPos= -1.0;
        *yPos = -1.0;
        *spd = -1.0;
        return;
        }

    }  // namespace ns3
