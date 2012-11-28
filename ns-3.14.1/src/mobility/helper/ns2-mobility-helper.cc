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

#if 0
    struct DestinationPoint
	{
	Vector m_startPosition; // Start position of last movement
	Vector m_finalPosition;// Final destination to be reached before next schedule. Replaced with actually reached if needed.
	Vector m_speed;// Speed of the last movement (needed to derive reached destination at next schedule = start + velocity * actuallyTravelled)

	DestinationPoint () :
	m_startPosition (Vector (0,0,0)),
	m_finalPosition (Vector (0,0,0)),
	m_speed (Vector (0,0,0))
	    {};
	};

#define MAX_NODE_CNT 1024

    struct VehicleInfo
	{
	int validStatCnt;
	int nodeId[MAX_NODE_CNT];
	double x[MAX_NODE_CNT];
	double y[MAX_NODE_CNT];
	double speed[MAX_NODE_CNT];
	};

    static map<int, DestinationPoint> last_pos;
    static VehicleInfo vInfo;
    static int nodeIdCnt[MAX_NODE_CNT];
//static void parseStatus(char* buf, VehicleInfo* vInfo);
    static void parseStatus(char* buf);
    static int getNodeId(char* str);

// Set way points and speed for movement.
    static DestinationPoint SetMovement (Ptr<ConstantVelocityMobilityModel> model,
	    Vector lastPos,
	    double xFinalPosition,
	    double yFinalPosition,
	    double speed);

// Set initial position for a node
    static Vector SetInitialPosition (Ptr<ConstantVelocityMobilityModel> model,
	    double xcoord, double ycoord);
#endif

    // Declare static member variables
    static int prevStatCnt = 0;
    static int nodeId[MAX_NODE_CNT];
    static double xcoord[MAX_NODE_CNT];
    static double ycoord[MAX_NODE_CNT];
    static double speed[MAX_NODE_CNT];
    static void GetVehicleStatus(string path, int nId, double* x, double* y,
	    double* spd);

    Ns2MobilityHelper::Ns2MobilityHelper(std::string filename)
	{
	// Do nothing
	}

    void Ns2MobilityHelper::HookAppCallbacks()
	{
	Config::Connect(
		"/NodeList/*/ApplicationList/0/$ns3::VanetMonitorApplication/SumoCmd",
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
	m_event = Simulator::Schedule(Seconds(1),
		&Ns2MobilityHelper::StartMotionUpdate, this);
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

	if (parseStatus(statBuf) == -1)
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

#if 0
	    if (nodeId >= MAX_NODE_CNT)
		{
		NS_LOG_DEBUG ("Node number is greater than max");
		continue;
		}
	    nodeIdCnt[nodeId] = nodeIdCnt[nodeId] + 1;
#endif

	    Ptr<ConstantVelocityMobilityModel> model = GetMobilityModel(nId);
	    if (model == 0)
		{
		NS_LOG_DEBUG ("Model couldn't be obtained for node : " << nId);
		continue;
		}

	    if (std::find(nodeIdEnteredSim.begin(), nodeIdEnteredSim.end(), nId)
		    != nodeIdEnteredSim.end())
		{
		//
		// Node seen for the first time

		Ns2MobilityHelper::DestinationPoint point;
		point.m_finalPosition = SetInitialPosition(model, xcoord[i],
			ycoord[i]);
		last_pos[nId] = point;
		NS_LOG_DEBUG ("Initial position for node " << nId <<
			" position = " << last_pos[nId].m_finalPosition);

		// Save the Node Ids that are seen so far
		nodeIdEnteredSim.push_back(nId);
		}
	    else
		{
		//
		// Node seen second time onwards

		last_pos[nId] = SetMovement(model,
			last_pos[nId].m_finalPosition, xcoord[i], ycoord[i],
			speed[i]);
		NS_LOG_DEBUG ("Positions after parse for node " << " " << nId
			<< " position =" << last_pos[nId].m_finalPosition);
		}
	    }

	m_event = Simulator::Schedule(Seconds(1),
		&Ns2MobilityHelper::StartMotionUpdate, this);
	}

    void Ns2MobilityHelper::SetMobilityModelForAll()
	{
	int nodeId = 0;
        //Ptr<Object> object = store.Get (nodeId);
	Ptr<Object> object = GetObject(nodeId);

	while (object != 0)
	    {
	    NS_LOG_DEBUG("Set mobility for node "<< nodeId);

	    Ptr<ConstantVelocityMobilityModel> model = CreateObject<
		    ConstantVelocityMobilityModel>();
	    object->AggregateObject(model);
	    //object = store.Get(++nodeId);
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

    Ptr<ConstantVelocityMobilityModel>
//Ns2MobilityHelper::GetMobilityModel (int id, const ObjectStore &store) const
    Ns2MobilityHelper::GetMobilityModel(int id)
	{
	//Ptr<Object> object = store.Get (id);
	Ptr<Object> object = GetObject(id);
	if (object == 0)
	    {
	    NS_LOG_ERROR("No Object for ID "<< id);
	    return 0;
	    }

	Ptr<ConstantVelocityMobilityModel> model = object->GetObject<
		ConstantVelocityMobilityModel>();
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
	//retval.m_travelStartTime = at;
	//retval.m_targetArrivalTime = at;

	if (speed == 0)
	    {
	    // We have to maintain last position, and stop the movement
	    //retval.m_stopEvent = Simulator::Schedule (Seconds (at), &ConstantVelocityMobilityModel::SetVelocity, model,
	    //                                          Vector (0, 0, 0));
	    model->SetVelocity(Vector(0, 0, 0));
	    return retval;
	    }
	if (speed > 0)
	    {
	    // first calculate the time; time = distance / speed
	    double time = sqrt(
		    pow(xFinalPosition - retval.m_finalPosition.x, 2)
			    + pow(yFinalPosition - retval.m_finalPosition.y, 2))
		    / speed;
	    //NS_LOG_DEBUG ("at=" << at << " time=" << time);
	    NS_LOG_DEBUG (" time=" << time);
	    if (time == 0)
		{
		return retval;
		}
	    // now calculate the xSpeed = distance / time
	    double xSpeed = (xFinalPosition - retval.m_finalPosition.x) / time;
	    double ySpeed = (yFinalPosition - retval.m_finalPosition.y) / time; // & same with ySpeed
	    retval.m_speed = Vector(xSpeed, ySpeed, 0);

	    // quick and dirty set zSpeed = 0
	    double zSpeed = 0;

	    NS_LOG_DEBUG ("Calculated Speed: X=" << xSpeed << " Y=" << ySpeed << " Z=" << zSpeed);

	    // Set the Values
	    //Simulator::Schedule (Seconds (at), &ConstantVelocityMobilityModel::SetVelocity, model, Vector (xSpeed, ySpeed, zSpeed));
	    //retval.m_stopEvent = Simulator::Schedule (Seconds (at + time), &ConstantVelocityMobilityModel::SetVelocity, model, Vector (0, 0, 0));
	    model->SetVelocity(Vector(xSpeed, ySpeed, zSpeed));
	    retval.m_finalPosition.x += xSpeed * time;
	    retval.m_finalPosition.y += ySpeed * time;
	    //retval.m_targetArrivalTime += time;
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
    //static void parseStatus(char* buf, VehicleInfo* vInfo)
    int Ns2MobilityHelper::parseStatus(char* buf)
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
		    nodeId[cnt] = getNodeId(pParams);
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
	    //cout << endl;
	    }

	// Put the count once the array has been updated
	prevStatCnt = statCnt;
	return 0;
	}

    // This is how conversion from vehicle Id to node Id is done.
    // Vehicle id is veh0 or veh1 etc. Node Id needs just the integer
    // number associated with each veh0 or veh1 etc. Extract the id
    // string then convert to integer.
    int Ns2MobilityHelper::getNodeId(char* str)
	{
	if (str == NULL)
	    return -1;

	char strId[10];
	bzero(strId, sizeof(strId));

	// Skip the "veh" part
	strcpy(strId, &str[3]);
	return atoi(strId);
	}

    bool Ns2MobilityHelper::isNodeSeenFirstTime(int id)
	{
	for (vector<int>::iterator it = nodeIdEnteredSim.begin();
		it != nodeIdEnteredSim.end(); it++)
	    {
	    if (*it == id)
		{
		return true;
		}
	    }
	return false;
	}

    void GetVehicleStatus(string path, int nId, double* x, double* y,
	    double* spd)

	{
	NS_LOG_DEBUG ("Called");

	if ((x == NULL) || (y == NULL) || (spd == NULL))
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
		NS_LOG_DEBUG ("Matched node Id :" << nId);
		*x = xcoord[i];
		*y = ycoord[i];
		*spd = speed[i];
		return;
		}
	    }

	// Node Id does not match. Notify the caller by setting
	// negative values.
	NS_LOG_DEBUG ("No matching node Id");
	*x = -1.0;
	*y = -1.0;
	*spd = -1.0;
	return;
	}

    } // namespace ns3
