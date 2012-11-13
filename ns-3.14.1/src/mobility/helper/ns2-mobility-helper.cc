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
#include "ns3/ns2-mobility-helper.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <vector>
#include "TraciClient.h"

NS_LOG_COMPONENT_DEFINE ("Ns2MobilityHelper");

using namespace std;

namespace ns3 {

struct DestinationPoint
{
  Vector m_startPosition;     // Start position of last movement
  Vector m_finalPosition;     // Final destination to be reached before next schedule. Replaced with actually reached if needed.
  Vector m_speed;             // Speed of the last movement (needed to derive reached destination at next schedule = start + velocity * actuallyTravelled)

  DestinationPoint () :
    m_startPosition (Vector (0,0,0)),
    m_finalPosition (Vector (0,0,0)),
    m_speed (Vector (0,0,0))
  {};
};


struct VehicleInfo {
	int statCnt;
	vector<int> nodeId;
	vector<double> x;
	vector<double> y;
	vector<double> speed;
};

#define MAX_NODE_CNT 1024

//static int sumoStepCnt = 0;
static map<int, DestinationPoint> last_pos;
static int nodeIdCnt[MAX_NODE_CNT];
static void parseStatus(char* buf, VehicleInfo* vInfo);
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

static void notifyPktRx (Ptr<const Packet> pkt, const Address &from)
{

}

Ns2MobilityHelper::Ns2MobilityHelper (std::string filename)
{
   //Nothing
  	Config::Connect ("/NodeList/*/ApplicationList/1/$ns3::PacketSink/Rx",
  	  	  	          MakeCallback (&notifyPktRx));
}

void
Ns2MobilityHelper::Install ()
{
  //Install (NodeList::Begin (), NodeList::End ());
	m_nodelist_begin = NodeList::Begin ();
	m_nodelist_end  = NodeList::End ();
	ConfigNodesMovements();
}

//void Ns2MobilityHelper::SetMobilityModelForAll (const ObjectStore &store) const
void Ns2MobilityHelper::SetMobilityModelForAll ()
{
	int nodeId = 0;
	//Ptr<Object> object = store.Get (nodeId);
	Ptr<Object> object = GetObject (nodeId);

	while(object != 0) {
		NS_LOG_DEBUG("Set mobility for node "<< nodeId);

		Ptr<ConstantVelocityMobilityModel> model =
							CreateObject<ConstantVelocityMobilityModel> ();
		object->AggregateObject (model);
		//object = store.Get(++nodeId);
		object = GetObject (++nodeId);
	}
}

Ptr<Object>
Ns2MobilityHelper::GetObject(int id)
{
	NodeList::Iterator iterator = m_nodelist_begin;
	iterator += id;
	if (iterator >= m_nodelist_end) {
		NS_LOG_ERROR("No Object for ID "<< id);
		return 0;
	}

	return *iterator;
}

Ptr<ConstantVelocityMobilityModel>
//Ns2MobilityHelper::GetMobilityModel (int id, const ObjectStore &store) const
Ns2MobilityHelper::GetMobilityModel (int id)
{
	//Ptr<Object> object = store.Get (id);
	Ptr<Object> object = GetObject(id);
	if (object == 0) {
		NS_LOG_ERROR("No Object for ID "<< id);
		return 0;
	}

	Ptr<ConstantVelocityMobilityModel> model = object->GetObject<ConstantVelocityMobilityModel> ();
	if (model == 0) {
	NS_LOG_DEBUG("Mobility Model not found for ID "<< id);
		model = CreateObject<ConstantVelocityMobilityModel> ();
		object->AggregateObject (model);
	}
	return model;
}

void Ns2MobilityHelper::StartMotionUpdate ()
{
	//int inetAddrSentFlag = 0;
	int n = -1;
	char cmdBuf[256];
	char statBuf[1024];

#if 0
	if(sumoStepCnt++ < 70) {
		bzero(cmdBuf, sizeof(cmdBuf));
		if(sumoStepCnt == 20) {
			strcpy(cmdBuf, "veh0,maxspeed;nil,simstep");
		} else if(sumoStepCnt == 60){
			strcpy(cmdBuf, "veh1,maxspeedb;nil,simstep");
		} else {
			strcpy(cmdBuf, "nil,simstep"); }
		n = m_traci_client->sendData(cmdBuf, strlen(cmdBuf));
		if (n < 0) {
			NS_LOG_ERROR ("ERROR sending to socket");
			exit(1);
		}
#endif

		// Request for vehicle status to Traci
		bzero(cmdBuf, sizeof(cmdBuf));
		strcpy(cmdBuf, "nil,stat");
		n = m_traci_client->sendData(cmdBuf, strlen(cmdBuf));
		if (n < 0) {
			NS_LOG_ERROR ("ERROR sending to socket");
			return;
		}

		// Receive vehicle status from Traci
		bzero(statBuf, sizeof(statBuf));
		n = m_traci_client->recvData(statBuf, sizeof(statBuf));
		if (n < 0) {
			NS_LOG_ERROR ("ERROR receiving from socket");
			return;
		}
		//NS_LOG_DEBUG ("Echo from server : " << statBuf);

		VehicleInfo vInfo;
		parseStatus(statBuf, &vInfo);

		// Move the nodes whose status is obtained from the SUMO traci client
		// server
		for(int i = 0; i < vInfo.statCnt; i++) {

			NS_LOG_DEBUG ("Vehicle status from SUMO : " << "ID=" << vInfo.nodeId[i]
			                                            << " x=" << vInfo.x[i]
			                                            << " y=" << vInfo.y[i]
			                                            << " speed=" << vInfo.speed[i]);

			// Get the node Id
			int nodeId  = vInfo.nodeId[i];
			if (nodeId < 0) {
				NS_LOG_DEBUG ("Node number is invalid");
				continue;
			}
			if (nodeId >= MAX_NODE_CNT) {
				NS_LOG_DEBUG ("Node number is greater than max");
				continue;
			}
			nodeIdCnt[nodeId] = nodeIdCnt[nodeId] + 1;
			Ptr<ConstantVelocityMobilityModel> model = GetMobilityModel (nodeId);
			if (model == 0) {
				NS_LOG_DEBUG ("Model couldn't be obtained for node : " << nodeId);
				continue;
			}

			if(nodeIdCnt[nodeId] == 1) { // Node seen for the first time

				DestinationPoint point;
				point.m_finalPosition = SetInitialPosition (model, vInfo.x[i], vInfo.y[i]);
				last_pos[nodeId] = point;
				NS_LOG_DEBUG ("Initial position for node " << nodeId <<
							  " position = " << last_pos[nodeId].m_finalPosition);

			} else { // Node seen second time onwards

				last_pos[nodeId] = SetMovement (model, last_pos[nodeId].m_finalPosition, vInfo.x[i], vInfo.y[i], vInfo.speed[i]);
				NS_LOG_DEBUG ("Positions after parse for node " << " " << nodeId << " position =" << last_pos[nodeId].m_finalPosition);
			}
	    }

		m_event = Simulator::Schedule (Seconds(1), &Ns2MobilityHelper::StartMotionUpdate, this);
#if 0
	} else {

		// Stop the sumo traci client program
		bzero(cmdBuf, sizeof(cmdBuf));
		strcpy(cmdBuf, "bye");
		m_traci_client->sendData(cmdBuf, strlen(cmdBuf));
		m_traci_client->stop();
	}
#endif
}

void
Ns2MobilityHelper::ConfigNodesMovements ()
{
	// First, attach mobility model for all nodes in ns3.
	 SetMobilityModelForAll();

	// Prepare the Traci Client
	m_traci_client = new TraciClient();
	m_traci_client->start();
    StartMotionUpdate();
}

//DestinationPoint
//SetMovement (Ptr<ConstantVelocityMobilityModel> model, Vector last_pos, double at,
//             double xFinalPosition, double yFinalPosition, double speed)
static DestinationPoint
SetMovement (Ptr<ConstantVelocityMobilityModel> model, Vector last_pos,
             double xFinalPosition, double yFinalPosition, double speed)
{
  DestinationPoint retval;
  retval.m_startPosition = last_pos;
  retval.m_finalPosition = last_pos;
  //retval.m_travelStartTime = at;
  //retval.m_targetArrivalTime = at;

  if (speed == 0)
    {
      // We have to maintain last position, and stop the movement
      //retval.m_stopEvent = Simulator::Schedule (Seconds (at), &ConstantVelocityMobilityModel::SetVelocity, model,
      //                                          Vector (0, 0, 0));
	  model->SetVelocity( Vector (0, 0, 0));
      return retval;
    }
  if (speed > 0)
    {
      // first calculate the time; time = distance / speed
      double time = sqrt (pow (xFinalPosition - retval.m_finalPosition.x, 2) + pow (yFinalPosition - retval.m_finalPosition.y, 2)) / speed;
      //NS_LOG_DEBUG ("at=" << at << " time=" << time);
      NS_LOG_DEBUG (" time=" << time);
      if (time == 0)
        {
          return retval;
        }
      // now calculate the xSpeed = distance / time
      double xSpeed = (xFinalPosition - retval.m_finalPosition.x) / time;
      double ySpeed = (yFinalPosition - retval.m_finalPosition.y) / time; // & same with ySpeed
      retval.m_speed = Vector (xSpeed, ySpeed, 0);

      // quick and dirty set zSpeed = 0
      double zSpeed = 0;

      NS_LOG_DEBUG ("Calculated Speed: X=" << xSpeed << " Y=" << ySpeed << " Z=" << zSpeed);

      // Set the Values
      //Simulator::Schedule (Seconds (at), &ConstantVelocityMobilityModel::SetVelocity, model, Vector (xSpeed, ySpeed, zSpeed));
      //retval.m_stopEvent = Simulator::Schedule (Seconds (at + time), &ConstantVelocityMobilityModel::SetVelocity, model, Vector (0, 0, 0));
      model->SetVelocity(Vector (xSpeed, ySpeed, zSpeed));
      retval.m_finalPosition.x += xSpeed * time;
      retval.m_finalPosition.y += ySpeed * time;
      //retval.m_targetArrivalTime += time;
    }
  return retval;
}


static Vector
SetInitialPosition (Ptr<ConstantVelocityMobilityModel> model, double xcoord, double ycoord)
{
	Vector position;
	position.x = xcoord;
	position.y = ycoord;
	position.z = 0.0;
	model->SetPosition (position);

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
static void parseStatus(char* buf, VehicleInfo* vInfo)
{
	vector<string> allStatus;

	// Get vehicles status
	int statCnt = 0;
	char* pStatus = strtok (buf, ";");
	while (pStatus != NULL) {
		allStatus.push_back(string(pStatus));
		pStatus = strtok (NULL, ";");
		statCnt++;
	}
	vInfo->statCnt = statCnt;

	// Get vehicle parameters
	vector<string>::iterator it;
	for(it = allStatus.begin(); it < allStatus.end(); it++) {

		int paramCnt = 0;
		char* pParams = strtok((char*)(*it).c_str(), ",");
		while (pParams != NULL) {
			//cout << pParams << " ";
			if(paramCnt == 0) {
				vInfo->nodeId.push_back(getNodeId(pParams));
			} else if(paramCnt == 1) {
				vInfo->x.push_back(atof(pParams));
			} else if(paramCnt == 2) {
				vInfo->y.push_back(atof(pParams));
			} else if(paramCnt == 3) {
				vInfo->speed.push_back(atof(pParams));
			}
			paramCnt++;
			pParams = strtok (NULL, ",");
		}
		//cout << endl;
	}

}

// This is how conversion from vehicle Id to node Id is done.
// Vehicle id is veh0 or veh1 etc. Node Id needs just the integer
// number associated with each veh0 or veh1 etc. Extract the id
// string then convert to integer.
static int getNodeId(char* str)
{
	if(str == NULL)
		return -1;

	char strId[10];
	bzero(strId, sizeof(strId));

	// Skip the "veh" part
	strcpy(strId, &str[3]);
	return atoi(strId);
}

} // namespace ns3
