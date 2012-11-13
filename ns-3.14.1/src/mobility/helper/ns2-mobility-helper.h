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
#ifndef NS2_MOBILITY_HELPER_H
#define NS2_MOBILITY_HELPER_H

#include <string>
#include <stdint.h>
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "TraciClient.h"

namespace ns3 {

class ConstantVelocityMobilityModel;

class Ns2MobilityHelper
{
public:
  /**
   * \param
   */
  Ns2MobilityHelper (std::string filename);

  /**
   * Read the ns2 trace file and configure the movement
   * patterns of all nodes contained in the global ns3::NodeList
   * whose nodeId is matches the nodeId of the nodes in the trace
   * file.
   */
  void Install ();

private:
  EventId m_event;   // Event. May be canceled if needed.
  NodeList::Iterator m_nodelist_begin;
  NodeList::Iterator m_nodelist_end;
  TraciClient* m_traci_client;

  void ConfigNodesMovements ();
  Ptr<ConstantVelocityMobilityModel> GetMobilityModel (int id);
  void SetMobilityModelForAll ();
  Ptr<Object> GetObject(int id);
  void StartMotionUpdate ();
};

} // namespace ns3

#endif /* NS2_MOBILITY_HELPER_H */
