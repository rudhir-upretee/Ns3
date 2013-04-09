/*
 * MSVehicleStateTable.cpp
 *
 *  Created on: Mar 20, 2013
 *      Author: rudhir
 */
#include <algorithm>
#include <iostream>
#include "MSVehicleStateTable.h"

MSVehicleStateTable::MSVehicleStateTable()
    {
    }

MSVehicleStateTable::~MSVehicleStateTable()
    {
    }

void MSVehicleStateTable::initVehicleStateTable()
    {
    }

void MSVehicleStateTable::clearVehicleStateTable()
    {
    // Erase all the keys
    m_VSTable.erase(m_VSTable.begin(), m_VSTable.end());
    }

void MSVehicleStateTable::addValueVehicleState(std::string Id, VehicleState& vState)
    {
    VSTable::iterator iter = m_VSTable.find(Id);
    if (iter != m_VSTable.end())
        {
        // Key exists. Add value to the end of the vector.
        iter->second.push_back(vState);
        }
    else
        {
        // Key does not exist. Add new key and new value to the vector.
        m_VSTable[Id].push_back(vState);
        }
    }

std::string MSVehicleStateTable::getReceiverVehicleIdAt(int index)
    {
    if(index > (m_VSTable.size()-1))
        {
        // Empty string
        return "";
        }

    VSTable::iterator iter = m_VSTable.begin();
    for(int i = 0; i < index; i++)
        {
        iter++;
        }
    return iter->first;
    }

std::vector<MSVehicleStateTable::VehicleState> MSVehicleStateTable::getSenderVehicleListAt(int index)
    {
    if(index > (m_VSTable.size()-1))
        {
        // Empty vector
        return std::vector<MSVehicleStateTable::VehicleState>();
        }

    VSTable::iterator iter = m_VSTable.begin();
    for(int i = 0; i < index; i++)
        {
        iter++;
        }
    return iter->second;
    }

int MSVehicleStateTable::getTableListCount()
    {
    int rowCnt = 0;
    for(VSTable::iterator iter = m_VSTable.begin();iter != m_VSTable.end();iter++)
        {
        rowCnt++;
        }
    return rowCnt;
    }

int MSVehicleStateTable::getTableListItemCount()
    {
    int itemCnt = 0;
    for(VSTable::iterator iter = m_VSTable.begin();iter != m_VSTable.end();iter++)
        {
        for(unsigned int i = 0; i < iter->second.size(); i++)
            {
            itemCnt++;
            }
        }
    return itemCnt;
    }

void MSVehicleStateTable::displayVehicleStateTable()
    {
    bool empty = true;
    int listCnt = getTableListCount();
    int listItemCnt = getTableListItemCount();

    std::cout << "List cnt: " << listCnt << " List Item cnt: " << listItemCnt << std::endl;
    for(int i = 0; i < listCnt; i++)
        {
        empty = false;
        std::cout << "Receiver ID: " << getReceiverVehicleIdAt(i)
                    << ", Sender List: ";
        displayVehicleList(getSenderVehicleListAt(i));
        std::cout << std::endl;
        }
    if(empty)
        {
        std::cout << "Table is empty" << std::endl;
        }
    }

void MSVehicleStateTable::displayVehicleList(std::vector<MSVehicleStateTable::VehicleState> vList)
    {
    for(unsigned int i = 0; i < vList.size(); i++)
        {
        std::cout << "(" << vList.at(i).Id << ": " << vList.at(i).speed
                                            << ", " << vList.at(i).pos_x
                                            << ", " << vList.at(i).pos_y
                                            << ") ";
        }
    }

void MSVehicleStateTable::testFillVSTable()
    {
    VehicleState vs;

    vs.Id = "veh0"; vs.speed = 1.0; vs.pos_x = 1.0; vs.pos_y = 1.0;
    addValueVehicleState("veh1", vs);
    vs.Id = "veh2"; vs.speed = 1.2; vs.pos_x = 1.2; vs.pos_y= 1.2;
    addValueVehicleState("veh1", vs);

    vs.Id = "veh1"; vs.speed = 0.1; vs.pos_x = 0.1; vs.pos_y = 0.1;
    addValueVehicleState("veh0", vs);
    vs.Id = "veh2"; vs.speed = 0.2; vs.pos_x = 0.2; vs.pos_y = 0.2;
    addValueVehicleState("veh0", vs);

    vs.Id = "veh1"; vs.speed = 3.1; vs.pos_x = 3.1; vs.pos_y = 3.1;
    addValueVehicleState("veh3", vs);
    vs.Id = "veh2"; vs.speed = 3.2; vs.pos_x = 3.2; vs.pos_y = 3.2;
    addValueVehicleState("veh3", vs);
    vs.Id = "veh4"; vs.speed = 3.4; vs.pos_x = 3.4; vs.pos_y = 3.4;
    addValueVehicleState("veh3", vs);

    vs.Id = "veh3"; vs.speed = 2.3; vs.pos_x = 2.3; vs.pos_y = 2.3;
    addValueVehicleState("veh2", vs);

    vs.Id = "veh1"; vs.speed = 4.1; vs.pos_x = 4.1; vs.pos_y = 4.1;
    addValueVehicleState("veh4", vs);
    vs.Id = "veh2"; vs.speed = 4.2; vs.pos_x = 4.2; vs.pos_y = 4.2;
    addValueVehicleState("veh4", vs);
    vs.Id = "veh3"; vs.speed = 4.3; vs.pos_x = 4.3; vs.pos_y = 4.3;
    addValueVehicleState("veh4", vs);
    }
