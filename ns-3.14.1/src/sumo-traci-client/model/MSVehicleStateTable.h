/*
 * MSVehicleStateTable.h
 *
 *  Created on: Mar 20, 2013
 *      Author: rudhir
 */

#ifndef MSVEHICLESTATETABLE_H_
#define MSVEHICLESTATETABLE_H_

#include <vector>
#include <string>
#include <map>

class MSVehicleStateTable
    {
    public:
    MSVehicleStateTable();
    virtual ~MSVehicleStateTable();

    void initVehicleStateTable();
    void clearVehicleStateTable();
    typedef struct
         {
         std::string Id;
         double speed;
         double pos_x;
         double pos_y;
         }VehicleState;
    void addValueVehicleState(std::string Id, VehicleState& vState);
    std::string getReceiverVehicleIdAt(int index);
    std::vector<VehicleState> getSenderVehicleListAt(int index);
    int getTableListCount();
    int getTableListItemCount();

    void displayVehicleList(std::vector<MSVehicleStateTable::VehicleState> vList);
    void displayVehicleStateTable();
    void testFillVSTable();

    private:
    typedef std::map<std::string, std::vector<VehicleState> > VSTable;
    VSTable m_VSTable;
    };

#endif /* MSVEHICLESTATETABLE_H_ */
