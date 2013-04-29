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

    /*
     * Inputs
     * Id         : Vehicle Id of broadcaster
     * speed      : Current speed of broadcaster
     * pos_x      : Current x Cartesian Coordinate of broadcaster. This is
     *              needed by NS3 to drive its mobility model.
     * pos_y      : Current y Cartesian Coordinate of broadcaster. This is
     *              needed by NS3 to drive its mobility model.
     * pos_on_lane: Position on lane of broadcaster. This is required
     *              by SUMO because SUMO works with position on
     *              lane variable. It can be calculated from pos_x and
     *              pos_y but the conversion could make the values
     *              different from the ones sent to NS3.
     */
    typedef struct
         {
         std::string Id;
         double speed;
         double pos_x;
         double pos_y;
         double pos_on_lane;
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
