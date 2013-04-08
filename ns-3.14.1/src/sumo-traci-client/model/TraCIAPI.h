/****************************************************************************/
/// @file    TraCIAPI.h
/// @author  Daniel Krajzewicz
/// @date    30.05.2012
/// @version $Id: TraCIAPI.h 13107 2012-12-02 13:57:34Z behrisch $
///
// C++ TraCI client API implementation
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef TraCIAPI_h
#define TraCIAPI_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include "TraCIConfig.h"
#endif

#include <vector>
#include <string>
#include "TraCISocket.h"
#include "SUMOTime.h"


// ===========================================================================
// global definitions
// ===========================================================================
#define DEFAULT_VIEW "View #0"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class TraCIAPI
 * @brief C++ TraCI client API implementation
 */
class TraCIAPI {
public:
    /// @name Structures definitions
    /// @{

    /** @struct TraCIPosition
     * @brief A 3D-position
     */
    struct TraCIPosition {
        double x, y, z;
    };

    /** @struct TraCIPosition
     * @brief A color
     */
    struct TraCIColor {
        int r, g, b, a;
    };

    /** @struct TraCIPositionVector
     * @brief A list of positions
     */
    typedef std::vector<TraCIPosition> TraCIPositionVector;

    /** @struct TraCIBoundary
     * @brief A 3D-bounding box
     */
    struct TraCIBoundary {
        double xMin, yMin, zMin;
        double xMax, yMax, zMax;
    };



    class TraCIPhase {
    public:
        TraCIPhase(SUMOTime _duration, const std::string& _phase, SUMOTime _duration1, SUMOTime _duration2)
            : duration(_duration), phase(_phase), duration1(_duration1), duration2(_duration2) {}
        ~TraCIPhase() {}

        SUMOTime duration;
        std::string phase;
        SUMOTime duration1, duration2;

    };


    class TraCILogic {
    public:
        TraCILogic(const std::string& _subID, int _type, const std::map<std::string, SUMOReal>& _subParameter, unsigned int _currentPhaseIndex, const std::vector<TraCIPhase>& _phases)
            : subID(_subID), type(_type), subParameter(_subParameter), currentPhaseIndex(_currentPhaseIndex), phases(_phases) {}
        ~TraCILogic() {}

        std::string subID;
        int type;
        std::map<std::string, SUMOReal> subParameter;
        unsigned int currentPhaseIndex;
        std::vector<TraCIPhase> phases;
    };

    class TraCILink {
    public:
        TraCILink(const std::string& _from, const std::string& _via, const std::string& _to)
            : from(_from), via(_via), to(_to) {}
        ~TraCILink() {}

        std::string from;
        std::string via;
        std::string to;
    };

    /// @}



    /** @brief Constructor
     */
    TraCIAPI();


    /// @brief Destructor
    ~TraCIAPI();


    /// @name Connection handling
    /// @{

    /** @brief Connects to the specified SUMO server
     * @param[in] host The name of the host to connect to
     * @param[in] port The port to connect to
     * @exception tcpip::SocketException if the connection fails
     */
    void connect(const std::string& host, int port);


    /// @brief Closes the connection
    void close();
    /// @}



    /// @name Atomar getter
    /// @{

    SUMOTime getSUMOTime(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    int getUnsignedByte(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    int getByte(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    int getInt(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    SUMOReal getFloat(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    SUMOReal getDouble(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    TraCIBoundary getBoundingBox(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    TraCIPositionVector getPolygon(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    TraCIPosition getPosition(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    std::string getString(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    std::vector<std::string> getStringVector(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    TraCIColor getColor(int cmd, int var, const std::string& id, tcpip::Storage* add = 0);
    /// @}



    /** @class TraCIScopeWrapper
     * @brief An abstract interface for accessing type-dependent values
     *
     * Must be derived by interfaces which implement access methods to certain object types
     */
    class TraCIScopeWrapper {
    public:
        /** @brief Constructor
         * @param[in] parent The parent TraCI client which offers the connection
         */
        TraCIScopeWrapper(TraCIAPI& parent) : myParent(parent) {}

        /// @brief Destructor
        virtual ~TraCIScopeWrapper() {}


    protected:
        /// @brief The parent TraCI client which offers the connection
        TraCIAPI& myParent;


    private:
        /// @brief invalidated copy constructor
        TraCIScopeWrapper(const TraCIScopeWrapper& src);

        /// @brief invalidated assignment operator
        TraCIScopeWrapper& operator=(const TraCIScopeWrapper& src);

    };





    /** @class EdgeScope
     * @brief Scope for interaction with edges
     */
    class EdgeScope : public TraCIScopeWrapper {
    public:
        EdgeScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~EdgeScope() {}

        std::vector<std::string> getIDList() const;
        unsigned int getIDCount() const;
        SUMOReal getAdaptedTraveltime(const std::string& edgeID, SUMOTime time) const;
        SUMOReal getEffort(const std::string& edgeID, SUMOTime time) const;
        SUMOReal getCO2Emission(const std::string& edgeID) const;
        SUMOReal getCOEmission(const std::string& edgeID) const;
        SUMOReal getHCEmission(const std::string& edgeID) const;
        SUMOReal getPMxEmission(const std::string& edgeID) const;
        SUMOReal getNOxEmission(const std::string& edgeID) const;
        SUMOReal getFuelConsumption(const std::string& edgeID) const;
        SUMOReal getNoiseEmission(const std::string& edgeID) const;
        SUMOReal getLastStepMeanSpeed(const std::string& edgeID) const;
        SUMOReal getLastStepOccupancy(const std::string& edgeID) const;
        SUMOReal getLastStepLength(const std::string& edgeID) const;
        SUMOReal getTraveltime(const std::string& edgeID) const;
        unsigned int getLastStepVehicleNumber(const std::string& edgeID) const;
        SUMOReal getLastStepHaltingNumber(const std::string& edgeID) const;
        std::vector<std::string> getLastStepVehicleIDs(const std::string& edgeID) const;

        void adaptTraveltime(const std::string& edgeID, SUMOReal time) const;
        void setEffort(const std::string& edgeID, SUMOReal effort) const;
        void setMaxSpeed(const std::string& edgeID, SUMOReal speed) const;

    private:
        /// @brief invalidated copy constructor
        EdgeScope(const EdgeScope& src);

        /// @brief invalidated assignment operator
        EdgeScope& operator=(const EdgeScope& src);

    };





    /** @class GUIScope
     * @brief Scope for interaction with the gui
     */
    class GUIScope : public TraCIScopeWrapper {
    public:
        GUIScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~GUIScope() {}

        std::vector<std::string> getIDList() const;
        SUMOReal getZoom(const std::string& viewID = DEFAULT_VIEW) const;
        TraCIPosition getOffset(const std::string& viewID = DEFAULT_VIEW) const;
        std::string getSchema(const std::string& viewID = DEFAULT_VIEW) const;
        TraCIBoundary getBoundary(const std::string& viewID = DEFAULT_VIEW) const;
        void setZoom(const std::string& viewID, SUMOReal zoom) const;
        void setOffset(const std::string& viewID, SUMOReal x, SUMOReal y) const;
        void setSchema(const std::string& viewID, const std::string& schemeName) const;
        void setBoundary(const std::string& viewID, SUMOReal xmin, SUMOReal ymin, SUMOReal xmax, SUMOReal ymax) const;
        void screenshot(const std::string& viewID, const std::string& filename) const;
        void trackVehicle(const std::string& viewID, const std::string& vehID) const;

    private:
        /// @brief invalidated copy constructor
        GUIScope(const GUIScope& src);

        /// @brief invalidated assignment operator
        GUIScope& operator=(const GUIScope& src);

    };





    /** @class InductionLoopScope
     * @brief Scope for interaction with inductive loops
     */
    class InductionLoopScope : public TraCIScopeWrapper {
    public:
        InductionLoopScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~InductionLoopScope() {}

        std::vector<std::string> getIDList() const;
        SUMOReal  getPosition(const std::string& loopID) const;
        std::string getLaneID(const std::string& loopID) const;
        unsigned int getLastStepVehicleNumber(const std::string& loopID) const;
        SUMOReal getLastStepMeanSpeed(const std::string& loopID) const;
        std::vector<std::string> getLastStepVehicleIDs(const std::string& loopID) const;
        SUMOReal getLastStepOccupancy(const std::string& loopID) const;
        SUMOReal getLastStepMeanLength(const std::string& loopID) const;
        SUMOReal getTimeSinceDetection(const std::string& loopID) const;
        unsigned int getVehicleData(const std::string& loopID) const;

    private:
        /// @brief invalidated copy constructor
        InductionLoopScope(const InductionLoopScope& src);

        /// @brief invalidated assignment operator
        InductionLoopScope& operator=(const InductionLoopScope& src);

    };





    /** @class JunctionScope
     * @brief Scope for interaction with junctions
     */
    class JunctionScope : public TraCIScopeWrapper {
    public:
        JunctionScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~JunctionScope() {}

        std::vector<std::string> getIDList() const;
        TraCIPosition getPosition(const std::string& junctionID) const;

    private:
        /// @brief invalidated copy constructor
        JunctionScope(const JunctionScope& src);

        /// @brief invalidated assignment operator
        JunctionScope& operator=(const JunctionScope& src);

    };





    /** @class LaneScope
     * @brief Scope for interaction with lanes
     */
    class LaneScope : public TraCIScopeWrapper {
    public:
        LaneScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~LaneScope() {}

        std::vector<std::string> getIDList() const;
        SUMOReal getLength(const std::string& laneID) const;
        SUMOReal getMaxSpeed(const std::string& laneID) const;
        SUMOReal getWidth(const std::string& laneID) const;
        std::vector<std::string> getAllowed(const std::string& laneID) const;
        std::vector<std::string> getDisallowed(const std::string& laneID) const;
        unsigned int getLinkNumber(const std::string& laneID) const;
        TraCIPositionVector getShape(const std::string& laneID) const;
        std::string getEdgeID(const std::string& laneID) const;
        SUMOReal getCO2Emission(const std::string& laneID) const;
        SUMOReal getCOEmission(const std::string& laneID) const;
        SUMOReal getHCEmission(const std::string& laneID) const;
        SUMOReal getPMxEmission(const std::string& laneID) const;
        SUMOReal getNOxEmission(const std::string& laneID) const;
        SUMOReal getFuelConsumption(const std::string& laneID) const;
        SUMOReal getNoiseEmission(const std::string& laneID) const;
        SUMOReal getLastStepMeanSpeed(const std::string& laneID) const;
        SUMOReal getLastStepOccupancy(const std::string& laneID) const;
        SUMOReal getLastStepLength(const std::string& laneID) const;
        SUMOReal getTraveltime(const std::string& laneID) const;
        unsigned int getLastStepVehicleNumber(const std::string& laneID) const;
        unsigned int getLastStepHaltingNumber(const std::string& laneID) const;
        std::vector<std::string> getLastStepVehicleIDs(const std::string& laneID) const;

        void setAllowed(const std::string& laneID, const std::vector<std::string>& allowedClasses) const;
        void setDisallowed(const std::string& laneID, const std::vector<std::string>& disallowedClasses) const;
        void setMaxSpeed(const std::string& laneID, SUMOReal speed) const;
        void setLength(const std::string& laneID, SUMOReal length) const;

    private:
        /// @brief invalidated copy constructor
        LaneScope(const LaneScope& src);

        /// @brief invalidated assignment operator
        LaneScope& operator=(const LaneScope& src);

    };





    /** @class MeMeScope
     * @brief Scope for interaction with multi entry/-exit detectors
     */
    class MeMeScope : public TraCIScopeWrapper {
    public:
        MeMeScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~MeMeScope() {}

        std::vector<std::string> getIDList() const;
        unsigned int getLastStepVehicleNumber(const std::string& detID) const;
        SUMOReal getLastStepMeanSpeed(const std::string& detID) const;
        std::vector<std::string> getLastStepVehicleIDs(const std::string& detID) const;
        unsigned int getLastStepHaltingNumber(const std::string& detID) const;

    private:
        /// @brief invalidated copy constructor
        MeMeScope(const MeMeScope& src);

        /// @brief invalidated assignment operator
        MeMeScope& operator=(const MeMeScope& src);

    };





    /** @class POIScope
     * @brief Scope for interaction with POIs
     */
    class POIScope : public TraCIScopeWrapper {
    public:
        POIScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~POIScope() {}

        std::vector<std::string> getIDList() const;
        std::string getType(const std::string& poiID) const;
        TraCIPosition getPosition(const std::string& poiID) const;
        TraCIColor getColor(const std::string& poiID) const;

        void setType(const std::string& poiID, const std::string& setType) const;
        void setPosition(const std::string& poiID, SUMOReal x, SUMOReal y) const;
        void setColor(const std::string& poiID, const TraCIColor& c) const;
        void add(const std::string& poiID, SUMOReal x, SUMOReal y, const TraCIColor& c, const std::string& type, int layer) const;
        void remove(const std::string& poiID, int layer = 0) const;

    private:
        /// @brief invalidated copy constructor
        POIScope(const POIScope& src);

        /// @brief invalidated assignment operator
        POIScope& operator=(const POIScope& src);

    };





    /** @class PolygonScope
     * @brief Scope for interaction with polygons
     */
    class PolygonScope : public TraCIScopeWrapper {
    public:
        PolygonScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~PolygonScope() {}

        std::vector<std::string> getIDList() const;
        std::string getType(const std::string& polygonID) const;
        TraCIPositionVector getShape(const std::string& polygonID) const;
        TraCIColor getColor(const std::string& polygonID) const;
        void setType(const std::string& polygonID, const std::string& setType) const;
        void setShape(const std::string& polygonID, const TraCIPositionVector& shape) const;
        void setColor(const std::string& polygonID, const TraCIColor& c) const;
        void add(const std::string& polygonID, const TraCIPositionVector& shape, const TraCIColor& c, bool fill, const std::string& type, int layer) const;
        void remove(const std::string& polygonID, int layer = 0) const;

    private:
        /// @brief invalidated copy constructor
        PolygonScope(const PolygonScope& src);

        /// @brief invalidated assignment operator
        PolygonScope& operator=(const PolygonScope& src);

    };





    /** @class RouteScope
     * @brief Scope for interaction with routes
     */
    class RouteScope : public TraCIScopeWrapper {
    public:
        RouteScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~RouteScope() {}

        std::vector<std::string> getIDList() const;
        std::vector<std::string> getEdges(const std::string& routeID) const;

        void add(const std::string& routeID, const std::vector<std::string>& edges) const;

    private:
        /// @brief invalidated copy constructor
        RouteScope(const RouteScope& src);

        /// @brief invalidated assignment operator
        RouteScope& operator=(const RouteScope& src);

    };





    /** @class SimulationScope
     * @brief Scope for interaction with the simulation
     */
    class SimulationScope : public TraCIScopeWrapper {
    public:
        SimulationScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~SimulationScope() {}

        SUMOTime getCurrentTime() const;
        unsigned int getLoadedNumber() const;
        std::vector<std::string> getLoadedIDList() const;
        unsigned int getDepartedNumber() const;
        std::vector<std::string> getDepartedIDList() const;
        unsigned int getArrivedNumber() const;
        std::vector<std::string> getArrivedIDList() const;
        unsigned int getStartingTeleportNumber() const;
        std::vector<std::string> getStartingTeleportIDList() const;
        unsigned int getEndingTeleportNumber() const;
        std::vector<std::string> getEndingTeleportIDList() const;
        SUMOTime getDeltaT() const;
        TraCIBoundary getNetBoundary() const;
        unsigned int getMinExpectedNumber() const;

    private:
        /// @brief invalidated copy constructor
        SimulationScope(const SimulationScope& src);

        /// @brief invalidated assignment operator
        SimulationScope& operator=(const SimulationScope& src);

    };





    /** @class TrafficLightScope
     * @brief Scope for interaction with traffic lights
     */
    class TrafficLightScope : public TraCIScopeWrapper {
    public:
        TrafficLightScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~TrafficLightScope() {}

        std::vector<std::string> getIDList() const;
        std::string getRedYellowGreenState(const std::string& tlsID) const;
        std::vector<TraCIAPI::TraCILogic> getCompleteRedYellowGreenDefinition(const std::string& tlsID) const;
        std::vector<std::string> getControlledLanes(const std::string& tlsID) const;
        std::vector<TraCIAPI::TraCILink> getControlledLinks(const std::string& tlsID) const;
        std::string getProgram(const std::string& tlsID) const;
        unsigned int getPhase(const std::string& tlsID) const;
        unsigned int getNextSwitch(const std::string& tlsID) const;

        void setRedYellowGreenState(const std::string& tlsID, const std::string& state) const;
        void setPhase(const std::string& tlsID, unsigned int index) const;
        void setProgram(const std::string& tlsID, const std::string& programID) const;
        void setPhaseDuration(const std::string& tlsID, unsigned int phaseDuration) const;
        void setCompleteRedYellowGreenDefinition(const std::string& tlsID, const TraCIAPI::TraCILogic& logic) const;

    private:
        /// @brief invalidated copy constructor
        TrafficLightScope(const TrafficLightScope& src);

        /// @brief invalidated assignment operator
        TrafficLightScope& operator=(const TrafficLightScope& src);

    };





    /** @class VehicleTypeScope
     * @brief Scope for interaction with vehicle types
     */
    class VehicleTypeScope : public TraCIScopeWrapper {
    public:
        VehicleTypeScope(TraCIAPI& parent) : TraCIScopeWrapper(parent) {}
        virtual ~VehicleTypeScope() {}

        std::vector<std::string> getIDList() const;
        SUMOReal getLength(const std::string& typeID) const;
        SUMOReal getMaxSpeed(const std::string& typeID) const;
        SUMOReal getSpeedFactor(const std::string& typeID) const;
        SUMOReal getSpeedDeviation(const std::string& typeID) const;
        SUMOReal getAccel(const std::string& typeID) const;
        SUMOReal getDecel(const std::string& typeID) const;
        SUMOReal getImperfection(const std::string& typeID) const;
        SUMOReal getTau(const std::string& typeID) const;
        std::string getVehicleClass(const std::string& typeID) const;
        std::string getEmissionClass(const std::string& typeID) const;
        std::string getShapeClass(const std::string& typeID) const;
        SUMOReal getMinGap(const std::string& typeID) const;
        SUMOReal getWidth(const std::string& typeID) const;
        TraCIColor getColor(const std::string& typeID) const;

        void setLength(const std::string& typeID, SUMOReal length) const;
        void setMaxSpeed(const std::string& typeID, SUMOReal speed) const;
        void setVehicleClass(const std::string& typeID, const std::string& clazz) const;
        void setSpeedFactor(const std::string& typeID, SUMOReal factor) const;
        void setSpeedDeviation(const std::string& typeID, SUMOReal deviation) const;
        void setEmissionClass(const std::string& typeID, const std::string& clazz) const;
        void setWidth(const std::string& typeID, SUMOReal width) const;
        void setMinGap(const std::string& typeID, SUMOReal minGap) const;
        void setShapeClass(const std::string& typeID, const std::string& clazz) const;
        void setAccel(const std::string& typeID, SUMOReal accel) const;
        void setDecel(const std::string& typeID, SUMOReal decel) const;
        void setImperfection(const std::string& typeID, SUMOReal imperfection) const;
        void setTau(const std::string& typeID, SUMOReal tau) const;
        void setColor(const std::string& typeID, const TraCIColor& c) const;

    private:
        /// @brief invalidated copy constructor
        VehicleTypeScope(const VehicleTypeScope& src);

        /// @brief invalidated assignment operator
        VehicleTypeScope& operator=(const VehicleTypeScope& src);

    };

public:
    /// @brief Scope for interaction with edges
    EdgeScope edge;
    /// @brief Scope for interaction with the gui
    GUIScope gui;
    /// @brief Scope for interaction with inductive loops
    InductionLoopScope inductionloop;
    /// @brief Scope for interaction with junctions
    JunctionScope junction;
    /// @brief Scope for interaction with lanes
    LaneScope lane;
    /// @brief Scope for interaction with multi-entry/-exit detectors
    MeMeScope multientryexit;
    /// @brief Scope for interaction with POIs
    POIScope poi;
    /// @brief Scope for interaction with polygons
    PolygonScope polygon;
    /// @brief Scope for interaction with routes
    RouteScope route;
    /// @brief Scope for interaction with the simulation
    SimulationScope simulation;
    /// @brief Scope for interaction with traffic lights
    TrafficLightScope trafficlights;
    /// @brief Scope for interaction with vehicle types
    VehicleTypeScope vehicletype;


protected:
    /// @name Command sending methods
    /// @{

    /** @brief Sends a SimulationStep command
     */
    void send_commandSimulationStep(SUMOTime time) const;


    /** @brief Sends a Close command
     */
    void send_commandClose() const;


    /** @brief Sends a GetVariable request
     * @param[in] domID The domain of the variable
     * @param[in] varID The variable to retrieve
     * @param[in] objID The object to retrieve the variable from
     * @param[in] add Optional additional parameter
     */
    void send_commandGetVariable(int domID, int varID, const std::string& objID, tcpip::Storage* add = 0) const;


    /** @brief Sends a SetVariable request
     * @param[in] domID The domain of the variable
     * @param[in] varID The variable to set
     * @param[in] objID The object to change
     * @param[in] content The value of the variable
     */
    void send_commandSetValue(int domID, int varID, const std::string& objID, tcpip::Storage& content) const;


    /** @brief Sends a SubscribeVariable request
     * @param[in] domID The domain of the variable
     * @param[in] objID The object to subscribe the variables from
     * @param[in] beginTime The begin time step of subscriptions
     * @param[in] endTime The end time step of subscriptions
     * @param[in] vars The variables to subscribe
     */
    void send_commandSubscribeObjectVariable(int domID, const std::string& objID, int beginTime, int endTime, const std::vector<int>& vars) const;


    /** @brief Sends a SubscribeContext request
     * @param[in] domID The domain of the variable
     * @param[in] objID The object to subscribe the variables from
     * @param[in] beginTime The begin time step of subscriptions
     * @param[in] endTime The end time step of subscriptions
     * @param[in] domain The domain of the objects which values shall be returned
     * @param[in] range The range around the obj to investigate
     * @param[in] vars The variables to subscribe
     */
    void send_commandSubscribeObjectContext(int domID, const std::string& objID, int beginTime, int endTime,
                                            int domain, SUMOReal range, const std::vector<int>& vars) const;
    /// @}



    /// @name Command sending methods
    /// @{

    /** @brief Validates the result state of a command
     * @param[in] inMsg The buffer to read the message from
     * @param[in] command The original command id
     * @param[in] ignoreCommandId Whether the returning command id shall be validated
     * @param[in] acknowledgement Pointer to an existing string into which the acknowledgement message shall be inserted
     */
    void check_resultState(tcpip::Storage& inMsg, int command, bool ignoreCommandId = false, std::string* acknowledgement = 0) const;

    void check_commandGetResult(tcpip::Storage& inMsg, int command, int expectedType = -1, bool ignoreCommandId = false) const;

    void processGET(tcpip::Storage& inMsg, int command, int expectedType, bool ignoreCommandId = false) const;
    /// @}


protected:
    /// @brief The socket
    tcpip::Socket* mySocket;


};


#endif

/****************************************************************************/

