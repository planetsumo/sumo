/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSVehicleControl.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Wed, 10. Dec 2003
/// @version $Id$
///
// The class responsible for building and deletion of vehicles
/****************************************************************************/
#ifndef MSVehicleControl_h
#define MSVehicleControl_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cmath>
#include <string>
#include <map>
#include <set>
#include <utils/distribution/RandomDistributor.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/SUMOVehicleClass.h>
#include "MSNet.h"


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVehicle;
class SUMOVehicleParameter;
class MSVehicle;
class MSRoute;
class MSVehicleType;
class OutputDevice;
class MSEdge;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSVehicleControl
 * @brief The class responsible for building and deletion of vehicles
 *
 * This class is responsible for vehicle building and deletion. It stores
 *  vehicle types, vehicles and statistics about the last.
 *
 * This class also realizes the tripinfos and the vehroutes - outputs, both
 *  generated when a vehicle is removed from the simulation, see
 *  scheduleVehicleRemoval.
 *
 * Use this class for the pure microsim and GUIVehicleControl within the gui.
 *
 * @see GUIVehicleControl
 */
class MSVehicleControl {
public:
    /// @brief Definition of the internal vehicles map iterator
    typedef std::map<std::string, SUMOVehicle*>::const_iterator constVehIt;

public:
    /// @brief Constructor
    MSVehicleControl();


    /// @brief Destructor
    virtual ~MSVehicleControl();


    /// @name Vehicle creation
    /// @{

    /** @brief Builds a vehicle, increases the number of built vehicles
     *
     * Builds a MSVehicle instance using the given parameter.
     *  Increases the number of loaded vehicles ("myLoadedVehNo").
     *
     * @param[in] defs The parameter defining the vehicle
     * @param[in] route The route of this vehicle
     * @param[in] type The type of this vehicle
     * @param[in] ignoreStopErrors whether invalid stops trigger a warning only
     * @param[in] fromRouteFile whether we are just reading the route file or creating via trigger, traci, ...
     * @return The built vehicle (MSVehicle instance)
     */
    virtual SUMOVehicle* buildVehicle(SUMOVehicleParameter* defs, const MSRoute* route,
                                      MSVehicleType* type,
                                      const bool ignoreStopErrors, const bool fromRouteFile = true);
    /// @}



    /// @name Insertion, deletion and retrieval of vehicles
    /// @{

    /** @brief Tries to insert the vehicle into the internal vehicle container
     *
     * Checks whether another vehicle with the same id exists; returns false
     *  if so. Otherwise, the vehicle is added to "myVehicleDict".
     *  It also checks whether the vehicle has a "triggered" departure
     *  and registers it accordingly.
     *
     * The vehicle control gets responsible for vehicle deletion.
     *
     * @param[in] id The id of the vehicle
     * @param[in] v The vehicle
     * @return Whether the vehicle could be inserted (no other vehicle with the same id was inserted before)
     */
    virtual bool addVehicle(const std::string& id, SUMOVehicle* v);


    /** @brief Returns the vehicle with the given id
     *
     * If no vehicle with the given id is store din "myVehicleDict", 0
     *  is returned.
     *
     * @param[in] id The id of the vehicle to retrieve
     * @return The vehicle with the given id, 0 if no such vehicle exists
     */
    SUMOVehicle* getVehicle(const std::string& id) const;


    /** @brief Deletes the vehicle
     *
     * @param[in] v The vehicle to delete
     * @param[discard] Whether the vehicle is discard during loading (scale < 1)
     * @todo Isn't this quite insecure?
     */
    virtual void deleteVehicle(SUMOVehicle* v, bool discard = false);


    /** @brief Removes a vehicle after it has ended
     *
     * Writes output to tripinfos and vehroutes if wished; decrements
     *  the number of running vehicles and increments the number of ended
     *  vehicles. Then deletes the vehicle using "deleteVehicle".
     *
     * This method should be called for each vehicle that was inserted
     *  into the network and quits its ride.
     *
     * @param[in] veh The vehicle to remove
     */
    void scheduleVehicleRemoval(SUMOVehicle* veh);


    /** @brief Returns the begin of the internal vehicle map
     * @return The begin of the internal vehicle map
     */
    constVehIt loadedVehBegin() const {
        return myVehicleDict.begin();
    }


    /** @brief Returns the end of the internal vehicle map
     * @return The end of the internal vehicle map
     */
    constVehIt loadedVehEnd() const {
        return myVehicleDict.end();
    }
    /// @}



    /// @name Setting vehicle statistics
    /// @{

    /** @brief Informs this control about a vehicle's departure
     *
     * If the mean waiting time shall be computed (f.e. for summary-output),
     *  the absolut waiting time is increased by the waiting time of the given
     *  vehicle.
     * @param[in] v The inserted vehicle
     */
    void vehicleDeparted(const SUMOVehicle& v);
    /// @}



    /// @name Retrieval of vehicle statistics (always accessable)
    /// @{

    /** @brief Returns the number of build vehicles
     * @return The number of loaded (build) vehicles
     */
    int getLoadedVehicleNo() const {
        return myLoadedVehNo;
    }


    /** @brief Returns the number of removed vehicles
     * @return The number of vehicles that have left the simulation
     */
    int getEndedVehicleNo() const {
        return myEndedVehNo;
    }


    /** @brief Returns the number of build and inserted, but not yet deleted vehicles
     * @return The number simulated vehicles (including those in teleporter)
     */
    int getRunningVehicleNo() const {
        return myRunningVehNo;
    }


    /** @brief Returns the number of inserted vehicles
     * @return The number of vehicles that have entered the simulation so far
     */
    int getDepartedVehicleNo() const {
        return myRunningVehNo + myEndedVehNo - myDiscarded;
    }


    /** @brief Returns the number of instances of the current vehicle that shall be emitted
     * considering that "frac" of all vehicles shall be emitted overall
     * if a negative fraction is given the demand scaling factor is used
     * (--scale)
     * @return the number of vehicles to create (something between 0 and ceil(frac))
     */
    int getQuota(double frac = -1) const;


    /** @brief Returns the number of build vehicles that have not been removed or
     * need to wait for a passenger or a container
     * @return Number of active vehicles
     */
    int getActiveVehicleCount() const {
        return myLoadedVehNo - (myWaitingForPerson + myWaitingForContainer + myEndedVehNo);
    }


    /// @brief return the number of collisions
    int getCollisionCount() const {
        return myCollisions;
    }

    /// @brief return the number of teleports due to jamming
    int getTeleportsJam() const {
        return myTeleportsJam;
    }

    /// @brief return the number of teleports due to vehicles stuck on a minor road
    int getTeleportsYield() const {
        return myTeleportsYield;
    }

    /// @brief return the number of teleports due to vehicles stuck on the wrong lane
    int getTeleportsWrongLane() const {
        return myTeleportsWrongLane;
    }

    /// @brief return the number of teleports (including collisions)
    int getTeleportCount() const;

    /// @brief return the number of emergency stops
    int getEmergencyStops() const {
        return myEmergencyStops;
    }


    /** @brief Returns the total departure delay
     * @return Sum of steps vehicles had to wait until being inserted
     */
    double getTotalDepartureDelay() const {
        return myTotalDepartureDelay;
    }


    /** @brief Returns the total travel time
     * @return Sum of travel times of arrived vehicles
     */
    double getTotalTravelTime() const {
        return myTotalTravelTime;
    }
    /// @}



    /// @name Insertion and retrieval of vehicle types
    /// @{

    /** @brief Adds a vehicle type
     *
     * If another vehicle type (or distribution) with the same id exists, false is returned.
     *  Otherwise, the vehicle type is added to the internal vehicle type
     *  container "myVTypeDict".
     *
     * This control get responsible for deletion of the added vehicle
     *  type.
     *
     * @param[in] vehType The vehicle type to add
     * @return Whether the vehicle type could be added
     */
    bool addVType(MSVehicleType* vehType);

    /*
     * @param[in] vehType The vehicle type to remove
     * @return Whether the vehicle type could be removed
     */
    void removeVType(const MSVehicleType* vehType);


    /** @brief Adds a vehicle type distribution
     *
     * If another vehicle type (or distribution) with the same id exists, false is returned.
     *  Otherwise, the vehicle type distribution is added to the internal vehicle type distribution
     *  container "myVTypeDistDict".
     *
     * This control get responsible for deletion of the added vehicle
     *  type distribution.
     *
     * @param[in] id The id of the distribution to add
     * @param[in] vehTypeDistribution The vehicle type distribution to add
     * @return Whether the vehicle type could be added
     */
    bool addVTypeDistribution(const std::string& id, RandomDistributor<MSVehicleType*>* vehTypeDistribution);


    /** @brief Asks for existence of a vehicle type
     *
     * If vehicle type or distribution with the id exists, true is returned, false otherwise.
     *
     * @param[in] id The id of the type or distribution
     * @return Whether the vehicle type or distribution exists
     */
    bool hasVType(const std::string& id) const;


    /** @brief Asks for a vehicle type distribution
     *
     * If vehicle type distribution with the id exists, true is returned, false otherwise.
     *
     * @param[in] id The id of the distribution
     * @return Whether the vehicle type distribution exists
     */
    bool hasVTypeDistribution(const std::string& id) const;


    /** @brief Returns the named vehicle type or a sample from the named distribution
     * @param[in] id The id of the vehicle type to return. If left out, the default type is returned.
     * @return The named vehicle type, or 0 if no such type exists
     */
    MSVehicleType* getVType(const std::string& id = DEFAULT_VTYPE_ID, std::mt19937* rng = 0);


    /** @brief Inserts ids of all known vehicle types and vehicle type distributions to the given vector
     * @param[in] into The vector to fill with ids
     */
    void insertVTypeIDs(std::vector<std::string>& into) const;
    /// @}

    /// @brief Adds a vehicle to the list of waiting vehiclse to a given edge
    void addWaiting(const MSEdge* const edge, SUMOVehicle* vehicle);

    /// @brief Removes a vehicle from the list of waiting vehicles to a given edge
    void removeWaiting(const MSEdge* const edge, SUMOVehicle* vehicle);

    /* @brief returns a vehicle of the given lines that is waiting for a for a person or a container at this edge at the given positions
     * @param[in] edge The edge at which the vehicle is positioned.
     * @param[in] lines The set of lines from which at least one must correspond to the line of the vehicle
     * @param[in] position The vehicle shall be positioned in the interval [position - t, position + t], where t is some tolerance
     * @param[in] ridingID The id of the person or container that wants to ride
     */
    SUMOVehicle* getWaitingVehicle(const MSEdge* const edge, const std::set<std::string>& lines, const double position, const std::string ridingID);

    /** @brief increases the count of vehicles waiting for a transport to allow recognition of person / container related deadlocks
     */
    void registerOneWaiting(const bool isPerson) {
        if (isPerson) {
            myWaitingForPerson++;
        } else {
            myWaitingForContainer++;
        }
    }

    /** @brief decreases the count of vehicles waiting for a transport to allow recognition of person / container related deadlocks
     */
    void unregisterOneWaiting(const bool isPerson) {
        if (isPerson) {
            myWaitingForPerson--;
        } else {
            myWaitingForContainer--;
        }
    }

    /// @brief registers one collision-related teleport
    void registerCollision() {
        myCollisions++;
    }

    /// @brief register one non-collision-related teleport
    void registerTeleportJam() {
        myTeleportsJam++;
    }

    /// @brief register one non-collision-related teleport
    void registerTeleportYield() {
        myTeleportsYield++;
    }

    /// @brief register one non-collision-related teleport
    void registerTeleportWrongLane() {
        myTeleportsWrongLane++;
    }

    /// @brief register emergency stop
    void registerEmergencyStop() {
        myEmergencyStops++;
    }

    /// @name State I/O
    /// @{

    /** @brief Sets the current state variables as loaded from the stream
     */
    void setState(int runningVehNo, int loadedVehNo, int endedVehNo, double totalDepartureDelay, double totalTravelTime);

    /** @brief Saves the current state into the given stream
     */
    void saveState(OutputDevice& out);
    /// @}

    /// @brief avoid counting a vehicle twice if it was loaded from state and route input
    void discountStateLoaded(bool removed = false) {
        if (removed) {
            myRunningVehNo--;
            myDiscarded++;
            myEndedVehNo++;
        } else {
            myLoadedVehNo--;
        }
    }


    /** @brief informes about all waiting vehicles (deletion in destructor)
     */
    void abortWaiting();

    /// @brief compute (optional) random offset to the departure time
    SUMOTime computeRandomDepartOffset() const;

    /// @brief return the maximum speed factor for all vehicles that ever entered the network
    double getMaxSpeedFactor() const {
        return myMaxSpeedFactor;
    }

    /// @brief return the minimum deceleration capability for all vehicles that ever entered the network
    double getMinDeceleration() const {
        return myMinDeceleration;
    }

    void adaptIntermodalRouter(MSNet::MSIntermodalRouter& router) const;

private:
    /** @brief Checks whether the vehicle type (distribution) may be added
     *
     * This method checks also whether the default type may still be replaced
     * @param[in] id The id of the vehicle type (distribution) to add
     * @return Whether the type (distribution) may be added
     */
    bool checkVType(const std::string& id);

protected:
    /// @name Vehicle statistics (always accessible)
    /// @{

    /// @brief The number of build vehicles
    int myLoadedVehNo;

private:
    /// @brief The number of vehicles within the network (build and inserted but not removed)
    int myRunningVehNo;

    /// @brief The number of removed vehicles
    int myEndedVehNo;

    /// @brief The number of vehicles which were discarded while loading
    int myDiscarded;

    /// @brief The number of collisions
    int myCollisions;

    /// @brief The number of teleports due to jam
    int myTeleportsJam;

    /// @brief The number of teleports due to vehicles stuck on a minor road
    int myTeleportsYield;

    /// @brief The number of teleports due to vehicles stuck on the wrong lane
    int myTeleportsWrongLane;

    /// @brief The number of emergency stops
    int myEmergencyStops;

    /// @}


    /// @name Vehicle statistics
    /// @{

    /// @brief The aggregated time vehicles had to wait for departure (in seconds)
    double myTotalDepartureDelay;

    /// @brief The aggregated time vehicles needed to aacomplish their route (in seconds)
    double myTotalTravelTime;
    /// @}


protected:
    /// @name Vehicle container
    /// @{

    /// @brief Vehicle dictionary type
    typedef std::map< std::string, SUMOVehicle* > VehicleDictType;
    /// @brief Dictionary of vehicles
    VehicleDictType myVehicleDict;
    /// @}


private:
    /// @name Vehicle type container
    /// @{

    /// @brief Vehicle type dictionary type
    typedef std::map< std::string, MSVehicleType* > VTypeDictType;
    /// @brief Dictionary of vehicle types
    VTypeDictType myVTypeDict;

    /// @brief Vehicle type distribution dictionary type
    typedef std::map< std::string, RandomDistributor<MSVehicleType*>* > VTypeDistDictType;
    /// @brief A distribution of vehicle types (probability->vehicle type)
    VTypeDistDictType myVTypeDistDict;

    /// @brief Whether no vehicle type was loaded
    bool myDefaultVTypeMayBeDeleted;

    /// @brief Whether no pedestrian type was loaded
    bool myDefaultPedTypeMayBeDeleted;

    /// the lists of waiting vehicles to a given edge
    std::map<const MSEdge* const, std::vector<SUMOVehicle*> > myWaiting;

    /// the number of vehicles wainting for persons contained in myWaiting which can only continue by being triggered
    int myWaitingForPerson;

    /// the number of vehicles wainting for containers contained in myWaiting which can only continue by being triggered
    int myWaitingForContainer;

    /// @brief The scaling factor (especially for inc-dua)
    double myScale;

    /// @brief The maximum random offset to be added to vehicles departure times (non-negative)
    SUMOTime myMaxRandomDepartOffset;

    /// @brief The maximum speed factor for all vehicles in the network
    double myMaxSpeedFactor;

    /// @brief The minimum deceleration capability for all vehicles in the network
    double myMinDeceleration;

    /// @brief List of vehicles which belong to public transport
    std::vector<SUMOVehicle*> myPTVehicles;

private:
    /// @brief invalidated copy constructor
    MSVehicleControl(const MSVehicleControl& s);

    /// @brief invalidated assignment operator
    MSVehicleControl& operator=(const MSVehicleControl& s);


};


#endif

/****************************************************************************/

