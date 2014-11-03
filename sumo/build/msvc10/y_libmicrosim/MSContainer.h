/****************************************************************************/
/// @file    MSContainer.h
/// @author  Melanie Weber
/// @author  Andreas Kendziorra
/// @date    Thu, 12 Jun 2014
/// @version $$
///
// The class for modelling container-movements
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2001-2014 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSContainer_h
#define MSContainer_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <set>
#include <utils/common/SUMOTime.h>
//#include <utils/common/Command.h>
#include <utils/geom/Position.h>
#include <utils/geom/PositionVector.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSNet;
class MSEdge;
class MSLane;
class OutputDevice;
//class SUMOVehicleParameter;
//class MSBusStop;
class SUMOVehicle;
//class MSVehicleType;
//class MSPModel;
//class PedestrianState;

//typedef std::vector<const MSEdge*> MSEdgeVector;


// ===========================================================================
// class definitions
// ===========================================================================
/**
  * @class MSContainer
  *
  * The class holds a simulated container together with its movement stages
  */


class MSContainer {
public:
	enum StageType {
        DRIVING = 0,
        WAITING = 1
    };

	 /**
     * The "abstract" class for a single stage of a container movement
     * Contains the destination of the current movement step
     */
    class MSContainerStage {
    public:
        /// constructor
        MSContainerStage(const MSEdge& destination, StageType type);

        /// destructor
        virtual ~MSContainerStage();

		/// returns the destination edge
        const MSEdge& getDestination() const;

		/// Returns the current edge
        virtual const MSEdge* getEdge() const = 0;
        virtual const MSEdge* getFromEdge() const = 0;
        virtual SUMOReal getEdgePos(SUMOTime now) const = 0;

		///
        virtual Position getPosition(SUMOTime now) const = 0;

		///
        StageType getStageType() const {
            return myType;
        }

        /// @brief return string representation of the current stage
        virtual std::string getStageTypeName() const = 0;

        /// proceeds to the next step
		virtual void proceed(MSNet* net, MSContainer* container, SUMOTime now, MSEdge* previousEdge, const SUMOReal at) = 0;

		/// logs end of the step
        void setDeparted(SUMOTime now);

        /// logs end of the step
        void setArrived(SUMOTime now);

        /// Whether the container waits for a vehicle of the line specified.
        virtual bool isWaitingFor(const std::string& line) const;

        /// @brief Whether the container waits for a vehicle
        virtual bool isWaiting4Vehicle() const {
            return false;
        }
		
        /// @brief the time this container spent waiting
        virtual SUMOTime getWaitingTime(SUMOTime now) const = 0;

        /// @brief the speed of the container
        virtual SUMOReal getSpeed() const = 0;

        /// @brief get position on edge e at length at with orthogonal offset
        Position getEdgePosition(const MSEdge* e, SUMOReal at, SUMOReal offset) const;

        /// @brief get position on lane at length at with orthogonal offset
        Position getLanePosition(const MSLane* lane, SUMOReal at, SUMOReal offset) const;

		/** @brief Called on writing tripinfo output
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void tripInfoOutput(OutputDevice& os) const = 0;

		/** @brief Called on writing vehroute output
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void routeOutput(OutputDevice& os) const = 0;

		/** @brief Called for writing the events output (begin of an action)
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void beginEventOutput(const MSContainer& container, SUMOTime t, OutputDevice& os) const = 0;

		/** @brief Called for writing the events output (end of an action)
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void endEventOutput(const MSContainer& container, SUMOTime t, OutputDevice& os) const = 0;
	
	protected:
        /// the next edge to reach by getting transported
        const MSEdge& myDestination;

		/// the time at which this stage started
        SUMOTime myDeparted;

		/// the time at which this stage ended
        SUMOTime myArrived;

        /// The type of this stage
        StageType myType;

    private:
        /// @brief Invalidated copy constructor.
        MSContainerStage(const MSContainerStage&);

        /// @brief Invalidated assignment operator.
        MSContainerStage& operator=(const MSContainerStage&);

    };

	/**
     * A "real" stage performing the travelling by a transport system
     * The given route will be chosen. The travel time is computed by the simulation
     */
    class MSContainerStage_Driving : public MSContainerStage {
    public:
        /// constructor
        MSContainerStage_Driving(const MSEdge& destination, MSTerminalStop* toTS,
                              const std::vector<std::string>& lines);

		/// destructor
        ~MSContainerStage_Driving();

        /// proceeds to the next step
        virtual void proceed(MSNet* net, MSContainer* container, SUMOTime now, MSEdge* previousEdge, const SUMOReal at);
		
        /// Returns the current edge
        const MSEdge* getEdge() const;
        const MSEdge* getFromEdge() const;
        SUMOReal getEdgePos(SUMOTime now) const;

        ///
        Position getPosition(SUMOTime now) const;

        std::string getStageTypeName() const;

        /// Whether the container waits for a vehicle of the line specified.
        bool isWaitingFor(const std::string& line) const;

		/// @brief Whether the container waits for a vehicle
        bool isWaiting4Vehicle() const;

        /// @brief time spent waiting for a ride
        SUMOTime getWaitingTime(SUMOTime now) const;

		/// @brief the speed of the container
        SUMOReal getSpeed() const;

		/// @brief assign a vehicle to the container
		void setVehicle(SUMOVehicle* v) {
            myVehicle = v;
        }

		/** @brief Called on writing tripinfo output
         *
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void tripInfoOutput(OutputDevice& os) const;

        /** @brief Called on writing vehroute output
         *
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void routeOutput(OutputDevice& os) const;

        /** @brief Called for writing the events output
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void beginEventOutput(const MSContainer& container, SUMOTime t, OutputDevice& os) const;

        /** @brief Called for writing the events output (end of an action)
         * @param[in] os The stream to write the information into
         * @exception IOError not yet implemented
         */
        virtual void endEventOutput(const MSContainer& container, SUMOTime t, OutputDevice& os) const;

	private:
        /// the lines  to choose from
        const std::set<std::string> myLines;

        /// @brief The taken vehicle
        SUMOVehicle* myVehicle;

        MSBusStop* myDestinationBusStop;
        SUMOReal myWaitingPos;
        /// @brief The time since which this person is waiting for a ride
        SUMOTime myWaitingSince;
        const MSEdge* myWaitingEdge;
