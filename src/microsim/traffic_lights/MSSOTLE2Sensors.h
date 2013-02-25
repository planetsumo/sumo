/****************************************************************************/
/// @file    MSSOTLE2Sensors.h
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSSOTLE2Sensors.h 3 2010-03-03 15:00:00Z gslager $
///
// The class for SOTL sensors of "E2" type
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2009 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSSOTLE2Sensors_h
#define MSSOTLE2Sensors_h

#include "MSSOTLSensors.h"
#include <math.h>
#include <map>
#include <utility>
#include <microsim/output/MSE2Collector.h>

class MSSOTLE2Sensors :	public MSSOTLSensors
{
protected :
	void buildSensorForLane(MSLane* lane, NLDetectorBuilder &nb);

public:
	//Every lane has its own sensor
	//Sensors can be retrieved by lanes pointer
	typedef std::pair<MSLane*, MSE2Collector*> MSLane_MSE2Collector;
    typedef std::map<MSLane*, MSE2Collector*> MSLane_MSE2CollectorMap;
	//Sensors can be retrieved by lane Id
	typedef std::pair<std::string, MSE2Collector*> MSLaneID_MSE2Collector;
	typedef std::map<std::string, MSE2Collector*> MSLaneID_MSE2CollectorMap;

	//Every lane has its speed limit, it can be retrieved by lane Id
	typedef std::pair<std::string, double> MSLaneID_MaxSpeed;
	typedef std::map<std::string, double> MSLaneID_MaxSpeedMap;

public:
	/*
	* @brief This sensor logic contructor
	*/
	MSSOTLE2Sensors(std::string tlLogicID, MSTrafficLightLogic::Phases *phases);

	/*
	* @brief This sensor logic destructor
	*/
	~MSSOTLE2Sensors(void);

	void buildSensors(MSTrafficLightLogic::LaneVectorVector controlledLanes, NLDetectorBuilder &nb);

	/*
	 * Returns the number of vehicles currently approaching the 
	 * junction for the given lane.
	 * Vehicles are effectively counted or guessed in the space from the sensor to the junction.
	 * @param[in] lane The lane to count vehicles
	 */
	unsigned int countVehicles(MSLane* lane);

	/*
	 * Returns the number of vehicles currently approaching the 
	 * junction for the given lane.
	 * Vehicles are effectively counted or guessed in the space from the sensor to the junction.
	 * @param[in] lane The lane to count vehicles given by ID
	 */
	unsigned int countVehicles(std::string laneId);

	/*
	* @param[in] The lane given by Id
	* @return The maximum speed allowed for the given laneId
	*/
	virtual double getMaxSpeed(std::string laneId);

	/*
	 * Returns the average speed of vehicles currently approaching the 
	 * junction for the given lane.
	 * Vehicles speed is effectively sensed or guessed in the space from the sensor.
	 * @param[in] lane The lane to count vehicles
	 */
	virtual double meanVehiclesSpeed(MSLane* lane);

	/*
	 * Returns the average speed of vehicles currently approaching the 
	 * junction for the given lane.
	 * Vehicles speed is effectively sensed or guessed in the space from the sensor.
	 * @param[in] laneID The lane to count vehicles by ID
	 */
	virtual double meanVehiclesSpeed(std::string laneId);

protected:
	MSLane_MSE2CollectorMap mySensorsMap;
	MSLaneID_MSE2CollectorMap mySensorsIDMap;
	MSLaneID_MaxSpeedMap myMaxSpeedMap;
};

#endif
/****************************************************************************/