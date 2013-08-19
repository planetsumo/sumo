/****************************************************************************/
/// @file    MSSOTLE2Sensors.cpp
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSSOTLE2Sensors.cpp 3 2010-03-03 15:00:00Z gslager $
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

#include "MSSOTLE2Sensors.h"

MSSOTLE2Sensors::MSSOTLE2Sensors(std::string tlLogicID, const MSTrafficLightLogic::Phases *phases)
: MSSOTLSensors(tlLogicID, phases) {
}

MSSOTLE2Sensors::~MSSOTLE2Sensors(void)
{
	//Delete sensors
	for (MSLane_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsMap.begin(); sensorsIterator != mySensorsMap.end(); sensorsIterator++) {
		delete (sensorsIterator->second);
	}
}

void
MSSOTLE2Sensors::buildSensors(MSTrafficLightLogic::LaneVectorVector controlledLanes, NLDetectorBuilder &nb) {
	//for each lane build an appropriate sensor on it
	MSLane *currentLane = NULL;
	for (MSTrafficLightLogic::LaneVectorVector::const_iterator laneVector = controlledLanes.begin(); laneVector != controlledLanes.end(); laneVector++) {
		for (MSTrafficLightLogic::LaneVector::const_iterator lane = laneVector->begin(); lane != laneVector->end(); lane++) {
			currentLane = (*lane);
			buildSensorForLane(currentLane, nb);
		}
	}
}

void
MSSOTLE2Sensors::buildSensorForLane(MSLane* lane, NLDetectorBuilder &nb) {
	float sensorPos;
	double lensorLength;
	MSE2Collector *newSensor = NULL;
	//Check not to have more than a sensor for lane
	if (mySensorsMap.find(lane) == mySensorsMap.end()) {
		//Check and set zero if the lane is not long enough for the specified sensor start
		sensorPos = SENSOR_START <= lane->getLength() ? SENSOR_START : 0;
		//Check and trim if the lane is not long enough for the specified sensor lenght
		lensorLength = SENSOR_LENGTH <= (lane->getLength() - sensorPos) ? SENSOR_LENGTH : (lane->getLength() - sensorPos);
		//Create sensor for lane and insert it into the map<MSLane*, MSE2Collector*>
		newSensor = nb.buildSingleLaneE2Det("SOTL_E2_lane:"+lane->getID()+"_tl:"+tlLogicID, DU_TL_CONTROL, lane, (lane->getLength() - sensorPos - lensorLength), lensorLength, HALTING_TIME_THRS, HALTING_SPEED_THRS, DIST_THRS);
		mySensorsMap.insert(MSLane_MSE2Collector(lane, newSensor));
		mySensorsIDMap.insert(MSLaneID_MSE2Collector(lane->getID(), newSensor));
		myMaxSpeedMap.insert(MSLaneID_MaxSpeed(lane->getID(), lane->getSpeedLimit()));
	}
}

unsigned int
MSSOTLE2Sensors::countVehicles(MSLane* lane) {
	MSLane_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsMap.find(lane);
	if (sensorsIterator == mySensorsMap.end()) {assert(0); return 0;}
	else 
		return sensorsIterator->second->getCurrentVehicleNumber();
}

unsigned int
MSSOTLE2Sensors::countVehicles(std::string laneId) {
	MSLaneID_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsIDMap.find(laneId);
	if (sensorsIterator == mySensorsIDMap.end()) {assert(0); return 0;}
	else 
		return sensorsIterator->second->getCurrentVehicleNumber();
}

double
MSSOTLE2Sensors::getMaxSpeed(std::string laneId) {
	MSLaneID_MaxSpeedMap::const_iterator sensorsIterator = myMaxSpeedMap.find(laneId);
	if (sensorsIterator == myMaxSpeedMap.end()) {assert(0); return 0;}
	else
		return sensorsIterator->second;
}

double
MSSOTLE2Sensors::meanVehiclesSpeed(MSLane* lane) {
	MSLane_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsMap.find(lane);
	if (sensorsIterator == mySensorsMap.end()) {assert(0); return 0;}
	else 
		return sensorsIterator->second->getCurrentMeanSpeed();
}

double
MSSOTLE2Sensors::meanVehiclesSpeed(std::string laneId) {
	MSLaneID_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsIDMap.find(laneId);
	if (sensorsIterator == mySensorsIDMap.end()) {assert(0); return 0;}
	else 
		return sensorsIterator->second->getCurrentMeanSpeed();
}