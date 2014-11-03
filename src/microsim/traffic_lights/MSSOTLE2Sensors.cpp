/****************************************************************************/
/// @file    MSSOTLE2Sensors.cpp
/// @author  Gianfilippo Slager
/// @author  Alessio Bonfietti
/// @date    Feb 2010
/// @version $Id: MSSOTLE2Sensors.cpp 3 2010-03-03 15:00:00Z gslager $
///
// The class for SOTL sensors of "E2" type
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
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
#include <microsim/output/MSDetectorControl.h>

MSSOTLE2Sensors::MSSOTLE2Sensors(std::string tlLogicID,
		const MSTrafficLightLogic::Phases *phases) :
		MSSOTLSensors(tlLogicID, phases) {
}

MSSOTLE2Sensors::~MSSOTLE2Sensors(void) {
	//Delete sensors
	for (MSLane_MSE2CollectorMap::const_iterator sensorsIterator =
			mySensorsMap_InLanes.begin(); sensorsIterator != mySensorsMap_InLanes.end();
			sensorsIterator++) {
		delete (sensorsIterator->second);
	}
}

void MSSOTLE2Sensors::buildSensors(
		MSTrafficLightLogic::LaneVectorVector controlledLanes,
		NLDetectorBuilder &nb){
	buildSensors(controlledLanes,nb,INPUT_SENSOR_LENGTH);
}

void MSSOTLE2Sensors::buildSensors(
		MSTrafficLightLogic::LaneVectorVector controlledLanes,
		NLDetectorBuilder &nb, double sensorLength) {
	//for each lane build an appropriate sensor on it
	MSLane *currentLane = NULL;

	//input and ouput lanes
	for (MSTrafficLightLogic::LaneVectorVector::const_iterator laneVector =
			controlledLanes.begin(); laneVector != controlledLanes.end();
			laneVector++) {
		for (MSTrafficLightLogic::LaneVector::const_iterator lane =
				laneVector->begin(); lane != laneVector->end(); lane++) {
			currentLane = (*lane);
			buildSensorForLane(currentLane, nb,sensorLength);
		}
	}
}

void MSSOTLE2Sensors::buildOutSensors(
		MSTrafficLightLogic::LaneVectorVector controlledLanes,
		NLDetectorBuilder &nb){
	buildOutSensors(controlledLanes,nb,OUTPUT_SENSOR_LENGTH);
}
void MSSOTLE2Sensors::buildOutSensors(
		MSTrafficLightLogic::LaneVectorVector controlledLanes,
		NLDetectorBuilder &nb, double sensorLength) {
	//for each lane build an appropriate sensor on it
	MSLane *currentLane = NULL;

	//input and ouput lanes
	for (MSTrafficLightLogic::LaneVectorVector::const_iterator laneVector =
			controlledLanes.begin(); laneVector != controlledLanes.end();
			laneVector++) {
		for (MSTrafficLightLogic::LaneVector::const_iterator lane =
				laneVector->begin(); lane != laneVector->end(); lane++) {
			currentLane = (*lane);
			buildSensorForOutLane(currentLane, nb, sensorLength);
		}
	}
}

/* @brief Builds an e2 detector that lies on only one lane
 *
 * @param[in] id The id the detector shall have
 * @param[in] usage Information how the detector is used within the simulation
 * @param[in] lane The lane the detector is placed at
 * @param[in] pos The position on the lane the detector is placed at
 * @param[in] length The length the detector has
 * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
 * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
 * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
 * @todo Check whether this method is really needful
 */
/*
 Using
 MSE2Collector* buildSingleLaneE2Det(const std::string& id,
 DetectorUsage usage, MSLane* lane, SUMOReal pos, SUMOReal length,
 SUMOTime haltingTimeThreshold, SUMOReal haltingSpeedThreshold,
 SUMOReal jamDistThreshold);

 */

void MSSOTLE2Sensors::buildSensorForLane(MSLane* lane, NLDetectorBuilder &nb){
	buildSensorForLane(lane,nb,INPUT_SENSOR_LENGTH);
}

void MSSOTLE2Sensors::buildSensorForLane(MSLane* lane, NLDetectorBuilder &nb, double sensorLength) {
	float sensorPos;
	double lensorLength;
	MSE2Collector *newSensor = NULL;
	//Check not to have more than a sensor for lane
	if (mySensorsMap_InLanes.find(lane) == mySensorsMap_InLanes.end()) {

		//Check and set zero if the lane is not long enough for the specified sensor start
		sensorPos = SENSOR_START <= lane->getLength() ? SENSOR_START : 0;

		//Original:
		//double sensorLength = INPUT_SENSOR_LENGTH;

		//Check and trim if the lane is not long enough for the specified sensor lenght
		lensorLength =
				sensorLength <= (lane->getLength() - sensorPos) ?
						sensorLength : (lane->getLength() - sensorPos);

		//TODO check this lengths
		DBG(
		std::ostringstream phero_str;
		phero_str << " lane " << lane->getID() << " sensorPos= " << sensorPos
				<< " ,SENSOR_START  " << SENSOR_START << "; lane->getLength = "
				<< lane->getLength() << " ,lensorLength= " << lensorLength
				<< " ,SENSOR_LENGTH= " << INPUT_SENSOR_LENGTH;
		WRITE_MESSAGE(
				"MSSOTLE2Sensors::buildSensorForLane::" + phero_str.str());
		)

		//Create sensor for lane and insert it into the map<MSLane*, MSE2Collector*>
		newSensor = nb.buildSingleLaneE2Det(
				"SOTL_E2_lane:" + lane->getID() + "_tl:" + tlLogicID,
				DU_TL_CONTROL, lane,
				(lane->getLength() - sensorPos - lensorLength), lensorLength,
				HALTING_TIME_THRS, HALTING_SPEED_THRS, DIST_THRS);
//newSensor = nb.buildSingleLaneE2Det("SOTL_E2_lane:"+lane->getID()+"_tl:"+tlLogicID, DU_TL_CONTROL, lane, (lane->getLength() - sensorPos- 5), lensorLength, HALTING_TIME_THRS, HALTING_SPEED_THRS, DIST_THRS);

		MSNet::getInstance()->getDetectorControl().add(
				SUMO_TAG_LANE_AREA_DETECTOR, newSensor);

		mySensorsMap_InLanes.insert(MSLane_MSE2Collector(lane, newSensor));
		mySensorsIDMap_InLanes.insert(MSLaneID_MSE2Collector(lane->getID(), newSensor));
		myMaxSpeedMap_InLanes.insert(
				MSLaneID_MaxSpeed(lane->getID(), lane->getSpeedLimit()));
	}
}

void MSSOTLE2Sensors::buildSensorForOutLane(MSLane* lane,
		NLDetectorBuilder &nb){
	buildSensorForOutLane(lane,nb,OUTPUT_SENSOR_LENGTH);
}

void MSSOTLE2Sensors::buildSensorForOutLane(MSLane* lane,
		NLDetectorBuilder &nb, double sensorLength) {
	float sensorPos;
	double lensorLength;
	MSE2Collector *newSensor = NULL;
	//Check not to have more than a sensor for lane
	if (mySensorsMap_OutLanes.find(lane) == mySensorsMap_OutLanes.end()) {

		//Original:
		//double sensorLength = OUTPUT_SENSOR_LENGTH;
		//Check and set zero if the lane is not long enough for the specified sensor start
		sensorPos = (lane->getLength() - sensorLength)
				- (SENSOR_START <= lane->getLength() ? SENSOR_START : 0);

		//Check and trim if the lane is not long enough for the specified sensor lenght
		lensorLength =
				sensorLength <= (lane->getLength() - sensorPos) ?
						sensorLength : (lane->getLength() - sensorPos);

		//TODO check this lengths
		DBG(
		std::ostringstream phero_str;
		phero_str << " lane " << lane->getID() << " sensorPos= " << sensorPos
				<< " ,SENSOR_START  " << SENSOR_START << "; lane->getLength = "
				<< lane->getLength() << " ,lensorLength= " << lensorLength
				<< " ,SENSOR_LENGTH= " << INPUT_SENSOR_LENGTH;
		WRITE_MESSAGE(
				"MSSOTLE2Sensors::buildSensorForLane::" + phero_str.str());
		)

		//Create sensor for lane and insert it into the map<MSLane*, MSE2Collector*>
		newSensor = nb.buildSingleLaneE2Det(
				"SOTL_E2_lane:" + lane->getID() + "_tl:" + tlLogicID,
				DU_TL_CONTROL, lane,
				(lane->getLength() - sensorPos - lensorLength), lensorLength,
				HALTING_TIME_THRS, HALTING_SPEED_THRS, DIST_THRS);
		//newSensor = nb.buildSingleLaneE2Det("SOTL_E2_lane:"+lane->getID()+"_tl:"+tlLogicID, DU_TL_CONTROL, lane, (lane->getLength() - sensorPos- 5), lensorLength, HALTING_TIME_THRS, HALTING_SPEED_THRS, DIST_THRS);

		MSNet::getInstance()->getDetectorControl().add(
				SUMO_TAG_LANE_AREA_DETECTOR, newSensor);

		mySensorsMap_OutLanes.insert(MSLane_MSE2Collector(lane, newSensor));
		mySensorsIDMap_OutLanes.insert(
				MSLaneID_MSE2Collector(lane->getID(), newSensor));
		myMaxSpeedMap_OutLanes.insert(
				MSLaneID_MaxSpeed(lane->getID(), lane->getSpeedLimit()));
	}
}

unsigned int MSSOTLE2Sensors::countVehicles(MSLane* lane) {
	MSLane_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsMap_InLanes.find(
			lane);
	if (sensorsIterator == mySensorsMap_InLanes.end()) {
		assert(0);
		return 0;
	} else
		return sensorsIterator->second->getCurrentVehicleNumber();
}

/*
 * Estimate queue lenght according to the distance of the last vehicles
 */

double MSSOTLE2Sensors::getEstimateQueueLenght (std::string laneId)
{
	MSLaneID_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsIDMap_InLanes.find(laneId);
	if (sensorsIterator == mySensorsIDMap_InLanes.end())
	{
		assert(0);
		return 0;
	}
	else
	{
		double estQL = sensorsIterator->second->getEstimateQueueLength();
		if(estQL==-1)
			return 0;
		else
			DBG(
				std::ostringstream str;
				str << "MSSOTLE2Sensors::getEstimateQueueLenght lane " << sensorsIterator->second->getLane()->getID()
						<< " laneLenght " << sensorsIterator->second->getLane()->getLength() << " estimateQueueLenght " <<  estQL;
				WRITE_MESSAGE(str.str());
			)
			return estQL;
	}
}

/*
 * Estimate queue lenght according to the distance of the last vehicles that exceed a threshold
 */

unsigned int MSSOTLE2Sensors::estimateVehicles (std::string laneId)
{
	MSLaneID_MSE2CollectorMap::const_iterator sensorsIterator = mySensorsIDMap_InLanes.find(laneId);
	if (sensorsIterator == mySensorsIDMap_InLanes.end())
	{
		assert(0);
		return 0;
	}
	else
	{
		return sensorsIterator->second->getEstimatedCurrentVehicleNumber(speedThresholdParam);
	}
}

unsigned int MSSOTLE2Sensors::countVehicles(std::string laneId) {
	MSLaneID_MSE2CollectorMap::const_iterator sensorsIterator =
			mySensorsIDMap_InLanes.find(laneId);
	if (sensorsIterator == mySensorsIDMap_InLanes.end()) {
		assert(0);
		return 0;
	} else
		return sensorsIterator->second->getCurrentVehicleNumber();
}

double MSSOTLE2Sensors::getMaxSpeed(std::string laneId) {
	MSLaneID_MaxSpeedMap::const_iterator sensorsIteratorIn = myMaxSpeedMap_InLanes.find(
			laneId);

	if (sensorsIteratorIn == myMaxSpeedMap_InLanes.end()) {
		MSLaneID_MaxSpeedMap::const_iterator sensorsIteratorOut =
				myMaxSpeedMap_OutLanes.find(laneId);
		if (sensorsIteratorOut == myMaxSpeedMap_OutLanes.end()) {
			assert(0);
			WRITE_ERROR(
					"MSSOTLE2Sensors::meanVehiclesSpeed:: No lane found "
							+ laneId);
			return 0;
		} else {
			return sensorsIteratorOut->second;
		}
	} else
		return sensorsIteratorIn->second;
}

double MSSOTLE2Sensors::meanVehiclesSpeed(MSLane* lane) {
	MSLane_MSE2CollectorMap::const_iterator sensorsIteratorOut =
			mySensorsMap_OutLanes.find(lane);

	if (sensorsIteratorOut == mySensorsMap_OutLanes.end()) {

		MSLane_MSE2CollectorMap::const_iterator sensorsIteratorIn =
				mySensorsMap_InLanes.find(lane);
		if (sensorsIteratorIn == mySensorsMap_InLanes.end()) {
			assert(0);
			WRITE_ERROR(
					"MSSOTLE2Sensors::meanVehiclesSpeed:: No lane found "
							+ lane->getID());
			return 0;
		} else
			return sensorsIteratorIn->second->getCurrentMeanSpeed();
	} else
		return sensorsIteratorOut->second->getCurrentMeanSpeed();
}

double MSSOTLE2Sensors::meanVehiclesSpeed(std::string laneId) {
	MSLaneID_MSE2CollectorMap::const_iterator sensorsIteratorOut =
			mySensorsIDMap_OutLanes.find(laneId);
	if (sensorsIteratorOut == mySensorsIDMap_OutLanes.end()) {
		MSLaneID_MSE2CollectorMap::const_iterator sensorsIteratorIn =
				mySensorsIDMap_InLanes.find(laneId);
		if (sensorsIteratorIn == mySensorsIDMap_InLanes.end()) {
			assert(0);
			WRITE_ERROR(
					"MSSOTLE2Sensors::meanVehiclesSpeed:: No lane found "
							+ laneId);
			return 0;
		} else {
			return sensorsIteratorIn->second->getCurrentMeanSpeed();
		}
	} else
		return sensorsIteratorOut->second->getCurrentMeanSpeed();
}
