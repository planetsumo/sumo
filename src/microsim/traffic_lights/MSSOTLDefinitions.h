/****************************************************************************/
/// @file    MSSOTLDefinitions.h
/// @author  Gianfilippo Slager
/// @date    Mar 2010
/// @version $Id: MSSwarmTrafficLightLogic.h 1 2010-03-06 17:52:00Z gslager $
///
// The repository for definitions about SOTL and Swarm-based logics
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

#ifndef MSSOTLDefinitions_h
#define MSSOTLDefinitions_h

///For MSSOTLSensors
//SENSOR_START in meters, counting from the traffic light and moving backward with respect to traffic direction
#define SENSOR_START 0.0f
//SENSOR_LENGTH in meters, counting from SENSOR_START and moving backward with respect to traffic direction
#define SENSOR_LENGTH 30.0f

////For MSSOTLE2Sensors
//E2 Detector parameter: the time in seconds a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
#define HALTING_TIME_THRS 10
//E2 Detector parameter: the speed in meters/second a vehicle's speed must be below to be assigned as jammed
#define HALTING_SPEED_THRS 1
//E2 Detector parameter: the distance in meters between two vehicles in order to not count them to one jam
#define DIST_THRS 20.0

////For MSSOTLTrafficLightLogic
#define THRESHOLD 10
//#define SENSORS_TYPE "e2"
#define SENSORS_TYPE_E1 1
#define SENSORS_TYPE_E2 2
#define SENSORS_TYPE SENSORS_TYPE_E2

////For MSSOTLRequestTrafficLightLogic
#define MIN_DECISIONAL_PHASE_DUR 5000

////For MSSwarmTrafficLightLogic
#define PHERO_MAXVAL 10.0
#define BETA_NO 0.99
#define GAMMA_NO 1.0
#define BETA_SP 0.99
#define GAMMA_SP 1.0
#define CHANGE_PLAN_PROBABILITY 0.003
#define THETA_MAX 0.8
#define THETA_MIN 0.2
#define THETA_INIT 0.5
#define LEARNING_COX   0.0005
#define FORGETTING_COX 0.0005

//// For stimulus function: stimulus = cox * exp(-pow(getPheromoneForInputLanes() - offsetIn, 2)/divisor -pow(getPheromoneForOutputLanes() - offsetOut, 2)/divisor); 
#define REQUEST_STIM_COX .63662
#define REQUEST_STIM_OFFSET_IN 0
#define REQUEST_STIM_OFFSET_OUT 0
#define REQUEST_STIM_DIVISOR 2

#define PHASE_STIM_COX .0805782
#define PHASE_STIM_OFFSET_IN 5
#define PHASE_STIM_OFFSET_OUT 0
#define PHASE_STIM_DIVISOR 8

#define PLATOON_STIM_COX .127326
#define PLATOON_STIM_OFFSET_IN 0
#define PLATOON_STIM_OFFSET_OUT 0
#define PLATOON_STIM_DIVISOR 10

#define MARCHING_STIM_COX .0407958
#define MARCHING_STIM_OFFSET_IN 5
#define MARCHING_STIM_OFFSET_OUT 5
#define MARCHING_STIM_DIVISOR 8



#endif
/****************************************************************************/