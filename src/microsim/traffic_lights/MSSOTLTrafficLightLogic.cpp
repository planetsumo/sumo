/****************************************************************************/
/// @file    MSSOTLTrafficLightLogic.cpp
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSPhasedTrafficLightLogic.cpp 2 2010-03-03 15:00:00Z gslager $
///
// The base abstract class for SOTL logics
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

#include "MSSOTLTrafficLightLogic.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// member method definitions
// ===========================================================================
MSSOTLTrafficLightLogic::MSSOTLTrafficLightLogic(MSTLLogicControl &tlcontrol, const std::string &id, const std::string &subid, const Phases &phases, unsigned int step, SUMOTime delay) throw()
: MSPhasedTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
	this->mySensors = NULL;
	sensorsSelfBuilt = true;
	checkPhases();
	setupCTS();
	setToATargetPhase();
}

MSSOTLTrafficLightLogic::MSSOTLTrafficLightLogic(MSTLLogicControl &tlcontrol, const std::string &id, const std::string &subid, const Phases &phases, unsigned int step, SUMOTime delay, MSSOTLSensors *sensors) throw() 
: MSPhasedTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
	this->mySensors = sensors;
	sensorsSelfBuilt = false;
	checkPhases();
	setupCTS();
	setToATargetPhase();
}

MSSOTLTrafficLightLogic::~MSSOTLTrafficLightLogic() throw() {
	for (size_t i=0; i<myPhases.size(); i++) {
		delete myPhases[i];
	}
	if (sensorsSelfBuilt)
		delete mySensors;
}

void
MSSOTLTrafficLightLogic::checkPhases() throw() {
	for (size_t step=0; step<myPhases.size(); step++) {
		if (myPhases[step]->isUndefined()) {
			MsgHandler::getErrorInstance()->inform("Step " + toString(step) + " of traffic light logic " + myID + " phases declaration has its type undeclared!");
		}
	}
}

void
MSSOTLTrafficLightLogic::setupCTS() throw() {
	for (unsigned int phaseStep = 0; phaseStep<myPhases.size(); phaseStep++) {
		if (myPhases[phaseStep]->isTarget()) {
			targetPhasesCTS[phaseStep] = 0;
			lastCheckForTargetPhase[phaseStep] = MSNet::getInstance()->getCurrentTimeStep();
		}
	}
}

void
MSSOTLTrafficLightLogic::setToATargetPhase() {
	for (size_t step=0; step<myPhases.size(); step++) {
		if (myPhases[step]->isTarget()) {
			myStep = step;
			lastTargetPhase = step;
			return;
		}
	}
	MsgHandler::getErrorInstance()->inform("No phase of type target found for traffic light logic " + myID + " The logic could malfunction. Check phases declaration.");
}

void 
MSSOTLTrafficLightLogic::init(NLDetectorBuilder &nb) throw(ProcessError) {
	//In case sensors has been externally assigned, there is no need to build a new set of sensors
	if (sensorsSelfBuilt) {
	//Building SOTLSensors
		if (SENSORS_TYPE == "e1") {
			assert(0);
		}
		else if (SENSORS_TYPE == "e2") {
			mySensors = new MSSOTLE2Sensors(myID, &myPhases);
			mySensors->buildSensors(myLanes, nb);
			mySensors->stepChanged(myStep);
		}
	}
}

MSSOTLTrafficLightLogic::CTS
MSSOTLTrafficLightLogic::getCTSForTargetPhase(size_t phaseStep) throw() {
	TargetPhase_CarsTimestepsMap::iterator phaseIterator = targetPhasesCTS.find(phaseStep);
	if (phaseIterator != targetPhasesCTS.end())
		return phaseIterator->second;
	else
		return 0;
}

void
MSSOTLTrafficLightLogic::resetCTSForTargetPhase(size_t phaseStep) throw() {
	TargetPhase_CarsTimestepsMap::iterator phaseIterator = targetPhasesCTS.find(phaseStep);
	if (phaseIterator != targetPhasesCTS.end())
		phaseIterator->second = 0;
}

void
MSSOTLTrafficLightLogic::updateCTS() throw() {
	SUMOTime elapsedTimeSteps = 0;
	//Iterate over the target phase map and update CTS value for every target phase except for the one belonging to the current steps chain
	for (TargetPhase_CarsTimestepsMap::iterator mapIterator = targetPhasesCTS.begin(); mapIterator != targetPhasesCTS.end(); mapIterator++) {
		if (mapIterator->first != lastTargetPhase) {
			//Get the number of timesteps since the last check for that phase
			elapsedTimeSteps = MSNet::getInstance()->getCurrentTimeStep() - lastCheckForTargetPhase[mapIterator->first];
			//Update the last check time
			lastCheckForTargetPhase[mapIterator->first] = MSNet::getInstance()->getCurrentTimeStep();
			//Increment the CTS
			mapIterator->second += elapsedTimeSteps * countVehiclesForTargetPhase(mapIterator->first);
		}
	}
}

unsigned int 
MSSOTLTrafficLightLogic::countVehiclesForTargetPhase(size_t phaseNumber) throw() {
	if (!myPhases[phaseNumber]->isTarget()) return 0;

	unsigned int accumulator = 0;
	//Iterate over the target lanes for the current target phase to get the number of approaching vehicles
	MSPhaseDefinition::LaneIdVector targetLanes = myPhases[phaseNumber]->getTargetLaneSet();
	for (MSPhaseDefinition::LaneIdVector::const_iterator laneIterator = targetLanes.begin(); laneIterator!=targetLanes.end(); laneIterator++) {
		accumulator += mySensors->countVehicles((*laneIterator));
	}
	return accumulator;
}

bool 
MSSOTLTrafficLightLogic::isThresholdPassed() throw() {
	for (TargetPhase_CarsTimestepsMap::const_iterator iterator = targetPhasesCTS.begin(); iterator!= targetPhasesCTS.end(); iterator++) {
		//Note that the current chain is not eligible to be directly targeted again, it would be unfair
		if ((iterator->first != lastTargetPhase) && (THRESHOLD <= iterator->second)) {
			return true;
		}
	}
	return false;
}

size_t
MSSOTLTrafficLightLogic::getPhaseIndexWithMaxCTS() throw() {
	size_t bestIndex;
	CTS maxCTS = 0;
	for (TargetPhase_CarsTimestepsMap::const_iterator iterator = targetPhasesCTS.begin(); iterator!= targetPhasesCTS.end(); iterator++) {
		if ((iterator->first != lastTargetPhase) && (maxCTS <= iterator->second)) {
			bestIndex = iterator->first;
			maxCTS = iterator->second;
		}
	}
	return bestIndex;
}

SUMOTime 
MSSOTLTrafficLightLogic::trySwitch(bool) throw() {
	//To check if decideNextPhase changes the step
	unsigned int previousStep = myStep;
	//Update CTS according to sensors
	updateCTS();
	//Invoking the function member, specialized for each SOTL logic
	SUMOTime nextTimeStep = decideNextPhase();

	//At the end, check if new step started
	if (myStep != previousStep) {
		//Check if a new steps chain started
		if (myPhases[myStep]->isTarget())  {
			//Reset CTS for the ending steps chain
			resetCTSForTargetPhase(lastTargetPhase);
			//Update lastTargetPhase
			lastTargetPhase = myStep;
		}
		//Inform the sensors logic
		mySensors->stepChanged(myStep);
		//Store the time the new phase started
		myPhases[myStep]->myLastSwitch = MSNet::getInstance()->getCurrentTimeStep();
	}

	return nextTimeStep;
}