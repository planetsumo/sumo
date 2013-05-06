/****************************************************************************/
/// @file    MSSOTLTrafficLightLogic.cpp
/// @author  Gianfilippo Slager
/// @author  Anna Chiara Bellini
/// @date    Apr 2013
/// @version $Id: MSPhasedTrafficLightLogic.cpp 2 2010-03-03 15:00:00Z gslager $
///
// The base abstract class for SOTL logics
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2013 DLR (http://www.dlr.de/) and contributors
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
MSSOTLTrafficLightLogic::MSSOTLTrafficLightLogic(
	MSTLLogicControl &tlcontrol, 
	const string &id, 
	const string &subid, 
	const Phases &phases, 
	unsigned int step, 
	SUMOTime delay) throw()
: MSPhasedTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
	this->mySensors = NULL;
	sensorsSelfBuilt = true;
	checkPhases();
	setupCTS();
	setToATargetPhase();
}

MSSOTLTrafficLightLogic::MSSOTLTrafficLightLogic(
	MSTLLogicControl &tlcontrol, 
	const string &id, 
	const string &subid, 
	const Phases &phases, 
	unsigned int step, 
	SUMOTime delay, 
	MSSOTLSensors *sensors) throw() 
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

void MSSOTLTrafficLightLogic::logStatus() throw() {
	
}

void
MSSOTLTrafficLightLogic::checkPhases() throw() {
	for (size_t step=0; step < getPhases().size(); step++) {
		if (getPhase(step).isUndefined()) {
			MsgHandler::getErrorInstance()->inform("Step " + toString(step) + " of traffic light logic " + myID + " phases declaration has its type undeclared!");
		}
	}
}

void
MSSOTLTrafficLightLogic::setupCTS() throw() {
	for (unsigned int phaseStep = 0; phaseStep< getPhases().size(); phaseStep++) {


		if (getPhase(phaseStep).isTarget()) {
			targetPhasesCTS[phaseStep] = 0;
			lastCheckForTargetPhase[phaseStep] = MSNet::getInstance()->getCurrentTimeStep();
		}
	}
}

void
MSSOTLTrafficLightLogic::setToATargetPhase() {
	for (size_t step=0; step<getPhases().size(); step++) {
		if (getPhase(step).isTarget()) {
			setStep(step);
			lastChain = step;
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
		switch (SENSORS_TYPE) {
		case SENSORS_TYPE_E1:
			assert(0); // Throw exception because TLS can only handle E2 sensors
		case SENSORS_TYPE_E2:
			mySensors = new MSSOTLE2Sensors(myID, &(getPhases()));
			mySensors->buildSensors(myLanes, nb);
			mySensors->stepChanged(getCurrentPhaseIndex());
		}
	}

	/*
	MsgHandler::getMessageInstance() ->inform("Listing lanes for TLS "+getID());
	LaneVectorVector lvv = getLanes();
	
	for (unsigned int i = 0; i<lvv.size(); i++) { 
		LaneVector lv = lvv[i];
	
		for (unsigned int j = 0; j < lv.size(); j++) {
			MSLane* lane = lv[j];
	    	MsgHandler::getMessageInstance() ->inform(lane ->getID());
		}
	}
	*/

}

unsigned int
MSSOTLTrafficLightLogic::getCTS(size_t chainStartingPhase) throw() {
	map<size_t, unsigned int>::iterator phaseIterator = targetPhasesCTS.find(chainStartingPhase);
	if (phaseIterator != targetPhasesCTS.end()) {
		return phaseIterator->second;
	} else {
		return 0;
	}
}

void
MSSOTLTrafficLightLogic::resetCTS(size_t phaseStep) throw() {
	map<size_t, unsigned int>::iterator phaseIterator = targetPhasesCTS.find(phaseStep);
	if (phaseIterator != targetPhasesCTS.end()) {
		phaseIterator->second = 0;
	}
}

void
MSSOTLTrafficLightLogic::updateCTS() throw() {
	SUMOTime elapsedTimeSteps = 0;
	//Iterate over the target phase map and update CTS value for every target phase except for the one belonging to the current steps chain
	for (map<size_t, unsigned int>::iterator mapIterator = targetPhasesCTS.begin(); mapIterator != targetPhasesCTS.end(); mapIterator++) {
		unsigned int chain = mapIterator->first;
		SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();

		if (chain != lastChain) {
			//Get the number of timesteps since the last check for that phase
			elapsedTimeSteps = now - lastCheckForTargetPhase[chain];
			//Update the last check time
			lastCheckForTargetPhase[chain] = now;
			//Increment the CTS
			mapIterator->second += elapsedTimeSteps * countVehicles(getPhase(chain));
		}
	}
}

unsigned int 
MSSOTLTrafficLightLogic::countVehicles(MSPhaseDefinition phase) throw() {
	
	if (!phase.isTarget()) return 0;

	unsigned int accumulator = 0;
	//Iterate over the target lanes for the current target phase to get the number of approaching vehicles
	MSPhaseDefinition::LaneIdVector targetLanes =phase.getTargetLaneSet();
	for (MSPhaseDefinition::LaneIdVector::const_iterator laneIterator = targetLanes.begin(); laneIterator!=targetLanes.end(); laneIterator++) {
		accumulator += mySensors->countVehicles((*laneIterator));
	}
	return accumulator;
}

bool 
MSSOTLTrafficLightLogic::isThresholdPassed() throw() {
	for (map<size_t, unsigned int>::const_iterator iterator = targetPhasesCTS.begin(); iterator!= targetPhasesCTS.end(); iterator++) {
		//Note that the current chain is not eligible to be directly targeted again, it would be unfair
		if ((iterator->first != lastChain) && (THRESHOLD <= iterator->second)) {
			return true;
		}
	}
	return false;
}


SUMOTime 
MSSOTLTrafficLightLogic::getCurrentPhaseElapsed() throw () {
	MSPhaseDefinition currentPhase = getCurrentPhaseDef();
	
	SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
	SUMOTime elapsed = now - currentPhase.myLastSwitch;
	
	return elapsed;
}


size_t
MSSOTLTrafficLightLogic::getPhaseIndexWithMaxCTS() throw() {
	size_t bestIndex;
	unsigned int maxCTS = 0;
	for (map<size_t, unsigned int>::const_iterator iterator = targetPhasesCTS.begin(); iterator!= targetPhasesCTS.end(); iterator++) {
		if ((iterator->first != lastChain) && (maxCTS <= iterator->second)) {
			bestIndex = iterator->first;
			maxCTS = iterator->second;
		}
	}
	return bestIndex;
}

size_t
MSSOTLTrafficLightLogic::decideNextPhase() throw() {
	MSPhaseDefinition currentPhase = getCurrentPhaseDef();
	SUMOTime elapsed = getCurrentPhaseElapsed();
	//If the junction was in a commit step
	//=> go to the target step that gives green to the set with the current highest CTS
	//   and return computeReturnTime()
	if (currentPhase.isCommit()) {
		// decide which chain to activate. Gotta work on this
		return getPhaseIndexWithMaxCTS();
	}  
	if (currentPhase.isTransient()) {
		//If the junction was in a transient step
		//=> go to the next step and return computeReturnTime()
		return getCurrentPhaseIndex() + 1;
	} 
	
	if (currentPhase.isDecisional()) {

		if (canRelease()) {
			return getCurrentPhaseIndex() + 1;
		} 
	}
	
	return getCurrentPhaseIndex();
}

SUMOTime 
MSSOTLTrafficLightLogic::trySwitch(bool) throw() {
	// To check if decideNextPhase changes the step
	unsigned int previousStep = getCurrentPhaseIndex() ;
	// Update CTS according to sensors
	updateCTS();

	// Invoking the function member, specialized for each SOTL logic
	setStep(decideNextPhase());
	MSPhaseDefinition currentPhase = getCurrentPhaseDef();

	//At the end, check if new step started
	if (getCurrentPhaseIndex() != previousStep) {
		//Check if a new steps chain started
		if (currentPhase.isTarget())  {
			//Reset CTS for the ending steps chain
			resetCTS(lastChain);
			//Update lastTargetPhase
			lastChain = getCurrentPhaseIndex();
		}
		//Inform the sensors logic
		mySensors->stepChanged(getCurrentPhaseIndex());
		//Store the time the new phase started
		currentPhase.myLastSwitch = MSNet::getInstance()->getCurrentTimeStep();
	}

	return computeReturnTime();
}