/****************************************************************************/
/// @file    MSSwarmTrafficLightLogic.cpp
/// @author  Gianfilippo Slager
/// @date    Mar 2010
/// @version $Id: MSSwarmTrafficLightLogic.h 1 2010-03-06 12:40:00Z gslager $
///
// The class for Swarm-based logics
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

#include "MSSwarmTrafficLightLogic.h"

MSSwarmTrafficLightLogic::MSSwarmTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const std::string &id, const std::string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
	//Setting the startup policy
	currentPolicy = SOTLPhase;
	//Initializing the random number generator to a time-dependent seed
	srand((unsigned int)time(NULL));
	//Initializing pheromone maps according to input lanes
	//For each lane insert a pair into maps
	MSLane *currentLane = NULL;
	for (MSTrafficLightLogic::LaneVectorVector::const_iterator laneVector = myLanes.begin(); laneVector != myLanes.end(); laneVector++) {
		for (MSTrafficLightLogic::LaneVector::const_iterator lane = laneVector->begin(); lane != laneVector->end(); lane++) {
			currentLane = (*lane);
			if (pheromoneVehNumberInputLanes.find(currentLane->getID()) == pheromoneVehNumberInputLanes.end()) {
				pheromoneVehNumberInputLanes.insert(MSLaneId_Pheromone(currentLane->getID(), 0.0));
				pheromoneVehSpeedInputLanes.insert(MSLaneId_Pheromone(currentLane->getID(), 0.0));
			}
		}
	}
	//Initializing thresholds for theta evaluations
	thresholds = std::vector<ThetaVal>::vector(NPolicies, THETA_INIT);
	lastThresholdsUpdate = MSNet::getInstance()->getCurrentTimeStep();

	MsgHandler::getMessageInstance()->inform("*** Intersection " + id + " will run using MSSwarmTrafficLightLogic ***");
}

SUMOTime MSSwarmTrafficLightLogic::decideNextPhase() {
	//Update pheromone levels
	updatePheromoneLevels();
	//Decide the current policy according to pheromone levels
	decidePolicy();
	//Update learning and forgetting thresholds
	updateThresholds();
	//Execute current policy
	return executePolicy();
}

//TODO check for soundness
void MSSwarmTrafficLightLogic::updatePheromoneLevels() {
	//Updating input lanes pheromone: all input lanes without distinction
	for (MSLaneId_PheromoneMap::iterator laneIterator = pheromoneVehNumberInputLanes.begin(); laneIterator != pheromoneVehNumberInputLanes.end(); laneIterator++) {
		laneIterator->second = BETA_NO * laneIterator->second + GAMMA_NO * (getSensors()->countVehicles(laneIterator->first));
	}
	//Updating output lanes pheromone: only input lanes currently having green light. Pheromone for non green lanes is "freezed"
	if (myPhases[myStep]->isDecisional()) {
		MSPhaseDefinition::LaneIdVector inputLanes = myPhases[myStep]->getTargetLaneSet();
		for (MSPhaseDefinition::LaneIdVector::const_iterator laneIterator = inputLanes.begin(); laneIterator != inputLanes.end(); laneIterator++) {
			pheromoneVehSpeedInputLanes[*laneIterator] = BETA_SP * pheromoneVehSpeedInputLanes[*laneIterator] + 
				GAMMA_SP * (getSensors()->getMaxSpeed(*laneIterator) - getSensors()->meanVehiclesSpeed(*laneIterator));
		}
	}
}

void MSSwarmTrafficLightLogic::updateThresholds() throw() {
	SUMOTime elapsedTime = MSNet::getInstance()->getCurrentTimeStep() - lastThresholdsUpdate;
	lastThresholdsUpdate = MSNet::getInstance()->getCurrentTimeStep();
	for (int i=0; i<NPolicies; i++) {
		if (i == currentPolicy) {
			//Learning
			thresholds[i] = thresholds[i] - LEARNING_COX * elapsedTime;
			if (thresholds[i] < THETA_MIN) thresholds[i] = THETA_MIN;
		} else {
			//Forgetting
			thresholds[i] = thresholds[i] + FORGETTING_COX * elapsedTime;
			if (thresholds[i] > THETA_MAX) thresholds[i] = THETA_MAX;
		}
	}
}

MSSwarmTrafficLightLogic::Pheromone MSSwarmTrafficLightLogic::getPheromoneForInputLanes() {
	Pheromone accumulator = 0;
	unsigned int counter = 0;
	for (MSLaneId_PheromoneMap::const_iterator iterator = pheromoneVehNumberInputLanes.begin(); iterator != pheromoneVehNumberInputLanes.end(); iterator++) {
		accumulator += iterator->second;
		counter++;
	}
	if (counter==0) return 0;
	else return accumulator/counter;
}

MSSwarmTrafficLightLogic::Pheromone MSSwarmTrafficLightLogic::getPheromoneForOutputLanes() {
	Pheromone accumulator = 0;
	unsigned int counter = 0;
	for (MSLaneId_PheromoneMap::const_iterator iterator = pheromoneVehSpeedInputLanes.begin(); iterator != pheromoneVehSpeedInputLanes.end(); iterator++) {
		accumulator += iterator->second;
		counter++;
	}
	if (counter==0) return 0;
	else return accumulator/counter;
}

void MSSwarmTrafficLightLogic::decidePolicy() {
	// Decide if it is the case to check for another plan
	if ((double)rand() <= CHANGE_PLAN_PROBABILITY * (double)RAND_MAX) {
		//Preparing a vector containing [Ttheta0 Ttheta0+Ttheta1...sum(k=0 to j - 1)(Tthetak)...sum(k=0 n-dyn-plans - 1)(Tthetak)]
		std::vector<ThetaVal> thetaIntervals(NPolicies);
		ThetaVal partialSum = 0;
		for (unsigned int nPolicy=0; nPolicy<NPolicies; nPolicy++) {
			partialSum += computeThetaVal((Policy)nPolicy);
			thetaIntervals[nPolicy] = partialSum;
		}
		//Getting a random double value in the interval [0 sum(k=0 n-dyn-plans - 1)(Tthetak))
		ThetaVal random = (double)rand() / RAND_MAX * partialSum;

		//Finding the interval the random value belongs to: the interval is the chosen policy
		for (unsigned int i = 0; i < thetaIntervals.size(); i++) {
			if (random < thetaIntervals[i]) {
				currentPolicy = (Policy)i;
				return;
			}
		}
		assert(0);
	}
}

MSSwarmTrafficLightLogic::ThetaVal MSSwarmTrafficLightLogic::computeThetaVal(Policy policy) throw() {
	Stimulus stimulus = computeStimulus(policy);
	ThetaVal thetaVal = pow(stimulus, 2) / (pow(stimulus, 2) + pow(thresholds[policy], 2));
	return thetaVal;
}

MSSwarmTrafficLightLogic::Stimulus MSSwarmTrafficLightLogic::computeStimulus(Policy policy) throw() {
	Stimulus stimulus = 0;
	switch (policy) {
		case SOTLRequest : stimulus = .63662 * exp(-pow(getPheromoneForInputLanes(), 2)/2 -pow(getPheromoneForOutputLanes(), 2)/2); 
			break;
		case SOTLPhase : stimulus = .0805782 * exp(-pow(getPheromoneForInputLanes()-5, 2)/8 -pow(getPheromoneForOutputLanes(), 2)/8);
			break;
		case SOTLPlatoon : stimulus = .127326 * exp(-pow(getPheromoneForInputLanes(), 2)/10 -pow(getPheromoneForOutputLanes(), 2)/10);
			break;
		case SOTLMarching : stimulus = .0407958 * exp(-pow(getPheromoneForInputLanes()-5, 2)/8 -pow(getPheromoneForOutputLanes()-5, 2)/8);
			break;
		default: assert(0);
	}
	return stimulus;
}

SUMOTime MSSwarmTrafficLightLogic::executePolicy() {
	//If the junction was in a commit step
	//=> go to the target step that gives green to the set with the current highest CTS
	//   and return computeReturnTime()
	if (myPhases[myStep]->isCommit()) {
		myStep = (unsigned int)getPhaseIndexWithMaxCTS();
		return computeReturnTime();
	}

	//If the junction was in a transient step
	//=> go to the next step and return computeReturnTime()
	else if (myPhases[myStep]->isTransient()) {
		myStep++;
		return computeReturnTime();
	}

	//If the junction was in a decisional step, check CTS value value for each set of synchronized lights.
	//If there is at least one set exceeding theta (threshold)
	//=> go to the next step (triggers prologue) and return computeReturnTime()
	//Else
	//=> remain into the current step and return computeReturnTime()
	else if (myPhases[myStep]->isDecisional()) {
		switch (currentPolicy) {
			case SOTLRequest : evaluateDecStepSOTLRequest(); break;
			case SOTLPhase : evaluateDecStepSOTLPhase(); break;
			case SOTLPlatoon : evaluateDecStepSOTLPlatoon(); break;
			case SOTLMarching : evaluateDecStepSOTLMarching(); break;
			default : assert(0);
		}
		return computeReturnTime();
	}
	assert(0);
}

void MSSwarmTrafficLightLogic::evaluateDecStepSOTLRequest() {
	if ((MSNet::getInstance()->getCurrentTimeStep() - myPhases[myStep]->myLastSwitch) >= MIN_DECISIONAL_PHASE_DUR) {
		if (isThresholdPassed()) {
			myStep++;
		}
	}
}

void MSSwarmTrafficLightLogic::evaluateDecStepSOTLPhase() {
	if ((MSNet::getInstance()->getCurrentTimeStep() - myPhases[myStep]->myLastSwitch) >= myPhases[myStep]->minDuration) {
		if (isThresholdPassed()) {
			myStep++;
		}
	}
}

void MSSwarmTrafficLightLogic::evaluateDecStepSOTLPlatoon() {
	if ((MSNet::getInstance()->getCurrentTimeStep() - myPhases[myStep]->myLastSwitch) >= myPhases[myStep]->minDuration) {
		if (isThresholdPassed()) {
			//If there are no other vehicles approaching green lights 
			//or the declared maximum duration has been reached
			if ((countVehiclesForTargetPhase(myStep) == 0) || ((MSNet::getInstance()->getCurrentTimeStep() - myPhases[myStep]->myLastSwitch) >= myPhases[myStep]->maxDuration)) {
				myStep++;
			}
		}
	}
}

void MSSwarmTrafficLightLogic::evaluateDecStepSOTLMarching() {
	if ((MSNet::getInstance()->getCurrentTimeStep() - myPhases[myStep]->myLastSwitch) >= myPhases[myStep]->duration) {
		myStep++;
	}
}