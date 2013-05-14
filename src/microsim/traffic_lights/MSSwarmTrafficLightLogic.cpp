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
							  const Phases &phases, unsigned int step, SUMOTime delay) 
: MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
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
	thresholds = std::vector<double>(NPolicies, THETA_INIT);
	lastThresholdsUpdate = MSNet::getInstance()->getCurrentTimeStep();

	MsgHandler::getMessageInstance()->inform("*** Intersection " + id + " will run using MSSwarmTrafficLightLogic ***");
}

 unsigned int MSSwarmTrafficLightLogic::decideNextPhase() {
	//Update pheromone levels
	updatePheromoneLevels();
	//Decide the current policy according to pheromone levels
	decidePolicy();
	//Update learning and forgetting thresholds
	updateThresholds();
	//Execute current policy
	return MSSOTLTrafficLightLogic::decideNextPhase();
	
}

//TODO check for soundness
void MSSwarmTrafficLightLogic::updatePheromoneLevels() {
	//Updating input lanes pheromone: all input lanes without distinction
	for (MSLaneId_PheromoneMap::iterator laneIterator = pheromoneVehNumberInputLanes.begin(); laneIterator != pheromoneVehNumberInputLanes.end(); laneIterator++) {
		laneIterator->second = BETA_NO * laneIterator->second + GAMMA_NO * (getSensors()->countVehicles(laneIterator->first));
	}
	//Updating output lanes pheromone: only input lanes currently having green light. Pheromone for non green lanes is "freezed"
	if (getCurrentPhaseDef(). isDecisional()) {
		MSPhaseDefinition::LaneIdVector inputLanes = getCurrentPhaseDef().getTargetLaneSet();
		for (MSPhaseDefinition::LaneIdVector::const_iterator laneIterator = inputLanes.begin(); laneIterator != inputLanes.end(); laneIterator++) {
			pheromoneVehSpeedInputLanes[*laneIterator] = BETA_SP * pheromoneVehSpeedInputLanes[*laneIterator] + 
				GAMMA_SP * (getSensors()->getMaxSpeed(*laneIterator) - getSensors()->meanVehiclesSpeed(*laneIterator));
		}
	}
}

void MSSwarmTrafficLightLogic::updateThresholds() {
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


string getPolicyName(MSSwarmTrafficLightLogic::Policy p) {
	char* names[] = {"request", "phase", "platoon", "marching"};
	return names[p];	
}


void MSSwarmTrafficLightLogic::decidePolicy() {
	Policy current = currentPolicy;
	// Decide if it is the case to check for another plan
	if ((double)rand() <= CHANGE_PLAN_PROBABILITY * (double)RAND_MAX) {
		//Preparing a vector containing [Ttheta0 Ttheta0+Ttheta1...sum(k=0 to j - 1)(Tthetak)...sum(k=0 n-dyn-plans - 1)(Tthetak)]
		std::vector<double> thetaIntervals(NPolicies);
		double partialSum = 0;
		for (unsigned int nPolicy=0; nPolicy<NPolicies; nPolicy++) {
			partialSum += computeThetaVal((Policy)nPolicy);
			thetaIntervals[nPolicy] = partialSum;
		}
		//Getting a random double value in the interval [0 sum(k=0 n-dyn-plans - 1)(Tthetak))
		double random = (double)rand() / RAND_MAX * partialSum;

		//Finding the interval the random value belongs to: the interval is the chosen policy
		for (unsigned int i = 0; i < thetaIntervals.size(); i++) {
			if (random < thetaIntervals[i]) {
				currentPolicy = (Policy)i;
				break;
			}
		}
		if (current != currentPolicy) {
			MsgHandler::getMessageInstance()->inform("TL " +getID()+" switched policy. Was: " + getPolicyName(current) + " now: "+getPolicyName(currentPolicy));
		}
	}

}
double
	MSSwarmTrafficLightLogic::computeThetaVal(Policy policy) {
	double stimulus = computeStimulus(policy);
	double thetaVal = pow(stimulus, 2) / (pow(stimulus, 2) + pow(thresholds[policy], 2));
	return thetaVal;
}

double
	MSSwarmTrafficLightLogic::computeStimulus(Policy policy) {
		
	double cox = .1;
	int offsetIn = 0;
	int offsetOut = 0;
	int divisor = 1;
	switch (policy) {

		case SOTLRequest : 
			cox = REQUEST_STIM_COX;
			offsetIn = REQUEST_STIM_OFFSET_IN;
			offsetOut = REQUEST_STIM_OFFSET_OUT;
			divisor = REQUEST_STIM_DIVISOR;
		break;
		case SOTLPhase : 
			cox = PHASE_STIM_COX;
			offsetIn = PHASE_STIM_OFFSET_IN;
			offsetOut = PHASE_STIM_OFFSET_OUT;
			divisor = PHASE_STIM_DIVISOR;
			break;
		case SOTLPlatoon : 
			cox = PLATOON_STIM_COX;
			offsetIn = PLATOON_STIM_OFFSET_IN;
			offsetOut = PLATOON_STIM_OFFSET_OUT;
			divisor = PLATOON_STIM_DIVISOR;
			break;
		case SOTLMarching : 
			cox = MARCHING_STIM_COX;
			offsetIn = MARCHING_STIM_OFFSET_IN;
			offsetOut = MARCHING_STIM_OFFSET_OUT;
			divisor = MARCHING_STIM_DIVISOR;
			break;
		default: assert(0);
	}
	double stimulus = cox * exp(
		-pow(getPheromoneForInputLanes() - offsetIn, 2)/divisor 
		-pow(getPheromoneForOutputLanes() - offsetOut, 2)/divisor
		); 

	return stimulus;
}

bool MSSwarmTrafficLightLogic::canRelease() {
	bool proceed;
	switch (currentPolicy) {
		case SOTLRequest :  proceed = evaluateDecStepSOTLRequest(); break;
		case SOTLPhase : proceed = evaluateDecStepSOTLPhase(); break;
		case SOTLPlatoon : proceed = evaluateDecStepSOTLPlatoon(); break;
		case SOTLMarching : proceed = evaluateDecStepSOTLMarching(); break;
		default : assert(0);
	}
	return proceed;
}

bool MSSwarmTrafficLightLogic::evaluateDecStepSOTLRequest() {
	if (getCurrentPhaseElapsed() >= MIN_DECISIONAL_PHASE_DUR) {
		return isThresholdPassed();
	}
	return false;
}

bool MSSwarmTrafficLightLogic::evaluateDecStepSOTLPhase() {
	if (getCurrentPhaseElapsed() >= getCurrentPhaseDef().minDuration) {
		return isThresholdPassed();
	}
	return false;
}

bool MSSwarmTrafficLightLogic::evaluateDecStepSOTLPlatoon() {
	if (getCurrentPhaseElapsed() >= getCurrentPhaseDef().minDuration) {
		if (isThresholdPassed()) {
			//If there are no other vehicles approaching green lights 
			//or the declared maximum duration has been reached
			return ((countVehicles(getCurrentPhaseDef()) == 0) || (getCurrentPhaseElapsed() >= getCurrentPhaseDef().maxDuration));
		}
	}
	return false;
}

bool MSSwarmTrafficLightLogic::evaluateDecStepSOTLMarching() {
	return (getCurrentPhaseElapsed() >= getCurrentPhaseDef().duration);
}
