/****************************************************************************/
/// @file    MSSOTLTrafficLightLogic.cpp
/// @author  Gianfilippo Slager
/// @author  Anna Chiara Bellini
/// @date    Apr 2013
/// @version $Id: MSPhasedTrafficLightLogic.cpp 2 2010-03-03 15:00:00Z gslager $
///
// The base abstract class for SOTL logics
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
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
	SUMOTime delay,
	const std::map<std::string, std::string>& parameters)
: MSPhasedTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, parameters) {
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
	const std::map<std::string, std::string>& parameters,
	MSSOTLSensors *sensors) 
: MSPhasedTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, parameters) {
	this->mySensors = sensors;
	sensorsSelfBuilt = false;
	checkPhases();
	setupCTS();
	setToATargetPhase();
}

MSSOTLTrafficLightLogic::~MSSOTLTrafficLightLogic() {
	for (size_t i=0; i<myPhases.size(); i++) {
		delete myPhases[i];
	}
	if (sensorsSelfBuilt)
		delete mySensors;
}

void MSSOTLTrafficLightLogic::logStatus() {
	
}

void
MSSOTLTrafficLightLogic::checkPhases() {
	for (size_t step=0; step < getPhases().size(); step++) {
		if (getPhase(step).isUndefined()) {
			MsgHandler::getErrorInstance()->inform("Step " + toString(step) + " of traffic light logic " + myID + " phases declaration has its type undeclared!");
		}
	}
}

void
MSSOTLTrafficLightLogic::setupCTS() {
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

	MSTrafficLightLogic::init(nb);

	if(isDecayThresholdActivated())
		decayThreshold = 1;
	if (sensorsSelfBuilt) {
        //Building SOTLSensors
		switch (SENSORS_TYPE) {
            case SENSORS_TYPE_E1:
                assert(0); // Throw exception because TLS can only handle E2 sensors
            case SENSORS_TYPE_E2:

                //Adding Sensors to the ingoing Lanes

                LaneVectorVector lvv = getLaneVectors();

                DBG(
					WRITE_MESSAGE("Listing lanes for TLS "+getID());

					for (unsigned int i = 0; i<lvv.size(); i++) {
						LaneVector lv = lvv[i];

						for (unsigned int j = 0; j < lv.size(); j++) {
							MSLane* lane = lv[j];
							WRITE_MESSAGE(lane ->getID());
						}
					}
                )

                mySensors = new MSSOTLE2Sensors(myID, &(getPhases()));
                ((MSSOTLE2Sensors *)mySensors)->buildSensors(myLanes, nb,getInputSensorsLength());
                mySensors->stepChanged(getCurrentPhaseIndex());

				//threshold speed param for tuning with irace
				((MSSOTLE2Sensors *)mySensors)->setSpeedThresholdParam(getSpeedThreshold());


                //Adding Sensors to the outgoing Lanes

                LinkVectorVector myLinks = getLinks();


                DBG(
					WRITE_MESSAGE("Listing output lanes");
					for (unsigned int i = 0; i<myLinks.size(); i++){
						LinkVector oneLink = getLinksAt(i);

						for (unsigned int j = 0; j<oneLink.size(); j++){

							MSLane* lane  = oneLink[j]->getLane();
							WRITE_MESSAGE(lane ->getID());

						}
					}
                )


                LaneVectorVector myLaneVector;

                LaneVector outLanes;
                LinkVectorVector myoutLinks = getLinks();

                for (unsigned int i = 0; i<myLinks.size(); i++){
                    LinkVector oneLink = getLinksAt(i);

                    for (unsigned int j = 0; j<oneLink.size(); j++){

                        MSLane* lane  = oneLink[j]->getLane();
                        outLanes.push_back(lane);


                    }
                }

                if(outLanes.size() >0)
                    myLaneVector.push_back(outLanes);
                if(myLaneVector.size() > 0 )
                    ((MSSOTLE2Sensors *)mySensors)->buildOutSensors(myLaneVector, nb,getOutputSensorsLength());


		}
	}
}

unsigned int
MSSOTLTrafficLightLogic::getCTS(size_t chainStartingPhase) {
	map<size_t, unsigned int>::iterator phaseIterator = targetPhasesCTS.find(chainStartingPhase);
	if (phaseIterator != targetPhasesCTS.end()) {
		return phaseIterator->second;
	} else {
		return 0;
	}
}

void
MSSOTLTrafficLightLogic::resetCTS(size_t phaseStep) {
	map<size_t, unsigned int>::iterator phaseIterator = targetPhasesCTS.find(phaseStep);
	if (phaseIterator != targetPhasesCTS.end()) {
		phaseIterator->second = 0;
		lastCheckForTargetPhase[phaseStep] = STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep());
	}
}

void
MSSOTLTrafficLightLogic::updateCTS() {
	SUMOTime elapsedTimeSteps = 0;
	//Iterate over the target phase map and update CTS value for every target phase except for the one belonging to the current steps chain
	for (map<size_t, unsigned int>::iterator mapIterator = targetPhasesCTS.begin(); mapIterator != targetPhasesCTS.end(); mapIterator++) {
		unsigned int chain = mapIterator->first;
		SUMOTime now = STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep());

		if (chain != lastChain) {
			//Get the number of timesteps since the last check for that phase
			elapsedTimeSteps = now - lastCheckForTargetPhase[chain];
			//Update the last check time
			lastCheckForTargetPhase[chain] = now;
			//Increment the CTS
			//SWITCH between 3 counting vehicles function
			switch (getMode()) {
			case (0):
				mapIterator->second += elapsedTimeSteps
						* countVehicles(getPhase(chain)); //SUMO
				break;
			case (1):
				mapIterator->second += elapsedTimeSteps
						* countVehicles(getPhase(chain)); //COMPLEX
				break;
			case (2):
				mapIterator->second = countVehicles(getPhase(chain)); //QUEUE
				break;
			default:
				WRITE_ERROR("Unrecognized traffic threshold calculation mode");
			}
		}
		if(isDecayThresholdActivated())
			updateDecayThreshold();
	}
}

unsigned int 
MSSOTLTrafficLightLogic::countVehicles(MSPhaseDefinition phase) {
	
	if (!phase.isTarget()) return 0;

	unsigned int accumulator = 0;
	//Iterate over the target lanes for the current target phase to get the number of approaching vehicles
	MSPhaseDefinition::LaneIdVector targetLanes =phase.getTargetLaneSet();
	for (MSPhaseDefinition::LaneIdVector::const_iterator laneIterator = targetLanes.begin(); laneIterator!=targetLanes.end(); laneIterator++) {
		//SWITCH between 3 counting vehicles function
		switch(getMode())
					{
					case (0):
						accumulator += mySensors->countVehicles((*laneIterator)); //SUMO
					break;
					case (1):
						accumulator += ((MSSOTLE2Sensors *)mySensors)->estimateVehicles((*laneIterator)); //COMPLEX
					break;
					case (2):
						accumulator = max((int)((MSSOTLE2Sensors *)mySensors)->getEstimateQueueLenght((*laneIterator)),(int)accumulator); //QUEUE
					break;
					default:
						WRITE_ERROR("Unrecognized traffic threshold calculation mode");
					}
	}
	return accumulator;
}

void
MSSOTLTrafficLightLogic::updateDecayThreshold()
{
	if(getCurrentPhaseDef().isGreenPhase())
		decayThreshold = decayThreshold * exp(getDecayConstant());
	DBG(
		std::stringstream out;
		out << decayThreshold;
		WRITE_MESSAGE("\n" +time2string(MSNet::getInstance()->getCurrentTimeStep()) +"\tMSSOTLTrafficLightLogic::updateDecayThreshold()::  " + out.str());
	)
}
bool 
MSSOTLTrafficLightLogic::isThresholdPassed() {

	DBG(
			WRITE_MESSAGE("\n" +time2string(MSNet::getInstance()->getCurrentTimeStep()) +"\tMSSOTLTrafficLightLogic::isThresholdPassed()::  " + " tlsid=" + getID());
			std::ostringstream threshold_str;
			threshold_str << "tlsid=" << getID() << " targetPhaseCTS size=" << targetPhasesCTS.size();
			WRITE_MESSAGE(threshold_str.str());
		)
			/*
			 * if a dynamic threshold based on the exponential decrease, is passed we force the phase change
			 */
	double random = ((double) RandHelper::rand(RAND_MAX) / (RAND_MAX));
	DBG(
			if(isDecayThresholdActivated()){
				std::ostringstream str;
				str << "\n" << time2string(MSNet::getInstance()->getCurrentTimeStep()) << "\tMSSOTLTrafficLightLogic::isThresholdPassed()::  "
						<< " tlsid=" << getID() << " decayThreshold=" << decayThreshold << " random=" <<random << ">"<<(1-decayThreshold);
				WRITE_MESSAGE(str.str());
			}
	)
	if (!isDecayThresholdActivated() || (isDecayThresholdActivated() && random > (1 - decayThreshold))) {
		for (map<size_t, unsigned int>::const_iterator iterator =
				targetPhasesCTS.begin(); iterator != targetPhasesCTS.end();
				iterator++) {
			DBG(
					std::ostringstream threshold_str; threshold_str << "(getThreshold()= " << getThreshold() << ", targetPhaseCTS= " << iterator->second << " )" << " phase="<<getPhase(iterator->first).getState(); SUMOTime step = MSNet::getInstance()->getCurrentTimeStep(); WRITE_MESSAGE("\tTL " +getID()+" time " +time2string(step)+threshold_str.str());)
			//Note that the current chain is not eligible to be directly targeted again, it would be unfair
			if ((iterator->first != lastChain)
					&& (getThreshold() <= iterator->second)) {
				return true;
			}
		}
		return false;
	} else {
		return true;
	}
}


SUMOTime 
MSSOTLTrafficLightLogic::getCurrentPhaseElapsed() {
	MSPhaseDefinition currentPhase = getCurrentPhaseDef();
	
	SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
	SUMOTime elapsed = now - currentPhase.myLastSwitch;
	
	return elapsed;
}


size_t
MSSOTLTrafficLightLogic::getPhaseIndexWithMaxCTS() {
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
MSSOTLTrafficLightLogic::decideNextPhase() {
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
MSSOTLTrafficLightLogic::trySwitch(bool) {
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
			if(isDecayThresholdActivated())
				decayThreshold = 1;
		}
		//Inform the sensors logic
		mySensors->stepChanged(getCurrentPhaseIndex());
		//Store the time the new phase started
		currentPhase.myLastSwitch = MSNet::getInstance()->getCurrentTimeStep();
		if(isDecayThresholdActivated())
			decayThreshold = 1;
	}

	return computeReturnTime();
}

