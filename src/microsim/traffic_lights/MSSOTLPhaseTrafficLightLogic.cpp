/****************************************************************************/
/// @file    MSSOTLPhaseTrafficLightLogic.cpp
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSSOTLPhaseTrafficLightLogic.cpp 0 2010-02-18 12:40:00Z gslager $
///
// The class for SOTL Phase logics
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

#include "MSSOTLPhaseTrafficLightLogic.h"

MSSOTLPhaseTrafficLightLogic::MSSOTLPhaseTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const std::string &id, const std::string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
								  MsgHandler::getMessageInstance()->inform("*** Intersection " + id + " will run using MSSOTLPhaseTrafficLightLogic ***");
}

MSSOTLPhaseTrafficLightLogic::MSSOTLPhaseTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const std::string &id, const std::string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay, MSSOTLSensors *sensors) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, sensors){
}

SUMOTime
MSSOTLPhaseTrafficLightLogic::decideNextPhase() throw() {
	//If the junction was in a commit step
	//=> go to the target step that gives green to the set with the current highest CTS
	//   and return computeReturnTime()
	if (myPhases[myStep]->isCommit()) {
		myStep = getPhaseIndexWithMaxCTS();
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
	//	and the step executed for at least its declared minimum duration
	//=> go to the next step (triggers prologue) and return computeReturnTime()
	//Else
	//=> remain into the current step and return computeReturnTime()
	else if (myPhases[myStep]->isDecisional()) {
		if ((MSNet::getInstance()->getCurrentTimeStep() - myPhases[myStep]->myLastSwitch) >= myPhases[myStep]->minDuration) {
			if (isThresholdPassed()) {
				myStep++;
				return computeReturnTime();
			}
		}
		return computeReturnTime();
	}
	assert(0);
}
