/****************************************************************************/
/// @file    MSSOTLRequestTrafficLightLogic.cpp
/// @author  Gianfilippo Slager
/// @author  Anna Chiara Bellini 
/// @date    Apr 2013
/// @version $Id: MSSOTLRequestTrafficLightLogic.cpp 2 2013-04-05 15:00:00Z acbellini $
///
// The class for SOTL Request logics
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

#include "MSSOTLRequestTrafficLightLogic.h"

MSSOTLRequestTrafficLightLogic::MSSOTLRequestTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const string &id, const string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
								  MsgHandler::getMessageInstance()->inform("*** Intersection " + id + " will run using MSSOTLRequestTrafficLightLogic ---");
}

MSSOTLRequestTrafficLightLogic::MSSOTLRequestTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const string &id, const string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay, MSSOTLSensors *sensors) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, sensors){
								   MsgHandler::getMessageInstance()->inform("*** Intersection " + id + " will run using MSSOTLRequestTrafficLightLogic ***");
}

bool
MSSOTLRequestTrafficLightLogic::canRelease() throw() {
	if (getCurrentPhaseElapsed() >= MIN_DECISIONAL_PHASE_DUR) {
		return isThresholdPassed();
	}
	return false;
}
