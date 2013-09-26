/****************************************************************************/
/// @file    MSSOTLWaveTrafficLightLogic.cpp
/// @author  Riccardo Belletti
/// @author  Anna Chiara Bellini
/// @date    Sep 2013
/// @version $Id: MSSOTLWaveTrafficLightLogic.h 0 2013-09-25 10:08:00Z rickybo89 $
///
// The class for SOTL Platoon logics
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

#include "MSSOTLWaveTrafficLightLogic.h"


MSSOTLWaveTrafficLightLogic::MSSOTLWaveTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const std::string &id, const std::string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay) {
								  MsgHandler::getMessageInstance()->inform("*** Intersection " + id + " will run using MSSOTLWaveTrafficLightLogic ***");
								  lastDuration = (getCurrentPhaseDef().duration);
}

MSSOTLWaveTrafficLightLogic::MSSOTLWaveTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const std::string &id, const std::string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay, MSSOTLSensors *sensors) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, sensors){
								  lastDuration = (getCurrentPhaseDef().duration);
}

bool 
MSSOTLWaveTrafficLightLogic::canRelease() throw() {

	//1s for testing purpose, it must be changed to some sort of % of lastDuration
	SUMOTime delta =1000;

	if(getCurrentPhaseElapsed() >= getCurrentPhaseDef().minDuration){
		if(getCurrentPhaseElapsed() >= lastDuration-delta){
			if(
				(countVehicles(getCurrentPhaseDef())==0)							//no other vehicles approaching green lights
				||(getCurrentPhaseElapsed()>= lastDuration+delta)					//maximum value of the window surrounding lastDuration
				||(getCurrentPhaseElapsed()>= getCurrentPhaseDef().maxDuration)		//declared maximum duration has been reached
				) {

				lastDuration=getCurrentPhaseElapsed();
				return true;
			}
		}
	}
	return false;
}