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
								  (*myPhases[getCurrentPhaseIndex()]).lastDuration = (getCurrentPhaseDef().duration);
}

MSSOTLWaveTrafficLightLogic::MSSOTLWaveTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const std::string &id, const std::string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay, MSSOTLSensors *sensors) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, sensors){
								  (*myPhases[getCurrentPhaseIndex()]).lastDuration = (getCurrentPhaseDef().duration);
}

bool 
MSSOTLWaveTrafficLightLogic::canRelease() throw() {

	//% of lastDuration
	SUMOTime delta =5*(*myPhases[getCurrentPhaseIndex()]).lastDuration/100;
	
	//this allows a minimum variation of +-10ms
	if(delta<10)
		delta=10;

	if(getCurrentPhaseElapsed() >= getCurrentPhaseDef().minDuration){
		if(getCurrentPhaseElapsed() >= (*myPhases[getCurrentPhaseIndex()]).lastDuration-delta){
			if(
				(countVehicles(getCurrentPhaseDef())==0)							//no other vehicles approaching green lights
				||(getCurrentPhaseElapsed()>= (*myPhases[getCurrentPhaseIndex()]).lastDuration+delta)					//maximum value of the window surrounding lastDuration
				||(getCurrentPhaseElapsed()>= getCurrentPhaseDef().maxDuration)		//declared maximum duration has been reached
				) {

				(*myPhases[getCurrentPhaseIndex()]).lastDuration=getCurrentPhaseElapsed();
				return true;
			}
		}
	}
	return false;
}