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
								  //sets the lastDuration of every phase to the same value as the default duration of that phase
								  for (int i = 0; i < getPhaseNumber(); i++)
								  {
									  (*myPhases[i]).lastDuration = (*myPhases[i]).duration;
								  }
								  //only for evaluation purpose
								  MsgHandler::getMessageInstance()->inform("*** The following data serves only for evaluation purpose ***\nid,state,duration,simulation time");
}

MSSOTLWaveTrafficLightLogic::MSSOTLWaveTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const std::string &id, const std::string &subid,
							  const Phases &phases, unsigned int step, SUMOTime delay, MSSOTLSensors *sensors) throw() : MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, sensors){
								  //sets the lastDuration of every phase to the same value as the default duration of that phase
								  for (int i = 0; i < getPhaseNumber(); i++)
								  {
									  (*myPhases[i]).lastDuration = (*myPhases[i]).duration;
								  }
}

bool 
MSSOTLWaveTrafficLightLogic::canRelease() throw() {

	//check if isTarget -> salvare targetlane OPPURE cerco le linee col verde

	//10% of lastDuration
	SUMOTime delta =10*getCurrentPhaseDef().lastDuration/100;
	
	//this allows a minimum variation of +-10ms
	if(delta<10)
		delta=10;
	//here for debugging purpose
	myID;
	if(getCurrentPhaseElapsed() >= getCurrentPhaseDef().minDuration){
		if(getCurrentPhaseElapsed() >= getCurrentPhaseDef().lastDuration-delta){
	//		cout << getID()<<"\n";
	//		countVehicles();
	//		cout << "vehicles on "<< getID()<<":"<< countVehicles()<<"\n";
	//		if(getCurrentPhaseDef().isTarget()){
	//				cout << getID()<<" "<<MSSOTLTrafficLightLogic::countVehicles(getCurrentPhaseDef())<<" "<<countVehicles()<<"\n";
	//			}
			if(
				(countVehicles()==0)							//no other vehicles approaching green lights
				||(getCurrentPhaseElapsed()>= getCurrentPhaseDef().lastDuration+delta)					//maximum value of the window surrounding lastDuration
				||(getCurrentPhaseElapsed()>= getCurrentPhaseDef().maxDuration)		//declared maximum duration has been reached
				) {

				(*myPhases[getCurrentPhaseIndex()]).lastDuration=getCurrentPhaseElapsed();
				//only for evaluation purpose
				MsgHandler::getMessageInstance()->inform(getID()+","+getCurrentPhaseDef().getState()+","+time2string(getCurrentPhaseElapsed())+","+time2string(MSNet::getInstance()->getCurrentTimeStep()));
				return true;
			}
		}
	}
	return false;
}

unsigned int
	MSSOTLWaveTrafficLightLogic::countVehicles(){

		string state = getCurrentPhaseDef().getState();
		int vehicles = 0;
		for(int i=0; i<getLanes().size();i++){
			if(i>0 && ((getLanes()[i][0]->getID()).compare(getLanes()[i-1][0]->getID())==0)){
				continue;
			}
			if(state[i]!='r'){
				vehicles += getSensors()->countVehicles(getLanes()[i][0]);
			//	cout << getLanes()[i][j]->getID()<<"\n";
			}	
		//	cout << state[i]<<"->"<<getLanes()[i][0]->getID()<<"\n";

		}
		cout <<"\n";
		return vehicles;
}