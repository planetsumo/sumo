/****************************************************************************/
/// @file    MSSimpleTrafficLightLogic.cpp
/// @author  Daniel Krajzewicz
/// @author  Julia Ringel
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Friedemann Wesner
/// @date    Sept 2002
/// @version $Id$
///
// A fixed traffic light logic
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cassert>
#include <utility>
#include <vector>
#include <bitset>
#include <sstream>
#include <microsim/MSEventControl.h>
//#include "MSTrafficLightLogic.h"
#include "MSSimpleTrafficLightLogic.h"
#include "MSPhasedTrafficLightLogic.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
MSSimpleTrafficLightLogic::MSSimpleTrafficLightLogic(MSTLLogicControl& tlcontrol,
        const std::string& id, const std::string& subid, const Phases& phases,
        unsigned int step, SUMOTime delay,
        const std::map<std::string, std::string>& parameters) 
            : MSPhasedTrafficLightLogic(tlcontrol, id, subid, phases, step, delay, parameters) {}



MSSimpleTrafficLightLogic::~MSSimpleTrafficLightLogic() {
}


// ------------ Switching and setting current rows
SUMOTime
MSSimpleTrafficLightLogic::trySwitch(bool) {
    // check whether the current duration shall be increased
    if (myCurrentDurationIncrement > 0) {
        SUMOTime delay = myCurrentDurationIncrement;
        myCurrentDurationIncrement = 0;
        return delay;
    }
	proceedToNextStep();
    // check whether the next duration was overridden
    if (myOverridingTimes.size() > 0) {
        SUMOTime nextDuration = myOverridingTimes[0];
        myOverridingTimes.erase(myOverridingTimes.begin());
        return nextDuration;
    }
    // return offset to the next switch
    return getCurrentPhaseDef().duration;
}


/****************************************************************************/

