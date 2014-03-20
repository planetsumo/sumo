/****************************************************************************/
/// @file    MSSOTLPlatoonPolicy.cpp
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSSOTLPlatoonPolicy.cpp 0 2010-02-18 12:40:00Z gslager $
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

#include "MSSOTLPlatoonPolicy.h"

MSSOTLPlatoonPolicy::MSSOTLPlatoonPolicy(
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Platoon", parameters) {
}

MSSOTLPlatoonPolicy::MSSOTLPlatoonPolicy(
		MSSOTLPolicyDesirability *desirabilityAlgorithm) :
		MSSOTLPolicy("Platoon", desirabilityAlgorithm) {
	getDesirabilityAlgorithm()->setKeyPrefix("PLATOON");
}

MSSOTLPlatoonPolicy::MSSOTLPlatoonPolicy(
		MSSOTLPolicyDesirability *desirabilityAlgorithm,
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Platoon", desirabilityAlgorithm, parameters) {
	getDesirabilityAlgorithm()->setKeyPrefix("PLATOON");
}

bool MSSOTLPlatoonPolicy::canRelease(int elapsed, bool thresholdPassed,
		const MSPhaseDefinition* stage, int vehicleCount) {
	DBG(
			std::ostringstream str; str << "\n" << "invoked MSTLPlatoonPolicy::canRelease()"; WRITE_MESSAGE(str.str());)

	if (elapsed >= stage->minDuration) {
		if (thresholdPassed) {
			//If there are no other vehicles approaching green lights 
			//or the declared maximum duration has been reached
			return ((vehicleCount == 0) || (elapsed >= stage->maxDuration));
		}
	}
	return false;
}
