/****************************************************************************/
/// @file    MSSOTLPhasePolicy.cpp
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSSOTLPhasePolicy.cpp 0 2010-02-18 12:40:00Z gslager $
///
// The class for SOTL Phase logics
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright 2001-2009 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#include "MSSOTLPhasePolicy.h"

MSSOTLPhasePolicy::MSSOTLPhasePolicy(
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Phase", parameters) {
}

MSSOTLPhasePolicy::MSSOTLPhasePolicy(
		MSSOTLPolicyDesirability *desirabilityAlgorithm) :
		MSSOTLPolicy("Phase", desirabilityAlgorithm) {
	getDesirabilityAlgorithm()->setKeyPrefix("PHASE");
}

MSSOTLPhasePolicy::MSSOTLPhasePolicy(
		MSSOTLPolicyDesirability *desirabilityAlgorithm,
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Phase", desirabilityAlgorithm, parameters) {
	getDesirabilityAlgorithm()->setKeyPrefix("PHASE");
}

bool MSSOTLPhasePolicy::canRelease(int elapsed, bool thresholdPassed,
		const MSPhaseDefinition* stage, int vehicleCount) {
	if (elapsed >= stage->minDuration) {
		return thresholdPassed;
	}
	return false;
}
