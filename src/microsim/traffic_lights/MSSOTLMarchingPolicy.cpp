/****************************************************************************/
/// @file    MSSOTLMarchingPolicy.h
/// @author  Alessio Bonfietti
/// @author  Riccardo Belletti
/// @date    Feb 2014
/// @version $Id: MSSOTLMarchingPolicy.h 0 2014-02-28 10:33:00Z riccardo_belletti $
///
// The class for SOTL Marching logics
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

#include "MSSOTLMarchingPolicy.h"

MSSOTLMarchingPolicy::MSSOTLMarchingPolicy(
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Marching", parameters) {
}

MSSOTLMarchingPolicy::MSSOTLMarchingPolicy(
		MSSOTLPolicyDesirability *desirabilityAlgorithm) :
		MSSOTLPolicy("Marching", desirabilityAlgorithm) {
	getDesirabilityAlgorithm()->setKeyPrefix("MARCHING");
}

MSSOTLMarchingPolicy::MSSOTLMarchingPolicy(
		MSSOTLPolicyDesirability *desirabilityAlgorithm,
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Marching", desirabilityAlgorithm, parameters) {
	getDesirabilityAlgorithm()->setKeyPrefix("MARCHING");
}

bool MSSOTLMarchingPolicy::canRelease(int elapsed, bool thresholdPassed,
		const MSPhaseDefinition* stage, int vehicleCount) {
	return (elapsed >= stage->duration);
}
