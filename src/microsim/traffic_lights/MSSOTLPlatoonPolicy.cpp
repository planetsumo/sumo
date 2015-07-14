/****************************************************************************/
/// @file    MSSOTLPlatoonPolicy.cpp
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSSOTLPlatoonPolicy.cpp 0 2010-02-18 12:40:00Z gslager $
///
// The class for SOTL Platoon logics
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

#include "MSSOTLPlatoonPolicy.h"
#include <math.h>
#include "utils/common/RandHelper.h"

MSSOTLPlatoonPolicy::MSSOTLPlatoonPolicy(const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Platoon", parameters)
{
	init();
}

MSSOTLPlatoonPolicy::MSSOTLPlatoonPolicy(MSSOTLPolicyDesirability *desirabilityAlgorithm) :
		MSSOTLPolicy("Platoon", desirabilityAlgorithm)
{
	getDesirabilityAlgorithm()->setKeyPrefix("PLATOON");
	init();
}

MSSOTLPlatoonPolicy::MSSOTLPlatoonPolicy(MSSOTLPolicyDesirability *desirabilityAlgorithm,
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicy("Platoon", desirabilityAlgorithm, parameters)
{
	getDesirabilityAlgorithm()->setKeyPrefix("PLATOON");
	init();
}

bool MSSOTLPlatoonPolicy::canRelease(int elapsed, bool thresholdPassed, bool pushButtonPressed,
		const MSPhaseDefinition* stage, int vehicleCount)
{
	DBG(
		std::ostringstream str;
		str << "invoked MSTLPlatoonPolicy::canRelease()";
		WRITE_MESSAGE(str.str());
	);
	DBG(
		std::ostringstream str;
		str << "MSSOTLPlatoonPolicy::canRelease elapsed " << elapsed << " threshold " << thresholdPassed << " pushbutton "
		        << pushButtonPressed << " vcount " << vehicleCount <<" minD "
				<< stage->minDuration << " maxD " << stage->maxDuration; str << " will return "
				<< ((thresholdPassed && ((vehicleCount == 0) || (elapsed >= stage->maxDuration)))?"true":"false");
		WRITE_MESSAGE(str.str());
	);
	if (elapsed >= stage->minDuration)
	{
		//pushbutton logic
		if (pushButtonPressed && elapsed >= stage->duration)
		{
			//If the stage duration has been passed
			DBG(
				std::ostringstream oss;
				oss << "MSSOTLPlatoonPolicy::canRelease pushButtonPressed cars " << vehicleCount << " elapsed " << elapsed << " stage duration " << stage->duration;
				WRITE_MESSAGE(oss.str());
			);
			return true;
		}
		if (thresholdPassed)
		{
			//If there are no other vehicles approaching green lights 
			//or the declared maximum duration has been reached
			return ((vehicleCount == 0) || (elapsed >= stage->maxDuration));
		} else
		{
			//use the sigmoid logic
			if (m_useSigmoid && vehicleCount == 0)
			{
				double sigmoidValue = 1.0 / (1.0 + exp(-m_k * (elapsed / 1000 - stage->duration / 1000)));
				double rnd = RandHelper::rand();
				DBG(
					std::ostringstream oss;
					oss << "MSSOTLPlatoonPolicy::canRelease sigmoid [k=" << m_k << " elapsed " << elapsed << " stage->duration " << stage->duration << " ] value "
							<< sigmoidValue; oss << " rnd " << rnd << " retval " << (rnd < sigmoidValue? "true" : "false");
					WRITE_MESSAGE(oss.str())
				);
				return rnd < sigmoidValue;
			}
		}
	}
return false;
}

void MSSOTLPlatoonPolicy::init()
{
	m_useSigmoid = getParameter("PLATOON_USE_SIGMOID", "0") != "0";
	m_k = s2f(getParameter("PLATOON_SIGMOID_K_VALUE", "1"));
	DBG(
    WRITE_MESSAGE("MSSOTLPlatoonPolicy::init use " + getParameter("PLATOON_USE_SIGMOID", "0") + " k " + getParameter("PLATOON_SIGMOID_K_VALUE", "1"));
//    for (int elapsed = 10; elapsed < 51; ++elapsed)
//    {
//        double sigmoidValue = 1.0 / (1.0 + exp(-m_k * (elapsed - 31)));
//        std::ostringstream oss;
//        oss << "elapsed " << elapsed << " value " << sigmoidValue;
//        WRITE_MESSAGE(oss.str())
//    }
	)
}
