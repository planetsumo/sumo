/****************************************************************************/
/// @file    MSSOTLPolicyDesirability.cpp
/// @author  Riccardo Belletti
/// @date    Mar 2014
/// @version $Id: MSSOTLPolicyDesirability.cpp 0  $
///
// The class for Swarm-based low-level policy
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2013 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#include "MSSOTLPolicyStimulus.h"

MSSOTLPolicyStimulus::MSSOTLPolicyStimulus(
		const std::map<std::string, std::string>& parameters) :
		MSSOTLPolicyDesirability(parameters) {

	stimCoxDVal = 1;
	stimOffsetInDVal = 0;
	stimOffsetOutDVal = 0;
	stimDivInDVal = 1;
	stimDivOutDVal = 1;
	stimCoxExpInDVal = 1;
	stimCoxExpOutDVal = 1;
}

double MSSOTLPolicyStimulus::computeDesirability(double vehInMeasure,
		double vehOutMeasure) {
	DBG(
			std::ostringstream str; str << "cox=" << getStimCox() << ", cox_exp_in=" << getStimCoxExpIn() << ", cox_exp_out=" << getStimCoxExpOut() << ", off_in=" << getStimOffsetIn() << ", off_out=" << getStimOffsetOut() << ", div_in=" << getStimDivisorIn() << ", div_out=" << getStimDivisorOut(); WRITE_MESSAGE(str.str());)

	//		it seems to be not enough, a strange segmentation fault appears...
	//	 if((getStimCoxExpIn()!=0.0 && getStimDivisorIn()==0.0)||(getStimCoxExpOut()!=0.0 && getStimDivisorOut()==0.0)){
	if (getStimDivisorIn() == 0 || getStimDivisorOut() == 0) {
		std::ostringstream errorMessage;
		errorMessage << "INCORRECT VALUES" << "\nStimCoxExpIn="
				<< getStimCoxExpIn() << ", StimDivisorIn=" << getStimDivisorIn()
				<< ", StimCoxExpOut=" << getStimCoxExpOut()
				<< ", StimDivisorOut=" << getStimDivisorOut();
		WRITE_ERROR(errorMessage.str());
		assert(-1);
		return -1;
	} else {
		double stimulus = getStimCox()
				* exp(
						-getStimCoxExpIn()
								* pow(vehInMeasure - getStimOffsetIn(), 2)
								/ getStimDivisorIn()
								- getStimCoxExpOut()
										* pow(
												vehOutMeasure
														- getStimOffsetOut(), 2)
										/ getStimDivisorOut());
		return stimulus;
	}
}
