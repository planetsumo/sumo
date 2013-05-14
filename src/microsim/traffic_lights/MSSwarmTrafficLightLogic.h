/****************************************************************************/
/// @file    MSSwarmTrafficLightLogic.h
/// @author  Gianfilippo Slager
/// @date    Mar 2010
/// @version $Id: MSSwarmTrafficLightLogic.h 0 2010-03-04 12:40:00Z gslager $
///
// The class for Swarm-based logics
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
#ifndef MSSwarmTrafficLightLogic_h
#define MSSwarmTrafficLightLogic_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSSOTLTrafficLightLogic.h"
class MSSwarmTrafficLightLogic :
	public MSSOTLTrafficLightLogic
{
public:

	enum Policy {
		SOTLRequest = 0,
		SOTLPhase,
		SOTLPlatoon,
		SOTLMarching
	};

	static const unsigned int NPolicies = 4;

	//****************************************************
	//Type definitions to implement the pheromone paradigm

	/*
	* This value should be constrained to the interval [0, PHERO_MAXVAL]
	*/
	typedef double Pheromone;
	

	typedef pair<string, Pheromone> MSLaneId_Pheromone;
	/*
	* This map type definition identifies a set of lanes, connected to a kind of pheromone.
	* Pheromone can be of different kinds to express different stimuli
	*/
	typedef map<string, Pheromone> MSLaneId_PheromoneMap;

	//****************************************************

	/** 
	 * @brief Constructor without sensors passed
     * @param[in] tlcontrol The tls control responsible for this tls
     * @param[in] id This tls' id
     * @param[in] subid This tls' sub-id (program id)
     * @param[in] phases Definitions of the phases
     * @param[in] step The initial phase index
     * @param[in] delay The time to wait before the first switch
     */
	MSSwarmTrafficLightLogic(MSTLLogicControl &tlcontrol,
                              const string &id, const string &subid,
                              const Phases &phases, unsigned int step, SUMOTime delay);

	Policy getCurrentPolicy() { return currentPolicy; }

protected:
	/*
	* The policy currently selected and in execution
	*/
	Policy currentPolicy;

	/*
	* This pheronome is an indicator of congestion on input lanes.
	* Its levels refer to the density of vehicles using the input lane: 
	* the more the vehicles the higher the pheromone.
	* These levels are updated on every input lane, independently on lights state.
	*/
	MSLaneId_PheromoneMap pheromoneVehNumberInputLanes;

	/*
	* This pheromone is an indicator of congestion on output lanes.
	* Its levels refer to the average speed of vehicles passing the input lane:
	* the lower the speed the higher the pheromone.
	* These levels are updated only when input lanes have their lights on green state, because it regards speed.
	*/
	MSLaneId_PheromoneMap pheromoneVehSpeedInputLanes;

	/*
	* This vector contains the thresholds for each policy,
	* s.t. theta value are computed with respect to thresholds.
	*/
	vector<double> thresholds;

	/*
	* This member keeps track of the last thresholds update, s.t.
	* updates can be correctly performed even on time-variable interations.
	* @see MSSwarmTrafficLightLogic::updateThresholds()
	*/
	SUMOTime lastThresholdsUpdate;

	/*
	 * This member has to contain the switching logic for SOTL policies
	 */
	
	unsigned int decideNextPhase();


	bool canRelease();
	
	/*
	* @return The average pheromone level regarding congestion on input lanes
	*/
	Pheromone getPheromoneForInputLanes();

	/*
	* @return The average pheromone level regarding congestion on output lanes
	*/
	Pheromone getPheromoneForOutputLanes();

	/*
	* @brief Update pheromone levels
	* Pheromone on input lanes is costantly updated 
	* Pheromone follows a discrete-time dynamic law "pheromone(k+1) = beta*pheromone(k) + gamma * sensed_val(k)"
	*/
	void updatePheromoneLevels();

	/*
	* After a policy has been chosen, for every iteration thresholds has to be updated.
	* Thresholds reinforcement lowers the threshold for the current policy and raises the ones for currently unused policies.
	* Thresholds belongs to the interval [THETA_MIN THETA_MAX]
	*/
	void updateThresholds();

	/*
	* @brief Decide the current policy according to pheromone levels
	* The decision reflects on currentPolicy value
	*/
	void decidePolicy();

	/*
	* Stimulus is conditioned by dynamic thresholds.
	* This is the base for learning capabilities of swarm-based approaches.
	* @return 
	*/
	double computeThetaVal(Policy policy);

	/*
	* Compute the stimulus functions for the given policy.
	* A stimulus function is defined over the closed domain [0 PHERO_MAXVAL] X [0 PHERO_MAXVAL] and its 
	* returned value is normalized, s.t. the definite integral over the domain is unitary.
	* @return the stimulus, normalized
	*/
	double computeStimulus(Policy policy);

	//Evalation of a decisional step following SOTLRequest policy logic
	bool evaluateDecStepSOTLRequest();
	//Evalation of a decisional step following SOTLPhase policy logic
	bool evaluateDecStepSOTLPhase();
	//Evalation of a decisional step following SOTLPlatoon policy logic
	bool evaluateDecStepSOTLPlatoon();
	//Evalation of a decisional step following SOTLMarching policy logic
	bool evaluateDecStepSOTLMarching();
};

#endif
/****************************************************************************/
