/****************************************************************************/
/// @file    MSPModel_NonInteracting.h
/// @author  Jakob Erdmann
/// @date    Mon, 13 Jan 2014
/// @version $Id: MSPModel_NonInteracting.cpp 16023 2014-03-25 10:36:03Z namdre $
///
// The pedestrian following model (prototype)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2014-2014 DLR (http://www.dlr.de/) and contributors
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

#include <math.h>
#include <algorithm>
#include <utils/common/RandHelper.h>
#include <utils/options/OptionsCont.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSJunction.h>
#include "MSPModel_NonInteracting.h"


// ===========================================================================
// DEBUGGING HELPERS
// ===========================================================================
//
#define DEBUG1 "disabled"
#define DEBUG2 "disabled"
#define DEBUGCOND(PEDID) (PEDID == DEBUG1 || PEDID == DEBUG2)
#define LOG_ALL false

// ===========================================================================
// named (internal) constants
// ===========================================================================


// ===========================================================================
// static members
// ===========================================================================


// ===========================================================================
// MSPModel_NonInteracting method definitions
// ===========================================================================

MSPModel_NonInteracting::MSPModel_NonInteracting(const OptionsCont& oc, MSNet* net) :
    myNet(net)
{
    assert(myNet != 0);
    UNUSED_PARAMETER(oc);
}


MSPModel_NonInteracting::~MSPModel_NonInteracting() {
}


PedestrianState*
MSPModel_NonInteracting::add(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, SUMOTime now) {
    PState* state = new PState();
    const SUMOTime firstEdgeDuration = state->computeWalkingTime(0, *stage, now);
    myNet->getBeginOfTimestepEvents().addEvent(new MoveToNextEdge(person, *stage),
            now + firstEdgeDuration, MSEventControl::ADAPT_AFTER_EXECUTION);

    //std::cout << SIMTIME << " pedestrian inserted on " << stage->getEdge()->getID() << "\n";
    return state;
}


bool 
MSPModel_NonInteracting::blockedAtDist(const MSLane* lane, SUMOReal distToCrossing, std::vector<const MSPerson*>* collectBlockers) {
    return false;
}

SUMOTime 
MSPModel_NonInteracting::MoveToNextEdge::execute(SUMOTime currentTime) {
    PState* state = dynamic_cast<PState*>(myParent.getPedestrianState());
    const MSEdge* old = myParent.getEdge();
    const bool arrived = myParent.moveToNextEdge(myPerson, currentTime);
    if (arrived) {
        // walk finished. clean up state
        delete state;
        //std::cout << SIMTIME << " pedestrian arrived on " << old->getID() << "\n";
        return 0;
    } else {
        //std::cout << SIMTIME << " pedestrian moves to " << myParent.getEdge()->getID() << "\n";
        return state->computeWalkingTime(old, myParent, currentTime);
    }
}


SUMOTime 
MSPModel_NonInteracting::PState::computeWalkingTime(const MSEdge* prev, const MSPerson::MSPersonStage_Walking& stage, SUMOTime currentTime) {
    myLastEntryTime = currentTime;
    const MSEdge* edge = stage.getEdge();
    const MSEdge* next = stage.getNextRouteEdge();
    int dir = UNDEFINED_DIRECTION;
    if (prev == 0) {
        myCurrentBeginPos = stage.getDepartPos();
    } else {
        dir = (edge->getFromJunction() == prev->getToJunction() || edge->getFromJunction() == prev->getFromJunction()) ? FORWARD : BACKWARD;
        myCurrentBeginPos = dir == FORWARD ? 0 : edge->getLength();
    }
    if (next == 0) {
        myCurrentEndPos = stage.getArrivalPos();
    } else {
        if (dir == UNDEFINED_DIRECTION) {
            dir = (edge->getToJunction() == next->getFromJunction() || edge->getToJunction() == next->getToJunction()) ? FORWARD : BACKWARD;
        }
        myCurrentEndPos = dir == FORWARD ? edge->getLength() : 0;
    }
    myCurrentDuration = TIME2STEPS(fabs(myCurrentEndPos - myCurrentBeginPos) / stage.getSpeed());
    //std::cout << SIMTIME << " dir=" << dir << " curBeg=" << myCurrentBeginPos << " curEnd=" << myCurrentEndPos << " dur=" << myCurrentDuration << "\n";
    return myCurrentDuration;
}


SUMOReal 
MSPModel_NonInteracting::PState::getEdgePos(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    //std::cout << SIMTIME << " pos=" << (myCurrentBeginPos + (myCurrentEndPos - myCurrentBeginPos) / myCurrentDuration * (now - myLastEntryTime)) << "\n";
    return myCurrentBeginPos + (myCurrentEndPos - myCurrentBeginPos) / myCurrentDuration * (now - myLastEntryTime);
}


Position 
MSPModel_NonInteracting::PState::getPosition(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    const MSLane* lane = getSidewalk(stage.getEdge());
    const SUMOReal lateral_offset = lane->allowsVehicleClass(SVC_PEDESTRIAN) ? 0 : SIDEWALK_OFFSET;
    return stage.getLanePosition(lane, getEdgePos(stage, now), lateral_offset);
}


SUMOReal 
MSPModel_NonInteracting::PState::getAngle(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    //std::cout << SIMTIME << " rawAngle=" << stage.getEdgeAngle(stage.getEdge(), getEdgePos(stage, now)) << " angle=" << stage.getEdgeAngle(stage.getEdge(), getEdgePos(stage, now)) + (myCurrentEndPos < myCurrentBeginPos ? 180 : 0) << "\n";
    return -stage.getEdgeAngle(stage.getEdge(), getEdgePos(stage, now)) + (myCurrentEndPos < myCurrentBeginPos ? 180 : 0);
}


SUMOTime 
MSPModel_NonInteracting::PState::getWaitingTime(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    return 0;
}

/****************************************************************************/
