/****************************************************************************/
/// @file    MSCModel_NonInteracting.h
/// @author  Melanie Weber
/// @author  Andreas Kendziorra
/// @date    Tue, 29 July 2014
/// @version $Id: MSCModel_NonInteracting.cpp 16797 2014-07-28 12:08:15Z kend-an $
///
// The container following model for transfer (prototype)
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
//
#include <math.h>
#include <algorithm>
#include <utils/common/RandHelper.h>
#include <utils/options/OptionsCont.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSJunction.h>
#include "MSCModel_NonInteracting.h"


// named constants
const int MSCModel_NonInteracting::FORWARD(1);
const int MSCModel_NonInteracting::BACKWARD(-1);
const int MSCModel_NonInteracting::UNDEFINED_DIRECTION(0);
const SUMOReal MSCModel_NonInteracting::LATERAL_OFFSET(3);

// ===========================================================================
// MSCModel_NonInteracting method definitions
// ===========================================================================

MSCModel_NonInteracting::MSCModel_NonInteracting(const OptionsCont& oc, MSNet* net) :
    myNet(net) {
    assert(myNet != 0);
    UNUSED_PARAMETER(oc);
}


MSCModel_NonInteracting::~MSCModel_NonInteracting() {
}


MSCModel_NonInteracting::CState*
MSCModel_NonInteracting::add(MSContainer* container, MSContainer::MSContainerStage_Transfer* stage, SUMOTime now) {
    CState* state = new CState();
    const SUMOTime firstEdgeDuration = state->computeTransferTime(0, *stage, now);
    myNet->getBeginOfTimestepEvents().addEvent(new MoveToNextEdge(container, *stage),
            now + firstEdgeDuration, MSEventControl::ADAPT_AFTER_EXECUTION);
    return state;
}


//bool
//MSCModel_NonInteracting::blockedAtDist(const MSLane*, SUMOReal, std::vector<const MSPerson*>*) {
//    return false;
//}


SUMOTime
MSCModel_NonInteracting::MoveToNextEdge::execute(SUMOTime currentTime) {
    CState* state = myParent.getContainerState();
    const MSEdge* old = myParent.getEdge();
    const bool arrived = myParent.moveToNextEdge(myContainer, currentTime);
    if (arrived) {
        // transfer finished. clean up state
        delete state;
        return 0;
    } else {
        return state->computeTransferTime(old, myParent, currentTime);
    }
}

//MSCModel_NonInteracting::CState() {
//}


//MSCModel_NonInteracting::~CState {
//}

SUMOReal
MSCModel_NonInteracting::CState::getEdgePos(const MSContainer::MSContainerStage_Transfer&, SUMOTime now) const {
    return myCurrentBeginPos + (myCurrentEndPos - myCurrentBeginPos) / myCurrentDuration * (now - myLastEntryTime);
}


Position
MSCModel_NonInteracting::CState::getPosition(const MSContainer::MSContainerStage_Transfer& stage, SUMOTime now) const {
    const MSLane* lane = stage.getEdge()->getLanes().front();
    //containers get always an offset with respect to the edge
    return stage.getLanePosition(lane, getEdgePos(stage, now), LATERAL_OFFSET);
}


SUMOReal
MSCModel_NonInteracting::CState::getAngle(const MSContainer::MSContainerStage_Transfer& stage, SUMOTime now) const {
    SUMOReal angle = stage.getEdgeAngle(stage.getEdge(), getEdgePos(stage, now)) + (myCurrentEndPos < myCurrentBeginPos ? 180 : 0);
    if (angle > 180) {
        angle -= 360;
    }
    return angle;
}


SUMOReal
MSCModel_NonInteracting::CState::getSpeed(const MSContainer::MSContainerStage_Transfer& stage) const {
    return stage.getMaxSpeed();
}


SUMOTime
MSCModel_NonInteracting::CState::computeTransferTime(const MSEdge* prev, const MSContainer::MSContainerStage_Transfer& stage, SUMOTime currentTime) {
    myLastEntryTime = currentTime;
    const MSEdge* edge = stage.getEdge();
    const MSEdge* next = stage.getNextRouteEdge();
    int dir = UNDEFINED_DIRECTION;
    if (prev == 0) {
        myCurrentBeginPos = stage.getDepartPos();
    } else {
        // default to FORWARD if not connected
        dir = (edge->getToJunction() == prev->getToJunction() || edge->getToJunction() == prev->getFromJunction()) ? BACKWARD : FORWARD;
        myCurrentBeginPos = dir == FORWARD ? 0 : edge->getLength();
    }
    if (next == 0) {
        myCurrentEndPos = stage.getArrivalPos();
    } else {
        if (dir == UNDEFINED_DIRECTION) {
            // default to FORWARD if not connected
            dir = (edge->getFromJunction() == next->getFromJunction() || edge->getFromJunction() == next->getToJunction()) ? BACKWARD : FORWARD;
        }
        myCurrentEndPos = dir == FORWARD ? edge->getLength() : 0;
    }
    // ensure that a result > 0 is returned even if the transfer ends immediately
    myCurrentDuration = MAX2((SUMOTime)1, TIME2STEPS(fabs(myCurrentEndPos - myCurrentBeginPos) / stage.getMaxSpeed()));
    return myCurrentDuration;
}



/****************************************************************************/
