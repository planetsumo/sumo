/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2014-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSPModel_NonInteracting.cpp
/// @author  Jakob Erdmann
/// @date    Mon, 13 Jan 2014
/// @version $Id$
///
// The pedestrian following model (prototype)
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cmath>
#include <algorithm>
#include <utils/common/RandHelper.h>
#include <utils/geom/GeomHelper.h>
#include <utils/options/OptionsCont.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSJunction.h>
#include <microsim/MSEventControl.h>
#include "MSPModel_NonInteracting.h"


// ===========================================================================
// DEBUGGING HELPERS
// ===========================================================================
//
#define DEBUG1 "disabled"
#define DEBUG2 "disabled"
#define DEBUGCOND(PEDID) (PEDID == DEBUG1 || PEDID == DEBUG2)

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
    myNet(net) {
    assert(myNet != 0);
    UNUSED_PARAMETER(oc);
}


MSPModel_NonInteracting::~MSPModel_NonInteracting() {
}


PedestrianState*
MSPModel_NonInteracting::add(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, SUMOTime now) {
    MoveToNextEdge* cmd = new MoveToNextEdge(person, *stage);
    PState* state = new PState(cmd);
    const SUMOTime firstEdgeDuration = state->computeWalkingTime(0, *stage, now);
    myNet->getBeginOfTimestepEvents()->addEvent(cmd, now + firstEdgeDuration);

    //if DEBUGCOND(person->getID()) std::cout << SIMTIME << " " << person->getID() << " inserted on " << stage->getEdge()->getID() << "\n";
    return state;
}


void
MSPModel_NonInteracting::remove(PedestrianState* state) {
    dynamic_cast<PState*>(state)->getCommand()->abortWalk();
}


SUMOTime
MSPModel_NonInteracting::MoveToNextEdge::execute(SUMOTime currentTime) {
    if (myPerson == 0) {
        return 0; // descheduled
    }
    PState* state = dynamic_cast<PState*>(myParent.getPedestrianState());
    const MSEdge* old = myParent.getEdge();
    const bool arrived = myParent.moveToNextEdge(myPerson, currentTime);
    if (arrived) {
        // walk finished. clean up state
        delete state;
        //if DEBUGCOND(myPerson->getID()) std::cout << SIMTIME << " " << myPerson->getID() << " arrived on " << old->getID() << "\n";
        return 0;
    } else {
        //if DEBUGCOND(myPerson->getID()) std::cout << SIMTIME << " " << myPerson->getID() << " moves to " << myParent.getEdge()->getID() << "\n";
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
    // ensure that a result > 0 is returned even if the walk ends immediately
    myCurrentDuration = MAX2((SUMOTime)1, TIME2STEPS(fabs(myCurrentEndPos - myCurrentBeginPos) / stage.getMaxSpeed(myCommand->getPerson())));
    //std::cout << SIMTIME << " dir=" << dir << " curBeg=" << myCurrentBeginPos << " curEnd=" << myCurrentEndPos << " speed=" << stage.getMaxSpeed() << " dur=" << myCurrentDuration << "\n";
    return myCurrentDuration;
}


double
MSPModel_NonInteracting::PState::getEdgePos(const MSPerson::MSPersonStage_Walking&, SUMOTime now) const {
    //std::cout << SIMTIME << " pos=" << (myCurrentBeginPos + (myCurrentEndPos - myCurrentBeginPos) / myCurrentDuration * (now - myLastEntryTime)) << "\n";
    return myCurrentBeginPos + (myCurrentEndPos - myCurrentBeginPos) / myCurrentDuration * (now - myLastEntryTime);
}


Position
MSPModel_NonInteracting::PState::getPosition(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    const MSLane* lane = getSidewalk<MSEdge, MSLane>(stage.getEdge());
    if (lane == 0) {
        //std::string error = "Pedestrian '" + myCommand->myPerson->getID() + "' could not find sidewalk on edge '" + state.getEdge()->getID() + "', time="
        //    + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".";
        //if (!OptionsCont::getOptions().getBool("ignore-route-errors")) {
        //    throw ProcessError(error);
        //}
        lane = stage.getEdge()->getLanes().front();
    }
    const double lateral_offset = (lane->allowsVehicleClass(SVC_PEDESTRIAN) ? 0 : SIDEWALK_OFFSET
                                   * (MSNet::getInstance()->lefthand() ? -1 : 1));
    return stage.getLanePosition(lane, getEdgePos(stage, now), lateral_offset);
}


double
MSPModel_NonInteracting::PState::getAngle(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    //std::cout << SIMTIME << " rawAngle=" << stage.getEdgeAngle(stage.getEdge(), getEdgePos(stage, now)) << " angle=" << stage.getEdgeAngle(stage.getEdge(), getEdgePos(stage, now)) + (myCurrentEndPos < myCurrentBeginPos ? 180 : 0) << "\n";
    double angle = stage.getEdgeAngle(stage.getEdge(), getEdgePos(stage, now)) + (myCurrentEndPos < myCurrentBeginPos ? M_PI : 0);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}


SUMOTime
MSPModel_NonInteracting::PState::getWaitingTime(const MSPerson::MSPersonStage_Walking&, SUMOTime) const {
    return 0;
}


double
MSPModel_NonInteracting::PState::getSpeed(const MSPerson::MSPersonStage_Walking& stage) const {
    return stage.getMaxSpeed(myCommand->getPerson());
}


const MSEdge*
MSPModel_NonInteracting::PState::getNextEdge(const MSPerson::MSPersonStage_Walking& stage) const {
    return stage.getNextRouteEdge();
}

/****************************************************************************/
