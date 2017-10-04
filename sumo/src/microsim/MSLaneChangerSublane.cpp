/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSLaneChangerSublane.cpp
/// @author  Jakob Erdmann
/// @date    Oct 2015
/// @version $Id$
///
// Performs sub-lane changing of vehicles
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSLaneChangerSublane.h"
#include "MSNet.h"
#include "MSVehicle.h"
#include "MSVehicleType.h"
#include "MSVehicleTransfer.h"
#include "MSGlobals.h"
#include <cassert>
#include <iterator>
#include <cstdlib>
#include <cmath>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/MsgHandler.h>
#include <utils/geom/GeomHelper.h>


// ===========================================================================
// member method definitions
// ===========================================================================
MSLaneChangerSublane::MSLaneChangerSublane(const std::vector<MSLane*>* lanes, bool allowChanging) :
    MSLaneChanger(lanes, allowChanging) {
}


MSLaneChangerSublane::~MSLaneChangerSublane() {}

void
MSLaneChangerSublane::initChanger() {
    MSLaneChanger::initChanger();
    // Prepare myChanger with a safe state.
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
        ce->ahead = ce->lane->getPartialBeyond();
        //std::cout << SIMTIME << " initChanger lane=" << ce->lane->getID() << " vehicles=" << toString(ce->lane->myVehicles) << "\n";
    }
}



void
MSLaneChangerSublane::updateChanger(bool vehHasChanged) {
    MSLaneChanger::updateChanger(vehHasChanged);
    if (!vehHasChanged) {
        MSVehicle* lead = myCandi->lead;
        //std::cout << SIMTIME << " updateChanger lane=" << myCandi->lane->getID() << " lead=" << Named::getIDSecure(lead) << "\n";
        myCandi->ahead.addLeader(lead, false, 0);
        MSLane* shadowLane = lead->getLaneChangeModel().getShadowLane();
        if (shadowLane != 0) {
            const double latOffset = lead->getLane()->getRightSideOnEdge() - shadowLane->getRightSideOnEdge();
            //std::cout << SIMTIME << " updateChanger shadowLane=" << shadowLane->getID() << " lead=" << Named::getIDSecure(lead) << "\n";
            (myChanger.begin() + shadowLane->getIndex())->ahead.addLeader(lead, false, latOffset);
        }
    }
    //std::cout << SIMTIME << " updateChanger: lane=" << myCandi->lane->getID() << " lead=" << Named::getIDSecure(myCandi->lead) << " ahead=" << myCandi->ahead.toString() << " vehHasChanged=" << vehHasChanged << "\n";
    //for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
    //    std::cout << " lane=" << ce->lane->getID() << " vehicles=" << toString(ce->lane->myVehicles) << "\n";
    //}
}


bool
MSLaneChangerSublane::change() {
    // variant of change() for the sublane case
    myCandi = findCandidate();
    MSVehicle* vehicle = veh(myCandi);
    assert(vehicle->getLane() == (*myCandi).lane);
    assert(!vehicle->getLaneChangeModel().isChangingLanes());
    if (/*!myAllowsChanging || vehicle->getLaneChangeModel().alreadyChanged() ||*/ vehicle->isStoppedOnLane()) {
        registerUnchanged(vehicle);
        return false;
    }
#ifndef NO_TRACI
    if (vehicle->isRemoteControlled()) {
        registerUnchanged(vehicle);
        return false;
    }
#endif
    vehicle->updateBestLanes(); // needed?
    for (int i = 0; i < (int) myChanger.size(); ++i) {
        vehicle->adaptBestLanesOccupation(i, myChanger[i].dens);
    }

    // update expected speeds
    int sublaneIndex = 0;
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
        vehicle->getLaneChangeModel().updateExpectedSublaneSpeeds(ce->ahead, sublaneIndex, ce->lane->getIndex());
        sublaneIndex += ce->ahead.numSublanes();
    }

    LaneChangeAction alternatives = (LaneChangeAction)((mayChange(-1) ? LCA_RIGHT : LCA_NONE)
                                    | (mayChange(1) ? LCA_LEFT : LCA_NONE));

    StateAndDist right = checkChangeHelper(vehicle, -1, alternatives);
    StateAndDist left = checkChangeHelper(vehicle, 1, alternatives);
    StateAndDist current = checkChangeHelper(vehicle, 0, alternatives);

    StateAndDist decision = vehicle->getLaneChangeModel().decideDirection(current,
                            vehicle->getLaneChangeModel().decideDirection(right, left));
    vehicle->getLaneChangeModel().setOwnState(decision.state);
    if ((decision.state & LCA_WANTS_LANECHANGE) != 0 && (decision.state & LCA_BLOCKED) == 0) {
        // change if the vehicle wants to and is allowed to change
        if (vehicle->getLaneChangeModel().debugVehicle()) {
            std::cout << SIMTIME << " decision=" << toString((LaneChangeAction)decision.state) << " latDist=" << decision.latDist << "\n";
        }
        return startChangeSublane(vehicle, myCandi, decision.latDist);
    } else {
        // @note this assumes vehicles can instantly abort any maneuvre in case of emergency
        vehicle->getLaneChangeModel().setSpeedLat(0);
    }

    if ((right.state & (LCA_URGENT)) != 0 && (left.state & (LCA_URGENT)) != 0) {
        // ... wants to go to the left AND to the right
        // just let them go to the right lane...
        left.state = 0;
    }
    registerUnchanged(vehicle);
    return false;
}


MSLaneChangerSublane::StateAndDist
MSLaneChangerSublane::checkChangeHelper(MSVehicle* vehicle, int laneOffset, LaneChangeAction alternatives) {
    StateAndDist result = StateAndDist(0, 0, 0);
    if (mayChange(laneOffset)) {
        const std::vector<MSVehicle::LaneQ>& preb = vehicle->getBestLanes();
        result.state = checkChangeSublane(laneOffset, alternatives, preb, result.latDist);
        result.dir = laneOffset;
        if ((result.state & LCA_WANTS_LANECHANGE) != 0 && (result.state & LCA_URGENT) != 0 && (result.state & LCA_BLOCKED) != 0) {
            (myCandi + laneOffset)->lastBlocked = vehicle;
            if ((myCandi + laneOffset)->firstBlocked == 0) {
                (myCandi + laneOffset)->firstBlocked = vehicle;
            }
        }
    }
    return result;
}


bool
MSLaneChangerSublane::startChangeSublane(MSVehicle* vehicle, ChangerIt& from, double latDist) {
    // 1) update vehicles lateral position according to latDist and target lane
    vehicle->myState.myPosLat += latDist;
    vehicle->myCachedPosition = Position::INVALID;
    vehicle->getLaneChangeModel().setSpeedLat(DIST2SPEED(latDist));

    // 2) distinguish several cases
    //   a) vehicle moves completely within the same lane
    //   b) vehicle intersects another lane
    //      - vehicle must be moved to the lane where it's midpoint is (either old or new)
    //      - shadow vehicle must be created/moved to the other lane if the vehicle intersects it
    // 3) updated dens of all lanes that hold the vehicle or its shadow
    const int direction = vehicle->getLateralPositionOnLane() < 0 ? -1 : 1;
    ChangerIt to = from;
    if (mayChange(direction)) {
        to = from + direction;
    } else {
        /// XXX assert(false);
    }
    const bool changedToNewLane = to != from && fabs(vehicle->getLateralPositionOnLane()) > 0.5 * vehicle->getLane()->getWidth() && mayChange(direction);
    if (changedToNewLane) {
        vehicle->myState.myPosLat -= direction * 0.5 * (from->lane->getWidth() + to->lane->getWidth());
        to->lane->myTmpVehicles.insert(to->lane->myTmpVehicles.begin(), vehicle);
        to->dens += vehicle->getVehicleType().getLengthWithGap();
        if (MSAbstractLaneChangeModel::haveLCOutput()) {
            vehicle->getLaneChangeModel().setLeaderGaps(getLeaders(to, vehicle));
            vehicle->getLaneChangeModel().setFollowerGaps(to->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true));
        }
        vehicle->getLaneChangeModel().startLaneChangeManeuver(from->lane, to->lane, direction);
        to->ahead.addLeader(vehicle, false, 0);
    } else {
        registerUnchanged(vehicle);
        from->ahead.addLeader(vehicle, false, 0);
    }

    MSLane* oldShadowLane = vehicle->getLaneChangeModel().getShadowLane();
    vehicle->getLaneChangeModel().updateShadowLane();
    MSLane* shadowLane = vehicle->getLaneChangeModel().getShadowLane();
    if (shadowLane != 0 && shadowLane != oldShadowLane) {
        assert(to != from);
        const double latOffset = vehicle->getLane()->getRightSideOnEdge() - shadowLane->getRightSideOnEdge();
        (myChanger.begin() + shadowLane->getIndex())->ahead.addLeader(vehicle, false, latOffset);
    }


    // compute new angle of the vehicle from the x- and y-distances travelled within last time step
    // (should happen last because primaryLaneChanged() also triggers angle computation)
    // this part of the angle comes from the orientation of our current lane
    double laneAngle = vehicle->getLane()->getShape().rotationAtOffset(vehicle->getLane()->interpolateLanePosToGeometryPos(vehicle->getPositionOnLane())) ;
    // this part of the angle comes from the vehicle's lateral movement
    double changeAngle = 0;
    // avoid flicker
    if (fabs(latDist) > NUMERICAL_EPS) {
        // angle is between vehicle front and vehicle back (and depending on travelled distance)
        changeAngle = atan2(latDist, vehicle->getVehicleType().getLength() + SPEED2DIST(vehicle->getSpeed()));
    }
    if (vehicle->getLaneChangeModel().debugVehicle()) std::cout << SIMTIME << " startChangeSublane shadowLane"
                << " latDist=" << latDist
                << " old=" << Named::getIDSecure(oldShadowLane)
                << " new=" << Named::getIDSecure(vehicle->getLaneChangeModel().getShadowLane())
                << " laneA=" << RAD2DEG(laneAngle)
                << " changeA=" << RAD2DEG(changeAngle)
                << " oldA=" << RAD2DEG(vehicle->getAngle())
                << " newA=" << RAD2DEG(laneAngle + changeAngle)
                << "\n";
    vehicle->setAngle(laneAngle + changeAngle);

    return changedToNewLane;
}


MSLeaderDistanceInfo
MSLaneChangerSublane::getLeaders(const ChangerIt& target, const MSVehicle* ego) const {
    //if (ego->getID() == "C" && SIMTIME == 17) {
    //    std::cout << "DEBUG\n";
    //}
    // get the leading vehicle on the lane to change to
    if (gDebugFlag1) {
        std::cout << SIMTIME << " getLeaders lane=" << target->lane->getID() << " ego=" << ego->getID() << " ahead=" << target->ahead.toString() << "\n";
    }
    MSLeaderDistanceInfo result(target->lane, 0, 0);
    for (int i = 0; i < target->ahead.numSublanes(); ++i) {
        const MSVehicle* veh = target->ahead[i];
        if (veh != 0) {
            const double gap = veh->getBackPositionOnLane(target->lane) - ego->getPositionOnLane() - ego->getVehicleType().getMinGap();
            if (gDebugFlag1) {
                std::cout << " ahead lead=" << veh->getID() << " leadBack=" << veh->getBackPositionOnLane() << " gap=" << gap << "\n";
            }
            result.addLeader(veh, gap, 0, i);
        }
    }
    // if there are vehicles on the target lane with the same position as ego,
    // they may not have been added to 'ahead' yet
    const MSLeaderInfo& aheadSamePos = target->lane->getLastVehicleInformation(0, 0, ego->getPositionOnLane(), false);
    for (int i = 0; i < aheadSamePos.numSublanes(); ++i) {
        const MSVehicle* veh = aheadSamePos[i];
        if (veh != 0 && veh != ego) {
            const double gap = veh->getBackPositionOnLane(target->lane) - ego->getPositionOnLane() - ego->getVehicleType().getMinGap();
            if (gDebugFlag1) {
                std::cout << " further lead=" << veh->getID() << " leadBack=" << veh->getBackPositionOnLane(target->lane) << " gap=" << gap << "\n";
            }
            result.addLeader(veh, gap, 0, i);
        }
    }

    if (result.numFreeSublanes() > 0) {
        MSLane* targetLane = target->lane;

        double seen = ego->getLane()->getLength() - ego->getPositionOnLane();
        double speed = ego->getSpeed();
        double dist = ego->getCarFollowModel().brakeGap(speed) + ego->getVehicleType().getMinGap();
        if (seen > dist) {
            return result;
        }
        const std::vector<MSLane*>& bestLaneConts = veh(myCandi)->getBestLanesContinuation(targetLane);
        if (gDebugFlag1) {
            std::cout << " add consecutive before=" << result.toString() << " dist=" << dist;
        }
        target->lane->getLeadersOnConsecutive(dist, seen, speed, ego, bestLaneConts, result);
        if (gDebugFlag1) {
            std::cout << " after=" << result.toString() << "\n";
        }
    }
    return result;
}


int
MSLaneChangerSublane::checkChangeSublane(
    int laneOffset,
    LaneChangeAction alternatives,
    const std::vector<MSVehicle::LaneQ>& preb,
    double& latDist) const {

    ChangerIt target = myCandi + laneOffset;
    MSVehicle* vehicle = veh(myCandi);
    const MSLane& neighLane = *(target->lane);
    int blocked = 0;

    //gDebugFlag1 = vehicle->getLaneChangeModel().debugVehicle();

    MSLeaderDistanceInfo neighLeaders = getLeaders(target, vehicle);
    MSLeaderDistanceInfo neighFollowers = target->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true);
    MSLeaderDistanceInfo neighBlockers(&neighLane, vehicle, vehicle->getLane()->getRightSideOnEdge() - neighLane.getRightSideOnEdge());
    MSLeaderDistanceInfo leaders = getLeaders(myCandi, vehicle);
    MSLeaderDistanceInfo followers = myCandi->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true);
    MSLeaderDistanceInfo blockers(vehicle->getLane(), vehicle, 0);

    if (gDebugFlag1) std::cout << SIMTIME
                                   << " checkChangeSublane: veh=" << vehicle->getID()
                                   << " laneOffset=" << laneOffset
                                   << "\n  leaders=" << leaders.toString()
                                   << "\n  neighLeaders=" << neighLeaders.toString()
                                   << "\n  followers=" << followers.toString()
                                   << "\n  neighFollowers=" << neighFollowers.toString()
                                   << "\n";


    const int wish = vehicle->getLaneChangeModel().wantsChangeSublane(
                         laneOffset, alternatives,
                         leaders, followers, blockers,
                         neighLeaders, neighFollowers, neighBlockers,
                         neighLane, preb,
                         &(myCandi->lastBlocked), &(myCandi->firstBlocked), latDist, blocked);
    int state = blocked | wish;

    // XXX
    // do are more carefull (but expensive) check to ensure that a
    // safety-critical leader is not being overloocked

    // XXX
    // ensure that a continuous lane change manoeuvre can be completed
    // before the next turning movement

    const int oldstate = state;
#ifndef NO_TRACI
    // let TraCI influence the wish to change lanes and the security to take
    state = vehicle->influenceChangeDecision(state);
    //if (vehicle->getID() == "150_2_36000000") {
    //    std::cout << STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep()) << " veh=" << vehicle->getID() << " oldstate=" << oldstate << " newstate=" << state << "\n";
    //}
#endif
    vehicle->getLaneChangeModel().saveState(laneOffset, oldstate, state);
    gDebugFlag1 = false;
    return state;
}

/****************************************************************************/

