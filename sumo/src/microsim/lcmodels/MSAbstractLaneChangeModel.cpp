/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSAbstractLaneChangeModel.cpp
/// @author  Daniel Krajzewicz
/// @author  Friedemann Wesner
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Fri, 29.04.2005
/// @version $Id$
///
// Interface for lane-change models
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/options/OptionsCont.h>
#include "MSAbstractLaneChangeModel.h"
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSGlobals.h>
#include "MSLCM_DK2008.h"
#include "MSLCM_LC2013.h"
#include "MSLCM_SL2015.h"

/* -------------------------------------------------------------------------
 * static members
 * ----------------------------------------------------------------------- */
bool MSAbstractLaneChangeModel::myAllowOvertakingRight(false);
bool MSAbstractLaneChangeModel::myLCOutput(false);
const double MSAbstractLaneChangeModel::NO_NEIGHBOR(std::numeric_limits<double>::max());

/* -------------------------------------------------------------------------
 * MSAbstractLaneChangeModel-methods
 * ----------------------------------------------------------------------- */

void
MSAbstractLaneChangeModel::initGlobalOptions(const OptionsCont& oc) {
    myAllowOvertakingRight = oc.getBool("lanechange.overtake-right");
    myLCOutput = oc.isSet("lanechange-output");
}


MSAbstractLaneChangeModel*
MSAbstractLaneChangeModel::build(LaneChangeModel lcm, MSVehicle& v) {
    if (MSGlobals::gLateralResolution > 0 && lcm != LCM_SL2015 && lcm != LCM_DEFAULT) {
        throw ProcessError("Lane change model '" + toString(lcm) + "' is not compatible with sublane simulation");
    }
    switch (lcm) {
        case LCM_DK2008:
            return new MSLCM_DK2008(v);
        case LCM_LC2013:
            return new MSLCM_LC2013(v);
        case LCM_SL2015:
            return new MSLCM_SL2015(v);
        case LCM_DEFAULT:
            if (MSGlobals::gLateralResolution <= 0) {
                return new MSLCM_LC2013(v);
            } else {
                return new MSLCM_SL2015(v);
            }
        default:
            throw ProcessError("Lane change model '" + toString(lcm) + "' not implemented");
    }
}


MSAbstractLaneChangeModel::MSAbstractLaneChangeModel(MSVehicle& v, const LaneChangeModel model) :
    myVehicle(v),
    myOwnState(0),
    mySpeedLat(0),
    myCommittedSpeed(0),
    myLaneChangeCompletion(1.0),
    myLaneChangeDirection(0),
    myAlreadyChanged(false),
    myShadowLane(0),
    myCarFollowModel(v.getCarFollowModel()),
    myModel(model),
    myLastLaneChangeOffset(0),
    myAmOpposite(false) {
}


MSAbstractLaneChangeModel::~MSAbstractLaneChangeModel() {
}

void
MSAbstractLaneChangeModel::setOwnState(const int state) {
    myOwnState = state;
    // reset lateral influence after step is completed
    if (myVehicle.hasInfluencer()) {
        myVehicle.getInfluencer().setSublaneChange(0);
    }
}

bool
MSAbstractLaneChangeModel::congested(const MSVehicle* const neighLeader) {
    if (neighLeader == 0) {
        return false;
    }
    // Congested situation are relevant only on highways (maxSpeed > 70km/h)
    // and congested on German Highways means that the vehicles have speeds
    // below 60km/h. Overtaking on the right is allowed then.
    if ((myVehicle.getLane()->getSpeedLimit() <= 70.0 / 3.6) || (neighLeader->getLane()->getSpeedLimit() <= 70.0 / 3.6)) {

        return false;
    }
    if (myVehicle.congested() && neighLeader->congested()) {
        return true;
    }
    return false;
}



bool
MSAbstractLaneChangeModel::predInteraction(const std::pair<MSVehicle*, double>& leader) {
    if (leader.first == 0) {
        return false;
    }
    // let's check it on highways only
    if (leader.first->getSpeed() < (80.0 / 3.6)) {
        return false;
    }
    return leader.second < myCarFollowModel.interactionGap(&myVehicle, leader.first->getSpeed());
}


bool
MSAbstractLaneChangeModel::startLaneChangeManeuver(MSLane* source, MSLane* target, int direction) {
    if (&source->getEdge() != &target->getEdge()) {
        changedToOpposite();
    }
    if (MSGlobals::gLaneChangeDuration > DELTA_T) {
        myLaneChangeCompletion = 0;
        myLaneChangeDirection = direction;
        mySpeedLat = (target->getCenterOnEdge() - source->getCenterOnEdge()) * (double)DELTA_T / (double)MSGlobals::gLaneChangeDuration;
        myVehicle.switchOffSignal(MSVehicle::VEH_SIGNAL_BLINKER_RIGHT | MSVehicle::VEH_SIGNAL_BLINKER_LEFT);
        myVehicle.switchOnSignal(direction == 1 ? MSVehicle::VEH_SIGNAL_BLINKER_LEFT : MSVehicle::VEH_SIGNAL_BLINKER_RIGHT);
        return true;
    } else {
        primaryLaneChanged(source, target, direction);
        return false;
    }
}


void
MSAbstractLaneChangeModel::primaryLaneChanged(MSLane* source, MSLane* target, int direction) {
    initLastLaneChangeOffset(direction);
    myVehicle.leaveLane(MSMoveReminder::NOTIFICATION_LANE_CHANGE, target);
    source->leftByLaneChange(&myVehicle);
    myVehicle.enterLaneAtLaneChange(target);
    target->enteredByLaneChange(&myVehicle);
    if (myLCOutput) {
        OutputDevice& of = OutputDevice::getDeviceByOption("lanechange-output");
        of.openTag("change");
        of.writeAttr(SUMO_ATTR_ID, myVehicle.getID());
        of.writeAttr(SUMO_ATTR_TYPE, myVehicle.getVehicleType().getID());
        of.writeAttr(SUMO_ATTR_TIME, time2string(MSNet::getInstance()->getCurrentTimeStep()));
        of.writeAttr(SUMO_ATTR_FROM, source->getID());
        of.writeAttr(SUMO_ATTR_TO, target->getID());
        of.writeAttr(SUMO_ATTR_DIR, direction);
        of.writeAttr(SUMO_ATTR_SPEED, myVehicle.getSpeed());
        of.writeAttr(SUMO_ATTR_POSITION, myVehicle.getPositionOnLane());
        of.writeAttr("reason", toString((LaneChangeAction)(myOwnState & ~(LCA_RIGHT | LCA_LEFT))));
        of.writeAttr("leaderGap", myLastLeaderGap == NO_NEIGHBOR ? "None" : toString(myLastLeaderGap));
        of.writeAttr("leaderSecureGap", myLastLeaderSecureGap == NO_NEIGHBOR ? "None" : toString(myLastLeaderSecureGap));
        of.writeAttr("followerGap", myLastFollowerGap == NO_NEIGHBOR ? "None" : toString(myLastFollowerGap));
        of.writeAttr("followerSecureGap", myLastFollowerSecureGap == NO_NEIGHBOR ? "None" : toString(myLastFollowerSecureGap));
        if (MSGlobals::gLateralResolution > 0) {
            const double latGap = direction < 0 ? myLastLateralGapRight : myLastLateralGapLeft;
            of.writeAttr("latGap", latGap == NO_NEIGHBOR ? "None" : toString(latGap));
        }
        of.closeTag();
    }
    changed();
}


bool
MSAbstractLaneChangeModel::updateCompletion() {
    const bool pastBefore = pastMidpoint();
    myLaneChangeCompletion += (double)DELTA_T / (double)MSGlobals::gLaneChangeDuration;
    return !pastBefore && pastMidpoint();
}


void
MSAbstractLaneChangeModel::endLaneChangeManeuver(const MSMoveReminder::Notification reason) {
    UNUSED_PARAMETER(reason);
    myLaneChangeCompletion = 1;
    cleanupShadowLane();
    myNoPartiallyOccupatedByShadow.clear();
    myVehicle.switchOffSignal(MSVehicle::VEH_SIGNAL_BLINKER_RIGHT | MSVehicle::VEH_SIGNAL_BLINKER_LEFT);
    myVehicle.fixPosition();
    if (myAmOpposite) {
        changedToOpposite();
    }
}


MSLane*
MSAbstractLaneChangeModel::getShadowLane(const MSLane* lane, double posLat) const {
    if (std::find(myNoPartiallyOccupatedByShadow.begin(), myNoPartiallyOccupatedByShadow.end(), lane) == myNoPartiallyOccupatedByShadow.end()) {
        // initialize shadow lane
        const double overlap = myVehicle.getLateralOverlap(posLat);
        if (debugVehicle()) {
            std::cout << SIMTIME << " veh=" << myVehicle.getID() << " posLat=" << posLat << " overlap=" << overlap << "\n";
        }
        if (overlap > NUMERICAL_EPS ||
                // "reserve" target lane even when there is no overlap yet
                (isChangingLanes() && myLaneChangeCompletion < 0.5)) {
            const int shadowDirection = posLat < 0 ? -1 : 1;
            return lane->getParallelLane(shadowDirection);
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}


MSLane*
MSAbstractLaneChangeModel::getShadowLane(const MSLane* lane) const {
    return getShadowLane(lane, myVehicle.getLateralPositionOnLane());
}


void
MSAbstractLaneChangeModel::cleanupShadowLane() {
    if (myShadowLane != 0) {
        if (debugVehicle()) {
            std::cout << SIMTIME << " cleanupShadowLane\n";
        }
        myShadowLane->resetPartialOccupation(&myVehicle);
        myShadowLane = 0;
    }
    for (std::vector<MSLane*>::const_iterator it = myShadowFurtherLanes.begin(); it != myShadowFurtherLanes.end(); ++it) {
        if (debugVehicle()) {
            std::cout << SIMTIME << " cleanupShadowLane2\n";
        }
        (*it)->resetPartialOccupation(&myVehicle);
    }
    myShadowFurtherLanes.clear();
    myNoPartiallyOccupatedByShadow.clear();
}


bool
MSAbstractLaneChangeModel::cancelRequest(int state) {
    int ret = myVehicle.influenceChangeDecision(state);
    return ret != state;
}


void
MSAbstractLaneChangeModel::initLastLaneChangeOffset(int dir) {
    if (dir > 0) {
        myLastLaneChangeOffset = 1;
    } else if (dir < 0) {
        myLastLaneChangeOffset = -1;
    }
}

void
MSAbstractLaneChangeModel::updateShadowLane() {
    if (myShadowLane != 0) {
        if (debugVehicle()) {
            std::cout << SIMTIME << " updateShadowLane\n";
        }
        myShadowLane->resetPartialOccupation(&myVehicle);
    }
    myShadowLane = getShadowLane(myVehicle.getLane());
    std::vector<MSLane*> passed;
    if (myShadowLane != 0) {
        myShadowLane->setPartialOccupation(&myVehicle);
        const std::vector<MSLane*>& further = myVehicle.getFurtherLanes();
        const std::vector<double>& furtherPosLat = myVehicle.getFurtherLanesPosLat();
        assert(further.size() == furtherPosLat.size());
        passed.push_back(myShadowLane);
        for (int i = 0; i < (int)further.size(); ++i) {
            MSLane* shadowFurther = getShadowLane(further[i], furtherPosLat[i]);
            if (debugVehicle()) {
                std::cout << SIMTIME << "   further=" << further[i]->getID() << " shadowFurther=" << Named::getIDSecure(shadowFurther) << "\n";
            }
            if (shadowFurther != 0 && MSLinkContHelper::getConnectingLink(*shadowFurther, *passed.back()) != 0) {
                passed.push_back(shadowFurther);
            }
        }
        std::reverse(passed.begin(), passed.end());
    } else {
        if (isChangingLanes() && myVehicle.getLateralOverlap() > NUMERICAL_EPS) {
            WRITE_WARNING("Vehicle '" + myVehicle.getID() + "' could not finish continuous lane change (lane disappeared) time=" +
                          time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
            endLaneChangeManeuver();
        }
    }
    if (debugVehicle()) std::cout << SIMTIME << " updateShadowLane veh=" << myVehicle.getID()
                                      << " newShadowLane=" << Named::getIDSecure(myShadowLane)
                                      << "\n   before:" << " myShadowFurtherLanes=" << toString(myShadowFurtherLanes) << " further=" << toString(myVehicle.getFurtherLanes()) << " passed=" << toString(passed)
                                      << "\n";
    myVehicle.updateFurtherLanes(myShadowFurtherLanes, myShadowFurtherLanesPosLat, passed);
    if (debugVehicle()) std::cout
                << "\n   after:" << " myShadowFurtherLanes=" << toString(myShadowFurtherLanes) << "\n";
}


int
MSAbstractLaneChangeModel::getShadowDirection() const {
    if (isChangingLanes()) {
        if (pastMidpoint()) {
            return -myLaneChangeDirection;
        } else {
            return myLaneChangeDirection;
        }
    } else if (myShadowLane == 0) {
        return 0;
    } else {
        assert(&myShadowLane->getEdge() == &myVehicle.getLane()->getEdge());
        return myShadowLane->getIndex() - myVehicle.getLane()->getIndex();
    }
}


double
MSAbstractLaneChangeModel::getAngleOffset() const {
    const double angleOffset = 60 / STEPS2TIME(MSGlobals::gLaneChangeDuration) * (pastMidpoint() ? 1 - myLaneChangeCompletion : myLaneChangeCompletion);
    return myLaneChangeDirection * angleOffset;
}


SUMOTime
MSAbstractLaneChangeModel::remainingTime() const {
    return (SUMOTime)((1. - myLaneChangeCompletion) * MSGlobals::gLaneChangeDuration);
}


void
MSAbstractLaneChangeModel::setShadowApproachingInformation(MSLink* link) const {
    //std::cout << SIMTIME << " veh=" << myVehicle.getID() << " @=" << &myVehicle << " set shadow approaching=" << link->getViaLaneOrLane()->getID() << "\n";
    myApproachedByShadow.push_back(link);
}

void
MSAbstractLaneChangeModel::removeShadowApproachingInformation() const {
    for (std::vector<MSLink*>::iterator it = myApproachedByShadow.begin(); it != myApproachedByShadow.end(); ++it) {
        //std::cout << SIMTIME << " veh=" << myVehicle.getID() << " @=" << &myVehicle << " remove shadow approaching=" << (*it)->getViaLaneOrLane()->getID() << "\n";
        (*it)->removeApproaching(&myVehicle);
    }
    myApproachedByShadow.clear();
}


void
MSAbstractLaneChangeModel::changedToOpposite() {
    myAmOpposite = !myAmOpposite;
    myAlreadyChanged = true;
}

void
MSAbstractLaneChangeModel::setFollowerGaps(CLeaderDist follower, double secGap)  {
    if (follower.first != 0) {
        myLastFollowerGap = follower.second + follower.first->getVehicleType().getMinGap();
        myLastFollowerSecureGap = secGap;
    }
}

void
MSAbstractLaneChangeModel::setLeaderGaps(CLeaderDist leader, double secGap) {
    if (leader.first != 0) {
        myLastLeaderGap = leader.second + myVehicle.getVehicleType().getMinGap();
        myLastLeaderSecureGap = secGap;
    }
}

void
MSAbstractLaneChangeModel::setFollowerGaps(const MSLeaderDistanceInfo& vehicles) {
    int rightmost;
    int leftmost;
    vehicles.getSubLanes(&myVehicle, 0, rightmost, leftmost);
    for (int i = rightmost; i <= leftmost; ++i) {
        CLeaderDist vehDist = vehicles[i];
        if (vehDist.first != 0) {
            const MSVehicle* leader = &myVehicle;
            const MSVehicle* follower = vehDist.first;
            const double netGap = vehDist.second + follower->getVehicleType().getMinGap();
            if (netGap < myLastFollowerGap) {
                myLastFollowerGap = netGap;
                myLastFollowerSecureGap = follower->getCarFollowModel().getSecureGap(follower->getSpeed(), leader->getSpeed(), leader->getCarFollowModel().getMaxDecel());
            }
        }
    }
}

void
MSAbstractLaneChangeModel::setLeaderGaps(const MSLeaderDistanceInfo& vehicles) {
    int rightmost;
    int leftmost;
    vehicles.getSubLanes(&myVehicle, 0, rightmost, leftmost);
    for (int i = rightmost; i <= leftmost; ++i) {
        CLeaderDist vehDist = vehicles[i];
        if (vehDist.first != 0) {
            const MSVehicle* leader = vehDist.first;
            const MSVehicle* follower = &myVehicle;
            const double netGap = vehDist.second + follower->getVehicleType().getMinGap();
            if (netGap < myLastLeaderGap) {
                myLastLeaderGap = netGap;
                myLastLeaderSecureGap = follower->getCarFollowModel().getSecureGap(follower->getSpeed(), leader->getSpeed(), leader->getCarFollowModel().getMaxDecel());
            }
        }
    }
}
