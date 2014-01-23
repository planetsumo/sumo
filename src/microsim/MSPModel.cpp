/****************************************************************************/
/// @file    MSPModel.h
/// @author  Jakob Erdmann
/// @date    Mon, 13 Jan 2014
/// @version $Id$
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
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSJunction.h>
#include "MSPModel.h"

#define DEBUG1 "disabled"
#define DEBUG2 "disabled"
#define LOG_ALL false

// ===========================================================================
// static members
// ===========================================================================
int MSPModel::myNumActivePedestrians(0);

MSPModel::ActiveLanes MSPModel::myActiveLanes;

MSPModel::Pedestrians MSPModel::noPedestrians;

// named constants
const int MSPModel::FORWARD(1);
const int MSPModel::BACKWARD(-1);

// model parameters
const SUMOReal MSPModel::SAFETY_GAP(1.0);
const SUMOReal MSPModel::STRIPE_WIDTH(0.75);
const SUMOReal MSPModel::LOOKAHEAD(10.0);
const SUMOReal MSPModel::LATERAL_PENALTY(-1.0);
const SUMOReal MSPModel::SQUEEZE(0.7);
const SUMOReal MSPModel::BLOCKER_LOOKAHEAD(10.0);
const SUMOReal MSPModel::ONCOMIN_PENALTY(0.0);
const SUMOReal MSPModel::ONCOMIN_PENALTY_FACTOR(0.6);
const SUMOReal MSPModel::RESERVE_FOR_ONCOMING_FACTOR(0.25);

// ===========================================================================
// MSPModel method definitions
// ===========================================================================

void
MSPModel::add(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, MSNet* net) {
    assert(person->getCurrentStageType() == MSPerson::WALKING);
    const MSLane* lane = getSidwalk(person->getEdge());
    myActiveLanes[lane].push_back(Pedestrian(person, stage, lane));
    if (net != 0 && myNumActivePedestrians == 0) {
        net->getBeginOfTimestepEvents().addEvent(new MovePedestrians(), net->getCurrentTimeStep() + DELTA_T, MSEventControl::ADAPT_AFTER_EXECUTION);
    }
    myNumActivePedestrians++;
}


bool 
MSPModel::blockedAtDist(const MSLane* lane, SUMOReal distToCrossing) {
    const Pedestrians& pedestrians = getPedestrians(lane);
    for (Pedestrians::const_iterator it_ped = pedestrians.begin(); it_ped != pedestrians.end(); ++it_ped) {
        const Pedestrian& ped = *it_ped;
        const SUMOReal leaderBackDist = (ped.myDir == FORWARD 
                ? distToCrossing - (ped.myX - ped.getLength() - MSPModel::SAFETY_GAP)
                : (ped.myX + ped.getLength() + MSPModel::SAFETY_GAP) - distToCrossing); 
        //std::cout << SIMTIME << " foe=" << foeLane->getID() << " dir=" << p.myDir << " pX=" << ped.myX << " pL=" << ped.getLength() << " fDTC=" << distToCrossing << " lBD=" << leaderBackDist << "\n";
        if (leaderBackDist >= 0 && leaderBackDist <= BLOCKER_LOOKAHEAD) {
            // found one pedestrian that is not completely past the crossing point
            return true;
        }
    }
    return false;
}


MSPModel::Pedestrians& 
MSPModel::getPedestrians(const MSLane* lane) {
    ActiveLanes::iterator it = myActiveLanes.find(lane);
    if (it != myActiveLanes.end()) {
        //std::cout << " found lane=" << lane->getID() << " n=" << it->second.size() << "\n";
        return (it->second);
    } else {
        return noPedestrians;
    }
}


void
MSPModel::cleanup() {
    myActiveLanes.clear();
    myNumActivePedestrians = 0;
}


void
MSPModel::addToLane(Pedestrian& ped, int oldStripes, const MSJunction* junction, const MSLane* newLane) {
    if (newLane != 0) {
        //if (ped.myPerson->getID() == DEBUG1) {
        //    std::cout << SIMTIME << " x=" << ped.myX << " junction=" << junction->getID() << " newLane=" << newLane->getID() << " newTo=" << newLane->getEdge().getToJunction()->getID() << "\n";
        //}
        ped.updateDirection(newLane, junction);
        //if (ped.myPerson->getID() == DEBUG1) {
        //    std::cout << SIMTIME << " after update x=" << ped.myX << "\n";
        //}
        ped.myY += 0.5 * (numStripes(newLane) - oldStripes);
        ped.updateLocation(newLane);
        myActiveLanes[newLane].push_back(ped);
    } else {
        myNumActivePedestrians--;
    }
}


MSLane* 
MSPModel::getSidwalk(const MSEdge* edge) {
    if (edge == 0) {
        return 0;
    }
    const std::vector<MSLane*>& lanes = edge->getLanes();
    for (std::vector<MSLane*>::const_iterator it = lanes.begin(); it != lanes.end(); ++it) {
        if ((*it)->getPermissions() == SVC_PEDESTRIAN) {
            return *it;
        }
    }
    return lanes.front();
}

int 
MSPModel::numStripes(const MSLane* lane) {
    return (int)floor(lane->getWidth() / STRIPE_WIDTH); 
}


const MSLane*
MSPModel::getNextLane(const MSLane* currentLane, const Pedestrian& ped) {
    const MSLane* nextRouteLane = getSidwalk(ped.myStage->getNextEdge());
    const MSLane* nextLane = nextRouteLane;
    if (nextRouteLane != 0) {
        // route continues, check for internal edge in between
        if (ped.myDir == FORWARD) {
            nextLane = MSLinkContHelper::getInternalFollowingLane(currentLane, nextRouteLane);
        } else {
            nextLane = MSLinkContHelper::getInternalFollowingLane(nextRouteLane, currentLane);
        }
        if (nextLane == 0 || nextLane->getLength() <= POSITION_EPS) {
            // no internal lane found or it's too short, jump directly to next route lane
            nextLane = nextRouteLane;
        }
    }
    return nextLane;
}


int
MSPModel::getDirection(const MSEdge* edge, const MSEdge* nextEdge) {
    assert(edge != 0);
    assert(nextEdge != 0);
    if (edge->isInternal()) {
        assert(!nextEdge->isInternal());
        assert(edge->getNoFollowing() == 1);
        return edge->getFollower(0) == nextEdge ? 1 : -1;
    }
    if (nextEdge->isInternal()) {
        return edge == &getSidwalk(nextEdge)->getLogicalPredecessorLane()->getEdge() ? 1 : -1;
    }
    // check junction topology
    if (edge->getToJunction() == nextEdge->getFromJunction() 
            || edge->getToJunction() == nextEdge->getToJunction() ) {
        return 1;
    } else if  (edge->getFromJunction() == nextEdge->getToJunction()
            || edge->getFromJunction() == nextEdge->getFromJunction()) {
        return -1;
    } else {
        return 1; // not connected it any way. just walk forward.
    }
}


int
MSPModel::countWaitingToEnter(const std::vector<Pedestrian>& pedestrians) {
    int count = 0;
    for (Pedestrians::const_iterator it = pedestrians.begin(); it != pedestrians.end(); ++it) {
        if ((*it).myWaitingToEnter) {
            count++;
        }
    }
    return count;
}


void 
MSPModel::moveInDirection(SUMOTime currentTime, int dir) {
    for (ActiveLanes::iterator it_lane = myActiveLanes.begin(); it_lane != myActiveLanes.end(); ++it_lane) {
        // move forward
        const MSLane* lane = it_lane->first;
        Pedestrians& pedestrians = it_lane->second;
        const int stripes = numStripes(lane);
        const int waitingToEnter = countWaitingToEnter(pedestrians);
        //std::cout << SIMTIME << ">>> lane=" << lane->getID() << " waitingToEnter=" << waitingToEnter << "\n";
        sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter(dir));
        for (int ii = 0; ii < pedestrians.size(); ++ii) {
            Pedestrian& p = pedestrians[ii];
            if (p.myDir != dir) {
                continue;
            }
            std::vector<SUMOReal> vSafe(stripes, LOOKAHEAD * p.myStage->getSpeed());
            const MSLane* nextLane = getNextLane(lane, p);
            const SUMOReal dist = p.distToLaneEnd();
            assert(dist > 0);
            if (nextLane != 0) {
                // update speeds for next lane
                // XXX consider waitingToEnter on nextLane
                Pedestrians& nextPedestrians = getPedestrians(nextLane);
                sort(nextPedestrians.begin(), nextPedestrians.end(), by_xpos_sorter(dir));
                const SUMOReal relativeX = (dir == FORWARD ? -dist : nextLane->getLength() + dist);
                Pedestrian::updateVSafe(vSafe, 
                        nextPedestrians.end() - MIN2((int)nextPedestrians.size(), 2 * numStripes(nextLane)),
                        nextPedestrians.end(),
                        relativeX, p, p.myNextDir);
                // check link state
                MSLink* link = 0;
                if (dir == FORWARD) {
                    link = MSLinkContHelper::getConnectingLink(*lane, *nextLane);
                } else {
                    if (nextLane->getEdge().isInternal()) {
                        // get the entry link to the junction
                        link = MSLinkContHelper::getConnectingLink(*nextLane->getLogicalPredecessorLane(), *nextLane);
                    } else {
                        // either we are on the junction or there is no internal
                        // lane (network without, length <= POSITION_EPS)
                        if (lane->getEdge().isInternal()) {
                            // get the exit link from the junction
                            link = MSLinkContHelper::getConnectingLink(*lane, *lane->getLinkCont()[0]->getLane());
                        } else {
                            link = MSLinkContHelper::getConnectingLink(*nextLane, *lane);
                        }
                    }
                }
                assert(link != 0);
                if (link->getState() == LINKSTATE_TL_RED) {
                    // prevent movement passed a closed link
                    for (int i = 0; i < (int)vSafe.size(); ++i) {
                        vSafe[i] = MIN2(dist - POSITION_EPS, vSafe[i]);
                    }
                }
                // XXX check for oncoming vehicles with priority
            }
            p.walk(vSafe, pedestrians.begin() + MAX2(0, ii - 2 * stripes - waitingToEnter),
                    pedestrians.begin() + MIN2((int)pedestrians.size(), ii + 2 * stripes + waitingToEnter));
            //std::cout << SIMTIME << p.myPerson->getID() << " lane=" << lane->getID() << " x=" << p.myX << "\n";
            p.updateLocation(lane);
        }
        // advance to the next lane
        const MSEdge& edge = lane->getEdge();
        const MSJunction* junction = (dir == FORWARD ? edge.getToJunction() : edge.getFromJunction());
        sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter(dir));
        bool checkAdvance = true; 
        while (checkAdvance) {
            checkAdvance = false;;
            if (pedestrians.size() > 0) {
                Pedestrian p = pedestrians.front();
                if (p.myDir != dir) {
                    continue;
                }
                if (p.moveToNextLane()) {
                    pedestrians.erase(pedestrians.begin());
                    checkAdvance = true;
                    const MSLane* nextLane = getNextLane(lane, p);
                    const bool nextIsInternal = (nextLane != 0 && nextLane->getEdge().getPurpose() == MSEdge::EDGEFUNCTION_INTERNAL);
                    const SUMOReal done = p.myStage->moveToNextEdge(p.myPerson, currentTime, nextIsInternal ? &nextLane->getEdge() : 0);
                    if (done != 0) {
                        // XXX pedestrian may move again if nextLane is handled after lane
                        addToLane(p, stripes, junction, nextLane);
                    } else {
                        myNumActivePedestrians--;
                    }
                }
            }
        }
    }
}


// ===========================================================================
// MSPModel::Pedestrian method definitions
// ===========================================================================


MSPModel::Pedestrian::Pedestrian(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, const MSLane* lane): 
    myPerson(person),
    myStage(stage),
    myX(stage->getCurrentBeginPos()),
    myY(0), 
    myWaitingToEnter(true)
{ 
    updateDirection(lane);
    if (myDir == FORWARD) {
        // start at the right side of the sidewalk
        myY = STRIPE_WIDTH * (numStripes(lane) - 1);
    }
    updateLocation(lane);
}


void 
MSPModel::Pedestrian::updateDirection(const MSLane* lane, const MSJunction* junction) {
    const MSEdge* next = myStage->getNextEdge();
    if (next == 0) {
        if (lane->getEdge().getToJunction() == junction) {
            assert(!lane->getEdge().isInternal());
            // start at the end of newLane
            myX = lane->getLength() - myX;
        }
        myDir = (myX <= myStage->getCurrentEndPos()) ? 1 : -1;
        myNextDir = 0; // irrelevant
    } else {
        //if (myPerson->getID() == DEBUG1) {
        //    std::cout << SIMTIME << " lane=" << lane->getID() << " next=" << next->getID() << "\n";
        //}
        myDir = getDirection(&lane->getEdge(), next);
        const MSJunction* nextJunction = (myDir == FORWARD ? 
                lane->getEdge().getToJunction() : lane->getEdge().getFromJunction());
        myNextDir = (nextJunction == next->getToJunction() ? -1 : 1);
        if (junction != 0 && myDir == BACKWARD) {
            // start at the end of newLane
            myX = lane->getLength() - myX;
        }
    }
}


SUMOReal 
MSPModel::Pedestrian::getLength() const {
    return myPerson->getVehicleType().getLength();
}


int 
MSPModel::Pedestrian::stripe(int max) const {
    return MIN2(MAX2(0, (int)floor((myY + 0.5 * STRIPE_WIDTH) / STRIPE_WIDTH)), max);
}


int 
MSPModel::Pedestrian::otherStripe(int max) const {
    const SUMOReal offset = myY - stripe(max) * STRIPE_WIDTH;
    int result;
    if (offset > (STRIPE_WIDTH - SQUEEZE *  myPerson->getVehicleType().getWidth())) {
        result = stripe(max) + 1;
    } else if (offset < -(STRIPE_WIDTH -  SQUEEZE * myPerson->getVehicleType().getWidth())) {
        result = stripe(max) - 1;
    } else {
        result = stripe(max);
    }
    return MIN2(MAX2(0, result), max);
}


SUMOReal 
MSPModel::Pedestrian::distToLaneEnd() const {
    if (myDir == FORWARD || myStage->getNextEdge() == 0) {
        return myDir * (myStage->getCurrentEndPos() - myX);
    } else {
        return myX;
    } 
}


bool
MSPModel::Pedestrian::moveToNextLane() {
    const SUMOReal dist = distToLaneEnd();
    //if (myPerson->getID() == DEBUG1) {
    //    std::cout << SIMTIME << " myX=" << myX << " dist=" << dist << "\n";
    //}
    if (dist <= 0) {
        myX = abs(dist); // always add forward and correct in updateDirection()
        return true;
    } else {
        return false;
    }
}


void
MSPModel::Pedestrian::updateVSafe(
        std::vector<SUMOReal>& vSafe,
        Pedestrians::iterator maxLeader, 
        Pedestrians::iterator minFollower, 
        SUMOReal x, 
        const MSPModel::Pedestrian& ego,
        int dir) {

    const int sMax = (int)vSafe.size() - 1;
    for (Pedestrians::iterator it = maxLeader; it != minFollower; ++it) {
        const MSPModel::Pedestrian& ped = *it;
        if (ped.myPerson == ego.myPerson || (!ego.myWaitingToEnter && ped.myWaitingToEnter)) {
            continue; // ignore self and non-inserted
        }
        const SUMOReal gap = (ped.myX - x) * dir;
        if (gap > 0) {
            const SUMOReal v = MAX2((SUMOReal)0, gap - ped.getLength() - ego.myPerson->getVehicleType().getMinGap());
            const SUMOReal penalty = MIN2(v, (ped.myDir == ego.myDir ? 0.0 : ONCOMIN_PENALTY + v * ONCOMIN_PENALTY_FACTOR));
            int l = ped.stripe(sMax);
            // penalty for oncoming pedestrians only applies if ego is not
            // already on the rightmost lane
            int penaltyApplies = ((ego.myDir == FORWARD && l < sMax) || (ego.myDir == BACKWARD && l > 0)) ? 1 : 0;
            vSafe[l] = MIN2(vSafe[l], v - penaltyApplies * penalty);
            l = ped.otherStripe(sMax);
            penaltyApplies = ((ego.myDir == FORWARD && l < sMax) || ego.myDir == BACKWARD && l > 0);
            vSafe[l] = MIN2(vSafe[l], v - penaltyApplies * penalty);
        } else if (-gap < ego.getLength() + ped.myPerson->getVehicleType().getMinGap() 
                && (ego.myWaitingToEnter || ped.stripe(sMax) != ego.stripe(sMax))) {
            // stripes are blocked
            vSafe[ped.stripe(sMax)] = -1;
            vSafe[ped.otherStripe(sMax)] = -1;
        }
    }
    // DEBUG
    if (false && (ego.myPerson->getID() == DEBUG1 || ego.myPerson->getID() == DEBUG2)) {
        std::cout << SIMTIME 
            << " updateVSafe: ego=" << ego.myPerson->getID()
            << " edge=" << ego.myStage->getEdge()->getID()
            << " x=" << x
            << " myX=" << ego.myX
            << " myY=" << ego.myY
            << " dir=" << ego.myDir
            << " s=" << ego.stripe(sMax)
            << " o=" << ego.otherStripe(sMax)
            << " nDir=" << ego.myNextDir
            << " vSafe=" << toString(vSafe) << "\n  ";
        for (Pedestrians::iterator it = maxLeader; it != minFollower; ++it) {
            std::cout 
                << "(" << (*it).myPerson->getID() 
                << " x=" << (*it).myX  
                << " y=" << (*it).myY  
                << " s=" << (*it).stripe(sMax)  
                << " o=" << (*it).otherStripe(sMax)
                << " w=" << (*it).myWaitingToEnter
                << ")   ";
        }
        std::cout << "\n";
    }
}


void 
MSPModel::Pedestrian::walk(std::vector<SUMOReal> vSafe, Pedestrians::iterator maxLeader, Pedestrians::iterator minFollower) {
    const SUMOReal vMax = myStage->getSpeed();
    const int sMax = (int)vSafe.size() - 1;
    // compute vSafe on all stripes
    updateVSafe(vSafe, maxLeader, minFollower, myX, *this, myDir);
    // chose stripe
    int chosen = stripe(sMax);
    // penalize lateral movement
    for (int i = 0; i < (int)vSafe.size(); ++i) {
        vSafe[i] += abs(i - chosen) * LATERAL_PENALTY;
    }
    // forbid a portion of the leftmost stripes (in walking direction). 
    // lanes with stripes less than 1 / RESERVE_FOR_ONCOMING_FACTOR
    // may still deadlock in heavy pedestrian traffic
    const int reserved = (int)floor(vSafe.size() * RESERVE_FOR_ONCOMING_FACTOR);
    if (myDir == FORWARD) {
        for (int i = 0; i < reserved; ++i) {
            vSafe[i] = -1;
        }
    } else {
        for (int i = sMax; i > sMax - reserved; --i) {
            vSafe[i] = -1;
        }
    }
    // select best stripe 
    for (int i = 0; i < (int)vSafe.size(); ++i) {
        if (vSafe[chosen] < vSafe[i]) {
            chosen = i;
        }
    }
    // compute speed components along both axes
    // XXX ensure that diagonal speed <= vMax
    const SUMOReal xSpeed = MAX2(SUMOReal(0), MIN3(vSafe[stripe(sMax)], vSafe[otherStripe(sMax)], vMax));
    const SUMOReal ySteps = MAX2(SUMOReal(1), ceil(vSafe[stripe(sMax)] / vMax));
    const SUMOReal yDist = (chosen * STRIPE_WIDTH) - myY;
    const SUMOReal ySpeed = (yDist > 0 ? 
            MIN2( vMax, yDist / ySteps + NUMERICAL_EPS) :
            MAX2(-vMax, yDist / ySteps + NUMERICAL_EPS));
    // DEBUG
    if (true && (myPerson->getID() == DEBUG1 || myPerson->getID() == DEBUG2)) {
        std::cout << SIMTIME 
            << " ped=" << myPerson->getID()
            << " edge=" << myStage->getEdge()->getID()
            << " x=" << myX
            << " y=" << myY
            << " dir=" << myDir
            << " nDir=" << myNextDir
            << " c=" << chosen
            << " vx=" << xSpeed
            << " vy=" << ySpeed
            << " ys=" << ySteps
            << " yd=" << yDist
            << " vSafe=" << toString(vSafe) << "\n  ";
        for (Pedestrians::iterator it = maxLeader; it != minFollower; ++it) {
            std::cout 
                << "(" << (*it).myPerson->getID() 
                << " x=" << (*it).myX  
                << " y=" << (*it).myY  
                << " s=" << (*it).stripe(sMax)  
                << " o=" << (*it).otherStripe(sMax)
                << " w=" << (*it).myWaitingToEnter
                << ")   ";
        }
        std::cout << "\n";
    }
    myX += xSpeed * myDir;
    myY += ySpeed;
    if (xSpeed > 0) {
        myWaitingToEnter = false;
    }
}


void 
MSPModel::Pedestrian::updateLocation(const MSLane* lane) {
    const SUMOReal shift = myY + (STRIPE_WIDTH - lane->getWidth()) * 0.5;
    myStage->updateLocationSecure(myPerson, lane, myX, shift, myDir);
}


// ===========================================================================
// MSPModel::MovePedestrians method definitions
// ===========================================================================
//
MSPModel::MovePedestrians::MovePedestrians()
{}

SUMOTime 
MSPModel::MovePedestrians::execute(SUMOTime currentTime) {
    MSPModel::moveInDirection(currentTime, 1);
    MSPModel::moveInDirection(currentTime, -1);
    // DEBUG
    if (LOG_ALL) {
        for (ActiveLanes::iterator it_lane = myActiveLanes.begin(); it_lane != myActiveLanes.end(); ++it_lane) {
            const MSLane* lane = it_lane->first;
            Pedestrians& pedestrians = it_lane->second;
            if (pedestrians.size() == 0) {
                continue;
            }
            sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter(FORWARD));
            std::cout << SIMTIME << " lane=" << lane->getID();
            for (int ii = 0; ii < pedestrians.size(); ++ii) {
                Pedestrian& p = pedestrians[ii];
                std::cout << " (" << p.myPerson->getID() << " " << p.myX << "," << p.myY << " " << p.myDir << ")";
            }
            std::cout << "\n";
        }
    }
    return DELTA_T;
}

/****************************************************************************/
