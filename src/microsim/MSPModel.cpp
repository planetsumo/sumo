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
#define DEBUGCOND(PEDID) (PEDID == DEBUG1 || PEDID == DEBUG2)
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
const int MSPModel::UNDEFINED_DIRECTION(0);

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
            //std::cout << SIMTIME << " blocking pedestrian foeLane=" << lane->getID() << " ped=" << ped.myPerson->getID() << " dir=" << ped.myDir << " pX=" << ped.myX << " pL=" << ped.getLength() << " fDTC=" << distToCrossing << " lBD=" << leaderBackDist << "\n";
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
MSPModel::addToLane(Pedestrian& ped, int oldStripes, const MSLane* newLane, int newDir, const PositionVector& walkingAreaShape) {
    if (newLane != 0) {
        if (ped.myPerson->getID() == DEBUG1) {
            std::cout << SIMTIME << " addToLane x=" << ped.myX << " newDir=" << newDir << " newLane=" << newLane->getID() << " walkingAreaShape=" << walkingAreaShape << "\n";
        }
        if (newDir == BACKWARD) {
            const SUMOReal newLength = (walkingAreaShape.size() == 0 ? newLane->getLength() : walkingAreaShape.length());
            ped.myX = newLength - ped.myX;
        }
        // adjust to differences in sidewalk width
        //std::cout << " changing to " << newLane->getID() << " myY=" << ped.myY << " oldStripes=" << oldStripes << " newStripes=" << numStripes(newLane);
        ped.myY += 0.5 * STRIPE_WIDTH * (numStripes(newLane) - oldStripes);
        //std::cout << " newY=" << ped.myY << " myDir=" << ped.myDir << " newDir=" << newDir;
        // adjust to change in direction
        if (ped.myDir != newDir) {
            ped.myY = (numStripes(newLane) - 1) * STRIPE_WIDTH - ped.myY;
        }
        //std::cout << " newY2=" << ped.myY << "\n";
        ped.myDir = newDir;
        //if (ped.myPerson->getID() == DEBUG1) {
        //    std::cout << SIMTIME << " addToLane after update x=" << ped.myX << "\n";
        //}
        ped.updateLocation(newLane, walkingAreaShape);
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

int 
MSPModel::connectedDirection(const MSLane* from, const MSLane* to) {
    if (from == 0 || to == 0) {
        return UNDEFINED_DIRECTION;
    } else if (MSLinkContHelper::getConnectingLink(*from, *to)) {
        return FORWARD;
    } else if (MSLinkContHelper::getConnectingLink(*to, *from)) {
        return BACKWARD;
    } else {
        return UNDEFINED_DIRECTION;
    }
}

const MSLane*
MSPModel::getNextLane(const MSLane* currentLane, const Pedestrian& ped, 
            MSLink*& link, int& nextDir) {
    const MSEdge* currentEdge = &currentLane->getEdge();
    const MSEdge* nextRouteEdge = ped.myStage->getNextEdge();
    const MSLane* nextRouteLane = getSidwalk(nextRouteEdge);
    const MSLane* nextLane = nextRouteLane;
    link = 0;
    nextDir = UNDEFINED_DIRECTION;
    if (nextRouteLane != 0) {
        if (currentEdge->isInternal()) {
            const MSJunction* junction = currentEdge->getToJunction();
            assert(junction == currentEdge->getFromJunction());
            nextDir = junction == nextRouteEdge->getFromJunction() ? FORWARD : BACKWARD;
            if DEBUGCOND(ped.myPerson->getID()) std::cout << "  internal\n";
        } else if (currentEdge->isCrossing()) {
            nextDir = ped.myDir;
            if (ped.myDir == FORWARD) {
                nextLane = currentLane->getLinkCont()[0]->getLane();
            } else {
                nextLane = currentLane->getLogicalPredecessorLane();
            }
            if DEBUGCOND(ped.myPerson->getID()) std::cout << "  crossing\n";
        } else if (currentEdge->isWalkingArea())  {
            // can we go directly to the next route edge?
            nextDir = connectedDirection(currentLane, nextLane);
            if (nextDir != UNDEFINED_DIRECTION) {
                if DEBUGCOND(ped.myPerson->getID()) std::cout << "  walkingArea0\n";
            } else {
                // need to cross a street 
                // check forward crossings
                const MSLinkCont& lc = currentLane->getLinkCont();
                for (MSLinkCont::const_iterator it_l = lc.begin(); it_l != lc.end(); it_l++) {
                    MSLane* cand = (*it_l)->getLane();
                    if (cand->getEdge().isCrossing()) {
                        MSLane* walkingArea = cand->getLinkCont()[0]->getLane();
                        if (connectedDirection(walkingArea, nextRouteLane) != UNDEFINED_DIRECTION) {
                            nextLane = cand;
                            nextDir = FORWARD;
                            link = *it_l;
                            if DEBUGCOND(ped.myPerson->getID()) std::cout << "  walkingArea1\n";
                            break;
                        }
                    }
                }
                if (nextDir == UNDEFINED_DIRECTION) {
                    // check backward crossings
                    const std::vector<MSEdge*>& pred = currentEdge->getIncomingEdges();
                    for (std::vector<MSEdge*>::const_iterator it_e = pred.begin(); it_e != pred.end(); it_e++) {
                        if ((*it_e)->isCrossing()) {
                            MSLane* cand = (*it_e)->getLanes()[0];
                            MSLane* walkingArea = cand->getLogicalPredecessorLane();
                            if (connectedDirection(walkingArea, nextRouteLane) != UNDEFINED_DIRECTION) {
                                nextLane = cand;
                                nextDir = BACKWARD;
                                link = MSLinkContHelper::getConnectingLink(*walkingArea, *cand);
                                if DEBUGCOND(ped.myPerson->getID()) std::cout << "  walkingArea2\n";
                                break;
                            }
                        }
                    }
                }
                if (nextDir == UNDEFINED_DIRECTION) {
                    nextLane = nextRouteLane;
                    // resolve direction when the laneAfterNext is known
                    if DEBUGCOND(ped.myPerson->getID()) std::cout << "  skipping forward on walkingArea";
                }
            }
        } else  {
            // normal edge. continue with the next walking area or use a direct connection
            nextDir = ped.myDir;
            if (ped.myDir == FORWARD) {
                link = MSLinkContHelper::getConnectingLink(*currentLane, *nextLane);
                if (link != 0) {
                    // simple forward case. there is a natural edge to be followed
                    nextLane = MSLinkContHelper::getInternalFollowingLane(currentLane, nextRouteLane);
                    if DEBUGCOND(ped.myPerson->getID()) std::cout << "  direct forward\n";
                } else {
                    // go forward to the next walking area
                    if (currentLane->getLinkCont().size() == 1) {
                        const MSLane* cand = currentLane->getLinkCont()[0]->getLane();
                        if (cand->getEdge().isWalkingArea()) {
                            if DEBUGCOND(ped.myPerson->getID()) std::cout << "  forward to walkingArea\n";
                            nextLane = cand;
                        } else {
                            if DEBUGCOND(ped.myPerson->getID()) std::cout << "  skipping forward\n";
                            nextLane = nextRouteLane;
                        }
                    } else {
                        if DEBUGCOND(ped.myPerson->getID()) std::cout << "  forward failure: links=" << currentLane->getLinkCont().size() << "\n";
                    }
                }
            } else {
                link = MSLinkContHelper::getConnectingLink(*nextRouteLane, *currentLane);
                if (link != 0) {
                    // simple backward case. there is a natural edge to be followed
                    nextLane = MSLinkContHelper::getInternalFollowingLane(nextRouteLane, currentLane);
                    if DEBUGCOND(ped.myPerson->getID()) std::cout << "  direct backward\n";
                } else {
                    // go backward to the next walking area
                    if (currentLane->getLogicalPredecessorLane() != 0) {
                        const MSLane* cand = currentLane->getLogicalPredecessorLane();
                        assert(nextLane->getEdge().isWalkingArea());
                        if (cand->getEdge().isWalkingArea()) {
                            if DEBUGCOND(ped.myPerson->getID()) std::cout << "  backward to walkingArea\n";
                            nextLane = cand;
                        } else {
                            if DEBUGCOND(ped.myPerson->getID()) std::cout << "  skipping backward\n";
                            nextLane = nextRouteLane;
                        }
                    } else {
                        if DEBUGCOND(ped.myPerson->getID()) std::cout << "  backward failure: links=" << currentLane->getLinkCont().size() << "\n";
                    }
                }
            }
            if (nextLane == 0 || nextLane->getLength() <= POSITION_EPS) {
                // no internal lane found or it's too short, jump directly to next route lane
                nextLane = nextRouteLane;
                link = 0;
            }
        }
    }
    if DEBUGCOND(ped.myPerson->getID()) {
        std::cout << SIMTIME 
            << " p=" << ped.myPerson->getID() 
            << " l=" << currentLane->getID() 
            << " nl=" << (nextLane == 0 ? "NULL" :  nextLane->getID()) 
            << " d=" << nextDir 
            << " pedDir=" << ped.myDir 
            << "\n";
    }
    return nextLane;
}


PositionVector
MSPModel::getWalkingAreaShape(const MSLane* from, const MSLane* walkingArea, int walkingAreaDir,  const Pedestrian& ped) {
    if (walkingArea != 0 && walkingArea->getEdge().isWalkingArea()) {
        PositionVector result;
        const MSEdge* nextRouteEdge = ped.myStage->getNextEdge();
        const MSLane* nextRouteLane = getSidwalk(nextRouteEdge);
        result.push_back(ped.myDir == FORWARD ? from->getShape().back() : from->getShape().front());
        PositionVector fromShp = from->getShape();
        fromShp.extrapolate(walkingArea->getWidth() / 2);
        result.push_back(ped.myDir == FORWARD ? fromShp.back() : fromShp.front());
        MSLink* linkDummy;
        int nextDir;
        const MSLane* nextLane = getNextLane(walkingArea, ped, linkDummy, nextDir);
        PositionVector nextShp = nextLane->getShape();
        nextShp.extrapolate(walkingArea->getWidth() / 2);
        result.push_back(nextDir == FORWARD ? nextShp.front() : nextShp.back());
        result.push_back(nextDir == FORWARD ? nextLane->getShape().front() : nextLane->getShape().back());
        return (walkingAreaDir == FORWARD ? result : result.reverse());
    } else {
        return PositionVector();
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
            int nextDir;
            MSLink* link;
            const MSLane* nextLane = getNextLane(lane, p, link, nextDir);
            const SUMOReal dist = p.distToLaneEnd();
            assert(dist > 0);
            if (nextLane != 0) {
                // update speeds for next lane
                // XXX consider waitingToEnter on nextLane
                Pedestrians& nextPedestrians = getPedestrians(nextLane);
                sort(nextPedestrians.begin(), nextPedestrians.end(), by_xpos_sorter(dir));
                SUMOReal relativeX = -dist;
                if (dir == BACKWARD) {
                    const PositionVector walkingAreaShape = getWalkingAreaShape(lane, nextLane, nextDir, p);
                    const SUMOReal nextLength = (walkingAreaShape.size() == 0 ? nextLane->getLength() : walkingAreaShape.length());
                    relativeX = nextLength + dist;
                }
                Pedestrian::updateVSafe(vSafe, 
                        nextPedestrians.end() - MIN2((int)nextPedestrians.size(), 2 * numStripes(nextLane)),
                        nextPedestrians.end(),
                        relativeX, p, nextDir);
                // check link state
                if (link != 0 && link->getState() == LINKSTATE_TL_RED) {
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
                    int nextDir;
                    MSLink* dummyLink;
                    const MSLane* nextLane = getNextLane(lane, p, dummyLink, nextDir);
                    const bool nextIsNormal = (nextLane == 0 || nextLane->getEdge().getPurpose() == MSEdge::EDGEFUNCTION_NORMAL);
                    const SUMOReal done = p.myStage->moveToNextEdge(p.myPerson, currentTime, nextIsNormal ? 0 : &nextLane->getEdge());
                    if (done != 0) {
                        if (nextDir == UNDEFINED_DIRECTION) {
                            const MSLane* laneAfterNext = getSidwalk(p.myStage->getNextEdge());
                            nextDir == connectedDirection(nextLane, laneAfterNext);
                            if (nextDir == UNDEFINED_DIRECTION) {
                                nextDir = FORWARD;
                            }
                        }
                        // XXX pedestrian may move again if nextLane is handled after lane
                        addToLane(p, stripes, nextLane, nextDir, getWalkingAreaShape(lane, nextLane, nextDir, p));
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
    myDir(FORWARD),
    myWaitingToEnter(true)
{ 
    const MSEdge* nextRouteEdge = myStage->getNextEdge();
    if (nextRouteEdge != 0) {
        // initialize myDir based on junction topology
        if (lane->getEdge().getFromJunction() == nextRouteEdge->getToJunction()
                || lane->getEdge().getFromJunction() == nextRouteEdge->getFromJunction()) {
            if (lane->getEdge().getToJunction() == nextRouteEdge->getToJunction()
                    || lane->getEdge().getToJunction() == nextRouteEdge->getFromJunction()) {
                // the connectivity is ambiguous
                // walk to the nearest junction XXX this is a hack and there may be no connection there
                myDir = (myX >= lane->getLength() / 2) ? FORWARD : BACKWARD;
            } else {
                myDir = BACKWARD;
            }
        } else if (lane->getEdge().getToJunction() != nextRouteEdge->getToJunction()
                && lane->getEdge().getFromJunction() != nextRouteEdge->getFromJunction()) {
            // there is no connectivity, walk forward by default
        }
    } else {
        myDir = (myX <= myStage->getCurrentEndPos()) ? 1 : -1;
    }
    if (myDir == FORWARD) {
        // start at the right side of the sidewalk
        myY = STRIPE_WIDTH * (numStripes(lane) - 1);
    }
    if DEBUGCOND(myPerson->getID()) std::cout << "  added new pedestrian " << myPerson->getID() << " on " << lane->getID() << " myX=" << myX << " myY=" << myY << " dir=" << myDir << " nextRouteEdge=" << (nextRouteEdge == 0 ? "NULL" : nextRouteEdge->getID()) << "\n";

    updateLocation(lane);
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
        myX = -dist; // always keep forward offset and correct in addToLane()
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
                << " dir=" << (*it).myDir
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
MSPModel::Pedestrian::updateLocation(const MSLane* lane, const PositionVector& walkingAreaShape) {
    const SUMOReal shift = myY + (STRIPE_WIDTH - lane->getWidth()) * 0.5;
    myStage->updateLocationSecure(myPerson, lane, myX, shift, myDir, walkingAreaShape);
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
