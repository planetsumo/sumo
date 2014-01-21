/****************************************************************************/
/// @file    MSPModel.h
/// @author  Jakob Erdmann
/// @date    Mon, 13 Jan 2014
/// @version $Id: MSPModel.h 14658 2013-09-10 15:31:59Z namdre $
///
// The pedestrian following model (prototype)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2001-2013 DLR (http://www.dlr.de/) and contributors
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
#include "MSPModel.h"


// ===========================================================================
// static members
// ===========================================================================
int MSPModel::myNumActivePedestrians(0);

MSPModel::ActiveLanes MSPModel::myActiveLanes;

MSPModel::Pedestrians MSPModel::noPedestrians;

const SUMOReal MSPModel::SAFETY_GAP(2.0);
const SUMOReal MSPModel::DEFAULT_SIDEWALK_WIDTH(3.0);
const SUMOReal MSPModel::STRIPE_WIDTH(0.75);
const SUMOReal MSPModel::LOOKAHEAD(5.0);
const SUMOReal MSPModel::SQUEEZE(0.5);

// ===========================================================================
// MSPModel method definitions
// ===========================================================================

void
MSPModel::add(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, MSNet* net) {
    assert(person->getCurrentStageType() == MSPerson::WALKING);
    //MSEdge* edge = dynamic_cast<MSPerson::MSPersonStage_Walking*>(person->getCurrentStage());
    const MSLane* lane = getSidwalk(person->getEdge());
    myActiveLanes[lane].push_back(Pedestrian(person, stage));
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
        const SUMOReal leaderBack = ped.myX - ped.getLength() - MSPModel::SAFETY_GAP; 
        const SUMOReal leaderBackDist = distToCrossing - leaderBack;
        //std::cout << SIMTIME << " foe=" << foeLane->getID() << " pX=" << ped.myX << " pL=" << ped.getLength() << " fDTC=" << foeDistToCrossing << " lBD=" << leaderBackDist << "\n";
        if (leaderBackDist >= 0) {
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
MSPModel::addToLane(Pedestrian ped, const MSLane* newLane) {
    if (newLane != 0) {
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
MSPModel::getNextLane(const MSLane* currentLane, Pedestrian& ped) {
    const MSLane* nextRouteLane = getSidwalk(ped.myStage->getNextEdge());
    const MSLane* nextLane = nextRouteLane;
    if (nextRouteLane != 0) {
        // route continues, check for internal edge in between
        nextLane = MSLinkContHelper::getInternalFollowingLane(currentLane, nextRouteLane);
        if (nextLane == 0 || nextLane->getLength() <= POSITION_EPS) {
            // no internal lane found or it's too short, jump directly to next route lane
            nextLane = nextRouteLane;
        }
    }
    return nextLane;
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


// ===========================================================================
// MSPModel::Pedestrian method definitions
// ===========================================================================


MSPModel::Pedestrian::Pedestrian(MSPerson* person, MSPerson::MSPersonStage_Walking* stage): 
    myPerson(person),
    myStage(stage),
    myX(stage->getCurrentBeginPos()),
    myY(0), // XXX randomize?
    myWaitingToEnter(true)
{ }


SUMOReal 
MSPModel::Pedestrian::getLength() const {
    return myPerson->getVehicleType().getLength();
}


int 
MSPModel::Pedestrian::stripe() const {
    return (int)floor((myY + 0.5 * STRIPE_WIDTH) / STRIPE_WIDTH);
}


int 
MSPModel::Pedestrian::otherStripe() const {
    const SUMOReal offset = myY - stripe() * STRIPE_WIDTH;
    if (offset > (STRIPE_WIDTH - SQUEEZE *  myPerson->getVehicleType().getWidth())) {
        return stripe() + 1;
    } else if (offset < -(STRIPE_WIDTH -  SQUEEZE * myPerson->getVehicleType().getWidth())) {
        return stripe() - 1;
    } else {
        return stripe();
    }
}


void
MSPModel::Pedestrian::updateVSafe(
        std::vector<SUMOReal>& vSafe,
        Pedestrians::iterator maxLeader, 
        Pedestrians::iterator minFollower, 
        SUMOReal x, const MSPModel::Pedestrian& ego) {

    for (Pedestrians::iterator it = maxLeader; it != minFollower; ++it) {
        const MSPModel::Pedestrian& ped = *it;
        if (ped.myPerson == ego.myPerson || (!ego.myWaitingToEnter && ped.myWaitingToEnter)) {
            continue; // ignore self and non-inserted
        }
        if (ped.myX > x) {
            const SUMOReal v = MAX2((SUMOReal)0, ped.myX - x - ped.myPerson->getVehicleType().getLength() - ego.myPerson->getVehicleType().getMinGap());
            int l = ped.stripe();
            vSafe[l] = MIN2(vSafe[l], v);
            l = ped.otherStripe();
            vSafe[l] = MIN2(vSafe[l], v);
        } else if (ped.myX + ego.myPerson->getVehicleType().getLength() + ped.myPerson->getVehicleType().getMinGap() > x 
                && (ego.myWaitingToEnter || ped.stripe() != ego.stripe())) {
            // stripes are blocked
            vSafe[ped.stripe()] = -1;
            vSafe[ped.otherStripe()] = -1;
        }
    }
}


void 
MSPModel::Pedestrian::walk(std::vector<SUMOReal> vSafe, Pedestrians::iterator maxLeader, Pedestrians::iterator minFollower) {
    const SUMOReal vMax = myStage->getSpeed();
    // compute vSafe on all stripes
    updateVSafe(vSafe, maxLeader, minFollower, myX, *this);
    // chose stripe
    int chosen = stripe();
    // disallow stripes which are to far away
    for (int i = 0; i < (int)vSafe.size(); ++i) {
        if (abs(i - chosen) > 1) {
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
    const SUMOReal xSpeed = MAX2(SUMOReal(0), MIN3(vSafe[stripe()], vSafe[otherStripe()], vMax));
    const SUMOReal ySteps = MAX2(SUMOReal(1), ceil(vSafe[stripe()] / vMax));
    const SUMOReal yDist = (chosen * STRIPE_WIDTH) - myY;
    const SUMOReal ySpeed = (yDist > 0 ? 
            MIN2( vMax, yDist / ySteps + NUMERICAL_EPS) :
            MAX2(-vMax, yDist / ySteps + NUMERICAL_EPS));
    // DEBUG
    if (false && (myPerson->getID() == "p20" || myPerson->getID() == "p17")) {
        std::cout << SIMTIME 
            << " ped=" << myPerson->getID()
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
                << " s=" << (*it).stripe()  
                << " o=" << (*it).otherStripe()
                << " w=" << (*it).myWaitingToEnter
                << ")   ";
        }
        std::cout << "\n";
    }
    myX += xSpeed;
    myY += ySpeed;
    if (xSpeed > 0) {
        myWaitingToEnter = false;
    }
}

// ===========================================================================
// MSPModel::MovePedestrians method definitions
// ===========================================================================
//
MSPModel::MovePedestrians::MovePedestrians()
{}

SUMOTime 
MSPModel::MovePedestrians::execute(SUMOTime currentTime) {
    for (ActiveLanes::iterator it_lane = myActiveLanes.begin(); it_lane != myActiveLanes.end(); ++it_lane) {
        // move forward
        Pedestrians& pedestrians = it_lane->second;
        const int stripes = numStripes(it_lane->first);
        const int waitingToEnter = countWaitingToEnter(pedestrians);
        //std::cout << SIMTIME << ">>> lane=" << it_lane->first->getID() << " waitingToEnter=" << waitingToEnter << "\n";
        sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter());
        for (int ii = 0; ii < pedestrians.size(); ++ii) {
            Pedestrian& p = pedestrians[ii];
            std::vector<SUMOReal> vSafe(stripes, LOOKAHEAD);
            const MSLane* nextLane = getNextLane(it_lane->first, p);
            const SUMOReal dist = p.myStage->getCurrentEndPos() - p.myX; 
            if (nextLane != 0) {
                MSLink* link = MSLinkContHelper::getConnectingLink(*it_lane->first, *nextLane);
                assert(link != 0);
                if (link->getState() == LINKSTATE_TL_RED) {
                    // prevent movement passed a closed link
                    // XXX check for oncoming vehicles with priority
                    vSafe = std::vector<SUMOReal>(stripes, dist);
                } else {
                    // initialize vSafes according to link state and pedestrians on the next Lane
                    // XXX consider waitingToEnter on nextLane
                    Pedestrians& nextPedestrians = getPedestrians(nextLane);
                    sort(nextPedestrians.begin(), nextPedestrians.end(), by_xpos_sorter());
                    Pedestrian::updateVSafe(vSafe, 
                            nextPedestrians.end() - MIN2((int)nextPedestrians.size(), 2 * stripes),
                            nextPedestrians.end(),
                            -dist, p);
                }
            }
            p.walk(vSafe, pedestrians.begin() + MAX2(0, ii - 2 * stripes - waitingToEnter),
                    pedestrians.begin() + MIN2((int)pedestrians.size(), ii + 2 * stripes + waitingToEnter));
            //std::cout << SIMTIME << p.myPerson->getID() << " lane=" << it_lane->first->getID() << " x=" << p.myX << "\n";
            p.myStage->updateLocationSecure(p.myPerson, it_lane->first, p.myX, p.myY);
        }
        // advance to the next lane
        sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter());
        bool checkAdvance = true; 
        while (checkAdvance) {
            checkAdvance = false;;
            if (pedestrians.size() > 0) {
                Pedestrian p = pedestrians.front();
                if (p.myX > p.myStage->getCurrentEndPos()) {
                    p.myX -= p.myStage->getCurrentEndPos();
                    pedestrians.erase(pedestrians.begin());
                    checkAdvance = true;
                    const MSLane* nextLane = getNextLane(it_lane->first, p);
                    const bool nextIsInternal = (nextLane != 0 && nextLane->getEdge().getPurpose() == MSEdge::EDGEFUNCTION_INTERNAL);
                    const SUMOReal done = p.myStage->moveToNextEdge(p.myPerson, currentTime, nextIsInternal ? &nextLane->getEdge() : 0);
                    if (done != 0) {
                        p.myStage->updateLocationSecure(p.myPerson, nextLane, p.myX, p.myY);
                        //std::cout << SIMTIME << p.myPerson->getID() << " lane=" << nextLane->getID() << " x=" << p.myX << "\n";
                        addToLane(p, nextLane);
                    } else {
                        myNumActivePedestrians--;
                    }
                }
            }
        }
    }
    return DELTA_T;
}

/****************************************************************************/
