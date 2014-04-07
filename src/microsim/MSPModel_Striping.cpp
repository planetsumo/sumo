/****************************************************************************/
/// @file    MSPModel_Striping.h
/// @author  Jakob Erdmann
/// @date    Mon, 13 Jan 2014
/// @version $Id: MSPModel_Striping.cpp 16023 2014-03-25 10:36:03Z namdre $
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
#include "MSPModel_Striping.h"


// ===========================================================================
// DEBUGGING HELPERS
// ===========================================================================
//
#define DEBUG1 "disabled"
#define DEBUG2 "disabled"
#define DEBUGCOND(PEDID) (PEDID == DEBUG1 || PEDID == DEBUG2)
#define LOG_ALL false

void MSPModel_Striping::DEBUG_PRINT(const Obstacles& obs) {
    for (int i = 0; i < (int)obs.size(); ++i) {
        std::cout 
            << "(" << obs[i].description
            << " x=" << obs[i].x  
            << " s=" << obs[i].speed
            << ")   ";
    }
    std::cout << "\n";
}

// ===========================================================================
// named (internal) constants
// ===========================================================================


// need to be able to subtract without underflow
#define BLOCKED_STRIPE -10000
#define FAR_AWAY 10000
#define ONCOMING_CONFLICT -1000

// ===========================================================================
// static members
// ===========================================================================

MSPModel_Striping::WalkingAreaPaths MSPModel_Striping::myWalkingAreaPaths;
MSPModel_Striping::Pedestrians MSPModel_Striping::noPedestrians;


// model parameters (static to simplify access from class Pedestrian
SUMOReal MSPModel_Striping::STRIPE_WIDTH;
SUMOReal MSPModel_Striping::DAWDLING;
const SUMOReal MSPModel_Striping::LOOKAHEAD_SAMEDIR(4.0); // seconds
const SUMOReal MSPModel_Striping::LOOKAHEAD_ONCOMING(10.0); // seconds
const SUMOReal MSPModel_Striping::LATERAL_PENALTY(-1);
const SUMOReal MSPModel_Striping::SQUEEZE(0.7);
const SUMOReal MSPModel_Striping::BLOCKER_LOOKAHEAD(10.0);
const SUMOReal MSPModel_Striping::RESERVE_FOR_ONCOMING_FACTOR(0.0);
const SUMOReal MSPModel_Striping::MAX_WAIT_TOLERANCE(120);
const SUMOReal MSPModel_Striping::LATERAL_SPEED_FACTOR(0.4);


// ===========================================================================
// MSPModel_Striping method definitions
// ===========================================================================

MSPModel_Striping::MSPModel_Striping(const OptionsCont& oc, MSNet* net) :
    myNumActivePedestrians(0)
{
    myCommand = new MovePedestrians(this);
    net->getBeginOfTimestepEvents().addEvent(myCommand, net->getCurrentTimeStep() + DELTA_T, MSEventControl::ADAPT_AFTER_EXECUTION);
    initWalkingAreaPaths(net);
    // configurable parameters
    STRIPE_WIDTH = oc.getFloat("pedestrian.stripe-width");
    DAWDLING = oc.getFloat("pedestrian.dawdling");
}


MSPModel_Striping::~MSPModel_Striping() {
}


PedestrianState*
MSPModel_Striping::add(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, SUMOTime) {
    assert(person->getCurrentStageType() == MSPerson::WALKING);
    const MSLane* lane = getSidewalk(person->getEdge());
    Pedestrian* ped = new Pedestrian(person, stage, lane);
    myActiveLanes[lane].push_back(ped);
    myNumActivePedestrians++;
    return ped;
}


bool 
MSPModel_Striping::blockedAtDist(const MSLane* lane, SUMOReal distToCrossing, std::vector<const MSPerson*>* collectBlockers) {
    const Pedestrians& pedestrians = getPedestrians(lane);
    for (Pedestrians::const_iterator it_ped = pedestrians.begin(); it_ped != pedestrians.end(); ++it_ped) {
        const Pedestrian& ped = **it_ped;
        const SUMOReal leaderBackDist = (ped.myDir == FORWARD 
                ? distToCrossing - (ped.myX - ped.getLength() - MSPModel_Striping::SAFETY_GAP)
                : (ped.myX + ped.getLength() + MSPModel_Striping::SAFETY_GAP) - distToCrossing); 
        //std::cout << SIMTIME << " foe=" << foeLane->getID() << " dir=" << p.myDir << " pX=" << ped.myX << " pL=" << ped.getLength() << " fDTC=" << distToCrossing << " lBD=" << leaderBackDist << "\n";
        if (leaderBackDist >= 0 && leaderBackDist <= BLOCKER_LOOKAHEAD) {
            // found one pedestrian that is not completely past the crossing point
            //std::cout << SIMTIME << " blocking pedestrian foeLane=" << lane->getID() << " ped=" << ped.myPerson->getID() << " dir=" << ped.myDir << " pX=" << ped.myX << " pL=" << ped.getLength() << " fDTC=" << distToCrossing << " lBD=" << leaderBackDist << "\n";
            if (collectBlockers == 0) {
                return true;
            } else {
                collectBlockers->push_back(ped.myPerson);
            }
        }
    }
    if (collectBlockers == 0) {
        return false;
    } else {
        return collectBlockers->size() > 0;
    }
}


MSPModel_Striping::Pedestrians& 
MSPModel_Striping::getPedestrians(const MSLane* lane) {
    ActiveLanes::iterator it = myActiveLanes.find(lane);
    if (it != myActiveLanes.end()) {
        //std::cout << " found lane=" << lane->getID() << " n=" << it->second.size() << "\n";
        return (it->second);
    } else {
        return noPedestrians;
    }
}


void
MSPModel_Striping::cleanupHelper() {
    myActiveLanes.clear();
    myNumActivePedestrians = 0;
}


int 
MSPModel_Striping::numStripes(const MSLane* lane) {
    return (int)floor(lane->getWidth() / STRIPE_WIDTH); 
}

int 
MSPModel_Striping::connectedDirection(const MSLane* from, const MSLane* to) {
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


void 
MSPModel_Striping::initWalkingAreaPaths(const MSNet* net) {
    if (myWalkingAreaPaths.size() > 0) {
        return;
    }
    for (size_t i = 0; i < MSEdge::dictSize(); ++i) {
        const MSEdge* edge = MSEdge::dictionary(i);
        if (edge->isWalkingArea()) {
            const MSLane* walkingArea = getSidewalk(edge);
            // build all possible paths across this walkingArea
            const MSJunction* junction = edge->getFromJunction();
            assert(junction == edge->getToJunction());
            // gather all incident lanes
            std::vector<MSLane*> lanes;
            const std::vector<MSEdge*>& incoming = edge->getIncomingEdges();
            for (int j = 0; j < (int)incoming.size(); ++j) {
                lanes.push_back(getSidewalk(incoming[j]));
            }
            for (int j = 0; j < (int)edge->getNoFollowing(); ++j) {
                lanes.push_back(getSidewalk(edge->getFollower(j)));
            }
            // build all combinations
            for (int j = 0; j < (int)lanes.size(); ++j) {
                for (int k = 0; k < (int)lanes.size(); ++k) {
                    if (j != k) {
                        // build the walkingArea
                        const MSLane* from = lanes[j];
                        const MSLane* to = lanes[k];
                        const int fromDir = MSLinkContHelper::getConnectingLink(*from, *walkingArea) != 0 ? FORWARD : BACKWARD;
                        const int toDir = MSLinkContHelper::getConnectingLink(*walkingArea, *to) != 0 ? FORWARD : BACKWARD;
                        PositionVector shape;
                        Position fromPos = from->getShape()[fromDir == FORWARD ? -1 : 0];
                        Position toPos = to->getShape()[toDir == FORWARD ? 0 : -1];
                        const SUMOReal maxExtent = fromPos.distanceTo2D(toPos) / 4; // prevent sharp corners
                        const SUMOReal extrapolateBy = MIN2(maxExtent, walkingArea->getWidth() / 2);
                        // assemble shape
                        shape.push_back(fromPos);
                        if (extrapolateBy > POSITION_EPS) {
                            PositionVector fromShp = from->getShape();
                            fromShp.extrapolate(extrapolateBy);
                            shape.push_back(fromDir == FORWARD ? fromShp.back() : fromShp.front());
                            PositionVector nextShp = to->getShape();
                            nextShp.extrapolate(extrapolateBy);
                            shape.push_back(toDir == FORWARD ? nextShp.front() : nextShp.back());
                        }
                        shape.push_back(toPos);
                        if (fromDir == BACKWARD) {
                            // will be walking backward on walkingArea
                            shape = shape.reverse();
                        }
                        myWalkingAreaPaths[std::make_pair(from, to)] = WalkingAreaPath(from, walkingArea, to, shape);
                    }
                }
            }
        }
    }
}


MSPModel_Striping::NextLaneInfo 
MSPModel_Striping::getNextLane(const Pedestrian& ped, const MSLane* currentLane, const MSLane* prevLane) {
    const MSEdge* currentEdge = &currentLane->getEdge();
    const MSJunction* junction = ped.myDir == FORWARD ? currentEdge->getToJunction() : currentEdge->getFromJunction();
    const MSEdge* nextRouteEdge = ped.myStage->getNextRouteEdge();
    const MSLane* nextRouteLane = getSidewalk(nextRouteEdge);
    // result values
    const MSLane* nextLane = nextRouteLane;
    MSLink* link = 0;
    int nextDir = UNDEFINED_DIRECTION;

   if (nextRouteLane != 0) {
       if (currentEdge->isInternal()) {
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
           std::vector<const MSEdge*> crossingRoute;
           // departPos can be 0 because the direction of the walkingArea does not matter
           // for the arrivalPos, we need to make sure that the route does not deviate across other junctions
           const int nextRouteEdgeDir = nextRouteEdge->getFromJunction() == junction ? FORWARD : BACKWARD;
           const SUMOReal arrivalPos = (nextRouteEdge == ped.myStage->getRoute().back() 
                   ? ped.myStage->getArrivalPos() 
                   : (nextRouteEdgeDir == FORWARD ? 0 : nextRouteEdge->getLength()));
           MSNet::getInstance()->getPedestrianRouter().compute(currentEdge, nextRouteEdge, 0, arrivalPos, ped.myStage->getMaxSpeed(), 0, junction, crossingRoute, true);
           if DEBUGCOND(ped.myPerson->getID()) { 
               std::cout 
                   << "   nreDir=" << nextRouteEdgeDir
                   << "   aPos=" << arrivalPos
                   << " crossingRoute=" << toString(crossingRoute) 
                   << "\n";
           }
           if (crossingRoute.size() > 1) {
               const MSEdge* nextEdge = crossingRoute[1];
               nextLane = getSidewalk(crossingRoute[1]);
               // need to check that the route actually goes across the current junction
               // XXX limit search depth to avoid search the whole network in case of failure?
               if ((nextEdge->getFromJunction() == junction || nextEdge->getToJunction() == junction)
                       && nextLane != prevLane) {
                   nextDir = connectedDirection(currentLane, nextLane);
                   if DEBUGCOND(ped.myPerson->getID()) std::cout << " nextDir=" << nextDir << "\n";
                   assert(nextDir != UNDEFINED_DIRECTION);
                   if (nextDir == FORWARD) {
                       link = MSLinkContHelper::getConnectingLink(*currentLane, *nextLane);
                   } else if (nextEdge->isCrossing()) {
                       const MSLane* oppositeWalkingArea = nextLane->getLogicalPredecessorLane();
                       link = MSLinkContHelper::getConnectingLink(*oppositeWalkingArea, *nextLane);
                   } else {
                       link = MSLinkContHelper::getConnectingLink(*nextLane, *currentLane);
                   }
                   assert(link != 0);
               } else {
                   WRITE_WARNING("Could not find route across junction from '" + (currentEdge == 0 ? "NULL" : currentEdge->getID())
                           + "' to '" + (nextRouteEdge == 0 ? "NULL" : nextRouteEdge->getID())
                           + "' for pedestrian '" + ped.myPerson->getID() + "' (found only detour using " + nextLane->getID() + ").");
               }
           } else {
               if DEBUGCOND(ped.myPerson->getID()) {
                   std::cout << SIMTIME
                       << " no route from '" << (currentEdge == 0 ? "NULL" : currentEdge->getID())
                       << "' to '" << (nextRouteEdge == 0 ? "NULL" : nextRouteEdge->getID())
                       << "\n";
               }
               nextDir = FORWARD; // fallback
           }
       } else {
           // normal edge. by default use next / previous walking area
           nextDir = ped.myDir;
           nextLane = getNextWalkingArea(currentLane, ped.myDir, link);
           if (nextLane != 0) {
               // walking area found
               if DEBUGCOND(ped.myPerson->getID()) std::cout << "  next walkingArea " << (nextDir == FORWARD ? "forward" : "backward") << "\n";
           } else {
               // try to use a direct link as fallback
               // direct links only exist if built explicitly. They are used to model tl-controlled links if there are no crossings
               if (ped.myDir == FORWARD) {
                   link = MSLinkContHelper::getConnectingLink(*currentLane, *nextRouteLane);
                   if (link != 0) {
                       if DEBUGCOND(ped.myPerson->getID()) std::cout << "  direct forward\n";
                       nextLane = MSLinkContHelper::getInternalFollowingLane(currentLane, nextRouteLane);
                   }
               } else {
                   link = MSLinkContHelper::getConnectingLink(*nextRouteLane, *currentLane);
                   if (link != 0) {
                       if DEBUGCOND(ped.myPerson->getID()) std::cout << "  direct backward\n";
                       nextLane = MSLinkContHelper::getInternalFollowingLane(nextRouteLane, currentLane);
                   }
               }
           }
           if (nextLane == 0) {
               // no internal lane found
               nextLane = nextRouteLane;
               if DEBUGCOND(ped.myPerson->getID()) std::cout << SIMTIME << " no next lane found for " << currentLane->getID() << " dir=" << ped.myDir << "\n";
           } else if (nextLane->getLength() <= POSITION_EPS) {
               // internal lane too short
               nextLane = nextRouteLane;
           }
       }
   }
   if DEBUGCOND(ped.myPerson->getID()) {
       std::cout << SIMTIME
           << " p=" << ped.myPerson->getID()
           << " l=" << currentLane->getID()
           << " nl=" << (nextLane == 0 ? "NULL" : nextLane->getID())
           << " nrl=" << (nextRouteLane == 0 ? "NULL" : nextRouteLane->getID())
           << " d=" << nextDir
           << " link=" << (link == 0 ? "NULL" : link->getViaLaneOrLane()->getID())
           << " pedDir=" << ped.myDir
           << "\n";
   }
    return NextLaneInfo(nextLane, link, nextDir);
}


const MSLane*
MSPModel_Striping::getNextWalkingArea(const MSLane* currentLane, const int dir, MSLink*& link) {
    if (dir == FORWARD) {
        const MSLinkCont& links = currentLane->getLinkCont();
        for (MSLinkCont::const_iterator it = links.begin(); it != links.end(); ++it) {
            if ((*it)->getLane()->getEdge().isWalkingArea()) {
                link = *it;
                return (*it)->getLane();
            }
        }
    } else {
        const std::vector<MSLane::IncomingLaneInfo>& laneInfos = currentLane->getIncomingLanes();
        for (std::vector<MSLane::IncomingLaneInfo>::const_iterator it = laneInfos.begin(); it != laneInfos.end(); ++it) {
            if ((*it).lane->getEdge().isWalkingArea()) {
                link = (*it).viaLink;
                return (*it).lane;
            }
        }
    }
    return 0;
}


MSPModel_Striping::Obstacles
MSPModel_Striping::mergeObstacles(const Obstacles& obs1, const Obstacles& obs2, int dir) {
    Obstacles result(obs1.begin(), obs1.end());
    for (int i = 0; i < (int)obs1.size(); ++i) {
        if ((obs2[i].x - obs1[i].x) * dir < 0) {
            result[i] = obs2[i];
        }
    }
    return result;
}


MSPModel_Striping::Obstacles
MSPModel_Striping::getNeighboringObstacles(const Pedestrians& pedestrians, int egoIndex, int stripes) {
    const Pedestrian& ego = *pedestrians[egoIndex];
    const SUMOReal egoBack = ego.myX - ego.getLength();
    int index = egoIndex + 1;
    Obstacles obs(stripes, Obstacle(ego.myDir));
    while (index < (int)pedestrians.size() && ego.myDir * (pedestrians[index]->myX - egoBack) > 0) {
        const Pedestrian& p = *pedestrians[index];
        Obstacle o(p, ego.myDir);
        obs[p.stripe()] = o;
        obs[p.otherStripe()] = o;
        index++;
    }
    return obs;
}


const MSPModel_Striping::Obstacles&
MSPModel_Striping::getNextLaneObstacles(NextLanesObstacles& nextLanesObs, const MSLane* nextLane, int stripes, int nextDir, 
        SUMOReal currentLength, int currentDir) {
    if (nextLanesObs.count(nextLane) == 0) {
        // figure out the which pedestrians are ahead on the next lane
        const int offset = (stripes - numStripes(nextLane)) / 2;
        Obstacles obs(stripes, Obstacle(nextDir));
        Pedestrians& pedestrians = getPedestrians(nextLane);
        // XXX consider waitingToEnter on nextLane
        sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter(nextDir));
        for (int ii = 0; ii < pedestrians.size(); ++ii) {
            const Pedestrian& p = *pedestrians[ii];
            Obstacle o(p, nextDir);
            if (currentDir == FORWARD) {
                o.x += currentLength;
            } else {
                o.x *= -1;
            }
            int mappedStripe = p.stripe() + offset;
            if (mappedStripe >= 0 && mappedStripe < stripes) {
                obs[mappedStripe] = o;
            }
            mappedStripe = p.otherStripe() + offset;
            if (mappedStripe >= 0 && mappedStripe < stripes) {
                obs[mappedStripe] = o;
            }
        }
        nextLanesObs[nextLane] = obs;
    }
    return nextLanesObs[nextLane];
}


void 
MSPModel_Striping::moveInDirection(SUMOTime currentTime, std::set<MSPerson*>& changedLane, int dir) {
    for (ActiveLanes::iterator it_lane = myActiveLanes.begin(); it_lane != myActiveLanes.end(); ++it_lane) {
        const MSLane* lane = it_lane->first;
        const int stripes = numStripes(lane);
        Pedestrians& pedestrians = it_lane->second;
        //std::cout << SIMTIME << ">>> lane=" << lane->getID() << " numPeds=" << pedestrians.size() << " waitingToEnter=" << waitingToEnter << "\n";
        
        // move forward
        Obstacles obs(stripes, Obstacle(dir)); // continously updated
        NextLanesObstacles nextLanesObs; // continously updated
        sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter(dir));
        for (int ii = 0; ii < pedestrians.size(); ++ii) {
            Pedestrian& p = *pedestrians[ii];
            //std::cout << SIMTIME << "CHECKING" << p.myPerson->getID() << "\n";
            Obstacles currentObs = obs;
            if (p.myDir != dir || changedLane.count(p.myPerson) !=0) {
                Obstacle o(p, dir);
                obs[p.stripe()] = o;
                obs[p.otherStripe()] = o;
                continue;
            }
            if DEBUGCOND(p.myPerson->getID()) {
                std::cout << SIMTIME << " ped=" << p.myPerson->getID()<< "  currentObs=";
                DEBUG_PRINT(currentObs);
            } 
            const MSLane* nextLane = p.myNLI.lane;
            const MSLink* link = p.myNLI.link;
            const SUMOReal dist = p.distToLaneEnd();
            const SUMOReal speed = p.myStage->getMaxSpeed();
            if (nextLane != 0) {
                currentObs = mergeObstacles(currentObs, getNextLaneObstacles(
                            nextLanesObs, nextLane, stripes, dir, lane->getLength(), dir), dir);
            }
            if DEBUGCOND(p.myPerson->getID()) {
                std::cout << SIMTIME << " ped=" << p.myPerson->getID()<< "  obsWithNext=";
                DEBUG_PRINT(currentObs);
            } 
            currentObs = mergeObstacles(currentObs, getNeighboringObstacles(pedestrians, ii, stripes), dir);
            if DEBUGCOND(p.myPerson->getID()) {
                std::cout << SIMTIME << " ped=" << p.myPerson->getID()<< "  obsWithNeigh=";
                DEBUG_PRINT(currentObs);
            } 
            // check link state 
            if (link != 0 
                    && dist < speed  // only check close before junction
                    && (!link->opened(currentTime, speed, speed, p.getLength(), p.getImpatience(currentTime), speed, 0)
                        // XXX check for presence of vehicles blocking the path
                       )) {
                // prevent movement passed a closed link
                Obstacles closedLink(stripes, Obstacle(p.myX + dir * dist - POSITION_EPS, 0, "closedLink"));
                currentObs = mergeObstacles(currentObs, closedLink, dir);
                // consider rerouting over another crossing
                if (p.myWalkingAreaPath != 0) {
                    // XXX actually another path would be needed starting at the current position
                    p.myNLI = getNextLane(p, p.myLane, p.myWalkingAreaPath->from);
                }
            }
            p.walk(currentObs, currentTime);
            Obstacle o(p, dir);
            obs[p.stripe()] = o;
            obs[p.otherStripe()] = o;
            //std::cout << SIMTIME << p.myPerson->getID() << " lane=" << lane->getID() << " x=" << p.myX << "\n";
        }

        // advance to the next lane
        const MSEdge& edge = lane->getEdge();
        sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter(dir));
        bool checkAdvance = true; 
        while (checkAdvance) {
            checkAdvance = false;;
            if (pedestrians.size() > 0) {
                Pedestrian* p = pedestrians.front();
                if (p->myDir != dir) {
                    continue;
                }
                if (p->moveToNextLane(currentTime)) {
                    pedestrians.erase(pedestrians.begin());
                    checkAdvance = true;
                    const MSLane* nextLane = p->myNLI.lane;
                    if (p->myLane != 0) {
                        changedLane.insert(p->myPerson);
                        myActiveLanes[p->myLane].push_back(p);
                    } else {
                        delete p;
                        myNumActivePedestrians--;
                    }
                }
            }
        }
    }
}

// ===========================================================================
// MSPModel_Striping::Obstacle method definitions
// ===========================================================================
MSPModel_Striping::Obstacle::Obstacle(int dir) :
    x(dir * FAR_AWAY), // far away when seen in dir
    speed(0),
    description("") 
{}


MSPModel_Striping::Obstacle::Obstacle(const Pedestrian& ped, int dir) :
    description(ped.myPerson->getID())
{
    if (ped.myWaitingToEnter) {
        x = dir * FAR_AWAY;
        speed = 0;
        description += "_ignored";
    } else {
        if (dir == ped.myDir) {
            speed = ped.mySpeed;
            x = ped.myX - dir * ped.getLength();
        } else {
            speed = -ped.mySpeed;
            x = ped.myX;
        }
    }
}


// ===========================================================================
// MSPModel_Striping::Pedestrian method definitions
// ===========================================================================


MSPModel_Striping::Pedestrian::Pedestrian(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, const MSLane* lane): 
    myPerson(person),
    myStage(stage),
    myLane(lane),
    myX(stage->getDepartPos()),
    myY(0), 
    myDir(FORWARD),
    mySpeed(0),
    myBlockedByOncoming(false),
    myWaitingToEnter(true),
    myWaitingTime(0),
    myWalkingAreaPath(0)
{ 
    const MSEdge* currentEdge = &lane->getEdge();
    assert(!currentEdge->isWalkingArea());
    const MSEdge* nextRouteEdge = myStage->getNextRouteEdge();
    const MSEdge* nextEdge = nextRouteEdge;
    if (nextRouteEdge != 0) {
        // initialize myDir by routing
        const MSJunction* nextJunction = 0;
        SUMOReal arrivalPos = myStage->getArrivalPos(); 
        if (nextRouteEdge != myStage->getRoute().back()) {
            // figure out nextJunction from the edge after nextRouteEdge
            const MSEdge* next2 = myStage->getRoute()[2];
            const int nextRouteEdgeDir = (
                    nextRouteEdge->getToJunction() == next2->getFromJunction() ||
                    nextRouteEdge->getToJunction() == next2->getToJunction() 
                    ? FORWARD : BACKWARD);
            arrivalPos = nextRouteEdgeDir == FORWARD ? 0 : nextRouteEdge->getLength();
        }
        std::vector<const MSEdge*> crossingRoute;
        MSNet::getInstance()->getPedestrianRouter().compute(currentEdge, nextRouteEdge, myX, arrivalPos, myStage->getMaxSpeed(), 0, nextJunction, crossingRoute, true);
        if (crossingRoute.size() > 1 && crossingRoute.back() == nextRouteEdge) {
            // route found
            nextEdge = crossingRoute[1];
            if (nextEdge->getFromJunction() == currentEdge->getFromJunction() || nextEdge->getToJunction() == currentEdge->getFromJunction()) {
                myDir = BACKWARD;
            }
        } else {
            // there is no connectivity, walk forward by default
        }
    } else {
        // only a single edge, move towards end pos
        myDir = (myX <= myStage->getArrivalPos()) ? FORWARD : BACKWARD;
    }
    if (myDir == FORWARD) {
        // start at the right side of the sidewalk
        myY = STRIPE_WIDTH * (numStripes(lane) - 1);
    }
    if DEBUGCOND(myPerson->getID()) std::cout << "  added new pedestrian " << myPerson->getID() << " on " << lane->getID() << " myX=" << myX << " myY=" << myY << " dir=" << myDir << " nextRouteEdge=" << (nextRouteEdge == 0 ? "NULL" : nextRouteEdge->getID()) << " route=" << toString(myStage->getRoute()) << "\n";

    myNLI = getNextLane(*this, lane, 0);
}


SUMOReal 
MSPModel_Striping::Pedestrian::getLength() const {
    return myPerson->getVehicleType().getLength();
}


int 
MSPModel_Striping::Pedestrian::stripe() const {
    const int max = numStripes(myLane) - 1;
    return MIN2(MAX2(0, (int)floor((myY + 0.5 * STRIPE_WIDTH) / STRIPE_WIDTH)), max);
}


int 
MSPModel_Striping::Pedestrian::otherStripe() const {
    const int max = numStripes(myLane) - 1;
    const int s = stripe();
    const SUMOReal offset = myY - s * STRIPE_WIDTH;
    const SUMOReal threshold = MAX2((SUMOReal)NUMERICAL_EPS, STRIPE_WIDTH - SQUEEZE * myPerson->getVehicleType().getWidth());
    int result;
    if (offset > threshold) {
        result = s + 1;
    } else if (offset < -threshold) {
        result = s - 1;
    } else {
        result = s;
    }
    std::cout.setf(std::ios::fixed , std::ios::floatfield);
    std::cout << std::setprecision(5);
    //if DEBUGCOND(myPerson->getID()) std::cout << "  otherStripe " << myPerson->getID() << " offset=" << offset << " threshold=" << threshold << " rawResult=" << result << "\n";
    return MIN2(MAX2(0, result), max);
}


SUMOReal 
MSPModel_Striping::Pedestrian::distToLaneEnd() const {
    if (myStage->getNextRouteEdge() == 0) {
        return myDir * (myStage->getArrivalPos() - myX);
    } else {
        const SUMOReal length = myWalkingAreaPath == 0 ? myLane->getLength() : myWalkingAreaPath->length;
        return myDir == FORWARD ? length - myX : myX;
    } 
}


bool
MSPModel_Striping::Pedestrian::moveToNextLane(SUMOTime currentTime) {
    const SUMOReal dist = distToLaneEnd();
    //if (myPerson->getID() == DEBUG1) {
    //    std::cout << SIMTIME << " myX=" << myX << " dist=" << dist << "\n";
    //}
    if (dist <= 0) {
        //if (ped.myPerson->getID() == DEBUG1) {
        //    std::cout << SIMTIME << " addToLane x=" << ped.myX << " newDir=" << newDir << " newLane=" << newLane->getID() << " walkingAreaShape=" << walkingAreaShape << "\n";
        //}
        //std::cout << " changing to " << newLane->getID() << " myY=" << ped.myY << " oldStripes=" << oldStripes << " newStripes=" << numStripes(newLane);
        //std::cout << " newY=" << ped.myY << " myDir=" << ped.myDir << " newDir=" << newDir;
        const int oldStripes = numStripes(myLane);
        const int oldDir = myDir;
        const MSLane* oldLane = myLane;
        myLane = myNLI.lane;
        myDir = myNLI.dir;
        const bool normalLane = (myLane == 0 || myLane->getEdge().getPurpose() == MSEdge::EDGEFUNCTION_NORMAL);
        if DEBUGCOND(myPerson->getID()) { 
            std::cout << SIMTIME 
                << " ped=" << myPerson->getID() 
                << " moveToNextLane old=" << oldLane->getID() 
                << " new=" << (myLane == 0 ? "NULL" : myLane->getID())
                << "\n";
        }
        myStage->moveToNextEdge(myPerson, currentTime, normalLane ? 0 : &myLane->getEdge());
        if (myLane != 0) {
            assert(myDir != UNDEFINED_DIRECTION);
            myNLI = getNextLane(*this, myLane, oldLane);
            if DEBUGCOND(myPerson->getID()) std::cout << "    nextLane=" << (myNLI.lane == 0 ? "NULL" : myNLI.lane->getID()) << "\n";
            if (myLane->getEdge().isWalkingArea()) {
                myWalkingAreaPath = &myWalkingAreaPaths[std::make_pair(oldLane, myNLI.lane)];
                assert(myWalkingAreaPath->from != 0);
                assert(myWalkingAreaPath->to != 0);
                assert(myWalkingAreaPath->shape.size() >= 2);
                if DEBUGCOND(myPerson->getID()) std::cout << "  mWAPath shape=" << myWalkingAreaPath->shape << " length=" << myWalkingAreaPath->length << "\n";
            } else {
                myWalkingAreaPath = 0;
            }
            // adapt x to fit onto the new lane
            if (myDir == BACKWARD) {
                const SUMOReal newLength = (myWalkingAreaPath == 0 ? myLane->getLength() : myWalkingAreaPath->length);
                myX = newLength + dist;
            } else {
                myX = -dist; 
            }
            // adjust to change in direction
            if (myDir != oldDir) {
                myY = (numStripes(oldLane) - 1) * STRIPE_WIDTH - myY;
            }
            // adjust to differences in sidewalk width
            myY += 0.5 * STRIPE_WIDTH * (numStripes(myLane) - oldStripes);
        }
        return true;
    } else {
        return false;
    }
}


void 
MSPModel_Striping::Pedestrian::walk(const Obstacles& obs, SUMOTime currentTime) {
    const int stripes = (int)obs.size();
    const int sMax =  stripes - 1;
    assert(stripes == numStripes(myLane));
    const SUMOReal vMax = myStage->getMaxSpeed();
    // ultimate goal is to chose the prefered stripe (chosen)
    const int current = stripe();
    const int other = otherStripe();
    int chosen = current;
    // compute utility for all stripes
    std::vector<SUMOReal> utility(stripes, 0);

    // penalize lateral movement (may increase jamming)
    for (int i = 0; i < stripes; ++i) {
        utility[i] += abs(i - current) * LATERAL_PENALTY;
    }
    // compute distances
    std::vector<SUMOReal> distance(stripes);
    for (int i = 0; i < stripes; ++i) {
        distance[i] += myDir * (obs[i].x - myX);
    }
    // forbid stripes which are blocked and also all stripes behind them
    for (int i = 0; i < stripes; ++i) {
        if (distance[i] <= 0) {
            if (i < current) {
                for (int j = 0; j <= i; ++j) {
                    utility[j] = 2 * BLOCKED_STRIPE;
                }
            } else {
                for (int j = i; j < stripes; ++j) {
                    utility[j] = 2 * BLOCKED_STRIPE;
                }
            }
        }
    }
    // forbid a portion of the leftmost stripes (in walking direction). 
    // lanes with stripes less than 1 / RESERVE_FOR_ONCOMING_FACTOR
    // may still deadlock in heavy pedestrian traffic
    const int reserved = (int)floor(stripes * RESERVE_FOR_ONCOMING_FACTOR);
    if (myDir == FORWARD) {
        for (int i = 0; i < reserved; ++i) {
            utility[i] = BLOCKED_STRIPE;
        }
    } else {
        for (int i = sMax; i > sMax - reserved; --i) {
            utility[i] = BLOCKED_STRIPE;
        }
    }
    // adapt utility based on obstacles
    for (int i = 0; i < stripes; ++i) {
        if (obs[i].speed < 0) {
            // penalize evasion to the left
            if (myDir == FORWARD && i > 0) {
                utility[i - 1] -= 0.5;
            } else if (myDir == BACKWARD && i < sMax) {
                utility[i + 1] -= 0.5;
            }
        }
        // compute expected distance achievable by staying on this stripe for a time horizon
        const SUMOReal lookAhead = obs[i].speed >= 0 ? LOOKAHEAD_SAMEDIR : LOOKAHEAD_ONCOMING;
        const SUMOReal expectedDist = MIN2(vMax * LOOKAHEAD_SAMEDIR, distance[i] + obs[i].speed * lookAhead);
        if (DEBUGCOND(myPerson->getID())) std::cout << " util=" << utility[i] << " exp" << expectedDist << "\n";
        if (expectedDist >= 0) {
            utility[i] += expectedDist;
        } else {
            // let only the distance count
            utility[i] += ONCOMING_CONFLICT + distance[i];
        }
    }
    // bonus to remain on the rightmost lane (in walking direction) if there
    // are oncoming
    if (((myDir == FORWARD && current == sMax)
             || (myDir == BACKWARD && current == 0))
            && obs[current].speed < 0) {
        utility[current] -= ONCOMING_CONFLICT;
    }

    // select best stripe 
    for (int i = 0; i < stripes; ++i) {
        if (utility[chosen] < utility[i]) {
            chosen = i;
        }
    }
    // compute speed components along both axes
    const int next = (chosen == current ? current : (chosen < current ? current - 1 : current + 1));
    const SUMOReal xDist = MIN3(distance[current], distance[other], distance[next]);
    // XXX preferred gap differs between approaching a standing obstacle or a moving obstacle
    const SUMOReal preferredGap = myPerson->getVehicleType().getMinGap() + xDist * 0.5;
    SUMOReal xSpeed = MIN2(vMax, MAX2((SUMOReal)0, xDist - preferredGap));
    // avoid tiny steps
    // XXX pressure from behind?
    if (mySpeed == 0 && xSpeed < 0.5 * vMax) {
        xSpeed = 0;
    }
    // dawdling
    const SUMOReal dawdle = MIN2(xSpeed, RandHelper::rand() * vMax * DAWDLING);
    xSpeed -= dawdle;

    // XXX ensure that diagonal speed <= vMax
    // avoid deadlocks on narrow sidewalks
    //if (oncoming && xSpeed == 0 && myStage->getWaitingTime(currentTime) > TIME2STEPS(ONCOMIN_PATIENCE)) {
    //    if DEBUGCOND(myPerson->getID()) std::cout << "  stepping asside to resolve oncoming deadlock\n";
    //    xSpeed = POSITION_EPS; // reset myWaitingTime
    //     if (myDir == FORWARD && chosen < sMax) {
    //         chosen += 1;
    //     } else if (myDir == BACKWARD && chosen > 0) {
    //         chosen -= 1;
    //     }
    //}
    const SUMOReal maxYSpeed = MAX2(vMax * LATERAL_SPEED_FACTOR, vMax - xSpeed);
    SUMOReal ySpeed = 0;
    const SUMOReal yDist = (chosen * STRIPE_WIDTH) - myY;
    if (fabs(yDist) > NUMERICAL_EPS) {
        ySpeed = (yDist > 0 ? 
                MIN2( maxYSpeed, yDist) :
                MAX2(-maxYSpeed, yDist));
    }
    // DEBUG
    if (true && DEBUGCOND(myPerson->getID())) {
        std::cout << SIMTIME 
            << " ped=" << myPerson->getID()
            << " edge=" << myStage->getEdge()->getID()
            << " x=" << myX
            << " y=" << myY
            << " d=" << myDir
            << " pvx=" << mySpeed
            << " cur=" << current
            << " cho=" << chosen
            << " oth=" << other
            << " nxt=" << next
            << " vx=" << xSpeed
            << " dawdle=" << dawdle
            << " vy=" << ySpeed
            << " xd=" << xDist
            << " yd=" << yDist
            << " vMax=" << myStage->getMaxSpeed()
            << " wTime=" << myStage->getWaitingTime(currentTime)
            << "\n distance=" << toString(distance) 
            << " utility=" << toString(utility) 
            << "\n";
        DEBUG_PRINT(obs);
    }
    myX += xSpeed * myDir;
    myY += ySpeed;
    mySpeed = xSpeed;
    if (xSpeed > 0) {
        myWaitingToEnter = false;
        myWaitingTime = 0;
    } else {
        myWaitingTime += DELTA_T;
    }
}


SUMOReal 
MSPModel_Striping::Pedestrian::getImpatience(SUMOTime now) const {
    return MAX2((SUMOReal)0, MIN2(SUMOReal(1), 
                myPerson->getVehicleType().getImpatience() 
                + STEPS2TIME(myStage->getWaitingTime(now)) / MAX_WAIT_TOLERANCE));
}


SUMOReal 
MSPModel_Striping::Pedestrian::getEdgePos(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    UNUSED_PARAMETER(stage);
    return myX;
}


Position 
MSPModel_Striping::Pedestrian::getPosition(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    const SUMOReal lateral_offset = myY + (STRIPE_WIDTH - myLane->getWidth()) * 0.5;
    if (myWalkingAreaPath == 0) {
        return stage.getLanePosition(myLane, myX, lateral_offset);
    } else {
        PositionVector shp = myWalkingAreaPath->shape;
        try {
            shp.move2side(lateral_offset);
        } catch (const InvalidArgument& e) {
            WRITE_WARNING("could not shift walkingArea " + myLane->getEdge().getID() + " shape " + toString(shp));
        }
        return shp.positionAtOffset(myX);
    }
}


SUMOReal 
MSPModel_Striping::Pedestrian::getAngle(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    UNUSED_PARAMETER(stage);
    const PositionVector& shp = myWalkingAreaPath == 0 ? myLane->getShape() : myWalkingAreaPath->shape;
    return -shp.rotationDegreeAtOffset(myX) + (myDir == MSPModel::BACKWARD ? 180 : 0);
}


SUMOTime 
MSPModel_Striping::Pedestrian::getWaitingTime(const MSPerson::MSPersonStage_Walking& stage, SUMOTime now) const {
    return myWaitingTime;
}


SUMOReal 
MSPModel_Striping::Pedestrian::getSpeed(const MSPerson::MSPersonStage_Walking& stage) const {
    UNUSED_PARAMETER(stage);
    return mySpeed;
}

// ===========================================================================
// MSPModel_Striping::MovePedestrians method definitions
// ===========================================================================
//

SUMOTime 
MSPModel_Striping::MovePedestrians::execute(SUMOTime currentTime) {
    std::set<MSPerson*> changedLane;
    myModel->moveInDirection(currentTime, changedLane, FORWARD);
    myModel->moveInDirection(currentTime, changedLane, BACKWARD);
    // DEBUG
    if (LOG_ALL) {
        for (ActiveLanes::const_iterator it_lane = myModel->getActiveLanes().begin(); it_lane != myModel->getActiveLanes().end(); ++it_lane) {
            const MSLane* lane = it_lane->first;
            Pedestrians pedestrians = it_lane->second;
            if (pedestrians.size() == 0) {
                continue;
            }
            sort(pedestrians.begin(), pedestrians.end(), by_xpos_sorter(FORWARD));
            std::cout << SIMTIME << " lane=" << lane->getID();
            for (int ii = 0; ii < pedestrians.size(); ++ii) {
                const Pedestrian& p = *pedestrians[ii];
                std::cout << " (" << p.myPerson->getID() << " " << p.myX << "," << p.myY << " " << p.myDir << ")";
            }
            std::cout << "\n";
        }
    }
    return DELTA_T;
}

