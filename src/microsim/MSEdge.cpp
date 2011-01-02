/****************************************************************************/
/// @file    MSEdge.cpp
/// @author  Christian Roessel
/// @date    Tue, 06 Mar 2001
/// @version $Id$
///
// A road/street connecting two junctions
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2010 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
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

#include "MSEdge.h"
#include "MSLane.h"
#include "MSLaneChanger.h"
#include "MSGlobals.h"
#include <algorithm>
#include <iostream>
#include <cassert>
#include "MSVehicle.h"
#include "MSVehicleTransfer.h"
#include <utils/common/StringTokenizer.h>
#include <utils/options/OptionsCont.h>
#include "MSEdgeWeightsStorage.h"
#include "MSAbstractLaneChangeModel.h"

#ifdef HAVE_MESOSIM
#include <mesosim/MELoop.h>
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

//#define DEBUG_VEHICLE_GUI_SELECTION 1
#ifdef DEBUG_VEHICLE_GUI_SELECTION
#include <utils/gui/div/GUIGlobalSelection.h>
#include <guisim/GUIVehicle.h>
#include <guisim/GUILane.h>
#endif


// ===========================================================================
// static member definitions
// ===========================================================================
MSEdge::DictType MSEdge::myDict;
std::vector<MSEdge*> MSEdge::myEdges;


// ===========================================================================
// member method definitions
// ===========================================================================
MSEdge::MSEdge(const std::string &id, unsigned int numericalID) throw()
        : myID(id), myNumericalID(numericalID), myLanes(0),
        myLaneChanger(0), myVaporizationRequests(0), myLastFailedEmissionTime(-1) {
}


MSEdge::~MSEdge() throw() {
    delete myLaneChanger;
    for (AllowedLanesCont::iterator i1=myAllowed.begin(); i1!=myAllowed.end(); i1++) {
        delete(*i1).second;
    }
    for (ClassedAllowedLanesCont::iterator i2=myClassedAllowed.begin(); i2!=myClassedAllowed.end(); i2++) {
        for (AllowedLanesCont::iterator i1=(*i2).second.begin(); i1!=(*i2).second.end(); i1++) {
            delete(*i1).second;
        }
    }
    delete myLanes;
    // Note: Lanes are delete using MSLane::clear();
}


void
MSEdge::initialize(MSLane* departLane,
                   std::vector<MSLane*>* lanes, EdgeBasicFunction function) throw() {
    assert(function == EDGEFUNCTION_DISTRICT || lanes!=0);
    myDepartLane = departLane;
    myLanes = lanes;
    myFunction = function;
    for (std::vector<MSLane*>::iterator i=myLanes->begin(); i!=myLanes->end(); ++i) {
        Supi supi;
        supi.lane = *i;
        supi.veh = (*i)->myVehicles.end();
        mySupis.push_back(supi);
    }
    if (myLanes && myLanes->size() > 1 && function!=EDGEFUNCTION_INTERNAL) {
        myLaneChanger = new MSLaneChanger(myLanes, OptionsCont::getOptions().getBool("lanechange.allow-swap"));
    }
}


void
MSEdge::closeBuilding() {
    myAllowed[0] = new std::vector<MSLane*>();
    for (std::vector<MSLane*>::iterator i=myLanes->begin(); i!=myLanes->end(); ++i) {
        myAllowed[0]->push_back(*i);
        const MSLinkCont &lc = (*i)->getLinkCont();
        for (MSLinkCont::const_iterator j=lc.begin(); j!=lc.end(); ++j) {
            MSLane *toL = (*j)->getLane();
            if (toL!=0) {
                MSEdge &to = toL->getEdge();
                //
                if (std::find(mySuccessors.begin(), mySuccessors.end(), &to)==mySuccessors.end()) {
                    mySuccessors.push_back(&to);
                }
                if (std::find(to.myPredeccesors.begin(), to.myPredeccesors.end(), this)==to.myPredeccesors.end()) {
                    to.myPredeccesors.push_back(this);
                }
                //
                if (myAllowed.find(&to)==myAllowed.end()) {
                    myAllowed[&to] = new std::vector<MSLane*>();
                }
                myAllowed[&to]->push_back(*i);
            }
        }
    }
    std::sort(mySuccessors.begin(), mySuccessors.end(), by_id_sorter());
    rebuildAllowedLanes();
}


void
MSEdge::rebuildAllowedLanes() throw() {
    // build the classed allowed lanes
    myHaveClassConstraints = false;
    // build list of vehicle classes that are constrained
    // ... all others will be not regarded (allowed) ...
    std::set<SUMOVehicleClass> vclasses;
    for (std::vector<MSLane*>::const_iterator i2=myLanes->begin(); i2!=myLanes->end(); ++i2) {
        const std::vector<SUMOVehicleClass> &allowed = (*i2)->getAllowedClasses();
        for (std::vector<SUMOVehicleClass>::const_iterator j=allowed.begin(); j!=allowed.end(); j++) {
            vclasses.insert(*j);
        }
        const std::vector<SUMOVehicleClass> &disallowed = (*i2)->getNotAllowedClasses();
        for (std::vector<SUMOVehicleClass>::const_iterator j=disallowed.begin(); j!=disallowed.end(); j++) {
            vclasses.insert(*j);
        }
    }
    // go through these classes
    for (std::set<SUMOVehicleClass>::const_iterator j=vclasses.begin(); j!=vclasses.end(); ++j) {
        // go through connected edges
        for (AllowedLanesCont::iterator i1=myAllowed.begin(); i1!=myAllowed.end(); ++i1) {
            delete myClassedAllowed[*j][(*i1).first];
            myClassedAllowed[*j][(*i1).first] = new std::vector<MSLane*>();
            // go through lanes approaching current edge
            for (std::vector<MSLane*>::iterator i2=(*i1).second->begin(); i2!=(*i1).second->end(); ++i2) {
                // allows the current vehicle class?
                if ((*i2)->allowsVehicleClass(*j)) {
                    // -> may be used
                    myClassedAllowed[*j][(*i1).first]->push_back(*i2);
                }
            }
            // assert that 0 is returned if no connection is allowed for a class
            if (myClassedAllowed[*j][(*i1).first]->size()==0) {
                delete myClassedAllowed[*j][(*i1).first];
                myClassedAllowed[*j][(*i1).first] = 0;
            }
        }
        myHaveClassConstraints = true;
    }
}


std::pair<MSVehicle * const, SUMOReal>
MSEdge::getRealLeader(const Supi &target, MSVehicle *vehicle, MSLane *lane) const throw() {
    // get the leading vehicle on the lane to change to
    MSVehicle* neighLead = target.veh!=target.vehEnd ? *target.veh : 0;
    // check whether the hopped vehicle got the leader
    if (neighLead==0) {
        MSLane* targetLane = target.lane;
        MSVehicle *predP = targetLane->getPartialOccupator();
        if (predP!=0) {
            return std::pair<MSVehicle *, SUMOReal>(predP, targetLane->getPartialOccupatorEnd() - vehicle->getPositionOnLane());
        }
        const std::vector<MSLane*> &bestLaneConts = vehicle->getBestLanesContinuation(lane);
        SUMOReal seen = lane->getLength() - vehicle->getPositionOnLane();
        SUMOReal speed = vehicle->getSpeed();
        SUMOReal dist = vehicle->getCarFollowModel().brakeGap(speed);
        if (seen>dist) {
            return std::pair<MSVehicle * const, SUMOReal>(static_cast<MSVehicle *>(0), -1);
        }
        return target.lane->getLeaderOnConsecutive(dist, seen, speed, *vehicle, bestLaneConts);
    } else {
        return std::pair<MSVehicle * const, SUMOReal>(neighLead, neighLead->getPositionOnLane()-neighLead->getVehicleType().getLength()-vehicle->getPositionOnLane());
    }
}

std::pair<MSVehicle * const, SUMOReal>
MSEdge::getRealFollower(const Supi &target, MSVehicle *vehicle, MSLane *lane) const throw() {
    MSVehicle* neighFollow = target.vehicleBuffer.size()!=0 ? target.vehicleBuffer.back() : 0;//target.veh!=target.lane->myVehicles.begin() ? *(target.veh-1) : 0;
    if (neighFollow==0) {
        SUMOReal speed = target.lane->getMaxSpeed();
        // in order to look back, we'd need the minimum braking ability of vehicles in the net...
        // we'll assume it to be 4m/s^2
        // !!!revisit
        SUMOReal dist = speed * speed / (2.*4.) + SPEED2DIST(speed);
        dist = MIN2(dist, (SUMOReal) 500.);
        SUMOReal seen = vehicle->getPositionOnLane()-vehicle->getVehicleType().getLength();
        return target.lane->getFollowerOnConsecutive(dist, seen, vehicle->getSpeed(), vehicle->getPositionOnLane() - vehicle->getVehicleType().getLength(), 4.5);
    } else {
        return std::pair<MSVehicle * const, SUMOReal>(neighFollow, vehicle->getPositionOnLane()-vehicle->getVehicleType().getLength()-neighFollow->getPositionOnLane());
    }
}



int
MSEdge::change2right(MSVehicle *vehicle, Supi &source, Supi &destination,
                     const std::pair<MSVehicle * const, SUMOReal> &leader,
                     const std::pair<MSVehicle * const, SUMOReal> &rLead,
                     const std::pair<MSVehicle * const, SUMOReal> &rFollow) const throw() {
    int blocked = 0;
    // overlap
    if (rFollow.first!=0&&rFollow.second<0) {
        blocked |= (LCA_BLOCKED_BY_RIGHT_FOLLOWER);
    }
    if (rLead.first!=0&&rLead.second<0) {
        blocked |= (LCA_BLOCKED_BY_RIGHT_LEADER);
    }
    // safe back gap
    if (rFollow.first!=0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        if (rFollow.second<rFollow.first->getCarFollowModel().getSecureGap(rFollow.first->getSpeed(), vehicle->getSpeed(), vehicle->getCarFollowModel().getMaxDecel())) {
            blocked |= LCA_BLOCKED_BY_RIGHT_FOLLOWER;
        }
    }

    // safe front gap
    if (rLead.first!=0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        if (rLead.second<vehicle->getCarFollowModel().getSecureGap(vehicle->getSpeed(), rLead.first->getSpeed(), rLead.first->getCarFollowModel().getMaxDecel())) {
            blocked |= LCA_BLOCKED_BY_RIGHT_LEADER;
        }
    }

    MSAbstractLaneChangeModel::MSLCMessager msg(leader.first, rLead.first, rFollow.first);
    return blocked | vehicle->getLaneChangeModel().wantsChangeToRight(
               msg, blocked, leader, rLead, rFollow, *destination.lane, vehicle->getBestLanes(), source.wantedChanges, source.wantedChangesIt);
}



int
MSEdge::change2left(MSVehicle *vehicle, Supi &source, Supi &destination, const std::pair<MSVehicle * const, SUMOReal> &leader,
                    const std::pair<MSVehicle * const, SUMOReal> &rLead,
                    const std::pair<MSVehicle * const, SUMOReal> &rFollow) const throw() {
    int blocked = 0;
    // overlap
    if (rFollow.first!=0&&rFollow.second<0) {
        blocked |= (LCA_BLOCKED_BY_LEFT_FOLLOWER);
    }
    if (rLead.first!=0&&rLead.second<0) {
        blocked |= (LCA_BLOCKED_BY_LEFT_LEADER);
    }
    // safe back gap
    if (rFollow.first!=0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        if (rFollow.second<rFollow.first->getCarFollowModel().getSecureGap(rFollow.first->getSpeed(), vehicle->getSpeed(), vehicle->getCarFollowModel().getMaxDecel())) {
            blocked |= LCA_BLOCKED_BY_LEFT_FOLLOWER;
        }
    }
    // safe front gap
    if (rLead.first!=0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        if (rLead.second<vehicle->getCarFollowModel().getSecureGap(vehicle->getSpeed(), rLead.first->getSpeed(), rLead.first->getCarFollowModel().getMaxDecel())) {
            blocked |= LCA_BLOCKED_BY_LEFT_LEADER;
        }
    }
    MSAbstractLaneChangeModel::MSLCMessager msg(leader.first, rLead.first, rFollow.first);
    return blocked | vehicle->getLaneChangeModel().wantsChangeToLeft(
               msg, blocked, leader, rLead, rFollow, *destination.lane, vehicle->getBestLanes(), source.wantedChanges, source.wantedChangesIt);
}

void
MSEdge::move1(SUMOTime t) {
    if (t==180000&&myID=="17") {
        int bla = 0;
    }
    if (myID=="85") {
        int bla = 0;
    }
    // init
    int vehicleNumber = 0;
    for (std::vector<Supi>::iterator i=mySupis.begin(); i!=mySupis.end(); ++i) {
        (*i).leftVehicleLength = (*i).lane->myVehicleLengthSum;
        (*i).collisions.clear();
        (*i).vehicleBuffer.clear();
		(*i).wantedChangesIt = (*i).wantedChanges.begin();
		(*i).nextWantedChanges.clear();

        MSLane::VehCont& vehicles = (*i).lane->myVehicles;
        (*i).vehEnd = vehicles.end();
        vehicleNumber += vehicles.size();
        if (vehicles.empty()) {
            (*i).veh = vehicles.end();
            continue;
        }
        (*i).veh = vehicles.begin();
    }
    for (std::vector<Supi>::iterator i=mySupis.begin(); i!=mySupis.end(); ++i) {
        for (MSLane::VehCont::const_iterator j=(*i).lane->myVehicles.begin(); j!=(*i).lane->myVehicles.end(); ++j) {
            if ((*j)->myLane!=(*i).lane) {
                int bla = 0;
            }
        }
    }
    if (vehicleNumber>1) {
        int bla = 0;
    }
    // go along
    while (--vehicleNumber>=0) {
        // get next vehicle

        //  not needed on one-lane edges
        std::vector<Supi>::iterator current = mySupis.end();
        for (std::vector<Supi>::iterator i=mySupis.begin(); i!=mySupis.end(); ++i) {
            if ((*i).veh==(*i).vehEnd) {
                continue;
            }
            if (current==mySupis.end()) {
                current = i;
                continue;
            }
            if ((*(*i).veh)->getPositionOnLane() < (*(*current).veh)->getPositionOnLane()) {
                current = i;
            }
        }
        if (current==mySupis.end()) {
            throw 1;
        }



        // the current one;
        //  we have everything stored in supis
        //  now move (compute best/wished/...) velocities



        // prepare
        Supi &currentSupi = *current;
        MSVehicle *currentVeh = *(currentSupi.veh);
        currentSupi.leftVehicleLength -= currentVeh->getVehicleType().getLength();
#ifdef DEBUG_VEHICLE_GUI_SELECTION
        if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(currentVeh)->getGlID())) {
            int bla = 0;
        }
#endif

        if (t>=186000&&currentVeh->getID()=="Borgo_20_22") {
            int bla = 0;
        }


        std::vector<Supi>::iterator finalSupi = current;

        // move forward


        // lane changing
        bool madeLaneChange = false;
        MSAbstractLaneChangeModel &lcModel = currentVeh->getLaneChangeModel();
        const std::vector<MSVehicle::LaneQ> &preb = currentVeh->getBestLanes();
        for (int i=0; i<(int) mySupis.size(); ++i) {
            ((std::vector<MSVehicle::LaneQ>&) preb)[i].occupation = mySupis[i].leftVehicleLength + preb[i].nextOccupation; /// !!! density in front?
        }
		while(currentSupi.wantedChangesIt!=currentSupi.wantedChanges.end()&&currentSupi.wantedChangesIt->pos<currentVeh->getPositionOnLane()) {
			++currentSupi.wantedChangesIt;
		}

        lcModel.prepareStep();
        // check whether the vehicle wants and is able to change to right lane
        /*
        std::pair<MSVehicle * const, SUMOReal> rLead = std::pair<MSVehicle *, SUMOReal>(static_cast<MSVehicle *>(0), -1);
        std::pair<MSVehicle * const, SUMOReal> lLead = std::pair<MSVehicle *, SUMOReal>(static_cast<MSVehicle *>(0), -1);
        std::pair<MSVehicle * const, SUMOReal> rFollow = std::pair<MSVehicle *, SUMOReal>(static_cast<MSVehicle *>(0), -1);
        std::pair<MSVehicle * const, SUMOReal> lFollow = std::pair<MSVehicle *, SUMOReal>(static_cast<MSVehicle *>(0), -1);
        */
        ++currentSupi.veh; // move to next vehicle
        std::pair<MSVehicle * const, SUMOReal> leader = getRealLeader(*current, currentVeh, current->lane);//getRealThisLeader(current);

        int state1 = 0;
        if (current!=mySupis.begin()&&(current-1)->lane->allowsVehicleClass(currentVeh->getVehicleType().getVehicleClass())) {
            std::pair<MSVehicle * const, SUMOReal> rLead = getRealLeader(*(current-1), currentVeh, current->lane);
            std::pair<MSVehicle * const, SUMOReal> rFollow = getRealFollower(*(current-1), currentVeh, current->lane);
            state1 = change2right(currentVeh, *current, *(current-1), leader, rLead, rFollow);
            if ((state1&LCA_URGENT)!=0||(state1&LCA_SPEEDGAIN)!=0) {
                state1 |= LCA_RIGHT;
            }
            bool changingAllowed = (state1&LCA_BLOCKED)==0;
            // change if the vehicle wants to and is allowed to change
            if ((state1&LCA_RIGHT)!=0&&changingAllowed) {
#ifndef NO_TRACI
                // inform lane change model about this change
                lcModel.fulfillChangeRequest(REQUEST_RIGHT);
#endif
                if (rLead.first!=0&&rFollow.first!=0) {
                    int bla = 0;
                }
                Supi &rightSupi = *(current - 1);
                finalSupi = current - 1;
                //rightSupi.vehicleBuffer.push_back(currentVeh);
                currentVeh->leaveLane(MSMoveReminder::NOTIFICATION_LANE_CHANGE);
                currentSupi.lane->leftByLaneChange(currentVeh);
                currentVeh->enterLaneAtLaneChange(rightSupi.lane);
                rightSupi.lane->enteredByLaneChange(currentVeh);
                currentVeh->myLastLaneChangeOffset = 0;
                currentVeh->getLaneChangeModel().changed();
                madeLaneChange = true;
                if (rightSupi.lane!=currentVeh->myLane) {
                    int bla = 0;
                }
            }
        }

        // check whether the vehicle wants and is able to change to left lane
        int state2 = 0;
        if (!madeLaneChange&&current+1!=mySupis.end()&&(current+1)->lane->allowsVehicleClass(currentVeh->getVehicleType().getVehicleClass())) {
			if(currentVeh->getID()=="bus_1_480"&&t>=950000) {
				int bla = 0;
			}
            std::pair<MSVehicle * const, SUMOReal> lLead = getRealLeader(*(current+1), currentVeh, current->lane);
            std::pair<MSVehicle * const, SUMOReal> lFollow = getRealFollower(*(current+1), currentVeh, current->lane);
            state2 = change2left(currentVeh, *current, *(current+1), leader, lLead, lFollow);
            if ((state2&LCA_URGENT)!=0||(state2&LCA_SPEEDGAIN)!=0) {
                state2 |= LCA_LEFT;
            }
            bool changingAllowed = (state2&LCA_BLOCKED)==0;
            // change if the vehicle wants to and is allowed to change
            if ((state2&LCA_LEFT)!=0&&changingAllowed) {
#ifndef NO_TRACI
                // inform lane change model about this change
                lcModel.fulfillChangeRequest(REQUEST_LEFT);
#endif
                Supi &leftSupi = *(current + 1);
                finalSupi = current + 1;
                //leftSupi.vehicleBuffer.push_back(currentVeh);
                currentVeh->leaveLane(MSMoveReminder::NOTIFICATION_LANE_CHANGE);
                currentSupi.lane->leftByLaneChange(currentVeh);
                currentVeh->enterLaneAtLaneChange(leftSupi.lane);
                leftSupi.lane->enteredByLaneChange(currentVeh);
                currentVeh->myLastLaneChangeOffset = 0;
                lcModel.changed();
                madeLaneChange = true;
                if (leftSupi.lane!=currentVeh->myLane) {
                    int bla = 0;
                }
            }
        }
        lcModel.setOwnState(state2|state1);

        if ((state1&(LCA_URGENT))!=0&&(state2&(LCA_URGENT))!=0) {
            // ... wants to go to the left AND to the right
            // just let them go to the right lane...
            state2 = 0;
            lcModel.setOwnState(state1);
        }
        // check whether the vehicles should be swapped
        if (!madeLaneChange&&myAllowsSwap&&((state1&(LCA_URGENT))!=0||(state2&(LCA_URGENT))!=0)) {
            // get the direction ...
            std::vector<Supi>::iterator target;
            int dir;
            if ((state1&(LCA_URGENT))!=0) {
                // ... wants to go right
                target = current - 1;
                dir = -1;
            }
            if ((state2&(LCA_URGENT))!=0) {
                // ... wants to go left
                target = current + 1;
                dir = 1;
            }
			MSVehicle *prohibitor = target->veh!=target->vehEnd ? *target->veh : 0;//target->vehicleBuffer.size()!=0 ? target->vehicleBuffer.back() : 0;
        if (prohibitor!=0&&t>=409000&&prohibitor->getID()=="1000066_1000000_1__10") {
            int bla = 0;
        }
            if (prohibitor!=0
                    &&
                    ((prohibitor->getLaneChangeModel().getOwnState()&(LCA_URGENT/*|LCA_SPEEDGAIN*/))!=0
                     &&
                     (prohibitor->getLaneChangeModel().getOwnState()&(LCA_LEFT|LCA_RIGHT))
                     !=
                     (lcModel.getOwnState()&(LCA_LEFT|LCA_RIGHT))
                    )
               ) {

                // check for position and speed
                if (prohibitor->getVehicleType().getLength()-currentVeh->getVehicleType().getLength()==0) {
                    // ok, may be swapped
                    // remove vehicle to swap with
                    MSLane::VehCont::iterator i = find(target->vehicleBuffer.begin(), target->vehicleBuffer.end(), prohibitor);
                    if (i!=target->vehicleBuffer.end()) {
                        MSVehicle *bla = *i;
                        assert(bla==prohibitor);
                        //target->vehicleBuffer.erase(i);
                        //target->vehicleBuffer.push_back(currentVeh);
                        finalSupi = target;
                        currentSupi.vehicleBuffer.push_back(prohibitor);

                        // leave lane and detectors
                        currentVeh->leaveLane(MSMoveReminder::NOTIFICATION_LANE_CHANGE);
                        currentSupi.lane->leftByLaneChange(currentVeh);
                        prohibitor->leaveLane(MSMoveReminder::NOTIFICATION_LANE_CHANGE);
                        target->lane->leftByLaneChange(prohibitor);
                        // patch position and speed
                        SUMOReal p1 = currentVeh->getPositionOnLane();
                        currentVeh->myState.myPos = prohibitor->myState.myPos;
                        prohibitor->myState.myPos = p1;
                        p1 = currentVeh->getSpeed();
                        currentVeh->myState.mySpeed = prohibitor->myState.mySpeed;
                        prohibitor->myState.mySpeed = p1;
                        // enter lane and detectors
                        currentVeh->enterLaneAtLaneChange(target->lane);
                        prohibitor->enterLaneAtLaneChange(current->lane);
                        // mark lane change
                        currentVeh->getLaneChangeModel().changed();
                        currentVeh->myLastLaneChangeOffset = 0;
                        prohibitor->getLaneChangeModel().changed();
                        prohibitor->myLastLaneChangeOffset = 0;
                        madeLaneChange = true;
                    }
                }
            }
        }
        // Candidate didn't change lane.
		currentVeh->setBlinkerInformation();

        MSVehicle *neighborLeader = 0;
        if (finalSupi+1!=mySupis.end()&&(finalSupi+1)->veh!=(finalSupi+1)->vehEnd) {
            neighborLeader = *(finalSupi+1)->veh;
        }
		MSVehicle *thisLeader = 0;
		if(finalSupi->veh!=finalSupi->vehEnd) {
			thisLeader = *finalSupi->veh;
		}
		/*
		if(currentVeh->moveRegardingCritical(t, (*finalSupi).lane, thisLeader, neighborLeader, (*finalSupi).leftVehicleLength)) {
            MsgHandler::getWarningInstance()->inform("Teleporting vehicle '" + currentVeh->getID() + "'; collision, lane='" + (*finalSupi).lane->getID() + "', time=" + time2string(t) + ".");
            (*finalSupi).lane->myVehicleLengthSum -= currentVeh->getVehicleType().getLength();
            MSVehicleTransfer::getInstance()->addVeh(t, currentVeh);
		} else {
		*/
			finalSupi->vehicleBuffer.push_back(currentVeh);
            if (!madeLaneChange) {
				if((lcModel.getOwnState()&LCA_URGENT)!=0) {
					if((lcModel.getOwnState()&LCA_RIGHT)!=0) {
						(*(current-1)).nextWantedChanges.push_back(MSEdge::KeptVehInfo(currentVeh->getPositionOnLane(), currentVeh->getSpeed(), 4.5/*!!! currentVeh->getVehicleType().getMaxDecel()*/, currentVeh->getVehicleType().getLength()));
					} else {
						(*(current+1)).nextWantedChanges.push_back(MSEdge::KeptVehInfo(currentVeh->getPositionOnLane(), currentVeh->getSpeed(), 4.5/*!!! currentVeh->getVehicleType().getMaxDecel()*/, currentVeh->getVehicleType().getLength()));
					}
				}
                //currentSupi.vehicleBuffer.push_back(currentVeh);
                if (currentSupi.lane!=currentVeh->myLane) {
                    int bla = 0;
                }
                currentVeh->myLastLaneChangeOffset += DELTA_T;
            }
		//}
    }


    for (std::vector<Supi>::iterator i=mySupis.begin(); i!=mySupis.end(); ++i) {
        (*i).lane->getVehiclesSecure();
        (*i).lane->myVehicles = (*i).vehicleBuffer;
        (*i).lane->releaseVehicles();
		if((*i).vehicleBuffer.size()!=0) {
			(*i).lane->moveCritical(t);
		}
		(*i).wantedChanges = (*i).nextWantedChanges;
		/*
        for (MSLane::VehCont::const_iterator j=(*i).lane->myVehicles.begin(); j!=(*i).lane->myVehicles.end(); ++j) {
            if ((*j)->myLane!=(*i).lane) {
                int bla = 0;
            }
            if (j+1!=(*i).lane->myVehicles.end()) {
                if ((*j)->getPositionOnLane()>(*(j+1))->getPositionOnLane()-(*(j+1))->getVehicleType().getLength()) {
                    int bla = 0;
                }
            }
        }
		*/
    }
}


// ------------ Access to the edge's lanes
MSLane*
MSEdge::leftLane(const MSLane * const lane) const throw() {
    std::vector<MSLane*>::iterator laneIt = find(myLanes->begin(), myLanes->end(), lane);
    if (laneIt==myLanes->end()||laneIt==myLanes->end()-1) {
        return 0;
    }
    return *(laneIt+1);
}


MSLane*
MSEdge::rightLane(const MSLane * const lane) const throw() {
    std::vector<MSLane*>::iterator laneIt = find(myLanes->begin(), myLanes->end(), lane);
    if (laneIt==myLanes->end()||laneIt==myLanes->begin()) {
        return 0;
    }
    return *(laneIt-1);
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(const MSEdge& destination, SUMOVehicleClass vclass) const throw() {
    return allowedLanes(&destination, vclass);
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(SUMOVehicleClass vclass) const throw() {
    return allowedLanes(0, vclass);
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(const MSEdge *destination, SUMOVehicleClass vclass) const throw() {
    if (myHaveClassConstraints&&vclass!=SVC_UNKNOWN) {
        ClassedAllowedLanesCont::const_iterator i = myClassedAllowed.find(vclass);
        if (i!=myClassedAllowed.end()) {
            const AllowedLanesCont &c = (*i).second;
            AllowedLanesCont::const_iterator j = (*i).second.find(destination);
            if (j==c.end()) {
                // Destination-edge not found.
                return 0;
            }
            return (*j).second;
        }
    }
    AllowedLanesCont::const_iterator it = myAllowed.find(destination);
    if (it!=myAllowed.end()) {
        return it->second;
    } else {
        // Destination-edge not found.
        return 0;
    }
}


// ------------
SUMOTime
MSEdge::incVaporization(SUMOTime) throw(ProcessError) {
    ++myVaporizationRequests;
    return 0;
}


SUMOTime
MSEdge::decVaporization(SUMOTime) throw(ProcessError) {
    --myVaporizationRequests;
    return 0;
}


MSLane *
MSEdge::getFreeLane(const std::vector<MSLane*>* allowed, const SUMOVehicleClass vclass) const throw() {
    if (allowed==0) {
        allowed = allowedLanes(vclass);
    }
    MSLane* res = 0;
    if (allowed != 0) {
        unsigned int noCars = INT_MAX;
        for (std::vector<MSLane*>::const_iterator i=allowed->begin(); i!=allowed->end(); ++i) {
            if ((*i)->getVehicleNumber()<noCars) {
                res = (*i);
                noCars = (*i)->getVehicleNumber();
            }
        }
    }
    return res;
}


MSLane *
MSEdge::getDepartLane(const MSVehicle &veh) const throw() {
    switch (veh.getParameter().departLaneProcedure) {
    case DEPART_LANE_GIVEN:
        if ((int) myLanes->size() <= veh.getParameter().departLane || !(*myLanes)[veh.getParameter().departLane]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
            return 0;
        }
        return (*myLanes)[veh.getParameter().departLane];
    case DEPART_LANE_RANDOM:
        return RandHelper::getRandomFrom(*allowedLanes(veh.getVehicleType().getVehicleClass()));
    case DEPART_LANE_FREE:
        return getFreeLane(0, veh.getVehicleType().getVehicleClass());
    case DEPART_LANE_ALLOWED_FREE:
        if (veh.getRoute().size()==1) {
            return getFreeLane(0, veh.getVehicleType().getVehicleClass());
        } else {
            return getFreeLane(allowedLanes(**(veh.getRoute().begin()+1)), veh.getVehicleType().getVehicleClass());
        }
    case DEPART_LANE_BEST_FREE: {
        const std::vector<MSVehicle::LaneQ> &bl = veh.getBestLanes(false, (*myLanes)[0]);
        SUMOReal bestLength = -1;
        for (std::vector<MSVehicle::LaneQ>::const_iterator i=bl.begin(); i!=bl.end(); ++i) {
            if ((*i).length>bestLength) {
                bestLength = (*i).length;
            }
        }
        std::vector<MSLane*> *bestLanes = new std::vector<MSLane*>();
        for (std::vector<MSVehicle::LaneQ>::const_iterator i=bl.begin(); i!=bl.end(); ++i) {
            if ((*i).length==bestLength) {
                bestLanes->push_back((*i).lane);
            }
        }
        MSLane *ret = getFreeLane(bestLanes, veh.getVehicleType().getVehicleClass());
        delete bestLanes;
        return ret;
    }
    case DEPART_LANE_DEPARTLANE:
    case DEPART_LANE_DEFAULT:
    default:
        break;
    }
    if (!myDepartLane->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
        return 0;
    }
    return myDepartLane;
}


bool
MSEdge::emit(SUMOVehicle &v, SUMOTime time) const throw(ProcessError) {
    // when vaporizing, no vehicles are emitted...
    if (isVaporizing()) {
        return false;
    }
    const SUMOVehicleParameter &pars = v.getParameter();
#ifdef HAVE_MESOSIM
    if (MSGlobals::gUseMesoSim) {
        SUMOReal pos = 0.0;
        switch (pars.departPosProcedure) {
        case DEPART_POS_GIVEN:
            if (pars.departPos >= 0.) {
                pos = pars.departPos;
            } else {
                pos = pars.departPos + getLanes()[0]->getLength();
            }
            break;
        case DEPART_POS_RANDOM:
        case DEPART_POS_RANDOM_FREE:
            pos = RandHelper::rand(getLanes()[0]->getLength());
            break;
        default:
            break;
        }
        bool result = false;
        bool insertToNet = false;
        MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this, pos);
        MEVehicle* veh = static_cast<MEVehicle*>(&v);
        if (pars.departPosProcedure == DEPART_POS_FREE) {
            while (segment != 0 && !result) {
                result = segment->initialise(veh, time, insertToNet);
                segment = segment->getNextSegment();
            }
        } else {
            result = segment->initialise(veh, time, insertToNet);
        }
        if (insertToNet) {
            MSGlobals::gMesoNet->addCar(veh);
        }
        return result;
    }
#endif
    MSLane* emitLane = getDepartLane(static_cast<MSVehicle&>(v));
    return emitLane != 0 && emitLane->emit(static_cast<MSVehicle&>(v));
}


void
MSEdge::changeLanes(SUMOTime t) throw() {
    if (myFunction==EDGEFUNCTION_INTERNAL) {
        return;
    }
    assert(myLaneChanger != 0);
    myLaneChanger->laneChange(t);
}



#ifdef HAVE_INTERNAL_LANES
const MSEdge *
MSEdge::getInternalFollowingEdge(MSEdge *followerAfterInternal) const throw() {
    //@ to be optimized
    for (std::vector<MSLane*>::const_iterator i=myLanes->begin(); i!=myLanes->end(); ++i) {
        MSLane *l = *i;
        const MSLinkCont &lc = l->getLinkCont();
        for (MSLinkCont::const_iterator j=lc.begin(); j!=lc.end(); ++j) {
            MSLink *link = *j;
            if (&link->getLane()->getEdge()==followerAfterInternal) {
                return &link->getViaLane()->getEdge();
            }
        }
    }
    return 0;
}
#endif


SUMOReal
MSEdge::getCurrentTravelTime() const throw() {
    SUMOReal v = 0;
#ifdef HAVE_MESOSIM
    if (MSGlobals::gUseMesoSim) {
        MESegment *first = MSGlobals::gMesoNet->getSegmentForEdge(*this);
        unsigned segments = 0;
        do {
            v += first->getMeanSpeed();
            first = first->getNextSegment();
            segments++;
        } while (first!=0);
        v /= (SUMOReal) segments;
    } else {
#endif
        for (std::vector<MSLane*>::iterator i=myLanes->begin(); i!=myLanes->end(); ++i) {
            v += (*i)->getMeanSpeed();
        }
        v /= (SUMOReal) myLanes->size();
#ifdef HAVE_MESOSIM
    }
#endif
    if (v!=0) {
        return (*myLanes)[0]->getLength() / v;
    } else {
        return 1000000.;
    }
}


bool
MSEdge::prohibits(const SUMOVehicle * const vehicle) const throw() {
    if (myFunction == EDGEFUNCTION_DISTRICT || !myHaveClassConstraints) {
        return false;
    }
    SUMOVehicleClass vclass = vehicle->getVehicleType().getVehicleClass();
    for (std::vector<MSLane*>::iterator i=myLanes->begin(); i!=myLanes->end(); ++i) {
        if ((*i)->allowsVehicleClass(vclass)) {
            return false;
        }
    }
    return true;
}


bool
MSEdge::dictionary(const std::string &id, MSEdge* ptr) throw() {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        myDict[id] = ptr;
        while (myEdges.size()<ptr->getNumericalID()+1) {
            myEdges.push_back(0);
        }
        myEdges[ptr->getNumericalID()] = ptr;
        return true;
    }
    return false;
}


MSEdge*
MSEdge::dictionary(const std::string &id) throw() {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        return 0;
    }
    return it->second;
}


MSEdge*
MSEdge::dictionary(size_t id) throw() {
    assert(myEdges.size()>id);
    return myEdges[id];
}


size_t
MSEdge::dictSize() throw() {
    return myDict.size();
}


void
MSEdge::clear() throw() {
    for (DictType::iterator i=myDict.begin(); i!=myDict.end(); ++i) {
        delete(*i).second;
    }
    myDict.clear();
}


void
MSEdge::insertIDs(std::vector<std::string> &into) throw() {
    for (DictType::iterator i=myDict.begin(); i!=myDict.end(); ++i) {
        into.push_back((*i).first);
    }
}


void
MSEdge::parseEdgesList(const std::string &desc, std::vector<const MSEdge*> &into,
                       const std::string &rid) throw(ProcessError) {
    StringTokenizer st(desc);
    parseEdgesList(st.getVector(), into, rid);
}


void
MSEdge::parseEdgesList(const std::vector<std::string> &desc, std::vector<const MSEdge*> &into,
                       const std::string &rid) throw(ProcessError) {
    for (std::vector<std::string>::const_iterator i=desc.begin(); i!=desc.end(); ++i) {
        const MSEdge *edge = MSEdge::dictionary(*i);
        // check whether the edge exists
        if (edge==0) {
            throw ProcessError("The edge '" + *i + "' within the route " + rid + " is not known."
                               + "\n The route can not be build.");
        }
        into.push_back(edge);
    }
}


/****************************************************************************/

