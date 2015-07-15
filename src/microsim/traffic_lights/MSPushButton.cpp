/****************************************************************************/
/// @file    MSPushButton.h
/// @author  Federico Caselli
/// @date    May 2015
/// @version $Id: MSPushButton.h 0 2015-05-21 11:40:00Z  $
///
// The class for a PushButton
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright 2001-2009 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#define SWARM_DEBUG
#include <utils/common/SwarmDebug.h>
#include "MSPushButton.h"
#include "MSPhaseDefinition.h"
#include "../MSEdge.h"
#include "../MSLane.h"
#include <microsim/pedestrians/MSPerson.h>

MSPushButton::MSPushButton(const MSEdge* edge, const MSEdge* crossingEdge)
{
    m_edge = edge;
    m_crossingEdge = crossingEdge;
    assert(crossingEdge->isCrossing());
}

MSPushButton::~MSPushButton()
{
    /// do not delete the MSEdge here
}

bool MSPushButton::anyActive(const std::vector<MSPushButton*> & pushButtons)
{
    for (std::vector<MSPushButton*>::const_iterator it = pushButtons.begin(); it != pushButtons.end(); ++it)
    {
        if (it.operator *()->isActivated())
            return true;
    }
    return false;
}

MSPedestrianPushButton::MSPedestrianPushButton(const MSEdge * walkingEdge, const MSEdge* crossingEdge)
        : MSPushButton(walkingEdge, crossingEdge)
{
    assert(edge->isWalkingArea());
}

bool MSPedestrianPushButton::isActivated() const
{
    return isActiveForEdge(m_edge, m_crossingEdge);
}

bool MSPedestrianPushButton::isActiveForEdge(const MSEdge * walkingEdge, const MSEdge* crossing)
{
    const std::set<MSTransportable*> persons = walkingEdge->getPersons();
    for (std::set<MSTransportable*>::const_iterator pIt = persons.begin(); pIt != persons.end(); ++pIt)
    {
        const MSPerson * person = (MSPerson *)*pIt;
        const MSEdge * nextEdge = person->getNextEdgePtr();
        ///TODO keep using >= 1 or switch to ==1. Should change return value from always active to active only when pressed?
        ///TODO If changed the swarm logic must be changed since it relays on this behavior that keeps it active
        if (person->getWaitingSeconds() >= 1 && nextEdge != 0 && nextEdge->isCrossing()
                && nextEdge->getID() == crossing->getID())
        {
            DBG(
            std::ostringstream oss;
            oss << "MSPedestrianPushButton::isActiveForEdge Pushbutton active for edge " << walkingEdge->getID() << " crossing " << crossing->getID()
                    << " per " << person->getID() << " wait " << person->getWaitingSeconds();
            WRITE_MESSAGE(oss.str());
            );
            return true;
        }
    }
    DBG(
    std::ostringstream oss;
    oss << "MSPedestrianPushButton::isActiveForEdge Pushbutton not active for edge " << walkingEdge->getID() << " crossing " << crossing->getID()
            << " num Persons " << persons.size();
    WRITE_MESSAGE(oss.str());
    );
    return false;
}


///@brief Checks between the edges to find a walking area which has as successor or predecessor the crossing
const MSEdge* getWalking(const std::vector<MSEdge*>& edges, const MSEdge * crossing)
{
    for (std::vector<MSEdge*>::const_iterator it = edges.begin(); it != edges.end(); ++it)
    {
        const MSEdge * tmpEdge = *it;
        if (tmpEdge->isWalkingArea()
                && (std::find(tmpEdge->getIncomingEdges().begin(), tmpEdge->getIncomingEdges().end(), crossing)
                        != tmpEdge->getIncomingEdges().end()
                        || std::find(tmpEdge->getOutgoingEdges().begin(), tmpEdge->getOutgoingEdges().end(), crossing)
                                != tmpEdge->getOutgoingEdges().end()))
            return tmpEdge;
    }
    return 0;
}

///@brief Get the walking area on the side of edge for the crossing
const MSEdge * getWalkingForEdgeAndCrossing(const MSEdge* edge, const MSEdge * crossing)
{
    const std::vector<MSEdge*> outEdges = edge->getOutgoingEdges();
    const std::vector<MSEdge*> inEdges = edge->getIncomingEdges();
    const MSEdge * walking = getWalking(outEdges, crossing);
    if (walking)
        return walking;
    return getWalking(inEdges, crossing);
}

bool MSPedestrianPushButton::isActiveOnAnySideOfTheRoad(const MSEdge * crossing)
{
    for (std::vector<std::string>::const_iterator it = crossing->getCrossingEdges().begin();
            it != crossing->getCrossingEdges().end(); ++it)
    {
        MSEdge* crossed = MSEdge::dictionary(*it);
        if (crossed)
        {
            const MSEdge* walking = getWalkingForEdgeAndCrossing(crossed, crossing);
            if (walking && isActiveForEdge(walking, crossing))
            {
                DBG(WRITE_MESSAGE("MSPedestrianPushButton::isActiveOnAnySideOfTheRoad crossing edge " + crossing->getID() + " crossed edge " + crossed->getID() + " walking edge" + walking->getID()););
                return true;
            }
        }
    }
    return false;
}

///@brief Add the crossing edges that cross the specified edge
void addCrossing(const std::vector<MSEdge*> & edgesToCheck, const std::string edgeToCross,
        std::vector<MSEdge*>& crossings)
{
    for (std::vector<MSEdge*>::const_iterator it = edgesToCheck.begin(); it != edgesToCheck.end(); ++it)
    {
        MSEdge* crossing = *it;
        if (crossing->isCrossing()
                && std::find(crossing->getCrossingEdges().begin(), crossing->getCrossingEdges().end(), edgeToCross)
                        != crossing->getCrossingEdges().end())
            crossings.push_back(crossing);
    }
}

std::vector<MSPushButton*> MSPedestrianPushButton::loadPushButtons(const MSPhaseDefinition* phase)
{
    std::vector<MSPushButton*> pushButtons;
    const std::vector<std::string> lanes = phase->getTargetLaneSet();
//    Multiple lane can be of the same edge, so I avoid readding them
    std::set<std::string> controlledEdges;
    for (std::vector<std::string>::const_iterator lIt = lanes.begin(); lIt != lanes.end(); ++lIt)
    {
        MSLane * lane = MSLane::dictionary(*lIt);
        if (lane)
        {
            MSEdge* laneEdge = &lane->getEdge();
            if (controlledEdges.count(laneEdge->getID()) != 0)
                continue;
            controlledEdges.insert(laneEdge->getID());
//            all the crossing that cross the controlled edge
            std::vector<MSEdge*> crossingEdges;
//            check between the outgoing edges of the controlled edge to find a walking area
            const std::vector<MSEdge*> outgoingEdges = laneEdge->getOutgoingEdges();
            for (std::vector<MSEdge*>::const_iterator eIt = outgoingEdges.begin(); eIt != outgoingEdges.end(); ++eIt)
            {
                const MSEdge* walking = *eIt;
                if (walking->isWalkingArea())
                {
                    addCrossing(walking->getIncomingEdges(), laneEdge->getID(), crossingEdges);
                    addCrossing(walking->getOutgoingEdges(), laneEdge->getID(), crossingEdges);
                }
            }
            for (std::vector<MSEdge*>::const_iterator cIt = crossingEdges.begin(); cIt != crossingEdges.end(); ++cIt)
            {
                MSEdge* crossing = *cIt;
                for (std::vector<std::string>::const_iterator it = crossing->getCrossingEdges().begin();
                            it != crossing->getCrossingEdges().end(); ++it)
                {
                    MSEdge* crossed = MSEdge::dictionary(*it);
                    if (crossed)
                    {
                        const MSEdge* walking = getWalkingForEdgeAndCrossing(crossed, crossing);
                        if (walking)
                        {
                            DBG(WRITE_MESSAGE("MSPedestrianPushButton::loadPushButtons Added pushButton for walking edge " + walking->getID() + " crossing edge "
                                    + crossing->getID() + " crossed edge " + crossed->getID() + ". Phase state " + phase->getState()););
                            pushButtons.push_back(new MSPedestrianPushButton(walking,crossing));
                        }
                    }
                }
            }
        }
    }
    return pushButtons;
}

