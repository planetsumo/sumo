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
/// @file    MSRightOfWayJunction.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Wed, 12 Dez 2001
/// @version $Id$
///
// junction.
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSRightOfWayJunction.h"
#include "MSLane.h"
#include "MSEdge.h"
#include "MSJunctionLogic.h"
#include "MSGlobals.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <utils/common/RandHelper.h>


// ===========================================================================
// method definitions
// ===========================================================================
MSRightOfWayJunction::MSRightOfWayJunction(const std::string& id,
        SumoXMLNodeType type,
        const Position& position,
        const PositionVector& shape,
        std::vector<MSLane*> incoming,
        std::vector<MSLane*> internal,
        MSJunctionLogic* logic) : MSLogicJunction(id, type, position, shape, incoming, internal),
    myLogic(logic) {}


MSRightOfWayJunction::~MSRightOfWayJunction() {
    delete myLogic;
}


void
MSRightOfWayJunction::postloadInit() {
    // inform links where they have to report approaching vehicles to
    int requestPos = 0;
    std::vector<MSLane*>::iterator i;
    // going through the incoming lanes...
    int maxNo = 0;
    std::vector<std::pair<MSLane*, MSLink*> > sortedLinks;
    for (i = myIncomingLanes.begin(); i != myIncomingLanes.end(); ++i) {
        const MSLinkCont& links = (*i)->getLinkCont();
        // ... set information for every link
        for (MSLinkCont::const_iterator j = links.begin(); j != links.end(); j++) {
            if ((*j)->getLane()->getEdge().isWalkingArea() ||
                    ((*i)->getEdge().isWalkingArea() && !(*j)->getLane()->getEdge().isCrossing())) {
                continue;
            }
            sortedLinks.push_back(std::make_pair(*i, *j));
            ++maxNo;
        }
    }

    const bool hasFoes = myLogic->hasFoes();
    for (i = myIncomingLanes.begin(); i != myIncomingLanes.end(); ++i) {
        const MSLinkCont& links = (*i)->getLinkCont();
        // ... set information for every link
        const MSLane* walkingAreaFoe = 0;
        for (MSLinkCont::const_iterator j = links.begin(); j != links.end(); j++) {
            if ((*j)->getLane()->getEdge().isWalkingArea()) {
                if ((*i)->getPermissions() != SVC_PEDESTRIAN) {
                    // vehicular lane connects to a walkingarea
                    walkingAreaFoe = (*j)->getLane();
                }
                continue;
            } else if (((*i)->getEdge().isWalkingArea() && !(*j)->getLane()->getEdge().isCrossing())) {
                continue;
            }
            if (myLogic->getLogicSize() <= requestPos) {
                throw ProcessError("Found invalid logic position of a link for junction '" + getID() + "' (" + toString(requestPos) + ", max " + toString(myLogic->getLogicSize()) + ") -> (network error)");
            }
            const MSLogicJunction::LinkBits& linkResponse = myLogic->getResponseFor(requestPos); // SUMO_ATTR_RESPONSE
            const MSLogicJunction::LinkBits& linkFoes = myLogic->getFoesFor(requestPos); // SUMO_ATTR_FOES
            bool cont = myLogic->getIsCont(requestPos);
            myLinkFoeLinks[*j] = std::vector<MSLink*>();
            for (int c = 0; c < maxNo; ++c) {
                if (linkResponse.test(c)) {
                    MSLink* foe = sortedLinks[c].second;
                    myLinkFoeLinks[*j].push_back(foe);
                    if (MSGlobals::gUsingInternalLanes && foe->getViaLane() != 0) {
                        assert(foe->getViaLane()->getLinkCont().size() == 1);
                        MSLink* foeExitLink = foe->getViaLane()->getLinkCont()[0];
                        // add foe links after an internal junction
                        if (foeExitLink->getViaLane() != 0) {
                            myLinkFoeLinks[*j].push_back(foeExitLink);
                        }
                    }
                }
            }
            std::vector<MSLink*> foes;
            for (int c = 0; c < maxNo; ++c) {
                if (linkFoes.test(c)) {
                    MSLink* foe = sortedLinks[c].second;
                    foes.push_back(foe);
                    MSLane* l = foe->getViaLane();
                    if (l == 0) {
                        continue;
                    }
                    // add foe links after an internal junction
                    const MSLinkCont& lc = l->getLinkCont();
                    for (MSLinkCont::const_iterator q = lc.begin(); q != lc.end(); ++q) {
                        if ((*q)->getViaLane() != 0) {
                            foes.push_back(*q);
                        }
                    }
                }
            }

            myLinkFoeInternalLanes[*j] = std::vector<MSLane*>();
            if (MSGlobals::gUsingInternalLanes && myInternalLanes.size() > 0) {
                int li = 0;
                for (int c = 0; c < (int)sortedLinks.size(); ++c) {
                    if (sortedLinks[c].second->getLane() == 0) { // dead end
                        continue;
                    }
                    if (linkFoes.test(c)) {
                        myLinkFoeInternalLanes[*j].push_back(myInternalLanes[li]);
                        if (linkResponse.test(c)) {
                            const std::vector<MSLane::IncomingLaneInfo>& l = myInternalLanes[li]->getIncomingLanes();
                            if (l.size() == 1 && l[0].lane->getEdge().isInternal()) {
                                myLinkFoeInternalLanes[*j].push_back(l[0].lane);
                            }
                        }
                    }
                    ++li;
                }
            }
            (*j)->setRequestInformation((int)requestPos, hasFoes, cont, myLinkFoeLinks[*j], myLinkFoeInternalLanes[*j]);
            // the exit link for a link before an internal junction is handled in MSInternalJunction
            // so we need to skip if cont=true
            if (MSGlobals::gUsingInternalLanes && (*j)->getViaLane() != 0 && !cont) {
                assert((*j)->getViaLane()->getLinkCont().size() == 1);
                MSLink* exitLink = (*j)->getViaLane()->getLinkCont()[0];
                exitLink->setRequestInformation((int)requestPos, false, false, std::vector<MSLink*>(),
                                                myLinkFoeInternalLanes[*j], (*j)->getViaLane());
            }
            // the exit link for a crossing is needed for the pedestrian model
            if (MSGlobals::gUsingInternalLanes && (*j)->getLane()->getEdge().isCrossing()) {
                MSLink* exitLink = (*j)->getLane()->getLinkCont()[0];
                exitLink->setRequestInformation((int)requestPos, false, false, std::vector<MSLink*>(),
                                                myLinkFoeInternalLanes[*j], (*j)->getLane());
            }
            for (std::vector<MSLink*>::const_iterator k = foes.begin(); k != foes.end(); ++k) {
                (*j)->addBlockedLink(*k);
                (*k)->addBlockedLink(*j);
            }
            requestPos++;
        }
        if (walkingAreaFoe != 0 && links.size() > 1) {
            for (MSLinkCont::const_iterator j = links.begin(); j != links.end(); j++) {
                if (!(*j)->getLane()->getEdge().isWalkingArea()) {
                    MSLink* exitLink = (*j)->getViaLane()->getLinkCont()[0];
                    exitLink->addWalkingAreaFoe(walkingAreaFoe);
                }
            }
        }
    }
}


/****************************************************************************/

