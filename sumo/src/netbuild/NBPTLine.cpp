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
/// @file    NBPTLine.cpp
/// @author  Gregor Laemmel
/// @author  Nikita Cherednychek
/// @date    Tue, 20 Mar 2017
/// @version $Id$
///
// The representation of one direction of a single pt line
/****************************************************************************/
#include <utils/iodevices/OutputDevice.h>

#include <utility>
#include <utils/common/ToString.h>
#include <utils/common/StringUtils.h>
#include "NBEdgeCont.h"
#include "NBPTLine.h"
#include "NBPTStop.h"

NBPTLine::NBPTLine(const std::string& name, const std::string& type) : 
    myName(name), 
    myType(type),
    myPTLineId(-1), 
    myRef(name) 
{ }

void NBPTLine::addPTStop(NBPTStop* pStop) {
    myPTStops.push_back(pStop);

}
const std::string& NBPTLine::getName() const {
    return myName;
}

long long int 
NBPTLine::getLineID() const {
    return myPTLineId;
}

std::vector<NBPTStop*> NBPTLine::getStops() {
    return myPTStops;
}
void NBPTLine::write(OutputDevice& device, NBEdgeCont& ec) {
    device.openTag(SUMO_TAG_PT_LINE);
    device.writeAttr(SUMO_ATTR_ID, myPTLineId);
    if (!myName.empty()) {
        device.writeAttr(SUMO_ATTR_NAME, StringUtils::escapeXML(myName));
    }

    device.writeAttr(SUMO_ATTR_LINE, myRef);
    device.writeAttr(SUMO_ATTR_TYPE, myType);
    device.writeAttr("completeness", toString((double)myPTStops.size()/(double)myNumOfStops));

    std::vector<std::string> validEdgeIDs;
    // filter out edges that have been removed due to joining junctions 
    // (therest of the route is valid)
    for (NBEdge* e : myRoute) {
        if (ec.retrieve(e->getID())) {
            validEdgeIDs.push_back(e->getID());
        }
    }
    if (!myRoute.empty()) {
        device.openTag(SUMO_TAG_ROUTE);
        device.writeAttr(SUMO_ATTR_EDGES, validEdgeIDs);
        device.closeTag();
    }

    for (auto& myPTStop : myPTStops) {
        device.openTag(SUMO_TAG_BUS_STOP);
        device.writeAttr(SUMO_ATTR_ID, myPTStop->getID());
        device.writeAttr(SUMO_ATTR_NAME, myPTStop->getName());
        device.closeTag();
    }
//    device.writeAttr(SUMO_ATTR_LANE, myLaneId);
//    device.writeAttr(SUMO_ATTR_STARTPOS, myStartPos);
//    device.writeAttr(SUMO_ATTR_ENDPOS, myEndPos);
//    device.writeAttr(SUMO_ATTR_FRIENDLY_POS, "true");
    device.closeTag();

}
void NBPTLine::setId(long long int id) {
    myPTLineId = id;
}
void NBPTLine::addWayNode(long long int way, long long int node) {
    std::string wayStr = toString(way);
    if (wayStr != myCurrentWay) {
        myCurrentWay = wayStr;
        myWays.push_back(wayStr);
    }
    myWaysNodes[wayStr].push_back(node);

}
const std::vector<std::string>& NBPTLine::getMyWays() const {
    return myWays;
}
std::vector<long long int>* NBPTLine::getWaysNodes(std::string wayId) {
    if (myWaysNodes.find(wayId) != myWaysNodes.end()) {
        return &myWaysNodes[wayId];
    }
    return nullptr;
}
void NBPTLine::setRef(std::string ref) {
    myRef = std::move(ref);
}

void NBPTLine::addEdgeVector(std::vector<NBEdge*>::iterator fr, std::vector<NBEdge*>::iterator to) {
    myRoute.insert(myRoute.end(), fr, to);

}
void NBPTLine::setMyNumOfStops(int numStops) {
    myNumOfStops = numStops;
}
const std::vector<NBEdge*>& NBPTLine::getRoute() const {
    return myRoute;
}
