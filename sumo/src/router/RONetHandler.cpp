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
/// @file    RONetHandler.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Christian Roessel
/// @author  Michael Behrisch
/// @author  Yun-Pang Floetteroed
/// @date    Sept 2002
/// @version $Id$
///
// The handler for SUMO-Networks
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/options/OptionsCont.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/ToString.h>
#include <utils/xml/SUMORouteHandler.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include "ROEdge.h"
#include "ROLane.h"
#include "RONode.h"
#include "RONet.h"
#include "RONetHandler.h"
#include "ROAbstractEdgeBuilder.h"


// ===========================================================================
// method definitions
// ===========================================================================
RONetHandler::RONetHandler(RONet& net,
                           ROAbstractEdgeBuilder& eb)
    : SUMOSAXHandler("sumo-network"),
      myNet(net), myCurrentName(),
      myCurrentEdge(0), myCurrentStoppingPlace(0),
      myEdgeBuilder(eb) {}


RONetHandler::~RONetHandler() {}


void
RONetHandler::myStartElement(int element,
                             const SUMOSAXAttributes& attrs) {
    switch (element) {
        case SUMO_TAG_EDGE:
            // in the first step, we do need the name to allocate the edge
            // in the second, we need it to know to which edge we have to add
            //  the following edges to
            parseEdge(attrs);
            break;
        case SUMO_TAG_LANE:
            parseLane(attrs);
            break;
        case SUMO_TAG_JUNCTION:
            parseJunction(attrs);
            break;
        case SUMO_TAG_CONNECTION:
            parseConnection(attrs);
            break;
        case SUMO_TAG_BUS_STOP:
        case SUMO_TAG_TRAIN_STOP:
        case SUMO_TAG_CONTAINER_STOP:
        case SUMO_TAG_PARKING_AREA:
            parseStoppingPlace(attrs, (SumoXMLTag)element);
            break;
        case SUMO_TAG_ACCESS:
            parseAccess(attrs);
            break;
        case SUMO_TAG_TAZ:
            parseDistrict(attrs);
            break;
        case SUMO_TAG_TAZSOURCE:
            parseDistrictEdge(attrs, true);
            break;
        case SUMO_TAG_TAZSINK:
            parseDistrictEdge(attrs, false);
            break;
        case SUMO_TAG_TYPE: {
            bool ok = true;
            myCurrentTypeID = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
            break;
        }
        case SUMO_TAG_RESTRICTION: {
            bool ok = true;
            const SUMOVehicleClass svc = getVehicleClassID(attrs.get<std::string>(SUMO_ATTR_VCLASS, myCurrentTypeID.c_str(), ok));
            const double speed = attrs.get<double>(SUMO_ATTR_SPEED, myCurrentTypeID.c_str(), ok);
            if (ok) {
                myNet.addRestriction(myCurrentTypeID, svc, speed);
            }
            break;
        }
        default:
            break;
    }
}


void
RONetHandler::myEndElement(int element) {
    switch (element) {
        case SUMO_TAG_NET:
            // build junction graph
            for (std::set<std::string>::const_iterator it = myUnseenNodeIDs.begin(); it != myUnseenNodeIDs.end(); ++it) {
                WRITE_ERROR("Unknown node '" + *it + "'.");
            }
            break;
        default:
            break;
    }
}


void
RONetHandler::parseEdge(const SUMOSAXAttributes& attrs) {
    // get the id, report an error if not given or empty...
    bool ok = true;
    myCurrentName = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }
    const SumoXMLEdgeFunc func = attrs.getEdgeFunc(ok);
    if (!ok) {
        WRITE_ERROR("Edge '" + myCurrentName + "' has an unknown type.");
        return;
    }
    // get the edge
    std::string from;
    std::string to;
    int priority;
    myCurrentEdge = 0;
    if (func == EDGEFUNC_INTERNAL || func == EDGEFUNC_CROSSING || func == EDGEFUNC_WALKINGAREA) {
        assert(myCurrentName[0] == ':');
        const std::string junctionID = myCurrentName.substr(1, myCurrentName.rfind('_') - 1);
        from = junctionID;
        to = junctionID;
        priority = 0;
    } else {
        from = attrs.get<std::string>(SUMO_ATTR_FROM, myCurrentName.c_str(), ok);
        to = attrs.get<std::string>(SUMO_ATTR_TO, myCurrentName.c_str(), ok);
        priority = attrs.get<int>(SUMO_ATTR_PRIORITY, myCurrentName.c_str(), ok);
        if (!ok) {
            return;
        }
    }
    RONode* fromNode = myNet.getNode(from);
    if (fromNode == 0) {
        myUnseenNodeIDs.insert(from);
        fromNode = new RONode(from);
        myNet.addNode(fromNode);
    }
    RONode* toNode = myNet.getNode(to);
    if (toNode == 0) {
        myUnseenNodeIDs.insert(to);
        toNode = new RONode(to);
        myNet.addNode(toNode);
    }
    // build the edge
    myCurrentEdge = myEdgeBuilder.buildEdge(myCurrentName, fromNode, toNode, priority);
    // set the type
    myCurrentEdge->setRestrictions(myNet.getRestrictions(attrs.getOpt<std::string>(SUMO_ATTR_TYPE, myCurrentName.c_str(), ok, "")));
    myCurrentEdge->setFunction(func);

    if (myNet.addEdge(myCurrentEdge)) {
        fromNode->addOutgoing(myCurrentEdge);
        toNode->addIncoming(myCurrentEdge);
    } else {
        myCurrentEdge = 0;
    }
}


void
RONetHandler::parseLane(const SUMOSAXAttributes& attrs) {
    if (myCurrentEdge == 0) {
        // was an internal edge to skip or an error occured
        return;
    }
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    // get the speed
    double maxSpeed = attrs.get<double>(SUMO_ATTR_SPEED, id.c_str(), ok);
    double length = attrs.get<double>(SUMO_ATTR_LENGTH, id.c_str(), ok);
    std::string allow = attrs.getOpt<std::string>(SUMO_ATTR_ALLOW, id.c_str(), ok, "");
    std::string disallow = attrs.getOpt<std::string>(SUMO_ATTR_DISALLOW, id.c_str(), ok, "");
    if (!ok) {
        return;
    }
    // get the length
    // get the vehicle classes
    SVCPermissions permissions = parseVehicleClasses(allow, disallow);
    if (permissions != SVCAll) {
        myNet.setPermissionsFound();
    }
    // add when both values are valid
    if (maxSpeed > 0 && length > 0 && id.length() > 0) {
        myCurrentEdge->addLane(new ROLane(id, myCurrentEdge, length, maxSpeed, permissions));
    } else {
        WRITE_WARNING("Ignoring lane '" + id + "' with speed " + toString(maxSpeed) + " and length " + toString(length));
    }
}


void
RONetHandler::parseJunction(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (attrs.getNodeType(ok) == NODETYPE_INTERNAL) {
        return;
    }
    myUnseenNodeIDs.erase(id);
    // get the position of the node
    const double x = attrs.get<double>(SUMO_ATTR_X, id.c_str(), ok);
    const double y = attrs.get<double>(SUMO_ATTR_Y, id.c_str(), ok);
    const double z = attrs.getOpt<double>(SUMO_ATTR_Z, id.c_str(), ok, 0.);
    if (!ok) {
        return;
    }
    RONode* n = myNet.getNode(id);
    if (n == 0) {
        WRITE_WARNING("Skipping isolated junction '" + id + "'.");
    } else {
        n->setPosition(Position(x, y, z));
    }
}


void
RONetHandler::parseConnection(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    std::string fromID = attrs.get<std::string>(SUMO_ATTR_FROM, 0, ok);
    std::string toID = attrs.get<std::string>(SUMO_ATTR_TO, 0, ok);
    int fromLane = attrs.get<int>(SUMO_ATTR_FROM_LANE, 0, ok);
    int toLane = attrs.get<int>(SUMO_ATTR_TO_LANE, 0, ok);
    std::string dir = attrs.get<std::string>(SUMO_ATTR_DIR, 0, ok);
    std::string viaID = attrs.getOpt<std::string>(SUMO_ATTR_VIA, 0, ok, "");
    ROEdge* from = myNet.getEdge(fromID);
    ROEdge* to = myNet.getEdge(toID);
    if (from == 0) {
        throw ProcessError("unknown from-edge '" + fromID + "' in connection");
    }
    if (to == 0) {
        throw ProcessError("unknown to-edge '" + toID + "' in connection");
    }
    if ((int)from->getLanes().size() <= fromLane) {
        throw ProcessError("invalid fromLane '" + toString(fromLane) + "' in connection from '" + fromID + "'.");
    }
    if ((int)to->getLanes().size() <= toLane) {
        throw ProcessError("invalid toLane '" + toString(toLane) + "' in connection to '" + toID + "'.");
    }
    from->getLanes()[fromLane]->addOutgoingLane(to->getLanes()[toLane]);
    from->addSuccessor(to, dir);
    if (viaID != "") {
        ROEdge* via = myNet.getEdge(viaID.substr(0, viaID.rfind('_')));
        if (via == 0) {
            throw ProcessError("unknown via-edge '" + viaID + "' in connection");
        }
        from->addSuccessor(via, dir);
        return;
    }
}


void
RONetHandler::parseStoppingPlace(const SUMOSAXAttributes& attrs, const SumoXMLTag element) {
    bool ok = true;
    myCurrentStoppingPlace = new SUMOVehicleParameter::Stop();
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, toString(element).c_str(), ok);
    // get the lane
    myCurrentStoppingPlace->lane = attrs.get<std::string>(SUMO_ATTR_LANE, toString(element).c_str(), ok);
    if (!ok) {
        throw ProcessError();
    }
    const ROEdge* edge = myNet.getEdgeForLaneID(myCurrentStoppingPlace->lane);
    if (edge == 0) {
        throw InvalidArgument("Unknown lane '" + myCurrentStoppingPlace->lane + "' for " + toString(element) + " '" + id + "'.");
    }
    // get the positions
    myCurrentStoppingPlace->startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0);
    myCurrentStoppingPlace->endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, edge->getLength());
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    if (!ok || !SUMORouteHandler::checkStopPos(myCurrentStoppingPlace->startPos, myCurrentStoppingPlace->endPos, edge->getLength(), POSITION_EPS, friendlyPos)) {
        throw InvalidArgument("Invalid position for " + toString(element) + " '" + id + "'.");
    }
    if (element == SUMO_TAG_CONTAINER_STOP) {
        myNet.addContainerStop(id, myCurrentStoppingPlace);
    } else if (element == SUMO_TAG_PARKING_AREA) {
        myNet.addParkingArea(id, myCurrentStoppingPlace);
    } else {
        myNet.addBusStop(id, myCurrentStoppingPlace);
    }
}


void
RONetHandler::parseAccess(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string lane = attrs.get<std::string>(SUMO_ATTR_LANE, "access", ok);
    const ROEdge* edge = myNet.getEdgeForLaneID(lane);
    if (edge == 0) {
        throw InvalidArgument("Unknown lane '" + lane + "' for access.");
    }
    const double pos = attrs.getOpt<double>(SUMO_ATTR_POSITION, "access", ok, 0);
    if (!ok) {
        throw ProcessError();
    }
    myCurrentStoppingPlace->accessPos.insert(std::make_pair(lane, pos));
}


void
RONetHandler::parseDistrict(const SUMOSAXAttributes& attrs) {
    myCurrentEdge = 0;
    bool ok = true;
    myCurrentName = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    myNet.addDistrict(myCurrentName, myEdgeBuilder.buildEdge(myCurrentName + "-source", 0, 0, 0), myEdgeBuilder.buildEdge(myCurrentName + "-sink", 0, 0, 0));
    if (attrs.hasAttribute(SUMO_ATTR_EDGES)) {
        std::vector<std::string> desc = attrs.getStringVector(SUMO_ATTR_EDGES);
        for (std::vector<std::string>::const_iterator i = desc.begin(); i != desc.end(); ++i) {
            myNet.addDistrictEdge(myCurrentName, *i, true);
            myNet.addDistrictEdge(myCurrentName, *i, false);
        }
    }
}


void
RONetHandler::parseDistrictEdge(const SUMOSAXAttributes& attrs, bool isSource) {
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, myCurrentName.c_str(), ok);
    myNet.addDistrictEdge(myCurrentName, id, isSource);
}



/****************************************************************************/

