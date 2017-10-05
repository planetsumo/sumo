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
/// @file    GNERerouter.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
/// @version $Id$
///
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

#include <string>
#include <iostream>
#include <utility>
#include <utils/geom/GeomConvHelper.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/common/ToString.h>
#include <utils/geom/GeomHelper.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITextureSubSys.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/xml/SUMOSAXHandler.h>

#include "GNEViewNet.h"
#include "GNERerouter.h"
#include "GNERerouterDialog.h"
#include "GNERerouterInterval.h"
#include "GNELane.h"
#include "GNEEdge.h"
#include "GNEViewNet.h"
#include "GNEUndoList.h"
#include "GNENet.h"
#include "GNEChange_Attribute.h"


// ===========================================================================
// member method definitions
// ===========================================================================

GNERerouter::GNERerouter(const std::string& id, GNEViewNet* viewNet, Position pos, std::vector<GNEEdge*> edges, const std::string& filename, double probability, bool off) :
    GNEAdditional(id, viewNet, SUMO_TAG_REROUTER, ICON_REROUTER),
    myPosition(pos),
    myEdges(edges),
    myFilename(filename),
    myProbability(probability),
    myOff(off) {
    // Update geometry;
    updateGeometry();
}


GNERerouter::~GNERerouter() {
}


void
GNERerouter::updateGeometry() {
    // Clear shape
    myShape.clear();

    // Set block icon position
    myBlockIconPosition = myPosition;

    // Set block icon offset
    myBlockIconOffset = Position(-0.5, -0.5);

    // Set block icon rotation, and using their rotation for draw logo
    setBlockIconRotation();

    // Set position
    myShape.push_back(myPosition);

    /*
    // Add shape of childs (To avoid graphics errors)
    for (childEdges::iterator i = myChildEdges.begin(); i != myChildEdges.end(); i++) {
        myShape.append(i->edge->getLanes().at(0)->getShape());
    }

    // Update geometry of additional parent
    updateConnections();
    */
    // Refresh element (neccesary to avoid grabbing problems)
    myViewNet->getNet()->refreshAdditional(this);
}


Position
GNERerouter::getPositionInView() const {
    return myPosition;
}


void
GNERerouter::openAdditionalDialog() {
    // Open rerouter dialog
    GNERerouterDialog(this);
}


void
GNERerouter::moveGeometry(const Position& oldPos, const Position &offset) {
    // restore old position, apply offset and update Geometry
    myPosition = oldPos;
    myPosition.add(offset);
    updateGeometry();
}


void
GNERerouter::commitGeometryMoving(const Position& oldPos, GNEUndoList* undoList) {
    undoList->p_begin("position of " + toString(getTag()));
    undoList->p_add(new GNEChange_Attribute(this, SUMO_ATTR_POSITION, toString(myPosition), true, toString(oldPos)));
    undoList->p_end();
    // Refresh element
    myViewNet->getNet()->refreshAdditional(this);
}


void
GNERerouter::writeAdditional(OutputDevice& device) const {
    // Write parameters
    device.openTag(getTag());
    device.writeAttr(SUMO_ATTR_ID, getID());
    // obtain ID's of Edges
    std::vector<std::string> edgeIDs;
    for (std::vector<GNEEdge*>::const_iterator i = myEdges.begin(); i != myEdges.end(); i++) {
        edgeIDs.push_back((*i)->getID());
    }
    device.writeAttr(SUMO_ATTR_EDGES, joinToString(edgeIDs, " ").c_str());
    device.writeAttr(SUMO_ATTR_PROB, myProbability);
    if (!myFilename.empty()) {
        device.writeAttr(SUMO_ATTR_FILE, myFilename);
    }
    device.writeAttr(SUMO_ATTR_OFF, myOff);
    device.writeAttr(SUMO_ATTR_X, myPosition.x());
    device.writeAttr(SUMO_ATTR_Y, myPosition.y());

    // write intervals
    for (std::vector<GNERerouterInterval>::const_iterator i = myRerouterIntervals.begin(); i != myRerouterIntervals.end(); i++) {
        device.openTag(i->getTag());
        device.writeAttr(SUMO_ATTR_BEGIN, i->getBegin());
        device.writeAttr(SUMO_ATTR_END, i->getEnd());

        // write closing lane reroutes
        for (std::vector<GNEClosingLaneReroute>::const_iterator j = i->getClosingLaneReroutes().begin(); j != i->getClosingLaneReroutes().end(); j++) {
            device.openTag(j->getTag());
            device.writeAttr(SUMO_ATTR_LANE, j->getClosedLane()->getID());
            device.writeAttr(SUMO_ATTR_ALLOW, getVehicleClassNames(j->getAllowedVehicles()));
            device.writeAttr(SUMO_ATTR_DISALLOW, getVehicleClassNames(j->getDisallowedVehicles()));
            device.closeTag();
        }

        // write closing reroutes
        for (std::vector<GNEClosingReroute>::const_iterator j = i->getClosingReroutes().begin(); j != i->getClosingReroutes().end(); j++) {
            device.openTag(j->getTag());
            device.writeAttr(SUMO_ATTR_EDGE, j->getClosedEdge()->getID());
            device.writeAttr(SUMO_ATTR_ALLOW, getVehicleClassNames(j->getAllowedVehicles()));
            device.writeAttr(SUMO_ATTR_DISALLOW, getVehicleClassNames(j->getDisallowedVehicles()));
            device.closeTag();
        }

        // write dest prob reroutes
        for (std::vector<GNEDestProbReroute>::const_iterator j = i->getDestProbReroutes().begin(); j != i->getDestProbReroutes().end(); j++) {
            device.openTag(j->getTag());
            device.writeAttr(SUMO_ATTR_EDGE, j->getNewDestination()->getID());
            device.writeAttr(SUMO_ATTR_PROB, j->getProbability());
            device.closeTag();
        }

        // write route prob reroutes
        for (std::vector<GNERouteProbReroute>::const_iterator j = i->getRouteProbReroutes().begin(); j != i->getRouteProbReroutes().end(); j++) {
            device.openTag(j->getTag());
            device.writeAttr(SUMO_ATTR_ROUTE, j->getNewRouteId());
            device.writeAttr(SUMO_ATTR_PROB, j->getProbability());
            device.closeTag();
        }

        // Close tag
        device.closeTag();
    }

    // Close tag
    device.closeTag();
}


void
GNERerouter::addEdgeChild(GNEEdge* edge) {
    // Check that edge is valid and doesn't exist previously
    if (edge == NULL) {
        throw InvalidArgument("Trying to add an empty " + toString(SUMO_TAG_EDGE) + " child in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else if (std::find(myEdges.begin(), myEdges.end(), edge) != myEdges.end()) {
        throw InvalidArgument("Trying to add a duplicate " + toString(SUMO_TAG_EDGE) + " child in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else {
        myEdges.push_back(edge);
    }
}


void
GNERerouter::removeEdgeChild(GNEEdge* edge) {
    // Check that edge is valid and exist previously
    if (edge == NULL) {
        throw InvalidArgument("Trying to remove an empty " + toString(SUMO_TAG_EDGE) + " child in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else if (std::find(myEdges.begin(), myEdges.end(), edge) == myEdges.end()) {
        throw InvalidArgument("Trying to remove a non previously inserted " + toString(SUMO_TAG_EDGE) + " child in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else {
        myEdges.erase(std::find(myEdges.begin(), myEdges.end(), edge));
    }
}


const std::vector<GNEEdge*>&
GNERerouter::getEdgeChilds() const {
    return myEdges;
}


bool
GNERerouter::addRerouterInterval(const GNERerouterInterval& rerouterInterval) {
    // obtain a copy of current rerouter Intervals to check overlapping
    std::vector<GNERerouterInterval> copyOfMyRerouterIntervals;
    copyOfMyRerouterIntervals.push_back(rerouterInterval);
    if (checkOverlapping(copyOfMyRerouterIntervals)) {
        myRerouterIntervals = copyOfMyRerouterIntervals;
        return true;
    } else {
        return false;
    }
}


const std::vector<GNERerouterInterval>&
GNERerouter::getRerouterIntervals() const {
    return myRerouterIntervals;
}


bool
GNERerouter::setRerouterIntervals(const std::vector<GNERerouterInterval>& rerouterIntervals) {
    if (checkOverlapping(rerouterIntervals)) {
        myRerouterIntervals = rerouterIntervals;
        return true;
    } else {
        return false;
    }
}


const std::string&
GNERerouter::getParentName() const {
    return myViewNet->getNet()->getMicrosimID();
}


void
GNERerouter::drawGL(const GUIVisualizationSettings& s) const {
    // Start drawing adding an gl identificator
    glPushName(getGlID());

    // Add a draw matrix for drawing logo
    glPushMatrix();
    glTranslated(myShape[0].x(), myShape[0].y(), getType());
    glColor3d(1, 1, 1);
    glRotated(180, 0, 0, 1);

    // Draw icon depending of rerouter is or isn't selected
    if (isAdditionalSelected()) {
        GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_REROUTERSELECTED), 1);
    } else {
        GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_REROUTER), 1);
    }

    // Pop draw matrix
    glPopMatrix();

    // Show Lock icon depending of the Edit mode
    drawLockIcon(0.4);

    /*
    // Draw symbols in every lane
    const double exaggeration = s.addSize.getExaggeration(s);

    if (s.scale * exaggeration >= 3) {
        // draw rerouter symbol over all lanes

        for (childEdges::const_iterator i = myChildEdges.begin(); i != myChildEdges.end(); i++) {
            for (int lanePosIt = 0; lanePosIt < (int)i->positionsOverLanes.size(); lanePosIt++) {
                glPushMatrix();
                glTranslated(i->positionsOverLanes.at(lanePosIt).x(), i->positionsOverLanes.at(lanePosIt).y(), 0);
                glRotated(i->rotationsOverLanes.at(lanePosIt), 0, 0, 1);
                glTranslated(0, 0, getType());
                glScaled(exaggeration, exaggeration, 1);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                glBegin(GL_TRIANGLES);
                glColor3d(1, .8f, 0);
                // base
                glVertex2d(0 - 1.4, 0);
                glVertex2d(0 - 1.4, 6);
                glVertex2d(0 + 1.4, 6);
                glVertex2d(0 + 1.4, 0);
                glVertex2d(0 - 1.4, 0);
                glVertex2d(0 + 1.4, 6);
                glEnd();

                glTranslated(0, 0, .1);
                glColor3d(0, 0, 0);
                pfSetPosition(0, 0);
                pfSetScale(3.f);
                double w = pfdkGetStringWidth("U");
                glRotated(180, 0, 1, 0);
                glTranslated(-w / 2., 2, 0);
                pfXXDrawString("U");

                glTranslated(w / 2., -2, 0);
                std::string str = toString((int)(myProbability * 100)) + "%";
                pfSetPosition(0, 0);
                pfSetScale(.7f);
                w = pfdkGetStringWidth(str.c_str());
                glTranslated(-w / 2., 4, 0);
                pfXXDrawString(str.c_str());
                glPopMatrix();
            }
        }
        glPopName();
    }

    // Draw connections
    drawConnections();
    */
    // Pop name
    glPopName();

    // Draw name
    drawName(getCenteringBoundary().getCenter(), s.scale, s.addName);
}


std::string
GNERerouter::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_EDGES: {
            // obtain ID's of Edges
            std::vector<std::string> edgeIDs;
            for (std::vector<GNEEdge*>::const_iterator i = myEdges.begin(); i != myEdges.end(); i++) {
                edgeIDs.push_back((*i)->getID());
            }
            return joinToString(edgeIDs, " ");
        }
        case SUMO_ATTR_POSITION:
            return toString(myPosition);
        case SUMO_ATTR_FILE:
            return myFilename;
        case SUMO_ATTR_PROB:
            return toString(myProbability);
        case SUMO_ATTR_OFF:
            return toString(myOff);
        case GNE_ATTR_BLOCK_MOVEMENT:
            return toString(myBlocked);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERerouter::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_EDGES:
        case SUMO_ATTR_POSITION:
        case SUMO_ATTR_FILE:
        case SUMO_ATTR_PROB:
        case SUMO_ATTR_OFF:
        case GNE_ATTR_BLOCK_MOVEMENT:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            updateGeometry();
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNERerouter::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            if (isValidID(value) && (myViewNet->getNet()->getAdditional(getTag(), value) == NULL)) {
                return true;
            } else {
                return false;
            }
        case SUMO_ATTR_EDGES: {
            std::vector<std::string> edgeIds = GNEAttributeCarrier::parse<std::vector<std::string> > (value);
            // Empty Edges aren't valid
            if (edgeIds.empty()) {
                return false;
            }
            // Iterate over parsed edges and check that exists
            for (int i = 0; i < (int)edgeIds.size(); i++) {
                if (myViewNet->getNet()->retrieveEdge(edgeIds.at(i), false) == NULL) {
                    return false;
                }
            }
            // all edges exist, then is valid
            return true;
        }
        case SUMO_ATTR_POSITION:
            bool ok;
            return GeomConvHelper::parseShapeReporting(value, "user-supplied position", 0, ok, false).size() == 1;
        case SUMO_ATTR_FILE:
            return isValidFilename(value);
        case SUMO_ATTR_PROB:
            return canParse<double>(value) && (parse<double>(value) >= 0) && (parse<double>(value) <= 1);
        case SUMO_ATTR_OFF:
            return canParse<bool>(value);
        case GNE_ATTR_BLOCK_MOVEMENT:
            return canParse<bool>(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERerouter::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            setAdditionalID(value);
            break;
        case SUMO_ATTR_EDGES: {
            // Declare auxiliar variables
            std::vector<std::string> edgeIds = GNEAttributeCarrier::parse<std::vector<std::string> > (value);
            GNEEdge* edge;
            // first remove references of current rerouter in all edge childs
            for (std::vector<GNEEdge*>::iterator i = myEdges.begin(); i != myEdges.end(); i++) {
                (*i)->removeGNERerouter(this);
            }
            // clear previous edges
            myEdges.clear();
            // Iterate over parsed edges and obtain pointer to edges
            for (int i = 0; i < (int)edgeIds.size(); i++) {
                edge = myViewNet->getNet()->retrieveEdge(edgeIds.at(i), false);
                if (edge) {
                    myEdges.push_back(edge);
                    edge->addGNERerouter(this);
                } else {
                    throw InvalidArgument("Trying to set an non-valid edge in " + getID());
                }
            }
            break;
        }
        case SUMO_ATTR_POSITION:
            bool ok;
            myPosition = GeomConvHelper::parseShapeReporting(value, "user-supplied position", 0, ok, false)[0];
            updateGeometry();
            getViewNet()->update();
            break;
        case SUMO_ATTR_FILE:
            myFilename = value;
            break;
        case SUMO_ATTR_PROB:
            myProbability = parse<double>(value);
            break;
        case SUMO_ATTR_OFF:
            myOff = parse<bool>(value);
            break;
        case GNE_ATTR_BLOCK_MOVEMENT:
            myBlocked = parse<bool>(value);
            getViewNet()->update();
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNERerouter::checkOverlapping(std::vector<GNERerouterInterval> rerouterIntervals) {
    // only can be overlapping if there are more than two elements
    if (rerouterIntervals.size() <= 1) {
        return true;
    }
    // first short vector
    std::sort(rerouterIntervals.begin(), rerouterIntervals.end());

    // first check that all Begins are differents
    for (std::vector<GNERerouterInterval>::const_iterator i = (rerouterIntervals.begin() + 1); i != rerouterIntervals.end(); i++) {
        if ((i - 1)->getBegin() == i->getBegin()) {
            return false;
        }
    }
    // now check that end of every interval isn't overlapped with the begin of the next interval
    for (std::vector<GNERerouterInterval>::const_iterator i = rerouterIntervals.begin(); i != (rerouterIntervals.end() - 1); i++) {
        if (i->getEnd() > (i + 1)->getBegin()) {
            return false;
        }
    }

    // all ok, then return true
    return true;
}
/****************************************************************************/
