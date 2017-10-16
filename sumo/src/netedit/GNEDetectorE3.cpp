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
/// @file    GNEDetectorE3.cpp
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
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/common/MsgHandler.h>

#include "GNEDetectorE3.h"
#include "GNEDetectorEntry.h"
#include "GNEDetectorExit.h"
#include "GNELane.h"
#include "GNEViewNet.h"
#include "GNEUndoList.h"
#include "GNENet.h"
#include "GNEChange_Attribute.h"
#include "GNEEdge.h"


// ===========================================================================
// member method definitions
// ===========================================================================

GNEDetectorE3::GNEDetectorE3(const std::string& id, GNEViewNet* viewNet, Position pos, double freq, const std::string& filename, const double timeThreshold, double speedThreshold) :
    GNEAdditional(id, viewNet, SUMO_TAG_E3DETECTOR, ICON_E3),
    myPosition(pos),
    myFreq(freq),
    myFilename(filename),
    myTimeThreshold(timeThreshold),
    mySpeedThreshold(speedThreshold) {
    // Update geometry;
    updateGeometry();
}


GNEDetectorE3::~GNEDetectorE3() {}


void
GNEDetectorE3::updateGeometry() {
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

    // Refresh element (neccesary to avoid grabbing problems)
    myViewNet->getNet()->refreshAdditional(this);

    // Clear all containers
    myShapeRotations.clear();
    myShapeLengths.clear();


    // iterate over entry childs and update their gemometries
    for (std::vector<GNEDetectorEntry*>::iterator i = myGNEDetectorEntrys.begin(); i != myGNEDetectorEntrys.end(); i++) {
        (*i)->updateGeometryByParent();
    }

    // iterate over entry childs and update their gemometries
    for (std::vector<GNEDetectorExit*>::iterator i = myGNEDetectorExits.begin(); i != myGNEDetectorExits.end(); i++) {
        (*i)->updateGeometryByParent();
    }

    // Update connection's geometry
    updateGeometryConnections();
}


Position
GNEDetectorE3::getPositionInView() const {
    return myPosition;
}


void
GNEDetectorE3::moveGeometry(const Position& oldPos, const Position &offset) {
    // restore old position, apply offset and update Geometry
    myPosition = oldPos;
    myPosition.add(offset);
    updateGeometry();
}


void
GNEDetectorE3::commitGeometryMoving(const Position& oldPos, GNEUndoList* undoList) {
    undoList->p_begin("position of " + toString(getTag()));
    undoList->p_add(new GNEChange_Attribute(this, SUMO_ATTR_X, toString(myPosition.x()), true, toString(oldPos.x())));
    undoList->p_add(new GNEChange_Attribute(this, SUMO_ATTR_Y, toString(myPosition.y()), true, toString(oldPos.y())));
    undoList->p_end();
    // Refresh element
    myViewNet->getNet()->refreshAdditional(this);
}


void
GNEDetectorE3::writeAdditional(OutputDevice& device) const {
    // Only save E3 if have Entry/Exits
    if ((myGNEDetectorEntrys.size() + myGNEDetectorExits.size()) > 0) {
        // Write parameters
        device.openTag(getTag());
        device.writeAttr(SUMO_ATTR_ID, getID());
        device.writeAttr(SUMO_ATTR_FREQUENCY, myFreq);
        if (!myFilename.empty()) {
            device.writeAttr(SUMO_ATTR_FILE, myFilename);
        }
        device.writeAttr(SUMO_ATTR_HALTING_TIME_THRESHOLD, myTimeThreshold);
        device.writeAttr(SUMO_ATTR_HALTING_SPEED_THRESHOLD, mySpeedThreshold);
        device.writeAttr(SUMO_ATTR_X, myPosition.x());
        device.writeAttr(SUMO_ATTR_Y, myPosition.y());

        // Write entrys
        for (auto i : myGNEDetectorEntrys) {
            device.openTag(i->getTag());
            device.writeAttr(SUMO_ATTR_LANE, i->getAttribute(SUMO_ATTR_LANE));
            device.writeAttr(SUMO_ATTR_POSITION, i->getAttribute(SUMO_ATTR_POSITION));
            device.writeAttr(SUMO_ATTR_FRIENDLY_POS, i->getAttribute(SUMO_ATTR_FRIENDLY_POS));
            device.closeTag();
        }

        // Write exits
        for (auto i : myGNEDetectorExits) {
            device.openTag(i->getTag());
            device.writeAttr(SUMO_ATTR_LANE, i->getAttribute(SUMO_ATTR_LANE));
            device.writeAttr(SUMO_ATTR_POSITION, i->getAttribute(SUMO_ATTR_POSITION));
            device.writeAttr(SUMO_ATTR_FRIENDLY_POS, i->getAttribute(SUMO_ATTR_FRIENDLY_POS));
            device.closeTag();
        }

        // Close E3 tag
        device.closeTag();
    } else {
        WRITE_WARNING(toString(getTag()) + " with ID '" + getID() + "' cannot be writed in additional file because doesn't have childs.");
    }
}


std::string
GNEDetectorE3::generateEntryID() {
    int counter = 0;
    while (myViewNet->getNet()->getAdditional(SUMO_TAG_DET_ENTRY, getID() + toString(SUMO_TAG_DET_ENTRY) + toString(counter)) != NULL) {
        counter++;
    }
    return (getID() + toString(SUMO_TAG_DET_ENTRY) + toString(counter));
}


std::string
GNEDetectorE3::generateExitID() {
    int counter = 0;
    while (myViewNet->getNet()->getAdditional(SUMO_TAG_DET_EXIT, getID() + toString(SUMO_TAG_DET_EXIT) + toString(counter)) != NULL) {
        counter++;
    }
    return (getID() + toString(SUMO_TAG_DET_EXIT) + toString(counter));
}


const std::string&
GNEDetectorE3::getParentName() const {
    return myViewNet->getNet()->getMicrosimID();
}


void
GNEDetectorE3::addEntryChild(GNEDetectorEntry* entry) {
    // Check that entry is valid and doesn't exist previously
    if (entry == NULL) {
        throw InvalidArgument("Trying to add an empty " + toString(SUMO_TAG_DET_ENTRY) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else if (std::find(myGNEDetectorEntrys.begin(), myGNEDetectorEntrys.end(), entry) != myGNEDetectorEntrys.end()) {
        throw InvalidArgument("Trying to add a duplicated " + toString(SUMO_TAG_DET_ENTRY) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else {
        myGNEDetectorEntrys.push_back(entry);
    }
}


void
GNEDetectorE3::removeEntryChild(GNEDetectorEntry* entry) {
    // Check that entry is valid and exist previously
    if (entry == NULL) {
        throw InvalidArgument("Trying to remove an empty " + toString(SUMO_TAG_DET_ENTRY) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else if (std::find(myGNEDetectorEntrys.begin(), myGNEDetectorEntrys.end(), entry) == myGNEDetectorEntrys.end()) {
        throw InvalidArgument("Trying to remove a non previously inserted " + toString(SUMO_TAG_DET_ENTRY) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else {
        myGNEDetectorEntrys.erase(std::find(myGNEDetectorEntrys.begin(), myGNEDetectorEntrys.end(), entry));
    }
}


void
GNEDetectorE3::addExitChild(GNEDetectorExit* exit) {
    // Check that exit is valid and doesn't exist previously
    if (exit == NULL) {
        throw InvalidArgument("Trying to add an empty " + toString(SUMO_TAG_DET_EXIT) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else if (std::find(myGNEDetectorExits.begin(), myGNEDetectorExits.end(), exit) != myGNEDetectorExits.end()) {
        throw InvalidArgument("Trying to add a duplicated " + toString(SUMO_TAG_DET_EXIT) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else {
        myGNEDetectorExits.push_back(exit);
    }
}


void
GNEDetectorE3::removeExitChild(GNEDetectorExit* exit) {
    // Check that exit is valid and exist previously
    if (exit == NULL) {
        throw InvalidArgument("Trying to remove an empty " + toString(SUMO_TAG_DET_EXIT) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else if (std::find(myGNEDetectorExits.begin(), myGNEDetectorExits.end(), exit) == myGNEDetectorExits.end()) {
        throw InvalidArgument("Trying to remove a non previously inserted " + toString(SUMO_TAG_DET_EXIT) + " child in " + toString(SUMO_TAG_E3DETECTOR) + " with ID='" + getID() + "'");
    } else {
        myGNEDetectorExits.erase(std::find(myGNEDetectorExits.begin(), myGNEDetectorExits.end(), exit));
    }
}


int
GNEDetectorE3::getNumberOfEntryChilds() const {
    return (int)myGNEDetectorEntrys.size();
}


int
GNEDetectorE3::getNumberOfExitChilds() const {
    return (int)myGNEDetectorExits.size();
}


void
GNEDetectorE3::drawGL(const GUIVisualizationSettings& s) const {
    // Start drawing adding an gl identificator
    glPushName(getGlID());

    // Add a draw matrix for drawing logo
    glPushMatrix();
    glTranslated(myShape[0].x(), myShape[0].y(), getType());
    glColor3d(1, 1, 1);
    glRotated(180, 0, 0, 1);
    // Draw icon depending of detector is or isn't selected
    if (isAdditionalSelected()) {
        GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_E3SELECTED), 1);
    } else {
        GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_E3), 1);
    }

    // Pop logo matrix
    glPopMatrix();

    // Show Lock icon depending of the Edit mode
    drawLockIcon(0.4);

    // Draw connections
    drawParentAndChildrenConnections();

    // Pop name
    glPopName();

    // Draw name
    drawName(getCenteringBoundary().getCenter(), s.scale, s.addName);
}


std::string
GNEDetectorE3::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_X:
            return toString(myPosition.x());
        case SUMO_ATTR_Y:
            return toString(myPosition.y());
        case SUMO_ATTR_FREQUENCY:
            return toString(myFreq);
        case SUMO_ATTR_FILE:
            return myFilename;
        case SUMO_ATTR_HALTING_TIME_THRESHOLD:
            return toString(myTimeThreshold);
        case SUMO_ATTR_HALTING_SPEED_THRESHOLD:
            return toString(mySpeedThreshold);
        case GNE_ATTR_BLOCK_MOVEMENT:
            return toString(myBlocked);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEDetectorE3::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_FREQUENCY:
        case SUMO_ATTR_X:
        case SUMO_ATTR_Y:
        case SUMO_ATTR_FILE:
        case SUMO_ATTR_HALTING_TIME_THRESHOLD:
        case SUMO_ATTR_HALTING_SPEED_THRESHOLD:
        case GNE_ATTR_BLOCK_MOVEMENT:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            updateGeometry();
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEDetectorE3::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            if (isValidID(value) && (myViewNet->getNet()->getAdditional(getTag(), value) == NULL)) {
                return true;
            } else {
                return false;
            }
        case SUMO_ATTR_X:
            return canParse<double>(value);
        case SUMO_ATTR_Y:
            return canParse<double>(value);
        case SUMO_ATTR_FREQUENCY:
            return canParse<double>(value) && (parse<double>(value) >= 0);
        case SUMO_ATTR_FILE:
            return isValidFilename(value);
        case SUMO_ATTR_HALTING_TIME_THRESHOLD:
            return canParse<double>(value) && (parse<double>(value) >= 0);
        case SUMO_ATTR_HALTING_SPEED_THRESHOLD:
            return canParse<double>(value) && (parse<double>(value) >= 0);
        case GNE_ATTR_BLOCK_MOVEMENT:
            return canParse<bool>(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEDetectorE3::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            setAdditionalID(value);
            // Change Ids of all Entry/Exits childs
            for (std::vector<GNEDetectorEntry*>::iterator i = myGNEDetectorEntrys.begin(); i != myGNEDetectorEntrys.end(); i++) {
                (*i)->setAdditionalID(generateEntryID());
            }
            for (std::vector<GNEDetectorExit*>::iterator i = myGNEDetectorExits.begin(); i != myGNEDetectorExits.end(); i++) {
                (*i)->setAdditionalID(generateExitID());
            }
            break;
        case SUMO_ATTR_X:
            myPosition.setx(parse<double>(value));
            updateGeometry();
            getViewNet()->update();
            break;
        case SUMO_ATTR_Y:
            myPosition.sety(parse<double>(value));
            updateGeometry();
            getViewNet()->update();
            break;
        case SUMO_ATTR_FREQUENCY:
            myFreq = parse<double>(value);
            break;
        case SUMO_ATTR_FILE:
            myFilename = value;
            break;
        case SUMO_ATTR_HALTING_TIME_THRESHOLD:
            myTimeThreshold = parse<double>(value);
            break;
        case SUMO_ATTR_HALTING_SPEED_THRESHOLD:
            mySpeedThreshold = parse<double>(value);
            break;
        case GNE_ATTR_BLOCK_MOVEMENT:
            myBlocked = parse<bool>(value);
            getViewNet()->update();
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEDetectorE3::updateGeometryConnections() {
    myConnectionPositions.clear();
    // Iterate over Entrys
    for (std::vector<GNEDetectorEntry*>::iterator i = myGNEDetectorEntrys.begin(); i != myGNEDetectorEntrys.end(); i++) {
        std::vector<Position> posConnection;
        double A = std::abs((*i)->getPositionInView().x() - getPositionInView().x());
        double B = std::abs((*i)->getPositionInView().y() - getPositionInView().y());
        // Set positions of connection's vertex. Connection is build from Entry to E3
        posConnection.push_back((*i)->getPositionInView());
        if (getPositionInView().x() > (*i)->getPositionInView().x()) {
            if (getPositionInView().y() > (*i)->getPositionInView().y()) {
                posConnection.push_back(Position((*i)->getPositionInView().x() + A, (*i)->getPositionInView().y()));
            } else {
                posConnection.push_back(Position((*i)->getPositionInView().x(), (*i)->getPositionInView().y() - B));
            }
        } else {
            if (getPositionInView().y() > (*i)->getPositionInView().y()) {
                posConnection.push_back(Position((*i)->getPositionInView().x(), (*i)->getPositionInView().y() + B));
            } else {
                posConnection.push_back(Position((*i)->getPositionInView().x() - A, (*i)->getPositionInView().y()));
            }
        }
        posConnection.push_back(getPositionInView());
        myConnectionPositions.push_back(posConnection);
    }
    // Iterate over exits
    for (std::vector<GNEDetectorExit*>::iterator i = myGNEDetectorExits.begin(); i != myGNEDetectorExits.end(); i++) {
        std::vector<Position> posConnection;
        double A = std::abs((*i)->getPositionInView().x() - getPositionInView().x());
        double B = std::abs((*i)->getPositionInView().y() - getPositionInView().y());
        // Set positions of connection's vertex. Connection is build from Entry to E3
        posConnection.push_back((*i)->getPositionInView());
        if (getPositionInView().x() > (*i)->getPositionInView().x()) {
            if (getPositionInView().y() > (*i)->getPositionInView().y()) {
                posConnection.push_back(Position((*i)->getPositionInView().x() + A, (*i)->getPositionInView().y()));
            } else {
                posConnection.push_back(Position((*i)->getPositionInView().x(), (*i)->getPositionInView().y() - B));
            }
        } else {
            if (getPositionInView().y() > (*i)->getPositionInView().y()) {
                posConnection.push_back(Position((*i)->getPositionInView().x(), (*i)->getPositionInView().y() + B));
            } else {
                posConnection.push_back(Position((*i)->getPositionInView().x() - A, (*i)->getPositionInView().y()));
            }
        }
        posConnection.push_back(getPositionInView());
        myConnectionPositions.push_back(posConnection);
    }
}

/****************************************************************************/
