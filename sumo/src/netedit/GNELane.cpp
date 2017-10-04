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
/// @file    GNELane.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
/// @version $Id$
///
// A class for visualizing Lane geometry (adapted from GNELaneWrapper)
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
#include <foreign/polyfonts/polyfonts.h>
#include <utils/foxtools/MFXUtils.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/common/ToString.h>
#include <utils/geom/GeomHelper.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/gui/images/GUITextureSubSys.h>

#include "GNELane.h"
#include "GNEEdge.h"
#include "GNEJunction.h"
#include "GNETLSEditorFrame.h"
#include "GNEInternalLane.h"
#include "GNEUndoList.h"
#include "GNENet.h"
#include "GNEChange_Attribute.h"
#include "GNEViewNet.h"
#include "GNEViewParent.h"
#include "GNEConnection.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================

// Object implementation
FXIMPLEMENT(GNELane, FXDelegator, 0, 0)

// ===========================================================================
// method definitions
// ===========================================================================

GNELane::GNELane(GNEEdge& edge, const int index) :
    GNENetElement(edge.getNet(), edge.getNBEdge()->getLaneID(index), GLO_LANE, SUMO_TAG_LANE, ICON_LANE),
    myParentEdge(edge),
    myIndex(index),
    mySpecialColor(0),
    myTLSEditor(0) {
}

GNELane::GNELane() :
    GNENetElement(NULL, "dummyConstructorGNELane", GLO_LANE, SUMO_TAG_LANE, ICON_LOCATEEDGE),
    myParentEdge(*static_cast<GNEEdge*>(0)),
    myIndex(-1),
    mySpecialColor(0),
    myTLSEditor(0) {
}


GNELane::~GNELane() {
}


void
GNELane::drawLinkNo(const GUIVisualizationSettings& s) const {
    const std::vector<NBEdge::Connection>& cons = myParentEdge.getNBEdge()->getConnectionsFromLane(myIndex);
    int noLinks = (int)cons.size();
    if (noLinks == 0) {
        return;
    }
    // draw all links
    glPushMatrix();
    glTranslated(0, 0, GLO_LANE + 0.1);
    double w = myParentEdge.getNBEdge()->getLaneWidth(myIndex) / (double) noLinks;
    double x1 = myParentEdge.getNBEdge()->getLaneWidth(myIndex) / 2;
    const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
    for (int i = noLinks; --i >= 0;) {
        double x2 = x1 - (double)(w / 2.);
        const int linkIndex = myParentEdge.getNBEdge()->getToNode()->getConnectionIndex(myParentEdge.getNBEdge(),
                              cons[lefthand ? noLinks - 1 - i : i]);
        GLHelper::drawTextAtEnd(toString(linkIndex), getShape(), x2, s.drawLinkJunctionIndex.size, s.drawLinkJunctionIndex.color);
        x1 -= w;
    }
    glPopMatrix();
}


void
GNELane::drawTLSLinkNo(const GUIVisualizationSettings& s) const {
    const std::vector<NBEdge::Connection>& cons = myParentEdge.getNBEdge()->getConnectionsFromLane(myIndex);
    int noLinks = (int)cons.size();
    if (noLinks == 0) {
        return;
    }
    // draw all links
    glPushMatrix();
    glTranslated(0, 0, GLO_LANE + 0.1);
    double w = myParentEdge.getNBEdge()->getLaneWidth(myIndex) / (double) noLinks;
    double x1 = myParentEdge.getNBEdge()->getLaneWidth(myIndex) / 2;
    const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
    for (int i = noLinks; --i >= 0;) {
        double x2 = x1 - (double)(w / 2.);
        int linkNo = cons[lefthand ? noLinks - 1 - i : i].tlLinkNo;
        GLHelper::drawTextAtEnd(toString(linkNo), getShape(), x2, s.drawLinkTLIndex.size, s.drawLinkTLIndex.color);
        x1 -= w;
    }
    glPopMatrix();
}


void
GNELane::drawLinkRules() const {
}


void
GNELane::drawArrows() const {
    const Position& end = getShape().back();
    const Position& f = getShape()[-2];
    double rot = (double) atan2((end.x() - f.x()), (f.y() - end.y())) * (double) 180.0 / (double)M_PI;
    glPushMatrix();
    glPushName(0);
    glTranslated(0, 0, GLO_JUNCTION + .1); // must draw on top of junction shape
    glColor3d(1, 1, 1);
    glTranslated(end.x(), end.y(), 0);
    glRotated(rot, 0, 0, 1);

    // draw all links
    const std::vector<NBEdge::Connection>& edgeCons = myParentEdge.getNBEdge()->myConnections;
    NBNode* dest = myParentEdge.getNBEdge()->myTo;
    for (std::vector<NBEdge::Connection>::const_iterator i = edgeCons.begin(); i != edgeCons.end(); ++i) {
        if ((*i).fromLane == myIndex) {
            LinkDirection dir = dest->getDirection(myParentEdge.getNBEdge(), i->toEdge, OptionsCont::getOptions().getBool("lefthand"));
            switch (dir) {
                case LINKDIR_STRAIGHT:
                    GLHelper::drawBoxLine(Position(0, 4), 0, 2, .05);
                    GLHelper::drawTriangleAtEnd(Position(0, 4), Position(0, 1), (double) 1, (double) .25);
                    break;
                case LINKDIR_LEFT:
                    GLHelper::drawBoxLine(Position(0, 4), 0, 1.5, .05);
                    GLHelper::drawBoxLine(Position(0, 2.5), 90, 1, .05);
                    GLHelper::drawTriangleAtEnd(Position(0, 2.5), Position(1.5, 2.5), (double) 1, (double) .25);
                    break;
                case LINKDIR_RIGHT:
                    GLHelper::drawBoxLine(Position(0, 4), 0, 1.5, .05);
                    GLHelper::drawBoxLine(Position(0, 2.5), -90, 1, .05);
                    GLHelper::drawTriangleAtEnd(Position(0, 2.5), Position(-1.5, 2.5), (double) 1, (double) .25);
                    break;
                case LINKDIR_TURN:
                    GLHelper::drawBoxLine(Position(0, 4), 0, 1.5, .05);
                    GLHelper::drawBoxLine(Position(0, 2.5), 90, .5, .05);
                    GLHelper::drawBoxLine(Position(0.5, 2.5), 180, 1, .05);
                    GLHelper::drawTriangleAtEnd(Position(0.5, 2.5), Position(0.5, 4), (double) 1, (double) .25);
                    break;
                case LINKDIR_TURN_LEFTHAND:
                    GLHelper::drawBoxLine(Position(0, 4), 0, 1.5, .05);
                    GLHelper::drawBoxLine(Position(0, 2.5), -90, 1, .05);
                    GLHelper::drawBoxLine(Position(-0.5, 2.5), -180, 1, .05);
                    GLHelper::drawTriangleAtEnd(Position(-0.5, 2.5), Position(-0.5, 4), (double) 1, (double) .25);
                    break;
                case LINKDIR_PARTLEFT:
                    GLHelper::drawBoxLine(Position(0, 4), 0, 1.5, .05);
                    GLHelper::drawBoxLine(Position(0, 2.5), 45, .7, .05);
                    GLHelper::drawTriangleAtEnd(Position(0, 2.5), Position(1.2, 1.3), (double) 1, (double) .25);
                    break;
                case LINKDIR_PARTRIGHT:
                    GLHelper::drawBoxLine(Position(0, 4), 0, 1.5, .05);
                    GLHelper::drawBoxLine(Position(0, 2.5), -45, .7, .05);
                    GLHelper::drawTriangleAtEnd(Position(0, 2.5), Position(-1.2, 1.3), (double) 1, (double) .25);
                    break;
                case LINKDIR_NODIR:
                    GLHelper::drawBoxLine(Position(1, 5.8), 245, 2, .05);
                    GLHelper::drawBoxLine(Position(-1, 5.8), 115, 2, .05);
                    glTranslated(0, 5, 0);
                    GLHelper::drawOutlineCircle(0.9, 0.8, 32);
                    glTranslated(0, -5, 0);
                    break;
            }
        }
    }
    glPopName();
    glPopMatrix();
}


void
GNELane::drawLane2LaneConnections() const {
    glPushMatrix();
    glPushName(0);
    glTranslated(0, 0, GLO_JUNCTION + .1); // must draw on top of junction shape
    std::vector<NBEdge::Connection> connections = myParentEdge.getNBEdge()->getConnectionsFromLane(myIndex);
    NBNode* node = myParentEdge.getNBEdge()->getToNode();
    const Position& startPos = getShape()[-1];
    for (std::vector<NBEdge::Connection>::iterator it = connections.begin(); it != connections.end(); it++) {
        const LinkState state = node->getLinkState(
                                    myParentEdge.getNBEdge(), it->toEdge, it->fromLane, it->toLane, it->mayDefinitelyPass, it->tlID);
        switch (state) {
            case LINKSTATE_TL_OFF_NOSIGNAL:
                glColor3d(1, 1, 0);
                break;
            case LINKSTATE_TL_OFF_BLINKING:
                glColor3d(0, 1, 1);
                break;
            case LINKSTATE_MAJOR:
                glColor3d(1, 1, 1);
                break;
            case LINKSTATE_MINOR:
                glColor3d(.4, .4, .4);
                break;
            case LINKSTATE_STOP:
                glColor3d(.7, .4, .4);
                break;
            case LINKSTATE_EQUAL:
                glColor3d(.7, .7, .7);
                break;
            case LINKSTATE_ALLWAY_STOP:
                glColor3d(.7, .7, 1);
            case LINKSTATE_ZIPPER:
                glColor3d(.75, .5, 0.25);
                break;
            default:
                throw ProcessError("Unexpected LinkState '" + toString(state) + "'");
        }
        const Position& endPos = it->toEdge->getLaneShape(it->toLane)[0];
        glBegin(GL_LINES);
        glVertex2d(startPos.x(), startPos.y());
        glVertex2d(endPos.x(), endPos.y());
        glEnd();
        GLHelper::drawTriangleAtEnd(startPos, endPos, (double) 1.5, (double) .2);
    }
    glPopName();
    glPopMatrix();
}


void
GNELane::drawGL(const GUIVisualizationSettings& s) const {
    // Push draw matrix 1
    glPushMatrix();
    // Push name
    glPushName(getGlID());
    // Traslate to fromt
    glTranslated(0, 0, getType());
    // Check if edge parent or this lane is selected
    const bool selectedEdge = gSelected.isSelected(myParentEdge.getType(), myParentEdge.getGlID());
    const bool selected = gSelected.isSelected(getType(), getGlID());
    // Set color
    if (mySpecialColor != 0) {
        // If special color is enabled, set it
        GLHelper::setColor(*mySpecialColor);
    } else if (selected && s.laneColorer.getActive() != 1) {
        // override with special colors (unless the color scheme is based on selection)
        GLHelper::setColor(GNENet::selectedLaneColor);
    } else if (selectedEdge && s.laneColorer.getActive() != 1) {
        // override with special colors (unless the color scheme is based on selection)
        GLHelper::setColor(GNENet::selectionColor);
    } else {
        // Get normal lane color
        const GUIColorer& c = s.laneColorer;
        if (!setFunctionalColor(c.getActive()) && !setMultiColor(c)) {
            GLHelper::setColor(c.getScheme().getColor(getColorValue(c.getActive())));
        }
    }

    // start drawing lane checking whether it is not too small
    const double selectionScale = selected || selectedEdge ? s.selectionScale : 1;
    double exaggeration = selectionScale * s.laneWidthExaggeration; // * s.laneScaler.getScheme().getColor(getScaleValue(s.laneScaler.getActive()));
    // XXX apply usefull scale values
    //exaggeration *= s.laneScaler.getScheme().getColor(getScaleValue(s.laneScaler.getActive()));

    // recognize full transparency and simply don't draw
    GLfloat color[4];
    glGetFloatv(GL_CURRENT_COLOR, color);
    if (color[3] == 0 || s.scale * exaggeration < s.laneMinSize) {
        // Pop draw matrix 1
        glPopMatrix();
    } else if (s.scale * exaggeration < 1.) {
        // draw as lines, depending of myShapeColors
        if (myShapeColors.size() > 0) {
            GLHelper::drawLine(getShape(), myShapeColors);
        } else {
            GLHelper::drawLine(getShape());
        }
        // Pop draw matrix 1
        glPopMatrix();
    } else {
        if (drawAsRailway(s)) {
            // draw as railway
            const double halfRailWidth = 0.725 * exaggeration;
            // Draw box depending of myShapeColors
            if (myShapeColors.size() > 0) {
                GLHelper::drawBoxLines(getShape(), myShapeRotations, myShapeLengths, myShapeColors, halfRailWidth);
            } else {
                GLHelper::drawBoxLines(getShape(), myShapeRotations, myShapeLengths, halfRailWidth);
            }
            // Save current color
            RGBColor current = GLHelper::getColor();
            // Set white color
            glColor3d(1, 1, 1);
            // Traslate matrix 1
            glTranslated(0, 0, .1);
            // Draw Box
            GLHelper::drawBoxLines(getShape(), myShapeRotations, myShapeLengths, halfRailWidth - 0.2);
            // Set current color back
            GLHelper::setColor(current);
            // Draw crossties
            drawCrossties(0.3 * exaggeration, 1 * exaggeration, 1 * exaggeration);
        } else {
            // Draw as a normal lane, and reduce width to make sure that a selected edge can still be seen
            const double halfWidth = exaggeration * (myParentEdge.getNBEdge()->getLaneWidth(myIndex) / 2 - (selectedEdge ? .3 : 0));
            if (myShapeColors.size() > 0) {
                GLHelper::drawBoxLines(getShape(), myShapeRotations, myShapeLengths, myShapeColors, halfWidth);
            } else {
                GLHelper::drawBoxLines(getShape(), myShapeRotations, myShapeLengths, halfWidth);
            }
        }
        // Pop draw matrix 1
        glPopMatrix();
        // only draw details depending of the scale
        if (s.scale >= 10) {
            // if exaggeration is 1, draw drawMarkings
            if (s.laneShowBorders && exaggeration == 1) {
                drawMarkings(selectedEdge, exaggeration);
            }
            // draw ROWs only if target junction has a valid logic)
            if (s.showLinkDecals && myParentEdge.getGNEJunctionDestiny()->isLogicValid() && s.scale > 3) {
                drawArrows();
            }
            // Draw direction indicators if the correspondient option is enabled
            if (s.showLaneDirection) {
                drawDirectionIndicators();
            }
            if (s.drawLinkJunctionIndex.show) {
                drawLinkNo(s);
            }
            if (s.drawLinkTLIndex.show) {
                drawTLSLinkNo(s);
            }
        }
        // If there are texture of restricted lanes to draw, and draw lane icons is enabled in options
        if ((OptionsCont::getOptions().getBool("disable-laneIcons") == false) && (myLaneRestrictedTexturePositions.size() > 0) && (s.scale >= 10)) {
            // Declare default width of icon (3)
            double iconWidth = 1;
            // Obtain width of icon, if width of lane is different
            if (myParentEdge.getNBEdge()->getLaneStruct(myIndex).width != -1) {
                iconWidth = myParentEdge.getNBEdge()->getLaneStruct(myIndex).width / 3;
            }
            // Draw list of icons
            for (int i = 0; i < (int)myLaneRestrictedTexturePositions.size(); i++) {
                // Push draw matrix 2
                glPushMatrix();
                // Set white color
                glColor3d(1, 1, 1);
                // Traslate matrix 2
                glTranslated(myLaneRestrictedTexturePositions.at(i).x(), myLaneRestrictedTexturePositions.at(i).y(), getType() + 0.1);
                // Rotate matrix 2
                glRotated(myLaneRestrictedTextureRotations.at(i), 0, 0, -1);
                glRotated(-90, 0, 0, 1);
                // draw texture box depending of type of restriction
                if (isRestricted(SVC_PEDESTRIAN)) {
                    GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_LANEPEDESTRIAN), iconWidth);
                } else if (isRestricted(SVC_BICYCLE)) {
                    GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_LANEBIKE), iconWidth);
                } else if (isRestricted(SVC_BUS)) {
                    GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_LANEBUS), iconWidth);
                }
                // Pop draw matrix 2
                glPopMatrix();
            }
        }
    }
    // Pop Name
    glPopName();
}


void
GNELane::drawMarkings(const bool& selectedEdge, double scale) const {
    glPushMatrix();
    glTranslated(0, 0, GLO_EDGE);

    const double halfWidth = myParentEdge.getNBEdge()->getLaneWidth(myIndex) * 0.5;
    // optionally draw inverse markings
    if (myIndex > 0 && (myParentEdge.getNBEdge()->getPermissions(myIndex - 1) & myParentEdge.getNBEdge()->getPermissions(myIndex)) != 0) {
        double mw = (halfWidth + SUMO_const_laneOffset + .01) * scale;
        int e = (int) getShape().size() - 1;
        for (int i = 0; i < e; ++i) {
            glPushMatrix();
            glTranslated(getShape()[i].x(), getShape()[i].y(), 0.1);
            glRotated(myShapeRotations[i], 0, 0, 1);
            for (double t = 0; t < myShapeLengths[i]; t += 6) {
                const double length = MIN2((double)3, myShapeLengths[i] - t);
                glBegin(GL_QUADS);
                glVertex2d(-mw, -t);
                glVertex2d(-mw, -t - length);
                glVertex2d(halfWidth * 0.5 * scale, -t - length);
                glVertex2d(halfWidth * 0.5 * scale, -t);
                glEnd();
            }
            glPopMatrix();
        }
    }

    // draw white boundings (and white markings) depending on selection
    if (selectedEdge) {
        glTranslated(0, 0, 0.2); // draw selection on top of regular markings
        GLHelper::setColor(GNENet::selectionColor);
    } else {
        glColor3d(1, 1, 1);
    }

    GLHelper::drawBoxLines(
        getShape(),
        getShapeRotations(),
        getShapeLengths(),
        (halfWidth + SUMO_const_laneOffset) * scale);
    glPopMatrix();

}


GUIGLObjectPopupMenu*
GNELane::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
    buildPopupHeader(ret, app);
    buildCenterPopupEntry(ret);
    new FXMenuCommand(ret, ("Copy " + toString(SUMO_TAG_EDGE) + " name to clipboard").c_str(), 0, ret, MID_COPY_EDGE_NAME);
    buildNameCopyPopupEntry(ret);
    buildSelectionPopupEntry(ret);
    buildPositionCopyEntry(ret, false);
    const int editMode = parent.getVisualisationSettings()->editMode;
    myTLSEditor = 0;
    if (editMode != GNE_MODE_CONNECT && editMode != GNE_MODE_TLS && editMode != GNE_MODE_CREATE_EDGE) {
        // Get icons
        FXIcon* pedestrianIcon = GUIIconSubSys::getIcon(ICON_LANEPEDESTRIAN);
        FXIcon* bikeIcon = GUIIconSubSys::getIcon(ICON_LANEBIKE);
        FXIcon* busIcon = GUIIconSubSys::getIcon(ICON_LANEBUS);
        // Create basic commands
        std::string plural = myNet->retrieveEdges(true).size() > 1? "s" : "";
        new FXMenuCommand(ret, ("Split " + toString(SUMO_TAG_EDGE) + " here").c_str(), 0, &parent, MID_GNE_EDGE_SPLIT);
        new FXMenuCommand(ret, ("Split " + toString(SUMO_TAG_EDGE) + "s in both direction here").c_str(), 0, &parent, MID_GNE_EDGE_SPLIT_BIDI);
        new FXMenuCommand(ret, ("Reverse " + toString(SUMO_TAG_EDGE)).c_str(), 0, &parent, MID_GNE_EDGE_REVERSE);
        new FXMenuCommand(ret, "Add reverse direction", 0, &parent, MID_GNE_EDGE_ADD_REVERSE);
        new FXMenuCommand(ret, "Set geometry endpoint here", 0, &parent, MID_GNE_EDGE_SET_ENDPOINT);
        new FXMenuCommand(ret, "Restore geometry endpoint", 0, &parent, MID_GNE_EDGE_RESET_ENDPOINT);
        new FXMenuCommand(ret, ("Straighten " + toString(SUMO_TAG_EDGE) + plural).c_str(), 0, &parent, MID_GNE_EDGE_STRAIGHTEN);
        if (gSelected.isSelected(GLO_LANE, getGlID())) {
            std::string plural = myNet->retrieveLanes(true).size() > 1? "s" : "";
            new FXMenuCommand(ret, ("Duplicate selected " + toString(SUMO_TAG_LANE) + plural).c_str(), 0, &parent, MID_GNE_LANE_DUPLICATE);
            // Create panel for lane operations
            FXMenuPane* addSpecialLanes = new FXMenuPane(ret);
            ret->insertMenuPaneChild(addSpecialLanes);
            FXMenuPane* removeSpecialLanes = new FXMenuPane(ret);
            ret->insertMenuPaneChild(removeSpecialLanes);
            FXMenuPane* transformSlanes = new FXMenuPane(ret);
            ret->insertMenuPaneChild(transformSlanes);
            // Create menu comands for all add special lanes
            new FXMenuCommand(addSpecialLanes, "Sidewalks", pedestrianIcon, &parent, MID_GNE_LANE_ADD_SIDEWALK);
            new FXMenuCommand(addSpecialLanes, "Bikelanes", bikeIcon, &parent, MID_GNE_LANE_ADD_BIKE);
            new FXMenuCommand(addSpecialLanes, "Buslanes", busIcon, &parent, MID_GNE_LANE_ADD_BUS);
            // Create menu comands for all remove special lanes and disable it
            new FXMenuCommand(removeSpecialLanes, "Sidewalks", pedestrianIcon, &parent, MID_GNE_LANE_REMOVE_SIDEWALK);
            new FXMenuCommand(removeSpecialLanes, "Bikelanes", bikeIcon, &parent, MID_GNE_LANE_REMOVE_BIKE);
            new FXMenuCommand(removeSpecialLanes, "Buslanes", busIcon, &parent, MID_GNE_LANE_REMOVE_BUS);
            // Create menu comands for all trasform special lanes and disable it
            new FXMenuCommand(transformSlanes, "Sidewalks", pedestrianIcon, &parent, MID_GNE_LANE_TRANSFORM_SIDEWALK);
            new FXMenuCommand(transformSlanes, "Bikelanes", bikeIcon, &parent, MID_GNE_LANE_TRANSFORM_BIKE);
            new FXMenuCommand(transformSlanes, "Buslanes", busIcon, &parent, MID_GNE_LANE_TRANSFORM_BUS);
            // add menuCascade for lane operations
            new FXMenuCascade(ret, ("add restricted " + toString(SUMO_TAG_LANE) + "s").c_str(), 0, addSpecialLanes);
            new FXMenuCascade(ret, ("remove restricted " + toString(SUMO_TAG_LANE) + "s").c_str(), 0, removeSpecialLanes);
            new FXMenuCascade(ret, ("transform to restricted " + toString(SUMO_TAG_LANE) + "s").c_str(), 0, transformSlanes);
        } else {
            new FXMenuCommand(ret, ("Duplicate " + toString(SUMO_TAG_LANE)).c_str(), 0, &parent, MID_GNE_LANE_DUPLICATE);
            // Declare flags
            bool edgeHasSidewalk = myParentEdge.hasRestrictedLane(SVC_PEDESTRIAN);
            bool edgeHasBikelane = myParentEdge.hasRestrictedLane(SVC_BICYCLE);
            bool edgeHasBuslane = myParentEdge.hasRestrictedLane(SVC_BUS);
            // Create panel for lane operations and insert it in ret
            FXMenuPane* addSpecialLanes = new FXMenuPane(ret);
            ret->insertMenuPaneChild(addSpecialLanes);
            FXMenuPane* removeSpecialLanes = new FXMenuPane(ret);
            ret->insertMenuPaneChild(removeSpecialLanes);
            FXMenuPane* transformSlanes = new FXMenuPane(ret);
            ret->insertMenuPaneChild(transformSlanes);
            // Create menu comands for all add special lanes
            FXMenuCommand* addSidewalk = new FXMenuCommand(addSpecialLanes, "Sidewalk", pedestrianIcon, &parent, MID_GNE_LANE_ADD_SIDEWALK);
            FXMenuCommand* addBikelane = new FXMenuCommand(addSpecialLanes, "Bikelane", bikeIcon, &parent, MID_GNE_LANE_ADD_BIKE);
            FXMenuCommand* addBuslane = new FXMenuCommand(addSpecialLanes, "Buslane", busIcon, &parent, MID_GNE_LANE_ADD_BUS);
            // Create menu comands for all remove special lanes and disable it
            FXMenuCommand* removeSidewalk = new FXMenuCommand(removeSpecialLanes, "Sidewalk", pedestrianIcon, &parent, MID_GNE_LANE_REMOVE_SIDEWALK);
            removeSidewalk->disable();
            FXMenuCommand* removeBikelane = new FXMenuCommand(removeSpecialLanes, "Bikelane", bikeIcon, &parent, MID_GNE_LANE_REMOVE_BIKE);
            removeBikelane->disable();
            FXMenuCommand* removeBuslane = new FXMenuCommand(removeSpecialLanes, "Buslane", busIcon, &parent, MID_GNE_LANE_REMOVE_BUS);
            removeBuslane->disable();
            // Create menu comands for all trasform special lanes and disable it
            FXMenuCommand* transformLaneToSidewalk = new FXMenuCommand(transformSlanes, "Sidewalk", pedestrianIcon, &parent, MID_GNE_LANE_TRANSFORM_SIDEWALK);
            FXMenuCommand* transformLaneToBikelane = new FXMenuCommand(transformSlanes, "Bikelane", bikeIcon, &parent, MID_GNE_LANE_TRANSFORM_BIKE);
            FXMenuCommand* transformLaneToBuslane = new FXMenuCommand(transformSlanes, "Buslane", busIcon, &parent, MID_GNE_LANE_TRANSFORM_BUS);
            // add menuCascade for lane operations
            FXMenuCascade* cascadeAddSpecialLane = new FXMenuCascade(ret, ("add restricted " + toString(SUMO_TAG_LANE)).c_str(), 0, addSpecialLanes);
            FXMenuCascade* cascadeRemoveSpecialLane = new FXMenuCascade(ret, ("remove restricted " + toString(SUMO_TAG_LANE)).c_str(), 0, removeSpecialLanes);
            new FXMenuCascade(ret, ("transform to restricted " + toString(SUMO_TAG_LANE)).c_str(), 0, transformSlanes);
            // Enable and disable options depending of current transform of the lane
            if (edgeHasSidewalk) {
                transformLaneToSidewalk->disable();
                addSidewalk->disable();
                removeSidewalk->enable();
            }
            if (edgeHasBikelane) {
                transformLaneToBikelane->disable();
                addBikelane->disable();
                removeBikelane->enable();
            }
            if (edgeHasBuslane) {
                transformLaneToBuslane->disable();
                addBuslane->disable();
                removeBuslane->enable();
            }
            // Check if cascade menus must be disabled
            if (edgeHasSidewalk && edgeHasBikelane && edgeHasBuslane) {
                cascadeAddSpecialLane->disable();
            }
            if (!edgeHasSidewalk && !edgeHasBikelane && !edgeHasBuslane) {
                cascadeRemoveSpecialLane->disable();
            }
        }
    } else if (editMode == GNE_MODE_TLS) {
        myTLSEditor = static_cast<GNEViewNet&>(parent).getViewParent()->getTLSEditorFrame();
        if (myTLSEditor->controlsEdge(myParentEdge)) {
            new FXMenuCommand(ret, "Select state for all links from this edge:", 0, 0, 0);
            const std::vector<std::string> names = GNEInternalLane::LinkStateNames.getStrings();
            for (std::vector<std::string>::const_iterator it = names.begin(); it != names.end(); it++) {
                FXuint state = GNEInternalLane::LinkStateNames.get(*it);
                FXMenuRadio* mc = new FXMenuRadio(ret, (*it).c_str(), this, FXDataTarget::ID_OPTION + state);
                mc->setSelBackColor(MFXUtils::getFXColor(GNEInternalLane::colorForLinksState(state)));
                mc->setBackColor(MFXUtils::getFXColor(GNEInternalLane::colorForLinksState(state)));
            }
        }
    } else {
        FXMenuCommand* mc = new FXMenuCommand(ret, "Additional options available in 'Inspect Mode'", 0, 0, 0);
        mc->handle(&parent, FXSEL(SEL_COMMAND, FXWindow::ID_DISABLE), 0);
    }
    // buildShowParamsPopupEntry(ret, false);
    new FXMenuSeparator(ret);
    const double pos = getShape().nearest_offset_to_point2D(parent.getPositionInformation());
    const double height = getShape().positionAtOffset2D(getShape().nearest_offset_to_point2D(parent.getPositionInformation())).z();
    new FXMenuCommand(ret, ("Shape pos: " + toString(pos)).c_str(), 0, 0, 0);
    new FXMenuCommand(ret, ("Length pos: " + toString(pos * getLaneParametricLength() / getLaneShapeLength())).c_str(), 0, 0, 0);
    new FXMenuCommand(ret, ("Height: " + toString(height)).c_str(), 0, 0, 0);
    // new FXMenuSeparator(ret);
    // buildPositionCopyEntry(ret, false);
    return ret;
}


GUIParameterTableWindow*
GNELane::getParameterWindow(GUIMainWindow& app, GUISUMOAbstractView&) {
    // get attributes
    std::vector<SumoXMLAttr> attributes = getAttrs();
    // Create table
    GUIParameterTableWindow* ret = new GUIParameterTableWindow(app, *this, (int)attributes.size());
    // Iterate over attributes
    for (auto i : attributes) {
        // Add attribute and set it dynamic if aren't unique
        if (GNEAttributeCarrier::isUnique(getTag(), i)) {
            ret->mkItem(toString(i).c_str(), false, getAttribute(i));
        } else {
            ret->mkItem(toString(i).c_str(), true, getAttribute(i));
        }
    }
    // close building
    ret->closeBuilding();
    return ret;
}


Boundary
GNELane::getCenteringBoundary() const {
    Boundary b = getShape().getBoxBoundary();
    b.grow(10);
    return b;
}


const PositionVector&
GNELane::getShape() const {
    return myParentEdge.getNBEdge()->getLaneShape(myIndex);
}


const std::vector<double>&
GNELane::getShapeRotations() const {
    return myShapeRotations;
}


const std::vector<double>&
GNELane::getShapeLengths() const {
    return myShapeLengths;
}


Boundary
GNELane::getBoundary() const {
    return myParentEdge.getNBEdge()->getLaneStruct(myIndex).shape.getBoxBoundary();
}


void
GNELane::updateGeometry() {
    // Clear containers
    myShapeRotations.clear();
    myShapeLengths.clear();
    myLaneRestrictedTexturePositions.clear();
    myLaneRestrictedTextureRotations.clear();
    //double length = myParentEdge.getLength(); // @todo see ticket #448
    // may be different from length

    // Obtain lane and shape rotations
    int segments = (int) getShape().size() - 1;
    if (segments >= 0) {
        myShapeRotations.reserve(segments);
        myShapeLengths.reserve(segments);
        for (int i = 0; i < segments; ++i) {
            const Position& f = getShape()[i];
            const Position& s = getShape()[i + 1];
            myShapeLengths.push_back(f.distanceTo2D(s));
            myShapeRotations.push_back((double) atan2((s.x() - f.x()), (f.y() - s.y())) * (double) 180.0 / (double)M_PI);
        }
    }
    // Update geometry of additionals vinculated with this lane
    for (AdditionalVector::iterator i = myAdditionals.begin(); i != myAdditionals.end(); i++) {
        (*i)->updateGeometry();
    }
    // In Move mode, connections aren't updated
    if (myNet->getViewNet() && myNet->getViewNet()->getCurrentEditMode() != GNE_MODE_MOVE) {
        // Update incoming connections of this lane
        std::vector<GNEConnection*> incomingConnections = getGNEIncomingConnections();
        for (std::vector<GNEConnection*>::iterator i = incomingConnections.begin(); i != incomingConnections.end(); i++) {
            (*i)->updateGeometry();
        }
        // Update outgoings connections of this lane
        std::vector<GNEConnection*> outGoingConnections = getGNEOutcomingConnections();
        for (std::vector<GNEConnection*>::iterator i = outGoingConnections.begin(); i != outGoingConnections.end(); i++) {
            (*i)->updateGeometry();
        }
    }
    // If lane has enought length for show textures of restricted lanes
    if ((getLaneShapeLength() > 4)) {
        // if lane is restricted
        if (isRestricted(SVC_PEDESTRIAN) || isRestricted(SVC_BICYCLE) || isRestricted(SVC_BUS)) {
            // get values for position and rotation of icons
            for (int i = 2; i < getLaneShapeLength() - 1; i += 15) {
                myLaneRestrictedTexturePositions.push_back(getShape().positionAtOffset(i));
                myLaneRestrictedTextureRotations.push_back(getShape().rotationDegreeAtOffset(i));
            }
        }
    }
}

int
GNELane::getIndex() const {
    return myIndex;
}

void
GNELane::setIndex(int index) {
    myIndex = index;
    setMicrosimID(myParentEdge.getNBEdge()->getLaneID(index));
}


double
GNELane::getSpeed() const {
    return myParentEdge.getNBEdge()->getLaneSpeed(myIndex);
}


double
GNELane::getLaneParametricLength() const  {
    double laneParametricLenght = myParentEdge.getNBEdge()->getLoadedLength();
    if (laneParametricLenght > 0) {
        return laneParametricLenght;
    } else {
        throw ProcessError("Lane Parametric Lenght cannot be never 0");
    }
}


double
GNELane::getLaneShapeLength() const {
    return getShape().length();
}

void
GNELane::addAdditionalChild(GNEAdditional* additional) {
    // Check if additional exist before remove
    if (std::find(myAdditionals.begin(), myAdditionals.end(), additional) == myAdditionals.end()) {
        myAdditionals.push_back(additional);
    } else {
        throw ProcessError(toString(getTag()) + " with ID='" + additional->getID() + "' was already inserted in lane with ID='" + getID() + "'");
    }
}


void
GNELane::removeAdditionalChild(GNEAdditional* additional) {
    AdditionalVector::iterator it = std::find(myAdditionals.begin(), myAdditionals.end(), additional);
    // Check if additional exist before remove
    if (it != myAdditionals.end()) {
        myAdditionals.erase(it);
    } else {
        throw ProcessError(toString(getTag()) + " with ID='" + additional->getID() + "' doesn't exist in lane with ID='" + getID() + "'");
    }
}


const std::vector<GNEAdditional*>&
GNELane::getAdditionalChilds() const {
    return myAdditionals;
}


bool
GNELane::isRestricted(SUMOVehicleClass vclass) const {
    return myParentEdge.getNBEdge()->getPermissions(myIndex) == vclass;
}


std::string
GNELane::getAttribute(SumoXMLAttr key) const {
    const NBEdge* edge = myParentEdge.getNBEdge();
    switch (key) {
        case SUMO_ATTR_ID:
            return getMicrosimID();
        case SUMO_ATTR_SPEED:
            return toString(edge->getLaneSpeed(myIndex));
        case SUMO_ATTR_ALLOW:
            return getVehicleClassNames(edge->getPermissions(myIndex));
        case SUMO_ATTR_DISALLOW:
            return getVehicleClassNames(invertPermissions(edge->getPermissions(myIndex)));
        case SUMO_ATTR_WIDTH:
            if (edge->getLaneStruct(myIndex).width == NBEdge::UNSPECIFIED_WIDTH) {
                return "default";
            } else {
                return toString(edge->getLaneStruct(myIndex).width);
            }
        case SUMO_ATTR_ENDOFFSET:
            return toString(edge->getLaneStruct(myIndex).endOffset);
        case SUMO_ATTR_ACCELERATION:
            return toString(edge->getLaneStruct(myIndex).accelRamp);
        case SUMO_ATTR_CUSTOMSHAPE:
            return toString(edge->getLaneStruct(myIndex).customShape);
        case SUMO_ATTR_INDEX:
            return toString(myIndex);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}

std::string
GNELane::getAttributeForSelection(SumoXMLAttr key) const {
    std::string result = getAttribute(key);
    if ((key == SUMO_ATTR_ALLOW || key == SUMO_ATTR_DISALLOW) && result.find("all") != std::string::npos) {
        result += " " + getVehicleClassNames(SVCAll, true);
    }
    return result;
}


void
GNELane::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    switch (key) {
        case SUMO_ATTR_ID:
            throw InvalidArgument("Modifying attribute '" + toString(key) + "' of " + toString(getTag()) + " isn't allowed");
        case SUMO_ATTR_SPEED:
        case SUMO_ATTR_ALLOW:
        case SUMO_ATTR_DISALLOW:
        case SUMO_ATTR_WIDTH:
        case SUMO_ATTR_ENDOFFSET:
        case SUMO_ATTR_ACCELERATION:
        case SUMO_ATTR_CUSTOMSHAPE:
        case SUMO_ATTR_INDEX:
            // no special handling
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNELane::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return false;
        case SUMO_ATTR_SPEED:
            return canParse<double>(value);
        case SUMO_ATTR_ALLOW:
        case SUMO_ATTR_DISALLOW:
            return canParseVehicleClasses(value);
        case SUMO_ATTR_WIDTH:
            if (value == "default") {
                return true;
            } else {
                return canParse<double>(value) && (isPositive<double>(value) || parse<double>(value) == NBEdge::UNSPECIFIED_WIDTH);
            }
        case SUMO_ATTR_ENDOFFSET:
            return canParse<double>(value);
        case SUMO_ATTR_ACCELERATION:
            return canParse<bool>(value);
        case SUMO_ATTR_CUSTOMSHAPE: {
            bool ok = true;
            PositionVector shape = GeomConvHelper::parseShapeReporting(value, "user-supplied position", 0, ok, true);
            return ok;
        }
        case SUMO_ATTR_INDEX:
            return canParse<int>(value) && (parse<int>(value) == myIndex);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNELane::setSpecialColor(const RGBColor* color) {
    mySpecialColor = color;
}

// ===========================================================================
// private
// ===========================================================================

void
GNELane::setAttribute(SumoXMLAttr key, const std::string& value) {
    NBEdge* edge = myParentEdge.getNBEdge();
    switch (key) {
        case SUMO_ATTR_ID:
            throw InvalidArgument("Modifying attribute '" + toString(key) + "' of " + toString(getTag()) + " isn't allowed");
        case SUMO_ATTR_SPEED:
            edge->setSpeed(myIndex, parse<double>(value));
            break;
        case SUMO_ATTR_ALLOW:
            edge->setPermissions(parseVehicleClasses(value), myIndex);
            updateGeometry();
            myNet->getViewNet()->update();
            break;
        case SUMO_ATTR_DISALLOW:
            edge->setPermissions(invertPermissions(parseVehicleClasses(value)), myIndex);
            updateGeometry();
            myNet->getViewNet()->update();
            break;
        case SUMO_ATTR_WIDTH:
            if (value == "default") {
                edge->setLaneWidth(myIndex, NBEdge::UNSPECIFIED_WIDTH);
            } else {
                edge->setLaneWidth(myIndex, parse<double>(value));
            }
            updateGeometry();
            myNet->getViewNet()->update();
            break;
        case SUMO_ATTR_ENDOFFSET:
            edge->setEndOffset(myIndex, parse<double>(value));
            break;
        case SUMO_ATTR_ACCELERATION:
            edge->setAcceleration(myIndex, parse<bool>(value));
            break;
        case SUMO_ATTR_CUSTOMSHAPE: {
            bool ok;
            edge->setLaneShape(myIndex, GeomConvHelper::parseShapeReporting(value, "user-supplied position", 0, ok, true));
            break;
        }
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNELane::setFunctionalColor(int activeScheme) const {
    switch (activeScheme) {
        case 6: {
            double hue = GeomHelper::naviDegree(getShape().beginEndAngle()); // [0-360]
            GLHelper::setColor(RGBColor::fromHSV(hue, 1., 1.));
            return true;
        }
        default:
            return false;
    }
}


bool
GNELane::setMultiColor(const GUIColorer& c) const {
    const int activeScheme = c.getActive();
    myShapeColors.clear();
    switch (activeScheme) {
        case 9: // color by height at segment start
            for (PositionVector::const_iterator ii = getShape().begin(); ii != getShape().end() - 1; ++ii) {
                myShapeColors.push_back(c.getScheme().getColor(ii->z()));
            }
            return true;
        case 11: // color by inclination  at segment start
            for (int ii = 1; ii < (int)getShape().size(); ++ii) {
                const double inc = (getShape()[ii].z() - getShape()[ii - 1].z()) / MAX2(POSITION_EPS, getShape()[ii].distanceTo2D(getShape()[ii - 1]));
                myShapeColors.push_back(c.getScheme().getColor(inc));
            }
            return true;
        default:
            return false;
    }
}


double
GNELane::getColorValue(int activeScheme) const {
    const SVCPermissions myPermissions = myParentEdge.getNBEdge()->getPermissions(myIndex);
    switch (activeScheme) {
        case 0:
            switch (myPermissions) {
                case SVC_PEDESTRIAN:
                    return 1;
                case SVC_BICYCLE:
                    return 2;
                case 0:
                    return 3;
                case SVC_SHIP:
                    return 4;
                default:
                    break;
            }
            if ((myPermissions & SVC_PASSENGER) != 0 || isRailway(myPermissions)) {
                return 0;
            } else {
                return 5;
            }
        case 1:
            return gSelected.isSelected(getType(), getGlID()) ||
                   gSelected.isSelected(GLO_EDGE, dynamic_cast<GNEEdge*>(&myParentEdge)->getGlID());
        case 2:
            return (double)myPermissions;
        case 3:
            return myParentEdge.getNBEdge()->getLaneSpeed(myIndex);
        case 4:
            return myParentEdge.getNBEdge()->getNumLanes();
        case 5: {
            return myParentEdge.getNBEdge()->getLoadedLength() / myParentEdge.getNBEdge()->getLength();
        }
        // case 6: by angle (functional)
        case 7: {
            return myParentEdge.getNBEdge()->getPriority();
        }
        case 8: {
            // color by z of first shape point
            return getShape()[0].z();
        }
        // case 9: by segment height
        case 10: {
            // color by incline
            return (getShape()[-1].z() - getShape()[0].z()) /  myParentEdge.getNBEdge()->getLength();
        }
    }
    return 0;
}


bool
GNELane::drawAsRailway(const GUIVisualizationSettings& s) const {
    return isRailway(myParentEdge.getNBEdge()->getPermissions(myIndex)) && s.showRails;
}


bool
GNELane::drawAsWaterway(const GUIVisualizationSettings& s) const {
    return isWaterway(myParentEdge.getNBEdge()->getPermissions(myIndex)) && s.showRails; // reusing the showRails setting
}


void
GNELane::drawCrossties(double length, double spacing, double halfWidth) const {
    glPushMatrix();
    // draw on top of of the white area between the rails
    glTranslated(0, 0, 0.1);
    int e = (int) getShape().size() - 1;
    for (int i = 0; i < e; ++i) {
        glPushMatrix();
        glTranslated(getShape()[i].x(), getShape()[i].y(), 0.0);
        glRotated(myShapeRotations[i], 0, 0, 1);
        for (double t = 0; t < myShapeLengths[i]; t += spacing) {
            glBegin(GL_QUADS);
            glVertex2d(-halfWidth, -t);
            glVertex2d(-halfWidth, -t - length);
            glVertex2d(halfWidth, -t - length);
            glVertex2d(halfWidth, -t);
            glEnd();
        }
        glPopMatrix();
    }
    glPopMatrix();
}


void
GNELane::drawDirectionIndicators() const {
    const double width = myParentEdge.getNBEdge()->getLaneWidth(myIndex);
    glColor3d(0.3, 0.3, 0.3);
    glPushMatrix();
    glTranslated(0, 0, GLO_JUNCTION + 0.1);
    int e = (int) getShape().size() - 1;
    for (int i = 0; i < e; ++i) {
        glPushMatrix();
        glTranslated(getShape()[i].x(), getShape()[i].y(), 0.1);
        glRotated(myShapeRotations[i], 0, 0, 1);
        for (double t = 0; t < myShapeLengths[i]; t += width) {
            const double length = MIN2(width * 0.5, myShapeLengths[i] - t);
            glBegin(GL_TRIANGLES);
            glVertex2d(0, -t - length);
            glVertex2d(-width * 0.25, -t);
            glVertex2d(+width * 0.25, -t);
            glEnd();
        }
        glPopMatrix();
    }
    glPopMatrix();
}



const std::string&
GNELane::getParentName() const {
    return myParentEdge.getMicrosimID();
}


long
GNELane::onDefault(FXObject* obj, FXSelector sel, void* data) {
    if (myTLSEditor != 0) {
        myTLSEditor->handleMultiChange(this, obj, sel, data);
    }
    return 1;
}


GNEEdge&
GNELane::getParentEdge() {
    return myParentEdge;
}


std::vector<GNEConnection*>
GNELane::getGNEIncomingConnections() {
    // Declare a vector to save incoming connections
    std::vector<GNEConnection*> incomingConnections;
    // Obtain incoming edges if junction source was already created
    GNEJunction* junctionSource =  myParentEdge.getGNEJunctionSource();
    if (junctionSource) {
        // Iterate over incoming GNEEdges of junction
        for (std::vector<GNEEdge*>::const_iterator i = junctionSource->getGNEIncomingEdges().begin(); i != junctionSource->getGNEIncomingEdges().end(); i++) {
            // Iterate over connection of incoming edges
            for (std::vector<GNEConnection*>::const_iterator j = (*i)->getGNEConnections().begin(); j != (*i)->getGNEConnections().end(); j++) {
                if ((*j)->getNBEdgeConnection().fromLane == getIndex()) {
                    incomingConnections.push_back(*j);
                }
            }
        }
    }
    return incomingConnections;
}


std::vector<GNEConnection*>
GNELane::getGNEOutcomingConnections() {
    // Obtain GNEConnection of edge parent
    const std::vector<GNEConnection*>& edgeConnections = myParentEdge.getGNEConnections();
    std::vector<GNEConnection*> outcomingConnections;
    // Obtain outgoing connections
    for (std::vector<GNEConnection*>::const_iterator i = edgeConnections.begin(); i != edgeConnections.end(); i++) {
        if ((*i)->getNBEdgeConnection().fromLane == getIndex()) {
            outcomingConnections.push_back(*i);
        }
    }
    return outcomingConnections;
}


void
GNELane::updateConnectionIDs() {
    // update incoming connections of lane
    std::vector<GNEConnection*> incomingConnections = getGNEIncomingConnections();
    for (std::vector<GNEConnection*>::iterator i = incomingConnections.begin(); i != incomingConnections.end(); i++) {
        (*i)->updateID();
    }
    // update outocming connections of lane
    std::vector<GNEConnection*> outcomingConnections = getGNEOutcomingConnections();
    for (std::vector<GNEConnection*>::iterator i = outcomingConnections.begin(); i != outcomingConnections.end(); i++) {
        (*i)->updateID();
    }
}

/****************************************************************************/
