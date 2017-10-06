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
/// @file    GUIContainer.cpp
/// @author  Melanie Weber
/// @author  Andreas Kendziorra
/// @date    Wed, 01.08.2014
/// @version $Id$
///
// A MSContainer extended by some values for usage within the gui
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cmath>
#include <vector>
#include <string>
#include <microsim/MSContainer.h>
#include <microsim/MSCModel_NonInteracting.h>
#include <microsim/logging/CastingFunctionBinding.h>
#include <microsim/logging/FunctionBinding.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/devices/MSDevice_Vehroutes.h>
#include <utils/common/StringUtils.h>
#include <utils/vehicle/SUMOVehicleParameter.h>
#include <utils/common/AbstractMutex.h>
#include <utils/geom/GeomHelper.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/div/GLObjectValuePassConnector.h>
#include <utils/geom/PositionVector.h>
#include <gui/GUIApplicationWindow.h>
#include <gui/GUIGlobals.h>
#include "GUIContainer.h"
#include "GUINet.h"
#include "GUIEdge.h"
#include <utils/gui/globjects/GLIncludes.h>

//#define GUIContainer_DEBUG_DRAW_WALKING_AREA_SHAPE

// ===========================================================================
// FOX callback mapping
// ===========================================================================
/*
FXDEFMAP(GUIContainer::GUIContainerPopupMenu) GUIContainerPopupMenuMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_SHOW_ALLROUTES, GUIContainer::GUIContainerPopupMenu::onCmdShowAllRoutes),
    FXMAPFUNC(SEL_COMMAND, MID_HIDE_ALLROUTES, GUIContainer::GUIContainerPopupMenu::onCmdHideAllRoutes),
    FXMAPFUNC(SEL_COMMAND, MID_SHOW_CURRENTROUTE, GUIContainer::GUIContainerPopupMenu::onCmdShowCurrentRoute),
    FXMAPFUNC(SEL_COMMAND, MID_HIDE_CURRENTROUTE, GUIContainer::GUIContainerPopupMenu::onCmdHideCurrentRoute),
    FXMAPFUNC(SEL_COMMAND, MID_SHOW_BEST_LANES, GUIContainer::GUIContainerPopupMenu::onCmdShowBestLanes),
    FXMAPFUNC(SEL_COMMAND, MID_HIDE_BEST_LANES, GUIContainer::GUIContainerPopupMenu::onCmdHideBestLanes),
    FXMAPFUNC(SEL_COMMAND, MID_START_TRACK, GUIContainer::GUIContainerPopupMenu::onCmdStartTrack),
    FXMAPFUNC(SEL_COMMAND, MID_STOP_TRACK, GUIContainer::GUIContainerPopupMenu::onCmdStopTrack),
    FXMAPFUNC(SEL_COMMAND, MID_SHOW_LFLINKITEMS, GUIContainer::GUIContainerPopupMenu::onCmdShowLFLinkItems),
    FXMAPFUNC(SEL_COMMAND, MID_HIDE_LFLINKITEMS, GUIContainer::GUIContainerPopupMenu::onCmdHideLFLinkItems),
};

// Object implementation
FXIMPLEMENT(GUIContainer::GUIContainerPopupMenu, GUIGLObjectPopupMenu, GUIContainerPopupMenuMap, ARRAYNUMBER(GUIContainerPopupMenuMap))
*/

#define WATER_WAY_OFFSET 6.0

// ===========================================================================
// method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * GUIContainer::GUIContainerPopupMenu - methods
 * ----------------------------------------------------------------------- */
GUIContainer::GUIContainerPopupMenu::GUIContainerPopupMenu(
    GUIMainWindow& app, GUISUMOAbstractView& parent,
    GUIGlObject& o, std::map<GUISUMOAbstractView*, int>& additionalVisualizations)
    : GUIGLObjectPopupMenu(app, parent, o), myVehiclesAdditionalVisualizations(additionalVisualizations) {
}


GUIContainer::GUIContainerPopupMenu::~GUIContainerPopupMenu() {}



/* -------------------------------------------------------------------------
 * GUIContainer - methods
 * ----------------------------------------------------------------------- */
GUIContainer::GUIContainer(const SUMOVehicleParameter* pars, MSVehicleType* vtype, MSTransportable::MSTransportablePlan* plan) :
    MSContainer(pars, vtype, plan),
    GUIGlObject(GLO_CONTAINER, pars->id) {
}


GUIContainer::~GUIContainer() {
}


GUIGLObjectPopupMenu*
GUIContainer::getPopUpMenu(GUIMainWindow& app,
                           GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIContainerPopupMenu(app, parent, *this, myAdditionalVisualizations);
    buildPopupHeader(ret, app);
    buildCenterPopupEntry(ret);
    buildNameCopyPopupEntry(ret);
    buildSelectionPopupEntry(ret);
    //
    buildShowParamsPopupEntry(ret);
    buildPositionCopyEntry(ret, false);
    return ret;
}


GUIParameterTableWindow*
GUIContainer::getParameterWindow(GUIMainWindow& app,
                                 GUISUMOAbstractView&) {
    GUIParameterTableWindow* ret =
        new GUIParameterTableWindow(app, *this, 8);
    // add items
    //ret->mkItem("type [NAME]", false, myType->getID());
    ret->mkItem("stage", false, getCurrentStageDescription());
    ret->mkItem("start edge [id]", false, getFromEdge()->getID());
    ret->mkItem("dest edge [id]", false, getDestination().getID());
    ret->mkItem("edge [id]", false, getEdge()->getID());
    ret->mkItem("position [m]", true, new FunctionBinding<GUIContainer, double>(this, &GUIContainer::getEdgePos));
    ret->mkItem("speed [m/s]", true, new FunctionBinding<GUIContainer, double>(this, &GUIContainer::getSpeed));
    ret->mkItem("angle [degree]", true, new FunctionBinding<GUIContainer, double>(this, &GUIContainer::getAngle));
    ret->mkItem("waiting time [s]", true, new FunctionBinding<GUIContainer, double>(this, &GUIContainer::getWaitingSeconds));
    // close building
    ret->closeBuilding();
    return ret;
}


Boundary
GUIContainer::getCenteringBoundary() const {
    Boundary b;
    // ensure that the vehicle is drawn, otherwise myPositionInVehicle will not be updated
    b.add(getPosition());
    b.grow(20);
    return b;
}


void
GUIContainer::drawGL(const GUIVisualizationSettings& s) const {
    glPushName(getGlID());
    glPushMatrix();
    Position p1 = getPosition();
    if (getCurrentStageType() == DRIVING && !isWaiting4Vehicle()) {
        p1 = myPositionInVehicle;
    }
    glTranslated(p1.x(), p1.y(), getType());
    glRotated(90, 0, 0, 1);
    // XXX use container specific gui settings
    // set container color
    setColor(s);
    // scale
    const double upscale = s.containerSize.getExaggeration(s);
    glScaled(upscale, upscale, 1);
    switch (s.containerQuality) {
        case 0:
        case 1:
        case 2:
            drawAction_drawAsPoly(s);
            break;
        case 3:
        default:
            drawAction_drawAsImage(s);
            break;
    }
    glPopMatrix();

    drawName(p1, s.scale, s.containerName);
    glPopName();
}


void
GUIContainer::drawGLAdditional(GUISUMOAbstractView* const /* parent */, const GUIVisualizationSettings& /* s */) const {
    glPushName(getGlID());
    glPushMatrix();
    /*
    glTranslated(0, 0, getType() - .1); // don't draw on top of other cars
    if (hasActiveAddVisualisation(parent, VO_SHOW_BEST_LANES)) {
        drawBestLanes();
    }
    if (hasActiveAddVisualisation(parent, VO_SHOW_ROUTE)) {
        drawRoute(s, 0, 0.25);
    }
    if (hasActiveAddVisualisation(parent, VO_SHOW_ALL_ROUTES)) {
        if (getNumberReroutes() > 0) {
            const int noReroutePlus1 = getNumberReroutes() + 1;
            for (int i = noReroutePlus1 - 1; i >= 0; i--) {
                double darken = double(0.4) / double(noReroutePlus1) * double(i);
                drawRoute(s, i, darken);
            }
        } else {
            drawRoute(s, 0, 0.25);
        }
    }
    if (hasActiveAddVisualisation(parent, VO_SHOW_LFLINKITEMS)) {
        for (DriveItemVector::const_iterator i = myLFLinkLanes.begin(); i != myLFLinkLanes.end(); ++i) {
            if((*i).myLink==0) {
                continue;
            }
            MSLink* link = (*i).myLink;
            MSLane *via = link->getViaLane();
            if (via == 0) {
                via = link->getLane();
            }
            if (via != 0) {
                Position p = via->getShape()[0];
                if((*i).mySetRequest) {
                    glColor3f(0, 1, 0);
                } else {
                    glColor3f(1, 0, 0);
                }
                glTranslated(p.x(), p.y(), -.1);
                GLHelper::drawFilledCircle(1);
                glTranslated(-p.x(), -p.y(), .1);
            }
        }
    }
    */
    glPopMatrix();
    glPopName();
}




void
GUIContainer::setColor(const GUIVisualizationSettings& s) const {
    const GUIColorer& c = s.containerColorer;
    if (!setFunctionalColor(c.getActive())) {
        GLHelper::setColor(c.getScheme().getColor(getColorValue(c.getActive())));
    }
}


bool
GUIContainer::setFunctionalColor(int activeScheme) const {
    switch (activeScheme) {
        case 0: {
            if (getParameter().wasSet(VEHPARS_COLOR_SET)) {
                GLHelper::setColor(getParameter().color);
                return true;
            }
            if (getVehicleType().wasSet(VTYPEPARS_COLOR_SET)) {
                GLHelper::setColor(getVehicleType().getColor());
                return true;
            }
            return false;
        }
        case 2: {
            if (getParameter().wasSet(VEHPARS_COLOR_SET)) {
                GLHelper::setColor(getParameter().color);
                return true;
            }
            return false;
        }
        case 3: {
            if (getVehicleType().wasSet(VTYPEPARS_COLOR_SET)) {
                GLHelper::setColor(getVehicleType().getColor());
                return true;
            }
            return false;
        }
        case 8: {
            double hue = GeomHelper::naviDegree(getAngle());
            GLHelper::setColor(RGBColor::fromHSV(hue, 1., 1.));
            return true;
        }
        default:
            return false;
    }
}


double
GUIContainer::getColorValue(int activeScheme) const {
    switch (activeScheme) {
        case 4:
            return getSpeed();
        case 5:
            if (isWaiting4Vehicle()) {
                return 3;
            } else {
                return (double)getCurrentStageType();
            }
        case 6:
            return getWaitingSeconds();
        case 7:
            return gSelected.isSelected(GLO_CONTAINER, getGlID());
    }
    return 0;
}


double
GUIContainer::getEdgePos() const {
    AbstractMutex::ScopedLocker locker(myLock);
    return MSContainer::getEdgePos();
}


Position
GUIContainer::getPosition() const {
    AbstractMutex::ScopedLocker locker(myLock);
    if (getCurrentStageType() == WAITING && getEdge()->getPermissions() == SVC_SHIP) {
        MSLane* lane = getEdge()->getLanes().front();   //the most right lane of the water way
        PositionVector laneShape = lane->getShape();
        return laneShape.positionAtOffset2D(getEdgePos(), WATER_WAY_OFFSET);
    }
    return MSContainer::getPosition();
}


double
GUIContainer::getAngle() const {
    AbstractMutex::ScopedLocker locker(myLock);
    return MSContainer::getAngle();
}


double
GUIContainer::getWaitingSeconds() const {
    AbstractMutex::ScopedLocker locker(myLock);
    return MSContainer::getWaitingSeconds();
}


double
GUIContainer::getSpeed() const {
    AbstractMutex::ScopedLocker locker(myLock);
    return MSContainer::getSpeed();
}


void
GUIContainer::drawAction_drawAsPoly(const GUIVisualizationSettings& /* s */) const {
    // draw pedestrian shape
    glRotated(RAD2DEG(getAngle() + M_PI / 2.), 0, 0, 1);
    glScaled(getVehicleType().getLength(), getVehicleType().getWidth(), 1);
    glBegin(GL_QUADS);
    glVertex2d(0, 0.5);
    glVertex2d(0, -0.5);
    glVertex2d(-1, -0.5);
    glVertex2d(-1, 0.5);
    glEnd();
    GLHelper::setColor(GLHelper::getColor().changedBrightness(-30));
    glTranslated(0, 0, .045);
    glBegin(GL_QUADS);
    glVertex2d(-0.1, 0.4);
    glVertex2d(-0.1, -0.4);
    glVertex2d(-0.9, -0.4);
    glVertex2d(-0.9, 0.4);
    glEnd();
}


void
GUIContainer::drawAction_drawAsImage(const GUIVisualizationSettings& s) const {
    const std::string& file = getVehicleType().getImgFile();
    if (file != "") {
        // @todo invent an option for controlling whether images should be rotated or not
        //if (getVehicleType().getGuiShape() == SVS_CONTAINER) {
        //    glRotated(RAD2DEG(getAngle() + M_PI / 2.), 0, 0, 1);
        //}
        int textureID = GUITexturesHelper::getTextureID(file);
        if (textureID > 0) {
            const double exaggeration = s.personSize.getExaggeration(s);
            const double halfLength = getVehicleType().getLength() / 2.0 * exaggeration;
            const double halfWidth = getVehicleType().getWidth() / 2.0 * exaggeration;
            GUITexturesHelper::drawTexturedBox(textureID, -halfWidth, -halfLength, halfWidth, halfLength);
        }
    } else {
        // fallback if no image is defined
        drawAction_drawAsPoly(s);
    }
}
/****************************************************************************/

