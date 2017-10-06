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
/// @file    GUIJunctionWrapper.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @author  Andreas Gaubatz
/// @date    Mon, 1 Jul 2003
/// @version $Id$
///
// }
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
#include <utility>
#ifdef HAVE_OSG
#include <osg/Geometry>
#endif
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSJunction.h>
#include <utils/geom/Position.h>
#include <microsim/MSNet.h>
#include <microsim/MSInternalJunction.h>
#include <gui/GUIApplicationWindow.h>
#include <gui/GUIGlobals.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include "GUIJunctionWrapper.h"
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/globjects/GLIncludes.h>

//#define GUIJunctionWrapper_DEBUG_DRAW_NODE_SHAPE_VERTICES

// ===========================================================================
// method definitions
// ===========================================================================
GUIJunctionWrapper::GUIJunctionWrapper(MSJunction& junction)
    : GUIGlObject(GLO_JUNCTION, junction.getID()),
      myJunction(junction) {
    if (myJunction.getShape().size() == 0) {
        Position pos = myJunction.getPosition();
        myBoundary = Boundary(pos.x() - 1., pos.y() - 1., pos.x() + 1., pos.y() + 1.);
    } else {
        myBoundary = myJunction.getShape().getBoxBoundary();
    }
    myMaxSize = MAX2(myBoundary.getWidth(), myBoundary.getHeight());
    myIsInternal = myJunction.getType() == NODETYPE_INTERNAL;
    myAmWaterway = myJunction.getIncoming().size() + myJunction.getOutgoing().size() > 0;
    for (ConstMSEdgeVector::const_iterator it = myJunction.getIncoming().begin(); it != myJunction.getIncoming().end(); ++it) {
        if (!(*it)->isInternal() && !isWaterway((*it)->getPermissions())) {
            myAmWaterway = false;
            break;
        }
    }
    for (ConstMSEdgeVector::const_iterator it = myJunction.getOutgoing().begin(); it != myJunction.getOutgoing().end(); ++it) {
        if (!(*it)->isInternal() && !isWaterway((*it)->getPermissions())) {
            myAmWaterway = false;
            break;
        }
    }
}


GUIJunctionWrapper::~GUIJunctionWrapper() {}


GUIGLObjectPopupMenu*
GUIJunctionWrapper::getPopUpMenu(GUIMainWindow& app,
                                 GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
    buildPopupHeader(ret, app);
    buildCenterPopupEntry(ret);
    buildNameCopyPopupEntry(ret);
    buildSelectionPopupEntry(ret);
    buildPositionCopyEntry(ret, false);
    return ret;
}


GUIParameterTableWindow*
GUIJunctionWrapper::getParameterWindow(GUIMainWindow& /*app*/,
                                       GUISUMOAbstractView&) {
    return 0;
}


Boundary
GUIJunctionWrapper::getCenteringBoundary() const {
    Boundary b = myBoundary;
    b.grow(1);
    return b;
}


void
GUIJunctionWrapper::drawGL(const GUIVisualizationSettings& s) const {
    // check whether it is not too small
    if (s.scale * myMaxSize < 1.) {
        return;
    }
    if (!myIsInternal && s.drawJunctionShape) {
        glPushMatrix();
        glPushName(getGlID());
        const double colorValue = getColorValue(s);
        GLHelper::setColor(s.junctionColorer.getScheme().getColor(colorValue));

        // recognize full transparency and simply don't draw
        GLfloat color[4];
        glGetFloatv(GL_CURRENT_COLOR, color);
        const double exaggeration = s.junctionSize.getExaggeration(s);
        if (color[3] != 0 && s.scale * exaggeration > s.junctionSize.minSize) {
            PositionVector shape = myJunction.getShape();
            shape.closePolygon();
            if (exaggeration > 1) {
                shape.scaleRelative(exaggeration);
            }
            glTranslated(0, 0, getType());
            if (s.scale * myMaxSize < 40.) {
                GLHelper::drawFilledPoly(shape, true);
            } else {
                GLHelper::drawFilledPolyTesselated(shape, true);
            }
#ifdef GUIJunctionWrapper_DEBUG_DRAW_NODE_SHAPE_VERTICES
            GLHelper::debugVertices(shape, 80 / s.scale);
#endif
        }
        glPopName();
        glPopMatrix();
    }
    if (myIsInternal) {
        drawName(myJunction.getPosition(), s.scale, s.internalJunctionName);
    } else {
        drawName(myJunction.getPosition(), s.scale, s.junctionName);
    }
}


double
GUIJunctionWrapper::getColorValue(const GUIVisualizationSettings& s) const {
    switch (s.junctionColorer.getActive()) {
        case 0:
            if (myAmWaterway) {
                return 1;
            } else {
                return 0;
            }
        case 1:
            return gSelected.isSelected(getType(), getGlID()) ? 1 : 0;
        case 2:
            switch (myJunction.getType()) {
                case NODETYPE_TRAFFIC_LIGHT:
                    return 0;
                case NODETYPE_TRAFFIC_LIGHT_NOJUNCTION:
                    return 1;
                case NODETYPE_PRIORITY:
                    return 2;
                case NODETYPE_PRIORITY_STOP:
                    return 3;
                case NODETYPE_RIGHT_BEFORE_LEFT:
                    return 4;
                case NODETYPE_ALLWAY_STOP:
                    return 5;
                case NODETYPE_DISTRICT:
                    return 6;
                case NODETYPE_NOJUNCTION:
                    return 7;
                case NODETYPE_DEAD_END:
                case NODETYPE_DEAD_END_DEPRECATED:
                    return 8;
                case NODETYPE_UNKNOWN:
                case NODETYPE_INTERNAL:
                    assert(false);
                    return 8;
                case NODETYPE_RAIL_SIGNAL:
                    return 9;
                case NODETYPE_ZIPPER:
                    return 10;
                case NODETYPE_TRAFFIC_LIGHT_RIGHT_ON_RED:
                    return 11;
                case NODETYPE_RAIL_CROSSING:
                    return 12;
            }
        default:
            assert(false);
            return 0;
    }
}


#ifdef HAVE_OSG
void
GUIJunctionWrapper::updateColor(const GUIVisualizationSettings& s) {
    const double colorValue = getColorValue(s);
    const RGBColor& col = s.junctionColorer.getScheme().getColor(colorValue);
    osg::Vec4ubArray* colors = dynamic_cast<osg::Vec4ubArray*>(myGeom->getColorArray());
    (*colors)[0].set(col.red(), col.green(), col.blue(), col.alpha());
    myGeom->setColorArray(colors);
}
#endif


/****************************************************************************/

