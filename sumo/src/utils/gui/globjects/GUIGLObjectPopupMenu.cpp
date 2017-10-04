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
/// @file    GUIGLObjectPopupMenu.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// The popup menu of a globject
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <cassert>
#include <utils/geom/GeoConvHelper.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GUIUserIO.h>
#include <utils/common/ToString.h>
#include "GUIGLObjectPopupMenu.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================
FXDEFMAP(GUIGLObjectPopupMenu) GUIGLObjectPopupMenuMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_CENTER,                  GUIGLObjectPopupMenu::onCmdCenter),
    FXMAPFUNC(SEL_COMMAND,  MID_COPY_NAME,               GUIGLObjectPopupMenu::onCmdCopyName),
    FXMAPFUNC(SEL_COMMAND,  MID_COPY_TYPED_NAME,         GUIGLObjectPopupMenu::onCmdCopyTypedName),
    FXMAPFUNC(SEL_COMMAND,  MID_COPY_EDGE_NAME,          GUIGLObjectPopupMenu::onCmdCopyEdgeName),
    FXMAPFUNC(SEL_COMMAND,  MID_COPY_CURSOR_POSITION,    GUIGLObjectPopupMenu::onCmdCopyCursorPosition),
    FXMAPFUNC(SEL_COMMAND,  MID_COPY_CURSOR_GEOPOSITION, GUIGLObjectPopupMenu::onCmdCopyCursorGeoPosition),
    FXMAPFUNC(SEL_COMMAND,  MID_SHOWPARS,                GUIGLObjectPopupMenu::onCmdShowPars),
    FXMAPFUNC(SEL_COMMAND,  MID_SHOWTYPEPARS,            GUIGLObjectPopupMenu::onCmdShowTypePars),
    FXMAPFUNC(SEL_COMMAND,  MID_ADDSELECT,               GUIGLObjectPopupMenu::onCmdAddSelected),
    FXMAPFUNC(SEL_COMMAND,  MID_REMOVESELECT,            GUIGLObjectPopupMenu::onCmdRemoveSelected)
};

// Object implementation
FXIMPLEMENT(GUIGLObjectPopupMenu, FXMenuPane, GUIGLObjectPopupMenuMap, ARRAYNUMBER(GUIGLObjectPopupMenuMap))


// ===========================================================================
// method definitions
// ===========================================================================

GUIGLObjectPopupMenu::GUIGLObjectPopupMenu(GUIMainWindow& app, GUISUMOAbstractView& parent, GUIGlObject& o) :
    FXMenuPane(&parent),
    myParent(&parent),
    myObject(&o),
    myApplication(&app),
    myNetworkPosition(parent.getPositionInformation()) {
}


GUIGLObjectPopupMenu::~GUIGLObjectPopupMenu() {
    // Delete MenuPaneChilds
    for (auto i : myMenuPanes) {
        delete i;
    }
}


void
GUIGLObjectPopupMenu::insertMenuPaneChild(FXMenuPane* child) {
    // Check that MenuPaneChild isn't NULL
    if (child == NULL) {
        throw ProcessError("MenuPaneChild cannot be NULL");
    }
    // Check that MenuPaneChild wasn't already inserted
    for (auto i : myMenuPanes) {
        if (i == child) {
            throw ProcessError("MenuPaneChild already inserted");
        }
    }
    // Insert MenuPaneChild
    myMenuPanes.push_back(child);
}


long
GUIGLObjectPopupMenu::onCmdCenter(FXObject*, FXSelector, void*) {
    // we already know where the object is since we clicked on it -> zoom on Boundary
    myParent->centerTo(myObject->getGlID(), true, -1);
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdCopyName(FXObject*, FXSelector, void*) {
    GUIUserIO::copyToClipboard(*myParent->getApp(), myObject->getMicrosimID());
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdCopyTypedName(FXObject*, FXSelector, void*) {
    GUIUserIO::copyToClipboard(*myParent->getApp(), myObject->getFullName());
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdCopyEdgeName(FXObject*, FXSelector, void*) {
    assert(myObject->getType() == GLO_LANE);
    GUIUserIO::copyToClipboard(*myParent->getApp(), myObject->getParentName());
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdCopyCursorPosition(FXObject*, FXSelector, void*) {
    GUIUserIO::copyToClipboard(*myParent->getApp(), toString(myNetworkPosition));
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdCopyCursorGeoPosition(FXObject*, FXSelector, void*) {
    Position pos = myNetworkPosition;
    GeoConvHelper::getFinal().cartesian2geo(pos);
    // formated for pasting into google maps
    const std::string posString = toString(pos.y(), gPrecisionGeo) + ", " + toString(pos.x(), gPrecisionGeo);
    GUIUserIO::copyToClipboard(*myParent->getApp(), posString);
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdShowPars(FXObject*, FXSelector, void*) {
    myObject->getParameterWindow(*myApplication, *myParent);
    return 1;
}



long
GUIGLObjectPopupMenu::onCmdShowTypePars(FXObject*, FXSelector, void*) {
    myObject->getTypeParameterWindow(*myApplication, *myParent);
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdAddSelected(FXObject*, FXSelector, void*) {
    gSelected.select(myObject->getGlID());
    myParent->update();
    return 1;
}


long
GUIGLObjectPopupMenu::onCmdRemoveSelected(FXObject*, FXSelector, void*) {
    gSelected.deselect(myObject->getGlID());
    myParent->update();
    return 1;
}

/****************************************************************************/

