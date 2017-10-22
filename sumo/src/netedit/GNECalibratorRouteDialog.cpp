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
/// @file    GNECalibratorRouteDialog.cpp
/// @author  Pablo Alvarez Lopez
/// @date    March 2017
/// @version $Id$
///
// Dialog for edit calibrator routes
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
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/common/MsgHandler.h>

#include "GNECalibratorRouteDialog.h"
#include "GNECalibratorDialog.h"
#include "GNECalibrator.h"
#include "GNEEdge.h"
#include "GNELane.h"
#include "GNEViewNet.h"
#include "GNENet.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNECalibratorRouteDialog) GNECalibratorRouteDialogMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONACCEPT,   GNECalibratorRouteDialog::onCmdAccept),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONCANCEL,   GNECalibratorRouteDialog::onCmdCancel),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONRESET,    GNECalibratorRouteDialog::onCmdReset),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CALIBRATORDIALOG_SET_VARIABLE,  GNECalibratorRouteDialog::onCmdSetVariable),
};

// Object implementation
FXIMPLEMENT(GNECalibratorRouteDialog, FXDialogBox, GNECalibratorRouteDialogMap, ARRAYNUMBER(GNECalibratorRouteDialogMap))

// ===========================================================================
// member method definitions
// ===========================================================================

GNECalibratorRouteDialog::GNECalibratorRouteDialog(GNECalibratorDialog* calibratorDialog, GNECalibratorRoute& calibratorRoute, bool updatingElement) :
    GNEAdditionalDialog(calibratorRoute.getCalibratorParent(), 400, 300),
    myCalibratorDialogParent(calibratorDialog),
    myCalibratorRoute(&calibratorRoute),
    myUpdatingElement(updatingElement),
    myCalibratorRouteValid(true) {
    // change default header
    changeAdditionalDialogHeader("Edit " + toString(calibratorRoute.getTag()) + " of " + toString(calibratorRoute.getCalibratorParent()->getTag()) +
                                 " '" + calibratorRoute.getCalibratorParent()->getID() + "'");

    // Create auxiliar frames for data
    FXHorizontalFrame* columns = new FXHorizontalFrame(myContentFrame, GUIDesignUniformHorizontalFrame);
    FXVerticalFrame* columnLeft = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);
    FXVerticalFrame* columnRight = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);

    // create ID's elements
    new FXLabel(columnLeft, toString(SUMO_ATTR_ID).c_str(), 0, GUIDesignLabelLeftThick);
    myTextFieldRouteID = new FXTextField(columnRight, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);

    // create list of edge's elements
    new FXLabel(columnLeft, toString(SUMO_ATTR_EDGES).c_str(), 0, GUIDesignLabelLeftThick);
    myTextFieldEdges = new FXTextField(columnRight, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);

    // create color's elements
    new FXLabel(columnLeft, toString(SUMO_ATTR_COLOR).c_str(), 0, GUIDesignLabelLeftThick);
    myTextFieldColor = new FXTextField(columnRight, GUIDesignTextFieldNCol, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignTextField);

    // create Edges of net's elements
    new FXLabel(columnLeft, (toString(SUMO_TAG_EDGE) + "s of " + toString(SUMO_TAG_NET)).c_str(), 0, GUIDesignLabelLeftThick);
    myListOfEdgesOfNet = new FXList(columnLeft, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignListExtended);

    // create Edges of route's elements
    new FXLabel(columnRight, ("current " + toString(SUMO_TAG_EDGE) + "s of " + toString(SUMO_TAG_ROUTE)).c_str(), 0, GUIDesignLabelLeftThick);
    myListOfEdgesOfRoute = new FXList(columnRight, this, MID_GNE_CALIBRATORDIALOG_SET_VARIABLE, GUIDesignListExtended);

    // fill list of net's edges
    std::vector<GNEEdge*> edgesOfNet = calibratorRoute.getCalibratorParent()->getViewNet()->getNet()->retrieveEdges();
    for (std::vector<GNEEdge*>::iterator i = edgesOfNet.begin(); i != edgesOfNet.end(); i++) {
        myListOfEdgesOfNet->appendItem((*i)->getID().c_str());
    }

    // create copy of GNECalibratorRoute
    myCopyOfCalibratorRoute = new GNECalibratorRoute(calibratorDialog);

    // copy all values of myCalibratorRoute into myCopyOfCalibratorRoute to set initial values
    (*myCopyOfCalibratorRoute) = (*myCalibratorRoute);

    // update tables
    updateCalibratorRouteValues();
}


GNECalibratorRouteDialog::~GNECalibratorRouteDialog() {
    // delete copy
    delete myCopyOfCalibratorRoute;
}


long
GNECalibratorRouteDialog::onCmdAccept(FXObject*, FXSelector, void*) {
    if (myCalibratorRouteValid == false) {
        // write warning if netedit is running in testing mode
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Opening FXMessageBox of type 'warning'");
        }
        FXMessageBox::warning(getApp(), MBOX_OK,
                              ("Error " + std::string(myUpdatingElement ? ("updating") : ("creating")) + " " + toString(myCalibratorRoute->getCalibratorParent()->getTag()) +
                               "'s " + toString(myCalibratorRoute->getTag())).c_str(), "%s",
                              (toString(myCalibratorRoute->getCalibratorParent()->getTag()) + "'s " + toString(myCalibratorRoute->getTag()) +
                               " cannot be " + std::string(myUpdatingElement ? ("updated") : ("created")) + " because parameter " + toString(myInvalidAttr) +
                               " is invalid.").c_str());
        // write warning if netedit is running in testing mode
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Closed FXMessageBox of type 'warning' with 'OK'");
        }
        return 0;
    } else {
        // copy all values of myCopyOfCalibratorRoute into myCalibratorRoute
        (*myCalibratorRoute) = (*myCopyOfCalibratorRoute);
        getApp()->stopModal(this, TRUE);
        return 1;
    }
}


long
GNECalibratorRouteDialog::onCmdCancel(FXObject*, FXSelector, void*) {
    // Stop Modal
    getApp()->stopModal(this, FALSE);
    return 1;
}


long
GNECalibratorRouteDialog::onCmdReset(FXObject*, FXSelector, void*) {
    // copy all values of myCalibratorRoute into myCopyOfCalibratorRoute to set initial values
    (*myCopyOfCalibratorRoute) = (*myCalibratorRoute);
    // update fields
    updateCalibratorRouteValues();
    return 1;
}


long
GNECalibratorRouteDialog::onCmdSetVariable(FXObject*, FXSelector, void*) {
    // At start we assumed, that all values are valid
    myCalibratorRouteValid = true;
    myInvalidAttr = SUMO_ATTR_NOTHING;

    // set color of myTextFieldRouteID, depending if current value is valid or not
    if (myCopyOfCalibratorRoute->getRouteID() == myTextFieldRouteID->getText().text()) {
        myTextFieldRouteID->setTextColor(FXRGB(0, 0, 0));
    } else if (myCopyOfCalibratorRoute->setRouteID(myTextFieldRouteID->getText().text())) {
        myTextFieldRouteID->setTextColor(FXRGB(0, 0, 0));
    } else {
        myTextFieldRouteID->setTextColor(FXRGB(255, 0, 0));
        myCalibratorRouteValid = false;
        myInvalidAttr = SUMO_ATTR_ID;
    }

    // set color of myTextFieldRouteEdges, depending if current value is valEdges or not
    if (myCopyOfCalibratorRoute->setEdges(myTextFieldEdges->getText().text())) {
        myTextFieldEdges->setTextColor(FXRGB(0, 0, 0));
        // fill list of router's edges
        myListOfEdgesOfRoute->clearItems();
        std::vector<GNEEdge*> edgesOfRouter = myCopyOfCalibratorRoute->getEdges();
        for (std::vector<GNEEdge*>::iterator i = edgesOfRouter.begin(); i != edgesOfRouter.end(); i++) {
            myListOfEdgesOfRoute->appendItem((*i)->getID().c_str());
        }
    } else {
        myTextFieldEdges->setTextColor(FXRGB(255, 0, 0));
        myCalibratorRouteValid = false;
        myInvalidAttr = SUMO_ATTR_EDGES;
    }

    // set color of myTextFieldColor, depending if current value is valid or not
    if (myCopyOfCalibratorRoute->getColor() == RGBColor::parseColor(myTextFieldColor->getText().text())) {
        myTextFieldColor->setTextColor(FXRGB(0, 0, 0));
    } else if (myCopyOfCalibratorRoute->setColor(myTextFieldColor->getText().text())) {
        myTextFieldColor->setTextColor(FXRGB(0, 0, 0));
    } else {
        myTextFieldColor->setTextColor(FXRGB(255, 0, 0));
        myCalibratorRouteValid = false;
        myInvalidAttr = SUMO_ATTR_COLOR;
    }
    return 1;
}


void
GNECalibratorRouteDialog::updateCalibratorRouteValues() {
    myTextFieldRouteID->setText(myCopyOfCalibratorRoute->getRouteID().c_str());
    myTextFieldEdges->setText(joinToString(myCopyOfCalibratorRoute->getEdgesIDs(), " ").c_str());
    myTextFieldColor->setText(toString(myCopyOfCalibratorRoute->getColor()).c_str());
    // fill list of router's edges
    myListOfEdgesOfRoute->clearItems();
    std::vector<GNEEdge*> edgesOfRouter = myCopyOfCalibratorRoute->getEdges();
    for (std::vector<GNEEdge*>::iterator i = edgesOfRouter.begin(); i != edgesOfRouter.end(); i++) {
        myListOfEdgesOfRoute->appendItem((*i)->getID().c_str());
    }
}

/****************************************************************************/
