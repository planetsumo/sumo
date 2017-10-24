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
/// @file    GNECalibratorDialog.cpp
/// @author  Pablo Alvarez Lopez
/// @date    March 2017
/// @version $Id$
///
// Dialog for edit calibrators
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
#include <utils/options/OptionsCont.h>

#include "GNECalibratorDialog.h"
#include "GNECalibrator.h"
#include "GNEChange_CalibratorItem.h"
#include "GNEViewNet.h"
#include "GNENet.h"
#include "GNECalibratorFlowDialog.h"
#include "GNECalibratorRouteDialog.h"
#include "GNECalibratorVehicleTypeDialog.h"
#include "GNEUndoList.h"
#include "GNECalibratorFlow.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNECalibratorDialog) GNECalibratorDialogMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CALIBRATORDIALOG_ADD_ROUTE,         GNECalibratorDialog::onCmdAddRoute),
    FXMAPFUNC(SEL_CLICKED,  MID_GNE_CALIBRATORDIALOG_TABLE_ROUTE,       GNECalibratorDialog::onCmdClickedRoute),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CALIBRATORDIALOG_ADD_FLOW,          GNECalibratorDialog::onCmdAddFlow),
    FXMAPFUNC(SEL_CLICKED,  MID_GNE_CALIBRATORDIALOG_TABLE_FLOW,        GNECalibratorDialog::onCmdClickedFlow),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CALIBRATORDIALOG_ADD_VEHICLETYPE,   GNECalibratorDialog::onCmdAddVehicleType),
    FXMAPFUNC(SEL_CLICKED,  MID_GNE_CALIBRATORDIALOG_TABLE_VEHICLETYPE, GNECalibratorDialog::onCmdClickedVehicleType),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONACCEPT,      GNECalibratorDialog::onCmdAccept),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONCANCEL,      GNECalibratorDialog::onCmdCancel),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ADDITIONALDIALOG_BUTTONRESET,       GNECalibratorDialog::onCmdReset),
};

// Object implementation
FXIMPLEMENT(GNECalibratorDialog, FXDialogBox, GNECalibratorDialogMap, ARRAYNUMBER(GNECalibratorDialogMap))

// ===========================================================================
// member method definitions
// ===========================================================================

GNECalibratorDialog::GNECalibratorDialog(GNECalibrator* editedCalibrator) :
    GNEAdditionalDialog(editedCalibrator, 640, 480),
    myEditedCalibrator(editedCalibrator) {

    // Create two columns, one for Routes and VehicleTypes, and other for Flows
    FXHorizontalFrame* columns = new FXHorizontalFrame(myContentFrame, GUIDesignUniformHorizontalFrame);
    FXVerticalFrame* columnLeft = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);
    FXVerticalFrame* columnRight = new FXVerticalFrame(columns, GUIDesignAuxiliarFrame);

    // create add buton and label for routes
    FXHorizontalFrame* buttonAndLabelRoute = new FXHorizontalFrame(columnLeft, GUIDesignAuxiliarHorizontalFrame);
    myAddRoute = new FXButton(buttonAndLabelRoute, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_CALIBRATORDIALOG_ADD_ROUTE, GUIDesignButtonIcon);
    new FXLabel(buttonAndLabelRoute, ("Add new " + toString(SUMO_TAG_ROUTE) + "s").c_str(), 0, GUIDesignLabelThick);

    // Create table in left frame
    myRouteList = new FXTable(columnLeft, this, MID_GNE_CALIBRATORDIALOG_TABLE_ROUTE, GUIDesignTableAdditionals);
    myRouteList->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myRouteList->setSelTextColor(FXRGBA(0, 0, 0, 255));
    myRouteList->setEditable(false);

    // create add buton and label for vehicle types
    FXHorizontalFrame* buttonAndLabelVehicleType = new FXHorizontalFrame(columnLeft, GUIDesignAuxiliarHorizontalFrame);
    myAddVehicleType = new FXButton(buttonAndLabelVehicleType, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_CALIBRATORDIALOG_ADD_VEHICLETYPE, GUIDesignButtonIcon);
    new FXLabel(buttonAndLabelVehicleType, ("Add new " + toString(SUMO_TAG_VTYPE) + "s").c_str(), 0, GUIDesignLabelThick);

    // Create table in left frame
    myVehicleTypeList = new FXTable(columnLeft, this, MID_GNE_CALIBRATORDIALOG_TABLE_VEHICLETYPE, GUIDesignTableAdditionals);
    myVehicleTypeList->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myVehicleTypeList->setSelTextColor(FXRGBA(0, 0, 0, 255));
    myVehicleTypeList->setEditable(false);

    // create add buton and label for flows in right frame
    FXHorizontalFrame* buttonAndLabelFlow = new FXHorizontalFrame(columnRight, GUIDesignAuxiliarHorizontalFrame);
    myAddFlow = new FXButton(buttonAndLabelFlow, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_CALIBRATORDIALOG_ADD_FLOW, GUIDesignButtonIcon);
    myLabelFlow = new FXLabel(buttonAndLabelFlow, ("Add new " + toString(SUMO_TAG_FLOW) + "s").c_str(), 0, GUIDesignLabelThick);

    // Create table in right frame
    myFlowList = new FXTable(columnRight, this, MID_GNE_CALIBRATORDIALOG_TABLE_FLOW, GUIDesignTableAdditionals);
    myFlowList->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myFlowList->setSelTextColor(FXRGBA(0, 0, 0, 255));
    myFlowList->setEditable(false);

    // set original calibrator values
    myModifiedCalibratorRoutes = myEditedCalibrator->getCalibratorRoutes();

    // update tables
    updateRouteTable();
    updateVehicleTypeTable();
    updateFlowTable();

    // Execute additional dialog (To make it modal)
    execute();
}


GNECalibratorDialog::~GNECalibratorDialog() {}


GNECalibrator* 
GNECalibratorDialog::getEditedCalibrator() const {
    return myEditedCalibrator;
}


const std::vector<GNECalibratorRoute>&
GNECalibratorDialog::getModifiedCalibratorRoutes() const {
    return myModifiedCalibratorRoutes;
}


std::string
GNECalibratorDialog::generateVehicleTypeID() const {
    int counter = 0;
    while (myEditedCalibrator->getViewNet()->getNet()->vehicleTypeExists(toString(SUMO_TAG_VTYPE) + toString(counter))) {
        counter++;
    }
    return (toString(SUMO_TAG_VTYPE) + toString(counter));
}


std::string
GNECalibratorDialog::generateFlowID() const {
    int counter = 0;
    while (myEditedCalibrator->getViewNet()->getNet()->flowExists(toString(SUMO_TAG_FLOW) + toString(counter))) {
        counter++;
    }
    return (toString(SUMO_TAG_FLOW) + toString(counter));
}


std::string
GNECalibratorDialog::generateRouteID() const {
    int counter = 0;
    while (myEditedCalibrator->getViewNet()->getNet()->routeExists(toString(SUMO_TAG_ROUTE) + toString(counter))) {
        counter++;
    }
    return (toString(SUMO_TAG_ROUTE) + toString(counter));
}


long
GNECalibratorDialog::onCmdAccept(FXObject*, FXSelector, void*) {
    // set new values allowing to undo it
    myEditedCalibrator->getViewNet()->getUndoList()->add(new GNEChange_CalibratorItem(myEditedCalibrator, myModifiedCalibratorRoutes), true);
    // Stop Modal
    getApp()->stopModal(this, TRUE);
    return 1;
}


long
GNECalibratorDialog::onCmdCancel(FXObject*, FXSelector, void*) {
    // Stop Modal
    getApp()->stopModal(this, FALSE);
    return 1;
}


long
GNECalibratorDialog::onCmdReset(FXObject*, FXSelector, void*) {
    // set original calibrator values
    myModifiedCalibratorRoutes = myEditedCalibrator->getCalibratorRoutes();
    // update tables
    updateRouteTable();
    updateVehicleTypeTable();
    updateFlowTable();
    return 1;
}


long
GNECalibratorDialog::onCmdAddRoute(FXObject*, FXSelector, void*) {
    // create empty calibrator route and configure it with GNECalibratorRouteDialog
    GNECalibratorRoute newRoute(this);
    if (GNECalibratorRouteDialog(this, newRoute, false).execute() == TRUE) {
        // if new route was sucesfully configured, add it to modified calibrator routes
        myModifiedCalibratorRoutes.push_back(newRoute);
        // update routes table
        updateRouteTable();
        return 1;
    } else {
        return 0;
    }
}


long
GNECalibratorDialog::onCmdClickedRoute(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myModifiedCalibratorRoutes.size(); i++) {
        if (myRouteList->getItem(i, 2)->hasFocus()) {
            // find all flows that contains route to delete as "route" parameter
            std::vector<GNECalibratorFlow*> calibratorFlowsToErase;
            for (auto j : myEditedCalibrator->getCalibratorFlows()) {
                if (j->getAttribute(SUMO_ATTR_ROUTE) == myRouteList->getItem(i, 0)->getText().text()) {
                    calibratorFlowsToErase.push_back(j);
                }
            }
            // if there are flows that has route to remove as "route" parameter
            if (calibratorFlowsToErase.size() > 0) {
                // write warning if netedit is running in testing mode
                if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
                    WRITE_WARNING("Opening FXMessageBox of type 'question'");
                }
                // open question dialog box
                FXuint answer = FXMessageBox::question(getApp(), MBOX_YES_NO, ("Remove " + toString(SUMO_TAG_FLOW) + "s").c_str(), "%s",
                                                       ("Deletion of " + toString(SUMO_TAG_ROUTE) + " '" + myRouteList->getItem(i, 0)->getText().text() +
                                                        "' will remove " + toString(calibratorFlowsToErase.size()) + " " + toString(SUMO_TAG_FLOW) + (calibratorFlowsToErase.size() > 1 ? ("s") : ("")) +
                                                        ". Continue?").c_str());
                if (answer != 1) { //1:yes, 2:no, 4:esc
                    // write warning if netedit is running in testing mode
                    if ((answer == 2) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                        WRITE_WARNING("Closed FXMessageBox of type 'question' with 'No'");
                    } else if ((answer == 4) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                        WRITE_WARNING("Closed FXMessageBox of type 'question' with 'ESC'");
                    }
                    // abort deletion of route
                    return 0;
                } else {
                    // write warning if netedit is running in testing mode
                    if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
                        WRITE_WARNING("Closed FXMessageBox of type 'question' with 'Yes'");
                    }
                    // remove calibrator flows
                    for(auto j : calibratorFlowsToErase) {
                        myEditedCalibrator->removeCalibratorFLow(j);
                        delete j;
                    }
                    // update flows table
                    updateFlowTable();
                    return 1;
                }
            } else {
                // remove route of calibrator routes
                myModifiedCalibratorRoutes.erase(myModifiedCalibratorRoutes.begin() + i);
                // update routes table
                updateRouteTable();
                return 1;
            }
        } else if (myRouteList->getItem(i, 0)->hasFocus() || myRouteList->getItem(i, 1)->hasFocus()) {
            // modify route of calibrator routes
            if (GNECalibratorRouteDialog(this, myModifiedCalibratorRoutes.at(i), true).execute() == TRUE) {
                // update routes table
                updateRouteTable();
            }
            return 1;
        }
    }
    // nothing to do
    return 0;
}


long
GNECalibratorDialog::onCmdAddFlow(FXObject*, FXSelector, void*) {
    if((myModifiedCalibratorRoutes.size() > 0) && (myEditedCalibrator->getCalibratorVehicleTypes().size() > 0)) {
        // create empty calibrator and insert it in myModifiedCalibratorFlows
        GNECalibratorFlow *newFlow = new GNECalibratorFlow(this);
        // pop it if modification with GNECalibratorFlowDialog isn't sucesfully
        if (GNECalibratorFlowDialog(this, newFlow, false).execute() == TRUE) {

            myEditedCalibrator->addCalibratorFlow(newFlow);
            // update flows table
            updateFlowTable();
            return 1;
        } else {
            delete newFlow;
            return 0;
        }
    } else {
        throw ProcessError("Neiter myModifiedCalibratorRoutes nor myModifiedCalibratorVehicleTypes can be empty");
    }
}


long
GNECalibratorDialog::onCmdClickedFlow(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myEditedCalibrator->getCalibratorFlows().size(); i++) {
        if (myFlowList->getItem(i, 3)->hasFocus()) {
            // remove flow of calibrator flows
            GNECalibratorFlow *flowToRemove = myEditedCalibrator->getCalibratorFlows().at(i);
            myEditedCalibrator->removeCalibratorFLow(flowToRemove);
            delete flowToRemove;
            // update flows table
            updateFlowTable();
            return 1;
        } else if (myFlowList->getItem(i, 0)->hasFocus() || myFlowList->getItem(i, 1)->hasFocus() || myFlowList->getItem(i, 2)->hasFocus()) {
            // modify flow of calibrator flows
            if (GNECalibratorFlowDialog(this, myEditedCalibrator->getCalibratorFlows().at(i), true).execute() == TRUE) {
                // update flows table
                updateFlowTable();
            }
            return 1;
        }
    }
    // nothing to do
    return 0;
}


long
GNECalibratorDialog::onCmdAddVehicleType(FXObject*, FXSelector, void*) {
    // create empty calibrator flow and configure it with GNECalibratorVehicleTypeDialog
    GNECalibratorVehicleType* newVehicleType = new GNECalibratorVehicleType(this);
    if (GNECalibratorVehicleTypeDialog(this, newVehicleType, false).execute() == TRUE) {
        // if new vehicle type was sucesfully configured, add it to modified calibrator vehicle types
        myEditedCalibrator->addCalibratorVehicleType(newVehicleType);
        // update vehicle types table
        updateVehicleTypeTable();
        return 1;
    } else {
        // delete created vehicleType
        delete newVehicleType;
        return 0;
    }
}


long
GNECalibratorDialog::onCmdClickedVehicleType(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myEditedCalibrator->getCalibratorVehicleTypes().size(); i++) {
        if (myVehicleTypeList->getItem(i, 2)->hasFocus()) {
            // find all flows that contains vehicle type to delete as "vehicle type" parameter
            std::vector<GNECalibratorFlow*> calibratorFlowsToErase;
            for (auto j : myEditedCalibrator->getCalibratorFlows()) {
                if (j->getAttribute(SUMO_ATTR_TYPE) == myVehicleTypeList->getItem(i, 0)->getText().text()) {
                    calibratorFlowsToErase.push_back(j);
                }
            }
            // if there are flows that has vehicle type to remove as "vehicle type" parameter
            if (calibratorFlowsToErase.size() > 0) {
                FXuint answer = FXMessageBox::question(getApp(), MBOX_YES_NO, ("Remove " + toString(SUMO_TAG_FLOW) + "s").c_str(), "%s",
                                                       ("Deletion of " + toString(SUMO_TAG_ROUTE) + " '" + myRouteList->getItem(i, 0)->getText().text() +
                                                        "' will remove " + toString(calibratorFlowsToErase.size()) + " " + toString(SUMO_TAG_FLOW) + (calibratorFlowsToErase.size() > 1 ? ("s") : ("")) +
                                                        ". Continue?").c_str());
                if (answer != 1) { //1:yes, 2:no, 4:esc
                    // write warning if netedit is running in testing mode
                    if ((answer == 2) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                        WRITE_WARNING("Closed FXMessageBox of type 'question' with 'No'");
                    } else if ((answer == 4) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                        WRITE_WARNING("Closed FXMessageBox of type 'question' with 'ESC'");
                    }
                    // abort deletion of vehicle type
                    return 0;
                } else {
                    // write warning if netedit is running in testing mode
                    if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
                        WRITE_WARNING("Closed FXMessageBox of type 'question' with 'Yes'");
                    }
                    // remove calibrator flows
                    for(auto j : calibratorFlowsToErase) {
                        myEditedCalibrator->removeCalibratorFLow(j);
                        delete j;
                    }
                    // update flows table
                    updateFlowTable();
                    return 1;
                }
            } else {
                GNECalibratorVehicleType* vehicleTypeToErase = myEditedCalibrator->getCalibratorVehicleTypes().at(i);
                // remove vehicle
                myEditedCalibrator->removeCalibratorVehicleType(vehicleTypeToErase);
                delete vehicleTypeToErase;
                // update vehicle types table
                updateVehicleTypeTable();
                return 1;
            }
        } else if (myVehicleTypeList->getItem(i, 0)->hasFocus() || myVehicleTypeList->getItem(i, 1)->hasFocus()) {
            // modify vehicle type of calibratorVehicleTypes
            if (GNECalibratorVehicleTypeDialog(this, myEditedCalibrator->getCalibratorVehicleTypes().at(i), true).execute() == TRUE) {
                // update vehicle types table
                updateVehicleTypeTable();
            }
            return 1;
        }
    }
    // nothing to do
    return 0;
}


void
GNECalibratorDialog::updateRouteTable() {
    // clear table
    myRouteList->clearItems();
    // set number of rows
    myRouteList->setTableSize(int(myModifiedCalibratorRoutes.size()), 3);
    // Configure list
    myRouteList->setVisibleColumns(4);
    myRouteList->setColumnWidth(0, 136);
    myRouteList->setColumnWidth(1, 136);
    myRouteList->setColumnWidth(2, GUIDesignTableIconCellWidth);
    myRouteList->setColumnText(0, toString(SUMO_ATTR_ID).c_str());
    myRouteList->setColumnText(1, toString(SUMO_ATTR_EDGES).c_str());
    myRouteList->setColumnText(2, "");
    myRouteList->getRowHeader()->setWidth(0);
    // Declare index for rows and pointer to FXTableItem
    int indexRow = 0;
    FXTableItem* item = 0;
    // iterate over routes
    for (auto i : myModifiedCalibratorRoutes) {
        // Set ID
        item = new FXTableItem(toString(i.getRouteID()).c_str());
        myRouteList->setItem(indexRow, 0, item);
        // Set edges
        item = new FXTableItem(toString(i.getEdgesIDs()).c_str());
        myRouteList->setItem(indexRow, 1, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myRouteList->setItem(indexRow, 2, item);
        // Update index
        indexRow++;
    }
    // enable or disable flow and label button
    updateFlowAndLabelButton();
}


void
GNECalibratorDialog::updateFlowTable() {
    // clear table
    myFlowList->clearItems();
    // set number of rows
    myFlowList->setTableSize(int(myEditedCalibrator->getCalibratorFlows().size()), 4);
    // Configure list
    myFlowList->setVisibleColumns(4);
    myFlowList->setColumnWidth(0, 92);
    myFlowList->setColumnWidth(1, 90);
    myFlowList->setColumnWidth(2, 90);
    myFlowList->setColumnWidth(3, GUIDesignTableIconCellWidth);
    myFlowList->setColumnText(0, toString(SUMO_ATTR_ID).c_str());
    myFlowList->setColumnText(1, toString(SUMO_ATTR_VCLASS).c_str());
    myFlowList->setColumnText(2, toString(SUMO_ATTR_ROUTE).c_str());
    myFlowList->setColumnText(3, "");
    myFlowList->getRowHeader()->setWidth(0);
    // Declare index for rows and pointer to FXTableItem
    int indexRow = 0;
    FXTableItem* item = 0;
    // iterate over flows
    for (auto i : myEditedCalibrator->getCalibratorFlows()) {
        // Set id
        item = new FXTableItem(i->getID().c_str());
        myFlowList->setItem(indexRow, 0, item);
        // Set vehicle type
        item = new FXTableItem(i->getAttribute(SUMO_ATTR_TYPE).c_str());
        myFlowList->setItem(indexRow, 1, item);
        // Set route
        item = new FXTableItem(i->getAttribute(SUMO_ATTR_ROUTE).c_str());
        myFlowList->setItem(indexRow, 2, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myFlowList->setItem(indexRow, 3, item);
        // Update index
        indexRow++;
    }
    // enable or disable flow and label button
    updateFlowAndLabelButton();
}


void
GNECalibratorDialog::updateVehicleTypeTable() {
    // clear table
    myVehicleTypeList->clearItems();
    // set number of rows
    myVehicleTypeList->setTableSize(int(myEditedCalibrator->getCalibratorVehicleTypes().size()), 3);
    // Configure list
    myVehicleTypeList->setVisibleColumns(4);
    myVehicleTypeList->setColumnWidth(0, 136);
    myVehicleTypeList->setColumnWidth(1, 136);
    myVehicleTypeList->setColumnWidth(2, GUIDesignTableIconCellWidth);
    myVehicleTypeList->setColumnText(0, toString(SUMO_ATTR_ID).c_str());
    myVehicleTypeList->setColumnText(1, toString(SUMO_ATTR_VCLASS).c_str());
    myVehicleTypeList->setColumnText(2, "");
    myVehicleTypeList->getRowHeader()->setWidth(0);
    // Declare index for rows and pointer to FXTableItem
    int indexRow = 0;
    FXTableItem* item = 0;
    // iterate over vehicle types
    for (auto i : myEditedCalibrator->getCalibratorVehicleTypes()) {
        // Set id
        item = new FXTableItem(i->getID().c_str());
        myVehicleTypeList->setItem(indexRow, 0, item);
        // Set VClass
        item = new FXTableItem(i->getAttribute(SUMO_ATTR_VCLASS).c_str());
        myVehicleTypeList->setItem(indexRow, 1, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myVehicleTypeList->setItem(indexRow, 2, item);
        // Update index
        indexRow++;
    }
    // enable or disable flow and label button
    updateFlowAndLabelButton();
}

void
GNECalibratorDialog::updateFlowAndLabelButton() {
    // Enable or disable AddFlow button and flow list depending of currently there are routes and vehicle types defined
    std::string errorMsg;
    if (myModifiedCalibratorRoutes.empty() && myEditedCalibrator->getCalibratorVehicleTypes().empty()) {
        errorMsg = " and ";
    }
    if (myModifiedCalibratorRoutes.size() == 0 || myEditedCalibrator->getCalibratorVehicleTypes().size() == 0) {
        myAddFlow->disable();
        myFlowList->disable();
        std::string errorMessage = "No " + (myModifiedCalibratorRoutes.empty() ? (toString(SUMO_TAG_ROUTE) + "s") : ("")) + errorMsg +
                                   (myEditedCalibrator->getCalibratorVehicleTypes().empty() ? (toString(SUMO_TAG_VTYPE) + "s") : ("")) + " defined";
        myLabelFlow->setText(errorMessage.c_str());
    } else {
        myAddFlow->enable();
        myFlowList->enable();
        myLabelFlow->setText(("Add new " + toString(SUMO_TAG_FLOW) + "s").c_str());
    }
}

/****************************************************************************/
