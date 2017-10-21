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
/// @file    GNECalibratorDialog.h
/// @author  Pablo Alvarez Lopez
/// @date    March 2017
/// @version $Id$
///
// Dialog for edit calibrators
/****************************************************************************/
#ifndef GNECalibratorDialog_h
#define GNECalibratorDialog_h

// ===========================================================================
// included modules
// ===========================================================================

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GNEAdditionalDialog.h"


// ===========================================================================
// class declarations
// ===========================================================================

class GNECalibrator;
class GNECalibratorRoute;
class GNECalibratorFlow;
class GNECalibratorVehicleType;

// ===========================================================================
// class definitions
// ===========================================================================

/**
 * @class GNECalibratorDialog
 * @brief Dialog for edit calibrators
 */
class GNECalibratorDialog : public GNEAdditionalDialog {
    /// @brief FOX-declaration
    FXDECLARE(GNECalibratorDialog)

public:
    /// @brief Constructor
    GNECalibratorDialog(GNECalibrator* calibratorParent);

    /// @brief destructor
    ~GNECalibratorDialog();

    /// @name FOX-callbacks
    /// @{
    /// @brief event after press accept button
    long onCmdAccept(FXObject*, FXSelector, void*);

    /// @brief event after press cancel button
    long onCmdCancel(FXObject*, FXSelector, void*);

    /// @brief event after press reset button
    long onCmdReset(FXObject*, FXSelector, void*);

    /// @brief add new route
    long onCmdAddRoute(FXObject*, FXSelector, void*);

    /// @brief remove or edit route
    long onCmdClickedRoute(FXObject*, FXSelector, void*);

    /// @brief add new flow
    long onCmdAddFlow(FXObject*, FXSelector, void*);

    /// @brief remove or edit flow
    long onCmdClickedFlow(FXObject*, FXSelector, void*);

    /// @brief add new vehicle type
    long onCmdAddVehicleType(FXObject*, FXSelector, void*);

    /// @brief remove or edit vehicle type
    long onCmdClickedVehicleType(FXObject*, FXSelector, void*);
    /// @}

protected:
    /// @brief FOX needs this
    GNECalibratorDialog() {}

    /// @brief pointer to calibrator parent
    GNECalibrator* myCalibratorParent;

    /// @brief vector with the modified calibrator routes 
    std::vector<GNECalibratorRoute> myModifiedCalibratorRoutes;

    /// @brief vector with the modified calibrator flows 
    std::vector<GNECalibratorFlow> myModifiedCalibratorFlows;

    /// @brief vector with the modified calibrator vehicle types 
    std::vector<GNECalibratorVehicleType> myModifiedCalibratorVehicleTypes;

    /// @brief button for add new route
    FXButton* myAddRoute;

    /// @brief list with routes
    FXTable* myRouteList;

    /// @brief label for flows
    FXLabel* myLabelFlow;

    /// @brief button for add new flow
    FXButton* myAddFlow;

    /// @brief list with flows
    FXTable* myFlowList;

    /// @brief button for add new vehicle type
    FXButton* myAddVehicleType;

    /// @brief list with vehicle types
    FXTable* myVehicleTypeList;

    /// @brief update data table with routes
    void updateRouteTable();

    /// @brief update data table with flows
    void updateFlowTable();

    /// @brief update data table with vehicle types
    void updateVehicleTypeTable();

    /// @brief update flow and label button
    void updateFlowAndLabelButton();

private:
    /// @brief Invalidated copy constructor.
    GNECalibratorDialog(const GNECalibratorDialog&) = delete;

    /// @brief Invalidated assignment operator.
    GNECalibratorDialog& operator=(const GNECalibratorDialog&) = delete;
};

#endif
