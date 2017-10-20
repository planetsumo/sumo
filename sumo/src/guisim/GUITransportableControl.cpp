/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2012-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    GUITransportableControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Wed, 13.06.2012
/// @version $Id$
///
// GUI-version of the person control for building gui persons
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <algorithm>
#include "GUINet.h"
#include "GUIContainer.h"
#include "GUIPerson.h"
#include "GUITransportableControl.h"


// ===========================================================================
// method definitions
// ===========================================================================
GUITransportableControl::GUITransportableControl() {}


GUITransportableControl::~GUITransportableControl() {
}


MSTransportable*
GUITransportableControl::buildPerson(const SUMOVehicleParameter* pars, MSVehicleType* vtype, MSTransportable::MSTransportablePlan* plan, const bool fromRouteFile) const {
    return new GUIPerson(pars, vtype, plan, fromRouteFile);
}


MSTransportable*
GUITransportableControl::buildContainer(const SUMOVehicleParameter* pars, MSVehicleType* vtype, MSTransportable::MSTransportablePlan* plan) const {
    return new GUIContainer(pars, vtype, plan);
}


void
GUITransportableControl::insertPersonIDs(std::vector<GUIGlID>& into) {
    into.reserve(myTransportables.size());
    for (std::map<std::string, MSTransportable*>::const_iterator it = myTransportables.begin(); it != myTransportables.end(); ++it) {
        if (it->second->getCurrentStageType() != MSTransportable::WAITING_FOR_DEPART) {
            into.push_back(static_cast<const GUIPerson*>(it->second)->getGlID());
        }
    }
}


/****************************************************************************/
