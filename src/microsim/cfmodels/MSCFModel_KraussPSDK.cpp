/****************************************************************************/
/// @file    MSCFModel_KraussPSDK.cpp
/// @author  Tobias Mayer
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Mon, 04 Aug 2009
/// @version $Id: MSCFModel_KraussPSDK.cpp 16537 2014-06-05 10:19:18Z dkrajzew $
///
// Krauss car-following model, changing accel and speed by slope
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2001-2014 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
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

#include <utils/geom/GeomHelper.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include "MSCFModel_KraussPSDK.h"


// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_KraussPSDK::MSCFModel_KraussPSDK(const MSVehicleType* vtype, SUMOReal accel, SUMOReal decel,
                                       SUMOReal dawdle, SUMOReal headwayTime)
    : MSCFModel_Krauss(vtype, accel, decel, dawdle, headwayTime) {
}


MSCFModel_KraussPSDK::~MSCFModel_KraussPSDK() {}



SUMOReal
MSCFModel_KraussPSDK::maxNextSpeed(SUMOReal speed, const MSVehicle* const veh) const {
    const SUMOReal lp = veh->getPositionOnLane();
    const MSLane* const lane = veh->getLane();
    const SUMOReal gp = lane->interpolateLanePosToGeometryPos(veh->getPositionOnLane());
    const SUMOReal slope = lane->getShape().slopeDegreeAtOffset(gp);
    const SUMOReal cSlope = sin(slope);
	SUMOReal aMax = getMaxAccel();
    aMax = aMax - aMax*cSlope;
	SUMOReal vMax = myType->getMaxSpeed();
    vMax = vMax - vMax*cSlope;
    return MIN2(speed + (SUMOReal) ACCEL2SPEED(aMax), vMax);
}



MSCFModel*
MSCFModel_KraussPSDK::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_KraussPSDK(vtype, myAccel, myDecel, myDawdle, myHeadwayTime);
}


//void MSCFModel::saveState(std::ostream &os) {}

