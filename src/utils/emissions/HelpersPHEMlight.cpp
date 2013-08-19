/****************************************************************************/
/// @file    HelpersPHEMlight.cpp
/// @author  Daniel Krajzewicz
/// @date    Sat, 20.04.2013
/// @version $Id: HelpersPHEMlight.cpp 13107 2012-12-02 13:57:34Z behrisch $
///
// Helper methods for PHEMlight-based emission computation
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
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

#include "HelpersPHEMlight.h"
#include "PHEMCEPHandler.h"
#include "PHEMConstants.h"
#include <limits>
#include <cmath>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
SUMOReal
	HelpersPHEMlight::getMaxAccel(SUMOEmissionClass c, double v, double a, double slope) {
    return -1;
}


SUMOReal
HelpersPHEMlight::computeCO(SUMOEmissionClass c, double v, double a, double slope) {
	PHEMCEP* currCep = PHEMCEPHandler::getHandlerInstance().GetCep(c);
	double power = CalcPower(v, a, slope, *currCep, currCep->GetVehicleLoading());
	return currCep->GetEmission("CO", power) / SECONDS_PER_HOUR;
}


SUMOReal
HelpersPHEMlight::computeCO2(SUMOEmissionClass c, double v, double a, double slope) {
	PHEMCEP* currCep = PHEMCEPHandler::getHandlerInstance().GetCep(c);
	double power = CalcPower(v, a, slope, *currCep, currCep->GetVehicleLoading());
	return currCep->GetEmission("FC", power) * 3.15 / SECONDS_PER_HOUR;
}


SUMOReal
HelpersPHEMlight::computeHC(SUMOEmissionClass c, double v, double a, double slope) {
	PHEMCEP* currCep = PHEMCEPHandler::getHandlerInstance().GetCep(c);
	double power = CalcPower(v, a, slope, *currCep, currCep->GetVehicleLoading());
	return currCep->GetEmission("HC", power) / SECONDS_PER_HOUR;
}


SUMOReal
HelpersPHEMlight::computeNOx(SUMOEmissionClass c, double v, double a, double slope) {
	PHEMCEP* currCep = PHEMCEPHandler::getHandlerInstance().GetCep(c);
	double power = CalcPower(v, a, slope, *currCep, currCep->GetVehicleLoading());
	return currCep->GetEmission("NOx", power) / SECONDS_PER_HOUR;
}


SUMOReal
HelpersPHEMlight::computePMx(SUMOEmissionClass c, double v, double a, double slope) {
	PHEMCEP* currCep = PHEMCEPHandler::getHandlerInstance().GetCep(c);
	double power = CalcPower(v, a, slope, *currCep, currCep->GetVehicleLoading());
	return currCep->GetEmission("PM", power) / SECONDS_PER_HOUR;
}


SUMOReal
HelpersPHEMlight::computeFuel(SUMOEmissionClass c, double v, double a, double slope) {
	PHEMCEP* currCep = PHEMCEPHandler::getHandlerInstance().GetCep(c);
	double power = CalcPower(v, a, slope, *currCep, currCep->GetVehicleLoading());
	return currCep->GetEmission("FC", power) / SECONDS_PER_HOUR;
}



double
HelpersPHEMlight::CalcPower(double v, double a, double slope, const PHEMCEP &vehicleCep, double loading) {
	double power = 0;
	power += (vehicleCep.GetMassVehicle() + loading) * GRAVITY_CONST * (vehicleCep.GetResistanceF0() + vehicleCep.GetResistanceF1() * v + vehicleCep.GetResistanceF4() * pow(v,4)) * v;
	power += (vehicleCep.GetCrossSectionalArea() * vehicleCep.GetCdValue() * AIR_DENSITY_CONST /2 ) * pow(v,3);
	power += (vehicleCep.GetMassVehicle() + vehicleCep.GetMassRot() + loading) * a * v;
	power += (vehicleCep.GetMassVehicle() + loading) * slope * 0.01 * v;
	return power/950;
}

/****************************************************************************/

