/****************************************************************************/
/// @file    MSMeanData_PHEMlight.cpp
/// @author  Daniel Krajzewicz
/// @date    Sat, 20.04.2013
/// @version $Id: MSMeanData_PHEMlight.cpp 13171 2012-12-18 11:12:01Z behrisch $
///
// Emission data collector for edges/lanes that uses PHEMlight
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

#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSMeanData_PHEMlight.h"
#include <utils/emissions/HelpersPHEMlight.h>
#include <utils/emissions/PollutantsInterface.h>
#include <limits>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// MSMeanData_PHEMlight::MSLaneMeanDataValues - methods
// ---------------------------------------------------------------------------
MSMeanData_PHEMlight::MSLaneMeanDataValues::MSLaneMeanDataValues(MSLane* const lane,
        const SUMOReal length, const bool doAdd,
        const std::set<std::string>* const vTypes,
        const MSMeanData_PHEMlight* parent)
    : MSMeanData::MeanDataValues(lane, length, doAdd, vTypes),
      CO2(0), CO(0), HC(0), NOx(0), PMx(0), fuel(0), myParent(parent) {}


MSMeanData_PHEMlight::MSLaneMeanDataValues::~MSLaneMeanDataValues() {
}


void
MSMeanData_PHEMlight::MSLaneMeanDataValues::reset(bool) {
    sampleSeconds = 0.;
    travelledDistance = 0.;
    CO2 = 0;
    CO = 0;
    HC = 0;
    NOx = 0;
    PMx = 0;
    fuel = 0;
}


void
MSMeanData_PHEMlight::MSLaneMeanDataValues::addTo(MSMeanData::MeanDataValues& val) const {
    MSLaneMeanDataValues& v = (MSLaneMeanDataValues&) val;
    v.sampleSeconds += sampleSeconds;
    v.travelledDistance += travelledDistance;
    v.CO2 += CO2;
    v.CO += CO;
    v.HC += HC;
    v.NOx += NOx;
    v.PMx += PMx;
    v.fuel += fuel;
}


void
MSMeanData_PHEMlight::MSLaneMeanDataValues::notifyMoveInternal(SUMOVehicle& veh, SUMOReal timeOnLane, SUMOReal speed) {
    sampleSeconds += timeOnLane;
    travelledDistance += speed * timeOnLane;
    const double a = veh.getAcceleration();
	const double slope = veh.getSlope();
    CO += (timeOnLane * HelpersPHEMlight::computeCO(veh.getVehicleType().getEmissionClass(), (double) speed, a, slope));
    CO2 += (timeOnLane * HelpersPHEMlight::computeCO2(veh.getVehicleType().getEmissionClass(), (double) speed, a, slope));
    HC += (timeOnLane * HelpersPHEMlight::computeHC(veh.getVehicleType().getEmissionClass(), (double) speed, a, slope));
    NOx += (timeOnLane * HelpersPHEMlight::computeNOx(veh.getVehicleType().getEmissionClass(), (double) speed, a, slope));
    PMx += (timeOnLane * HelpersPHEMlight::computePMx(veh.getVehicleType().getEmissionClass(), (double) speed, a, slope));
    fuel += (timeOnLane * HelpersPHEMlight::computeFuel(veh.getVehicleType().getEmissionClass(), (double) speed, a, slope));
}


void
MSMeanData_PHEMlight::MSLaneMeanDataValues::write(OutputDevice& dev, const SUMOTime period,
        const SUMOReal /*numLanes*/, const SUMOReal defaultTravelTime, const int /*numVehicles*/) const {
    const SUMOReal normFactor = SUMOReal(3600. / STEPS2TIME(period) / myLaneLength);
    dev << "\" CO_abs=\"" << OutputDevice::realString(CO, 6) <<
        "\" CO2_abs=\"" << OutputDevice::realString(CO2, 6) <<
        "\" HC_abs=\"" << OutputDevice::realString(HC, 6) <<
        "\" PMx_abs=\"" << OutputDevice::realString(PMx, 6) <<
        "\" NOx_abs=\"" << OutputDevice::realString(NOx, 6) <<
        "\" fuel_abs=\"" << OutputDevice::realString(fuel, 6) <<
        "\"\n            CO_normed=\"" << OutputDevice::realString(normFactor * CO, 6) <<
        "\" CO2_normed=\"" << OutputDevice::realString(normFactor * CO2, 6) <<
        "\" HC_normed=\"" << OutputDevice::realString(normFactor * HC, 6) <<
        "\" PMx_normed=\"" << OutputDevice::realString(normFactor * PMx, 6) <<
        "\" NOx_normed=\"" << OutputDevice::realString(normFactor * NOx, 6) <<
        "\" fuel_normed=\"" << OutputDevice::realString(normFactor * fuel, 6);
    if (sampleSeconds > myParent->myMinSamples) {
        SUMOReal vehFactor = myParent->myMaxTravelTime / sampleSeconds;
        SUMOReal traveltime = myParent->myMaxTravelTime;
        if (travelledDistance > 0.f) {
            vehFactor = MIN2(vehFactor, myLaneLength / travelledDistance);
            traveltime = MIN2(traveltime, myLaneLength * sampleSeconds / travelledDistance);
        }
        dev << "\"\n            traveltime=\"" << OutputDevice::realString(traveltime) <<
            "\" CO_perVeh=\"" << OutputDevice::realString(CO * vehFactor, 6) <<
            "\" CO2_perVeh=\"" << OutputDevice::realString(CO2 * vehFactor, 6) <<
            "\" HC_perVeh=\"" << OutputDevice::realString(HC * vehFactor, 6) <<
            "\" PMx_perVeh=\"" << OutputDevice::realString(PMx * vehFactor, 6) <<
            "\" NOx_perVeh=\"" << OutputDevice::realString(NOx * vehFactor, 6) <<
            "\" fuel_perVeh=\"" << OutputDevice::realString(fuel * vehFactor, 6);
    } else if (defaultTravelTime >= 0.) {
        const MSVehicleType* t = MSNet::getInstance()->getVehicleControl().getVType();
        const SUMOReal speed = MIN2(myLaneLength / defaultTravelTime, t->getMaxSpeed());
		const SUMOReal slope = 0; // !!! discuss/check
        dev << "\"\n            traveltime=\"" << OutputDevice::realString(defaultTravelTime) <<
            "\" CO_perVeh=\"" << OutputDevice::realString(PollutantsInterface::computeDefaultCO(t->getEmissionClass(), speed, t->getCarFollowModel().getMaxAccel(), slope, defaultTravelTime), 6) <<
            "\" CO2_perVeh=\"" << OutputDevice::realString(PollutantsInterface::computeDefaultCO2(t->getEmissionClass(), speed, t->getCarFollowModel().getMaxAccel(), slope, defaultTravelTime), 6) <<
            "\" HC_perVeh=\"" << OutputDevice::realString(PollutantsInterface::computeDefaultHC(t->getEmissionClass(), speed, t->getCarFollowModel().getMaxAccel(), slope, defaultTravelTime), 6) <<
            "\" PMx_perVeh=\"" << OutputDevice::realString(PollutantsInterface::computeDefaultPMx(t->getEmissionClass(), speed, t->getCarFollowModel().getMaxAccel(), slope, defaultTravelTime), 6) <<
            "\" NOx_perVeh=\"" << OutputDevice::realString(PollutantsInterface::computeDefaultNOx(t->getEmissionClass(), speed, t->getCarFollowModel().getMaxAccel(), slope, defaultTravelTime), 6) <<
            "\" fuel_perVeh=\"" << OutputDevice::realString(PollutantsInterface::computeDefaultFuel(t->getEmissionClass(), speed, t->getCarFollowModel().getMaxAccel(), slope, defaultTravelTime), 6);
    }
    dev << "\"";
    dev.closeTag();
}



// ---------------------------------------------------------------------------
// MSMeanData_PHEMlight - methods
// ---------------------------------------------------------------------------
MSMeanData_PHEMlight::MSMeanData_PHEMlight(const std::string& id,
                                   const SUMOTime dumpBegin,
                                   const SUMOTime dumpEnd,
                                   const bool useLanes, const bool withEmpty,
                                   const bool printDefaults,
                                   const bool withInternal,
                                   const bool trackVehicles,
                                   const SUMOReal maxTravelTime,
                                   const SUMOReal minSamples,
                                   const std::set<std::string> vTypes)
    : MSMeanData(id, dumpBegin, dumpEnd, useLanes, withEmpty, printDefaults,
                 withInternal, trackVehicles, maxTravelTime, minSamples, vTypes) {
}


MSMeanData_PHEMlight::~MSMeanData_PHEMlight() {}


MSMeanData::MeanDataValues*
MSMeanData_PHEMlight::createValues(MSLane* const lane, const SUMOReal length, const bool doAdd) const {
    return new MSLaneMeanDataValues(lane, length, doAdd, &myVehicleTypes, this);
}


/****************************************************************************/
