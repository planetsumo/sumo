/****************************************************************************/
/// @file    MSDevice_PHEMlight.cpp
/// @author  Daniel Krajzewicz
/// @date    Sat, 20.04.2013
/// @version $Id: MSDevice_PHEMlight.cpp 13171 2012-12-18 11:12:01Z behrisch $
///
// A device which collects vehicular emissions (using PHEMlight)
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

#include "MSDevice_PHEMlight.h"
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicleControl.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/HelpersPHEMlight.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_PHEMlight::insertOptions() {
    OptionsCont& oc = OptionsCont::getOptions();
    oc.addOptionSubTopic("Emissions");

    oc.doRegister("device.phem.probability", new Option_Float(0.));//!!! describe
    oc.addDescription("device.phem.probability", "Emissions", "The probability for a vehicle to have an emission logging device");

    oc.doRegister("device.phem.explicit", new Option_String());//!!! describe
    oc.addSynonyme("device.phem.explicit", "device.phem.knownveh", true);
    oc.addDescription("device.phem.explicit", "Emissions", "Assign a device to named vehicles");

    oc.doRegister("device.phem.deterministic", new Option_Bool(false)); //!!! describe
    oc.addDescription("device.phem.deterministic", "Emissions", "The devices are set deterministic using a fraction of 1000");
}


void
MSDevice_PHEMlight::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (oc.getFloat("device.phem.probability") == 0 && !oc.isSet("device.phem.explicit")) {
        // no route computation is modelled
        return;
    }
    // route computation is enabled
    bool haveByNumber = false;
    if (oc.getBool("device.phem.deterministic")) {
        haveByNumber = MSNet::getInstance()->getVehicleControl().isInQuota(oc.getFloat("device.phem.probability"));
    } else {
        haveByNumber = RandHelper::rand() <= oc.getFloat("device.phem.probability");
    }
    bool haveByName = oc.isSet("device.phem.explicit") && OptionsCont::getOptions().isInStringVector("device.phem.explicit", v.getID());
    if (haveByNumber || haveByName) {
        // build the device
        MSDevice_PHEMlight* device = new MSDevice_PHEMlight(v, "phem_" + v.getID());
        into.push_back(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_PHEMlight-methods
// ---------------------------------------------------------------------------
MSDevice_PHEMlight::MSDevice_PHEMlight(SUMOVehicle& holder, const std::string& id)
    : MSDevice(holder, id),
      myCO2(0), myCO(0), myHC(0), myPMx(0), myNOx(0), myFuel(0) {
}


MSDevice_PHEMlight::~MSDevice_PHEMlight() {
}


bool
MSDevice_PHEMlight::notifyMove(SUMOVehicle& veh, SUMOReal /*oldPos*/, SUMOReal /*newPos*/, SUMOReal newSpeed) {
    const SUMOEmissionClass c = veh.getVehicleType().getEmissionClass();
    const SUMOReal a = veh.getAcceleration();
	const SUMOReal slope = veh.getSlope();
    myCO2 += TS * HelpersPHEMlight::computeCO2(c, newSpeed, a, slope);
    myCO += TS * HelpersPHEMlight::computeCO(c, newSpeed, a, slope);
    myHC += TS * HelpersPHEMlight::computeHC(c, newSpeed, a, slope);
    myPMx += TS * HelpersPHEMlight::computePMx(c, newSpeed, a, slope);
    myNOx += TS * HelpersPHEMlight::computeNOx(c, newSpeed, a, slope);
    myFuel += TS * HelpersPHEMlight::computeFuel(c, newSpeed, a, slope);
    return true;
}


void
MSDevice_PHEMlight::generateOutput() const {
    OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-output");
    (os.openTag("emissions") <<
     " CO_abs=\"" << OutputDevice::realString(myCO, 6) <<
     "\" CO2_abs=\"" << OutputDevice::realString(myCO2, 6) <<
     "\" HC_abs=\"" << OutputDevice::realString(myHC, 6) <<
     "\" PMx_abs=\"" << OutputDevice::realString(myPMx, 6) <<
     "\" NOx_abs=\"" << OutputDevice::realString(myNOx, 6) <<
     "\" fuel_abs=\"" << OutputDevice::realString(myFuel, 6) <<
     "\"").closeTag();
}



/****************************************************************************/

