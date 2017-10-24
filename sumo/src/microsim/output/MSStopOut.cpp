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
/// @file    MSStopOut.cpp
/// @author  Jakob Erdmann
/// @date    Wed, 21.12.2016
/// @version $Id$
///
// Ouput information about planned vehicle stop
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/vehicle/SUMOVehicle.h>
#include <utils/options/OptionsCont.h>
#include <microsim/MSNet.h>
#include <microsim/MSParkingArea.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/trigger/MSChargingStation.h>
#include "MSStopOut.h"


// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
MSStopOut* MSStopOut::myInstance = 0;

void
MSStopOut::init() {
    if (OptionsCont::getOptions().isSet("stop-output")) {
        myInstance = new MSStopOut(OutputDevice::getDeviceByOption("stop-output"));
    }
}


// ===========================================================================
// method definitions
// ===========================================================================
MSStopOut::MSStopOut(OutputDevice& dev) :
    myDevice(dev) {
}

MSStopOut::~MSStopOut() {}


void
MSStopOut::stopStarted(const SUMOVehicle* veh, int numPersons, int numContainers) {
    assert(veh != 0);
    if (myStopped.count(veh) != 0) {
        WRITE_WARNING("Vehicle '" + veh->getID() + "' is already stopped.");
    }
    StopInfo stopInfo(MSNet::getInstance()->getCurrentTimeStep(), numPersons, numContainers);
    myStopped[veh] = stopInfo;
}

void
MSStopOut::loadedPersons(const SUMOVehicle* veh, int n) {
    myStopped[veh].loadedPersons += n;
}

void
MSStopOut::unloadedPersons(const SUMOVehicle* veh, int n) {
    myStopped[veh].unloadedPersons += n;
}

void
MSStopOut::loadedContainers(const SUMOVehicle* veh, int n) {
    myStopped[veh].loadedContainers += n;
}

void
MSStopOut::unloadedContainers(const SUMOVehicle* veh, int n) {
    myStopped[veh].unloadedContainers += n;
}

void
MSStopOut::stopEnded(const SUMOVehicle* veh, const MSVehicle::Stop& stop) {
    assert(veh != 0);
    if (myStopped.count(veh) == 0) {
        WRITE_WARNING("Vehicle '" + veh->getID() + "' is not stopped.");
        return;
    }
    StopInfo& si = myStopped[veh];
    myDevice.openTag("stopinfo");
    myDevice.writeAttr(SUMO_ATTR_ID, veh->getID());
    myDevice.writeAttr(SUMO_ATTR_TYPE, veh->getVehicleType().getID());
    myDevice.writeAttr(SUMO_ATTR_LANE, stop.lane->getID());
    myDevice.writeAttr(SUMO_ATTR_POSITION, veh->getPositionOnLane());
    myDevice.writeAttr(SUMO_ATTR_PARKING, stop.pars.parking);
    myDevice.writeAttr("started", time2string(si.started));
    myDevice.writeAttr("ended", time2string(MSNet::getInstance()->getCurrentTimeStep()));
    myDevice.writeAttr("initialPersons", si.initialNumPersons);
    myDevice.writeAttr("loadedPersons", si.loadedPersons);
    myDevice.writeAttr("unloadedPersons", si.unloadedPersons);
    myDevice.writeAttr("initialContainers", si.initialNumContainers);
    myDevice.writeAttr("loadedContainers", si.loadedContainers);
    myDevice.writeAttr("unloadedContainers", si.unloadedContainers);
    if (stop.busstop != 0) {
        myDevice.writeAttr(SUMO_ATTR_BUS_STOP, stop.busstop->getID());
    }
    if (stop.containerstop != 0) {
        myDevice.writeAttr(SUMO_ATTR_CONTAINER_STOP, stop.containerstop->getID());
    }
    if (stop.parkingarea != 0) {
        myDevice.writeAttr(SUMO_ATTR_PARKING_AREA, stop.parkingarea->getID());
    }
    if (stop.chargingStation != 0) {
        myDevice.writeAttr(SUMO_ATTR_CHARGING_STATION, stop.chargingStation->getID());
    }
    myDevice.closeTag();
    myStopped.erase(veh);
}

/****************************************************************************/
