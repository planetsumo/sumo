/****************************************************************************/
/// @file    MSContainer.cpp
/// @author  Melanie Weber
/// @author  Andreas Kendziorra
/// @date    Thu, 12 Jun 2014
/// @version $$
///
// The class for modelling container-movements
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
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

#include <string>
//#include <vector>
#include <utils/iodevices/OutputDevice.h>
//#include <utils/options/OptionsCont.h>
//#include <utils/common/ToString.h>
#include "MSNet.h"
#include "MSEdge.h"
#include "MSLane.h"
#include "MSContainer.h"
//#include "MSContainerControl.h"
//#include "MSInsertionControl.h"
#include "MSVehicle.h"
//#include "MSPModel.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

/* -------------------------------------------------------------------------
 * static member definitions
 * ----------------------------------------------------------------------- */

// ===========================================================================
// method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * MSContainer::MSContainerStage - methods
 * ----------------------------------------------------------------------- */
MSContainer::MSContainerStage::MSContainerStage(const MSEdge& destination, StageType type)
    : myDestination(destination), myDeparted(-1), myArrived(-1), myType(type) {}

MSContainer::MSContainerStage::~MSContainerStage() {}

const MSEdge&
MSContainer::MSContainerStage::getDestination() const {
    return myDestination;
}

void
MSContainer::MSContainerStage::setDeparted(SUMOTime now) {
    if (myDeparted < 0) {
        myDeparted = now;
    }
}

void
MSContainer::MSContainerStage::setArrived(SUMOTime now) {
    myArrived = now;
}

bool
MSContainer::MSContainerStage::isWaitingFor(const std::string& /*line*/) const {
    return false;
}

Position
MSContainer::MSContainerStage::getEdgePosition(const MSEdge* e, SUMOReal at, SUMOReal offset) const {
    return getLanePosition(e->getLanes()[0], at, offset);
}

Position
MSContainer::MSContainerStage::getLanePosition(const MSLane* lane, SUMOReal at, SUMOReal offset) const {
    return lane->getShape().positionAtOffset(lane->interpolateLanePosToGeometryPos(at), offset);
}




/* -------------------------------------------------------------------------
 * MSContainer::MSContainerStage_Driving - methods
 * ----------------------------------------------------------------------- */
MSContainer::MSContainerStage_Driving::MSContainerStage_Driving(const MSEdge& destination,
        MSTerminalStop* toTS, const std::vector<std::string>& lines)
    : MSContainerStage(destination, DRIVING), myLines(lines.begin(), lines.end()),
      myVehicle(0), myDestinationTerminalStop(toTS) {}


MSContainer::MSContainerStage_Driving::~MSContainerStage_Driving() {}

void
MSContainer::MSContainerStage_Driving::proceed(MSNet* net, MSContainer* container, SUMOTime now,
        MSEdge* previousEdge, const SUMOReal at) {
    myWaitingEdge = previousEdge;
    myWaitingPos = at;
    myWaitingSince = now;
    myVehicle = net->getVehicleControl().getWaitingVehicle(previousEdge, myLines);
	if (myVehicle != 0 && myVehicle->getParameter().departProcedure == DEPART_TRIGGERED) {
        previousEdge->removeContainer(container);
        myVehicle->addContainer(container);
        net->getInsertionControl().add(myVehicle);
        net->getVehicleControl().removeWaiting(previousEdge, myVehicle);
        net->getVehicleControl().unregisterOneWaitingForContainer();
    } else {
        net->getContainerControl().addWaiting(previousEdge, container);
        previousEdge->addContainer(container);
    }
}

const MSEdge*
MSContainer::MSContainerStage_Driving::getEdge() const {
    if (myVehicle != 0) {
        return myVehicle->getEdge();
    }
    return myWaitingEdge;
}


const MSEdge*
MSContainer::MSContainerStage_Driving::getFromEdge() const {
    return myWaitingEdge;
}


SUMOReal
MSContainer::MSContainerStage_Driving::getEdgePos(SUMOTime /* now */) const {
    if (myVehicle != 0) {
        // vehicle may already have passed the lane (check whether this is correct)
        return MIN2(myVehicle->getPositionOnLane(), getEdge()->getLength());
    }
    return myWaitingPos;
}

Position
MSContainer::MSContainerStage_Driving::getPosition(SUMOTime /* now */) const {
    if (myVehicle != 0) {
        /// @bug this fails while vehicle is driving across a junction
        return myVehicle->getEdge()->getLanes()[0]->getShape().positionAtOffset(myVehicle->getPositionOnLane());
    }
	//TODO: make class MSCModel
    return getEdgePosition(myWaitingEdge, myWaitingPos, MSCModel::CONTAINER_DEFAULT_OFFSET);//SIDEWALK_OFFSET
}

std::string
MSContainer::MSContainerStage_Driving::getStageTypeName() const {
    return isWaiting4Vehicle() ? "waiting for " + joinToString(myLines, ",") : "driving";
}

bool
MSContainer::MSContainerStage_Driving::isWaitingFor(const std::string& line) const {
    return myLines.count(line) > 0;
}

bool
MSContainer::MSContainerStage_Driving::isWaiting4Vehicle() const {
    return myVehicle == 0;
}

SUMOReal
MSContainer::MSContainerStage_Driving::getSpeed() const {
    return myVehicle == 0 ? 0 : myVehicle->getSpeed();
}

void
MSContainer::MSContainerStage_Driving::tripInfoOutput(OutputDevice& os) const {
    os.openTag("ride").writeAttr("depart", time2string(myDeparted)).writeAttr("arrival", time2string(myArrived)).closeTag();
}

void
MSContainer::MSContainerStage_Driving::routeOutput(OutputDevice& os) const {
    os.openTag("ride").writeAttr(SUMO_ATTR_FROM, getFromEdge()->getID()).writeAttr(SUMO_ATTR_TO, getDestination().getID());
    os.writeAttr(SUMO_ATTR_LINES, myLines).closeTag();
}

void
MSContainer::MSContainerStage_Driving::beginEventOutput(const MSContainer& container, SUMOTime t, OutputDevice& os) const {
    os.openTag("event").writeAttr("time", time2string(t)).writeAttr("type", "arrival").writeAttr("agent", container.getID()).writeAttr("link", getEdge()->getID()).closeTag();
}

void
MSContainer::MSContainerStage_Driving::endEventOutput(const MSContainer& container, SUMOTime t, OutputDevice& os) const {
    os.openTag("event").writeAttr("time", time2string(t)).writeAttr("type", "arrival").writeAttr("agent", container.getID()).writeAttr("link", getEdge()->getID()).closeTag();
}



/* -------------------------------------------------------------------------
 * MSContainer::MSContainerStage_Waiting - methods
 * ----------------------------------------------------------------------- */
MSContainer::MSContainerStage_Waiting::MSContainerStage_Waiting(const MSEdge& destination,
        SUMOTime duration, SUMOTime until, SUMOReal pos, const std::string& actType) :
    MSContainerStage(destination, WAITING),
    myWaitingDuration(duration),
    myWaitingUntil(until),
    myActType(actType),
    myStartPos(pos) {
    myStartPos = SUMOVehicleParameter::interpretEdgePos(
                     myStartPos, myDestination.getLength(), SUMO_ATTR_DEPARTPOS, "person stopping at " + myDestination.getID());
}