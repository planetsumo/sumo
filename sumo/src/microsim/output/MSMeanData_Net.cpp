/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2004-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSMeanData_Net.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Mon, 10.05.2004
/// @version $Id$
///
// Network state mean data collector for edges/lanes
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSMeanData_Net.h"

#include <microsim/MSGlobals.h>
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>

// ===========================================================================
// debug constants
// ===========================================================================
//#define DEBUG_OCCUPANCY
//#define DEBUG_OCCUPANCY2
//#define DEBUG_NOTIFY_ENTER
//#define DEBUG_COND (veh.getLane()->getID() == "31to211_0")
#define DEBUG_COND (false)


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// MSMeanData_Net::MSLaneMeanDataValues - methods
// ---------------------------------------------------------------------------
MSMeanData_Net::MSLaneMeanDataValues::MSLaneMeanDataValues(MSLane* const lane,
        const double length,
        const bool doAdd,
        const MSMeanData_Net* parent)
    : MSMeanData::MeanDataValues(lane, length, doAdd, parent),
      nVehDeparted(0), nVehArrived(0), nVehEntered(0), nVehLeft(0),
      nVehVaporized(0), waitSeconds(0),
      nVehLaneChangeFrom(0), nVehLaneChangeTo(0),
      frontSampleSeconds(0), frontTravelledDistance(0),
      vehLengthSum(0), occupationSum(0),
      minimalVehicleLength(INVALID_DOUBLE),
      myParent(parent) {}


MSMeanData_Net::MSLaneMeanDataValues::~MSLaneMeanDataValues() {
}


void
MSMeanData_Net::MSLaneMeanDataValues::reset(bool) {
    nVehDeparted = 0;
    nVehArrived = 0;
    nVehEntered = 0;
    nVehLeft = 0;
    nVehVaporized = 0;
    nVehLaneChangeFrom = 0;
    nVehLaneChangeTo = 0;
    sampleSeconds = 0.;
    travelledDistance = 0;
    waitSeconds = 0;
    frontSampleSeconds = 0;
    frontTravelledDistance = 0;
    vehLengthSum = 0;
    occupationSum = 0;
    minimalVehicleLength = INVALID_DOUBLE;
}


void
MSMeanData_Net::MSLaneMeanDataValues::addTo(MSMeanData::MeanDataValues& val) const {
    MSLaneMeanDataValues& v = (MSLaneMeanDataValues&) val;
    v.nVehDeparted += nVehDeparted;
    v.nVehArrived += nVehArrived;
    v.nVehEntered += nVehEntered;
    v.nVehLeft += nVehLeft;
    v.nVehVaporized += nVehVaporized;
    v.nVehLaneChangeFrom += nVehLaneChangeFrom;
    v.nVehLaneChangeTo += nVehLaneChangeTo;
    v.sampleSeconds += sampleSeconds;
    v.travelledDistance += travelledDistance;
    v.waitSeconds += waitSeconds;
    v.frontSampleSeconds += frontSampleSeconds;
    v.frontTravelledDistance += frontTravelledDistance;
    v.vehLengthSum += vehLengthSum;
    v.occupationSum += occupationSum;
    if (v.minimalVehicleLength == INVALID_DOUBLE) {
        v.minimalVehicleLength = minimalVehicleLength;
    } else {
        v.minimalVehicleLength = MIN2(minimalVehicleLength, v.minimalVehicleLength);
    }
}


void
MSMeanData_Net::MSLaneMeanDataValues::notifyMoveInternal(
    const SUMOVehicle& veh, const double frontOnLane,
    const double timeOnLane, const double /* meanSpeedFrontOnLane */,
    const double meanSpeedVehicleOnLane,
    const double travelledDistanceFrontOnLane,
    const double travelledDistanceVehicleOnLane,
    const double meanLengthOnLane) {
#ifdef DEBUG_OCCUPANCY
    if DEBUG_COND {
    std::cout << SIMTIME << "\n  MSMeanData_Net::MSLaneMeanDataValues::notifyMoveInternal()\n"
    << "  veh '" << veh.getID() << "' on lane '" << veh.getLane()->getID() << "'"
        << ", timeOnLane=" << timeOnLane
        << ", meanSpeedVehicleOnLane=" << meanSpeedVehicleOnLane
        << ",\ntravelledDistanceFrontOnLane=" << travelledDistanceFrontOnLane
        << ", travelledDistanceVehicleOnLane=" << travelledDistanceVehicleOnLane
        << ", meanLengthOnLane=" << meanLengthOnLane
        << std::endl;
    }
#endif
    sampleSeconds += timeOnLane;
    travelledDistance += travelledDistanceVehicleOnLane;
    vehLengthSum += veh.getVehicleType().getLength() * timeOnLane;
    if (MSGlobals::gUseMesoSim) {
        // For the mesosim case no information on whether the vehicle was occupying
        // the lane with its whole length is available. We assume the whole length
        // Therefore this increment is taken out with more information on the vehicle movement.
        occupationSum += veh.getVehicleType().getLength() * timeOnLane;
    } else {
        // for the microsim case more elaborate calculation of the average length on the lane,
        // is taken out in notifyMove(), refs #153
        occupationSum += meanLengthOnLane * TS;
    }
    if (myParent != 0 && meanSpeedVehicleOnLane < myParent->myHaltSpeed) {
        waitSeconds += timeOnLane;
    }
    frontSampleSeconds += frontOnLane;
    frontTravelledDistance += travelledDistanceFrontOnLane;
    if (minimalVehicleLength == INVALID_DOUBLE) {
        minimalVehicleLength = veh.getVehicleType().getLength();
    } else {
        minimalVehicleLength = MIN2(minimalVehicleLength, veh.getVehicleType().getLength());
    }
#ifdef DEBUG_OCCUPANCY2
    // refs #3265
    std::cout << SIMTIME << "ID: " << getDescription() << " minVehicleLength=" << minimalVehicleLength << std::endl;
#endif
}


bool
MSMeanData_Net::MSLaneMeanDataValues::notifyLeave(SUMOVehicle& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if ((myParent == 0 || myParent->vehicleApplies(veh)) && (getLane() == 0 || getLane() == static_cast<MSVehicle&>(veh).getLane())) {
        if (MSGlobals::gUseMesoSim) {
            removeFromVehicleUpdateValues(veh);
        }
        if (reason == MSMoveReminder::NOTIFICATION_ARRIVED) {
            ++nVehArrived;
        } else if (reason == MSMoveReminder::NOTIFICATION_LANE_CHANGE) {
            ++nVehLaneChangeFrom;
        } else if (myParent == 0 || reason != MSMoveReminder::NOTIFICATION_SEGMENT) {
            ++nVehLeft;
            if (reason == MSMoveReminder::NOTIFICATION_VAPORIZED) {
                ++nVehVaporized;
            }
        }
    }
    if (MSGlobals::gUseMesoSim) {
        return false;
    }
    return reason == MSMoveReminder::NOTIFICATION_JUNCTION;
}


bool
MSMeanData_Net::MSLaneMeanDataValues::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane) {
#ifdef DEBUG_NOTIFY_ENTER
    std::cout << "\n" << SIMTIME << " MSMeanData_Net::MSLaneMeanDataValues: veh '" << veh.getID() << "' enters lane '" << enteredLane->getID() << "'" << std::endl;
#else
    UNUSED_PARAMETER(enteredLane);
#endif
    if (myParent == 0 || myParent->vehicleApplies(veh)) {
        if (getLane() == 0 || getLane() == static_cast<MSVehicle&>(veh).getLane()) {
            if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
                ++nVehDeparted;
            } else if (reason == MSMoveReminder::NOTIFICATION_LANE_CHANGE) {
                ++nVehLaneChangeTo;
            } else if (myParent == 0 || reason != MSMoveReminder::NOTIFICATION_SEGMENT) {
                ++nVehEntered;
            }
        }
        return true;
    }
    return false;
}


bool
MSMeanData_Net::MSLaneMeanDataValues::isEmpty() const {
    return sampleSeconds == 0 && nVehDeparted == 0 && nVehArrived == 0 && nVehEntered == 0
           && nVehLeft == 0 && nVehVaporized == 0 && nVehLaneChangeFrom == 0 && nVehLaneChangeTo == 0;
}


void
MSMeanData_Net::MSLaneMeanDataValues::write(OutputDevice& dev, const SUMOTime period,
        const double numLanes, const double defaultTravelTime, const int numVehicles) const {

#ifdef DEBUG_OCCUPANCY2
    // tests #3264
    double occupancy = occupationSum / STEPS2TIME(period) / myLaneLength / numLanes * (double) 100;
    if (occupancy > 100) {
        std::cout << SIMTIME << " Encountered bad occupancy: " << occupancy
                  << ", myLaneLength=" << myLaneLength << ", period=" << STEPS2TIME(period) << ", occupationSum=" << occupationSum
                  << std::endl;
    }
    // refs #3265
    std::cout << SIMTIME << "ID: " << getDescription() << " minVehicleLength=" << minimalVehicleLength
              << "\ndensity=" << MIN2(sampleSeconds / STEPS2TIME(period) * (double) 1000 / myLaneLength, 1. / MAX2(minimalVehicleLength, NUMERICAL_EPS)) << std::endl;
#endif

    if (myParent == 0) {
        if (sampleSeconds > 0) {
            dev.writeAttr("density", MIN2(sampleSeconds / STEPS2TIME(period) * (double) 1000 / myLaneLength, 1000. / MAX2(minimalVehicleLength, NUMERICAL_EPS)))
            .writeAttr("occupancy", occupationSum / STEPS2TIME(period) / myLaneLength / numLanes * (double) 100)
            .writeAttr("waitingTime", waitSeconds).writeAttr("speed", travelledDistance / sampleSeconds);
        }
        dev.writeAttr("departed", nVehDeparted).writeAttr("arrived", nVehArrived).writeAttr("entered", nVehEntered).writeAttr("left", nVehLeft);
        if (nVehVaporized > 0) {
            dev.writeAttr("vaporized", nVehVaporized);
        }
        dev.closeTag();
        return;
    }
    if (sampleSeconds > myParent->myMinSamples) {
        double overlapTraveltime = myParent->myMaxTravelTime;
        if (travelledDistance > 0.f) {
            // one vehicle has to drive lane length + vehicle length before it has left the lane
            // thus we need to scale with an extended length, approximated by lane length + average vehicle length
            overlapTraveltime = MIN2(overlapTraveltime, (myLaneLength + vehLengthSum / sampleSeconds) * sampleSeconds / travelledDistance);
        }
        if (numVehicles > 0) {
            dev.writeAttr("traveltime", sampleSeconds / numVehicles).writeAttr("waitingTime", waitSeconds).writeAttr("speed", travelledDistance / sampleSeconds);
        } else {
            double traveltime = myParent->myMaxTravelTime;
            if (frontTravelledDistance > NUMERICAL_EPS) {
                traveltime = MIN2(traveltime, myLaneLength * frontSampleSeconds / frontTravelledDistance);
                dev.writeAttr("traveltime", traveltime);
            } else if (defaultTravelTime >= 0.) {
                dev.writeAttr("traveltime", defaultTravelTime);
            }
            dev.writeAttr("overlapTraveltime", overlapTraveltime)
            .writeAttr("density", MIN2(sampleSeconds / STEPS2TIME(period) * (double) 1000 / myLaneLength, 1000. / MAX2(minimalVehicleLength, NUMERICAL_EPS)))
            .writeAttr("occupancy", occupationSum / STEPS2TIME(period) / myLaneLength / numLanes * (double) 100)
            .writeAttr("waitingTime", waitSeconds).writeAttr("speed", travelledDistance / sampleSeconds);
        }
    } else if (defaultTravelTime >= 0.) {
        dev.writeAttr("traveltime", defaultTravelTime).writeAttr("speed", myLaneLength / defaultTravelTime);
    }
    dev.writeAttr("departed", nVehDeparted).writeAttr("arrived", nVehArrived).writeAttr("entered", nVehEntered).writeAttr("left", nVehLeft)
    .writeAttr("laneChangedFrom", nVehLaneChangeFrom).writeAttr("laneChangedTo", nVehLaneChangeTo);
    if (nVehVaporized > 0) {
        dev.writeAttr("vaporized", nVehVaporized);
    }
    dev.closeTag();
}

// ---------------------------------------------------------------------------
// MSMeanData_Net - methods
// ---------------------------------------------------------------------------
MSMeanData_Net::MSMeanData_Net(const std::string& id,
                               const SUMOTime dumpBegin,
                               const SUMOTime dumpEnd, const bool useLanes,
                               const bool withEmpty, const bool printDefaults,
                               const bool withInternal,
                               const bool trackVehicles,
                               const double maxTravelTime,
                               const double minSamples,
                               const double haltSpeed,
                               const std::string& vTypes)
    : MSMeanData(id, dumpBegin, dumpEnd, useLanes, withEmpty, printDefaults,
                 withInternal, trackVehicles, maxTravelTime, minSamples, vTypes),
      myHaltSpeed(haltSpeed) {
}


MSMeanData_Net::~MSMeanData_Net() {}


MSMeanData::MeanDataValues*
MSMeanData_Net::createValues(MSLane* const lane, const double length, const bool doAdd) const {
    return new MSLaneMeanDataValues(lane, length, doAdd, this);
}


/****************************************************************************/

