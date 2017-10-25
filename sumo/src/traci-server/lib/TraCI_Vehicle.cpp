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
/// @file    TraCI_Vehicle.cpp
/// @author  Jakob Erdmann
/// @date    15.03.2017
/// @version $Id$
///
// C++ Vehicle API
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
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>
#include <utils/common/TplConvert.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/xml/SUMOVehicleParserHelper.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/devices/MSDevice.h>
#include <microsim/MSEdgeWeightsStorage.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSVehicleType.h>
#include <microsim/MSInsertionControl.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <traci-server/TraCIDefs.h>
#include <traci-server/TraCIConstants.h>
#include "TraCI.h"
#include "TraCI_Vehicle.h"


// ===========================================================================
// member definitions
// ===========================================================================

MSVehicle*
TraCI_Vehicle::getVehicle(const std::string& id) {
    SUMOVehicle* sumoVehicle = MSNet::getInstance()->getVehicleControl().getVehicle(id);
    if (sumoVehicle == 0) {
        throw TraCIException("Vehicle '" + id + "' is not known");
    }
    MSVehicle* v = dynamic_cast<MSVehicle*>(sumoVehicle);
    if (v == 0) {
        throw TraCIException("Vehicle '" + id +  "' is not a micro-simulation vehicle");
    }
    return v;
}


bool
TraCI_Vehicle::isVisible(const MSVehicle* veh) {
    return veh->isOnRoad() || veh->isParking() || veh->wasRemoteControlled();
}


bool
TraCI_Vehicle::isOnInit(const std::string& vehicleID) {
    SUMOVehicle* sumoVehicle = MSNet::getInstance()->getVehicleControl().getVehicle(vehicleID);
    return sumoVehicle == 0 || sumoVehicle->getLane() == 0;
}

std::vector<std::string>
TraCI_Vehicle::getIDList() {
    std::vector<std::string> ids;
    MSVehicleControl& c = MSNet::getInstance()->getVehicleControl();
    for (MSVehicleControl::constVehIt i = c.loadedVehBegin(); i != c.loadedVehEnd(); ++i) {
        if ((*i).second->isOnRoad() || (*i).second->isParking()) {
            ids.push_back((*i).first);
        }
    }
    return ids;
}

int
TraCI_Vehicle::getIDCount() {
    return (int)getIDList().size();
}

double
TraCI_Vehicle::getSpeed(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getSpeed() : INVALID_DOUBLE_VALUE;
}


double
TraCI_Vehicle::getSpeedWithoutTraCI(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getSpeedWithoutTraciInfluence() : INVALID_DOUBLE_VALUE;
}


TraCIPosition
TraCI_Vehicle::getPosition(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (isVisible(veh)) {
        return TraCI::makeTraCIPosition(veh->getPosition());
    } else {
        TraCIPosition result;
        result.x = INVALID_DOUBLE_VALUE;
        result.y = INVALID_DOUBLE_VALUE;
        result.z = INVALID_DOUBLE_VALUE;
        return result;
    }
}


double
TraCI_Vehicle::getAngle(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? GeomHelper::naviDegree(veh->getAngle()) : INVALID_DOUBLE_VALUE;
}


double
TraCI_Vehicle::getSlope(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return veh->isOnRoad() ? veh->getSlope() : INVALID_DOUBLE_VALUE;
}


std::string
TraCI_Vehicle::getRoadID(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getLane()->getEdge().getID() : "";
}


std::string
TraCI_Vehicle::getLaneID(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return veh->isOnRoad() ? veh->getLane()->getID() : "";
}


int
TraCI_Vehicle::getLaneIndex(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return veh->isOnRoad() ? veh->getLane()->getIndex() : INVALID_INT_VALUE;
}

std::string
TraCI_Vehicle::getTypeID(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getVehicleType().getID();
}

std::string
TraCI_Vehicle::getRouteID(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getRoute().getID();
}

int
TraCI_Vehicle::getRouteIndex(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return veh->hasDeparted() ? veh->getRoutePosition() : INVALID_INT_VALUE;
}

TraCIColor
TraCI_Vehicle::getColor(const std::string& vehicleID) {
    return TraCI::makeTraCIColor(getVehicle(vehicleID)->getParameter().color);
}

double
TraCI_Vehicle::getLanePosition(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return veh->isOnRoad() ? veh->getPositionOnLane() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getLateralLanePosition(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return veh->isOnRoad() ? veh->getLateralPositionOnLane() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getCO2Emission(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getCO2Emissions() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getCOEmission(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getCOEmissions() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getHCEmission(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getHCEmissions() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getPMxEmission(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getPMxEmissions() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getNOxEmission(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getNOxEmissions() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getFuelConsumption(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getFuelConsumption() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getNoiseEmission(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getHarmonoise_NoiseEmissions() : INVALID_DOUBLE_VALUE;
}

double
TraCI_Vehicle::getElectricityConsumption(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    return isVisible(veh) ? veh->getElectricityConsumption() : INVALID_DOUBLE_VALUE;
}

int
TraCI_Vehicle::getPersonNumber(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getPersonNumber();
}


std::pair<std::string, double>
TraCI_Vehicle::getLeader(const std::string& vehicleID, double dist) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        std::pair<const MSVehicle* const, double> leaderInfo = veh->getLeader(dist);
        return std::make_pair(
                   leaderInfo.first != 0 ? leaderInfo.first->getID() : "",
                   leaderInfo.second);
    } else {
        return std::make_pair("", -1);
    }
}


double
TraCI_Vehicle::getWaitingTime(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getWaitingSeconds();
}


double
TraCI_Vehicle::getAccumulatedWaitingTime(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getAccumulatedWaitingSeconds();
}


double
TraCI_Vehicle::getAdaptedTraveltime(const std::string& vehicleID, const std::string& edgeID, int time) {
    MSVehicle* veh = getVehicle(vehicleID);
    MSEdge* edge = TraCI::getEdge(edgeID);
    double value = INVALID_DOUBLE_VALUE;;
    veh->getWeightsStorage().retrieveExistingTravelTime(edge, time, value);
    return value;
}


double
TraCI_Vehicle::getEffort(const std::string& vehicleID, const std::string& edgeID, int time) {
    MSVehicle* veh = getVehicle(vehicleID);
    MSEdge* edge = TraCI::getEdge(edgeID);
    double value = INVALID_DOUBLE_VALUE;;
    veh->getWeightsStorage().retrieveExistingEffort(edge, time, value);
    return value;
}


bool
TraCI_Vehicle::isRouteValid(const std::string& vehicleID) {
    std::string msg;
    return getVehicle(vehicleID)->hasValidRoute(msg);
}

std::vector<std::string>
TraCI_Vehicle::getEdges(const std::string& vehicleID) {
    std::vector<std::string> result;
    MSVehicle* veh = getVehicle(vehicleID);
    const MSRoute& r = veh->getRoute();
    for (MSRouteIterator i = r.begin(); i != r.end(); ++i) {
        result.push_back((*i)->getID());
    }
    return result;
}


int
TraCI_Vehicle::getSignalStates(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getSignals();
}

std::vector<TraCIBestLanesData>
TraCI_Vehicle::getBestLanes(const std::string& vehicleID) {
    std::vector<TraCIBestLanesData> result;
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        const std::vector<MSVehicle::LaneQ>& bestLanes = veh->getBestLanes();
        for (std::vector<MSVehicle::LaneQ>::const_iterator i = bestLanes.begin(); i != bestLanes.end(); ++i) {
            TraCIBestLanesData bld;
            const MSVehicle::LaneQ& lq = *i;
            bld.laneID = lq.lane->getID();
            bld.length = lq.length;
            bld.occupation = lq.nextOccupation;
            bld.bestLaneOffset = lq.bestLaneOffset;
            bld.allowsContinuation = lq.allowsContinuation;
            for (std::vector<MSLane*>::const_iterator j = lq.bestContinuations.begin(); j != lq.bestContinuations.end(); ++j) {
                if ((*j) != 0) {
                    bld.continuationLanes.push_back((*j)->getID());
                }
            }
            result.push_back(bld);
        }
    }
    return result;
}


std::vector<TraCINextTLSData>
TraCI_Vehicle::getNextTLS(const std::string& vehicleID) {
    std::vector<TraCINextTLSData> result;
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        const MSLane* lane = veh->getLane();
        const std::vector<MSLane*>& bestLaneConts = veh->getBestLanesContinuation(lane);
        double seen = veh->getLane()->getLength() - veh->getPositionOnLane();
        int view = 1;
        MSLinkCont::const_iterator link = MSLane::succLinkSec(*veh, view, *lane, bestLaneConts);
        while (!lane->isLinkEnd(link)) {
            if (!lane->getEdge().isInternal()) {
                if ((*link)->isTLSControlled()) {
                    TraCINextTLSData ntd;
                    ntd.id = (*link)->getTLLogic()->getID();
                    ntd.tlIndex = (*link)->getTLIndex();
                    ntd.dist = seen;
                    ntd.state = (char)(*link)->getState();
                    result.push_back(ntd);
                }
            }
            lane = (*link)->getViaLaneOrLane();
            if (!lane->getEdge().isInternal()) {
                view++;
            }
            seen += lane->getLength();
            link = MSLane::succLinkSec(*veh, view, *lane, bestLaneConts);
        }
    }
    return result;
}

int
TraCI_Vehicle::getStopState(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    int result = 0;
    if (veh->isStopped()) {
        const MSVehicle::Stop& stop = veh->getNextStop();
        result = (1 + (stop.pars.parking ? 2 : 0) +
                  (stop.pars.triggered ? 4 : 0) +
                  (stop.pars.containerTriggered ? 8 : 0) +
                  (stop.busstop != 0 ? 16 : 0) +
                  (stop.containerstop != 0 ? 32 : 0) +
                  (stop.chargingStation != 0 ? 64 : 0) +
                  (stop.parkingarea != 0 ? 128 : 0));
    }
    return result;
}

double
TraCI_Vehicle::getDistance(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        double distance = veh->getRoute().getDistanceBetween(veh->getDepartPos(), veh->getPositionOnLane(), veh->getRoute().getEdges()[0],  &veh->getLane()->getEdge());
        if (distance == std::numeric_limits<double>::max()) {
            return INVALID_DOUBLE_VALUE;
        } else {
            return distance;
        }
    } else {
        return INVALID_DOUBLE_VALUE;
    }
}


double
TraCI_Vehicle::getDrivingDistance(const std::string& vehicleID, const std::string& edgeID, double position, int /* laneIndex */) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        double distance = veh->getRoute().getDistanceBetween(veh->getPositionOnLane(), position,
                          veh->getEdge(), TraCI::getEdge(edgeID));
        if (distance == std::numeric_limits<double>::max()) {
            return INVALID_DOUBLE_VALUE;
        }
        return distance;
    } else {
        return INVALID_DOUBLE_VALUE;
    }
}


double
TraCI_Vehicle::getDrivingDistance2D(const std::string& vehicleID, double x, double y) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        std::pair<MSLane*, double> roadPos = TraCI::convertCartesianToRoadMap(Position(x, y));
        double distance = veh->getRoute().getDistanceBetween(veh->getPositionOnLane(), roadPos.second,
                          veh->getEdge(), &roadPos.first->getEdge());
        if (distance == std::numeric_limits<double>::max()) {
            return INVALID_DOUBLE_VALUE;
        }
        return distance;
    } else {
        return INVALID_DOUBLE_VALUE;
    }
}



double
TraCI_Vehicle::getAllowedSpeed(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        return veh->getLane()->getVehicleMaxSpeed(veh);
    } else {
        return INVALID_DOUBLE_VALUE;
    }
}

double
TraCI_Vehicle::getSpeedFactor(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getChosenSpeedFactor();
}


int
TraCI_Vehicle::getSpeedMode(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getInfluencer().getSpeedMode();
}

int
TraCI_Vehicle::getLanechangeMode(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getInfluencer().getLanechangeMode();
}

int
TraCI_Vehicle::getRoutingMode(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getInfluencer().getRoutingMode();
}

std::string
TraCI_Vehicle::getLine(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getParameter().line;
}

std::vector<std::string>
TraCI_Vehicle::getVia(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getParameter().via;
}


std::pair<int, int>
TraCI_Vehicle::getLaneChangeState(const std::string& vehicleID, int direction) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh->isOnRoad()) {
        return veh->getLaneChangeModel().getSavedState(direction);
    } else {
        return std::make_pair((int)LCA_UNKNOWN, (int)LCA_UNKNOWN);
    }
}


std::string
TraCI_Vehicle::getParameter(const std::string& vehicleID, const std::string& key) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (StringUtils::startsWith(key, "device.")) {
        StringTokenizer tok(key, ".");
        if (tok.size() < 3) {
            throw TraCIException("Invalid device parameter '" + key + "' for vehicle '" + vehicleID + "'");
        }
        try {
            return veh->getDeviceParameter(tok.get(1), key.substr(tok.get(0).size() + tok.get(1).size() + 2));
        } catch (InvalidArgument& e) {
            throw TraCIException("Vehicle '" + vehicleID + "' does not support device parameter '" + key + "' (" + e.what() + ").");
        }
    } else if (StringUtils::startsWith(key, "laneChangeModel.")) {
        const std::string attrName = key.substr(16);
        try {
            return veh->getLaneChangeModel().getParameter(attrName);
        } catch (InvalidArgument& e) {
            throw TraCIException("Vehicle '" + vehicleID + "' does not support laneChangeModel parameter '" + key + "' (" + e.what() + ").");
        }
    } else if (StringUtils::startsWith(key, "has.") && StringUtils::endsWith(key, ".device")) {
        StringTokenizer tok(key, ".");
        if (tok.size() != 3) {
            throw TraCIException("Invalid check for device. Expected format is 'has.DEVICENAME.device'");
        }
        return veh->hasDevice(tok.get(1)) ? "true" : "false";
    } else {
        return veh->getParameter().getParameter(key, "");
    }
}


const MSVehicleType&
TraCI_Vehicle::getVehicleType(const std::string& vehicleID) {
    return getVehicle(vehicleID)->getVehicleType();
}


std::string
TraCI_Vehicle::getEmissionClass(const std::string& vehicleID) {
    return PollutantsInterface::getName(getVehicleType(vehicleID).getEmissionClass());
}

std::string
TraCI_Vehicle::getShapeClass(const std::string& vehicleID) {
    return getVehicleShapeName(getVehicleType(vehicleID).getGuiShape());
}


double
TraCI_Vehicle::getLength(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getLength();
}


double
TraCI_Vehicle::getAccel(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getLength();
}


double
TraCI_Vehicle::getDecel(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getCarFollowModel().getMaxDecel();
}


double TraCI_Vehicle::getEmergencyDecel(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getCarFollowModel().getEmergencyDecel();
}
double TraCI_Vehicle::getApparentDecel(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getCarFollowModel().getApparentDecel();
}
double TraCI_Vehicle::getActionStepLength(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getActionStepLengthSecs();
}
double TraCI_Vehicle::getLastActionTime(const std::string& vehicleID) {
    return STEPS2TIME(getVehicle(vehicleID)->getLastActionTime());
}

double
TraCI_Vehicle::getTau(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getCarFollowModel().getHeadwayTime();
}


double
TraCI_Vehicle::getImperfection(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getCarFollowModel().getImperfection();
}


double
TraCI_Vehicle::getSpeedDeviation(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getSpeedFactor().getParameter()[1];
}


std::string
TraCI_Vehicle::getVClass(const std::string& vehicleID) {
    return toString(getVehicleType(vehicleID).getVehicleClass());
}


double
TraCI_Vehicle::getMinGap(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getMinGap();
}


double
TraCI_Vehicle::getMaxSpeed(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getMaxSpeed();
}


double
TraCI_Vehicle::getWidth(const std::string& vehicleID) {
    return getVehicleType(vehicleID).getWidth();
}


void
TraCI_Vehicle::setStop(const std::string& vehicleID,
                       const std::string& edgeID,
                       double endPos,
                       int laneIndex,
                       SUMOTime duration,
                       int flags,
                       double startPos,
                       SUMOTime until) {
    MSVehicle* veh = getVehicle(vehicleID);
    // optional stop flags
    bool parking = false;
    bool triggered = false;
    bool containerTriggered = false;
    SumoXMLTag stoppingPlaceType = SUMO_TAG_NOTHING;

    parking = ((flags & 1) != 0);
    triggered = ((flags & 2) != 0);
    containerTriggered = ((flags & 4) != 0);
    if ((flags & 8) != 0) {
        stoppingPlaceType = SUMO_TAG_BUS_STOP;
    }
    if ((flags & 16) != 0) {
        stoppingPlaceType = SUMO_TAG_CONTAINER_STOP;
    }
    if ((flags & 32) != 0) {
        stoppingPlaceType = SUMO_TAG_CHARGING_STATION;
    }
    if ((flags & 64) != 0) {
        stoppingPlaceType = SUMO_TAG_PARKING_AREA;
    }

    std::string error;
    if (stoppingPlaceType != SUMO_TAG_NOTHING) {
        // Forward command to vehicle
        if (!veh->addTraciStopAtStoppingPlace(edgeID, duration, until, parking, triggered, containerTriggered, stoppingPlaceType, error)) {
            throw TraCIException(error);
        }
    } else {
        // check
        if (startPos < 0) {
            throw TraCIException("Position on lane must not be negative.");
        }
        if (endPos < startPos) {
            throw TraCIException("End position on lane must be after start position.");
        }
        // get the actual lane that is referenced by laneIndex
        MSEdge* road = MSEdge::dictionary(edgeID);
        if (road == 0) {
            throw TraCIException("Unable to retrieve road with given id.");
        }
        const std::vector<MSLane*>& allLanes = road->getLanes();
        if ((laneIndex < 0) || laneIndex >= (int)(allLanes.size())) {
            throw TraCIException("No lane with index '" + toString(laneIndex) + "' on road '" + edgeID + "'.");
        }
        // Forward command to vehicle
        if (!veh->addTraciStop(allLanes[laneIndex], startPos, endPos, duration, until, parking, triggered, containerTriggered, error)) {
            throw TraCIException(error);
        }
    }
}


void
TraCI_Vehicle::resume(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (!veh->hasStops()) {
        throw TraCIException("Failed to resume vehicle '" + veh->getID() + "', it has no stops.");
    }
    if (!veh->resumeFromStopping()) {
        MSVehicle::Stop& sto = veh->getNextStop();
        std::ostringstream strs;
        strs << "reached: " << sto.reached;
        strs << ", duration:" << sto.duration;
        strs << ", edge:" << (*sto.edge)->getID();
        strs << ", startPos: " << sto.pars.startPos;
        std::string posStr = strs.str();
        throw TraCIException("Failed to resume from stoppingfor vehicle '" + veh->getID() + "', " + posStr);
    }
}


void 
TraCI_Vehicle::changeTarget(const std::string& vehicleID, const std::string& edgeID) {
    MSVehicle* veh = getVehicle(vehicleID);
    const MSEdge* destEdge = MSEdge::dictionary(edgeID);
    const bool onInit = isOnInit(vehicleID);
    if (destEdge == 0) {
        throw TraCIException("Can not retrieve road with ID " + edgeID);
    }
    // build a new route between the vehicle's current edge and destination edge
    ConstMSEdgeVector newRoute;
    const MSEdge* currentEdge = veh->getRerouteOrigin();
    MSNet::getInstance()->getRouterTT().compute(
            currentEdge, destEdge, (const MSVehicle * const) veh, MSNet::getInstance()->getCurrentTimeStep(), newRoute);
    // replace the vehicle's route by the new one
    if (!veh->replaceRouteEdges(newRoute, onInit)) {
        throw TraCIException("Route replacement failed for " + veh->getID());
    }
    // route again to ensure usage of via/stops
    try {
        veh->reroute(MSNet::getInstance()->getCurrentTimeStep(), MSNet::getInstance()->getRouterTT(), onInit);
    } catch (ProcessError& e) {
        throw TraCIException(e.what());
    }
}


void
TraCI_Vehicle::changeLane(const std::string& vehicleID, int laneIndex, SUMOTime duration) {
    std::vector<std::pair<SUMOTime, int> > laneTimeLine;
    laneTimeLine.push_back(std::make_pair(MSNet::getInstance()->getCurrentTimeStep(), laneIndex));
    laneTimeLine.push_back(std::make_pair(MSNet::getInstance()->getCurrentTimeStep() + duration, laneIndex));
    getVehicle(vehicleID)->getInfluencer().setLaneTimeLine(laneTimeLine);
}


void 
TraCI_Vehicle::changeSublane(const std::string& vehicleID, double latDist) {
    getVehicle(vehicleID)->getInfluencer().setSublaneChange(latDist);
}


void
TraCI_Vehicle::add(const std::string& vehicleID,
                   const std::string& routeID,
                   const std::string& typeID,
                   std::string depart,
                   const std::string& departLane,
                   const std::string& departPos,
                   const std::string& departSpeed,
                   const std::string& arrivalLane,
                   const std::string& arrivalPos,
                   const std::string& arrivalSpeed,
                   const std::string& fromTaz,
                   const std::string& toTaz,
                   const std::string& line,
                   int personCapacity,
                   int personNumber) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (veh != 0) {
        throw TraCIException("The vehicle " + vehicleID + " to add already exists.");
    }
}


void
TraCI_Vehicle::moveToXY(const std::string& vehicleID, const std::string& edgeID, const int lane, const double x, const double y, const double angle, const int keepRoute) {
    getVehicle(vehicleID);
}

void
TraCI_Vehicle::slowDown(const std::string& vehicleID, double speed, SUMOTime duration) {
    MSVehicle* veh = getVehicle(vehicleID);
    std::vector<std::pair<SUMOTime, double> > speedTimeLine;
    speedTimeLine.push_back(std::make_pair(MSNet::getInstance()->getCurrentTimeStep(), veh->getSpeed()));
    speedTimeLine.push_back(std::make_pair(MSNet::getInstance()->getCurrentTimeStep() + duration, speed));
    veh->getInfluencer().setSpeedTimeLine(speedTimeLine);
}

void
TraCI_Vehicle::setSpeed(const std::string& vehicleID, double speed) {
    getVehicle(vehicleID);
}

void 
TraCI_Vehicle::setType(const std::string& vehicleID, const std::string& typeID) {
    MSVehicleType* vehicleType = MSNet::getInstance()->getVehicleControl().getVType(typeID);
    if (vehicleType == 0) {
        throw TraCIException("Vehicle type '" + vehicleID + "' is not known");
    }
    getVehicle(vehicleID)->replaceVehicleType(vehicleType);
}

void 
TraCI_Vehicle::setRouteID(const std::string& vehicleID, const std::string& routeID) {
    MSVehicle* veh = getVehicle(vehicleID);
    const MSRoute* r = MSRoute::dictionary(routeID);
    if (r == 0) {
        throw TraCIException("The route '" + routeID + "' is not known.");
    }
    std::string msg;
    if (!veh->hasValidRoute(msg, r)) {
        WRITE_WARNING("Invalid route replacement for vehicle '" + veh->getID() + "'. " + msg);
        if (MSGlobals::gCheckRoutes) {
            throw TraCIException("Route replacement failed for " + veh->getID());
        }
    }

    if (!veh->replaceRoute(r, veh->getLane() == 0)) {
        throw TraCIException("Route replacement failed for " + veh->getID());
    }
}


void 
TraCI_Vehicle::setRoute(const std::string& vehicleID, const std::vector<std::string>& edgeIDs) {
    MSVehicle* veh = getVehicle(vehicleID);
    ConstMSEdgeVector edges;
    try {
        MSEdge::parseEdgesList(edgeIDs, edges, "<unknown>");
    } catch (ProcessError& e) {
        throw TraCIException("Invalid edge list for vehicle '" + veh->getID() + "' (" + e.what() + ")");
    }
    if (!veh->replaceRouteEdges(edges, veh->getLane() == 0, true)) {
        throw TraCIException("Route replacement failed for " + veh->getID());
    }
}

void 
TraCI_Vehicle::setAdaptedTraveltime(const std::string& vehicleID, const std::string& edgeID, 
            double time, SUMOTime begTime, SUMOTime endTime) {
    MSVehicle* veh = getVehicle(vehicleID);
    MSEdge* edge = MSEdge::dictionary(edgeID);
    if (edge == 0) {
        throw TraCIException("Referended edge '" + edgeID + "' is not known.");
    }
    if (time != INVALID_DOUBLE_VALUE) {
        // add time
        if (begTime == 0 && endTime == SUMOTime_MAX) {
            // clean up old values before setting whole range
            while (veh->getWeightsStorage().knowsTravelTime(edge)) {
                veh->getWeightsStorage().removeTravelTime(edge);
            }
        }
        veh->getWeightsStorage().addTravelTime(edge, begTime, endTime, time);
    } else {
        // remove time
        while (veh->getWeightsStorage().knowsTravelTime(edge)) {
            veh->getWeightsStorage().removeTravelTime(edge);
        }
    }
}


void 
TraCI_Vehicle::setEffort(const std::string& vehicleID, const std::string& edgeID, 
            double effort, SUMOTime begTime, SUMOTime endTime) {
    MSVehicle* veh = getVehicle(vehicleID);
    MSEdge* edge = MSEdge::dictionary(edgeID);
    if (edge == 0) {
        throw TraCIException("Referended edge '" + edgeID + "' is not known.");
    }
    if (effort != INVALID_DOUBLE_VALUE) {
        // add effort
        if (begTime == 0 && endTime == SUMOTime_MAX) {
            // clean up old values before setting whole range
            while (veh->getWeightsStorage().knowsEffort(edge)) {
                veh->getWeightsStorage().removeEffort(edge);
            }
        }
        veh->getWeightsStorage().addEffort(edge, begTime, endTime, effort);
    } else {
        // remove effort
        while (veh->getWeightsStorage().knowsEffort(edge)) {
            veh->getWeightsStorage().removeEffort(edge);
        }
    }
}


void 
TraCI_Vehicle::rerouteTraveltime(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    veh->reroute(MSNet::getInstance()->getCurrentTimeStep(), MSNet::getInstance()->getRouterTT(), isOnInit(vehicleID));
}


void 
TraCI_Vehicle::rerouteEffort(const std::string& vehicleID) {
    MSVehicle* veh = getVehicle(vehicleID);
    veh->reroute(MSNet::getInstance()->getCurrentTimeStep(), MSNet::getInstance()->getRouterEffort(), isOnInit(vehicleID));
}


void 
TraCI_Vehicle::setSignals(const std::string& vehicleID, int signals) {
    MSVehicle* veh = getVehicle(vehicleID);
    // set influencer to make the change persistent
    veh->getInfluencer().setSignals(signals);
    // set them now so that getSignals returns the correct value
    veh->switchOffSignal(0x0fffffff);
    if (signals >= 0) {
        veh->switchOnSignal(signals);
    }
}


void 
TraCI_Vehicle::moveTo(const std::string& vehicleID, const std::string& laneID, double position) {
    MSVehicle* veh = getVehicle(vehicleID);
    MSLane* l = MSLane::dictionary(laneID);
    if (l == 0) {
        throw TraCIException("Unknown lane '" + laneID + "'.");
    }
    MSEdge& destinationEdge = l->getEdge();
    if (!veh->willPass(&destinationEdge)) {
        throw TraCIException("Vehicle '" + laneID + "' may be set onto an edge to pass only.");
    }
    veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_TELEPORT);
    if (veh->getLane() != 0) {
        veh->getLane()->removeVehicle(veh, MSMoveReminder::NOTIFICATION_TELEPORT);
    } else {
        veh->setTentativeLaneAndPosition(l, position);
    }
    while (veh->getEdge() != &destinationEdge) {
        const MSEdge* nextEdge = veh->succEdge(1);
        // let the vehicle move to the next edge
        if (veh->enterLaneAtMove(nextEdge->getLanes()[0], true)) {
            MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
            continue;
        }
    }
    if (!veh->isOnRoad()) {
        MSNet::getInstance()->getInsertionControl().alreadyDeparted(veh);

    }
    l->forceVehicleInsertion(veh, position,
            veh->hasDeparted() ? MSMoveReminder::NOTIFICATION_TELEPORT : MSMoveReminder::NOTIFICATION_DEPARTED);
}


void
TraCI_Vehicle::setMaxSpeed(const std::string& vehicleID, double speed) {
    getVehicle(vehicleID)->getSingularType().setMaxSpeed(speed);
}

void
TraCI_Vehicle::setActionStepLength(const std::string& vehicleID, double actionStepLength, bool resetActionOffset) {
    if (actionStepLength < 0.0) {
        WRITE_ERROR("Invalid action step length (<0). Ignoring command setActionStepLength().");
        return;
    }
    MSVehicle* veh = getVehicle(vehicleID);
    if(actionStepLength==0.) {
        veh->resetActionOffset();
        return;
    }
    SUMOTime actionStepLengthMillisecs = SUMOVehicleParserHelper::processActionStepLength(actionStepLength);
    SUMOTime previousActionStepLength = veh->getActionStepLength();
    veh->getSingularType().setActionStepLength(actionStepLengthMillisecs, resetActionOffset);
    if(resetActionOffset) {
        veh->resetActionOffset();
    } else {
        veh->updateActionOffset(previousActionStepLength, actionStepLengthMillisecs);
    }
}

void
TraCI_Vehicle::remove(const std::string& vehicleID, char reason) {
    getVehicle(vehicleID);
}


void
TraCI_Vehicle::setColor(const std::string& vehicleID, const TraCIColor& col) {
    const SUMOVehicleParameter& p = getVehicle(vehicleID)->getParameter();
    p.color.set(col.r, col.g, col.b, col.a);
    p.parametersSet |= VEHPARS_COLOR_SET;
}


void
TraCI_Vehicle::setLine(const std::string& vehicleID, const std::string& line) {
    getVehicle(vehicleID)->getParameter().line = line;
}

void
TraCI_Vehicle::setVia(const std::string& vehicleID, const std::vector<std::string>& via) {
    getVehicle(vehicleID);
}

void
TraCI_Vehicle::setShapeClass(const std::string& vehicleID, const std::string& clazz) {
    getVehicle(vehicleID)->getSingularType().setShape(getVehicleShapeID(clazz));
}

void
TraCI_Vehicle::setEmissionClass(const std::string& vehicleID, const std::string& clazz) {
    getVehicle(vehicleID);
}


void
TraCI_Vehicle::setParameter(const std::string& vehicleID, const std::string& key, const std::string& value) {
    MSVehicle* veh = getVehicle(vehicleID);
    if (StringUtils::startsWith(key, "device.")) {
        StringTokenizer tok(key, ".");
        if (tok.size() < 3) {
            throw TraCIException("Invalid device parameter '" + key + "' for vehicle '" + vehicleID + "'");
        }
        try {
            veh->setDeviceParameter(tok.get(1), key.substr(tok.get(0).size() + tok.get(1).size() + 2), value);
        } catch (InvalidArgument& e) {
            throw TraCIException("Vehicle '" + vehicleID + "' does not support device parameter '" + key + "' (" + e.what() + ").");
        }
    } else if (StringUtils::startsWith(key, "laneChangeModel.")) {
        const std::string attrName = key.substr(16);
        try {
            veh->getLaneChangeModel().setParameter(attrName, value);
        } catch (InvalidArgument& e) {
            throw TraCIException("Vehicle '" + vehicleID + "' does not support laneChangeModel parameter '" + key + "' (" + e.what() + ").");
        }
    } else if (StringUtils::startsWith(key, "has.") && StringUtils::endsWith(key, ".device")) {
        StringTokenizer tok(key, ".");
        if (tok.size() != 3) {
            throw TraCIException("Invalid request for device status change. Expected format is 'has.DEVICENAME.device'");
        }
        const std::string deviceName = tok.get(1);
        bool create;
        try {
            create = TplConvert::_2bool(value.c_str());
        } catch (BoolFormatException) {
            throw TraCIException("Changing device status requires a 'true' or 'false'");
        }
        if (!create) {
            throw TraCIException("Device removal is not supported for device of type '" + deviceName + "'");
        }
        try {
            veh->createDevice(deviceName);
        } catch (InvalidArgument& e) {
            throw TraCIException("Cannot create vehicle device (" + std::string(e.what()) + ").");
        }
    } else {
        ((SUMOVehicleParameter&) veh->getParameter()).setParameter(key, value);
    }
}


/****************************************************************************/
