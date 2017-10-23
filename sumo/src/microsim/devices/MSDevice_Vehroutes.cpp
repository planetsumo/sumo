/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2009-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSDevice_Vehroutes.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Fri, 30.01.2009
/// @version $Id$
///
// A device which collects info on the vehicle trip
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSRoute.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice_String.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include "MSDevice_Vehroutes.h"


// ===========================================================================
// static member variables
// ===========================================================================
bool MSDevice_Vehroutes::mySaveExits = false;
bool MSDevice_Vehroutes::myLastRouteOnly = false;
bool MSDevice_Vehroutes::myDUAStyle = false;
bool MSDevice_Vehroutes::mySorted = false;
bool MSDevice_Vehroutes::myIntendedDepart = false;
bool MSDevice_Vehroutes::myRouteLength = false;
bool MSDevice_Vehroutes::mySkipPTLines = false;
MSDevice_Vehroutes::StateListener MSDevice_Vehroutes::myStateListener;
std::map<const SUMOTime, int> MSDevice_Vehroutes::myDepartureCounts;
std::map<const SUMOTime, std::map<const std::string, std::string> > MSDevice_Vehroutes::myRouteInfos;


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Vehroutes::init() {
    if (OptionsCont::getOptions().isSet("vehroute-output")) {
        OutputDevice::createDeviceByOption("vehroute-output", "routes", "routes_file.xsd");
        mySaveExits = OptionsCont::getOptions().getBool("vehroute-output.exit-times");
        myLastRouteOnly = OptionsCont::getOptions().getBool("vehroute-output.last-route");
        myDUAStyle = OptionsCont::getOptions().getBool("vehroute-output.dua");
        mySorted = myDUAStyle || OptionsCont::getOptions().getBool("vehroute-output.sorted");
        myIntendedDepart = OptionsCont::getOptions().getBool("vehroute-output.intended-depart");
        myRouteLength = OptionsCont::getOptions().getBool("vehroute-output.route-length");
        mySkipPTLines = OptionsCont::getOptions().getBool("vehroute-output.skip-ptlines");
        MSNet::getInstance()->addVehicleStateListener(&myStateListener);
    }
}


MSDevice_Vehroutes*
MSDevice_Vehroutes::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into, int maxRoutes) {
    if (maxRoutes < std::numeric_limits<int>::max()) {
        return new MSDevice_Vehroutes(v, "vehroute_" + v.getID(), maxRoutes);
    }
    if (OptionsCont::getOptions().isSet("vehroute-output")) {
        if (myLastRouteOnly) {
            maxRoutes = 0;
        }
        myStateListener.myDevices[&v] = new MSDevice_Vehroutes(v, "vehroute_" + v.getID(), maxRoutes);
        into.push_back(myStateListener.myDevices[&v]);
        return myStateListener.myDevices[&v];
    }
    return 0;
}


// ---------------------------------------------------------------------------
// MSDevice_Vehroutes::StateListener-methods
// ---------------------------------------------------------------------------
void
MSDevice_Vehroutes::StateListener::vehicleStateChanged(const SUMOVehicle* const vehicle, MSNet::VehicleState to) {
    if (to == MSNet::VEHICLE_STATE_NEWROUTE) {
        myDevices[vehicle]->addRoute();
    }
}


// ---------------------------------------------------------------------------
// MSDevice_Vehroutes-methods
// ---------------------------------------------------------------------------
MSDevice_Vehroutes::MSDevice_Vehroutes(SUMOVehicle& holder, const std::string& id, int maxRoutes) :
    MSDevice(holder, id),
    myCurrentRoute(&holder.getRoute()),
    myMaxRoutes(maxRoutes),
    myLastSavedAt(0),
    myDepartLane(-1),
    myDepartPos(-1),
    myDepartSpeed(-1),
    myDepartPosLat(0) {
    myCurrentRoute->addReference();
}


MSDevice_Vehroutes::~MSDevice_Vehroutes() {
    for (std::vector<RouteReplaceInfo>::iterator i = myReplacedRoutes.begin(); i != myReplacedRoutes.end(); ++i) {
        (*i).route->release();
    }
    myCurrentRoute->release();
    myStateListener.myDevices.erase(&myHolder);
}


bool
MSDevice_Vehroutes::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
        if (mySorted && myStateListener.myDevices[&veh] == this) {
            const SUMOTime departure = myIntendedDepart ? myHolder.getParameter().depart : MSNet::getInstance()->getCurrentTimeStep();
            myDepartureCounts[departure]++;
        }
        if (!MSGlobals::gUseMesoSim) {
            myDepartLane = static_cast<MSVehicle&>(veh).getLane()->getIndex();
            myDepartPosLat = static_cast<MSVehicle&>(veh).getLateralPositionOnLane();
        }
        myDepartSpeed = veh.getSpeed();
        myDepartPos = veh.getPositionOnLane();
    }
    return mySaveExits;
}


bool
MSDevice_Vehroutes::notifyLeave(SUMOVehicle& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (mySaveExits && reason != NOTIFICATION_LANE_CHANGE) {
        if (reason != NOTIFICATION_TELEPORT && myLastSavedAt == veh.getEdge()) { // need to check this for internal lanes
            myExits.back() = MSNet::getInstance()->getCurrentTimeStep();
        } else if (myLastSavedAt != veh.getEdge()) {
            myExits.push_back(MSNet::getInstance()->getCurrentTimeStep());
            myLastSavedAt = veh.getEdge();
        }
    }
    return mySaveExits;
}


void
MSDevice_Vehroutes::writeXMLRoute(OutputDevice& os, int index) const {
    if (index == 0 && myReplacedRoutes[index].route->size() == 2 &&
            myReplacedRoutes[index].route->getEdges().front()->isTazConnector() &&
            myReplacedRoutes[index].route->getEdges().back()->isTazConnector()) {
        return;
    }
    // check if a previous route shall be written
    os.openTag(SUMO_TAG_ROUTE);
    if (index >= 0) {
        assert((int)myReplacedRoutes.size() > index);
        // write edge on which the vehicle was when the route was valid
        os << " replacedOnEdge=\"";
        if (myReplacedRoutes[index].edge) {
            os << myReplacedRoutes[index].edge->getID();
        }
        // write the time at which the route was replaced
        os << "\" replacedAtTime=\"" << time2string(myReplacedRoutes[index].time) << "\" probability=\"0\" edges=\"";
        // get the route
        int i = index;
        while (i > 0 && myReplacedRoutes[i - 1].edge) {
            i--;
        }
        const MSEdge* lastEdge = 0;
        for (; i < index; ++i) {
            myReplacedRoutes[i].route->writeEdgeIDs(os, lastEdge, myReplacedRoutes[i].edge);
            lastEdge = myReplacedRoutes[i].edge;
        }
        myReplacedRoutes[index].route->writeEdgeIDs(os, lastEdge);
    } else {
        os << " edges=\"";
        const MSEdge* lastEdge = 0;
        int numWritten = 0;
        if (myHolder.getNumberReroutes() > 0) {
            assert((int)myReplacedRoutes.size() <= myHolder.getNumberReroutes());
            int i = (int)myReplacedRoutes.size();
            while (i > 0 && myReplacedRoutes[i - 1].edge) {
                i--;
            }
            for (; i < (int)myReplacedRoutes.size(); ++i) {
                numWritten += myReplacedRoutes[i].route->writeEdgeIDs(os, lastEdge, myReplacedRoutes[i].edge);
                lastEdge = myReplacedRoutes[i].edge;
            }
        }
        const MSEdge* upTo = 0;
        if (mySaveExits) {
            int remainingWithExitTime = (int)myExits.size() - numWritten;
            assert(remainingWithExitTime >= 0);
            assert(remainingWithExitTime <= (int)myCurrentRoute->size());
            if (remainingWithExitTime < (int)myCurrentRoute->size()) {
                upTo = *(myCurrentRoute->begin() + remainingWithExitTime);
            }
        }
        myCurrentRoute->writeEdgeIDs(os, lastEdge, upTo);
        if (mySaveExits) {
            os << "\" exitTimes=\"";
            for (std::vector<SUMOTime>::const_iterator it = myExits.begin(); it != myExits.end(); ++it) {
                if (it != myExits.begin()) {
                    os << " ";
                }
                os << time2string(*it);
            }
        }
    }
    (os << "\"").closeTag();
}


void
MSDevice_Vehroutes::generateOutput() const {
    writeOutput(true);
}


void
MSDevice_Vehroutes::writeOutput(const bool hasArrived) const {
    if (mySkipPTLines && myHolder.getParameter().line != "") {
        return;
    }
    OutputDevice& routeOut = OutputDevice::getDeviceByOption("vehroute-output");
    OutputDevice_String od(routeOut.isBinary(), 1);
    SUMOVehicleParameter tmp = myHolder.getParameter();
    tmp.depart = myIntendedDepart ? myHolder.getParameter().depart : myHolder.getDeparture();
    if (!MSGlobals::gUseMesoSim) {
        if (tmp.wasSet(VEHPARS_DEPARTLANE_SET)) {
            tmp.departLaneProcedure = DEPART_LANE_GIVEN;
            tmp.departLane = myDepartLane;
        }
        if (tmp.wasSet(VEHPARS_DEPARTPOSLAT_SET)) {
            tmp.departPosLatProcedure = DEPART_POSLAT_GIVEN;
            tmp.departPosLat = myDepartPosLat;
        }
    }
    if (tmp.wasSet(VEHPARS_DEPARTPOS_SET)) {
        tmp.departPosProcedure = DEPART_POS_GIVEN;
        tmp.departPos = myDepartPos;
    }
    if (tmp.wasSet(VEHPARS_DEPARTSPEED_SET)) {
        tmp.departSpeedProcedure = DEPART_SPEED_GIVEN;
        tmp.departSpeed = myDepartSpeed;
    }
    const std::string typeID = myHolder.getVehicleType().getID() != DEFAULT_VTYPE_ID ? myHolder.getVehicleType().getID() : "";
    tmp.write(od, OptionsCont::getOptions(), SUMO_TAG_VEHICLE, typeID);
    if (hasArrived) {
        od.writeAttr("arrival", time2string(MSNet::getInstance()->getCurrentTimeStep()));
        if (myRouteLength) {
            const bool includeInternalLengths = MSGlobals::gUsingInternalLanes && MSNet::getInstance()->hasInternalLinks();
            const double routeLength = myHolder.getRoute().getDistanceBetween(myHolder.getDepartPos(), myHolder.getArrivalPos(),
                                       myHolder.getRoute().begin(), myHolder.getCurrentRouteEdge(), includeInternalLengths);
            od.writeAttr("routeLength", routeLength);
        }
    }
    if (myDUAStyle) {
        const RandomDistributor<const MSRoute*>* const routeDist = MSRoute::distDictionary("!" + myHolder.getID());
        if (routeDist != 0) {
            const std::vector<const MSRoute*>& routes = routeDist->getVals();
            unsigned index = 0;
            while (index < routes.size() && routes[index] != myCurrentRoute) {
                ++index;
            }
            od.openTag(SUMO_TAG_ROUTE_DISTRIBUTION).writeAttr(SUMO_ATTR_LAST, index);
            const std::vector<double>& probs = routeDist->getProbs();
            for (int i = 0; i < (int)routes.size(); ++i) {
                od.setPrecision();
                od.openTag(SUMO_TAG_ROUTE).writeAttr(SUMO_ATTR_COST, routes[i]->getCosts());
                od.setPrecision(8);
                od.writeAttr(SUMO_ATTR_PROB, probs[i]);
                od.setPrecision();
                od << " edges=\"";
                routes[i]->writeEdgeIDs(od, *routes[i]->begin());
                (od << "\"").closeTag();
            }
            od.closeTag();
        } else {
            writeXMLRoute(od);
        }
    } else {
        if (myReplacedRoutes.size() > 0) {
            od.openTag(SUMO_TAG_ROUTE_DISTRIBUTION);
            for (int i = 0; i < (int)myReplacedRoutes.size(); ++i) {
                writeXMLRoute(od, i);
            }
        }
        writeXMLRoute(od);
        if (myReplacedRoutes.size() > 0) {
            od.closeTag();
        }
    }
    for (std::vector<SUMOVehicleParameter::Stop>::const_iterator i = myHolder.getParameter().stops.begin(); i != myHolder.getParameter().stops.end(); ++i) {
        i->write(od);
    }
    for (std::vector<SUMOVehicleParameter::Stop>::const_iterator i = myHolder.getRoute().getStops().begin(); i != myHolder.getRoute().getStops().end(); ++i) {
        i->write(od);
    }
    myHolder.getParameter().writeParams(od);
    od.closeTag();
    od.lf();
    if (mySorted) {
        myRouteInfos[tmp.depart][myHolder.getID()] = od.getString();
        myDepartureCounts[tmp.depart]--;
        std::map<const SUMOTime, int>::iterator it = myDepartureCounts.begin();
        while (it != myDepartureCounts.end() && it->second == 0) {
            std::map<const std::string, std::string>& infos = myRouteInfos[it->first];
            for (std::map<const std::string, std::string>::const_iterator it2 = infos.begin(); it2 != infos.end(); ++it2) {
                routeOut << it2->second;
            }
            myRouteInfos.erase(it->first);
            myDepartureCounts.erase(it);
            it = myDepartureCounts.begin();
        }
    } else {
        routeOut << od.getString();
    }
}


const MSRoute*
MSDevice_Vehroutes::getRoute(int index) const {
    if (index < (int)myReplacedRoutes.size()) {
        return myReplacedRoutes[index].route;
    } else {
        return 0;
    }
}


void
MSDevice_Vehroutes::addRoute() {
    if (myMaxRoutes > 0) {
        if (myHolder.hasDeparted()) {
            myReplacedRoutes.push_back(RouteReplaceInfo(myHolder.getEdge(), MSNet::getInstance()->getCurrentTimeStep(), myCurrentRoute));
        } else {
            myReplacedRoutes.push_back(RouteReplaceInfo(0, MSNet::getInstance()->getCurrentTimeStep(), myCurrentRoute));
        }
        if ((int)myReplacedRoutes.size() > myMaxRoutes) {
            myReplacedRoutes.front().route->release();
            myReplacedRoutes.erase(myReplacedRoutes.begin());
        }
    } else {
        myCurrentRoute->release();
    }
    myCurrentRoute = &myHolder.getRoute();
    myCurrentRoute->addReference();
}


void
MSDevice_Vehroutes::generateOutputForUnfinished() {
    for (std::map<const SUMOVehicle*, MSDevice_Vehroutes*, Named::NamedLikeComparatorIdLess<SUMOVehicle> >::const_iterator it = myStateListener.myDevices.begin();
            it != myStateListener.myDevices.end(); ++it) {
        if (it->first->hasDeparted()) {
            it->second->writeOutput(false);
        }
    }
}


void
MSDevice_Vehroutes::saveState(OutputDevice& out) const {
    out.openTag(SUMO_TAG_DEVICE);
    out.writeAttr(SUMO_ATTR_ID, getID());
    std::vector<std::string> internals;
    if (!MSGlobals::gUseMesoSim) {
        internals.push_back(toString(myDepartLane));
        internals.push_back(toString(myDepartPosLat));
    }
    internals.push_back(toString(myDepartSpeed));
    internals.push_back(toString(myDepartPos));
    internals.push_back(toString(myReplacedRoutes.size()));
    for (int i = 0; i < (int)myReplacedRoutes.size(); ++i) {
        const std::string replacedOnEdge = myReplacedRoutes[i].edge == 0 ? "!NULL" : myReplacedRoutes[i].edge->getID();
        internals.push_back(replacedOnEdge);
        internals.push_back(toString(myReplacedRoutes[i].time));
        internals.push_back(myReplacedRoutes[i].route->getID());
    }
    out.writeAttr(SUMO_ATTR_STATE, toString(internals));
    out.closeTag();
}


void
MSDevice_Vehroutes::loadState(const SUMOSAXAttributes& attrs) {
    std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
    if (!MSGlobals::gUseMesoSim) {
        bis >> myDepartLane;
        bis >> myDepartPosLat;
    }
    bis >> myDepartSpeed;
    bis >> myDepartPos;
    int size;
    bis >> size;
    for (int i = 0; i < size; ++i) {
        std::string edgeID;
        SUMOTime time;
        std::string routeID;
        bis >> edgeID;
        bis >> time;
        bis >> routeID;
        myReplacedRoutes.push_back(RouteReplaceInfo(MSEdge::dictionary(edgeID), time, MSRoute::dictionary(routeID)));
    }
}


/****************************************************************************/
