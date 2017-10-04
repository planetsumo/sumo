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
/// @file    NLTriggerBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Tino Morenz
/// @author  Jakob Erdmann
/// @author  Eric Nicolay
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Thu, 17 Oct 2002
/// @version $Id$
///
// Builds trigger objects for microsim
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
#include <microsim/MSEventControl.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSParkingArea.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/output/MSRouteProbe.h>
#include <microsim/trigger/MSLaneSpeedTrigger.h>
#include <microsim/trigger/MSTriggeredRerouter.h>
#include <microsim/trigger/MSCalibrator.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/trigger/MSChargingStation.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/WrappingCommand.h>
#include <utils/options/OptionsCont.h>
#include "NLHandler.h"
#include "NLTriggerBuilder.h"
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/xml/XMLSubSys.h>


#include <mesosim/MELoop.h>
#include <mesosim/METriggeredCalibrator.h>


// ===========================================================================
// method definitions
// ===========================================================================
NLTriggerBuilder::NLTriggerBuilder()
    : myHandler(0), myParkingArea(0) {}


NLTriggerBuilder::~NLTriggerBuilder() {}

void
NLTriggerBuilder::setHandler(NLHandler* handler) {
    myHandler = handler;
}


void
NLTriggerBuilder::buildVaporizer(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    MSEdge* e = MSEdge::dictionary(id);
    if (e == 0) {
        WRITE_ERROR("Unknown edge ('" + id + "') referenced in a vaporizer.");
        return;
    }
    SUMOTime begin = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, 0, ok);
    SUMOTime end = attrs.getSUMOTimeReporting(SUMO_ATTR_END, 0, ok);
    if (!ok) {
        return;
    }
    if (begin < 0) {
        WRITE_ERROR("A vaporization begin time is negative (edge id='" + id + "').");
        return;
    }
    if (begin >= end) {
        WRITE_ERROR("A vaporization ends before it starts (edge id='" + id + "').");
        return;
    }
    if (end >= string2time(OptionsCont::getOptions().getString("begin"))) {
        Command* cb = new WrappingCommand< MSEdge >(e, &MSEdge::incVaporization);
        MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(cb, begin);
        Command* ce = new WrappingCommand< MSEdge >(e, &MSEdge::decVaporization);
        MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(ce, end);
    }
}



void
NLTriggerBuilder::parseAndBuildLaneSpeedTrigger(MSNet& net, const SUMOSAXAttributes& attrs,
        const std::string& base) {
    // get the id, throw if not given or empty...
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    // get the file name to read further definitions from
    std::string file = getFileName(attrs, base, true);
    std::string objectid = attrs.get<std::string>(SUMO_ATTR_LANES, id.c_str(), ok);
    if (!ok) {
        throw InvalidArgument("The lanes to use within MSLaneSpeedTrigger '" + id + "' are not known.");
    }
    std::vector<MSLane*> lanes;
    std::vector<std::string> laneIDs;
    SUMOSAXAttributes::parseStringVector(objectid, laneIDs);
    for (std::vector<std::string>::iterator i = laneIDs.begin(); i != laneIDs.end(); ++i) {
        MSLane* lane = MSLane::dictionary(*i);
        if (lane == 0) {
            throw InvalidArgument("The lane to use within MSLaneSpeedTrigger '" + id + "' is not known.");
        }
        lanes.push_back(lane);
    }
    if (lanes.size() == 0) {
        throw InvalidArgument("No lane defined for MSLaneSpeedTrigger '" + id + "'.");
    }
    try {
        MSLaneSpeedTrigger* trigger = buildLaneSpeedTrigger(net, id, lanes, file);
        if (file == "") {
            trigger->registerParent(SUMO_TAG_VSS, myHandler);
        }
    } catch (ProcessError& e) {
        throw InvalidArgument(e.what());
    }
}

void
NLTriggerBuilder::parseAndBuildChargingStation(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;

    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);

    if (!ok) {
        throw ProcessError();
    }

    // get the lane
    MSLane* lane = getLane(attrs, "chargingStation", id);

    // get the positions
    double frompos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0);
    double topos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, lane->getLength());
    double chargingPower = attrs.getOpt<double>(SUMO_ATTR_CHARGINGPOWER, id.c_str(), ok, 0);
    double efficiency = attrs.getOpt<double>(SUMO_ATTR_EFFICIENCY, id.c_str(), ok, 0);
    bool chargeInTransit = attrs.getOpt<bool>(SUMO_ATTR_CHARGEINTRANSIT, id.c_str(), ok, 0);
    double chargeDelay = attrs.getOpt<double>(SUMO_ATTR_CHARGEDELAY, id.c_str(), ok, 0);

    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);

    if (!ok || !myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos)) {
        throw InvalidArgument("Invalid position for Charging Station '" + id + "'.");
    }

    // build the Charging Station
    buildChargingStation(net, id, lane, frompos, topos, chargingPower, efficiency, chargeInTransit, chargeDelay);
}


void
NLTriggerBuilder::parseAndBuildStoppingPlace(MSNet& net, const SUMOSAXAttributes& attrs, const SumoXMLTag element) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }

    //get the name, leave blank if not given
    const std::string ptStopName = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");

    // get the lane
    MSLane* lane = getLane(attrs, toString(element), id);
    // get the positions
    double frompos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0);
    double topos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, lane->getLength());
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    if (!ok || !myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos)) {
        throw InvalidArgument("Invalid position for " + toString(element) + " '" + id + "'.");
    }
    // get the lines
    std::vector<std::string> lines;
    SUMOSAXAttributes::parseStringVector(attrs.getOpt<std::string>(SUMO_ATTR_LINES, id.c_str(), ok, "", false), lines);
    // build the bus stop
    buildStoppingPlace(net, id, lines, lane, frompos, topos, element, ptStopName);
}


void
NLTriggerBuilder::addAccess(MSNet& /* net */, const SUMOSAXAttributes& attrs) {
    // get the lane
    MSLane* lane = getLane(attrs, "access" , "");
    // get the positions
    bool ok = true;
    double pos = attrs.getOpt<double>(SUMO_ATTR_POSITION, "access", ok, 0);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, "access", ok, false);
    if (!ok || !myHandler->checkStopPos(pos, pos, lane->getLength(), 0, friendlyPos)) {
        throw InvalidArgument("Invalid position for access in stop '" + myCurrentStop->getID() + "'.");
    }
    // add bus stop access
    if (myCurrentStop != 0) {
        myCurrentStop->addAccess(lane, pos);
    }
}


void
NLTriggerBuilder::parseAndBeginParkingArea(MSNet& net, const SUMOSAXAttributes& attrs) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }
    // get the lane
    MSLane* lane = getLane(attrs, "parkingArea", id);
    // get the positions
    double frompos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), ok, 0);
    double topos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, lane->getLength());
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    unsigned int capacity = attrs.getOpt<int>(SUMO_ATTR_ROADSIDE_CAPACITY, id.c_str(), ok, 0);
    double width = attrs.getOpt<double>(SUMO_ATTR_WIDTH, id.c_str(), ok, 0);
    double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, id.c_str(), ok, 0);
    double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, id.c_str(), ok, 0);
    if (!ok || !myHandler->checkStopPos(frompos, topos, lane->getLength(), POSITION_EPS, friendlyPos)) {
        throw InvalidArgument("Invalid position for parking area '" + id + "'.");
    }
    // get the lines
    std::vector<std::string> lines;
    SUMOSAXAttributes::parseStringVector(attrs.getOpt<std::string>(SUMO_ATTR_LINES, id.c_str(), ok, "", false), lines);
    // build the parking area
    beginParkingArea(net, id, lines, lane, frompos, topos, capacity, width, length, angle);
}



void
NLTriggerBuilder::parseAndAddLotEntry(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    // Check for open parking area
    if (myParkingArea == 0) {
        throw ProcessError();
    }
    // get the positions
    double x = attrs.get<double>(SUMO_ATTR_X, "", ok);
    if (!ok) {
        throw InvalidArgument("Invalid x position for lot entry.");
    }
    double y = attrs.get<double>(SUMO_ATTR_Y, "", ok);
    if (!ok) {
        throw InvalidArgument("Invalid y position for lot entry.");
    }
    double z = attrs.getOpt<double>(SUMO_ATTR_Z, "", ok, 0.);
    double width = attrs.getOpt<double>(SUMO_ATTR_WIDTH, "", ok, myParkingArea->getWidth());
    double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, "", ok, myParkingArea->getLength());
    double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, "", ok, myParkingArea->getAngle());
    // add the lot entry
    addLotEntry(x, y, z, width, length, angle);
}


void
NLTriggerBuilder::parseAndBuildCalibrator(MSNet& net, const SUMOSAXAttributes& attrs,
        const std::string& base) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }
    MSLane* lane = 0;
    MSEdge* edge = 0;
    // get the file name to read further definitions from
    if (attrs.hasAttribute(SUMO_ATTR_EDGE)) {
        std::string edgeID = attrs.get<std::string>(SUMO_ATTR_EDGE, id.c_str(), ok);
        edge = MSEdge::dictionary(edgeID);
        if (edge == 0) {
            throw InvalidArgument("The edge " + edgeID + " to use within the calibrator '" + id + "' is not known.");
        }
        if (attrs.hasAttribute(SUMO_ATTR_LANE)) {
            lane = getLane(attrs, "calibrator", id);
            if (&lane->getEdge() != edge) {
                throw InvalidArgument("The edge " + edgeID + " to use within the calibrator '" + id
                                      + "' does not match the calibrator lane '" + lane->getID() + ".");
            }
        }
    } else {
        lane = getLane(attrs, "calibrator", id);
        edge = &lane->getEdge();
    }
    const double pos = getPosition(attrs, lane, "calibrator", id, edge);
    const SUMOTime freq = attrs.getOptSUMOTimeReporting(SUMO_ATTR_FREQUENCY, id.c_str(), ok, DELTA_T); // !!! no error handling
    std::string file = getFileName(attrs, base, true);
    std::string outfile = attrs.getOpt<std::string>(SUMO_ATTR_OUTPUT, id.c_str(), ok, "");
    std::string routeProbe = attrs.getOpt<std::string>(SUMO_ATTR_ROUTEPROBE, id.c_str(), ok, "");
    MSRouteProbe* probe = 0;
    if (routeProbe != "") {
        probe = dynamic_cast<MSRouteProbe*>(net.getDetectorControl().getTypedDetectors(SUMO_TAG_ROUTEPROBE).get(routeProbe));
    }
    if (MSGlobals::gUseMesoSim) {
        if (lane != 0 && edge->getLanes().size() > 1) {
            WRITE_WARNING("Meso calibrator '" + id
                          + "' defined for lane '" + lane->getID()
                          + "' will collect data for all lanes of edge '" + edge->getID() + "'.");
        }
        METriggeredCalibrator* trigger = buildMECalibrator(net, id, edge, pos, file, outfile, freq, probe);
        if (file == "") {
            trigger->registerParent(SUMO_TAG_CALIBRATOR, myHandler);
        }
    } else {
        MSCalibrator* trigger = buildCalibrator(net, id, edge, lane, pos, file, outfile, freq, probe);
        if (file == "") {
            trigger->registerParent(SUMO_TAG_CALIBRATOR, myHandler);
        }
    }
}


void
NLTriggerBuilder::parseAndBuildRerouter(MSNet& net, const SUMOSAXAttributes& attrs,
                                        const std::string& base) {
    bool ok = true;
    // get the id, throw if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        throw ProcessError();
    }
    // get the file name to read further definitions from
    std::string file = getFileName(attrs, base, true);
    std::string objectid = attrs.get<std::string>(SUMO_ATTR_EDGES, id.c_str(), ok);
    if (!ok) {
        throw InvalidArgument("The edge to use within MSTriggeredRerouter '" + id + "' is not known.");
    }
    MSEdgeVector edges;
    std::vector<std::string> edgeIDs;
    SUMOSAXAttributes::parseStringVector(objectid, edgeIDs);
    for (std::vector<std::string>::iterator i = edgeIDs.begin(); i != edgeIDs.end(); ++i) {
        MSEdge* edge = MSEdge::dictionary(*i);
        if (edge == 0) {
            throw InvalidArgument("The edge to use within MSTriggeredRerouter '" + id + "' is not known.");
        }
        edges.push_back(edge);
    }
    if (edges.size() == 0) {
        throw InvalidArgument("No edges found for MSTriggeredRerouter '" + id + "'.");
    }
    double prob = attrs.getOpt<double>(SUMO_ATTR_PROB, id.c_str(), ok, 1);
    bool off = attrs.getOpt<bool>(SUMO_ATTR_OFF, id.c_str(), ok, false);
    if (!ok) {
        throw InvalidArgument("Could not parse MSTriggeredRerouter '" + id + "'.");
    }
    MSTriggeredRerouter* trigger = buildRerouter(net, id, edges, prob, file, off);
    // read in the trigger description
    if (file == "") {
        trigger->registerParent(SUMO_TAG_REROUTER, myHandler);
    } else if (!XMLSubSys::runParser(*trigger, file)) {
        throw ProcessError();
    }
}


// -------------------------


MSLaneSpeedTrigger*
NLTriggerBuilder::buildLaneSpeedTrigger(MSNet& /*net*/, const std::string& id,
                                        const std::vector<MSLane*>& destLanes,
                                        const std::string& file) {
    return new MSLaneSpeedTrigger(id, destLanes, file);
}


METriggeredCalibrator*
NLTriggerBuilder::buildMECalibrator(MSNet& /*net*/, const std::string& id,
                                    const MSEdge* edge,
                                    double pos,
                                    const std::string& file,
                                    const std::string& outfile,
                                    const SUMOTime freq, MSRouteProbe* probe) {
    return new METriggeredCalibrator(id, edge, pos, file, outfile, freq, MSGlobals::gMesoNet->getSegmentForEdge(*edge, pos)->getLength(), probe);
}


MSCalibrator*
NLTriggerBuilder::buildCalibrator(MSNet& /*net*/, const std::string& id,
                                  MSEdge* edge,
                                  MSLane* lane,
                                  double pos,
                                  const std::string& file,
                                  const std::string& outfile,
                                  const SUMOTime freq, const MSRouteProbe* probe) {
    return new MSCalibrator(id, edge, lane, pos, file, outfile, freq, edge->getLength(), probe);
}


MSTriggeredRerouter*
NLTriggerBuilder::buildRerouter(MSNet&, const std::string& id,
                                MSEdgeVector& edges,
                                double prob, const std::string& file, bool off) {
    return new MSTriggeredRerouter(id, edges, prob, file, off);
}


void
NLTriggerBuilder::buildStoppingPlace(MSNet& net, std::string id, std::vector<std::string> lines, MSLane* lane,
                                     double frompos, double topos, const SumoXMLTag element, std::string ptStopName) {
    myCurrentStop = new MSStoppingPlace(id, lines, *lane, frompos, topos, ptStopName);
    const bool success = element == SUMO_TAG_CONTAINER_STOP ? net.addContainerStop(myCurrentStop) : net.addBusStop(myCurrentStop);
    if (!success) {
        delete myCurrentStop;
        myCurrentStop = 0;
        throw InvalidArgument("Could not build " + toString(element) + " '" + id + "'; probably declared twice.");
    }
}


void
NLTriggerBuilder::beginParkingArea(MSNet& net, const std::string& id,
                                   const std::vector<std::string>& lines,
                                   MSLane* lane, double frompos, double topos,
                                   unsigned int capacity,
                                   double width, double length, double angle) {
    // Close previous parking area if there are not lots inside

    MSParkingArea* stop = new MSParkingArea(id, lines, *lane, frompos, topos, capacity, width, length, angle);
    if (!net.addParkingArea(stop)) {
        delete stop;
        throw InvalidArgument("Could not build parking area '" + id + "'; probably declared twice.");
    } else {
        myParkingArea = stop;
    }
}


void
NLTriggerBuilder::addLotEntry(double x, double y, double z,
                              double width, double length, double angle) {
    if (myParkingArea != 0) {
        myParkingArea->addLotEntry(x, y, z, width, length, angle);
    } else {
        throw InvalidArgument("Could not add lot entry outside a parking area.");
    }
}


void
NLTriggerBuilder::endParkingArea() {
    if (myParkingArea != 0) {
        myParkingArea = 0;
    } else {
        throw InvalidArgument("Could not end a parking area that is not opened.");
    }
}


void
NLTriggerBuilder::buildChargingStation(MSNet& net, const std::string& id, MSLane* lane, double frompos, double topos,
                                       double chargingPower, double efficiency, bool chargeInTransit, double chargeDelay) {
    MSChargingStation* chargingStation = new MSChargingStation(id, *lane, frompos, topos, chargingPower, efficiency, chargeInTransit, chargeDelay);

    if (!net.addChargingStation(chargingStation)) {
        delete chargingStation;
        throw InvalidArgument("Could not build Charging Station '" + id + "'; probably declared twice.");
    }
}

std::string
NLTriggerBuilder::getFileName(const SUMOSAXAttributes& attrs,
                              const std::string& base,
                              const bool allowEmpty) {
    // get the file name to read further definitions from
    bool ok = true;
    std::string file = attrs.getOpt<std::string>(SUMO_ATTR_FILE, 0, ok, "");
    if (file == "") {
        if (allowEmpty) {
            return file;
        }
        throw InvalidArgument("No filename given.");
    }
    // check whether absolute or relative filenames are given
    if (!FileHelpers::isAbsolute(file)) {
        return FileHelpers::getConfigurationRelative(base, file);
    }
    return file;
}


MSLane*
NLTriggerBuilder::getLane(const SUMOSAXAttributes& attrs,
                          const std::string& tt,
                          const std::string& tid) {
    bool ok = true;
    std::string objectid = attrs.get<std::string>(SUMO_ATTR_LANE, tid.c_str(), ok);
    MSLane* lane = MSLane::dictionary(objectid);
    if (lane == 0) {
        throw InvalidArgument("The lane " + objectid + " to use within the " + tt + " '" + tid + "' is not known.");
    }
    return lane;
}


double
NLTriggerBuilder::getPosition(const SUMOSAXAttributes& attrs,
                              MSLane* lane,
                              const std::string& tt, const std::string& tid,
                              MSEdge* edge) {
    assert(lane != 0 || edge != 0);
    const double length = lane != 0 ? lane->getLength() : edge->getLength();
    bool ok = true;
    double pos = attrs.get<double>(SUMO_ATTR_POSITION, 0, ok);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, 0, ok, false);
    if (!ok) {
        throw InvalidArgument("Error on parsing a position information.");
    }
    if (pos < 0) {
        pos = length + pos;
    }
    if (pos > length) {
        if (friendlyPos) {
            pos = length - (double) 0.1;
        } else {
            if (lane != 0) {
                throw InvalidArgument("The position of " + tt + " '" + tid + "' lies beyond the lane's '" + lane->getID() + "' length.");
            } else {
                throw InvalidArgument("The position of " + tt + " '" + tid + "' lies beyond the edges's '" + edge->getID() + "' length.");
            }
        }
    }
    return pos;
}



/****************************************************************************/
