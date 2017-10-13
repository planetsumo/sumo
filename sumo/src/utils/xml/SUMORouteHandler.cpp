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
/// @file    SUMORouteHandler.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
/// @version $Id$
///
// Parser for routes during their loading
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
#include <map>
#include <vector>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/common/UtilExceptions.h>
#include <utils/options/OptionsCont.h>
#include <utils/vehicle/SUMOVehicleParameter.h>
#include <utils/vehicle/SUMOVTypeParameter.h>
#include <utils/xml/SUMOVehicleParserHelper.h>
#include "SUMORouteHandler.h"


// ===========================================================================
// method definitions
// ===========================================================================
SUMORouteHandler::SUMORouteHandler(const std::string& file) :
    SUMOSAXHandler(file),
    myVehicleParameter(0),
    myLastDepart(-1),
    myActiveRouteColor(0),
    myCurrentVType(0),
    myBeginDefault(string2time(OptionsCont::getOptions().getString("begin"))),
    myEndDefault(string2time(OptionsCont::getOptions().getString("end"))),
    myFirstDepart(-1), myInsertStopEdgesAt(-1), myDefaultCFModel(SUMO_TAG_NOTHING) {
}


SUMORouteHandler::~SUMORouteHandler() {
    delete myCurrentVType;
}


SUMOTime
SUMORouteHandler::getLastDepart() const {
    return myLastDepart;
}


bool
SUMORouteHandler::checkLastDepart() {
    if (myVehicleParameter->departProcedure == DEPART_GIVEN) {
        if (myVehicleParameter->depart < myLastDepart) {
            WRITE_WARNING("Route file should be sorted by departure time, ignoring '" + myVehicleParameter->id + "'!");
            return false;
        }
    }
    return true;
}


void
SUMORouteHandler::registerLastDepart() {
    // register only non public transport to parse all public transport lines in advance
    if (myVehicleParameter->line == "" && myVehicleParameter->departProcedure == DEPART_GIVEN) {
        myLastDepart = myVehicleParameter->depart;
        if (myFirstDepart == -1) {
            myFirstDepart = myLastDepart;
        }
    }
    // else: we don't know when this vehicle will depart. keep the previous known depart time
}


void
SUMORouteHandler::myStartElement(int element,
                                 const SUMOSAXAttributes& attrs) {
    switch (element) {
        case SUMO_TAG_VEHICLE:
            delete myVehicleParameter;
            myVehicleParameter = SUMOVehicleParserHelper::parseVehicleAttributes(attrs);
            break;
        case SUMO_TAG_PERSON:
            delete myVehicleParameter;
            myVehicleParameter = SUMOVehicleParserHelper::parseVehicleAttributes(attrs, false, false, true);
            break;
        case SUMO_TAG_CONTAINER:
            delete myVehicleParameter;
            myVehicleParameter = SUMOVehicleParserHelper::parseVehicleAttributes(attrs);
            break;
        case SUMO_TAG_FLOW:
            delete myVehicleParameter;
            myVehicleParameter = SUMOVehicleParserHelper::parseFlowAttributes(attrs, myBeginDefault, myEndDefault);
            break;
        case SUMO_TAG_VTYPE:
            // XXX: Where is this deleted? Delegated to subclasses?! MSRouteHandler takes care of this, in case of RORouteHandler this is not obvious. Consider introduction of a shared_ptr
            myCurrentVType = SUMOVehicleParserHelper::beginVTypeParsing(attrs, getFileName(), myDefaultCFModel);
            break;
        case SUMO_TAG_VTYPE_DISTRIBUTION:
            openVehicleTypeDistribution(attrs);
            break;
        case SUMO_TAG_ROUTE:
            openRoute(attrs);
            break;
        case SUMO_TAG_ROUTE_DISTRIBUTION:
            openRouteDistribution(attrs);
            break;
        case SUMO_TAG_STOP:
            addStop(attrs);
            break;
        case SUMO_TAG_TRIP: {
            myVehicleParameter = SUMOVehicleParserHelper::parseVehicleAttributes(attrs, true);
            if (myVehicleParameter->id == "") {
                WRITE_WARNING("Omitting trip ids is deprecated!");
                myVehicleParameter->id = myIdSupplier.getNext();
            }
            myVehicleParameter->parametersSet |= VEHPARS_FORCE_REROUTE;
            myActiveRouteID = "!" + myVehicleParameter->id;
            break;
        }
        case SUMO_TAG_PERSONTRIP:
        case SUMO_TAG_WALK:
            if (attrs.hasAttribute(SUMO_ATTR_EDGES) || attrs.hasAttribute(SUMO_ATTR_ROUTE)) {
                addWalk(attrs);
            } else {
                addPersonTrip(attrs);
            }
            break;
        case SUMO_TAG_INTERVAL: {
            bool ok;
            myBeginDefault = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, 0, ok);
            myEndDefault = attrs.getSUMOTimeReporting(SUMO_ATTR_END, 0, ok);
            break;
        }
        case SUMO_TAG_PARAM:
            addParam(attrs);
            break;
        default:
            // parse embedded car following model information
            if (myCurrentVType != 0) {
                WRITE_WARNING("Defining car following parameters in a nested element is deprecated in vType '" + myCurrentVType->id + "', use attributes instead!");
                SUMOVehicleParserHelper::parseVTypeEmbedded(*myCurrentVType, (SumoXMLTag)element, attrs);
            }
            break;
    }
}


void
SUMORouteHandler::myEndElement(int element) {
    switch (element) {
        case SUMO_TAG_ROUTE:
            closeRoute();
            break;
        case SUMO_TAG_PERSON:
            closePerson();
            delete myVehicleParameter;
            myVehicleParameter = 0;
            break;
        case SUMO_TAG_CONTAINER:
            closeContainer();
            delete myVehicleParameter;
            myVehicleParameter = 0;
            break;
        case SUMO_TAG_VEHICLE:
            if (myVehicleParameter->repetitionNumber > 0) {
                myVehicleParameter->repetitionNumber++; // for backwards compatibility
                // it is a flow, thus no break here
            } else {
                closeVehicle();
                delete myVehicleParameter;
                myVehicleParameter = 0;
                break;
            }
        case SUMO_TAG_FLOW:
            closeFlow();
            break;
        case SUMO_TAG_VTYPE_DISTRIBUTION:
            closeVehicleTypeDistribution();
            break;
        case SUMO_TAG_ROUTE_DISTRIBUTION:
            closeRouteDistribution();
            break;
        case SUMO_TAG_INTERVAL:
            myBeginDefault = string2time(OptionsCont::getOptions().getString("begin"));
            myEndDefault = string2time(OptionsCont::getOptions().getString("end"));
            break;
        default:
            break;
    }
}


bool
SUMORouteHandler::checkStopPos(double& startPos, double& endPos, const double laneLength,
                               const double minLength, const bool friendlyPos) {
    if (minLength > laneLength) {
        return false;
    }
    if (startPos < 0) {
        startPos += laneLength;
    }
    if (endPos < 0) {
        endPos += laneLength;
    }
    if (endPos < minLength || endPos > laneLength) {
        if (!friendlyPos) {
            return false;
        }
        if (endPos < minLength) {
            endPos = minLength;
        }
        if (endPos > laneLength) {
            endPos = laneLength;
        }
    }
    if (startPos < 0 || startPos > endPos - minLength) {
        if (!friendlyPos) {
            return false;
        }
        if (startPos < 0) {
            startPos = 0;
        }
        if (startPos > endPos - minLength) {
            startPos = endPos - minLength;
        }
    }
    return true;
}


void
SUMORouteHandler::addParam(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string key = attrs.get<std::string>(SUMO_ATTR_KEY, 0, ok);
    // circumventing empty string test
    const std::string val = attrs.hasAttribute(SUMO_ATTR_VALUE) ? attrs.getString(SUMO_ATTR_VALUE) : "";
    if (myVehicleParameter != 0) {
        myVehicleParameter->setParameter(key, val);
    } else if (myCurrentVType != 0) {
        myCurrentVType->setParameter(key, val);
    }
}


bool
SUMORouteHandler::parseStop(SUMOVehicleParameter::Stop& stop, const SUMOSAXAttributes& attrs, std::string errorSuffix, MsgHandler* const errorOutput) {
    stop.parametersSet = 0;
    if (attrs.hasAttribute(SUMO_ATTR_ENDPOS)) {
        stop.parametersSet |= STOP_END_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_STARTPOS)) {
        stop.parametersSet |= STOP_START_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_TRIGGERED)) {
        stop.parametersSet |= STOP_TRIGGER_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_CONTAINER_TRIGGERED)) {
        stop.parametersSet |= STOP_CONTAINER_TRIGGER_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_PARKING)) {
        stop.parametersSet |= STOP_PARKING_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_EXPECTED)) {
        stop.parametersSet |= STOP_EXPECTED_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_EXPECTED_CONTAINERS)) {
        stop.parametersSet |= STOP_EXPECTED_CONTAINERS_SET;
    }
    bool ok = true;
    stop.busstop = attrs.getOpt<std::string>(SUMO_ATTR_BUS_STOP, 0, ok, "");
    stop.chargingStation = attrs.getOpt<std::string>(SUMO_ATTR_CHARGING_STATION, 0, ok, "");
    stop.containerstop = attrs.getOpt<std::string>(SUMO_ATTR_CONTAINER_STOP, 0, ok, "");
    stop.parkingarea = attrs.getOpt<std::string>(SUMO_ATTR_PARKING_AREA, 0, ok, "");
    if (stop.busstop != "") {
        errorSuffix = " at '" + stop.busstop + "'" + errorSuffix;
    } else if (stop.chargingStation != "") {
        errorSuffix = " at '" + stop.chargingStation + "'" + errorSuffix;
    } else if (stop.containerstop != "") {
        errorSuffix = " at '" + stop.containerstop + "'" + errorSuffix;
    } else if (stop.parkingarea != "") {
        errorSuffix = " at '" + stop.parkingarea + "'" + errorSuffix;
    } else {
        errorSuffix = " on lane '" + stop.lane + "'" + errorSuffix;
    }
    // get the standing duration
    if (!attrs.hasAttribute(SUMO_ATTR_DURATION) && !attrs.hasAttribute(SUMO_ATTR_UNTIL)) {
        if (attrs.hasAttribute(SUMO_ATTR_CONTAINER_TRIGGERED)) {
            stop.containerTriggered = attrs.getOpt<bool>(SUMO_ATTR_CONTAINER_TRIGGERED, 0, ok, true);
            stop.triggered = attrs.getOpt<bool>(SUMO_ATTR_TRIGGERED, 0, ok, false);
        } else {
            stop.triggered = attrs.getOpt<bool>(SUMO_ATTR_TRIGGERED, 0, ok, true);
            stop.containerTriggered = attrs.getOpt<bool>(SUMO_ATTR_CONTAINER_TRIGGERED, 0, ok, false);
        }
        stop.duration = -1;
        stop.until = -1;
    } else {
        stop.duration = attrs.getOptSUMOTimeReporting(SUMO_ATTR_DURATION, 0, ok, -1);
        stop.until = attrs.getOptSUMOTimeReporting(SUMO_ATTR_UNTIL, 0, ok, -1);
        if (!ok || (stop.duration < 0 && stop.until < 0)) {
            errorOutput->inform("Invalid duration or end time is given for a stop" + errorSuffix);
            return false;
        }
        stop.triggered = attrs.getOpt<bool>(SUMO_ATTR_TRIGGERED, 0, ok, false);
        stop.containerTriggered = attrs.getOpt<bool>(SUMO_ATTR_CONTAINER_TRIGGERED, 0, ok, false);
    }
    stop.parking = attrs.getOpt<bool>(SUMO_ATTR_PARKING, 0, ok, stop.triggered || stop.containerTriggered || stop.parkingarea != "");
    if (stop.parkingarea != "" && !stop.parking) {
        ok = false;
    }
    if (!ok) {
        errorOutput->inform("Invalid bool for 'triggered', 'containerTriggered' or 'parking' for stop" + errorSuffix);
        return false;
    }

    // expected persons
    std::string expectedStr = attrs.getOpt<std::string>(SUMO_ATTR_EXPECTED, 0, ok, "");
    std::set<std::string> personIDs;
    SUMOSAXAttributes::parseStringSet(expectedStr, personIDs);
    stop.awaitedPersons = personIDs;

    // expected containers
    std::string expectedContainersStr = attrs.getOpt<std::string>(SUMO_ATTR_EXPECTED_CONTAINERS, 0, ok, "");
    std::set<std::string> containerIDs;
    SUMOSAXAttributes::parseStringSet(expectedContainersStr, containerIDs);
    stop.awaitedContainers = containerIDs;

    const std::string idx = attrs.getOpt<std::string>(SUMO_ATTR_INDEX, 0, ok, "end");
    if (idx == "end") {
        stop.index = STOP_INDEX_END;
    } else if (idx == "fit") {
        stop.index = STOP_INDEX_FIT;
    } else {
        stop.index = attrs.get<int>(SUMO_ATTR_INDEX, 0, ok);
        if (!ok || stop.index < 0) {
            errorOutput->inform("Invalid 'index' for stop" + errorSuffix);
            return false;
        }
    }
    return true;
}

/****************************************************************************/
