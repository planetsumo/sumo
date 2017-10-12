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
/// @file    ROFrame.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// Sets and checks options for routing
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/RandHelper.h>
#include <utils/common/ToString.h>
#include <utils/common/SUMOTime.h>
#include "ROFrame.h"


// ===========================================================================
// method definitions
// ===========================================================================
void
ROFrame::fillOptions(OptionsCont& oc) {
    // register options
    // register I/O options
    oc.doRegister("output-file", 'o', new Option_FileName());
    oc.addSynonyme("output-file", "output");
    oc.addDescription("output-file", "Output", "Write generated routes to FILE");

    oc.doRegister("vtype-output", new Option_FileName());
    oc.addSynonyme("vtype-output", "vtype");
    oc.addDescription("vtype-output", "Output", "Write used vehicle types into separate FILE");

    oc.doRegister("keep-vtype-distributions", new Option_Bool(false));
    oc.addDescription("keep-vtype-distributions", "Output", "Keep vTypeDistribution ids when writing vehicles and their types");

    oc.doRegister("net-file", 'n', new Option_FileName());
    oc.addSynonyme("net-file", "net");
    oc.addDescription("net-file", "Input", "Use FILE as SUMO-network to route on");

    oc.doRegister("additional-files", 'a', new Option_FileName());
    oc.addSynonyme("additional-files", "d", true);
    oc.addSynonyme("additional-files", "additional");
    oc.addSynonyme("additional-files", "taz-files");
    oc.addSynonyme("additional-files", "districts", true);
    oc.addDescription("additional-files", "Input", "Read additional network data (districts, bus stops) from FILE(s)");

    oc.doRegister("route-files", 'r', new Option_FileName());
    oc.addSynonyme("route-files", "flow-files", true);
    oc.addSynonyme("route-files", "flows", true);
    oc.addSynonyme("route-files", "f", true);
    oc.addSynonyme("route-files", "alternative-files", true);
    oc.addSynonyme("route-files", "alternatives-files", true);
    oc.addSynonyme("route-files", "trip-files", true);
    oc.addSynonyme("route-files", "trips", true);
    oc.addDescription("route-files", "Input", "Read sumo routes, alternatives, flows, and trips from FILE(s)");

    // register the time settings
    oc.doRegister("begin", 'b', new Option_String("0", "TIME"));
    oc.addDescription("begin", "Time", "Defines the begin time; Previous trips will be discarded");

    oc.doRegister("end", 'e', new Option_String(SUMOTIME_MAXSTRING, "TIME"));
    oc.addDescription("end", "Time", "Defines the end time; Later trips will be discarded; Defaults to the maximum time that SUMO can represent");

    // register the processing options
    oc.doRegister("ignore-errors", new Option_Bool(false));
    oc.addSynonyme("ignore-errors", "continue-on-unbuild", true);
    oc.addDescription("ignore-errors", "Processing", "Continue if a route could not be build");

    oc.doRegister("unsorted-input", new Option_Bool(false));
    oc.addSynonyme("unsorted-input", "unsorted");
    oc.addDescription("unsorted-input", "Processing", "Assume input is unsorted");

    oc.doRegister("route-steps", 's', new Option_String("200", "TIME"));
    oc.addDescription("route-steps", "Processing", "Load routes for the next number of seconds ahead");

    oc.doRegister("randomize-flows", new Option_Bool(false));
    oc.addDescription("randomize-flows", "Processing", "generate random departure times for flow input");

    oc.doRegister("max-alternatives", new Option_Integer(5));
    oc.addDescription("max-alternatives", "Processing", "Prune the number of alternatives to INT");

    oc.doRegister("remove-loops", new Option_Bool(false));
    oc.addDescription("remove-loops", "Processing", "Remove loops within the route; Remove turnarounds at start and end of the route");

    oc.doRegister("repair", new Option_Bool(false));
    oc.addDescription("repair", "Processing", "Tries to correct a false route");

    oc.doRegister("repair.from", new Option_Bool(false));
    oc.addDescription("repair.from", "Processing", "Tries to correct an invalid starting edge by using the first usable edge instead");

    oc.doRegister("repair.to", new Option_Bool(false));
    oc.addDescription("repair.to", "Processing", "Tries to correct an invalid destination edge by using the last usable edge instead");

    oc.doRegister("weights.interpolate", new Option_Bool(false));
    oc.addSynonyme("weights.interpolate", "interpolate", true);
    oc.addDescription("weights.interpolate", "Processing", "Interpolate edge weights at interval boundaries");

    oc.doRegister("with-taz", new Option_Bool(false));
    oc.addDescription("with-taz", "Processing", "Use origin and destination zones (districts) for in- and output");

    oc.doRegister("bulk-routing", new Option_Bool(false));
    oc.addDescription("bulk-routing", "Processing", "Aggregate routing queries with the same origin");

    oc.doRegister("routing-threads", new Option_Integer(0));
    oc.addDescription("routing-threads", "Processing", "The number of parallel execution threads used for routing");

    // register defaults options
    oc.doRegister("departlane", new Option_String());
    oc.addDescription("departlane", "Defaults", "Assigns a default depart lane");

    oc.doRegister("departpos", new Option_String());
    oc.addDescription("departpos", "Defaults", "Assigns a default depart position");

    oc.doRegister("departspeed", new Option_String());
    oc.addDescription("departspeed", "Defaults", "Assigns a default depart speed");

    oc.doRegister("arrivallane", new Option_String());
    oc.addDescription("arrivallane", "Defaults", "Assigns a default arrival lane");

    oc.doRegister("arrivalpos", new Option_String());
    oc.addDescription("arrivalpos", "Defaults", "Assigns a default arrival position");

    oc.doRegister("arrivalspeed", new Option_String());
    oc.addDescription("arrivalspeed", "Defaults", "Assigns a default arrival speed");

    oc.doRegister("defaults-override", new Option_Bool(false));
    oc.addDescription("defaults-override", "Defaults", "Defaults will override given values");


    // register report options
    oc.doRegister("stats-period", new Option_Integer(-1));
    oc.addDescription("stats-period", "Report", "Defines how often statistics shall be printed");

    oc.doRegister("no-step-log", new Option_Bool(false));
    oc.addDescription("no-step-log", "Report", "Disable console output of route parsing step");
}


bool
ROFrame::checkOptions(OptionsCont& oc) {
    // check whether the output is valid and can be build
    if (!oc.isSet("output-file")) {
        WRITE_ERROR("No output specified.");
        return false;
    }
    //
    if (oc.getInt("max-alternatives") < 2) {
        WRITE_ERROR("At least two alternatives should be enabled.");
        return false;
    }
#ifndef HAVE_FOX
    if (oc.getInt("routing-threads") > 1) {
        WRITE_ERROR("Parallel routing is only possible when compiled with Fox.");
        return false;
    }
#endif
    return true;
}



/****************************************************************************/

