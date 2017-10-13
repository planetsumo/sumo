/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2003-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSGlobals.h
/// @author  Daniel Krajzewicz
/// @author  Christian Roessel
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    late summer 2003
/// @version $Id$
///
// Some static variables for faster access
/****************************************************************************/
#ifndef MSGlobals_h
#define MSGlobals_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <map>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MELoop;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSGlobals
 * This class holds some static variables, filled mostly with values coming
 *  from the command line or the simulation configuration file.
 * They are stored herein to allow a faster access than from the options
 *  container.
 */
class MSGlobals {
public:
    /// Information whether empty edges shall be written on dump
    static bool gOmitEmptyEdgesOnDump;

    /* Allows switching between time step integration methods
     * "Semi-Implicit Euler" (default) and the ballistic update rule. */
    static bool gSemiImplicitEulerUpdate;

    /** Information how long the simulation shall wait until it recognizes
        a vehicle as a grid lock participant */
    static SUMOTime gTimeToGridlock;

    /** The time to detect grid locks on highways */
    static SUMOTime gTimeToGridlockHighways;

    /** Information how long a vehicle must wait for impatience to grow from 0 to 1 */
    static SUMOTime gTimeToImpatience;

    /// Information whether the simulation regards internal lanes
    static bool gUsingInternalLanes;

    /** Vehicles on internal lanes (even partially) with a waiting time that exceeds this threshold
     * no longer block cross-traffic on the junction */
    static SUMOTime gIgnoreJunctionBlocker;

    /** information whether the network shall check for collisions */
    static bool gCheck4Accidents;

    /** information whether the routes shall be checked for connectivity */
    static bool gCheckRoutes;

    /** information Duration of a lane change maneuver */
    static SUMOTime gLaneChangeDuration;

    /** Lateral resolution within a lane */
    static double gLateralResolution;

    /// Information whether a state has been loaded
    static bool gStateLoaded;

    /** Information whether mesosim shall be used */
    static bool gUseMesoSim;

    /** Information whether limited junction control shall be used */
    static bool gMesoLimitedJunctionControl;

    /** Information whether overtaking is enabled in the mesoscopic simulation */
    static bool gMesoOvertaking;

    /** scaling factor for macroscopic time penalty when passing tls controlled intersection */
    static double gMesoTLSPenalty;

    /** penalty time for passing a minor link */
    static SUMOTime gMesoMinorPenalty;

    /// mesoscopic simulation infrastructure
    static MELoop* gMesoNet;

    /// length of memory for waiting times (in millisecs)
    static SUMOTime gWaitingTimeMemory;

    /// default value for the interval between two action points for MSVehicle (defaults to DELTA_T)
    static SUMOTime gActionStepLength;

};


#endif

/****************************************************************************/

