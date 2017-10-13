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
/// @file    MSGlobals.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    late summer 2003
/// @version $Id$
///
// Some static variables for faster access
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSGlobals.h"


// ===========================================================================
// static member variable definitions
// ===========================================================================
bool MSGlobals::gOmitEmptyEdgesOnDump;

bool MSGlobals::gUsingInternalLanes;
SUMOTime MSGlobals::gIgnoreJunctionBlocker;

SUMOTime MSGlobals::gTimeToGridlock;
SUMOTime MSGlobals::gTimeToGridlockHighways;
SUMOTime MSGlobals::gTimeToImpatience;

bool MSGlobals::gCheck4Accidents;

bool MSGlobals::gCheckRoutes;

SUMOTime MSGlobals::gLaneChangeDuration;

double MSGlobals::gLateralResolution;

bool MSGlobals::gStateLoaded;
bool MSGlobals::gUseMesoSim;
bool MSGlobals::gMesoLimitedJunctionControl;
bool MSGlobals::gMesoOvertaking;
double MSGlobals::gMesoTLSPenalty;
SUMOTime MSGlobals::gMesoMinorPenalty;
MELoop* MSGlobals::gMesoNet;

bool MSGlobals::gSemiImplicitEulerUpdate;

SUMOTime MSGlobals::gWaitingTimeMemory;

SUMOTime MSGlobals::gActionStepLength;
/****************************************************************************/

