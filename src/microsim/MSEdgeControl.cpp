/****************************************************************************/
/// @file    MSEdgeControl.cpp
/// @author  Christian Roessel
/// @date    Mon, 09 Apr 2001
/// @version $Id$
///
// Stores edges and lanes, performs moving of vehicle
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2010 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
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

#include "MSEdgeControl.h"
#include "MSEdge.h"
#include "MSLane.h"
#include <iostream>
#include <vector>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
MSEdgeControl::MSEdgeControl(const std::vector< MSEdge* > &edges) throw()
        : myEdges(edges) {
}


MSEdgeControl::~MSEdgeControl() throw() {
}


void
MSEdgeControl::moveCritical(SUMOTime t) throw() {
    for (std::vector<MSEdge*>::iterator i=myEdges.begin(); i!=myEdges.end(); ++i) {
        (*i)->move1(t);
    }
}


void
MSEdgeControl::moveFirst(SUMOTime t) throw() {
    myWithVehicles2Integrate.clear();
    for (std::vector<MSEdge*>::iterator i=myEdges.begin(); i!=myEdges.end(); ++i) {
        const std::vector<MSLane*> &lanes = (*i)->getLanes();
        for (std::vector<MSLane*>::const_iterator j=lanes.begin(); j!=lanes.end(); ++j) {
            (*j)->setCritical(t, myWithVehicles2Integrate);
        }
    }
    for (std::vector<MSLane*>::iterator i=myWithVehicles2Integrate.begin(); i!=myWithVehicles2Integrate.end(); ++i) {
        (*i)->integrateNewVehicle(t);
    }
}


void
MSEdgeControl::detectCollisions(SUMOTime timestep) throw() {
    for (std::vector<MSEdge*>::iterator i=myEdges.begin(); i!=myEdges.end(); ++i) {
        const std::vector<MSLane*> &lanes = (*i)->getLanes();
        for (std::vector<MSLane*>::const_iterator j=lanes.begin(); j!=lanes.end(); ++j) {
            (*j)->detectCollisions(timestep);
        }
    }
}


std::vector<std::string>
MSEdgeControl::getEdgeNames() const throw() {
    std::vector<std::string> ret;
    for (std::vector<MSEdge*>::const_iterator i=myEdges.begin(); i!=myEdges.end(); ++i) {
        ret.push_back((*i)->getID());
    }
    return ret;
}


/****************************************************************************/

