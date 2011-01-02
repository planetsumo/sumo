/****************************************************************************/
/// @file    MSEdgeControl.h
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
#ifndef MSEdgeControl_h
#define MSEdgeControl_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <list>
#include <set>
#include "MSEdge.h"


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class BinaryInputDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSEdgeControl
 * @brief Stores edges and lanes, performs moving of vehicle
 */
class MSEdgeControl {
public:
    /// @brief Container for edges.
    typedef std::vector< MSEdge* > EdgeCont;

public:
    /** @brief Constructor
     *
     * @param[in] edges The loaded edges
     * @todo Assure both containers are not 0
     */
    MSEdgeControl(const std::vector< MSEdge* > &edges) throw();


    /// @brief Destructor.
    ~MSEdgeControl() throw();


    /// @name Interfaces for longitudinal vehicle movement
    /// @{

    /** @brief Moves (precomputes) critical vehicles
     *
     * "Critical" are those vehicles that interact with the next junction and
     *  all first vehicles. They are not moved, in fact, but their speed along
     *  the next path is precomputed.
     *
     * This method goes through all active lanes calling their "moveCritical"
     *  implementation. If this call returns true, the lane is removed from the
     *  list of active lanes.
     *
     * @see MSLane::moveCritical
     */
    void moveCritical(SUMOTime t) throw();


    /** @brief Really moves critical vehicles
     *
     * "Critical" are those vehicles that interact with the next junction and
     *  all first vehicles.
     *
     * At first, this method goes through all active lanes calling their
     *  "setCritical" implementation. During this call, "myWithVehicles2Integrate"
     *  is filled with lanes that obtain new vehicles.
     *
     * Then, myWithVehicles2Integrate is gone through, calling "integrateNewVehicle"
     *  of each of the stored instances. 
     *
     * @see MSLane::setCritical
     * @see MSLane::integrateNewVehicle
     * @todo When moving to parallel processing, the usage of myWithVehicles2Integrate would get insecure!!
     */
    void moveFirst(SUMOTime t) throw();
    /// @}


    /** @brief Detect collisions
     *
     * Calls "detectCollisions" of each lane.
     * Shouldn't be necessary if model-implementation is correct.
     * The parameter is simply passed to the lane-instance for reporting.
     *
     * @param[in] timestep The current time step
     */
    void detectCollisions(SUMOTime timestep) throw();


    /** @brief Returns loaded edges
     *
     * @return the container storing one-lane edges
     * @todo Check: Is this secure?
     */
    const std::vector<MSEdge*> &getEdges() const throw() {
        return myEdges;
    }


    /** @brief Returns the list of names of all known edges
     *
     * @return a vector of names of all known edges
     */
    std::vector<std::string> getEdgeNames() const throw();


private:
    /// @brief Loaded edges
    std::vector<MSEdge*> myEdges;

    /// @brief A storage for lanes which shall be integrated because vehicles have moved onto them
    std::vector<MSLane*> myWithVehicles2Integrate;

private:
    /// @brief Copy constructor.
    MSEdgeControl(const MSEdgeControl&);

    /// @brief Assignment operator.
    MSEdgeControl& operator=(const MSEdgeControl&);

};


#endif

/****************************************************************************/

