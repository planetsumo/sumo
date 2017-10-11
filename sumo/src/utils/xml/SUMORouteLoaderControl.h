/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    SUMORouteLoaderControl.h
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Wed, 06 Nov 2002
/// @version $Id$
///
// Class responsible for loading of routes from some files
/****************************************************************************/
#ifndef SUMORouteLoaderControl_h
#define SUMORouteLoaderControl_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMORouteLoader;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SUMORouteLoaderControl
 *
 * SUMORouteLoaderControl
 * This controls is initialised with the list of route loaders and uses them
 * to load routes step wise.
 * The parameter myInAdvanceStepNo holds the number of time steps to read the
 * routes in forward. If it is 0 (default), all routes will be read at once.
 */
class SUMORouteLoaderControl {
public:
    /// constructor
    SUMORouteLoaderControl(SUMOTime inAdvanceStepNo);

    /// destructor
    ~SUMORouteLoaderControl();

    /// add another loader
    void add(SUMORouteLoader* loader);

    /// loads the next routes up to and including the given time step
    void loadNext(SUMOTime step);

    /// returns the timestamp of the first loaded vehicle or flow
    SUMOTime getFirstLoadTime() const {
        return myFirstLoadTime;
    }

    /// returns whether loading is completed
    bool haveAllLoaded() const {
        return myAllLoaded;
    }

private:
    /// the first time step for which vehicles were loaded
    SUMOTime myFirstLoadTime;

    /// the time step up to which vehicles were loaded
    SUMOTime myCurrentLoadTime;

    /// the number of routes to read in forward
    const SUMOTime myInAdvanceStepNo;

    /// the list of route loaders
    std::vector<SUMORouteLoader*> myRouteLoaders;

    /** information whether all routes shall be loaded and whether
        they were loaded */
    bool myLoadAll, myAllLoaded;

private:
    /// @brief Invalidated copy constructor
    SUMORouteLoaderControl(const SUMORouteLoaderControl& src);

    /// @brief Invalidated assignment operator
    SUMORouteLoaderControl& operator=(const SUMORouteLoaderControl& src);
};


#endif

/****************************************************************************/

