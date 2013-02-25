/****************************************************************************/
/// @file    MSSimpleTrafficLightLogic.h
/// @author  Daniel Krajzewicz
/// @author  Julia Ringel
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Friedemann Wesner
/// @date    Sept 2002
/// @version $Id$
///
// A fixed traffic light logic
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSSimpleTrafficLightLogic_h
#define MSSimpleTrafficLightLogic_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utility>
#include <vector>
#include <bitset>
#include <microsim/MSEventControl.h>
#include <microsim/MSNet.h>
//#include "MSTrafficLightLogic.h"
#include "MSPhasedTrafficLightLogic.h"
#include "MSPhaseDefinition.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSSimpleTrafficLightLogic
 * @brief A fixed traffic light logic
 *
 * The implementation of a simple traffic light which only switches between
 * it's phases and sets the lights to red in between.
 * Some functions are called with an information about the current step. This
 * is needed as a single logic may be used by many junctions and so the current
 * step is stored within them, not within the logic.
 */
class MSSimpleTrafficLightLogic : public MSPhasedTrafficLightLogic {
public:
    /** @brief Constructor
     * @param[in] tlcontrol The tls control responsible for this tls
     * @param[in] id This tls' id
     * @param[in] subid This tls' sub-id (program id)
     * @param[in] phases Definitions of the phases
     * @param[in] step The initial phase index
     * @param[in] delay The time to wait before the first switch
     */
    MSSimpleTrafficLightLogic(MSTLLogicControl& tlcontrol,
                              const std::string& id, const std::string& subid,
                              const Phases& phases, unsigned int step, SUMOTime delay,
                              const ParameterMap& parameters=ParameterMap());


    /// @brief Destructor
    ~MSSimpleTrafficLightLogic();

    /// @name Switching and setting current rows
    /// @{

    /** @brief Switches to the next phase
     * @param[in] isActive Whether this program is the currently used one
     * @return The time of the next switch
     * @see MSTrafficLightLogic::trySwitch
     */
    SUMOTime trySwitch(bool isActive);
    /// @}

};


#endif

/****************************************************************************/

