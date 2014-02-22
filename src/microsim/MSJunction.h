/****************************************************************************/
/// @file    MSJunction.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Wed, 12 Dez 2001
/// @version $Id$
///
// The base class for an intersection
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2001-2014 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSJunction_h
#define MSJunction_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <map>
#include <utils/geom/Position.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/Named.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/UtilExceptions.h>


class MSVehicle;
class MSLink;
class MSLane;
class MSEdge;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSJunction
 * @brief The base class for an intersection
 */
class MSJunction : public Named {
public:
    /** @brief Constructor
     * @param[in] id The id of the junction
     * @param[in] position The position of the junction
     * @param[in] shape The shape of the junction
     */
    MSJunction(const std::string& id, const Position& position,
               const PositionVector& shape);


    /// @brief Destructor.
    virtual ~MSJunction();


    /** performs some initialisation after the loading
        (e.g., link map computation) */
    virtual void postloadInit();


    /** returns the junction's position */
    const Position& getPosition() const;


    /** @brief Returns this junction's shape
     * @return The shape of this junction
     */
    const PositionVector& getShape() const {
        return myShape;
    }

    virtual const std::vector<MSLink*>& getFoeLinks(const MSLink* const /*srcLink*/) const {
        return myEmptyLinks;
    }

    virtual const std::vector<MSLane*>& getFoeInternalLanes(const MSLink* const /*srcLink*/) const {
        return myEmptyLanes;
    }

    inline const std::vector<const MSEdge*>& getIncoming() const {
        return myIncoming;
    }

    inline const std::vector<const MSEdge*>& getOutgoing() const {
        return myOutgoing;
    }

    void addIncoming(MSEdge* edge) {
        myIncoming.push_back(edge);
    }

    void addOutgoing(MSEdge* edge) {
        myOutgoing.push_back(edge);
    }

    /** @brief Inserts junction into the static dictionary
        Returns true if the key id isn't already in the dictionary. Otherwise
        returns false. */
    static bool dictionary(const std::string& id, MSJunction* junction);

    /** @brief Returns the MSEdge associated to the key id if exists, otherwise returns 0. */
    static MSJunction* dictionary(const std::string& id);

    static void clear() {
        myDict.clear();
    }

protected:
    /// @brief The position of the junction
    Position myPosition;

    /// @brief The shape of the junction
    PositionVector myShape;

    std::vector<MSLink*> myEmptyLinks;
    std::vector<MSLane*> myEmptyLanes;


    /// @brief incoming edges
    std::vector<const MSEdge*> myIncoming;
    /// @brief outgoing edges
    std::vector<const MSEdge*> myOutgoing;

    /// @brief definition of the static dictionary type
    typedef std::map<std::string, MSJunction* > DictType;

    /** @brief Static dictionary to associate string-ids with objects.
     * @deprecated Move to MSEdgeControl, make non-static
     */
    static DictType myDict;

private:
    /// @brief Invalidated copy constructor.
    MSJunction(const MSJunction&);

    /// @brief Invalidated assignment operator.
    MSJunction& operator=(const MSJunction&);

};


#endif

/****************************************************************************/

