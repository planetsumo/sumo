/****************************************************************************/
/// @file    PedestrianRouter.h
/// @author  Jakob Erdmann
/// @date    Mon, 03 March 2014
/// @version $Id$
///
// The Pedestrian Router build a special network and (delegegates to a SUMOAbstractRouter)
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
#ifndef PedestrianRouter_h
#define PedestrianRouter_h


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
#include <algorithm>
#include <assert.h>
#include <utils/common/SysUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/common/Named.h>
#include <utils/common/SUMOAbstractRouter.h>
#include <utils/common/DijkstraRouterTT.h>
#include <utils/common/AStarRouter.h>

//#define PedestrianRouter_DEBUG_NETWORK

template <class E, class L>
inline const L* getSidewalk(const E* edge) {
    if (edge == 0) {
        return 0;
    }
    const std::vector<L*>& lanes = edge->getLanes();
    for (typename std::vector<L*>::const_iterator it = lanes.begin(); it != lanes.end(); ++it) {
        if ((*it)->allowsVehicleClass(SVC_PEDESTRIAN)) {
            return *it;
        }
    }
    return lanes.front();
}

// ===========================================================================
// class definitions
// ===========================================================================

/// @brief the "vehicle" type that is given to the internal router (SUMOAbstractRouter)
template<class E>
struct PedestrianTrip {

    PedestrianTrip(const E* _from, const E* _to, SUMOReal _departPos, SUMOReal _arrivalPos, SUMOReal _speed) :
        from(_from),
        to(_to),
        departPos(_departPos < 0 ? _from->getLength() + _departPos : _departPos),
        arrivalPos(_arrivalPos < 0 ? _to->getLength() + _arrivalPos : _arrivalPos),
        speed(_speed)
    {}

    const E* from;
    const E* to;
    const SUMOReal departPos;
    const SUMOReal arrivalPos;
    const SUMOReal speed;
};


/// @brief the edge type that is given to the internal router (SUMOAbstractRouter)
template<class E, class L>
class PedestrianEdge : public Named {
    typedef std::pair<PedestrianEdge*, PedestrianEdge*> EdgePair;
    /* brief build the pedestrian network (once)
     * @param noE The number of edges in the dictionary of E
     */

public:
    static size_t dictSize() {
        return myEdgeDict.size();
    }

    static void initPedestrianNetwork(size_t noE) {
        if (myAmInitialized) {
            return;
        }
#ifdef PedestrianRouter_DEBUG_NETWORK
            std::cout << "initPedestrianNetwork\n";
#endif
        // build the Pedestrian edges
        unsigned int numericalID = 0;
        for (size_t i = 0; i < noE; i++) {
            E* edge = E::dictionary(i);
            if (edge->isInternal()) {
                continue;
            }
            // forward and backward edges
            myEdgeDict.push_back(PedestrianEdge(numericalID++, edge, true));
            myEdgeDict.push_back(PedestrianEdge(numericalID++, edge, false));
            // depart and arrival edges for (the router can decide the initial direction to take and the direction to arrive from)
            myEdgeDict.push_back(PedestrianEdge(numericalID++, edge, true, true));
            myEdgeDict.push_back(PedestrianEdge(numericalID++, edge, false, true));

        }
        // build the lookup tables after myEdgeDict is complete
        numericalID = 0;
        for (size_t i = 0; i < noE; i++) {
            E* edge = E::dictionary(i);
            if (edge->isInternal()) {
                continue;
            }
            myBidiLookup[edge] = std::make_pair(&myEdgeDict[numericalID], &myEdgeDict[numericalID + 1]);
            myFromToLookup[edge] = std::make_pair(&myEdgeDict[numericalID + 2], &myEdgeDict[numericalID + 3]);
            numericalID += 4;
        }

        // build the connections
        for (size_t i = 0; i < noE; i++) {
            E* edge = E::dictionary(i);
            if (edge->isInternal()) {
                continue;
            }
            // find all incoming and outgoing lanes for the sidewalk and
            // connect the corresponding PedestrianEdges
            const L* sidewalk = getSidewalk<E, L>(edge);
            const EdgePair& pair = getBothDirections(edge);

#ifdef PedestrianRouter_DEBUG_NETWORK
            std::cout << "  building connections from " << sidewalk->getID() << "\n";
#endif
            std::vector<const L*> outgoing = sidewalk->getOutgoingLanes();
            for (typename std::vector<const L*>::iterator it = outgoing.begin(); it != outgoing.end(); ++it) {
                const L* target = *it;
                const E* targetEdge = &(target->getEdge());
#ifdef PedestrianRouter_DEBUG_NETWORK
                std::cout << "   lane=" << getSidewalk<E, L>(targetEdge)->getID() << (target == getSidewalk<E, L>(targetEdge) ? "(used)" : "") << "\n";
#endif
                if (target == getSidewalk<E, L>(targetEdge)) {
                    const EdgePair& targetPair = getBothDirections(targetEdge);
                    pair.first->myFollowingEdges.push_back(targetPair.first);
                    targetPair.second->myFollowingEdges.push_back(pair.second);
#ifdef PedestrianRouter_DEBUG_NETWORK
                std::cout << "     " << pair.first->getID() << " -> " << targetPair.first->getID() << "\n";
                std::cout << "     " << targetPair.second->getID() << " -> " << pair.second->getID() << "\n";
#endif
                }
            }
            if (edge->isCrossing() || edge->isWalkingArea()) {
                continue;
            }
            // build connections from depart connector 
            PedestrianEdge* startConnector = getDepartEdge(edge);
            startConnector->myFollowingEdges.push_back(pair.first);
            startConnector->myFollowingEdges.push_back(pair.second);
            // build connections to arrival connector
            PedestrianEdge* endConnector = getArrivalEdge(edge);
            pair.first->myFollowingEdges.push_back(endConnector);
            pair.second->myFollowingEdges.push_back(endConnector);
#ifdef PedestrianRouter_DEBUG_NETWORK
            std::cout << "     " << startConnector->getID() << " -> " << pair.first->getID() << "\n";
            std::cout << "     " << startConnector->getID() << " -> " << pair.second->getID() << "\n";
            std::cout << "     " << pair.first->getID() << " -> " << endConnector->getID() << "\n";
            std::cout << "     " << pair.second->getID() << " -> " << endConnector->getID() << "\n";
#endif
        }
        myAmInitialized = true;
    }

    bool includeInRoute() const {
        return !myAmConnector && !myEdge->isCrossing() && !myEdge->isWalkingArea();
    }

    const E* getEdge() const {
        return myEdge;
    }

    /// @brief Returns the pair of forward and backward edge
    static const EdgePair& getBothDirections(const E* e) {
        typename std::map<const E*, EdgePair>::const_iterator it = myBidiLookup.find(e);
        if (it == myBidiLookup.end()) {
            assert(false);
            throw ProcessError("Edge '" + e->getID() + "' not found in pedestrian network '");
        }
        return (*it).second;
    }

    /// @brief Returns the departing Pedestrian edge
    static PedestrianEdge* getDepartEdge(const E* e) {
        typename std::map<const E*, EdgePair>::const_iterator it = myFromToLookup.find(e);
        if (it == myFromToLookup.end()) {
            assert(false);
            throw ProcessError("Edge '" + e->getID() + "' not found in pedestrian network '");
        }
        return (*it).second.first;
    }

    /// @brief Returns the arriving Pedestrian edge
    static PedestrianEdge* getArrivalEdge (const E* e) {
        typename std::map<const E*, EdgePair>::const_iterator it = myFromToLookup.find(e);
        if (it == myFromToLookup.end()) {
            assert(false);
            throw ProcessError("Edge '" + e->getID() + "' not found in pedestrian network '");
        }
        return (*it).second.second;
    }

    /// @name The interface as required by SUMOAbstractRouter routes
    /// @{

    unsigned int getNumericalID() const {
        return myNumericalID;
    }

    /// @brief Returns the PedstrianEdge with the given numericalID
    static const PedestrianEdge* dictionary(size_t index) {
        assert(index < myEdgeDict.size());
        return &myEdgeDict[index];
    }

    unsigned int getNoFollowing() const {
        return (unsigned int)myFollowingEdges.size();
    }

    PedestrianEdge* getFollower(unsigned int i) const {
        return myFollowingEdges[i];
    }

    bool prohibits(const PedestrianTrip<E>* const) const {
        // network only includes PedestrianEdges
        return false;
    }

    /// @}

    /*@brief the function called by RouterTT_direct
     * (distance is used as effort, effort is assumed to be independent of time
     */
    SUMOReal getEffort(const PedestrianTrip<E>* const trip, SUMOReal) const {
        UNUSED_PARAMETER(time);
        if (myAmConnector) {
            return 0;
        }
        // XXX length of walkingAreas != edgeLength
        SUMOReal length = myEdge->getLength();
        if (myEdge == trip->from) {
            if (myForward) {
                length -= trip->departPos;
            } else {
                length = trip->departPos;
            }
        }
        if (myEdge == trip->to) {
            if (myForward) {
                length = trip->arrivalPos;
            } else {
                length -= trip->arrivalPos;
            }
        }
        return length / trip->speed;
    }

private:
    PedestrianEdge(unsigned int numericalID, E* edge, bool forward, bool connector = false) :
        Named(edge->getID() + "_" + (forward ? "fwd" : "bwd") + (connector ? "_connector" : "")), 
        myNumericalID(numericalID),
        myEdge(edge),
        myForward(forward),
        myAmConnector(connector) { }

    /// @brief the index in myEdgeDict
    unsigned int myNumericalID;

    /// @brief  the original edge
    const E* myEdge;

    /// @brief the direction of this edge
    bool myForward;

    /// @brief the direction of this edge
    bool myAmConnector;

    /// @brief List of edges that may be approached from this edge
    std::vector<PedestrianEdge*> myFollowingEdges;

    /// @brief the edge dictionary
    static std::vector<PedestrianEdge> myEdgeDict;

    /// @brief retrieve the forward and backward edge for the given input edge E
    static std::map<const E*, EdgePair> myBidiLookup;

    /// @brief retrieve the depart and arrival edge for the given input edge E
    static std::map<const E*, EdgePair> myFromToLookup;

    static bool myAmInitialized;

};


/**
 * @class PedestrianRouter
 * The router for pedestrians (on a bidirectional network of sidewalks and crossings
 */
template<class E, class L, class INTERNALROUTER>
class PedestrianRouter : public SUMOAbstractRouter<E, PedestrianTrip<E> > {
public:

    typedef PedestrianEdge<E, L> _PedestrianEdge;
    typedef PedestrianTrip<E> _PedestrianTrip;

    /// Constructor
    PedestrianRouter():
        SUMOAbstractRouter<E, _PedestrianTrip>("PedestrianRouter")
    { 
        _PedestrianEdge::initPedestrianNetwork(E::dictSize());
        myInternalRouter = new INTERNALROUTER(_PedestrianEdge::dictSize(), true, &_PedestrianEdge::getEffort);
    }

    /// Destructor
    virtual ~PedestrianRouter() { 
        delete myInternalRouter;
    }

    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    void compute(const E* from, const E* to, SUMOReal departPos, SUMOReal arrivalPos, SUMOReal speed,
                         SUMOTime msTime, std::vector<const E*>& into) {
        //startQuery();
        _PedestrianTrip trip(from, to, departPos, arrivalPos, speed);
        std::vector<const _PedestrianEdge*> intoPed;
        myInternalRouter->compute(_PedestrianEdge::getDepartEdge(from), 
                _PedestrianEdge::getArrivalEdge(to), &trip, msTime, intoPed);
        if (intoPed.size() > 2) {
            for (size_t i = 1; i < intoPed.size(); ++i) {
                if (intoPed[i]->includeInRoute()) {
                    into.push_back(intoPed[i]->getEdge());
                }
            }
        }
        //endQuery();
    }

    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    void compute(const E* from, const E* to, const _PedestrianTrip* const vehicle,
                         SUMOTime msTime, std::vector<const E*>& into) {
        throw ProcessError("Do not use this method");
    }

    virtual SUMOReal recomputeCosts(const std::vector<const E*>& edges,
                                    const _PedestrianTrip* const v, SUMOTime msTime) const {
        throw ProcessError("Do not use this method");
    }

private:
    INTERNALROUTER* myInternalRouter;


private:
    /// @brief Invalidated assignment operator
    PedestrianRouter& operator=(const PedestrianRouter& s);

private:

};

// common specializations
template<class E, class L>
class PedestrianRouterDijkstra : public PedestrianRouter<E, L, 
    DijkstraRouterTT_Direct<PedestrianEdge<E, L>, PedestrianTrip<E>, prohibited_withRestrictions<PedestrianEdge<E, L>, PedestrianTrip<E> > > > { };


#endif


// ===========================================================================
// static member definitions (PedestrianEdge)
// ===========================================================================

template<class E, class L>
std::vector<PedestrianEdge<E, L> > PedestrianEdge<E, L>::myEdgeDict;

template<class E, class L>
std::map<const E*, typename PedestrianEdge<E, L>::EdgePair> PedestrianEdge<E, L>::myBidiLookup;

template<class E, class L>
std::map<const E*, typename PedestrianEdge<E, L>::EdgePair> PedestrianEdge<E, L>::myFromToLookup;

template<class E, class L>
bool PedestrianEdge<E, L>::myAmInitialized(false);

/****************************************************************************/

