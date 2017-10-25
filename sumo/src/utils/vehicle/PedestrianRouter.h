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
/// @file    PedestrianRouter.h
/// @author  Jakob Erdmann
/// @date    Mon, 03 March 2014
/// @version $Id$
///
// The Pedestrian Router builds a special network and delegates to a SUMOAbstractRouter.
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
#include <utils/common/MsgHandler.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include "SUMOAbstractRouter.h"
#include "DijkstraRouterTT.h"
#include "IntermodalNetwork.h"

//#define PedestrianRouter_DEBUG_ROUTES


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class PedestrianRouter
 * The router for pedestrians (on a bidirectional network of sidewalks and crossings)
 */
template<class E, class L, class N, class V, class INTERNALROUTER>
class PedestrianRouter : public SUMOAbstractRouter<E, IntermodalTrip<E, N, V> > {
public:

    typedef IntermodalEdge<E, L, N, V> _IntermodalEdge;
    typedef IntermodalNetwork<E, L, N, V> _IntermodalNetwork;
    typedef IntermodalTrip<E, N, V> _IntermodalTrip;

    /// Constructor
    PedestrianRouter():
        SUMOAbstractRouter<E, _IntermodalTrip>(0, "PedestrianRouter"), myAmClone(false) {
        myPedNet = new _IntermodalNetwork(E::getAllEdges());
        myInternalRouter = new INTERNALROUTER(myPedNet->getAllEdges(), true, &_IntermodalEdge::getTravelTimeStatic);
    }

    PedestrianRouter(_IntermodalNetwork* net):
        SUMOAbstractRouter<E, _IntermodalTrip>(0, "PedestrianRouter"), myAmClone(true) {
        myPedNet = net;
        myInternalRouter = new INTERNALROUTER(myPedNet->getAllEdges(), true, &_IntermodalEdge::getTravelTimeStatic);
    }

    /// Destructor
    virtual ~PedestrianRouter() {
        delete myInternalRouter;
        if (!myAmClone) {
            delete myPedNet;
        }
    }

    virtual SUMOAbstractRouter<E, _IntermodalTrip>* clone() {
        return new PedestrianRouter<E, L, N, V, INTERNALROUTER>(myPedNet);
    }

    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    double compute(const E* from, const E* to, double departPos, double arrivalPos, double speed,
                   SUMOTime msTime, const N* onlyNode, std::vector<const E*>& into, bool allEdges = false) {
        if (getSidewalk<E, L>(from) == 0) {
            WRITE_WARNING("Departure edge '" + from->getID() + "' does not allow pedestrians.");
            return false;
        }
        if (getSidewalk<E, L>(to) == 0) {
            WRITE_WARNING("Destination edge '" + to->getID() + "' does not allow pedestrians.");
            return false;
        }
        _IntermodalTrip trip(from, to, departPos, arrivalPos, speed, msTime, onlyNode);
        std::vector<const _IntermodalEdge*> intoPed;
        const bool success = myInternalRouter->compute(myPedNet->getDepartEdge(from),
                             myPedNet->getArrivalEdge(to),
                             &trip, msTime, intoPed);
        double time = 0.;
        if (success) {
            for (const _IntermodalEdge* pedEdge : intoPed) {
                if (pedEdge->includeInRoute(allEdges)) {
                    into.push_back(pedEdge->getEdge());
                }
                time += myInternalRouter->getEffort(pedEdge, &trip, time);
            }
        }
#ifdef PedestrianRouter_DEBUG_ROUTES
        std::cout << TIME2STEPS(msTime) << " trip from " << from->getID() << " to " << to->getID()
                  << " departPos=" << departPos
                  << " arrivalPos=" << arrivalPos
                  << " onlyNode=" << (onlyNode == 0 ? "NULL" : onlyNode->getID())
                  << " edges=" << toString(intoPed)
                  << " resultEdges=" << toString(into)
                  << " time=" << time
                  << "\n";
#endif
        return success ? time : -1.;
    }

    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    bool compute(const E*, const E*, const _IntermodalTrip* const,
                 SUMOTime, std::vector<const E*>&) {
        throw ProcessError("Do not use this method");
    }

    double recomputeCosts(const std::vector<const E*>&, const _IntermodalTrip* const, SUMOTime) const {
        throw ProcessError("Do not use this method");
    }

    void prohibit(const std::vector<E*>& toProhibit) {
        std::vector<_IntermodalEdge*> toProhibitPE;
        for (typename std::vector<E*>::const_iterator it = toProhibit.begin(); it != toProhibit.end(); ++it) {
            toProhibitPE.push_back(myPedNet->getBothDirections(*it).first);
            toProhibitPE.push_back(myPedNet->getBothDirections(*it).second);
        }
        myInternalRouter->prohibit(toProhibitPE);
    }

private:
    const bool myAmClone;
    INTERNALROUTER* myInternalRouter;
    _IntermodalNetwork* myPedNet;


private:
    /// @brief Invalidated assignment operator
    PedestrianRouter& operator=(const PedestrianRouter& s);

};

// common specializations
template<class E, class L, class N, class V>
class PedestrianRouterDijkstra : public PedestrianRouter < E, L, N, V,
    DijkstraRouterTT<IntermodalEdge<E, L, N, V>, IntermodalTrip<E, N, V>, prohibited_withPermissions<IntermodalEdge<E, L, N, V>, IntermodalTrip<E, N, V> > > > { };


#endif

/****************************************************************************/
