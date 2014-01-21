/****************************************************************************/
/// @file    MSPModel.h
/// @author  Jakob Erdmann
/// @date    Mon, 13 Jan 2014
/// @version $Id: MSPModel.h 14658 2013-09-10 15:31:59Z namdre $
///
// The pedestrian following model (prototype)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2001-2013 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSPModel_h
#define	MSPModel_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/common/SUMOTime.h>
#include <utils/common/Command.h>
#include <microsim/MSPerson.h>

// ===========================================================================
// class declarations
// ===========================================================================
class MSNet;
class MSLane;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSPModel
 * @brief The pedestrian following model
 *
 */
class MSPModel {
public:

    static void add(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, MSNet* net=0);

    static bool blockedAtDist(const MSLane* lane, SUMOReal distToCrossing);

    static void cleanup();

    static const SUMOReal SAFETY_GAP;
    static const SUMOReal STRIPE_WIDTH;
    static const SUMOReal LOOKAHEAD;
    static const SUMOReal SQUEEZE;

protected:
    class Pedestrian;
    typedef std::vector<Pedestrian> Pedestrians;
    typedef std::map<const MSLane*, Pedestrians> ActiveLanes;

    /**
     * @class Pedestrian
     * @brief Container for pedestrian state and individual position update function
     */
    class Pedestrian {
    public:

        Pedestrian(MSPerson* person, MSPerson::MSPersonStage_Walking* stage);
        ~Pedestrian() {};
        MSPerson* myPerson;
        MSPerson::MSPersonStage_Walking* myStage;
        /// @brief the advancement along the current lane
        SUMOReal myX; 
        /// @brief the orthogonal shift on the current lane
        SUMOReal myY; 

        bool myWaitingToEnter;

        SUMOReal getLength() const;

        /// @brief perform position update
        void walk(std::vector<SUMOReal> vSafe, Pedestrians::iterator maxLeader, Pedestrians::iterator minFollower);

        /// @brief compute safe speeds on all stripes and update vSafe
        static void updateVSafe(
                std::vector<SUMOReal>& vSafe,
                Pedestrians::iterator maxLeader, 
                Pedestrians::iterator minFollower, 
                SUMOReal x, const Pedestrian& ego);
            
    private:
        int stripe() const;
        int otherStripe() const;

    };

    class MovePedestrians : public Command {
        public:
            MovePedestrians();
            ~MovePedestrians() {};
            SUMOTime execute(SUMOTime currentTime);
        private:
            static const MSLane* getNextLane(const MSLane* currentLane, Pedestrian& ped);

            /// @brief Invalidated assignment operator.
            MovePedestrians& operator=(const MovePedestrians&);
    };
    friend class MovePedestrians;

    class by_xpos_sorter {
    public:
        /// constructor
        by_xpos_sorter() {}

    public:
        /// comparing operation
        int operator()(const Pedestrian& p1, const Pedestrian& p2) const {
            return p1.myX > p2.myX;
        }
    };

    /// @brief return the appropriate lane to walk on
    static MSLane* getSidwalk(const MSEdge* edge);

    /// @brief return the maximum number of pedestrians walking side by side
    static int numStripes(const MSLane* lane);

    static void moveToLane(Pedestrian ped, const MSLane* newLane);

    static Pedestrians& getPedestrians(const MSLane* lane);

    static int countWaitingToEnter(const std::vector<Pedestrian>& pedestrians);

protected:
    static int myNumActivePedestrians;

    static ActiveLanes myActiveLanes;
    static Pedestrians noPedestrians;

    static const SUMOReal DEFAULT_SIDEWALK_WIDTH;
};


#endif	/* MSPModel_h */

