/****************************************************************************/
/// @file    MSPModel.h
/// @author  Jakob Erdmann
/// @date    Mon, 13 Jan 2014
/// @version $Id$
///
// The pedestrian following model (prototype)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2014-2014 DLR (http://www.dlr.de/) and contributors
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
class MSJunction;


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

    /// @brief register the given person as a pedestrian
    static void add(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, MSNet* net=0);

    /// @brief whether a pedestrian is blocking the crossing of lane at offset distToCrossing
    static bool blockedAtDist(const MSLane* lane, SUMOReal distToCrossing);

    /// @brief remove state at simulation end
    static void cleanup();

    /// @brief return the maximum number of pedestrians walking side by side
    static int numStripes(const MSLane* lane);


    // @brief walking directions
    static const int FORWARD;
    static const int BACKWARD;
    static const int UNDEFINED_DIRECTION;

    // @brief the safety gap to keep between the car and the pedestrian in all directions
    static const SUMOReal SAFETY_GAP;

    // @brief the width of a pedstrian stripe
    static const SUMOReal STRIPE_WIDTH;

    // @brief the distance to look ahead for changing stripes
    static const SUMOReal LOOKAHEAD;

    // @brief the speed penalty for moving sideways
    static const SUMOReal LATERAL_PENALTY;

    // @brief the factor by which pedestrian width is reduced when sqeezing past each other
    static const SUMOReal SQUEEZE;

    // @brief the maximum distance at which oncoming pedestrians block right turning traffic
    static const SUMOReal BLOCKER_LOOKAHEAD;

    // @brief the penalty for staying on a stripe with oncoming pedestrians
    static const SUMOReal ONCOMIN_PENALTY;
    static const SUMOReal ONCOMIN_PENALTY_FACTOR;

    // @brief fraction of the leftmost lanes to reserve for oncoming traffic
    static const SUMOReal RESERVE_FOR_ONCOMING_FACTOR;

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

        Pedestrian(MSPerson* person, MSPerson::MSPersonStage_Walking* stage, const MSLane* lane);
        ~Pedestrian() {};
        MSPerson* myPerson;
        MSPerson::MSPersonStage_Walking* myStage;
        /// @brief the advancement along the current lane
        SUMOReal myX; 
        /// @brief the orthogonal shift on the current lane
        SUMOReal myY; 
        /// @brief the walking direction on the current lane (1 forward, -1 backward)
        int myDir;

        bool myWaitingToEnter;

        /// @brief return the length of the pedestrian
        SUMOReal getLength() const;

        /// @brief the absolute distance to the end of the lane in walking direction
        SUMOReal distToLaneEnd() const;

        /// @brief return whether this pedestrian has passed the end of the current lane and update myX if so
        bool moveToNextLane();

        /// @brief perform position update
        void walk(std::vector<SUMOReal> vSafe, Pedestrians::iterator maxLeader, Pedestrians::iterator minFollower);

        /// @brief update location data for MSPersonStage_Walking 
        void updateLocation(const MSLane* lane, const PositionVector& walkingAreaShape=PositionVector());

        /// @brief compute safe speeds on all stripes and update vSafe
        static void updateVSafe(
                std::vector<SUMOReal>& vSafe,
                Pedestrians::iterator maxLeader, 
                Pedestrians::iterator minFollower, 
                SUMOReal x, 
                const Pedestrian& ego,
                int dir);
            
    private:
        int stripe(int max) const;
        int otherStripe(int max) const;

    };

    class MovePedestrians : public Command {
        public:
            MovePedestrians();
            ~MovePedestrians() {};
            SUMOTime execute(SUMOTime currentTime);
        private:
            /// @brief Invalidated assignment operator.
            MovePedestrians& operator=(const MovePedestrians&);
    };

    class by_xpos_sorter {
    public:
        /// constructor
        by_xpos_sorter(int dir): myDir(dir) {}

    public:
        /// comparing operation
        bool operator()(const Pedestrian& p1, const Pedestrian& p2) const {
            return myDir * p1.myX > myDir * p2.myX;
        }

    private:
        const int myDir;
    };

    /// @brief move all pedestrians forward and advance to the next lane if applicable
    static void moveInDirection(SUMOTime currentTime, int dir);

    /// @brief return the appropriate lane to walk on
    static MSLane* getSidwalk(const MSEdge* edge);

    /// @brief adds the given pedestrian to the new lane unless the lane is 0
    static void addToLane(Pedestrian& ped, int oldStripes, const MSLane* newLane, int newDir, const PositionVector& walkingAreaShape);

    /// @brief retrieves the pedestian vector for the given lane (may be empty)
    static Pedestrians& getPedestrians(const MSLane* lane);

    /// @brief counts the number of pedestrians with status waitingToEnter 
    static int countWaitingToEnter(const std::vector<Pedestrian>& pedestrians);

    /** @brief computes the successor lane for the given pedestrian and sets the 
     * link as well as the direction to use on the succesor lane
     * @param[in] currentLane The lane the pedestrian is currently on
     * @param[in] ped The pedestrian for which to compute the next lane
     * @param[out] link The link between currentLane and next lane
     * @param[out] dir The direction to use on the next lane
     * @return The next lane or 0 if the route ends
     */
    static const MSLane* getNextLane(const MSLane* currentLane, const Pedestrian& ped, 
            MSLink*& link, int& nextDir);

    static PositionVector getWalkingAreaShape(const MSLane* from, const MSLane* walkingArea, const Pedestrian& ped); 

    /// @brief returns the direction in which these lanes are connectioned or 0 if they are not
    static int connectedDirection(const MSLane* from, const MSLane* to);

protected:
    static int myNumActivePedestrians;

    static ActiveLanes myActiveLanes;
    static Pedestrians noPedestrians;

    static const SUMOReal DEFAULT_SIDEWALK_WIDTH;
};


#endif	/* MSPModel_h */

