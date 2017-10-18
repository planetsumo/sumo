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
/// @file    MSAbstractLaneChangeModel.h
/// @author  Daniel Krajzewicz
/// @author  Friedemann Wesner
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Fri, 29.04.2005
/// @version $Id$
///
// Interface for lane-change models
/****************************************************************************/
#ifndef MSAbstractLaneChangeModel_h
#define MSAbstractLaneChangeModel_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <microsim/MSVehicle.h>

class MSLane;

// ===========================================================================
// used enumeration
// ===========================================================================

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSAbstractLaneChangeModel
 * @brief Interface for lane-change models
 */
class MSAbstractLaneChangeModel {
public:

    /** @class MSLCMessager
     * @brief A class responsible for exchanging messages between cars involved in lane-change interaction
     */
    class MSLCMessager {
    public:
        /** @brief Constructor
         * @param[in] leader The leader on the informed vehicle's lane
         * @param[in] neighLead The leader on the lane the vehicle want to change to
         * @param[in] neighFollow The follower on the lane the vehicle want to change to
         */
        MSLCMessager(MSVehicle* leader,  MSVehicle* neighLead, MSVehicle* neighFollow)
            : myLeader(leader), myNeighLeader(neighLead),
              myNeighFollower(neighFollow) { }


        /// @brief Destructor
        ~MSLCMessager() { }


        /** @brief Informs the leader on the same lane
         * @param[in] info The information to pass
         * @param[in] sender The sending vehicle (the lane changing vehicle)
         * @return Something!?
         */
        void* informLeader(void* info, MSVehicle* sender) {
            assert(myLeader != 0);
            return myLeader->getLaneChangeModel().inform(info, sender);
        }


        /** @brief Informs the leader on the desired lane
         * @param[in] info The information to pass
         * @param[in] sender The sending vehicle (the lane changing vehicle)
         * @return Something!?
         */
        void* informNeighLeader(void* info, MSVehicle* sender) {
            assert(myNeighLeader != 0);
            return myNeighLeader->getLaneChangeModel().inform(info, sender);
        }


        /** @brief Informs the follower on the desired lane
         * @param[in] info The information to pass
         * @param[in] sender The sending vehicle (the lane changing vehicle)
         * @return Something!?
         */
        void* informNeighFollower(void* info, MSVehicle* sender) {
            assert(myNeighFollower != 0);
            return myNeighFollower->getLaneChangeModel().inform(info, sender);
        }


    private:
        /// @brief The leader on the informed vehicle's lane
        MSVehicle* myLeader;
        /// @brief The leader on the lane the vehicle want to change to
        MSVehicle* myNeighLeader;
        /// @brief The follower on the lane the vehicle want to change to
        MSVehicle* myNeighFollower;

    };

    struct StateAndDist {
        // @brief LaneChangeAction flags
        int state;
        // @brief lateralDistance
        double latDist;
        // @brief direction that was checked
        int dir;

        StateAndDist(int _state, double _latDist, int _dir) :
            state(_state),
            latDist(_latDist),
            dir(_dir) {}

        bool sameDirection(const StateAndDist& other) const {
            return latDist * other.latDist > 0;
        }
    };

    /// @brief init global model parameters
    void static initGlobalOptions(const OptionsCont& oc);

    /** @brief Factory method for instantiating new lane changing models
     * @param[in] lcm The type of model to build
     * @param[in] vehicle The vehicle for which this model shall be built
     */
    static MSAbstractLaneChangeModel* build(LaneChangeModel lcm, MSVehicle& vehicle);

    /// @brief whether lanechange-output is active
    static bool haveLCOutput() {
        return myLCOutput;
    }

    /** @brief Constructor
     * @param[in] v The vehicle this lane-changer belongs to
     * @param[in] model The type of lane change model
     */
    MSAbstractLaneChangeModel(MSVehicle& v, const LaneChangeModel model);

    /// @brief Destructor
    virtual ~MSAbstractLaneChangeModel();

    inline int getOwnState() const {
        return myOwnState;
    }

    virtual void setOwnState(const int state);

    const std::pair<int, int>& getSavedState(const int dir) const {
        return mySavedStates.find(dir)->second;
    }

    void saveState(const int dir, const int stateWithoutTraCI, const int state) {
        mySavedStates[dir] = std::make_pair(stateWithoutTraCI, state);
    }

    void setFollowerGaps(CLeaderDist follower, double secGap);
    void setLeaderGaps(CLeaderDist, double secGap);
    void setOrigLeaderGaps(CLeaderDist, double secGap);
    void setFollowerGaps(const MSLeaderDistanceInfo& vehicles);
    void setLeaderGaps(const MSLeaderDistanceInfo& vehicles);
    void setOrigLeaderGaps(const MSLeaderDistanceInfo& vehicles);

    virtual void prepareStep() {
        saveState(-1, LCA_UNKNOWN, LCA_UNKNOWN);
        saveState(0, LCA_UNKNOWN, LCA_UNKNOWN);
        saveState(1, LCA_UNKNOWN, LCA_UNKNOWN);
        myLastLateralGapRight = NO_NEIGHBOR;
        myLastLateralGapLeft = NO_NEIGHBOR;
        myLastLeaderGap = NO_NEIGHBOR;
        myLastLeaderSecureGap = NO_NEIGHBOR;
        myLastFollowerGap = NO_NEIGHBOR;
        myLastFollowerSecureGap = NO_NEIGHBOR;
        myLastOrigLeaderGap = NO_NEIGHBOR;
        myLastOrigLeaderSecureGap = NO_NEIGHBOR;
        myCommittedSpeed = 0;
    }

    /** @brief Called to examine whether the vehicle wants to change
     * using the given laneOffset.
     * This method gets the information about the surrounding vehicles
     * and whether another lane may be more preferable */
    virtual int wantsChange(
        int laneOffset,
        MSAbstractLaneChangeModel::MSLCMessager& msgPass, int blocked,
        const std::pair<MSVehicle*, double>& leader,
        const std::pair<MSVehicle*, double>& neighLead,
        const std::pair<MSVehicle*, double>& neighFollow,
        const MSLane& neighLane,
        const std::vector<MSVehicle::LaneQ>& preb,
        MSVehicle** lastBlocked,
        MSVehicle** firstBlocked) {
        UNUSED_PARAMETER(laneOffset);
        UNUSED_PARAMETER(&msgPass);
        UNUSED_PARAMETER(blocked);
        UNUSED_PARAMETER(&leader);
        UNUSED_PARAMETER(&neighLead);
        UNUSED_PARAMETER(&neighFollow);
        UNUSED_PARAMETER(&neighLane);
        UNUSED_PARAMETER(&preb);
        UNUSED_PARAMETER(lastBlocked);
        UNUSED_PARAMETER(firstBlocked);
        throw ProcessError("Method not implemented by model " + toString(myModel));
    };

    virtual int wantsChangeSublane(
        int laneOffset,
        LaneChangeAction alternatives,
        const MSLeaderDistanceInfo& leaders,
        const MSLeaderDistanceInfo& followers,
        const MSLeaderDistanceInfo& blockers,
        const MSLeaderDistanceInfo& neighLeaders,
        const MSLeaderDistanceInfo& neighFollowers,
        const MSLeaderDistanceInfo& neighBlockers,
        const MSLane& neighLane,
        const std::vector<MSVehicle::LaneQ>& preb,
        MSVehicle** lastBlocked,
        MSVehicle** firstBlocked,
        double& latDist, int& blocked) {
        UNUSED_PARAMETER(laneOffset);
        UNUSED_PARAMETER(alternatives);
        UNUSED_PARAMETER(&leaders);
        UNUSED_PARAMETER(&followers);
        UNUSED_PARAMETER(&blockers);
        UNUSED_PARAMETER(&neighLeaders);
        UNUSED_PARAMETER(&neighFollowers);
        UNUSED_PARAMETER(&neighBlockers);
        UNUSED_PARAMETER(&neighLane);
        UNUSED_PARAMETER(&preb);
        UNUSED_PARAMETER(lastBlocked);
        UNUSED_PARAMETER(firstBlocked);
        UNUSED_PARAMETER(latDist);
        UNUSED_PARAMETER(blocked);
        throw ProcessError("Method not implemented by model " + toString(myModel));
    }

    /// @brief update expected speeds for each sublane of the current edge
    virtual void updateExpectedSublaneSpeeds(const MSLeaderInfo& ahead, int sublaneOffset, int laneIndex) {
        UNUSED_PARAMETER(&ahead);
        UNUSED_PARAMETER(sublaneOffset);
        UNUSED_PARAMETER(laneIndex);
        throw ProcessError("Method not implemented by model " + toString(myModel));
    }

    /// @brief decide in which direction to move in case both directions are desirable
    virtual StateAndDist decideDirection(StateAndDist sd1, StateAndDist sd2) const {
        UNUSED_PARAMETER(sd1);
        UNUSED_PARAMETER(sd2);
        throw ProcessError("Method not implemented by model " + toString(myModel));
    }

    virtual void* inform(void* info, MSVehicle* sender) = 0;

    /** @brief Called to adapt the speed in order to allow a lane change.
     *
     * It is guaranteed that min<=wanted<=max, but the implementation needs
     * to make sure that the return value is between min and max.
     *
     * @param min The minimum resulting speed
     * @param wanted The aspired speed of the car following model
     * @param max The maximum resulting speed
     * @param cfModel The model used
     * @return the new speed of the vehicle as proposed by the lane changer
     */
    virtual double patchSpeed(const double min, const double wanted, const double max,
                              const MSCFModel& cfModel) = 0;

    /* @brief called once when the primary lane of the vehicle changes (updates
     * the custom variables of each child implementation */
    virtual void changed() = 0;


    /// @brief whether the current vehicles shall be debugged
    virtual bool debugVehicle() const {
        return false;
    }

    /// @brief called when a vehicle changes between lanes in opposite directions
    void changedToOpposite();

    void unchanged() {
        if (myLastLaneChangeOffset > 0) {
            myLastLaneChangeOffset += DELTA_T;
        } else if (myLastLaneChangeOffset < 0) {
            myLastLaneChangeOffset -= DELTA_T;
        }
    }

    /** @brief Returns the lane the vehicles shadow is on during continuous/sublane lane change
     * @return The vehicle's shadow lane
     */
    MSLane* getShadowLane() const {
        return myShadowLane;
    }

    /// @brief return the shadow lane for the given lane
    MSLane* getShadowLane(const MSLane* lane) const;

    /// @brief return the shadow lane for the given lane and lateral offset
    MSLane* getShadowLane(const MSLane* lane, double posLat) const;

    /// @brief set the shadow lane
    void setShadowLane(MSLane* lane) {
        myShadowLane = lane;
    }

    const std::vector<MSLane*>& getShadowFurtherLanes() const {
        return myShadowFurtherLanes;
    }

    const std::vector<double>& getShadowFurtherLanesPosLat() const {
        return myShadowFurtherLanesPosLat;
    }

    inline SUMOTime getLastLaneChangeOffset() const {
        return myLastLaneChangeOffset;
    }


    /// @brief return whether the vehicle passed the midpoint of a continuous lane change maneuver
    inline bool pastMidpoint() const {
        return myLaneChangeCompletion >= 0.5;
    }

    /// @brief return whether the vehicle passed the midpoint of a continuous lane change maneuver
    SUMOTime remainingTime() const;

    /// @brief return true if the vehicle currently performs a lane change maneuver
    inline bool isChangingLanes() const {
        return myLaneChangeCompletion < (1 - NUMERICAL_EPS);
    }

    /// @brief return the direction of the current lane change maneuver
    inline int getLaneChangeDirection() const {
        return myLaneChangeDirection;
    }

    /// @brief return the direction in which the current shadow lane lies
    int getShadowDirection() const;

    /// @brief return the angle offset during a continuous change maneuver
    double getAngleOffset() const;

    /// @brief reset the flag whether a vehicle already moved to false
    inline bool alreadyChanged() const {
        return myAlreadyChanged;
    }

    /// @brief reset the flag whether a vehicle already moved to false
    void resetChanged() {
        myAlreadyChanged = false;
    }

    /// @brief start the lane change maneuver and return whether it continues
    bool startLaneChangeManeuver(MSLane* source, MSLane* target, int direction);

    /* @brief continue the lane change maneuver and return whether the midpoint
     * was passed in this step
     */
    bool updateCompletion();

    /* @brief update lane change shadow after the vehicle moved to a new lane */
    void updateShadowLane();

    /* @brief finish the lane change maneuver
     */
    void endLaneChangeManeuver(const MSMoveReminder::Notification reason = MSMoveReminder::NOTIFICATION_LANE_CHANGE);

    /* @brief clean up all references to the shadow vehicle
     */
    void cleanupShadowLane();

    /// @brief reserve space at the end of the lane to avoid dead locks
    virtual void saveBlockerLength(double length) {
        UNUSED_PARAMETER(length);
    };

    void setShadowPartialOccupator(MSLane* lane) {
        myPartiallyOccupatedByShadow.push_back(lane);
    }

    void setNoShadowPartialOccupator(MSLane* lane) {
        myNoPartiallyOccupatedByShadow.push_back(lane);
    }

    /// @brief called once when the vehicles primary lane changes
    void primaryLaneChanged(MSLane* source, MSLane* target, int direction);

    /// @brief set approach information for the shadow vehicle
    void setShadowApproachingInformation(MSLink* link) const;
    void removeShadowApproachingInformation() const;

    bool isOpposite() const {
        return myAmOpposite;
    }

    double getCommittedSpeed() const {
        return myCommittedSpeed;
    }

    /// @brief return the lateral speed of the current lane change maneuver
    double getSpeedLat() const {
        return mySpeedLat;
    }

    void setSpeedLat(double speedLat) {
        mySpeedLat = speedLat;
    }

    /// @brief try to retrieve the given parameter from this laneChangeModel. Throw exception for unsupported key
    virtual std::string getParameter(const std::string& key) const {
        throw InvalidArgument("Parameter '" + key + "' is not supported for laneChangeModel of type '" + toString(myModel) + "'");
    }

    /// @brief try to set the given parameter for this laneChangeModel. Throw exception for unsupported key
    virtual void setParameter(const std::string& key, const std::string& value) {
        UNUSED_PARAMETER(value);
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for laneChangeModel of type '" + toString(myModel) + "'");
    }

    static const double NO_NEIGHBOR;

protected:
    virtual bool congested(const MSVehicle* const neighLeader);

    virtual bool predInteraction(const std::pair<MSVehicle*, double>& leader);

    /// @brief whether the influencer cancels the given request
    bool cancelRequest(int state);


protected:
    /// @brief The vehicle this lane-changer belongs to
    MSVehicle& myVehicle;

    /// @brief The current state of the vehicle
    int myOwnState;
    std::map<int, std::pair<int, int> > mySavedStates;

    /// @brief the current lateral speed
    double mySpeedLat;

    /// @brief the speed when committing to a change maneuver
    double myCommittedSpeed;

    /// @brief progress of the lane change maneuver 0:started, 1:complete
    double myLaneChangeCompletion;

    /// @brief direction of the lane change maneuver -1 means right, 1 means left
    int myLaneChangeDirection;

    /// @brief whether the vehicle has already moved this step
    bool myAlreadyChanged;

    /// @brief A lane that is partially occupied by the front of the vehicle but that is not the primary lane
    MSLane* myShadowLane;

    /* @brief Lanes that are parially (laterally) occupied by the back of the
     * vehicle (analogue to MSVehicle::myFurtherLanes) */
    std::vector<MSLane*> myShadowFurtherLanes;
    std::vector<double> myShadowFurtherLanesPosLat;

    /// @brief The vehicle's car following model
    const MSCFModel& myCarFollowModel;

    /// @brief the type of this model
    const LaneChangeModel myModel;

    /// @brief list of lanes where the shadow vehicle is partial occupator
    std::vector<MSLane*> myPartiallyOccupatedByShadow;

    /* @brief list of lanes where there is no shadow vehicle partial occupator
     * (when changing to a lane that has no predecessor) */
    std::vector<MSLane*> myNoPartiallyOccupatedByShadow;

    /// @brief the minimum lateral gaps to other vehicles that were found when last changing to the left and right
    double myLastLateralGapLeft;
    double myLastLateralGapRight;

    /// @brief the actual minimum longitudinal distances to vehicles on the target lane
    double myLastLeaderGap;
    double myLastFollowerGap;
    /// @brief the minimum longitudinal distances to vehicles on the target lane that would be necessary for stringent security
    double myLastLeaderSecureGap;
    double myLastFollowerSecureGap;
    /// @brief acutal and secure distance to closest leader vehicle on the original when performing lane change
    double myLastOrigLeaderGap;
    double myLastOrigLeaderSecureGap;

    /* @brief to be called by derived classes in their changed() method.
     * If dir=0 is given, the current value remains unchanged */
    void initLastLaneChangeOffset(int dir);

    /// @brief whether overtaking on the right is permitted
    static bool myAllowOvertakingRight;

    /// @brief whether to record lane-changing
    static bool myLCOutput;


private:
    /* @brief information how long ago the vehicle has performed a lane-change,
     * sign indicates direction of the last change
     */
    SUMOTime myLastLaneChangeOffset;

    /// @brief links which are approached by the shadow vehicle
    mutable std::vector<MSLink*> myApproachedByShadow;

    /// @brief whether the vehicle is driving in the opposite direction
    bool myAmOpposite;


private:
    /// @brief Invalidated assignment operator
    MSAbstractLaneChangeModel& operator=(const MSAbstractLaneChangeModel& s);
};


#endif

/****************************************************************************/

