/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSDevice_SSM.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @author  Leonhard Luecken
/// @date    11.06.2013
/// @version $Id$
///
// An SSM-device logs encounters / conflicts of the carrying vehicle with other surrounding vehicles
// XXX: Preliminary implementation. Use with care. Especially rerouting vehicles could be problematic.
// TODO: implement SSM time-gap (estimated conflict entry and exit times are already calculated for PET calculation)
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <utils/common/StringTokenizer.h>
#include <utils/geom/GeomHelper.h>
#include <utils/common/TplConvert.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSNet.h>
#include <microsim/MSJunction.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleControl.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeoConvHelper.h>
#include "MSDevice_SSM.h"

// ===========================================================================
// Debug constants
// ===========================================================================
//#define DEBUG_SSM
//#define DEBUG_SSM_DRAC
//#define DEBUG_SSM_SURROUNDING
// #define DEBUG_SSM_NOTIFICATIONS

// ===========================================================================
// Constants
// ===========================================================================
// value indicating an invalid double parameter
#define INVALID std::numeric_limits<double>::max()

// default value for the detection range of potential opponents
#define DEFAULT_RANGE 50.0

// list of implemented SSMs (NOTE: To add more SSMs, identifiers are added to AVAILABLE_SSMS
//                                 and a default threshold must be defined. A corresponding
//                                 case should be added to the switch in buildVehicleDevices,
//                                 and in computeSSMs(), the SSM-value should be computed.)
#define AVAILABLE_SSMS "TTC DRAC PET"
#define DEFAULT_THRESHOLD_TTC 3. // in [s.], events get logged if below threshold (1.5s. is an appropriate criticality threshold according to Van der Horst, A. R. A. (1991). Time-to-collision as a Cue for Decision-making in Braking [also see Guido et al. 2011])
#define DEFAULT_THRESHOLD_DRAC 3. // in [m/s^2], events get logged if above threshold (3.4s. is an appropriate criticality threshold according to American Association of State Highway and Transportation Officials (2004). A Policy on Geometric Design of Highways and Streets [also see Guido et al. 2011])
#define DEFAULT_THRESHOLD_PET 2. // in seconds, events get logged if below threshold
#define DEFAULT_EXTRA_TIME 5.      // in seconds, events get logged for extra time even if encounter is over

// ===========================================================================
// method definitions
// ===========================================================================



/// Nicer output for EncounterType enum
std::ostream& operator<<(std::ostream& out, MSDevice_SSM::EncounterType type) {
    switch (type) {
        case MSDevice_SSM::ENCOUNTER_TYPE_NOCONFLICT_AHEAD:
            out << "NOCONFLICT_AHEAD";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_FOLLOWING:
            out << "FOLLOWING";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_FOLLOWING_FOLLOWER:
            out << "FOLLOWING_FOLLOWER";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_FOLLOWING_LEADER:
            out << "FOLLOWING_LEADER";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_ON_ADJACENT_LANES:
            out << "ON_ADJACENT_LANES";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_MERGING:
            out << "MERGING";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_MERGING_LEADER:
            out << "MERGING_LEADER";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_MERGING_FOLLOWER:
            out << "MERGING_FOLLOWER";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_MERGING_ADJACENT:
            out << "MERGING_ADJACENT";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_CROSSING:
            out << "CROSSING";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_CROSSING_LEADER:
            out << "CROSSING_LEADER";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_CROSSING_FOLLOWER:
            out << "CROSSING_FOLLOWER";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA:
            out << "EGO_ENTERED_CONFLICT_AREA";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA:
            out << "FOE_ENTERED_CONFLICT_AREA";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA:
            out << "BOTH_ENTERED_CONFLICT_AREA";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA:
            out << "EGO_LEFT_CONFLICT_AREA";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA:
            out << "FOE_LEFT_CONFLICT_AREA";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA:
            out << "BOTH_LEFT_CONFLICT_AREA";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_FOLLOWING_PASSED:
            out << "FOLLOWING_PASSED";
            break;
        case MSDevice_SSM::ENCOUNTER_TYPE_MERGING_PASSED:
            out << "MERGING_PASSED";
            break;
        // Collision (currently unused, might be differentiated further)
        case MSDevice_SSM::ENCOUNTER_TYPE_COLLISION:
            out << "COLLISION";
            break;
        default:
            out << "unknown type (" << int(type) << ")";
            break;
    }
    return out;
}


// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------

std::set<MSDevice*>* MSDevice_SSM::instances = new std::set<MSDevice*>();

const std::set<MSDevice*>&
MSDevice_SSM::getInstances() {
    return *instances;
}

void
MSDevice_SSM::cleanup() {
    // Close current encounters and flush conflicts to file for all existing devices
    if (instances != 0) {
        for (std::set<MSDevice*>::iterator ii = instances->begin(); ii != instances->end(); ++ii) {
            static_cast<MSDevice_SSM*>(*ii)->resetEncounters();
            static_cast<MSDevice_SSM*>(*ii)->flushConflicts(true);
        }
        instances->clear();
    }
}

void
MSDevice_SSM::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("SSM Device");
    insertDefaultAssignmentOptions("ssm", "SSM Device", oc);

    // custom options
    oc.doRegister("device.ssm.measures", new Option_String(""));
    oc.addDescription("device.ssm.measures", "SSM Device", "Specifies which measures will be logged (as a space seperated sequence of IDs in ('TTC', 'DRAC', 'PET')).");
    oc.doRegister("device.ssm.thresholds", new Option_String(""));
    oc.addDescription("device.ssm.thresholds", "SSM Device", "Specifies thresholds corresponding to the specified measures (see documentation and watch the order!). Only events exceeding the thresholds will be logged.");
    oc.doRegister("device.ssm.trajectories", new Option_Bool(false));
    oc.addDescription("device.ssm.trajectories", "SSM Device", "Specifies whether trajectories will be logged (if false, only the extremal values and times are reported, this is the default).");
    oc.doRegister("device.ssm.range", new Option_Float(DEFAULT_RANGE));
    oc.addDescription("device.ssm.range", "SSM Device", "Specifies the detection range in meters (default is " + toString(DEFAULT_RANGE) + "m.). For vehicles below this distance from the equipped vehicle, SSM values are traced.");
    oc.doRegister("device.ssm.extratime", new Option_Float(DEFAULT_EXTRA_TIME));
    oc.addDescription("device.ssm.extratime", "SSM Device", "Specifies the time in seconds to be logged after a conflict is over (default is " + toString(DEFAULT_EXTRA_TIME) + "secs.). Required >0 if PET is to be calculated for crossing conflicts.");
    oc.doRegister("device.ssm.geo", new Option_Bool(false));
    oc.addDescription("device.ssm.geo", "SSM Device", "Whether to use coordinates of the original reference system in output (default is false).");
}

void
MSDevice_SSM::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into) {
    if (equippedByDefaultAssignmentOptions(OptionsCont::getOptions(), "ssm", v)) {
        if (MSGlobals::gUseMesoSim) {
            WRITE_WARNING("SSM Device for vehicle '" + v.getID() + "' will not be built. (SSMs not supported in MESO)");
            return;
        }
        // ID for the device
        std::string deviceID = "ssm_" + v.getID();

        // Load parameters:

        // Measures and thresholds
        std::map<std::string, double> thresholds;
        bool success = getMeasuresAndThresholds(v, deviceID, thresholds);
        if (!success) {
            return;
        }

        // TODO: modify trajectory option: "all", "conflictPoints", ("position" && "speed" == "vehState"), "SSMs"!
        // Trajectories
        bool trajectories = requestsTrajectories(v);

        // detection range
        double range = getDetectionRange(v);

        // extra time
        double extraTime = getExtraTime(v);

        // File
        std::string file = getOutputFilename(v, deviceID);

        const bool useGeo = useGeoCoords(v, deviceID);

        // Build the device (XXX: who deletes it?)
        MSDevice_SSM* device = new MSDevice_SSM(v, deviceID, file, thresholds, trajectories, range, extraTime, useGeo);
        into.push_back(device);
    }
}


MSDevice_SSM::Encounter::Encounter(const MSVehicle* _ego, const MSVehicle* const _foe, double _begin, double extraTime) :
    currentType(ENCOUNTER_TYPE_NOCONFLICT_AHEAD),
    ego(_ego),
    foe(_foe),
    egoID(_ego->getID()),
    foeID(_foe->getID()),
    begin(_begin),
    end(-INVALID),
    remainingExtraTime(extraTime),
    egoConflictEntryTime(INVALID),
    egoConflictExitTime(INVALID),
    foeConflictEntryTime(INVALID),
    foeConflictExitTime(INVALID),
    minTTC(INVALID, Position::invalidPosition(), ENCOUNTER_TYPE_NOCONFLICT_AHEAD, INVALID),
    maxDRAC(INVALID, Position::invalidPosition(), ENCOUNTER_TYPE_NOCONFLICT_AHEAD, INVALID),
    PET(INVALID, Position::invalidPosition(), ENCOUNTER_TYPE_NOCONFLICT_AHEAD, INVALID),
    closingRequested(false) {
#ifdef DEBUG_SSM
    std::cout << "\n" << SIMTIME << " Constructing encounter of '"
              << ego->getID() << "' and '" << foe->getID() << "'" << std::endl;
#endif
}

MSDevice_SSM::Encounter::~Encounter() {
#ifdef DEBUG_SSM
    std::cout << "\n" << SIMTIME << " Destroying encounter of '"
              << egoID << "' and '" << foeID << "' (begin was " << begin << ")" << std::endl;
#endif
}


void
MSDevice_SSM::Encounter::add(double time, const EncounterType type, Position egoX, Position egoV, Position foeX, Position foeV,
                             Position conflictPoint, double egoDistToConflict, double foeDistToConflict, double ttc, double drac, std::pair<double, double> pet) {
#ifdef DEBUG_SSM
    std::cout << time << " Adding data point for encounter of '" << egoID << "' and '" << foeID << "':\n"
              << "type=" << type << ", egoDistToConflict=" << (egoDistToConflict == INVALID ? "NA" : toString(egoDistToConflict))
              << ", foeDistToConflict=" << (foeDistToConflict == INVALID ? "NA" : toString(foeDistToConflict))
              << ",\nttc=" << (ttc == INVALID ? "NA" : toString(ttc))
              << ", drac=" << (drac == INVALID ? "NA" : toString(drac))
              << ", pet=" << (pet.second == INVALID ? "NA" : toString(pet.second))
              << std::endl;
#endif
    currentType = type;

    timeSpan.push_back(time);
    typeSpan.push_back(type);
    egoTrajectory.x.push_back(egoX);
    egoTrajectory.v.push_back(egoV);
    foeTrajectory.x.push_back(foeX);
    foeTrajectory.v.push_back(foeV);
    conflictPointSpan.push_back(conflictPoint);
    egoDistsToConflict.push_back(egoDistToConflict);
    foeDistsToConflict.push_back(foeDistToConflict);

    TTCspan.push_back(ttc);
    if (ttc != INVALID && (ttc < minTTC.value || minTTC.value == INVALID)) {
        minTTC.value = ttc;
        minTTC.time = time;
        minTTC.pos = conflictPoint;
        minTTC.type = type;
    }

    DRACspan.push_back(drac);
    if (drac != INVALID && (drac > maxDRAC.value || maxDRAC.value == INVALID)) {
        maxDRAC.value = drac;
        maxDRAC.time = time;
        maxDRAC.pos = conflictPoint;
        maxDRAC.type = type;
    }

    if (pet.first != INVALID && (PET.value >= pet.second || PET.value == INVALID)) {
        PET.value = pet.second;
        PET.time = pet.first;
        PET.pos = conflictPoint;
        PET.type = type;
    }
}


void
MSDevice_SSM::Encounter::resetExtraTime(double value) {
    remainingExtraTime = value;
}


void
MSDevice_SSM::Encounter::countDownExtraTime(double amount) {
    remainingExtraTime -= amount;
}


double
MSDevice_SSM::Encounter::getRemainingExtraTime() const {
    return remainingExtraTime;
}


MSDevice_SSM::EncounterApproachInfo::EncounterApproachInfo(Encounter* e) :
    encounter(e),
    type(ENCOUNTER_TYPE_NOCONFLICT_AHEAD),
    conflictPoint(Position::invalidPosition()),
    egoConflictEntryDist(INVALID),
    foeConflictEntryDist(INVALID),
    egoConflictExitDist(INVALID),
    foeConflictExitDist(INVALID),
    egoEstimatedConflictEntryTime(INVALID),
    foeEstimatedConflictEntryTime(INVALID),
    egoEstimatedConflictExitTime(INVALID),
    foeEstimatedConflictExitTime(INVALID),
    egoConflictAreaLength(INVALID),
    foeConflictAreaLength(INVALID),
    egoLeftConflict(false),
    foeLeftConflict(false),
    ttc(INVALID),
    drac(INVALID),
    pet(std::make_pair(INVALID, INVALID)) {
}


void
MSDevice_SSM::updateAndWriteOutput() {
    if (myHolder.isOnRoad()) {
        update();
        // Write out past conflicts
        flushConflicts();
    } else {
        resetEncounters();
        // Write out past conflicts
        flushConflicts(true);
    }
}

void
MSDevice_SSM::update() {
#ifdef DEBUG_SSM
    std::cout << "\n" << SIMTIME << " Device '" << getID() << "' update()\n"
              << "Size of myActiveEncounters: " << myActiveEncounters.size()
              << "\nSize of myPastConflicts: " << myPastConflicts.size()
              << std::endl;
#endif
    // Scan surroundings for other vehicles
    FoeInfoMap foes;
    findSurroundingVehicles(*myHolderMS, myRange, foes);

#ifdef DEBUG_SSM
    if (foes.size() > 0) {
        std::cout << "Scanned surroundings: Found potential foes:\n";
        for (FoeInfoMap::const_iterator i = foes.begin(); i != foes.end(); ++i) {
            std::cout << i->first->getID() << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Scanned surroundings: No potential conflict could be identified." << std::endl;
    }
#endif

    // Update encounters and conflicts -> removes all foes (and deletes corresponding FoeInfos) for which already a corresponding encounter exists
    processEncounters(foes);

    // Make new encounters for all foes, which were not removed by processEncounters (and deletes corresponding FoeInfos)
    createEncounters(foes);
    foes.clear();
}


void
MSDevice_SSM::createEncounters(FoeInfoMap& foes) {
#ifdef DEBUG_SSM
    std::cout << "\n" << SIMTIME << " Device '" << getID() << "' createEncounters()" << std::endl;
    std::cout << "New foes:\n";
    for (FoeInfoMap::const_iterator vi = foes.begin(); vi != foes.end(); ++vi) {
        std::cout << vi->first->getID() << "\n";
    }
    std::cout << std::endl;
#endif

    for (FoeInfoMap::const_iterator foe = foes.begin(); foe != foes.end(); ++foe) {
        std::pair<MSLane*, MSLane*> conflictLanes;
        Encounter* e = new Encounter(myHolderMS, foe->first, SIMTIME, myExtraTime);
        updateEncounter(e, foe->second); // deletes foe->second
        if (myOldestActiveEncounterBegin == INVALID) {
            assert(myActiveEncounters.empty());
            myOldestActiveEncounterBegin = e->begin;
        }
        assert(myOldestActiveEncounterBegin <= e->begin);
        myActiveEncounters.push_back(e);
    }
}

void
MSDevice_SSM::resetEncounters() {
    // Call processEncounters() with empty vehicle set
    FoeInfoMap foes;
    // processEncounters with empty argument closes all encounters
    processEncounters(foes, true);
}

void
MSDevice_SSM::processEncounters(FoeInfoMap& foes, bool forceClose) {
#ifdef DEBUG_SSM
    std::cout << "\n" << SIMTIME << " Device '" << getID() << "' processEncounters()" << std::endl;
    std::cout << "Currently present foes:\n";
    for (FoeInfoMap::const_iterator vi = foes.begin(); vi != foes.end(); ++vi) {
        std::cout << vi->first->getID() << "\n";
    }
    std::cout << std::endl;
#endif

    // Run through active encounters. If corresponding foe is still present in foes update and
    // remove foe from foes. If the foe has disappeared close the encounter (check if it qualifies
    // as a conflict and in case transfer it to myPastConflicts).
    // Afterwards run through remaining elements in foes and create new encounters for them.

    EncounterVector::iterator ei = myActiveEncounters.begin();
    while (ei != myActiveEncounters.end()) {
        Encounter* e = *ei;
        // check whether foe is still on net
        bool foeExists = !(MSNet::getInstance()->getVehicleControl().getVehicle(e->foeID) == 0);
        if (!foeExists) {
            e->foe = 0;
        }
        if (foes.find(e->foe) != foes.end()) {
            FoeInfo* foeInfo = foes[e->foe];
            // Update encounter
            updateEncounter(e, foeInfo); // deletes foeInfo
            // Erase foes which were already encountered
            foes.erase(e->foe);
        } else {
            if (e->getRemainingExtraTime() <= 0. || forceClose || !foeExists) {
                // Close encounter, extra time has expired (deletes e if it does not qualify as conflict)
                e->closingRequested = true;
            } else {
                updateEncounter(e, 0); // counts down extra time
            }
        }

        if (e->closingRequested) {
            double eBegin = e->begin;
            closeEncounter(e);
            ei = myActiveEncounters.erase(ei);
            if (myActiveEncounters.empty()) {
                myOldestActiveEncounterBegin = INVALID;
            } else if (eBegin == myOldestActiveEncounterBegin) {
                // Erased the oldest encounter, update myOldestActiveEncounterBegin
                auto i = myActiveEncounters.begin();
                myOldestActiveEncounterBegin = (*i++)->begin;
                while (i != myActiveEncounters.end()) {
                    myOldestActiveEncounterBegin = MIN2(myOldestActiveEncounterBegin, (*i++)->begin);
                }
            }
        } else {
            ++ei;
        }
    }
}


bool
MSDevice_SSM::qualifiesAsConflict(Encounter* e) {
    // Check if conflict measure thresholds are exceeded (to decide whether to keep the encounter for writing out)
#ifdef DEBUG_SSM
    std::cout << SIMTIME << " qualifiesAsConflict() for encounter of vehicles '"
              << e->egoID << "' and '" << e->foeID
              << "'" << std::endl;
#endif

    if (myComputePET && e->PET.value != INVALID && e->PET.value <= myThresholds["PET"]) {
        return true;
    }
    if (myComputeTTC && e->minTTC.value != INVALID && e->minTTC.value <= myThresholds["TTC"]) {
        return true;
    }
    if (myComputeDRAC && e->maxDRAC.value != INVALID && e->maxDRAC.value >= myThresholds["DRAC"]) {
        return true;
    }
    return false;
}


void
MSDevice_SSM::closeEncounter(Encounter* e) {
    // erase pointers (encounter is stored before being destroyed and pointers could become invalid)
    e->ego = 0;
    e->foe = 0;
    e->end = e->timeSpan.back();
    bool wasConflict = qualifiesAsConflict(e);
    if (wasConflict) {
        myPastConflicts.push(e);
    } else {
        delete e;
    }
#ifdef DEBUG_SSM
    std::cout << SIMTIME << " closeEncounter() of vehicles '"
              << e->egoID << "' and '" << e->foeID
              << "' (was ranked as " << (wasConflict ? "conflict" : "non-conflict") << ")" << std::endl;
#endif

    return;
}


void
MSDevice_SSM::updateEncounter(Encounter* e, FoeInfo* foeInfo) {
#ifdef DEBUG_SSM
    std::cout << SIMTIME << " updateEncounter() of vehicles '"
              << e->egoID << "' and '" << e->foeID
              << "'" << std::endl;
#endif
    assert(e->foe != 0);

    // Struct storing distances (determined in classifyEncounter()) and times to potential conflict entry / exit (in estimateConflictTimes())
    EncounterApproachInfo eInfo(e);

    // Classify encounter type based on the present information
    // More details on follower/lead relation are determined in a second step below, see estimateConflictTimes()
    // If a crossing situation is ongoing (i.e. one of the vehicles entered the conflict area already in the last step,
    // this is handled by passedEncounter by only tracing the vehicle's movements)
    // The further developement of the encounter type is done in checkConflictEntryAndExit()
    eInfo.type = classifyEncounter(foeInfo, eInfo);

    if (eInfo.type == ENCOUNTER_TYPE_NOCONFLICT_AHEAD) {
        // At this state, eInfo.type == ENCOUNTER_TYPE_NOCONFLICT_AHEAD implies that the foe
        // is either out of the device's range or its route does not interfere with the ego's route.
#ifdef DEBUG_SSM
        std::cout << SIMTIME << " Encounter of vehicles '"
                  << e->egoID << "' and '" << e->foeID
                  << "' does not imply any conflict." << std::endl;
#endif
        updatePassedEncounter(e, foeInfo, eInfo);
//        return;
    } else if (eInfo.type == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA
               || eInfo.type == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA
               || eInfo.type == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
               || eInfo.type == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
               || eInfo.type == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA) {
        // Ongoing encounter. Treat with update passed encounter (trace covered distances)
        // eInfo.type only holds the previous type
        updatePassedEncounter(e, foeInfo, eInfo);

        // Estimate times until a possible conflict / collision
        estimateConflictTimes(eInfo);

    } else {
        // Estimate times until a possible conflict / collision
        // Not all are used for all types of encounters:
        // Follow/lead situation doesn't need them at all, currently (might change if more SSMs are implemented).
        // Crossing / Merging calculates entry times to determine leader/follower and calculates the exit time for the leader.
        estimateConflictTimes(eInfo);

        // reset the remaining extra time (foe could have re-entered the device range after beginning extra time countdown already)
        e->resetExtraTime(myExtraTime);
    }

    // update entry/exit times for conflict area
    checkConflictEntryAndExit(eInfo);

    // update (x,y)-coords of conflict point
    determineConflictPoint(eInfo);

    // Compute SSMs
    computeSSMs(eInfo);

    // Add current states to trajectories and update type
    e->add(SIMTIME, eInfo.type, e->ego->getPosition(), e->ego->getVelocityVector(), e->foe->getPosition(), e->foe->getVelocityVector(),
           eInfo.conflictPoint, eInfo.egoConflictEntryDist, eInfo.foeConflictEntryDist, eInfo.ttc, eInfo.drac, eInfo.pet);

    // free foeInfo
    delete foeInfo;

}


void
MSDevice_SSM::determineConflictPoint(EncounterApproachInfo& eInfo) {
    /*  Calculates the (x,y)-coordinate for the eventually predicted conflict point and stores the result in
     *         eInfo.conflictPoint. In case of MERGING and CROSSING, this is the entry point to conflict area for follower
     *         In case of FOLLOWING it is the position of leader's back. */

#ifdef DEBUG_SSM
    std::cout << SIMTIME << " determineConflictPoint()" << std::endl;
#endif

    const EncounterType& type = eInfo.type;
    const Encounter* e = eInfo.encounter;
    if (type == ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA
            || type == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA
            || type == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
            || type == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
            || type == ENCOUNTER_TYPE_COLLISION) {
        eInfo.conflictPoint = e->conflictPointSpan.back();
    } else if (type == ENCOUNTER_TYPE_CROSSING_FOLLOWER
               || type == ENCOUNTER_TYPE_MERGING_FOLLOWER
               || type == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA) {
        eInfo.conflictPoint = e->ego->getPositionAlongBestLanes(eInfo.egoConflictEntryDist);
    } else if (type == ENCOUNTER_TYPE_CROSSING_LEADER
               || type == ENCOUNTER_TYPE_MERGING_LEADER
               || type == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA) {
        eInfo.conflictPoint = e->foe->getPositionAlongBestLanes(eInfo.foeConflictEntryDist);
    } else if (type == ENCOUNTER_TYPE_FOLLOWING_FOLLOWER) {
        eInfo.conflictPoint = e->foe->getPosition(-e->foe->getLength());
    } else if (type == ENCOUNTER_TYPE_FOLLOWING_LEADER) {
        eInfo.conflictPoint = e->ego->getPosition(-e->ego->getLength());
    } else {
#ifdef DEBUG_SSM
        std::cout << "No conflict point associated with encounter type " << type << std::endl;
#endif
        return;
    }

#ifdef DEBUG_SSM
    std::cout << "    Conflict at " << eInfo.conflictPoint << std::endl;
#endif
}


void
MSDevice_SSM::estimateConflictTimes(EncounterApproachInfo& eInfo) {

    EncounterType& type = eInfo.type;
    Encounter* e = eInfo.encounter;

    assert(type != ENCOUNTER_TYPE_NOCONFLICT_AHEAD); // arrival times not defined, if no conflict is ahead.
#ifdef DEBUG_SSM
    std::cout << SIMTIME << " estimateConflictTimes() for ego '" << e->egoID << "' and foe '" << e->foeID << "'\n"
              << "    encounter type: " << eInfo.type << "\n"
              << "    egoConflictEntryDist=" << (eInfo.egoConflictEntryDist == INVALID ? "NA" : toString(eInfo.egoConflictEntryDist))
              << ", foeConflictEntryDist=" << (eInfo.foeConflictEntryDist == INVALID ? "NA" : toString(eInfo.foeConflictEntryDist))
              << "\n    ego speed=" << e->ego->getSpeed()
              << ", foe speed=" << e->foe->getSpeed()
              << std::endl;
#endif

    if (type == ENCOUNTER_TYPE_FOLLOWING_FOLLOWER || type == ENCOUNTER_TYPE_FOLLOWING_LEADER || type == ENCOUNTER_TYPE_MERGING_ADJACENT || type == ENCOUNTER_TYPE_ON_ADJACENT_LANES) {
        // No need to know the times until ...ConflictDistEntry, currently. They would correspond to an estimated time headway or similar.
        // TTC must take into account the movement of the leader, as would DRAC, PET doesn't need the time either, since it uses aposteriori
        // values.
#ifdef DEBUG_SSM
        std::cout << "    encouter type " << type << " -> no entry/exit times to be calculated."
                  << std::endl;
#endif
        return;
    }

    assert(type == ENCOUNTER_TYPE_MERGING || type == ENCOUNTER_TYPE_CROSSING
           || type == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA
           || type == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA
           || type == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
           || type == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
           || type == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA
           || type == ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA);

    // Determine exit distances
    if (type == ENCOUNTER_TYPE_MERGING)  {
        assert(type == ENCOUNTER_TYPE_MERGING);
        eInfo.egoConflictExitDist = eInfo.egoConflictEntryDist + e->ego->getVehicleType().getLength();
        eInfo.foeConflictExitDist = eInfo.foeConflictEntryDist + e->foe->getVehicleType().getLength();
    } else {
        eInfo.egoConflictExitDist = eInfo.egoConflictEntryDist + eInfo.egoConflictAreaLength + e->ego->getVehicleType().getLength();
        eInfo.foeConflictExitDist = eInfo.foeConflictEntryDist + eInfo.foeConflictAreaLength + e->foe->getVehicleType().getLength();
    }

    // Estimate entry times to stipulate a leader / follower relation for the encounter.
    if (eInfo.egoConflictEntryDist > 0.) {
        eInfo.egoEstimatedConflictEntryTime = e->ego->getCarFollowModel().estimateArrivalTime(eInfo.egoConflictEntryDist, e->ego->getSpeed(), e->ego->getMaxSpeedOnLane(), MIN2(0., e->ego->getAcceleration()));
        assert(eInfo.egoEstimatedConflictEntryTime > 0.);
    } else {
        // ego already entered conflict area
        eInfo.egoEstimatedConflictEntryTime = 0.;
    }
    if (eInfo.foeConflictEntryDist > 0.) {
        eInfo.foeEstimatedConflictEntryTime = e->foe->getCarFollowModel().estimateArrivalTime(eInfo.foeConflictEntryDist, e->foe->getSpeed(), e->foe->getMaxSpeedOnLane(), MIN2(0., e->foe->getAcceleration()));
        assert(eInfo.foeEstimatedConflictEntryTime > 0.);
    } else {
        // foe already entered conflict area
        eInfo.foeEstimatedConflictEntryTime = 0.;
    }

#ifdef DEBUG_SSM
    std::cout << "    Potential conflict type: " << (type == ENCOUNTER_TYPE_CROSSING ? "CROSSING" : "MERGING") << "\n"
              << "    egoConflictEntryTime=" << (eInfo.egoEstimatedConflictEntryTime == INVALID ? "INVALID" : toString(eInfo.egoEstimatedConflictEntryTime))
              << ", foeConflictEntryTime=" << (eInfo.foeEstimatedConflictEntryTime == INVALID ? "INVALID" : toString(eInfo.foeEstimatedConflictEntryTime))
              << std::endl;
#endif

    // Estimate exit times from conflict area for leader / follower.
    if (eInfo.egoConflictExitDist >= 0.) {
        eInfo.egoEstimatedConflictExitTime = e->ego->getCarFollowModel().estimateArrivalTime(eInfo.egoConflictExitDist, e->ego->getSpeed(), e->ego->getMaxSpeedOnLane(), MIN2(0., e->ego->getAcceleration()));
    } else {
        eInfo.egoEstimatedConflictExitTime = 0.;
    }
    if (eInfo.foeConflictExitDist >= 0.) {
        eInfo.foeEstimatedConflictExitTime = e->foe->getCarFollowModel().estimateArrivalTime(eInfo.foeConflictExitDist, e->foe->getSpeed(), e->foe->getMaxSpeedOnLane(), MIN2(0., e->foe->getAcceleration()));
    } else {
        eInfo.foeEstimatedConflictExitTime = 0.;
    }


    if (type != ENCOUNTER_TYPE_MERGING && type != ENCOUNTER_TYPE_CROSSING) {
        // this call is issued in context of an ongoing conflict, therefore complete type is already known for the encounter
        // (One of EGO_ENTERED_CONFLICT_AREA, FOE_ENTERED_CONFLICT_AREA, EGO_LEFT_CONFLICT_AREA, FOE_LEFT_CONFLICT_AREA, BOTH_ENTERED_CONFLICT_AREA)
        // --> no need to specify incomplete encounter type
        return;
    }

    // For merging and crossing situation, the leader/follower relation not determined by classifyEncounter()
    // This is done below based on the estimated conflict entry times
    if (eInfo.egoEstimatedConflictEntryTime == 0. && eInfo.foeEstimatedConflictEntryTime == 0.) {
        type = ENCOUNTER_TYPE_COLLISION;
        std::stringstream ss;
        ss << "SSM device of vehicle '" << e->egoID << "' detected collision with vehicle '" << e->foeID << "'";
        WRITE_WARNING(ss.str());
    } else if (eInfo.egoEstimatedConflictEntryTime < eInfo.foeEstimatedConflictEntryTime) {
        // ego is estimated first at conflict point
#ifdef DEBUG_SSM
        std::cout << "    -> ego is estimated leader at conflict entry."
                  << " egoConflictExitTime=" << (eInfo.egoEstimatedConflictExitTime == INVALID ? "NA" : toString(eInfo.egoEstimatedConflictExitTime))
                  << std::endl;
#endif
        type = type == ENCOUNTER_TYPE_CROSSING ? ENCOUNTER_TYPE_CROSSING_LEADER : ENCOUNTER_TYPE_MERGING_LEADER;
    } else {
        // ego is estimated second at conflict point
#ifdef DEBUG_SSM
        std::cout << "    -> foe is estimated leader at conflict entry."
                  << " foeConflictExitTime=" << (eInfo.foeEstimatedConflictExitTime == INVALID ? "NA" : toString(eInfo.foeEstimatedConflictExitTime))
                  << std::endl;
#endif
        type = type == ENCOUNTER_TYPE_CROSSING ? ENCOUNTER_TYPE_CROSSING_FOLLOWER : ENCOUNTER_TYPE_MERGING_FOLLOWER;
    }

}



void
MSDevice_SSM::computeSSMs(EncounterApproachInfo& eInfo) const {
#ifdef DEBUG_SSM
    Encounter* e = eInfo.encounter;
    std::cout << SIMTIME << " computeSSMs() for vehicles '"
              << e->ego->getID() << "' and '" << e->foe->getID()
              << "'" << std::endl;
#endif

    const EncounterType& type = eInfo.type;

    if (type == ENCOUNTER_TYPE_CROSSING_FOLLOWER || type == ENCOUNTER_TYPE_CROSSING_LEADER
            || type == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA || type == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA
            || type == ENCOUNTER_TYPE_MERGING_FOLLOWER || type == ENCOUNTER_TYPE_MERGING_LEADER
            || type == ENCOUNTER_TYPE_FOLLOWING_FOLLOWER || type == ENCOUNTER_TYPE_FOLLOWING_LEADER) {
        if (myComputeTTC || myComputeDRAC) {
            determineTTCandDRAC(eInfo);
        }
        determinePET(eInfo);
    } else if (type == ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA) {
        determinePET(eInfo);
    } else if (type == ENCOUNTER_TYPE_COLLISION) {
        // TODO: handle collision
    } else if (type == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA || type == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
               || type == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA || type == ENCOUNTER_TYPE_NOCONFLICT_AHEAD) {
        // No conflict measures apply for these states, which correspond to intermediate times between
        // one vehicle leaving the conflict area and the arrival time for the other (difference corresponds to the PET)
    } else if (type == ENCOUNTER_TYPE_ON_ADJACENT_LANES || type == ENCOUNTER_TYPE_MERGING_ADJACENT) {
        // No conflict measures apply for this state
    } else if (type == ENCOUNTER_TYPE_MERGING_PASSED || type == ENCOUNTER_TYPE_FOLLOWING_PASSED) {
        // No conflict measures apply for this state
    } else if (type == ENCOUNTER_TYPE_NOCONFLICT_AHEAD) {
        // No conflict measures apply for this state
    } else {
        std::stringstream ss;
        ss << "'" << type << "'";
        WRITE_WARNING("Unknown or undetermined encounter type at computeSSMs(): " + ss.str());
    }

#ifdef DEBUG_SSM
    std::cout << "computeSSMs() for encounter of vehicles '" << e->egoID << "' and '" << e->foeID << "':\n"
              << "  ttc=" << (eInfo.ttc == INVALID ? "INVALID" : toString(eInfo.ttc))
              << ", drac=" << (eInfo.drac == INVALID ? "INVALID" : toString(eInfo.drac))
              << ", pet=" << (eInfo.pet.second == INVALID ? "INVALID" : toString(eInfo.pet.second))
              << std::endl;
#endif
}


void
MSDevice_SSM::determinePET(EncounterApproachInfo& eInfo) const {
    Encounter* e = eInfo.encounter;
    if (e->size() == 0) {
        return;
    }
    const EncounterType& type = eInfo.type;
    std::pair<double, double>& pet = eInfo.pet;

#ifdef DEBUG_SSM
    std::cout << SIMTIME << " determinePET() for encounter of vehicles '" << e->egoID << "' and '" << e->foeID << "'"
              << std::endl;
#endif

    if (type == ENCOUNTER_TYPE_FOLLOWING_FOLLOWER || type == ENCOUNTER_TYPE_FOLLOWING_LEADER) {
        // For a following situation, the corresponding PET-value is merely the time-headway.
        //       Determining these could be done by comparison of memorized gaps with memorized covered distances
        //       Implementation is postponed. Tracing the time gaps (in contrast to crossing PET) corresponds to
        //       a vector of values not a single value.
        // pass
    } else if (type == ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA) {
        EncounterType prevType = static_cast<EncounterType>(e->typeSpan.back());
        if (prevType == ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA) {
#ifdef DEBUG_SSM
            std::cout << "PET for crossing encounter already calculated as " << e->PET.second
                      << std::endl;
#endif
            // pet must have been calculated already
            assert(e->PET.value != INVALID);
            return;
        }

        // this situation should have emerged from one of the following
        assert(prevType == ENCOUNTER_TYPE_CROSSING_FOLLOWER
               || prevType == ENCOUNTER_TYPE_CROSSING_LEADER
               || prevType == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
               || prevType == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
               || prevType == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA
               || prevType == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA
               || prevType == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA);


#ifdef DEBUG_SSM
        std::cout << "e->egoDistsToConflict.back() = " << e->egoDistsToConflict.back()
                  << "\ne->egoConflictEntryTime = " << e->egoConflictEntryTime
                  << "\ne->egoConflictExitTime = " << e->egoConflictExitTime
                  << "\ne->foeDistsToConflict.back() = " << e->foeDistsToConflict.back()
                  << "\ne->foeConflictEntryTime = " << e->foeConflictEntryTime
                  << "\ne->foeConflictExitTime = " << e->foeConflictExitTime
                  << std::endl;
#endif

        assert(e->foeConflictEntryTime != INVALID);
        assert(e->egoConflictEntryTime != INVALID);

        // Both have passed the conflict area
        assert(e->foeConflictExitTime != INVALID && e->egoConflictExitTime != INVALID);
        // both have left the conflict region already
        if (e->egoConflictEntryTime > e->foeConflictExitTime) {
            pet.first = e->egoConflictEntryTime;
            pet.second = e->egoConflictEntryTime - e->foeConflictExitTime;
        } else if (e->foeConflictEntryTime > e->egoConflictExitTime) {
            pet.first = e->foeConflictEntryTime;
            pet.second = e->foeConflictEntryTime - e->egoConflictExitTime;
        } else {

#ifdef DEBUG_SSM
            std::cout << "Unexpected branch in determinePET: Both passed conflict area in the same step."
                      << std::endl;
#endif
            pet.first = INVALID;
            pet.second = INVALID;
            assert(prevType != ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
                   && prevType != ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA);
        }

        // Reset entry and exit times two allow an eventual subsequent re-use
        e->egoConflictEntryTime = INVALID;
        e->egoConflictExitTime = INVALID;
        e->foeConflictEntryTime = INVALID;
        e->foeConflictExitTime = INVALID;

#ifdef DEBUG_SSM
        std::cout << "Calculated PET = " << pet.second << " (at t=" << pet.first << ")"
                  << std::endl;
#endif
    } else {
        // other cases (merging and pre-crossing situations) do not correspond to a PET calculation.
#ifdef DEBUG_SSM
        std::cout << "PET unappropriate for merging and pre-crossing situations. No calculation performed."
                  << std::endl;
#endif
        return;
    }
}


void
MSDevice_SSM::determineTTCandDRAC(EncounterApproachInfo& eInfo) const {
    Encounter* e = eInfo.encounter;
    const EncounterType& type = eInfo.type;
    double& ttc = eInfo.ttc;
    double& drac = eInfo.drac;

#ifdef DEBUG_SSM
    std::cout << SIMTIME << " determineTTCandDRAC() for encounter of vehicles '" << e->egoID << "' and '" << e->foeID << "' (type = " << eInfo.type << ")"
              << std::endl;
#endif

    // Dependent on the actual encounter situation (eInfo.type) calculate the TTC.
    // For merging and crossing, different cases occur when a collision during the merging / crossing process is predicted.
    if (type == ENCOUNTER_TYPE_FOLLOWING_FOLLOWER) {
        double gap = eInfo.egoConflictEntryDist;
        if (myComputeTTC) {
            ttc = computeTTC(gap, e->ego->getSpeed(), e->foe->getSpeed());
        }
        if (myComputeDRAC) {
            drac = computeDRAC(gap, e->ego->getSpeed(), e->foe->getSpeed());
        }
    } else if (type == ENCOUNTER_TYPE_FOLLOWING_LEADER) {
        double gap = eInfo.foeConflictEntryDist;
        if (myComputeTTC) {
            ttc = computeTTC(gap, e->foe->getSpeed(), e->ego->getSpeed());
        }
        if (myComputeDRAC) {
            drac = computeDRAC(gap, e->foe->getSpeed(), e->ego->getSpeed());
        }
    } else if (type == ENCOUNTER_TYPE_MERGING_FOLLOWER || type == ENCOUNTER_TYPE_MERGING_LEADER) {
        // TODO: calculate more specifically whether a following situation in the merge conflict area
        //       is predicted when assuming constant speeds or whether a side collision is predicted.
        //       Currently, we ignore any conflict area before the actual merging point of the lanes.

        // linearly extrapolated arrival times at the conflict
        // NOTE: These differ from the estimated times stored in eInfo
        double egoEntryTime = e->ego->getSpeed() > 0 ? eInfo.egoConflictEntryDist / e->ego->getSpeed() : INVALID;
        double egoExitTime = e->ego->getSpeed() > 0 ? eInfo.egoConflictExitDist / e->ego->getSpeed() : INVALID;
        double foeEntryTime = e->foe->getSpeed() > 0 ? eInfo.foeConflictEntryDist / e->foe->getSpeed() : INVALID;
        double foeExitTime = e->foe->getSpeed() > 0 ? eInfo.foeConflictExitDist / e->foe->getSpeed() : INVALID;

#ifdef DEBUG_SSM
        std::cout << "   Conflict times with constant speed extrapolation for merging situation:\n   "
                  << " egoEntryTime=" << (egoEntryTime == INVALID ? "NA" : toString(egoEntryTime))
                  << ", egoExitTime=" << (egoExitTime == INVALID ? "NA" : toString(egoExitTime))
                  << ", foeEntryTime=" << (foeEntryTime == INVALID ? "NA" : toString(foeEntryTime))
                  << ", foeExitTime=" << (foeExitTime == INVALID ? "NA" : toString(foeExitTime))
                  << std::endl;
#endif

        // based on that, we obtain
        if (egoEntryTime == INVALID || foeEntryTime == INVALID) {
            // at least one vehicle is stopped
            ttc = INVALID;
            drac = INVALID;
#ifdef DEBUG_SSM
            std::cout << "    No TTC and DRAC computed as one vehicle is stopped." << std::endl;
#endif
            return;
        }
        double leaderEntryTime = MIN2(egoEntryTime, foeEntryTime);
        double followerEntryTime = MAX2(egoEntryTime, foeEntryTime);
        double leaderExitTime = leaderEntryTime == egoEntryTime ? egoExitTime : foeExitTime;
        //double followerExitTime = leaderEntryTime==egoEntryTime?foeExitTime:egoExitTime;
        double leaderSpeed = leaderEntryTime == egoEntryTime ? e->ego->getSpeed() : e->foe->getSpeed();
        double followerSpeed = leaderEntryTime == egoEntryTime ? e->foe->getSpeed() : e->ego->getSpeed();
        double leaderConflictDist = leaderEntryTime == egoEntryTime ? eInfo.egoConflictEntryDist : eInfo.foeConflictEntryDist;
        double followerConflictDist = leaderEntryTime == egoEntryTime ? eInfo.foeConflictEntryDist : eInfo.egoConflictEntryDist;
        double leaderLength = leaderEntryTime == egoEntryTime ? e->ego->getLength() : e->foe->getLength();
        if (leaderExitTime >= followerEntryTime) {
            // collision would occur at merge area
            if (myComputeTTC) {
                ttc = computeTTC(followerConflictDist, followerSpeed, 0.);
            }
            // TODO: Calculate more specific drac for merging case here (complete stop is not always necessary -> see calculation for crossing case)
            //       Rather the
            if (myComputeDRAC) {
                drac = computeDRAC(followerConflictDist, followerSpeed, 0.);
            }
//            if (myComputeDRAC) drac = computeDRAC(eInfo);

#ifdef DEBUG_SSM
            std::cout << "    Extrapolation predicts collision *at* merge point with TTC=" << ttc
                      << ", drac=" << drac << std::endl;
#endif

        } else {
            // -> No collision at the merge area
            if (myComputeTTC) {
                // Check if after merge a collision would occur if speeds are hold constant.
                double gapAfterMerge = followerConflictDist - leaderExitTime * followerSpeed;
                assert(gapAfterMerge >= 0);

                // ttc as for following situation (assumes no collision until leader merged)
                double ttcAfterMerge = computeTTC(gapAfterMerge, followerSpeed, leaderSpeed);
                ttc = ttcAfterMerge == INVALID ? INVALID : leaderExitTime + ttcAfterMerge;
            }
            if (myComputeDRAC) {
                // Intitial gap. (May be negative only if the leader speed is higher than the follower speed, i.e., dv < 0)
                double g0 = followerConflictDist - leaderConflictDist - leaderLength;
                // Speed difference. (Must be positive if g0<0)
                double dv = leaderSpeed - followerSpeed;
                if (g0 < 0) {
                    assert(dv > 0);
                    // no deceleration needed for dv>0 and gap after merge >= 0
                    drac = INVALID;
                } else {
                    // compute drac as for a following situation
                    drac = computeDRAC(g0, followerSpeed, leaderSpeed);
                }
            }
#ifdef DEBUG_SSM
            if (ttc == INVALID) {
                assert(dv >= 0);
                assert(drac == INVALID);
                std::cout << "    Extrapolation does not predict any collision." << std::endl;
            } else {
                std::cout << "    Extrapolation predicts collision *after* merge point with TTC="
                          << (ttc == INVALID ? "NA" : toString(ttc))
                          << ", drac=" << (drac == INVALID ? "NA" : toString(drac)) << std::endl;
            }
#endif

        }

    } else if (type == ENCOUNTER_TYPE_CROSSING_FOLLOWER
               || type == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA) {
        if (myComputeDRAC) {
            drac = computeDRAC(eInfo);
        }
        if (eInfo.egoEstimatedConflictEntryTime <= eInfo.foeEstimatedConflictExitTime) {
            // follower's predicted arrival at the crossing area is earlier than the leader's predicted exit -> collision predicted
            double gap = eInfo.egoConflictEntryDist;
            if (myComputeTTC) {
                ttc = computeTTC(gap, e->ego->getSpeed(), 0.);
            }
        } else {
            // encounter is expected to happen without collision
            ttc = INVALID;
        }
    } else if (type == ENCOUNTER_TYPE_CROSSING_LEADER
               || type == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA) {
        if (myComputeDRAC) {
            drac = computeDRAC(eInfo);
        }
        if (eInfo.foeEstimatedConflictEntryTime <= eInfo.egoEstimatedConflictExitTime) {
            // follower's predicted arrival at the crossing area is earlier than the leader's predicted exit -> collision predicted
            double gap = eInfo.foeConflictEntryDist;
            if (myComputeTTC) {
                ttc = computeTTC(gap, e->foe->getSpeed(), 0.);
            }
        } else {
            // encounter is expected to happen without collision
            ttc = INVALID;
        }
    } else {
#ifdef DEBUG_SSM
        std::stringstream ss;
        ss << "'" << type << "'";
        WRITE_WARNING("Underspecified or unknown encounter type in MSDevice_SSM::determineTTCandDRAC(): " + ss.str());
#endif
    }

#ifdef DEBUG_SSM
    std::cout << "ttc=" << (ttc == INVALID ? "INVALID" : toString(ttc)) << ", drac=" << (drac == INVALID ? "INVALID" : toString(drac))
              << std::endl;
#endif
}


double
MSDevice_SSM::computeTTC(double gap, double followerSpeed, double leaderSpeed) const {
    // TODO: in merging situations, the TTC may be lower than the one computed here for following situations
    //  (currently only the cross section corresponding to the target lane's begin is considered)
    // More specifically, the minimum has to be taken from the two if a collision at merge was predicted.
#ifdef DEBUG_SSM
    std::cout << "computeTTC() with gap=" << gap << ", followerSpeed=" << followerSpeed << ", leaderSpeed=" << leaderSpeed
              << std::endl;
#endif
    if (gap <= 0.) {
        return 0.;    // collision already happend
    }
    double dv = followerSpeed - leaderSpeed;
    if (dv <= 0.) {
        return INVALID;    // no collision
    }

    return gap / dv;
}


double
MSDevice_SSM::computeDRAC(double gap, double followerSpeed, double leaderSpeed) {
#ifdef DEBUG_SSM_DRAC
    std::cout << "computeDRAC() with gap=" << gap << ", followerSpeed=" << followerSpeed << ", leaderSpeed=" << leaderSpeed
              << std::endl;
#endif
    if (gap <= 0.) {
        return INVALID;    // collision!
    }
    double dv = followerSpeed - leaderSpeed;
    if (dv <= 0.) {
        return 0.0;    // no need to break
    }
    assert(followerSpeed > 0.);
    return 0.5 * dv * dv / gap; // following Guido et al. (2011)
}

double
MSDevice_SSM::computeDRAC(const EncounterApproachInfo& eInfo) {
    // Introduce concise variable names
    double dEntry1 = eInfo.egoConflictEntryDist;
    double dEntry2 = eInfo.foeConflictEntryDist;
    double dExit1 = eInfo.egoConflictExitDist;
    double dExit2 = eInfo.foeConflictExitDist;
    double v1 = eInfo.encounter->ego->getSpeed();
    double v2 = eInfo.encounter->foe->getSpeed();
    double tEntry1 = eInfo.egoEstimatedConflictEntryTime;
    double tEntry2 = eInfo.foeEstimatedConflictEntryTime;
    double tExit1 = eInfo.egoEstimatedConflictExitTime;
    double tExit2 = eInfo.foeEstimatedConflictExitTime;
#ifdef DEBUG_SSM_DRAC
    std::cout << SIMTIME << "computeDRAC() with"
              << "\ndEntry1=" << dEntry1 << ", dEntry2=" << dEntry2
              << ", dExit1=" << dExit1 << ", dExit2=" << dExit2
              << ",\nv1=" << v1 << ", v2=" << v2
              << "\ntEntry1=" << (tEntry1 == INVALID ? "NA" : toString(tEntry1)) << ", tEntry2=" << (tEntry2 == INVALID ? "NA" : toString(tEntry2))
              << ", tExit1=" << (tExit1 == INVALID ? "NA" : toString(tExit1)) << ", tExit2=" << (tExit2 == INVALID ? "NA" : toString(tExit2))
              << std::endl;
#endif
    if (dExit1 <= 0. || dExit2 <= 0.) {
        // At least one vehicle already left or is not about to enter conflict area at all => no breaking needed.
#ifdef DEBUG_SSM_DRAC
        std::cout << "One already left conflict area -> drac == 0." << std::endl;
#endif
        return 0.;
    }
    if (dEntry1 <= 0. && dEntry2 <= 0.) {
        // collision... (both already entered conflict area but none left)
#ifdef DEBUG_SSM_DRAC
        std::cout << "Both entered conflict area but neither left. -> collision!" << std::endl;
#endif
        return INVALID;
    }

    double drac = std::numeric_limits<double>::max();
    if (dEntry1 > 0.) {
        // vehicle 1 could break
#ifdef DEBUG_SSM_DRAC
        std::cout << "Ego could break..." << std::endl;
#endif
        if (tExit2 != INVALID) {
            // Vehicle 2 is expected to leave conflict area at t2
            drac = MIN2(drac, 2 * (v1 - dEntry1 / tExit2) / tExit2);
#ifdef DEBUG_SSM_DRAC
            std::cout << "  Foe expected to leave in " << tExit2 << "-> Ego needs drac=" << drac << std::endl;
#endif
        } else {
            // Vehicle 2 is expected to stop on conflict area or earlier
            if (tEntry2 != INVALID) {
                // ... on conflict area => veh1 has to stop before entry
                drac = MIN2(drac, computeDRAC(dEntry1, v1, 0));
#ifdef DEBUG_SSM_DRAC
                std::cout << "  Foe is expected stop on conflict area -> Ego needs drac=" << drac << std::endl;
#endif
            } else {
                // ... before conflict area
#ifdef DEBUG_SSM_DRAC
                std::cout << "  Foe is expected stop before conflict area -> no drac computation for ego (will be done for foe if applicable)" << std::endl;
#endif
            }
        }
    }

    if (dEntry2 > 0.) {
        // vehicle 2 could break
#ifdef DEBUG_SSM_DRAC
        std::cout << "Foe could break..." << std::endl;
#endif
        if (tExit1 != INVALID) {
            // Vehicle 1 is expected to leave conflict area at t1
#ifdef DEBUG_SSM_DRAC
            std::cout << "  Ego expected to leave in " << tExit1 << "-> Foe needs drac=" << (2 * (v2 - dEntry2 / tExit1) / tExit1) << std::endl;
#endif
            drac = MIN2(drac, 2 * (v2 - dEntry2 / tExit1) / tExit1);
        } else {
            // Vehicle 1 is expected to stop on conflict area or earlier
            if (tEntry1 != INVALID) {
                // ... on conflict area => veh2 has to stop before entry
#ifdef DEBUG_SSM_DRAC
                std::cout << "  Ego is expected stop on conflict area -> Foe needs drac=" << computeDRAC(dEntry2, v2, 0) << std::endl;
#endif
                drac = MIN2(drac, computeDRAC(dEntry2, v2, 0));
            } else {
                // ... before conflict area
#ifdef DEBUG_SSM_DRAC
                std::cout << "  Ego is expected stop before conflict area -> no drac computation for foe (done for ego if applicable)" << std::endl;
#endif
            }
        }
    }

    return drac > 0 ? drac : INVALID;
}

void
MSDevice_SSM::checkConflictEntryAndExit(EncounterApproachInfo& eInfo) {
    // determine exact entry and exit times
    Encounter* e = eInfo.encounter;

#ifdef DEBUG_SSM
    std::cout << SIMTIME << " checkConflictEntryAndExit() for encounter of vehicles '" << e->egoID << "' and '" << e->foeID << "'" << std::endl;
#endif

    // Distances to conflict area boundaries in previous step
    double prevEgoConflictEntryDist = eInfo.egoConflictEntryDist + e->ego->getLastStepDist();
    double prevFoeConflictEntryDist = eInfo.foeConflictEntryDist + e->foe->getLastStepDist();
    double prevEgoConflictExitDist = prevEgoConflictEntryDist + eInfo.egoConflictAreaLength + e->ego->getLength();
    double prevFoeConflictExitDist = prevFoeConflictEntryDist + eInfo.foeConflictAreaLength + e->foe->getLength();
    EncounterType prevType = e->currentType;

    if (e->timeSpan.size() == 0) {
        // Encounter has just been created, no data points yet.
        // If a vehicle was already beyond conflict area boundary in last step, we set the passing time to the previous time step
        prevEgoConflictEntryDist = MAX2(prevEgoConflictEntryDist, 0.);
        prevFoeConflictEntryDist = MAX2(prevFoeConflictEntryDist, 0.);
        prevEgoConflictExitDist = MAX2(prevEgoConflictExitDist, 0.);
        prevFoeConflictExitDist = MAX2(prevFoeConflictExitDist, 0.);
    }


    // Check if ego entered in last step
    if (e->egoConflictEntryTime == INVALID && eInfo.egoConflictEntryDist < 0 && prevEgoConflictEntryDist >= 0) {
        // ego must have entered the conflict in the last step. Determine exact entry time
        e->egoConflictEntryTime = SIMTIME - TS + MSCFModel::passingTime(-prevEgoConflictEntryDist, 0., -eInfo.egoConflictEntryDist, e->ego->getPreviousSpeed(), e->ego->getSpeed());
#ifdef DEBUG_SSM
        std::cout << "    ego entered conflict area at t=" << e->egoConflictEntryTime << std::endl;
#endif
        // Update encounter type (only done here for entering, the other transitions are done in updatePassedEncounter)
        if (prevType == ENCOUNTER_TYPE_CROSSING_FOLLOWER
                || prevType == ENCOUNTER_TYPE_CROSSING_LEADER) {
            eInfo.type = ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA;
        }
    }

    // Check if foe entered in last step
    if (e->foeConflictEntryTime == INVALID && eInfo.foeConflictEntryDist < 0. && prevFoeConflictEntryDist >= 0) {
        // foe must have entered the conflict in the last step. Determine exact entry time
        e->foeConflictEntryTime = SIMTIME - TS + MSCFModel::passingTime(-prevFoeConflictEntryDist, 0., -eInfo.foeConflictEntryDist, e->foe->getPreviousSpeed(), e->foe->getSpeed());
#ifdef DEBUG_SSM
        std::cout << "    foe entered conflict area at t=" << e->foeConflictEntryTime << std::endl;
#endif
        // Update encounter type (only done here for entering, the other transitions are done in updatePassedEncounter)
        if (prevType == ENCOUNTER_TYPE_CROSSING_FOLLOWER
                || prevType == ENCOUNTER_TYPE_CROSSING_LEADER) {
            eInfo.type = ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA;
        }
    }

    // Check if ego left conflict area
    if (e->egoConflictExitTime == INVALID && eInfo.egoConflictExitDist < 0 && prevEgoConflictExitDist >= 0) {
        // ego must have left the conflict area in the last step. Determine exact exit time
        e->egoConflictExitTime = SIMTIME - TS + MSCFModel::passingTime(-prevEgoConflictExitDist, 0., -eInfo.egoConflictExitDist, e->ego->getPreviousSpeed(), e->ego->getSpeed());
        // Add cross section to calculate PET for foe
//        e->foePETCrossSections.push_back(std::make_pair(eInfo.foeConflictEntryCrossSection, e->egoConflictExitTime));
#ifdef DEBUG_SSM
        std::cout << "    ego left conflict area at t=" << e->egoConflictExitTime << std::endl;
#endif
        // Update encounter type (only done here for entering, the other transitions are done in updatePassedEncounter)
        if (prevType == ENCOUNTER_TYPE_CROSSING_FOLLOWER
                || prevType == ENCOUNTER_TYPE_CROSSING_LEADER) {
            eInfo.type = ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA;
        }
    }

    // Check if foe left conflict area
    if (e->foeConflictExitTime == INVALID && eInfo.foeConflictExitDist < 0 && prevFoeConflictExitDist >= 0) {
        // foe must have left the conflict area in the last step. Determine exact exit time
        e->foeConflictExitTime = SIMTIME - TS + MSCFModel::passingTime(-prevFoeConflictExitDist, 0., -eInfo.foeConflictExitDist, e->foe->getPreviousSpeed(), e->foe->getSpeed());
        // Add cross section to calculate PET for ego
//        e->egoPETCrossSections.push_back(std::make_pair(eInfo.egoConflictEntryCrossSection, e->foeConflictExitTime));
#ifdef DEBUG_SSM
        std::cout << "    foe left conflict area at t=" << e->foeConflictExitTime << std::endl;
#endif
        // Update encounter type (only done here for entering, the other transitions are done in updatePassedEncounter)
        if (prevType == ENCOUNTER_TYPE_CROSSING_FOLLOWER
                || prevType == ENCOUNTER_TYPE_CROSSING_LEADER) {
            eInfo.type = ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA;
        }
    }
}


void
MSDevice_SSM::updatePassedEncounter(Encounter* e, FoeInfo* foeInfo, EncounterApproachInfo& eInfo) {

#ifdef DEBUG_SSM
    std::cout << SIMTIME << " updatePassedEncounter() for vehicles '"
              << e->egoID << "' and '" << e->foeID << "'"
              << std::endl;
#endif

    if (foeInfo == 0) {
        // the foe is out of the device's range, proceed counting down the remaining extra time to trace
        e->countDownExtraTime(TS);
#ifdef DEBUG_SSM
        std::cout << "    Foe is out of range. Counting down extra time."
                  << " Remaining seconds before closing encounter: " << e->getRemainingExtraTime()
                  << std::endl;
#endif

    } else {
        // reset the remaining extra time (foe could have re-entered the device range after beginning extra time countdown already)
        e->resetExtraTime(myExtraTime);
    }

    // Check, whether this was really a potential conflict at some time:
    // Search through typeSpan for a type other than no conflict
    EncounterType lastPotentialConflictType = e->typeSpan.size() > 0 ? static_cast<EncounterType>(e->typeSpan.back()) : ENCOUNTER_TYPE_NOCONFLICT_AHEAD;

    if (lastPotentialConflictType == ENCOUNTER_TYPE_NOCONFLICT_AHEAD) {
        // This encounter was no conflict in the last step -> remains so
#ifdef DEBUG_SSM
        std::cout << "    This encounter wasn't classified as a potential conflict lately." << std::endl;
#endif
        if (foeInfo == 0) {
            // Encounter was either never a potential conflict and foe is out of range
            // or the foe has left the network
            // -> no use in further tracing this encounter
            e->closingRequested = true;
#ifdef DEBUG_SSM
            std::cout << "    Closing encounter." << std::endl;
#endif
            eInfo.type = ENCOUNTER_TYPE_NOCONFLICT_AHEAD;
        }
    } else if (lastPotentialConflictType == ENCOUNTER_TYPE_FOLLOWING_FOLLOWER
               || lastPotentialConflictType == ENCOUNTER_TYPE_FOLLOWING_LEADER
               || lastPotentialConflictType == ENCOUNTER_TYPE_FOLLOWING_PASSED) {
        // if a following situation leads to a no-conflict situation this encounter switches no-conflict, since no further computations (PET) are needed.
        eInfo.type = ENCOUNTER_TYPE_FOLLOWING_PASSED;
#ifdef DEBUG_SSM
        std::cout << "    Encounter was previously classified as a follow/lead situation." << std::endl;
#endif
    } else if (lastPotentialConflictType == ENCOUNTER_TYPE_MERGING_FOLLOWER
               || lastPotentialConflictType == ENCOUNTER_TYPE_MERGING_LEADER
               || lastPotentialConflictType == ENCOUNTER_TYPE_MERGING_PASSED) {
        // if a merging situation leads to a no-conflict situation the leader was either removed from the net (we disregard special treatment)
        // or route- or lane-changes removed the conflict.
        eInfo.type = ENCOUNTER_TYPE_MERGING_PASSED;
#ifdef DEBUG_SSM
        std::cout << "    Encounter was previously classified as a merging situation." << std::endl;
#endif
    }
    if (lastPotentialConflictType == ENCOUNTER_TYPE_CROSSING_FOLLOWER
            || lastPotentialConflictType == ENCOUNTER_TYPE_CROSSING_LEADER
            || lastPotentialConflictType == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA
            || lastPotentialConflictType == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA
            || lastPotentialConflictType == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA
            || lastPotentialConflictType == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
            || lastPotentialConflictType == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
            || lastPotentialConflictType == ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA
            || lastPotentialConflictType == ENCOUNTER_TYPE_COLLISION) {
        // Encounter has been a crossing situation.

#ifdef DEBUG_SSM
        std::cout << "    Encounter was previously classified as a crossing situation of type " << lastPotentialConflictType << "." << std::endl;
#endif
        // For passed encounters, the xxxConflictAreaLength variables are not determined before -> we use the stored values.

        // TODO: This could also more precisely be calculated wrt the angle of the crossing *at the conflict point*
        if (eInfo.egoConflictAreaLength == INVALID) {
            eInfo.egoConflictAreaLength = e->foe->getWidth();
        }
        if (eInfo.foeConflictAreaLength == INVALID) {
            eInfo.foeConflictAreaLength = e->ego->getWidth();
        }

        eInfo.egoConflictEntryDist = e->egoDistsToConflict.back() - e->ego->getLastStepDist();
        eInfo.egoConflictExitDist = eInfo.egoConflictEntryDist + eInfo.egoConflictAreaLength + e->ego->getLength();
        eInfo.foeConflictEntryDist = e->foeDistsToConflict.back() - e->foe->getLastStepDist();
        eInfo.foeConflictExitDist = eInfo.foeConflictEntryDist + eInfo.foeConflictAreaLength + e->foe->getLength();

#ifdef DEBUG_SSM
        std::cout << "    egoConflictEntryDist = " << eInfo.egoConflictEntryDist
                  << ", egoConflictExitDist = " << eInfo.egoConflictExitDist
                  << "\n    foeConflictEntryDist = " << eInfo.foeConflictEntryDist
                  << ", foeConflictExitDist = " << eInfo.foeConflictExitDist
                  << std::endl;
#endif

        // Determine actual encounter type
        bool egoEnteredConflict =  eInfo.egoConflictEntryDist < 0.;
        bool foeEnteredConflict =  eInfo.foeConflictEntryDist < 0.;
        bool egoLeftConflict =  eInfo.egoConflictExitDist < 0.;
        bool foeLeftConflict =  eInfo.foeConflictExitDist < 0.;

        if ((!egoEnteredConflict) && !foeEnteredConflict) {
            // XXX: do we need to recompute the follow/lead order, here?
            assert(lastPotentialConflictType == ENCOUNTER_TYPE_CROSSING_FOLLOWER
                   || lastPotentialConflictType == ENCOUNTER_TYPE_CROSSING_LEADER);
            eInfo.type = lastPotentialConflictType;
        } else if (egoEnteredConflict && !foeEnteredConflict) {
            eInfo.type = ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA;
        } else if ((!egoEnteredConflict) && foeEnteredConflict) {
            eInfo.type = ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA;
        } else { // (egoEnteredConflict && foeEnteredConflict) {
            eInfo.type = ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA;
        }

        if ((!egoLeftConflict) && !foeLeftConflict) {
            if (eInfo.type == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA) {
                eInfo.type = ENCOUNTER_TYPE_COLLISION;
            }
        } else if (egoLeftConflict && !foeLeftConflict) {
            if (eInfo.type != ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA) {
                eInfo.type = ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA;
            }
        } else if ((!egoLeftConflict) && foeLeftConflict) {
            if (eInfo.type != ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA) {
                eInfo.type = ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA;
            }
        } else {
            eInfo.type = ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA;
            // It should not occur that both leave the conflict at the same step
            assert(lastPotentialConflictType == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
                   || lastPotentialConflictType == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
                   || lastPotentialConflictType == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA
                   || lastPotentialConflictType == ENCOUNTER_TYPE_BOTH_LEFT_CONFLICT_AREA);
        }

        // TODO: adjust the conflict distances according to lateral movement for single ENTERED-cases

#ifdef DEBUG_SSM
        std::cout << "    Updated classification: " << eInfo.type << std::endl;
#endif
    }
}


MSDevice_SSM::EncounterType
MSDevice_SSM::classifyEncounter(const FoeInfo* foeInfo, EncounterApproachInfo& eInfo)  const {
#ifdef DEBUG_SSM
    std::cout << "classifyEncounter() called." << std::endl;
#endif
    if (foeInfo == 0) {
        // foeInfo == 0 signalizes, that no corresponding foe info was returned by findSurroundingVehicles(),
        // i.e. the foe is actually out of range (This may also mean that it has left the network)
        return ENCOUNTER_TYPE_NOCONFLICT_AHEAD;
    }
    const Encounter* e = eInfo.encounter;

    // previous classification (if encounter was not just created)
    EncounterType prevType = e->typeSpan.size() > 0 ? static_cast<EncounterType>(e->typeSpan.back()) : ENCOUNTER_TYPE_NOCONFLICT_AHEAD;
    if (e->typeSpan.size() > 0
            && (prevType == ENCOUNTER_TYPE_EGO_ENTERED_CONFLICT_AREA
                ||  prevType == ENCOUNTER_TYPE_FOE_ENTERED_CONFLICT_AREA
                ||  prevType == ENCOUNTER_TYPE_EGO_LEFT_CONFLICT_AREA
                ||  prevType == ENCOUNTER_TYPE_FOE_LEFT_CONFLICT_AREA
                ||  prevType == ENCOUNTER_TYPE_BOTH_ENTERED_CONFLICT_AREA)) {
        // This is an ongoing crossing situation with at least one of the vehicles not
        // having passed the conflict area.
        // -> Merely trace the change of distances to the conflict entry / exit
        // -> Derefer this to updatePassedEncounter, where this is done anyhow.
#ifdef DEBUG_SSM
        std::cout << "    Ongoing crossing conflict will be traced by passedEncounter()." << std::endl;
#endif
        return prevType;
    }


    // Ego's current Lane
    const MSLane* egoLane = e->ego->getLane();
    // Foe's current Lane
    const MSLane* foeLane = e->foe->getLane();

    // Ego's conflict lane is memorized in foeInfo
    const MSLane* egoConflictLane = foeInfo->egoConflictLane;
    double egoDistToConflictLane = foeInfo->egoDistToConflictLane;
    // Find conflicting lane and the distance to its entry link for the foe
    double foeDistToConflictLane;
    const MSLane* foeConflictLane = findFoeConflictLane(e->foe, foeInfo->egoConflictLane, foeDistToConflictLane);

#ifdef DEBUG_SSM
    std::cout << "egoConflictLane: '" << (egoConflictLane == 0 ? "NULL" : egoConflictLane->getID()) << "'\n"
              << "foeConflictLane: '" << (foeConflictLane == 0 ? "NULL" : foeConflictLane->getID()) << "'"
              << "\nEgo's distance to conflict lane: " << egoDistToConflictLane
              << "\nFoe's distance to conflict lane: " << foeDistToConflictLane
              << std::endl;
#endif

    // Treat different cases for foeConflictLane and egoConflictLane (internal or non-internal / equal to egoLane or to foeLane),
    // and thereby determine encounterType and the ego/foeEncounterDistance.
    // The encounter distance has a different meaning for different types of encounters:
    // 1) For rear-end conflicts (lead/follow situations) the follower's encounter distance is the distance to the actual back position of the leader. The leaders's distance is undefined.
    // 2) For merging encounters the encounter distance is the distance until the begin of the common target edge/lane.
    //    (XXX: Perhaps this should be adjusted to include the entry point to the region where a simultaneous occupancy of
    //          both merging lanes could imply a collision)
    // 3) For crossing encounters the encounter distances is the distance until the entry point to the conflicting lane.

    EncounterType type;

    if (foeConflictLane == 0) {
        // foe vehicle is not on course towards the ego's route (see findFoeConflictLane)
        type = ENCOUNTER_TYPE_NOCONFLICT_AHEAD;
#ifdef DEBUG_SSM
        std::cout << "-> Encounter type: No conflict." << std::endl;
#endif
    } else if (!egoConflictLane->isInternal()) {
        // The conflict lane is non-internal, therefore we either have no potential conflict or a lead/follow situation (i.e., no crossing or merging)
        if (egoConflictLane == egoLane) {
            // The conflict point is on the ego's current lane.
            if (foeLane == egoLane) {
                // Foe is on the same non-internal lane
                if (e->ego->getPositionOnLane() > e->foe->getPositionOnLane()) {
                    type = ENCOUNTER_TYPE_FOLLOWING_LEADER;
                    eInfo.foeConflictEntryDist = e->ego->getBackPositionOnLane() - e->foe->getBackPositionOnLane();
                } else {
                    type = ENCOUNTER_TYPE_FOLLOWING_FOLLOWER;
                    eInfo.egoConflictEntryDist = e->foe->getBackPositionOnLane() - e->ego->getBackPositionOnLane();
                }
#ifdef DEBUG_SSM
                std::cout << "-> Encounter type: Lead/follow-situation on non-internal lane '" << egoLane->getID() << "'" << std::endl;
#endif
            } else if (&(foeLane->getEdge()) == &(egoLane->getEdge())) {
                // Foe is on the same non-internal edge but not on the same lane. Treat this as no conflict for now
                // XXX: this disregards conflicts for vehicles on adjacent lanes
                type = ENCOUNTER_TYPE_ON_ADJACENT_LANES;
#ifdef DEBUG_SSM
                std::cout << "-> Encounter type: " << type << std::endl;
#endif
            } else {
                assert(&(egoLane->getEdge()) == &(foeConflictLane->getEdge()));
                assert(egoDistToConflictLane <= 0);
                // Foe must be on a route leading into the ego's edge
                if (foeConflictLane == egoLane) {
                    type = ENCOUNTER_TYPE_FOLLOWING_LEADER;
                    eInfo.foeConflictEntryDist = foeDistToConflictLane + e->ego->getBackPositionOnLane();

#ifdef DEBUG_SSM
                    std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' leads foe '"
                              << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                              << " (gap = " << eInfo.foeConflictEntryDist << ")"
                              << std::endl;
#endif
                } else {
                    // Foe's route leads to an adjacent lane of the current lane of the ego
                    type = ENCOUNTER_TYPE_ON_ADJACENT_LANES;
#ifdef DEBUG_SSM
                    std::cout << "-> Encounter type: " << type << std::endl;
#endif
                }
            }
        } else {
            // The egoConflictLane is a non-internal lane which is not the ego's current lane. Thus it must lie ahead of the ego vehicle and
            // is located on the foe's current edge see findSurroundingVehicles()
            // (otherwise the foe would have had to enter the ego's route along a junction and the corresponding
            // conflict lane would be internal)
            assert(&(foeLane->getEdge()) == &(egoConflictLane->getEdge()));
            assert(foeDistToConflictLane <= 0);
            if (foeLane == egoConflictLane) {
                type = ENCOUNTER_TYPE_FOLLOWING_FOLLOWER;
                eInfo.egoConflictEntryDist = egoDistToConflictLane + e->foe->getBackPositionOnLane();
#ifdef DEBUG_SSM
                std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' follows foe '"
                          << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                          << " (gap = " << eInfo.egoConflictEntryDist << ")"
                          << std::endl;
#endif
            } else {
                // Ego's route leads to an adjacent lane of the current lane of the foe
                type = ENCOUNTER_TYPE_ON_ADJACENT_LANES;
#ifdef DEBUG_SSM
                std::cout << "-> Encounter type: " << type << std::endl;
#endif
            }
        }
    } else {
        // egoConflictLane is internal, i.e., lies on a junction. Besides the lead/follow situation (which may stretch over different lanes of a connection),
        // merging or crossing of the conflict lanes is possible.
        assert(foeConflictLane->isInternal());
        MSLink* egoEntryLink = egoConflictLane->getEntryLink();
        MSLink* foeEntryLink = foeConflictLane->getEntryLink();
        if (&(egoEntryLink->getViaLane()->getEdge()) == &(foeEntryLink->getViaLane()->getEdge())) {
            if (egoEntryLink != foeEntryLink) {
                // XXX: this disregards conflicts for vehicles on adjacent internal lanes
                type = ENCOUNTER_TYPE_ON_ADJACENT_LANES;
#ifdef DEBUG_SSM
                std::cout << "-> Encounter type: " << type << std::endl;
#endif
            } else {
                // Lead / follow situation on connection
                if (egoLane == egoConflictLane && foeLane != foeConflictLane) {
                    // ego on junction, foe not yet
                    type = ENCOUNTER_TYPE_FOLLOWING_LEADER;
                    eInfo.foeConflictEntryDist = foeDistToConflictLane + e->ego->getBackPositionOnLane();
#ifdef DEBUG_SSM
                    std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' leads foe '"
                              << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                              << " (gap = " << eInfo.foeConflictEntryDist << ")"
                              << std::endl;
#endif
                } else if (egoLane != egoConflictLane && foeLane == foeConflictLane) {
                    // foe on junction, ego not yet
                    type = ENCOUNTER_TYPE_FOLLOWING_FOLLOWER;
                    eInfo.egoConflictEntryDist = egoDistToConflictLane + e->foe->getBackPositionOnLane();
#ifdef DEBUG_SSM
                    std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' follows foe '"
                              << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                              << " (gap = " << eInfo.egoConflictEntryDist << ")"
                              << std::endl;
#endif
                } else {
                    // Both must be already on the junction in a lead / follow situation on a connection
                    // (since they approach via the same link, findSurroundingVehicles() would have determined a
                    // different conflictLane if both are not on the junction)
                    assert(egoLane == egoConflictLane);
                    assert(foeLane == foeConflictLane);
                    if (egoLane == foeLane) {
                        // both on the same internal lane
                        if (e->ego->getPositionOnLane() > e->foe->getPositionOnLane()) {
                            type = ENCOUNTER_TYPE_FOLLOWING_LEADER;
                            eInfo.foeConflictEntryDist = foeDistToConflictLane + e->ego->getBackPositionOnLane();
#ifdef DEBUG_SSM
                            std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' leads foe '"
                                      << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                                      << " (gap = " << eInfo.foeConflictEntryDist << ")"
                                      << std::endl;
#endif
                        } else {
                            type = ENCOUNTER_TYPE_FOLLOWING_FOLLOWER;
                            eInfo.egoConflictEntryDist = egoDistToConflictLane + e->foe->getBackPositionOnLane();
#ifdef DEBUG_SSM
                            std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' follows foe '"
                                      << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                                      << " (gap = " << eInfo.egoConflictEntryDist << ")"
                                      << std::endl;
#endif
                        }
                    } else {
                        // ego and foe on distinct, consecutive internal lanes
#ifdef DEBUG_SSM
                        std::cout << "    Lead/follow situation on consecutive internal lanes." << std::endl;
#endif
                        MSLane* lane = egoEntryLink->getViaLane();
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4127) // do not warn about constant conditional expression
#endif
                        while (true) {
#ifdef _MSC_VER
#pragma warning(pop)
#endif
                            // Find first of egoLane and foeLane while crossing the junction (this dertermines who's the follower)
                            // Then set the conflict lane to the lane of the leader and adapt the follower's distance to conflict
                            if (egoLane == lane) {
                                // ego is follower
                                type = ENCOUNTER_TYPE_FOLLOWING_FOLLOWER;
                                // adapt conflict dist
                                eInfo.egoConflictEntryDist = egoDistToConflictLane;
                                while (lane != foeLane) {
                                    eInfo.egoConflictEntryDist += lane->getLength();
                                    lane = lane->getLinkCont()[0]->getViaLane();
                                    assert(lane != 0);
                                }
                                eInfo.egoConflictEntryDist += e->foe->getBackPositionOnLane();
                                egoConflictLane = lane;
#ifdef DEBUG_SSM
                                std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' follows foe '"
                                          << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                                          << " (gap = " << eInfo.egoConflictEntryDist << ")"
                                          << std::endl;
#endif
                                break;
                            } else if (foeLane == lane) {
                                // ego is leader
                                type = ENCOUNTER_TYPE_FOLLOWING_LEADER;
                                // adapt conflict dist
                                eInfo.foeConflictEntryDist = foeDistToConflictLane;
                                while (lane != egoLane) {
                                    eInfo.foeConflictEntryDist += lane->getLength();
                                    lane = lane->getLinkCont()[0]->getViaLane();
                                    assert(lane != 0);
                                }
                                eInfo.foeConflictEntryDist += e->ego->getBackPositionOnLane();
                                foeConflictLane = lane;
#ifdef DEBUG_SSM
                                std::cout << "-> Encounter type: Ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' leads foe '"
                                          << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                                          << " (gap = " << eInfo.foeConflictEntryDist << ")"
                                          << std::endl;
#endif
                                break;
                            }
                            lane = lane->getLinkCont()[0]->getViaLane();
                            assert(lane != 0);
                        }
                    }
#ifdef DEBUG_SSM
                    std::cout << "-> Encounter type: Lead/follow-situation on connection from '" << egoEntryLink->getLaneBefore()->getID()
                              << "' to '" << egoEntryLink->getLane()->getID() << "'" << std::endl;
#endif
                }
            }
        } else {
            // Entry links to junctions lead to different internal edges.
            // There are three possibilities, either the edges cross, merge or have no conflict
            const std::vector<MSLink*>& egoFoeLinks = egoEntryLink->getFoeLinks();
            const std::vector<MSLink*>& foeFoeLinks = foeEntryLink->getFoeLinks();
            // Determine whether ego and foe links are foes
            bool crossOrMerge = (find(egoFoeLinks.begin(), egoFoeLinks.end(), foeEntryLink) != egoFoeLinks.end()
                                 || find(foeFoeLinks.begin(), foeFoeLinks.end(), egoEntryLink) != foeFoeLinks.end());
            if (!crossOrMerge) {
//                if (&(foeEntryLink->getLane()->getEdge()) == &(egoEntryLink->getLane()->getEdge())) {
//                    // XXX: the situation of merging into adjacent lanes is disregarded for now <- the alleged situation appears to imply crossOrMerge!!!
//                    type = ENCOUNTER_TYPE_MERGING_ADJACENT;
//#ifdef DEBUG_SSM
//                    std::cout << "-> Encounter type: No conflict (adjacent lanes)." << std::endl;
//#endif
//                } else {
                type = ENCOUNTER_TYPE_NOCONFLICT_AHEAD;
#ifdef DEBUG_SSM
                std::cout << "-> Encounter type: No conflict." << std::endl;
#endif
//                }
            } else if (&(foeEntryLink->getLane()->getEdge()) == &(egoEntryLink->getLane()->getEdge())) {
                if (foeEntryLink->getLane() == egoEntryLink->getLane()) {
                    type = ENCOUNTER_TYPE_MERGING;
                    eInfo.egoConflictEntryDist = egoDistToConflictLane + egoEntryLink->getInternalLengthsAfter();
                    eInfo.foeConflictEntryDist = foeDistToConflictLane + foeEntryLink->getInternalLengthsAfter();
#ifdef DEBUG_SSM
                    std::cout << "-> Encounter type: Merging situation of ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' and foe '"
                              << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                              << "\nDistances to merge-point: ego: " << eInfo.egoConflictEntryDist << ", foe: " << eInfo.foeConflictEntryDist
                              << std::endl;
#endif
                } else {
                    // Links leading to the same edge but different lanes. XXX: Disregards conflicts on adjacent lanes
                    type = ENCOUNTER_TYPE_MERGING_ADJACENT;
#ifdef DEBUG_SSM
                    std::cout << "-> Encounter type: No conflict: " << type << std::endl;
#endif
                }
            } else {
                type = ENCOUNTER_TYPE_CROSSING;

                assert(egoConflictLane->isInternal());
                assert(foeConflictLane->getEdge().getToJunction() == egoConflictLane->getEdge().getToJunction());

                // If the conflict lanes are internal, they may not correspond to the
                // actually crossing parts of the corresponding connections.
                // Adjust the conflict lanes accordingly.
                // set back both to the first parts of the corresponding connections
                double offset = 0.;
                egoConflictLane = egoConflictLane->getFirstInternalInConnection(offset);
                egoDistToConflictLane -= offset;
                foeConflictLane = foeConflictLane->getFirstInternalInConnection(offset);
                foeDistToConflictLane -= offset;
                // find the distances to the conflict from the junction entry for both vehicles
                // Here we also determine the real crossing lanes (before the conflict lane is the first lane of the connection)
                // for the ego
                double egoDistToConflictFromJunctionEntry = INVALID;
                double foeInternalLaneLengthsBeforeCrossing = 0.;
                while (foeConflictLane != 0 && foeConflictLane->isInternal()) {
                    egoDistToConflictFromJunctionEntry = egoEntryLink->getLengthsBeforeCrossing(foeConflictLane);
                    if (egoDistToConflictFromJunctionEntry != INVALID) {
                        // found correct foeConflictLane
                        egoDistToConflictFromJunctionEntry += 0.5 * (foeConflictLane->getWidth() - e->foe->getVehicleType().getWidth());
                        break;
                    } else {
                        foeInternalLaneLengthsBeforeCrossing += foeConflictLane->getLength();
                    }
                    foeConflictLane = foeConflictLane->getCanonicalSuccessorLane();
                    assert(foeConflictLane != 0 && foeConflictLane->isInternal()); // this loop should be ended by the break! Otherwise the lanes do not cross, which should be the case here.
                }
                assert(egoDistToConflictFromJunctionEntry != INVALID);

                // for the foe
                double foeDistToConflictFromJunctionEntry = INVALID;
                double egoInternalLaneLengthsBeforeCrossing = 0.;
                foeDistToConflictFromJunctionEntry = INVALID;
                while (egoConflictLane != 0 && egoConflictLane->isInternal()) {
                    foeDistToConflictFromJunctionEntry = foeEntryLink->getLengthsBeforeCrossing(egoConflictLane);
                    if (foeDistToConflictFromJunctionEntry != INVALID) {
                        // found correct egoConflictLane
                        foeDistToConflictFromJunctionEntry += 0.5 * (egoConflictLane->getWidth() - e->ego->getVehicleType().getWidth());
                        break;
                    } else {
                        egoInternalLaneLengthsBeforeCrossing += egoConflictLane->getLength();
                    }
                    egoConflictLane = egoConflictLane->getCanonicalSuccessorLane();
                    assert(egoConflictLane != 0 && egoConflictLane->isInternal()); // this loop should be ended by the break! Otherwise the lanes do not cross, which should be the case here.
                }
                assert(foeDistToConflictFromJunctionEntry != INVALID);

                // store conflict entry information in eInfo

//                // TO-DO: equip these with exit times to store relevant PET sections in encounter
//                eInfo.egoConflictEntryCrossSection = std::make_pair(egoConflictLane, egoDistToConflictFromJunctionEntry - egoInternalLaneLengthsBeforeCrossing);
//                eInfo.foeConflictEntryCrossSection = std::make_pair(foeConflictLane, foeDistToConflictFromJunctionEntry - foeInternalLaneLengthsBeforeCrossing);

                // Take into account the lateral position for the exact determination of the conflict point
                // whether lateral position increases or decreases conflict distance depends on lane angles at conflict
                // -> conflictLaneOrientation in {-1,+1}
                // First, measure the angle between the two connection lines (straight lines from junction entry point to junction exit point)
                Position egoEntryPos = egoEntryLink->getViaLane()->getShape().front();
                Position egoExitPos = egoEntryLink->getCorrespondingExitLink()->getInternalLaneBefore()->getShape().back();
                PositionVector egoConnectionLine(egoEntryPos, egoExitPos);
                Position foeEntryPos = foeEntryLink->getViaLane()->getShape().front();
                Position foeExitPos = foeEntryLink->getCorrespondingExitLink()->getInternalLaneBefore()->getShape().back();
                PositionVector foeConnectionLine(foeEntryPos, foeExitPos);
                double angle = std::fmod(egoConnectionLine.rotationAtOffset(0.) - foeConnectionLine.rotationAtOffset(0.), (2 * M_PI));
                if (angle < 0) {
                    angle += 2 * M_PI;
                }
                assert(angle >= 0);
                assert(angle <= 2 * M_PI);
                if (angle > M_PI) {
                    angle -= 2 * M_PI;
                }
                assert(angle >= -M_PI);
                assert(angle <= M_PI);
                // Determine orientation of the connection lines. (Positive values mean that the ego vehicle approaches from the foe's left side.)
                double crossingOrientation = (angle < 0) - (angle > 0);

                // Adjust conflict dist to lateral positions
                // TODO: This could more precisely be calculated wrt the angle of the crossing *at the conflict point*
                egoDistToConflictFromJunctionEntry -= crossingOrientation * e->foe->getLateralPositionOnLane();
                foeDistToConflictFromJunctionEntry += crossingOrientation * e->ego->getLateralPositionOnLane();

                // Complete entry distances
                eInfo.egoConflictEntryDist = egoDistToConflictLane + egoDistToConflictFromJunctionEntry;
                eInfo.foeConflictEntryDist = foeDistToConflictLane + foeDistToConflictFromJunctionEntry;


#ifdef DEBUG_SSM
                std::cout << "    Determined exact conflict distances for crossing conflict."
                          << "\n    crossingOrientation=" << crossingOrientation
                          << ", egoCrossingAngle=" << egoConnectionLine.rotationAtOffset(0.)
                          << ", foeCrossingAngle=" << foeConnectionLine.rotationAtOffset(0.)
                          << ", relativeAngle=" << angle
                          << " (foe from " << (crossingOrientation > 0 ? "right)" : "left)")
                          << "\n    resulting offset for conflict entry distance:"
                          << "\n     ego=" << crossingOrientation* e->foe->getLateralPositionOnLane()
                          << ", foe=" << crossingOrientation* e->ego->getLateralPositionOnLane()
                          << "\n    resulting entry distances:"
                          << "\n     ego=" << eInfo.egoConflictEntryDist
                          << ", foe=" << eInfo.foeConflictEntryDist
                          << std::endl;
#endif

                // TODO: This could also more precisely be calculated wrt the angle of the crossing *at the conflict point*
                eInfo.egoConflictAreaLength = e->foe->getWidth();
                eInfo.foeConflictAreaLength = e->ego->getWidth();

                // resulting exit distances
                eInfo.egoConflictExitDist = eInfo.egoConflictEntryDist + eInfo.egoConflictAreaLength + e->ego->getLength();
                eInfo.foeConflictExitDist = eInfo.foeConflictEntryDist + eInfo.foeConflictAreaLength + e->foe->getLength();

#ifdef DEBUG_SSM
                std::cout << "real egoConflictLane: '" << (egoConflictLane == 0 ? "NULL" : egoConflictLane->getID()) << "'\n"
                          << "real foeConflictLane: '" << (foeConflictLane == 0 ? "NULL" : foeConflictLane->getID()) << "'\n"
                          << "-> Encounter type: Crossing situation of ego '" << e->ego->getID() << "' on lane '" << egoLane->getID() << "' and foe '"
                          << e->foe->getID() << "' on lane '" << foeLane->getID() << "'"
                          << "\nDistances to crossing-point: ego: " << eInfo.egoConflictEntryDist << ", foe: " << eInfo.foeConflictEntryDist
                          << std::endl;
#endif
            }
        }
    }
    return type;
}


const MSLane*
MSDevice_SSM::findFoeConflictLane(const MSVehicle* foe, const MSLane* egoConflictLane, double& distToConflictLane) const {

#ifdef DEBUG_SSM
    std::cout << SIMTIME << " findFoeConflictLane() for foe '"
              << foe->getID() << "' on lane '" << foe->getLane()->getID()
              << "' (with egoConflictLane=" << (egoConflictLane == 0 ? "NULL" : egoConflictLane->getID())
              << ")\nfoeBestLanes: " << toString(foe->getBestLanesContinuation())
              << std::endl;
#endif
    MSLane* foeLane = foe->getLane();
    std::vector<MSLane*>::const_iterator laneIter = foe->getBestLanesContinuation().begin();
    std::vector<MSLane*>::const_iterator foeBestLanesEnd = foe->getBestLanesContinuation().end();
    assert(foeLane->isInternal() || *laneIter == foeLane);
    distToConflictLane = -foe->getPositionOnLane();

    // Potential conflict lies on junction if egoConflictLane is internal
    const MSJunction* conflictJunction = egoConflictLane->isInternal() ? egoConflictLane->getEdge().getToJunction() : 0;
#ifdef DEBUG_SSM
    if (conflictJunction != 0) {
        std::cout << "Potential conflict on junction '" << conflictJunction->getID()
                  << std::endl;
    }
#endif
    if (foeLane->isInternal() && foeLane->getEdge().getToJunction() == conflictJunction) {
        // foe is already on the conflict junction
        return foeLane;
    }

    // Foe is not on the conflict junction

    // Leading internal lanes in bestlanes are resembled as a single NULL-pointer skip them
    if (*laneIter == 0) {
        while (foeLane != 0 && foeLane->isInternal()) {
            distToConflictLane += foeLane->getLength();
            foeLane = foeLane->getLinkCont()[0]->getViaLane();
        }
        ++laneIter;
        assert(laneIter == foeBestLanesEnd || *laneIter != 0);
    }

    // Look for the junction downstream along foeBestLanes
    while (laneIter != foeBestLanesEnd && distToConflictLane <= myRange) {
        // Eventual internal lanes were skipped
        assert(*laneIter == foeLane || foeLane == 0);
        foeLane = *laneIter;
        assert(!foeLane->isInternal());
        if (&foeLane->getEdge() == &egoConflictLane->getEdge()) {
#ifdef DEBUG_SSM
            std::cout << "Found conflict lane for foe: '" << foeLane->getID() << "'" << std::endl;
#endif
            // found the potential conflict edge along foeBestLanes
            return foeLane;
        }
        // No conflict on foeLane
        distToConflictLane += foeLane->getLength();

        // set laneIter to next non internal lane along foeBestLanes
        ++laneIter;
        if (laneIter == foeBestLanesEnd) {
            return 0;
        }
        MSLane* nextNonInternalLane = *laneIter;
        MSLink* link = foeLane->getLinkTo(nextNonInternalLane);
        // Set foeLane to first internal lane on the next junction
        foeLane = link->getViaLane();
        assert(foeLane->isInternal());
        if (foeLane->getEdge().getToJunction() == conflictJunction) {
#ifdef DEBUG_SSM
            std::cout << "Found conflict lane for foe: '" << foeLane->getID() << "'" << std::endl;
#endif
            // found egoConflictLane, resp. the conflict junction, along foeBestLanes
            return foeLane;
        }
        // No conflict on junction
        distToConflictLane += link->getInternalLengthsAfter();
        foeLane = nextNonInternalLane;
    }
    // Didn't find conflicting lane on foeBestLanes within range.
    return 0;
}

void
MSDevice_SSM::flushConflicts(bool flushAll) {
#ifdef DEBUG_SSM
    std::cout << "\n" << SIMTIME << " Device '" << getID() << "' flushConflicts()" << std::endl;
#endif
    while (!myPastConflicts.empty()) {
        Encounter* top = myPastConflicts.top();
        if (flushAll || top->begin <= myOldestActiveEncounterBegin) {
            writeOutConflict(top);
            myPastConflicts.pop();
            delete top;
        } else {
            break;
        }
    }
}

void
MSDevice_SSM::toGeo(Position& x) {
    GeoConvHelper::getFinal().cartesian2geo(x);
}

void
MSDevice_SSM::toGeo(PositionVector& xv) {
    for (Position& x : xv) {
        toGeo(x);
    }
}

void
MSDevice_SSM::writeOutConflict(Encounter* e) {
#ifdef DEBUG_SSM
    std::cout << SIMTIME << " writeOutConflict() of vehicles '"
              << e->egoID << "' and '" << e->foeID
              << "'\ntoGeo=" << myUseGeoCoords << std::endl;
#endif
    myOutputFile->openTag("conflict");
    myOutputFile->writeAttr("begin", e->begin).writeAttr("end", e->end);
    myOutputFile->writeAttr("ego", e->egoID).writeAttr("foe", e->foeID);

    if (mySaveTrajectories) {
        myOutputFile->openTag("timeSpan").writeAttr("values", e->timeSpan).closeTag();
        myOutputFile->openTag("typeSpan").writeAttr("values", e->typeSpan).closeTag();

        // Some useful snippets for that (from MSFCDExport.cpp):
        if (myUseGeoCoords) {
            toGeo(e->egoTrajectory.x);
            toGeo(e->foeTrajectory.x);
            toGeo(e->conflictPointSpan);
        }

        myOutputFile->openTag("egoPosition").writeAttr("values", toString(e->egoTrajectory.x, myUseGeoCoords ? gPrecisionGeo : gPrecision)).closeTag();
        myOutputFile->openTag("egoVelocity").writeAttr("values", toString(e->egoTrajectory.v)).closeTag();

        myOutputFile->openTag("foePosition").writeAttr("values", toString(e->foeTrajectory.x, myUseGeoCoords ? gPrecisionGeo : gPrecision)).closeTag();
        myOutputFile->openTag("foeVelocity").writeAttr("values", toString(e->foeTrajectory.v)).closeTag();

        myOutputFile->openTag("conflictPoint").writeAttr("values", toString(e->conflictPointSpan, myUseGeoCoords ? gPrecisionGeo : gPrecision)).closeTag();
    }

    if (myComputeTTC) {
        if (mySaveTrajectories) {
            myOutputFile->openTag("TTCSpan").writeAttr("values", makeStringWithNAs(e->TTCspan, INVALID)).closeTag();
        }
        if (e->minTTC.time == INVALID) {
            myOutputFile->openTag("minTTC").writeAttr("time", "NA").writeAttr("position", "NA").writeAttr("type", "NA").writeAttr("value", "NA").closeTag();
        } else {
            std::string time = toString(e->minTTC.time);
            std::string type = toString(int(e->minTTC.type));
            std::string value = toString(e->minTTC.value);
            if (myUseGeoCoords) {
                toGeo(e->minTTC.pos);
            }
            std::string position = toString(e->minTTC.pos, myUseGeoCoords ? gPrecisionGeo : gPrecision);
            myOutputFile->openTag("minTTC").writeAttr("time", time).writeAttr("position", position).writeAttr("type", type).writeAttr("value", value).closeTag();
        }
    }
    if (myComputeDRAC) {
        if (mySaveTrajectories) {
            myOutputFile->openTag("DRACSpan").writeAttr("values", makeStringWithNAs(e->DRACspan, {0.0, INVALID})).closeTag();
        }
        if (e->maxDRAC.time == INVALID) {
            myOutputFile->openTag("maxDRAC").writeAttr("time", "NA").writeAttr("position", "NA").writeAttr("type", "NA").writeAttr("value", "NA").closeTag();
        } else {
            std::string time = toString(e->maxDRAC.time);
            std::string type = toString(int(e->maxDRAC.type));
            std::string value = toString(e->maxDRAC.value);
            if (myUseGeoCoords) {
                toGeo(e->maxDRAC.pos);
            }
            std::string position = toString(e->maxDRAC.pos, myUseGeoCoords ? gPrecisionGeo : gPrecision);
            myOutputFile->openTag("maxDRAC").writeAttr("time", time).writeAttr("position", position).writeAttr("type", type).writeAttr("value", value).closeTag();
        }
    }
    if (myComputePET) {
        if (e->PET.time == INVALID) {
            myOutputFile->openTag("PET").writeAttr("time", "NA").writeAttr("position", "NA").writeAttr("type", "NA").writeAttr("value", "NA").closeTag();
        } else {
            std::string time = toString(e->PET.time);
            std::string type = toString(int(e->PET.type));
            std::string value = toString(e->PET.value);
            if (myUseGeoCoords) {
                toGeo(e->PET.pos);
            }
            std::string position = toString(e->PET.pos, myUseGeoCoords ? gPrecisionGeo : gPrecision);
            myOutputFile->openTag("PET").writeAttr("time", time).writeAttr("position", position).writeAttr("type", type).writeAttr("value", value).closeTag();
        }
    }
    myOutputFile->closeTag();
}

std::string
MSDevice_SSM::makeStringWithNAs(std::vector<double> v, double NA, std::string sep) {
    std::string res = "";
    for (std::vector<double>::const_iterator i = v.begin(); i != v.end(); ++i) {
        res += (i == v.begin() ? "" : sep) + (*i == NA ? "NA" : toString(*i));
    }
    return res;
}

std::string
MSDevice_SSM::makeStringWithNAs(std::vector<double> v, std::vector<double> NAs, std::string sep) {
    std::string res = "";
    for (std::vector<double>::const_iterator i = v.begin(); i != v.end(); ++i) {
        res += (i == v.begin() ? "" : sep) + (find(NAs.begin(), NAs.end(), *i) != NAs.end() ? "NA" : toString(*i));
    }
    return res;
}


// ---------------------------------------------------------------------------
// MSDevice_SSM-methods
// ---------------------------------------------------------------------------
MSDevice_SSM::MSDevice_SSM(SUMOVehicle& holder, const std::string& id, std::string outputFilename, std::map<std::string, double> thresholds,
                           bool trajectories, double range, double extraTime, bool useGeoCoords) :
    MSDevice(holder, id),
    myThresholds(thresholds),
    mySaveTrajectories(trajectories),
    myRange(range),
    myExtraTime(extraTime),
    myUseGeoCoords(useGeoCoords),
    myOldestActiveEncounterBegin(INVALID) {
    // Take care! Holder is currently being constructed. Cast occurs before completion.
    myHolderMS = static_cast<MSVehicle*>(&holder);

    myComputeTTC = myThresholds.find("TTC") != myThresholds.end();
    myComputeDRAC = myThresholds.find("DRAC") != myThresholds.end();
    myComputePET = myThresholds.find("PET") != myThresholds.end();

    myActiveEncounters = EncounterVector();
    myPastConflicts = EncounterQueue();

    // XXX: Who deletes the OutputDevice?
    myOutputFile = &OutputDevice::getDevice(outputFilename);
//    TODO: make xsd, include header
//    myOutputFile.writeXMLHeader("SSMLog", "SSMLog.xsd");
    myOutputFile->openTag("SSMLog");

    // register at static instance container
    instances->insert(this);

#ifdef DEBUG_SSM
    std::vector<std::string> measures;
    std::vector<double> threshVals;
    for (std::map<std::string, double>::const_iterator i = myThresholds.begin(); i != myThresholds.end(); ++i) {
        measures.push_back(i->first);
        threshVals.push_back(i->second);
    }
    std::cout << "Initialized ssm device '" << id << "' with "
              << "myMeasures=" << joinToString(measures, " ")
              << ", myThresholds=" << joinToString(threshVals, " ")
              << ", mySaveTrajectories=" << mySaveTrajectories
              << ", myRange=" << myRange << ", output file=" << outputFilename << ", extra time=" << myExtraTime << ", useGeo=" << myUseGeoCoords << "\n";
#endif
}

/// @brief Destructor.
MSDevice_SSM::~MSDevice_SSM() {
    // XXX: Who deletes this device?
    // unregister from static instance container
    instances->erase((MSDevice*) this);
    resetEncounters();
    flushConflicts(true);
    myOutputFile->closeTag();
}


bool
MSDevice_SSM::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
#ifdef DEBUG_SSM_NOTIFICATIONS
    std::cout << "device '" << getID() << "' notifyEnter: reason=" << reason << " currentEdge=" << veh.getLane()->getEdge().getID() << "\n";
#else
    UNUSED_PARAMETER(veh);
    UNUSED_PARAMETER(reason);
#endif
    return true; // keep the device
}

bool
MSDevice_SSM::notifyLeave(SUMOVehicle& veh, double /*lastPos*/,
                          MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
#ifdef DEBUG_SSM_NOTIFICATIONS
    std::cout << "device '" << getID() << "' notifyLeave: reason=" << reason << " currentEdge=" << veh.getLane()->getEdge().getID() << "\n";
#else
    UNUSED_PARAMETER(veh);
    UNUSED_PARAMETER(reason);
#endif
    return true; // keep the device
}

bool
MSDevice_SSM::notifyMove(SUMOVehicle& /* veh */, double /* oldPos */,
                         double /* newPos */, double newSpeed) {
#ifdef DEBUG_SSM_NOTIFICATIONS
    std::cout << "device '" << getID() << "' notifyMove: newSpeed=" << newSpeed << "\n";
#else
    UNUSED_PARAMETER(newSpeed);
#endif
    return true; // keep the device
}


void
MSDevice_SSM::findSurroundingVehicles(const MSVehicle& veh, double range, FoeInfoMap& foeCollector) {
#ifdef DEBUG_SSM_SURROUNDING
    std::cout << SIMTIME << " Looking for surrounding vehicles for ego vehicle '" << veh.getID()
              << "' on edge '" << veh.getLane()->getEdge().getID()
              << "'."
              << "\nVehicle's best lanes = " << toString(veh.getBestLanesContinuation())
              << std::endl;
#endif

    if (!veh.isOnRoad()) {
        return;
    }

    // The requesting vehicle's current route
    // XXX: Restriction to route scanning may have to be generalized to scanning of possible continuations when
    //      considering situations involving sudden route changes. See also the definition of the EncounterTypes.
    //      A second problem is that following situations on deviating routes may result in closing encounters
    //      too early if a leading foe is not traced on its new lane. (see test 'foe_leader_deviating_routes')

    // If veh is on an internal edge, the edgeIter points towards the last edge before the junction
    ConstMSEdgeVector::const_iterator edgeIter = veh.getCurrentRouteEdge();
    assert(*edgeIter != 0);

    // Best continuation lanes for the ego vehicle
    const std::vector<MSLane*> egoBestLanes = veh.getBestLanesContinuation();
    std::vector<MSLane*>::const_iterator laneIter = egoBestLanes.begin();

    // current lane in loop below
    const MSLane* lane = veh.getLane();
    assert(lane->isInternal() || lane == *laneIter);
    assert(lane != 0);
    // next non-internal lane on the route
    const MSLane* nextNonInternalLane = 0;

    const MSEdge* edge; // current edge in loop below

    // Init pos with vehicle's current position. Below pos is set to zero to denote
    // the beginning position of the currently considered edge
    double pos = veh.getPositionOnLane();
    // remainingRange is the range minus the distance that is already scanned downstream along the vehicles route
    double remainingDownstreamRange = range;
    // distToConflictLane is the distance of the ego vehicle to the start of the currently considered potential conflict lane (can be negative for its current lane)
    double distToConflictLane = -pos;
    // junctions that were already scanned (break search in recurrent nets)
    std::set<const MSJunction*> seenJunctions;

    // if the current edge is internal, collect all vehicles from the junction and below range upstream (except on the vehicles own edge),
    // this is analogous to the code treating junctions in the loop below. Note that the distance on the junction itself is not included into
    // range, so vehicles farther away than range can be collected, too.
    if (lane->isInternal()) {
        edge = &(lane->getEdge());

#ifdef DEBUG_SSM_SURROUNDING
        std::cout << SIMTIME << " Vehicle '" << veh.getID() << "' is on internal edge " << edge->getID() << "'.\n"
                  << "Previous edge of its route: '" << (*edgeIter)->getID() << "'" << std::endl;
#endif

        assert(edge->getToJunction() == edge->getFromJunction());

        const MSJunction* junction = edge->getToJunction();
        // Collect vehicles on the junction
        getVehiclesOnJunction(junction, distToConflictLane, lane, foeCollector);
        seenJunctions.insert(junction);

        // Collect vehicles on incoming edges.
        // Note that this includes the previous edge on the ego vehicle's route.
        // (The distance on the current internal edge is ignored)
        const ConstMSEdgeVector& incoming = junction->getIncoming();
        for (ConstMSEdgeVector::const_iterator ei = incoming.begin(); ei != incoming.end(); ++ei) {
            if ((*ei)->isInternal()) {
                continue;
            }
            // Upstream range is taken from the vehicle's back
            getUpstreamVehicles(*ei, (*ei)->getLength(), range + veh.getLength(), distToConflictLane, lane, foeCollector, seenJunctions);
        }

//        // Take into account internal distance covered on the current lane
//        (commented out, because upstream scanning disregards internal lanes on the last scanned junction
//        -- this makes the scanning symmetric between leader and follower)
//        remainingDownstreamRange -= lane->getLength() - pos;

        // Take into account non-internal lengths until next non-internal lane
        MSLink* link = lane->getLinkCont()[0];
        remainingDownstreamRange -= link->getInternalLengthsAfter();
        distToConflictLane += lane->getLength() + link->getInternalLengthsAfter();

        // The next non-internal lane
        pos = 0.;
        lane = *(++laneIter);
        edge = &lane->getEdge();
    } else {
        // Collect all vehicles in range behind ego vehicle
        edge = &(lane->getEdge());
        getUpstreamVehicles(edge, pos, range + veh.getLength(), distToConflictLane, lane, foeCollector, seenJunctions);
    }

    assert(lane != 0);
    assert(!lane->isInternal());

    // Advance downstream the ego vehicle's route for distance 'range'.
    // Collect all vehicles on the traversed Edges and on incoming edges at junctions.
    while (remainingDownstreamRange > 0.) {
#ifdef DEBUG_SSM_SURROUNDING
        std::cout << SIMTIME << " Scanning downstream for vehicle '" << veh.getID() << "' on lane '" << veh.getLane()->getID() << "', position=" << pos << ".\n"
                  << "Considering edge '" << edge->getID() << "' Remaining downstream range = " << remainingDownstreamRange
                  << "\nbestLanes=" << toString(egoBestLanes) << "\n"
                  << std::endl;
#endif
        assert(!edge->isInternal());
        assert(!lane->isInternal());
        assert(pos == 0 || lane == veh.getLane());
        if (pos + remainingDownstreamRange < lane->getLength()) {
            // scan range ends on this lane
            getUpstreamVehicles(edge, pos + remainingDownstreamRange, remainingDownstreamRange, distToConflictLane, lane, foeCollector, seenJunctions);
            // scanned required downstream range
            break;
        } else {
            // Also need to scan area that reaches beyond the lane
            // Collecting vehicles on non-internal edge ahead
            getUpstreamVehicles(edge, edge->getLength(), edge->getLength() - pos, distToConflictLane, lane, foeCollector, seenJunctions);
            // account for scanned distance on lane
            remainingDownstreamRange -= lane->getLength() - pos;
            distToConflictLane += lane->getLength();
            pos = 0.;

            // proceed to next non-internal lane
            ++laneIter;
            assert(laneIter == egoBestLanes.end() || *laneIter != 0);

            // If the vehicle's best lanes go on, collect vehicles on the upcoming junction
            if (laneIter != egoBestLanes.end()) {
                // Upcoming junction
                const MSJunction* junction = lane->getEdge().getToJunction();

                // Find connection for ego on the junction
                nextNonInternalLane = *laneIter;
                MSLink* link = lane->getLinkTo(nextNonInternalLane);
                assert(link != 0);
                // First lane of the connection
                lane = link->getViaLane();
                assert(lane != 0);

                if (seenJunctions.count(junction) == 0) {
                    // Collect vehicles on the junction, if it wasn't considered already
                    getVehiclesOnJunction(junction, distToConflictLane, lane, foeCollector);
                    seenJunctions.insert(junction);
                    // Collect vehicles on incoming edges (except the last edge, where we already collected). Use full range.
                    const ConstMSEdgeVector& incoming = junction->getIncoming();
                    for (ConstMSEdgeVector::const_iterator ei = incoming.begin(); ei != incoming.end(); ++ei) {
                        if (*ei == edge || (*ei)->isInternal()) {
                            continue;
                        }
                        getUpstreamVehicles(*ei, (*ei)->getLength(), range, distToConflictLane, lane, foeCollector, seenJunctions);
                    }
                    // account for scanned distance on junction
                    double linkLength = link->getInternalLengthsAfter();
                    remainingDownstreamRange -= linkLength;
                    distToConflictLane += linkLength;
#ifdef DEBUG_SSM_SURROUNDING
                    std::cout << "    Downstream Scan for vehicle '" << veh.getID() << "' proceeded over junction '" << junction->getID()
                              << "',\n    linkLength=" << linkLength << ", remainingDownstreamRange=" << remainingDownstreamRange
                              << std::endl;
#endif

                    // update ego's lane to next non internal edge
                    lane = nextNonInternalLane;
                    edge = &(lane->getEdge());
                } else {
#ifdef DEBUG_SSM_SURROUNDING
                    std::cout << "    Downstream Scan for vehicle '" << veh.getID() << "' stops at junction '" << junction->getID()
                              << "', which has already been scanned."
                              << std::endl;
#endif
                    break;
                }
            } else {
                // Further vehicle path unknown, break search
                break;
            }
        }
    }
    // remove ego vehicle
    foeCollector.erase(&veh);
}

void
MSDevice_SSM::getUpstreamVehicles(const MSEdge* edge, double pos, double range, double egoDistToConflictLane, const MSLane* const egoConflictLane, FoeInfoMap& foeCollector, std::set<const MSJunction*> seenJunctions) {
#ifdef DEBUG_SSM_SURROUNDING
    std::cout << SIMTIME << " getUpstreamVehicles() for edge '" << edge->getID() << "'"
              << " pos = " << pos << " range = " << range
              << "\nFound vehicles:"
              << std::endl;
#endif
    if (range <= 0) {
        return;
    }

    const std::vector<MSLane*>& lanes = edge->getLanes();
    // Collect vehicles on the given edge with position in [pos-range,pos]
    for (std::vector<MSLane*>::const_iterator li = lanes.begin(); li != lanes.end(); ++li) {
        MSLane* lane = *li;
        const MSLane::VehCont& vehicles = lane->getVehiclesSecure();
        for (MSLane::VehCont::const_iterator vi = vehicles.begin(); vi != vehicles.end(); ++vi) {
            MSVehicle* veh = *vi;
            if (veh->getPositionOnLane() - veh->getLength() <= pos && veh->getPositionOnLane() >= pos - range) {
#ifdef DEBUG_SSM
                std::cout << veh->getID()  << "\n";
#endif
                FoeInfo* c = new FoeInfo(); // c is deleted in updateEncounter()
                c->egoDistToConflictLane = egoDistToConflictLane;
                c->egoConflictLane = egoConflictLane;
                foeCollector[veh] = c;
            }
        }
        lane->releaseVehicles();
    }

#ifdef DEBUG_SSM
    std::cout << std::endl;
#endif

    // TODO: Gather vehicles from opposite direction. This should happen in any case, where opposite direction overtaking is possible.
    //       If it isn't it might still be nicer to trace oncoming vehicles for the resulting trajectories in the encounters
    //    if (edge->hasOpposite...)

    if (range <= pos) {
        return;
    }

    // Here we have: range > pos, i.e. we proceed collecting vehicles on preceding edges
    range -= pos;

    // Junction representing the origin of 'edge'
    const MSJunction* junction = edge->getFromJunction();
    if (seenJunctions.count(junction) == 0) {
        // Collect vehicles from incoming edges of the junction
        if (!edge->isInternal()) {
            // collect vehicles on preceding junction (for internal edges this is already done in caller,
            // i.e. findSurroundingVehicles() or the recursive call from getUpstreamVehicles())

            // Collect vehicles on the junction, if it wasn't considered already
            getVehiclesOnJunction(junction, egoDistToConflictLane, egoConflictLane, foeCollector);
            seenJunctions.insert(junction);
        }
        // Collect vehicles from incoming edges from the junction representing the origin of 'edge'
        const ConstMSEdgeVector& incoming = junction->getIncoming();
        for (ConstMSEdgeVector::const_iterator ei = incoming.begin(); ei != incoming.end(); ++ei) {
            if ((*ei)->isInternal()) {
                continue;
            }
            const MSEdge* inEdge = *ei;
            assert(inEdge != 0);
            double distOnJunction = edge->isInternal() ? 0. : inEdge->getInternalFollowingLengthTo(edge);
            if (distOnJunction >= range) {
                continue;
            }
            // account for vehicles on the predecessor edge
            getUpstreamVehicles(inEdge, inEdge->getLength(), range - distOnJunction, egoDistToConflictLane, egoConflictLane, foeCollector, seenJunctions);
        }
    } else {
#ifdef DEBUG_SSM_SURROUNDING
        std::cout << "    Downstream Scan for stops at junction '" << junction->getID()
                  << "', which has already been scanned."
                  << std::endl;
#endif
    }
}

void
MSDevice_SSM::getVehiclesOnJunction(const MSJunction* junction, double egoDistToConflictLane, const MSLane* const egoConflictLane, FoeInfoMap& foeCollector) {
#ifdef DEBUG_SSM_SURROUNDING
    std::cout << SIMTIME << " getVehiclesOnJunction() for junction '" << junction->getID() << "'"
              << "\nFound vehicles:"
              << std::endl;
#endif
    // Collect vehicles on internal lanes
    const std::vector<MSLane*> lanes = junction->getInternalLanes();
    for (std::vector<MSLane*>::const_iterator li = lanes.begin(); li != lanes.end(); ++li) {
        MSLane* lane = *li;
        const MSLane::VehCont& vehicles = lane->getVehiclesSecure();

        // Add FoeInfos (XXX: for some situations, a vehicle may be collected twice. Then the later finding overwrites the earlier in foeCollector.
        // This could lead to neglecting a conflict when determining foeConflictLane later.) -> TODO: test with twice intersecting routes
        for (MSLane::VehCont::const_iterator vi = vehicles.begin(); vi != vehicles.end(); ++vi) {
            FoeInfo* c = new FoeInfo();
            c->egoConflictLane = egoConflictLane;
            c->egoDistToConflictLane = egoDistToConflictLane;
            foeCollector[*vi] = c;
#ifdef DEBUG_SSM_SURROUNDING
            for (MSLane::VehCont::const_iterator vi = vehicles.begin(); vi != vehicles.end(); ++vi) {
                std::cout << (*vi)->getID() << "\n";
            }
#endif
        }
        lane->releaseVehicles();

        // If there is an internal continuation lane, also collect vehicles on that lane
        if (lane->getLinkCont().size() > 1 && lane->getLinkCont()[0]->getViaLane() != 0) {
            // There's a second internal lane of the connection
            lane = lane->getLinkCont()[0]->getViaLane();
            // This code must be modified, if more than two-piece internal lanes are allowed. Thus, assert:
            assert(lane->getLinkCont().size() == 0 || lane->getLinkCont()[0]->getViaLane() == 0);

            // collect vehicles
            const MSLane::VehCont& vehicles2 = lane->getVehiclesSecure();
            // Add FoeInfos. This duplicates the loop for the first internal lane
            for (MSLane::VehCont::const_iterator vi = vehicles2.begin(); vi != vehicles2.end(); ++vi) {
                FoeInfo* c = new FoeInfo();
                c->egoConflictLane = egoConflictLane;
                c->egoDistToConflictLane = egoDistToConflictLane;
                foeCollector[*vi] = c;
#ifdef DEBUG_SSM_SURROUNDING
                for (MSLane::VehCont::const_iterator vi = vehicles2.begin(); vi != vehicles2.end(); ++vi) {
                    std::cout << (*vi)->getID() << "\n";
                }
#endif
            }
            lane->releaseVehicles();
        }
    }

#ifdef DEBUG_SSM_SURROUNDING
    std::cout << std::endl;
#endif
}



void
MSDevice_SSM::generateOutput() const {
    // This is called once at vehicle removal.
    //       Also: flush myOutputFile? Or is this done automatically?
    // myOutputFile->closeTag();
}

// ---------------------------------------------------------------------------
// Static parameter load helpers
// ---------------------------------------------------------------------------
std::string
MSDevice_SSM::getOutputFilename(const SUMOVehicle& v, std::string deviceID) {
    std::string file = deviceID + ".xml";
    if (v.getParameter().knowsParameter("device.ssm.file")) {
        try {
            file = v.getParameter().getParameter("device.ssm.file", file);
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("device.ssm.file", file) + "'for vehicle parameter 'ssm.measures'");
        }
    } else if (v.getVehicleType().getParameter().knowsParameter("device.ssm.file")) {
        try {
            file = v.getVehicleType().getParameter().getParameter("device.ssm.file", file);
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.ssm.file", file) + "'for vType parameter 'ssm.measures'");
        }
    } else {
        std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'device.ssm.file'. Using default of '" << file << "'\n";
    }
    return file;
}

bool
MSDevice_SSM::useGeoCoords(const SUMOVehicle& v, std::string deviceID) {
    OptionsCont& oc = OptionsCont::getOptions();
    bool useGeo = false;
    if (v.getParameter().knowsParameter("device.ssm.geo")) {
        try {
            useGeo = TplConvert::_2bool(v.getParameter().getParameter("device.ssm.geo", "no").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("device.ssm.geo", "no") + "'for vehicle parameter 'ssm.geo'");
        }
    } else if (v.getVehicleType().getParameter().knowsParameter("device.ssm.geo")) {
        try {
            useGeo = TplConvert::_2bool(v.getVehicleType().getParameter().getParameter("device.ssm.geo", "no").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.ssm.geo", "no") + "'for vType parameter 'ssm.geo'");
        }
    } else {
        useGeo = oc.getBool("device.ssm.geo");
#ifdef DEBUG_SSM
        std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'device.ssm.geo'. Using default of '" << useGeo << "'\n";
#endif
    }
    return useGeo;
}


double
MSDevice_SSM::getDetectionRange(const SUMOVehicle& v) {
    OptionsCont& oc = OptionsCont::getOptions();
    double range = -INVALID;
    if (v.getParameter().knowsParameter("device.ssm.range")) {
        try {
            range = TplConvert::_2double(v.getParameter().getParameter("device.ssm.range", "").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("device.ssm.range", "") + "'for vehicle parameter 'ssm.range'");
        }
    } else if (v.getVehicleType().getParameter().knowsParameter("device.ssm.range")) {
        try {
            range = TplConvert::_2double(v.getVehicleType().getParameter().getParameter("device.ssm.range", "").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.ssm.range", "") + "'for vType parameter 'ssm.range'");
        }
    } else {
        range = oc.getFloat("device.ssm.range");
#ifdef DEBUG_SSM
        std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'device.ssm.range'. Using default of '" << range << "'\n";
#endif
    }
    return range;
}


double
MSDevice_SSM::getExtraTime(const SUMOVehicle& v) {
    OptionsCont& oc = OptionsCont::getOptions();
    double extraTime = INVALID;
    if (v.getParameter().knowsParameter("device.ssm.extratime")) {
        try {
            extraTime = TplConvert::_2double(v.getParameter().getParameter("device.ssm.extratime", "").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("device.ssm.extratime", "") + "'for vehicle parameter 'ssm.extratime'");
        }
    } else if (v.getVehicleType().getParameter().knowsParameter("device.ssm.extratime")) {
        try {
            extraTime = TplConvert::_2double(v.getVehicleType().getParameter().getParameter("device.ssm.extratime", "").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.ssm.extratime", "") + "'for vType parameter 'ssm.extratime'");
        }
    } else {
        extraTime = oc.getFloat("device.ssm.extratime");
#ifdef DEBUG_SSM
        std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'device.ssm.extratime'. Using default of '" << extraTime << "'\n";
#endif
    }
    if (extraTime < 0.) {
        extraTime = DEFAULT_EXTRA_TIME;
        WRITE_WARNING("Negative (or no) value encountered for vehicle parameter 'device.ssm.extratime' in vehicle '" + v.getID() + "' using default value " + toString(extraTime) + " instead");
    }
    return extraTime;
}


bool
MSDevice_SSM::requestsTrajectories(const SUMOVehicle& v) {
    OptionsCont& oc = OptionsCont::getOptions();
    bool trajectories = false;
    if (v.getParameter().knowsParameter("device.ssm.trajectories")) {
        try {
            trajectories = TplConvert::_2bool(v.getParameter().getParameter("device.ssm.trajectories", "no").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("device.ssm.trajectories", "no") + "'for vehicle parameter 'ssm.trajectories'");
        }
    } else if (v.getVehicleType().getParameter().knowsParameter("device.ssm.trajectories")) {
        try {
            trajectories = TplConvert::_2bool(v.getVehicleType().getParameter().getParameter("device.ssm.trajectories", "no").c_str());
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.ssm.trajectories", "no") + "'for vType parameter 'ssm.trajectories'");
        }
    } else {
        trajectories = oc.getBool("device.ssm.trajectories");
#ifdef DEBUG_SSM
        std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'device.ssm.trajectories'. Using default of '" << trajectories << "'\n";
#endif
    }
    return trajectories;
}


bool
MSDevice_SSM::getMeasuresAndThresholds(const SUMOVehicle& v, std::string deviceID, std::map<std::string, double>& thresholds) {
    OptionsCont& oc = OptionsCont::getOptions();

    // Measures
    std::string measures_str = "";
    if (v.getParameter().knowsParameter("device.ssm.measures")) {
        try {
            measures_str = v.getParameter().getParameter("device.ssm.measures", "");
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("device.ssm.measures", "") + "'for vehicle parameter 'ssm.measures'");
        }
    } else if (v.getVehicleType().getParameter().knowsParameter("device.ssm.measures")) {
        try {
            measures_str = v.getVehicleType().getParameter().getParameter("device.ssm.measures", "");
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.ssm.measures", "") + "'for vType parameter 'ssm.measures'");
        }
    } else {
        measures_str = oc.getString("device.ssm.measures");
        std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'device.ssm.measures'. Using default of '" << measures_str << "'\n";
    }

    // Check retrieved measures
    if (measures_str == "") {
        WRITE_WARNING("No measures specified for ssm device of vehicle '" + v.getID() + "'. Registering all available SSMs.");
        measures_str = AVAILABLE_SSMS;
    }
    StringTokenizer st = StringTokenizer(AVAILABLE_SSMS);
    std::vector<std::string> available = st.getVector();
    st = StringTokenizer(measures_str);
    std::vector<std::string> measures = st.getVector();
    for (std::vector<std::string>::const_iterator i = measures.begin(); i != measures.end(); ++i) {
        if (std::find(available.begin(), available.end(), *i) == available.end()) {
            // Given identifier is unknown
            WRITE_ERROR("SSM identifier '" + *i + "' is not supported. Aborting construction of SSM device '" + deviceID + "'.");
            return false;
        }
    }

    // Thresholds
    std::string thresholds_str = "";
    if (v.getParameter().knowsParameter("device.ssm.thresholds")) {
        try {
            thresholds_str = v.getParameter().getParameter("device.ssm.thresholds", "");
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("device.ssm.thresholds", "") + "'for vehicle parameter 'ssm.thresholds'");
        }
    } else if (v.getVehicleType().getParameter().knowsParameter("device.ssm.thresholds")) {
        try {
            thresholds_str = v.getVehicleType().getParameter().getParameter("device.ssm.thresholds", "");
        } catch (...) {
            WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("device.ssm.thresholds", "") + "'for vType parameter 'ssm.thresholds'");
        }
    } else {
        thresholds_str = oc.getString("device.ssm.thresholds");
#ifdef DEBUG_SSM
        std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'device.ssm.thresholds'. Using default of '" << thresholds_str << "'\n";
#endif
    }

    // Parse vector of doubles from threshold_str
    int count = 0;
    if (thresholds_str != "") {
        st = StringTokenizer(thresholds_str);
        while (count < (int)measures.size() && st.hasNext()) {
            double thresh = TplConvert::_2double(st.next().c_str());
            thresholds.insert(std::make_pair(measures[count], thresh));
            ++count;
        }
        if (thresholds.size() < measures.size() || st.hasNext()) {
            WRITE_ERROR("Given list of thresholds ('" + thresholds_str + "') is not of the same size as the list of measures ('" + measures_str + "').\nPlease specify exactly one threshold for each measure.");
            return false;
        }
    } else {
        // assume default thresholds if none are given
        for (std::vector<std::string>::const_iterator i = measures.begin(); i != measures.end(); ++i) {
            if (*i == "TTC") {
                thresholds.insert(std::make_pair(*i, DEFAULT_THRESHOLD_TTC));
            } else if (*i == "DRAC") {
                thresholds.insert(std::make_pair(*i, DEFAULT_THRESHOLD_DRAC));
            } else if (*i == "PET") {
                thresholds.insert(std::make_pair(*i, DEFAULT_THRESHOLD_PET));
            } else {
                WRITE_ERROR("Unknown SSM identifier '" + (*i) + "'. Aborting construction of ssm device."); // should never occur
                return false;
            }
        }
    }
    return true;
}



/****************************************************************************/

