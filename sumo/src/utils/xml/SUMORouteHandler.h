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
/// @file    SUMORouteHandler.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
/// @version $Id$
///
// Parser for routes during their loading
/****************************************************************************/
#ifndef SUMORouteHandler_h
#define SUMORouteHandler_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/IDSupplier.h>
#include <utils/common/SUMOTime.h>
#include <utils/vehicle/SUMOVehicleParameter.h>
#include <utils/xml/SUMOSAXHandler.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MsgHandler;
class SUMOVTypeParameter;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SUMORouteHandler
 * @brief Parser for routes during their loading
 *
 * SUMORouteHandler is the abstract super class for routers
 * and simulation loading routes.
 */
class SUMORouteHandler : public SUMOSAXHandler {
public:
    /// standard constructor
    SUMORouteHandler(const std::string& file);

    /// standard destructor
    virtual ~SUMORouteHandler();

    /// Returns the last loaded depart time
    SUMOTime getLastDepart() const;

    /// check start and end position of a stop
    static bool checkStopPos(double& startPos, double& endPos, const double laneLength,
                             const double minLength, const bool friendlyPos);

    /// @brief returns the first departure time that was ever read
    SUMOTime getFirstDepart() const {
        return myFirstDepart;
    }

protected:
    /// @name inherited from GenericSAXHandler
    //@{

    /** @brief Called on the opening of a tag;
     *
     * @param[in] element ID of the currently opened element
     * @param[in] attrs Attributes within the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myStartElement
     */
    virtual void myStartElement(int element,
                                const SUMOSAXAttributes& attrs);


    /** @brief Called when a closing tag occurs
     *
     * @param[in] element ID of the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myEndElement
     */
    virtual void myEndElement(int element);
    //@}


    /** opens a type distribution for reading */
    virtual void openVehicleTypeDistribution(const SUMOSAXAttributes& attrs) = 0;

    /** closes (ends) the building of a distribution */
    virtual void closeVehicleTypeDistribution() = 0;

    /** opens a route for reading */
    virtual void openRoute(const SUMOSAXAttributes& attrs) = 0;

    /** closes (ends) the building of a route.
        Afterwards no edges may be added to it;
        this method may throw exceptions when
        a) the route is empty or
        b) another route with the same id already exists */
    virtual void closeRoute(const bool mayBeDisconnected = false) = 0;

    /** opens a route distribution for reading */
    virtual void openRouteDistribution(const SUMOSAXAttributes& attrs) = 0;

    /** closes (ends) the building of a distribution */
    virtual void closeRouteDistribution() = 0;

    /// Ends the processing of a vehicle
    virtual void closeVehicle() = 0;

    /// Ends the processing of a person
    virtual void closePerson() = 0;

    /// Ends the processing of a container
    virtual void closeContainer() = 0;

    /// Ends the processing of a flow
    virtual void closeFlow() = 0;

    /// Processing of a stop
    virtual void addStop(const SUMOSAXAttributes& attrs) = 0;

    /// @brief add a routing request for a walking or intermodal person
    virtual void addPersonTrip(const SUMOSAXAttributes& attrs) = 0;

    /// @brief add a fully specified walk
    virtual void addWalk(const SUMOSAXAttributes& attrs) = 0;

    /// Checks whether the route file is sorted by departure time if needed
    bool checkLastDepart();

    /// save last depart (only to be used if vehicle is not discarded)
    void registerLastDepart();

    /// @brief assign arbitrary vehicle parameters
    void addParam(const SUMOSAXAttributes& attrs);

    /// @brief parses attributes common to all stops
    bool parseStop(SUMOVehicleParameter::Stop& stop, const SUMOSAXAttributes& attrs, std::string errorSuffix, MsgHandler* const errorOutput);

protected:
    /// @brief Parameter of the current vehicle, trip, person, container or flow
    SUMOVehicleParameter* myVehicleParameter;

    /// @brief The insertion time of the vehicle read last
    SUMOTime myLastDepart;

    /// @brief The id of the current route
    std::string myActiveRouteID;

    /// @brief The id of the route the current route references to
    std::string myActiveRouteRefID;

    /// @brief The probability of the current route
    double myActiveRouteProbability;

    /// @brief The currently parsed route's color
    const RGBColor* myActiveRouteColor;

    /// @brief The currently parsed route costs
    double myCurrentCosts;

    /// @brief List of the stops on the parsed route
    std::vector<SUMOVehicleParameter::Stop> myActiveRouteStops;

    /// @brief The currently parsed vehicle type
    SUMOVTypeParameter* myCurrentVType;

    /// generates numerical ids
    IDSupplier myIdSupplier;

    /// @brief The default value for flow begins
    SUMOTime myBeginDefault;

    /// @brief The default value for flow ends
    SUMOTime myEndDefault;

    /// @brief the first read departure time
    SUMOTime myFirstDepart;

    /// @brief where stop edges can be inserted into the current route (-1 means no insertion)
    int myInsertStopEdgesAt;

    /// @brief the default car following model
    SumoXMLTag myDefaultCFModel;

private:
    /// @brief Invalidated copy constructor
    SUMORouteHandler(const SUMORouteHandler& s);

    /// @brief Invalidated assignment operator
    SUMORouteHandler& operator=(const SUMORouteHandler& s);

};


#endif

/****************************************************************************/
