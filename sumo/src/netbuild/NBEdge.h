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
/// @file    NBEdge.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Tue, 20 Nov 2001
/// @version $Id$
///
// The representation of a single edge during network building
/****************************************************************************/
#ifndef NBEdge_h
#define NBEdge_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <map>
#include <vector>
#include <string>
#include <set>
#include <cassert>
#include <utils/common/Named.h>
#include <utils/common/Parameterised.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/VectorHelper.h>
#include <utils/geom/Bresenham.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include "NBCont.h"
#include "NBHelpers.h"
#include "NBSign.h"


// ===========================================================================
// class declarations
// ===========================================================================
class NBNode;
class NBConnection;
class NBNodeCont;
class NBEdgeCont;
class OutputDevice;
class GNELane;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NBEdge
 * @brief The representation of a single edge during network building
 */
class NBEdge : public Named, public Parameterised {
    friend class NBEdgeCont;

    /** used for visualization (NETEDIT) */
    friend class GNELane;
    friend class GNEEdge;
    friend class GNEJunction;

public:

    /** @enum EdgeBuildingStep
     * @brief Current state of the edge within the building process
     *
     * As the network is build in a cascaded way, considering loaded
     *  information, a counter holding the current step is needed. This is done
     *  by using this enumeration.
     */
    enum EdgeBuildingStep {
        /// @brief The edge has been loaded and connections shall not be added
        INIT_REJECT_CONNECTIONS,
        /// @brief The edge has been loaded, nothing is computed yet
        INIT,
        /// @brief The relationships between edges are computed/loaded
        EDGE2EDGES,
        /// @brief Lanes to edges - relationships are computed/loaded
        LANES2EDGES,
        /// @brief Lanes to lanes - relationships are computed; should be recheked
        LANES2LANES_RECHECK,
        /// @brief Lanes to lanes - relationships are computed; no recheck is necessary/wished
        LANES2LANES_DONE,
        /// @brief Lanes to lanes - relationships are loaded; no recheck is necessary/wished
        LANES2LANES_USER
    };


    /** @enum Lane2LaneInfoType
    * @brief Modes of setting connections between lanes
    */
    enum Lane2LaneInfoType {
        /// @brief The connection was computed
        L2L_COMPUTED,
        /// @brief The connection was given by the user
        L2L_USER,
        /// @brief The connection was computed and validated
        L2L_VALIDATED
    };


    /** @struct Lane
     * @brief An (internal) definition of a single lane of an edge
     */
    struct Lane : public Parameterised {
        /// @brief constructor
        Lane(NBEdge* e, const std::string& _origID);

        /// @brief The lane's shape
        PositionVector shape;

        /// @brief The speed allowed on this lane
        double speed;

        /// @brief List of vehicle types that are allowed on this lane
        SVCPermissions permissions;

        /// @brief List of vehicle types that are preferred on this lane
        SVCPermissions preferred;

        /// @brief This lane's offset to the intersection begin
        double endOffset;

        /// @brief This lane's width
        double width;

        /// @brief An opposite lane ID, if given
        std::string oppositeID;

        /// @brief Whether this lane is an acceleration lane
        bool accelRamp;

        /// @brief Whether connection information for this lane is already completed
        // @note (see NIImporter_DlrNavteq::ConnectedLanesHandler)
        bool connectionsDone;

        /// @brief A custom shape for this lane set by the user
        PositionVector customShape;
    };


    /** @struct Connection
     * @brief A structure which describes a connection between edges or lanes
     */
    struct Connection : public Parameterised {
        /** @brief Constructor
         * @param[in] fromLane_ The lane the connections starts at
         * @param[in] toEdge_ The edge the connections yields in
         * @param[in] toLane_ The lane the connections yields in
         */
        Connection(int fromLane_, NBEdge* toEdge_, int toLane_);

        /// @brief constructor with more parameters
        Connection(int fromLane_, NBEdge* toEdge_, int toLane_, bool mayDefinitelyPass_,
                   bool keepClear_ = true,
                   double contPos_ = UNSPECIFIED_CONTPOS,
                   double visibility_ = UNSPECIFIED_VISIBILITY_DISTANCE,
                   double speed_ = UNSPECIFIED_SPEED,
                   bool haveVia_ = false,
                   bool uncontrolled_ = false,
                   const PositionVector& customShape_ = PositionVector::EMPTY);

        /// @brief destructor
        ~Connection() { }

        /// @brief The lane the connections starts at
        int fromLane;

        /// @brief The edge the connections yields in
        NBEdge* toEdge;

        /// @brief The lane the connections yields in
        int toLane;

        /// @brief The id of the traffic light that controls this connection
        std::string tlID;

        /// @brief The index of this connection within the controlling traffic light
        int tlLinkNo;

        /// @brief Information about being definitely free to drive (on-ramps)
        bool mayDefinitelyPass;

        /// @brief whether the junction must be kept clear when using this connection
        bool keepClear;

        /// @brief custom position for internal junction on this connection
        double contPos;

        /// @brief custom foe visiblity for connection
        double visibility;

        /// @brief custom speed for connection
        double speed;

        /// @brief custom shape for connection
        PositionVector customShape;

        /// @brief id of Connection
        std::string id;

        /// @brief shape of Connection
        PositionVector shape;

        /// @brief maximun velocity
        double vmax;

        /// @brief check if Connection have a Via
        bool haveVia;

        /// @brief if Connection have a via, ID of it
        std::string viaID;

        /// @brief shape of via
        PositionVector viaShape;

        /// @brief FOE Internal links
        std::vector<int> foeInternalLinks;

        /// @brief FOE Incomings lanes
        std::string foeIncomingLanes;

        /// @brief The lane index of this internal lane within the internal edge
        int internalLaneIndex;

        /// @brief check if Connection is uncontrolled
        bool uncontrolled;

        /// @brief get ID of internal lane
        std::string getInternalLaneID() const;

        /// @brief get string describing this connection
        std::string getDescription(const NBEdge* parent) const;
    };


    /// @brief unspecified lane width
    static const double UNSPECIFIED_WIDTH;

    /// @brief unspecified lane offset
    static const double UNSPECIFIED_OFFSET;

    /// @brief unspecified lane speed
    static const double UNSPECIFIED_SPEED;

    /// @brief unspecified internal junction position
    static const double UNSPECIFIED_CONTPOS;

    /// @brief unspecified foe visibility for connections
    static const double UNSPECIFIED_VISIBILITY_DISTANCE;

    /// @brief no length override given
    static const double UNSPECIFIED_LOADED_LENGTH;

    /// @brief unspecified signal offset
    static const double UNSPECIFIED_SIGNAL_OFFSET;

    /// @brief the distance at which to take the default angle
    static const double ANGLE_LOOKAHEAD;
    /// @brief internal lane computation not yet done
    static const int UNSPECIFIED_INTERNAL_LANE_INDEX;

    /// @brief junction priority values set by setJunctionPriority
    enum JunctionPriority {
        MINOR_ROAD = 0,
        PRIORITY_ROAD = 1,
        ROUNDABOUT = 1000
    };

public:
    /** @brief Constructor
     *
     * Use this if no edge geometry is given.
     *
     * @param[in] id The id of the edge
     * @param[in] from The node the edge starts at
     * @param[in] to The node the edge ends at
     * @param[in] type The type of the edge (my be =="")
     * @param[in] speed The maximum velocity allowed on this edge
     * @param[in] nolanes The number of lanes this edge has
     * @param[in] priority This edge's priority
     * @param[in] width This edge's lane width
     * @param[in] offset Additional offset to the destination node
     * @param[in] streetName The street name (need not be unique)
     * @param[in] spread How the lateral offset of the lanes shall be computed
     * @see init
     * @see LaneSpreadFunction
     */
    NBEdge(const std::string& id,
           NBNode* from, NBNode* to, std::string type,
           double speed, int nolanes, int priority,
           double width, double offset,
           const std::string& streetName = "",
           LaneSpreadFunction spread = LANESPREAD_RIGHT);


    /** @brief Constructor
     *
     * Use this if the edge's geometry is given.
     *
     * @param[in] id The id of the edge
     * @param[in] from The node the edge starts at
     * @param[in] to The node the edge ends at
     * @param[in] type The type of the edge (may be =="")
     * @param[in] speed The maximum velocity allowed on this edge
     * @param[in] nolanes The number of lanes this edge has
     * @param[in] priority This edge's priority
     * @param[in] width This edge's lane width
     * @param[in] offset Additional offset to the destination node
     * @param[in] geom The edge's geomatry
     * @param[in] streetName The street name (need not be unique)
     * @param[in] origID The original ID in the source network (need not be unique)
     * @param[in] spread How the lateral offset of the lanes shall be computed
     * @param[in] tryIgnoreNodePositions Does not add node geometries if geom.size()>=2
     * @see init
     * @see LaneSpreadFunction
     */
    NBEdge(const std::string& id,
           NBNode* from, NBNode* to, std::string type,
           double speed, int nolanes, int priority,
           double width, double offset,
           PositionVector geom,
           const std::string& streetName = "",
           const std::string& origID = "",
           LaneSpreadFunction spread = LANESPREAD_RIGHT,
           bool tryIgnoreNodePositions = false);

    /** @brief Constructor
     *
     * Use this to copy attribuets from another edge
     *
     * @param[in] id The id of the edge
     * @param[in] from The node the edge starts at
     * @param[in] to The node the edge ends at
     * @param[in] tpl The template edge to copy attributes from
     * @param[in] geom The geometry to use (may be empty)
     * @param[in] numLanes The number of lanes of the new edge (copy from tpl by default)
     */
    NBEdge(const std::string& id,
           NBNode* from, NBNode* to,
           NBEdge* tpl,
           const PositionVector& geom = PositionVector(),
           int numLanes = -1);


    /// @brief Destructor
    ~NBEdge();


    /** @brief Resets initial values
     *
     * @param[in] from The node the edge starts at
     * @param[in] to The node the edge ends at
     * @param[in] type The type of the edge (may be =="")
     * @param[in] speed The maximum velocity allowed on this edge
     * @param[in] nolanes The number of lanes this edge has
     * @param[in] priority This edge's priority
     * @param[in] geom The edge's geomatry
     * @param[in] width This edge's lane width
     * @param[in] offset Additional offset to the destination node
     * @param[in] streetName The street name (need not be unique)
     * @param[in] spread How the lateral offset of the lanes shall be computed
     * @param[in] tryIgnoreNodePositions Does not add node geometries if geom.size()>=2
     */
    void reinit(NBNode* from, NBNode* to, const std::string& type,
                double speed, int nolanes, int priority,
                PositionVector geom, double width, double offset,
                const std::string& streetName,
                LaneSpreadFunction spread = LANESPREAD_RIGHT,
                bool tryIgnoreNodePositions = false);

    /** @brief Resets nodes but keeps all other values the same (used when joining)
     * @param[in] from The node the edge starts at
     * @param[in] to The node the edge ends at
     */
    void reinitNodes(NBNode* from, NBNode* to);

    /// @name Applying offset
    /// @{
    /** @brief Applies an offset to the edge
     * @param[in] xoff The x-offset to apply
     * @param[in] yoff The y-offset to apply
     */
    void reshiftPosition(double xoff, double yoff);

    /// @brief mirror coordinates along the x-axis
    void mirrorX();
    /// @}

    /// @name Atomar getter methods
    //@{

    /** @brief Returns the number of lanes
     * @returns This edge's number of lanes
     */
    int getNumLanes() const {
        return (int)myLanes.size();
    }

    /** @brief Returns the priority of the edge
     * @return This edge's priority
     */
    int getPriority() const {
        return myPriority;
    }

    /** @brief Returns the origin node of the edge
     * @return The node this edge starts at
     */
    NBNode* getFromNode() const {
        return myFrom;
    }

    /** @brief Returns the destination node of the edge
     * @return The node this edge ends at
     */
    NBNode* getToNode() const {
        return myTo;
    }

    /** @brief Returns the angle at the start of the edge
     * (relative to the node shape center)
     * The angle is computed in computeAngle()
     * @return This edge's start angle
     */
    inline double getStartAngle() const {
        return myStartAngle;
    }

    /** @brief Returns the angle at the end of the edge
     * (relative to the node shape center)
     * The angle is computed in computeAngle()
     * @return This edge's end angle
     */
    inline double getEndAngle() const {
        return myEndAngle;
    }

    /** @brief Returns the angle at the start of the edge
     * @note only using edge shape
     * @return This edge's start angle
     */
    double getShapeStartAngle() const;


    /** @brief Returns the angle at the end of the edge
     * @note only using edge shape
     * @note The angle is computed in computeAngle()
     * @return This edge's end angle
     */
    double getShapeEndAngle() const;

    /** @brief Returns the angle at the start of the edge
     * @note The angle is computed in computeAngle()
     * @return This edge's angle
     */
    inline double getTotalAngle() const {
        return myTotalAngle;
    }

    /** @brief Returns the computed length of the edge
     * @return The edge's computed length
     */
    double getLength() const {
        return myLength;
    }


    /** @brief Returns the length was set explicitly or the computed length if it wasn't set
     * @todo consolidate use of myLength and myLoaded length
     * @return The edge's specified length
     */
    double getLoadedLength() const {
        return myLoadedLength > 0 ? myLoadedLength : myLength;
    }

    /// @brief get length that will be assigned to the lanes in the final network
    double getFinalLength() const;

    /** @brief Returns whether a length was set explicitly
     * @return Wether the edge's length was specified
     */
    bool hasLoadedLength() const {
        return myLoadedLength > 0;
    }

    /** @brief Returns the speed allowed on this edge
     * @return The maximum speed allowed on this edge
     */
    double getSpeed() const {
        return mySpeed;
    }

    /** @brief The building step of this edge
     * @return The current building step for this edge
     * @todo Recheck usage!
     * @see EdgeBuildingStep
     */
    EdgeBuildingStep getStep() const {
        return myStep;
    }

    /** @brief Returns the default width of lanes of this edge
     * @return The width of lanes of this edge
     */
    double getLaneWidth() const {
        return myLaneWidth;
    }

    /** @brief Returns the width of the lane of this edge
     * @return The width of the lane of this edge
     */
    double getLaneWidth(int lane) const;

    /// @brief Returns the combined width of all lanes of this edge
    double getTotalWidth() const;

    /// @brief Returns the street name of this edge
    const std::string& getStreetName() const {
        return myStreetName;
    }

    /// @brief sets the street name of this edge
    void setStreetName(const std::string& name) {
        myStreetName = name;
    }

    /** @brief Returns the offset to the destination node
     * @return The offset to the destination node
     */
    double getEndOffset() const {
        return myEndOffset;
    }

    /** @brief Returns the offset to the destination node a the specified lane
     * @return The offset to the destination node
     */
    double getEndOffset(int lane) const;

    /// @brief Returns the offset of a traffic signal from the end of this edge
    double getSignalOffset() const {
        return mySignalOffset;
    }

    /// @brief sets the offset of a traffic signal from the end of this edge
    void setSignalOffset(double offset) {
        mySignalOffset = offset;
    }

    /** @brief Returns the lane definitions
     * @return The stored lane definitions
     */
    const std::vector<NBEdge::Lane>& getLanes() const {
        return myLanes;
    }
    //@}

    /** @brief return the first lane with permissions other than SVC_PEDESTRIAN and 0
     * @param[in] direction The direction in which the lanes shall be checked
     * @param[in] exclusive Whether lanes that allow pedestrians along with other classes shall be counted as non-pedestrian
     */
    int getFirstNonPedestrianLaneIndex(int direction, bool exclusive = false) const;

    /// @brif get first non-pedestrian lane
    NBEdge::Lane getFirstNonPedestrianLane(int direction) const;

    /// @brief return all permission variants within the specified lane range [iStart, iEnd[
    std::set<SVCPermissions> getPermissionVariants(int iStart, int iEnd) const;

    /// @brief return the angle for computing pedestrian crossings at the given node
    double getCrossingAngle(NBNode* node);

    /// @name Edge geometry access and computation
    //@{
    /** @brief Returns the geometry of the edge
     * @return The edge's geometry
     */
    const PositionVector& getGeometry() const {
        return myGeom;
    }

    /// @brief Returns the geometry of the edge without the endpoints
    const PositionVector getInnerGeometry() const;

    /// @brief Returns whether the geometry consists only of the node positions
    bool hasDefaultGeometry() const;

    /** @brief Returns whether the geometry is terminated by the node positions
     * This default may be violated by initializing with
     * tryIgnoreNodePositions=true' or with setGeometry()
     * non-default endpoints are useful to control the generated node shape
     */
    bool hasDefaultGeometryEndpoints() const;

    /** @brief Returns whether the geometry is terminated by the node positions
     * This default may be violated by initializing with
     * tryIgnoreNodePositions=true' or with setGeometry()
     * non-default endpoints are useful to control the generated node shape
     */
    bool hasDefaultGeometryEndpointAtNode(const NBNode* node) const;

    /** @brief (Re)sets the edge's geometry
     *
     * Replaces the edge's prior geometry by the given. Then, computes
     *  the geometries of all lanes using computeLaneShapes.
     * Definitely not the best way to have it accessable from outside...
     * @param[in] g The edge's new geometry
     * @param[in] inner whether g should be interpreted as inner points
     * @todo Recheck usage, disallow access
     * @see computeLaneShapes
     */
    void setGeometry(const PositionVector& g, bool inner = false);

    /** @brief Adds a further geometry point
     *
     * Some importer do not know an edge's geometry when it is initialised.
     *  This method allows to insert further geometry points after the edge
     *  has been built.
     *
     * @param[in] index The position at which the point shall be added
     * @param[in] p The point to add
     */
    void addGeometryPoint(int index, const Position& p);

    /// @brief linearly extend the geometry at the given node
    void extendGeometryAtNode(const NBNode* node, double maxExtent);

    /// @brief linearly extend the geometry at the given node
    void shortenGeometryAtNode(const NBNode* node, double reduction);

    /// @brief shift geometry at the given node to avoid overlap
    void shiftPositionAtNode(NBNode* node, NBEdge* opposite);

    /** @brief Recomputeds the lane shapes to terminate at the node shape
     * For every lane the intersection with the fromNode and toNode is
     * calculated and the lane shorted accordingly. The edge length is then set
     * to the average of all lane lenghts (which may differ). This average length is used as the lane
     * length when writing the network.
     * @note All lanes of an edge in a sumo net must have the same nominal length
     *  but may differ in actual geomtric length.
     * @note Depends on previous call to NBNodeCont::computeNodeShapes
     */
    void computeEdgeShape();

    /** @brief Returns the shape of the nth lane
     * @return The shape of the lane given by its index (counter from right)
     */
    const PositionVector& getLaneShape(int i) const;

    /** @brief (Re)sets how the lanes lateral offset shall be computed
     * @param[in] spread The type of lateral offset to apply
     * @see LaneSpreadFunction
     */
    void setLaneSpreadFunction(LaneSpreadFunction spread);

    /** @brief Returns how this edge's lanes' lateral offset is computed
     * @return The type of lateral offset that is applied on this edge
     * @see LaneSpreadFunction
     */
    LaneSpreadFunction getLaneSpreadFunction() const {
        return myLaneSpreadFunction;
    }

    /** @brief Splits this edge at geometry points
     * @param[in] ec The edge cont to add new edges to
     * @param[in] nc The node cont to add new nodes to
     * @return Whether the geometry was changed
     */
    bool splitGeometry(NBEdgeCont& ec, NBNodeCont& nc);

    /** @brief Removes points with a distance lesser than the given
     * @param[in] minDist The minimum distance between two position to keep the second
     */
    void reduceGeometry(const double minDist);

    /** @brief Check the angles of successive geometry segments
     * @param[in] maxAngle The maximum angle allowed
     * @param[in] minRadius The minimum turning radius allowed at the start and end
     * @param[in] fix Whether to prune geometry points to avoid sharp turns at start and end
     */
    void checkGeometry(const double maxAngle, const double minRadius, bool fix);
    //@}

    /// @name Setting and getting connections
    /// @{
    /** @brief Adds a connection to another edge
     *
     * If the given edge does not start at the node this edge ends on, false is returned.
     *
     * All other cases return true. Though, a connection may not been added if this edge
     *  is in step "INIT_REJECT_CONNECTIONS". Also, this method assures that a connection
     *  to an edge is set only once, no multiple connections to next edge are stored.
     *
     * After a first connection to an edge was set, the process step is set to "EDGE2EDGES".
     * @note Passing 0 implicitly removes all existing connections
     *
     * @param[in] dest The connection's destination edge
     * @return Whether the connection was valid
     */
    bool addEdge2EdgeConnection(NBEdge* dest);

    /** @brief Adds a connection between the specified this edge's lane and an approached one
     *
     * If the given edge does not start at the node this edge ends on, false is returned.
     *
     * All other cases return true. Though, a connection may not been added if this edge
     *  is in step "INIT_REJECT_CONNECTIONS". Before the lane-to-lane connection is set,
     *  a connection between edges is established using "addEdge2EdgeConnection". Then,
     *  "setConnection" is called for inserting the lane-to-lane connection.
     *
     * @param[in] fromLane The connection's starting lane (of this edge)
     * @param[in] dest The connection's destination edge
     * @param[in] toLane The connection's destination lane
     * @param[in] type The connections's type
     * @param[in] mayUseSameDestination Whether this connection may be set though connecting an already connected lane
     * @param[in] mayDefinitelyPass Whether this connection is definitely undistrubed (special case for on-ramps)
     * @return Whether the connection was added / exists
     * @see addEdge2EdgeConnection
     * @see setConnection
     * @todo Check difference between "setConnection" and "addLane2LaneConnection"
     */
    bool addLane2LaneConnection(int fromLane, NBEdge* dest,
                                int toLane, Lane2LaneInfoType type,
                                bool mayUseSameDestination = false,
                                bool mayDefinitelyPass = false,
                                bool keepClear = true,
                                double contPos = UNSPECIFIED_CONTPOS,
                                double visibility = UNSPECIFIED_VISIBILITY_DISTANCE,
                                double speed = UNSPECIFIED_SPEED,
                                const PositionVector& customShape = PositionVector::EMPTY);

    /** @brief Builds no connections starting at the given lanes
     *
     * If "invalidatePrevious" is true, a call to "invalidateConnections(true)" is done.
     * This method loops through the given connections to set, calling "addLane2LaneConnection"
     *  for each.
     *
     * @param[in] fromLane The first of the connections' starting lanes (of this edge)
     * @param[in] dest The connections' destination edge
     * @param[in] toLane The first of the connections' destination lanes
     * @param[in] no The number of connections to set
     * @param[in] type The connections' type
     * @param[in] invalidatePrevious Whether previously set connection shall be deleted
     * @param[in] mayDefinitelyPass Whether these connections are definitely undistrubed (special case for on-ramps)
     * @return Whether the connections were added / existed
     * @see addLane2LaneConnection
     * @see invalidateConnections
     */
    bool addLane2LaneConnections(int fromLane,
                                 NBEdge* dest, int toLane, int no,
                                 Lane2LaneInfoType type, bool invalidatePrevious = false,
                                 bool mayDefinitelyPass = false);

    /** @brief Adds a connection to a certain lane of a certain edge
     *
     * @param[in] lane The connection's starting lane (of this edge)
     * @param[in] destEdge The connection's destination edge
     * @param[in] destLane The connection's destination lane
     * @param[in] type The connections's type
     * @param[in] mayUseSameDestination Whether this connection may be set though connecting an already connected lane
     * @param[in] mayDefinitelyPass Whether this connection is definitely undistrubed (special case for on-ramps)
     * @todo Check difference between "setConnection" and "addLane2LaneConnection"
     */
    bool setConnection(int lane, NBEdge* destEdge,
                       int destLane,
                       Lane2LaneInfoType type,
                       bool mayUseSameDestination = false,
                       bool mayDefinitelyPass = false,
                       bool keepClear = true,
                       double contPos = UNSPECIFIED_CONTPOS,
                       double visibility = UNSPECIFIED_VISIBILITY_DISTANCE,
                       double speed = UNSPECIFIED_SPEED,
                       const PositionVector& customShape = PositionVector::EMPTY);

    /// @brief insert a previously created NBEdge::connection
    void insertConnection(NBEdge::Connection connection);

    /** @brief Returns connections from a given lane
     *
     * This method goes through "myConnections" and copies those which are
     *  starting at the given lane.
     * @param[in] lane The lane which connections shall be returned
     * @return The connections from the given lane
     * @see NBEdge::Connection
     */
    std::vector<Connection> getConnectionsFromLane(int lane) const;

    /** @brief Returns the specified connection
     * This method goes through "myConnections" and returns the specified one
     * @see NBEdge::Connection
     */
    Connection getConnection(int fromLane, const NBEdge* to, int toLane) const;

    /** @brief Returns reference to the specified connection
     * This method goes through "myConnections" and returns the specified one
     * @see NBEdge::Connection
     */
    Connection& getConnectionRef(int fromLane, const NBEdge* to, int toLane);

    /** @brief Retrieves info about a connection to a certain lane of a certain edge
     *
     * Turnaround edge is ignored!
     * @param[in] destEdge The connection's destination edge
     * @param[in] destLane The connection's destination lane
     * @param[in] fromLane If a value >= 0 is given, only return true if a connection from the given lane exists
     * @return whether a connection to the specified lane exists
     */
    bool hasConnectionTo(NBEdge* destEdge, int destLane, int fromLane = -1) const;

    /** @brief Returns the information whethe a connection to the given edge has been added (or computed)
     *
     * Turnaround edge is not ignored!
     * @param[in] e The destination edge
     * @return Whether a connection to the specified edge exists
     */
    bool isConnectedTo(const NBEdge* e) const;

    /** @brief Returns the connections
     * @return This edge's connections to following edges
     */
    const std::vector<Connection>& getConnections() const {
        return myConnections;
    }

    /** @brief Returns the connections
     * @return This edge's connections to following edges
     */
    std::vector<Connection>& getConnections() {
        return myConnections;
    }

    /** @brief Returns the list of outgoing edges without the turnaround sorted in clockwise direction
     * @return Connected edges, sorted clockwise
     */
    const EdgeVector* getConnectedSorted();

    /** @brief Returns the list of outgoing edges unsorted
     * @return Connected edges
     */
    EdgeVector getConnectedEdges() const;

    /** @brief Returns the list of incoming edges unsorted
     * @return Connected predecessor edges
     */
    EdgeVector getIncomingEdges() const;

    /** @brief Returns the list of lanes that may be used to reach the given edge
     * @return Lanes approaching the given edge
     */
    std::vector<int> getConnectionLanes(NBEdge* currentOutgoing) const;

    /// @brief sorts the outgoing connections by their angle relative to their junction
    void sortOutgoingConnectionsByAngle();

    /// @brief sorts the outgoing connections by their from-lane-index and their to-lane-index
    void sortOutgoingConnectionsByIndex();

    /** @brief Remaps the connection in a way that allows the removal of it
     *
     * This edge (which is a self loop edge, in fact) connections are spread over the valid incoming edges
     * @todo recheck!
     */
    void remapConnections(const EdgeVector& incoming);

    /** @brief Removes the specified connection(s)
     * @param[in] toEdge The destination edge
     * @param[in] fromLane The lane from which connections shall be removed; -1 means remove all
     * @param[in] toLane   The lane to which connections shall be removed; -1 means remove all
     * @param[in] tryLater If the connection does not exist, try again during recheckLanes()
     * @param[in] adaptToLaneRemoval we are in the process of removing a complete lane, adapt all connections accordingly
     */
    void removeFromConnections(NBEdge* toEdge, int fromLane = -1, int toLane = -1, bool tryLater = false, const bool adaptToLaneRemoval = false);

    /// @brief remove an existent connection of edge
    bool removeFromConnections(NBEdge::Connection connectionToRemove);

    /// @brief invalidate current connections of edge
    void invalidateConnections(bool reallowSetting = false);

    /// @brief replace in current connections of edge
    void replaceInConnections(NBEdge* which, NBEdge* by, int laneOff);

    /// @brief replace in current connections of edge
    void replaceInConnections(NBEdge* which, const std::vector<NBEdge::Connection>& origConns);

    /// @brief copy connections from antoher edge
    void copyConnectionsFrom(NBEdge* src);

    /// @brief modifify the toLane for all connections to the given edge
    void shiftToLanesToEdge(NBEdge* to, int laneOff);
    /// @}

    /** @brief Returns whether the given edge is the opposite direction to this edge
     * @param[in] edge The edge which may be the turnaround direction
     * @return Whether the given edge is this edge's turnaround direction
     * (regardless of whether a connection exists)
     */
    bool isTurningDirectionAt(const NBEdge* const edge) const;

    /** @brief Sets the turing destination at the given edge
     * @param[in] e The turn destination
     * @param[in] onlyPossible If true, only sets myPossibleTurnDestination
     */
    void setTurningDestination(NBEdge* e, bool onlyPossible = false);

    /// @name Setting/getting special types
    /// @{
    /// @brief Marks this edge as a macroscopic connector
    void setAsMacroscopicConnector() {
        myAmMacroscopicConnector = true;
    }

    /** @brief Returns whether this edge was marked as a macroscopic connector
     * @return Whether this edge was marked as a macroscopic connector
     */
    bool isMacroscopicConnector() const {
        return myAmMacroscopicConnector;
    }

    /// @brief Marks this edge being within an intersection
    void setIsInnerEdge() {
        myAmInnerEdge = true;
    }

    /** @brief Returns whether this edge was marked as being within an intersection
     * @return Whether this edge was marked as being within an intersection
     */
    bool isInnerEdge() const {
        return myAmInnerEdge;
    }
    /// @}

    /** @brief Sets the junction priority of the edge
     * @param[in] node The node for which the edge's priority is given
     * @param[in] prio The edge's new priority at this node
     * @todo Maybe the edge priority whould be stored in the node
     */
    void setJunctionPriority(const NBNode* const node, int prio);

    /** @brief Returns the junction priority (normalised for the node currently build)
     *
     * If the given node is neither the edge's start nor the edge's ending node, the behaviour
     *  is undefined.
     *
     * @param[in] node The node for which the edge's priority shall be returned
     * @return The edge's priority at the given node
     * @todo Maybe the edge priority whould be stored in the node
     */
    int getJunctionPriority(const NBNode* const node) const;

    /// @brief set loaded lenght
    void setLoadedLength(double val);

    /// @brief dimiss vehicle class information
    void dismissVehicleClassInformation();

    /// @brief get ID of type
    const std::string& getTypeID() const {
        return myType;
    }

    /// @brief whether at least one lane has values differing from the edges values
    bool needsLaneSpecificOutput() const;

    /// @brief whether at least one lane has restrictions
    bool hasPermissions() const;

    /// @brief whether lanes differ in allowed vehicle classes
    bool hasLaneSpecificPermissions() const;

    /// @brief whether lanes differ in speed
    bool hasLaneSpecificSpeed() const;

    /// @brief whether lanes differ in width
    bool hasLaneSpecificWidth() const;

    /// @brief whether lanes differ in offset
    bool hasLaneSpecificEndOffset() const;

    /// @brief whether one of the lanes is an acceleration lane
    bool hasAccelLane() const;

    /// @brief whether one of the lanes has a custom shape
    bool hasCustomLaneShape() const;

    /// @brief whether one of the lanes has parameters set
    bool hasLaneParams() const;

    /// @brief computes the edge (step1: computation of approached edges)
    bool computeEdge2Edges(bool noLeftMovers);

    /// @brief computes the edge, step2: computation of which lanes approach the edges)
    bool computeLanes2Edges();

    /// @brief recheck whether all lanes within the edge are all right and optimises the connections once again
    bool recheckLanes();

    /** @brief Add a connection to the previously computed turnaround, if wished
     *
     * If a turning direction exists (myTurnDestination!=0) and either the
     *  edge is not controlled by a tls or noTLSControlled is false, a connection
     *  to the edge stored in myTurnDestination is added (from the leftmost lane
     *  of this edge to the leftmost lane of myTurnDestination).
     * @param[in] noTLSControlled Whether the turnaround shall not be connected if this edge is controlled by a tls
     */
    void appendTurnaround(bool noTLSControlled, bool checkPermissions);

    /** @brief Returns the node at the given edges length (using an epsilon)
        @note When no node is existing at the given position, 0 is returned
        The epsilon is a static member of NBEdge, should be setable via program options */
    NBNode* tryGetNodeAtPosition(double pos, double tolerance = 5.0) const;

    /// @brief get max lane offset
    double getMaxLaneOffset();

    /// @brief Check if lanes were assigned
    bool lanesWereAssigned() const;

    /// @brief return true if certain connection must be controlled by TLS
    bool mayBeTLSControlled(int fromLane, NBEdge* toEdge, int toLane) const;

    /// @brief Returns if the link could be set as to be controlled
    bool setControllingTLInformation(const NBConnection& c, const std::string& tlID);

    /// @brief clears tlID for all connections
    void clearControllingTLInformation();

    /// @brief add crossing points as incoming with given outgoing
    void addCrossingPointsAsIncomingWithGivenOutgoing(NBEdge* o, PositionVector& into);

    /// @brief get the outer boundary of this edge when going clock-wise around the given node
    PositionVector getCWBoundaryLine(const NBNode& n) const;

    /// @brief get the outer boundary of this edge when going counter-clock-wise around the given node
    PositionVector getCCWBoundaryLine(const NBNode& n) const;

    /// @brief Check if Node is expandable
    bool expandableBy(NBEdge* possContinuation, std::string& reason) const;

    /// @brief append another edge
    void append(NBEdge* continuation);

    /// @brief Check if edge has signalised connections
    bool hasSignalisedConnectionTo(const NBEdge* const e) const;

    /// @brief move outgoing connection
    void moveOutgoingConnectionsFrom(NBEdge* e, int laneOff);

    /* @brief return the turn destination if it exists
     * @param[in] possibleDestination Wether myPossibleTurnDestination should be returned if no turnaround connection
     * exists
     */
    NBEdge* getTurnDestination(bool possibleDestination = false) const;

    /// @brief get Lane ID (Secure)
    std::string getLaneID(int lane) const;

    /// @brief get Lane ID (Insecure)
    std::string getLaneIDInsecure(int lane) const;

    /// @brief get lane speed
    double getLaneSpeed(int lane) const;

    /// @brief Check if edge is near enought to be joined to another edge
    bool isNearEnough2BeJoined2(NBEdge* e, double threshold) const;

    /** @brief Returns the angle of the edge's geometry at the given node
     *
     * The angle is signed, regards direction, and starts at 12 o'clock
     *  (north->south), proceeds positive clockwise.
     * @param[in] node The node for which the edge's angle shall be returned
     * @return This edge's angle at the given node
     */
    double getAngleAtNode(const NBNode* const node) const;

    /** @brief Returns the angle of from the node shape center to where the edge meets
     * the node shape
     *
     * The angle is signed, disregards direction, and starts at 12 o'clock
     *  (north->south), proceeds positive clockwise.
     * @param[in] node The node for which the edge's angle shall be returned
     * @return This edge's angle at the given node shape
     */
    double getAngleAtNodeToCenter(const NBNode* const node) const;

    /// @brief increment lane
    void incLaneNo(int by);

    /// @brief decrement lane
    void decLaneNo(int by);

    /// @brief delete lane
    void deleteLane(int index, bool recompute = true);

    /// @brief add lane
    void addLane(int index, bool recompute = true);

    /// @brief mark edge as in lane to state lane
    void markAsInLane2LaneState();

    /// @brief add a pedestrian sidewalk of the given width and shift existing connctions
    void addSidewalk(double width);

    /// @brief restore an previously added sidewalk
    void restoreSidewalk(std::vector<NBEdge::Lane> oldLanes, PositionVector oldGeometry, std::vector<NBEdge::Connection> oldConnections);

    /// add a bicycle lane of the given width and shift existing connctions
    void addBikeLane(double width);

    /// @brief restore an previously added BikeLane
    void restoreBikelane(std::vector<NBEdge::Lane> oldLanes, PositionVector oldGeometry, std::vector<NBEdge::Connection> oldConnections);

    /// @brief set allowed/disallowed classes for the given lane or for all lanes if -1 is given
    void setPermissions(SVCPermissions permissions, int lane = -1);

    /// @brief set preferred Vehicle Class
    void setPreferredVehicleClass(SVCPermissions permissions, int lane = -1);

    /// @brief set allowed class for the given lane or for all lanes if -1 is given
    void allowVehicleClass(int lane, SUMOVehicleClass vclass);

    /// @brief set disallowed class for the given lane or for all lanes if -1 is given
    void disallowVehicleClass(int lane, SUMOVehicleClass vclass);

    /// @brief prefer certain vehicle class
    void preferVehicleClass(int lane, SUMOVehicleClass vclass);

    /// @brief set lane specific width (negative lane implies set for all lanes)
    void setLaneWidth(int lane, double width);

    /// @brief set lane specific end-offset (negative lane implies set for all lanes)
    void setEndOffset(int lane, double offset);

    /// @brief set lane specific speed (negative lane implies set for all lanes)
    void setSpeed(int lane, double speed);

    /// @brief marks one lane as acceleration lane
    void setAcceleration(int lane, bool accelRamp);

    /// @brief sets a custom lane shape
    void setLaneShape(int lane, const PositionVector& shape);

    /// @brief get the union of allowed classes over all lanes or for a specific lane
    SVCPermissions getPermissions(int lane = -1) const;

    /// @brief set origID for all lanes
    void setOrigID(const std::string origID);

    /// @brief disable connections for TLS
    void disableConnection4TLS(int fromLane, NBEdge* toEdge, int toLane);

    // @brief returns a reference to the internal structure for the convenience of NETEDIT
    Lane& getLaneStruct(int lane) {
        assert(lane >= 0);
        assert(lane < (int)myLanes.size());
        return myLanes[lane];
    }

    // @brief returns a reference to the internal structure for the convenience of NETEDIT
    const Lane& getLaneStruct(int lane) const {
        assert(lane >= 0);
        assert(lane < (int)myLanes.size());
        return myLanes[lane];
    }

    /// @brief declares connections as fully loaded. This is needed to avoid recomputing connections if an edge has no connections intentionally.
    void declareConnectionsAsLoaded(EdgeBuildingStep step = LANES2LANES_USER) {
        myStep = step;
    }

    /* @brief fill connection attributes shape, viaShape, ...
     *
     * @param[in,out] edgeIndex The number of connections already handled
     * @param[in,out] splitIndex The number of via edges already built
     * @param[in] tryIgnoreNodePositions Does not add node geometries if geom.size()>=2
     */
    void buildInnerEdges(const NBNode& n, int noInternalNoSplits, int& linkIndex, int& splitIndex);

    /// @brief get Signs
    inline const std::vector<NBSign>& getSigns() const {
        return mySigns;
    }

    /// @brief add Sign
    inline void addSign(NBSign sign) {
        mySigns.push_back(sign);
    }

    /// @brief cut shape at the intersection shapes
    PositionVector cutAtIntersection(const PositionVector& old) const;

    /// @brief Set Node border
    void setNodeBorder(const NBNode* node, const Position& p, const Position& p2, bool rectangularCut);
    void resetNodeBorder(const NBNode* node);

private:
    /** @class ToEdgeConnectionsAdder
     * @brief A class that being a bresenham-callback assigns the incoming lanes to the edges
     */
    class ToEdgeConnectionsAdder : public Bresenham::BresenhamCallBack {
    private:
        /// @brief map of edges to this edge's lanes that reach them
        std::map<NBEdge*, std::vector<int> > myConnections;

        /// @brief the transition from the virtual lane to the edge it belongs to
        const EdgeVector& myTransitions;

    public:
        /// @brief constructor
        ToEdgeConnectionsAdder(const EdgeVector& transitions)
            : myTransitions(transitions) { }

        /// @brief destructor
        ~ToEdgeConnectionsAdder() { }

        /// @brief executes a bresenham - step
        void execute(const int lane, const int virtEdge);

        /// @brief get built connections
        const std::map<NBEdge*, std::vector<int> >& getBuiltConnections() const {
            return myConnections;
        }

    private:
        /// @brief Invalidated copy constructor.
        ToEdgeConnectionsAdder(const ToEdgeConnectionsAdder&);

        /// @brief Invalidated assignment operator.
        ToEdgeConnectionsAdder& operator=(const ToEdgeConnectionsAdder&);
    };


    /**
     * @class MainDirections
     * @brief Holds (- relative to the edge it is build from -!!!) the list of
     * main directions a vehicle that drives on this street may take on
     * the junction the edge ends in
     * The back direction is not regarded
     */
    class MainDirections {
    public:
        /// @brief enum of possible directions
        enum Direction { DIR_RIGHTMOST, DIR_LEFTMOST, DIR_FORWARD };

        /// @brief list of the main direction within the following junction relative to the edge
        std::vector<Direction> myDirs;

    public:
        /// @brief constructor
        MainDirections(const EdgeVector& outgoing, NBEdge* parent, NBNode* to, int indexOfStraightest);

        /// @brief destructor
        ~MainDirections();

        /// @brief returns the information whether no following street has a higher priority
        bool empty() const;

        /// @brief returns the information whether the street in the given direction has a higher priority
        bool includes(Direction d) const;

    private:
        /// @brief Invalidated copy constructor.
        MainDirections(const MainDirections&);

        /// @brief Invalidated assignment operator.
        MainDirections& operator=(const MainDirections&);
    };

    /// @brief Computes the shape for the given lane
    PositionVector computeLaneShape(int lane, double offset) const;

    /// @brief compute lane shapes
    void computeLaneShapes();

private:
    /** @brief Initialization routines common to all constructors
     *
     * Checks whether the number of lanes>0, whether the junction's from-
     *  and to-nodes are given (!=0) and whether they are distict. Throws
     *  a ProcessError if any of these checks fails.
     *
     * Adds the nodes positions to geometry if it shall not be ignored or
     *  if the geometry is empty.
     *
     * Computes the angle and length, and adds this edge to its node as
     *  outgoing/incoming. Builds lane informations.
     *
     * @param[in] noLanes The number of lanes this edge has
     * @param[in] tryIgnoreNodePositions Does not add node geometries if geom.size()>=2
     * @param[in] origID The original ID this edge had
     */
    void init(int noLanes, bool tryIgnoreNodePositions, const std::string& origID);

    /// @brief divides the lanes on the outgoing edges
    void divideOnEdges(const EdgeVector* outgoing);

    /// @brief divide selected lanes on edges
    void divideSelectedLanesOnEdges(const EdgeVector* outgoing, const std::vector<int>& availableLanes, const std::vector<int>* priorities);

    /// @brief add some straight connections
    void addStraightConnections(const EdgeVector* outgoing, const std::vector<int>& availableLanes, const std::vector<int>* priorities);

    /// @brief recomputes the edge priorities and manipulates them for a distribution of lanes on edges which is more like in real-life
    std::vector<int>* prepareEdgePriorities(const EdgeVector* outgoing);

    /// @brief computes the sum of the given list's entries (sic!)
    static int computePrioritySum(const std::vector<int>& priorities);

    /// @name Setting and getting connections
    /// @{
    /** @briefmoves a connection one place to the left;
     * @note Attention! no checking for field validity
     */
    void moveConnectionToLeft(int lane);

    /** @briefmoves a connection one place to the right;
     * @noteAttention! no checking for field validity
     */
    void moveConnectionToRight(int lane);

    /// @brief whether the connection can originate on newFromLane
    bool canMoveConnection(const Connection& con, int newFromLane) const;
    /// @}

    /** returns a modified version of laneShape which starts at the outside of startNode. laneShape may be shorted or extended
     * @note see [wiki:Developer/Network_Building_Process]
     */
    PositionVector startShapeAt(const PositionVector& laneShape, const NBNode* startNode, PositionVector nodeShape) const;

    /// @brief computes the angle of this edge and stores it in myAngle
    void computeAngle();

    /// @brief compute the first intersection point between the given lane geometries considering their rspective widths
    static double firstIntersection(const PositionVector& v1, const PositionVector& v2, double width2);

    /// @brief add a lane of the given width, restricted to the given class and shift existing connections
    void addRestrictedLane(double width, SUMOVehicleClass vclass);

    /// @brief restore a restricted lane
    void restoreRestrictedLane(SUMOVehicleClass vclass, std::vector<NBEdge::Lane> oldLanes, PositionVector oldGeometry, std::vector<NBEdge::Connection> oldConnections);

private:
    /** @brief The building step
     * @see EdgeBuildingStep
     */
    EdgeBuildingStep myStep;

    /// @brief The type of the edge
    std::string myType;

    /// @brief The source and the destination node
    NBNode* myFrom, *myTo;

    /// @brief The length of the edge
    double myLength;

    /// @brief The angles of the edge
    /// @{
    double myStartAngle;
    double myEndAngle;
    double myTotalAngle;
    /// @}

    /// @brief The priority of the edge
    int myPriority;

    /// @brief The maximal speed
    double mySpeed;

    /** @brief List of connections to following edges
     * @see Connection
     */
    std::vector<Connection> myConnections;

    /// @brief List of connections marked for delayed removal
    std::vector<Connection> myConnectionsToDelete;

    /// @brief The turn destination edge (if a connection exists)
    NBEdge* myTurnDestination;

    /// @brief The edge that would be the turn destination if there was one
    NBEdge* myPossibleTurnDestination;

    /// @brief The priority normalised for the node the edge is outgoing of
    int myFromJunctionPriority;

    /// @brief The priority normalised for the node the edge is incoming in
    int myToJunctionPriority;

    /// @brief The geometry for the edge
    PositionVector myGeom;

    /// @brief The information about how to spread the lanes
    LaneSpreadFunction myLaneSpreadFunction;

    /// @brief This edges's offset to the intersection begin (will be applied to all lanes)
    double myEndOffset;

    /// @brief This width of this edge's lanes
    double myLaneWidth;

    /** @brief Lane information
     * @see Lane
     */
    std::vector<Lane> myLanes;

    /// @brief An optional length to use (-1 if not valid)
    double myLoadedLength;

    /// @brief Information whether this is a junction-inner edge
    bool myAmInnerEdge;

    /// @brief Information whether this edge is a (macroscopic) connector
    bool myAmMacroscopicConnector;

    /// @brief TLS Disabled Connections
    struct TLSDisabledConnection {
        int fromLane;
        NBEdge* to;
        int toLane;
    };

    /// @brief vector with the disabled connections
    std::vector<TLSDisabledConnection> myTLSDisabledConnections;

    /// @brief The street name (or whatever arbitrary string you wish to attach)
    std::string myStreetName;

    /// @brief the street signs along this edge
    std::vector<NBSign> mySigns;

    /// @brief the offset of a traffic light signal from the end of this edge (-1 for None)
    double mySignalOffset;

    /// @brief intersection borders (because the node shape might be invalid)
    /// @{
    PositionVector myFromBorder;
    PositionVector myToBorder;
    /// @}

public:
    /// @class tls_disable_finder
    class tls_disable_finder {
    public:
        /// @brief constructor
        tls_disable_finder(const TLSDisabledConnection& tpl) : myDefinition(tpl) { }

        /// @brief operator ()
        bool operator()(const TLSDisabledConnection& e) const {
            if (e.to != myDefinition.to) {
                return false;
            }
            if (e.fromLane != myDefinition.fromLane) {
                return false;
            }
            if (e.toLane != myDefinition.toLane) {
                return false;
            }
            return true;
        }

    private:
        /// @brief definition of disable connection
        TLSDisabledConnection myDefinition;

    private:
        /// @brief invalidated assignment operator
        tls_disable_finder& operator=(const tls_disable_finder& s);
    };


    /// @class connections_toedge_finder
    class connections_toedge_finder {
    public:
        /// @brief constructor
        connections_toedge_finder(const NBEdge* const edge2find, bool hasFromLane = false) :
            myHasFromLane(hasFromLane),
            myEdge2Find(edge2find) { }

        /// @brief operator ()
        bool operator()(const Connection& c) const {
            return c.toEdge == myEdge2Find && (!myHasFromLane || c.fromLane != -1);
        }

    private:
        /// @brief check if has from lane
        const bool myHasFromLane;

        /// @brief edge to find
        const NBEdge* const myEdge2Find;

    private:
        /// @brief invalidated assignment operator
        connections_toedge_finder& operator=(const connections_toedge_finder& s);
    };

    /// @class connections_toedgelane_finder
    class connections_toedgelane_finder {
    public:
        /// @brief constructor
        connections_toedgelane_finder(NBEdge* const edge2find, int lane2find, int fromLane2find) :
            myEdge2Find(edge2find),
            myLane2Find(lane2find),
            myFromLane2Find(fromLane2find) { }

        /// @brief operator ()
        bool operator()(const Connection& c) const {
            return c.toEdge == myEdge2Find && c.toLane == myLane2Find && (myFromLane2Find < 0 || c.fromLane == myFromLane2Find);
        }

    private:
        /// @brief edge to find
        NBEdge* const myEdge2Find;

        /// @brief lane to find
        int myLane2Find;

        /// @brief from lane to find
        int myFromLane2Find;

    private:
        /// @brief invalidated assignment operator
        connections_toedgelane_finder& operator=(const connections_toedgelane_finder& s);

    };

    /// @class connections_finder
    class connections_finder {
    public:
        /// @brief constructor
        connections_finder(int fromLane, NBEdge* const edge2find, int lane2find, bool invertEdge2find = false) :
            myFromLane(fromLane), myEdge2Find(edge2find), myLane2Find(lane2find), myInvertEdge2find(invertEdge2find) { }

        /// @brief operator ()
        bool operator()(const Connection& c) const {
            return ((c.fromLane == myFromLane || myFromLane == -1)
                    && ((!myInvertEdge2find && c.toEdge == myEdge2Find) || (myInvertEdge2find && c.toEdge != myEdge2Find))
                    && (c.toLane == myLane2Find || myLane2Find == -1));
        }

    private:
        /// @brief index of from lane
        int myFromLane;

        /// @brief edge to find
        NBEdge* const myEdge2Find;

        /// @brief lane to find
        int myLane2Find;

        /// @brief invert edge to find
        bool myInvertEdge2find;

    private:
        /// @brief invalidated assignment operator
        connections_finder& operator=(const connections_finder& s);

    };

    /// @class connections_conflict_finder
    class connections_conflict_finder {
    public:
        /// @brief constructor
        connections_conflict_finder(int fromLane, NBEdge* const edge2find, bool checkRight) :
            myFromLane(fromLane), myEdge2Find(edge2find), myCheckRight(checkRight) { }

        /// @brief operator ()
        bool operator()(const Connection& c) const {
            return (((myCheckRight && c.fromLane < myFromLane) || (!myCheckRight && c.fromLane > myFromLane))
                    && c.fromLane >= 0 // already assigned
                    && c.toEdge == myEdge2Find);
        }

    private:
        /// @brief index of from lane
        int myFromLane;

        /// @brief edge to find
        NBEdge* const myEdge2Find;

        /// @brief check if is right
        bool myCheckRight;

    private:
        /// @brief invalidated assignment operator
        connections_conflict_finder& operator=(const connections_conflict_finder& s);

    };

    /// @class connections_fromlane_finder
    class connections_fromlane_finder {
    public:
        /// @briefconstructor
        connections_fromlane_finder(int lane2find) : myLane2Find(lane2find) { }

        /// @brief operator ()
        bool operator()(const Connection& c) const {
            return c.fromLane == myLane2Find;
        }

    private:
        /// @brief index of lane to find
        int myLane2Find;

    private:
        /// @brief invalidated assignment operator
        connections_fromlane_finder& operator=(const connections_fromlane_finder& s);

    };

    /// @brief connections_sorter sort by fromLane, toEdge and toLane
    static bool connections_sorter(const Connection& c1, const Connection& c2);

    /**
     * @class connections_relative_edgelane_sorter
     * @brief Class to sort edges by their angle
     */
    class connections_relative_edgelane_sorter {
    public:
        /// @brief constructor
        explicit connections_relative_edgelane_sorter(NBEdge* e) : myEdge(e) {}

    public:
        /// @brief comparing operation
        int operator()(const Connection& c1, const Connection& c2) const;

    private:
        /// @brief the edge to compute the relative angle of
        NBEdge* myEdge;
    };

private:
    /// @brief invalidated copy constructor
    NBEdge(const NBEdge& s);

    /// @brief invalidated assignment operator
    NBEdge& operator=(const NBEdge& s);
};


#endif

/****************************************************************************/

