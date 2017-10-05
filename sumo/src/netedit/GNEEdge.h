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
/// @file    GNEEdge.h
/// @author  Jakob Erdmann
/// @date    Feb 2011
/// @version $Id$
///
// A road/street connecting two junctions (netedit-version, adapted from GUIEdge)
// Basically a container for an NBEdge with drawing and editing capabilities
/****************************************************************************/
#ifndef GNEEdge_h
#define GNEEdge_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GNENetElement.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNENet;
class GNEJunction;
class GNELane;
class GNEConnection;
class GNEAdditional;
class GNERouteProbe;
class GNEVaporizer;
class GNERerouter;
class GNECrossing;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEEdge
 * @brief A road/street connecting two junctions (netedit-version)
 *
 * @see MSEdge
 */
class GNEEdge : public GNENetElement {

    /// @brief Friend class
    friend class GNEChange_Lane;
    friend class GNEChange_Connection;

public:
    /// @brief Definition of the lane's vector
    typedef std::vector<GNELane*> LaneVector;

    /// @brief Definition of the connection's vector
    typedef std::vector<GNEConnection*> ConnectionVector;

    /// @brief Definition of the additionals vector
    typedef std::vector<GNEAdditional*> AdditionalVector;

    /**@brief Constructor.
     * @param[in] nbe The represented edge
     * @param[in] net The net to inform about gui updates
     * @param[in] loaded Whether the edge was loaded from a file
     */
    GNEEdge(NBEdge& nbe, GNENet* net, bool wasSplit = false, bool loaded = false);

    /// @brief Destructor.
    ~GNEEdge();

    /**@brief update pre-computed geometry information
     * @note if current editing mode is Move, connection's geometry will not be updated
     */
    void updateGeometry();

    /**@brief return index of a vertex of shape, or of a new vertex if position is over an shape's edge
    * @param pos position of new/existent vertex
    * @param createIfNoExist enable or disable creation of new verte if there isn't another vertex in position
    * @return index of position vector
    */
    int getVertexIndex(const Position& pos, bool createIfNoExist = true);

    /**@brief return index of a vertex of shape, or of a new vertex if position is over an shape's edge
    * @param offset position over edge
    * @param createIfNoExist enable or disable creation of new verte if there isn't another vertex in position
    * @return index of position vector
    */
    int getVertexIndex(const double offset, bool createIfNoExist = true);

    /**@brief change position of a vertex of shape without commiting change
    * @param[in] index index of Vertex shape
    * @param[in] newPos The new position of vertex
    * @return index of vertex (in some cases index can change
    */
    int moveVertexShape(const int index, const Position& oldPos, const Position &offset);

    /**@brief move entire shape without commiting change
    * @param[in] oldShape the old shape of polygon before moving
    * @param[in] offset the offset of movement
    */
    void moveEntireShape(const PositionVector& oldShape, const Position& offset);

    /**@brief commit geometry changes in the attributes of an element after use of changeShapeGeometry(...)
    * @param[in] oldShape the old shape of polygon
    * @param[in] undoList The undoList on which to register changes
    */
    void commitShapeChange(const PositionVector& oldShape, GNEUndoList* undoList);

    /// @brief delete the geometry point closest to the given pos
    void deleteGeometryPoint(const Position& pos, bool allowUndo = true);

    /// @brief update edge geometry after junction move
    void updateJunctionPosition(GNEJunction* junction, const Position& origPos);

    /// @brief Returns the street's geometry
    Boundary getBoundary() const;

    /// @brief return true if edge is inverted (Angle between origin and destiny junction is -PI/2 <= angle < PI/2
    bool isInverted() const;

    /// @name inherited from GUIGlObject
    /// @{
    /**@brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent);

    /**@brief Returns the boundary to which the view shall be centered in order to show the object
     *
     * @return The boundary the object is within
     * @see GUIGlObject::getCenteringBoundary
     */
    Boundary getCenteringBoundary() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    /// @}

    /// @brief returns the internal NBEdge
    NBEdge* getNBEdge();

    /// @brief returns the source-junction
    GNEJunction* getGNEJunctionSource() const;

    /// @brief returns the destination-junction
    GNEJunction* getGNEJunctionDestiny() const;

    /// @brief get opposite edge
    GNEEdge* getOppositeEdge() const;

    /// @brief makes pos the new geometry endpoint at the appropriate end
    void setEndpoint(Position pos, GNEUndoList* undoList);

    /// @brief restores the endpoint to the junction position at the appropriate end
    void resetEndpoint(const Position& pos, GNEUndoList* undoList);

    /// @name inherited from GNEAttributeCarrier
    /// @{
    /* @brief method for getting the Attribute of an XML key
     * @param[in] key The attribute key
     * @return string with the value associated to key
     */
    std::string getAttribute(SumoXMLAttr key) const;
    std::string getAttributeForSelection(SumoXMLAttr key) const;

    /* @brief method for setting the attribute and letting the object perform additional changes
     * @param[in] key The attribute key
     * @param[in] value The new value
     * @param[in] undoList The undoList on which to register changes
     */
    void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList);

    /* @brief method for setting the attribute and letting the object perform additional changes
     * @param[in] key The attribute key
     * @param[in] value The new value
     * @param[in] undoList The undoList on which to register changes
     */
    bool isValid(SumoXMLAttr key, const std::string& value);
    /// @}

    /// @brief set responsibility for deleting internal strctures
    void setResponsible(bool newVal);

    /**@brief update edge geometry and inform the lanes
     * @param[in] geom The new geometry
     * @param[in] inner Whether geom is only the inner points
     */
    void setGeometry(PositionVector geom, bool inner);

    /// @brief remake connections
    void remakeGNEConnections();

    /// @brief copy edge attributes from tpl
    void copyTemplate(GNEEdge* tpl, GNEUndoList* undolist);

    /// @brief returns GLIDs of all lanes
    std::set<GUIGlID> getLaneGlIDs();

    /// @brief returns a reference to the lane vector
    const std::vector<GNELane*>& getLanes();

    /// @brief returns a reference to the GNEConnection vector
    const std::vector<GNEConnection*>& getGNEConnections();

    /// @brief get GNEConnection if exist, and if not create it if create is enabled
    GNEConnection* retrieveGNEConnection(int fromLane, NBEdge* to, int toLane, bool createIfNoExist = true);

    /// @brief whether this edge was created from a split
    bool wasSplit();

    /* @brief compute a splitting position which keeps the resulting edges
     * straight unless the user clicked near a geometry point */
    Position getSplitPos(const Position& clickPos);

    /// @brief override to also set lane ids
    void setMicrosimID(const std::string& newID);

    /// @brief add additional child to this edge
    void addAdditionalChild(GNEAdditional* additional);

    /// @brief remove additional child from this edge
    void removeAdditionalChild(GNEAdditional* additional);

    /// @brief return list of additionals associated with this edge
    const std::vector<GNEAdditional*>& getAdditionalChilds() const;

    /// @brief add a reference to a rerouter that has this edge as parameter
    void addGNERerouter(GNERerouter* rerouter);

    /// @brief remove a reference to a rerouter that has this edge as parameter
    void removeGNERerouter(GNERerouter* rerouter);

    /// @brief get rerouters vinculated with this edge
    const std::vector<GNERerouter*>& getGNERerouters() const;

    /// @brief get number of rerouters that has this edge as parameters
    int getNumberOfGNERerouters() const;

    /// @brief check if edge has a restricted lane
    bool hasRestrictedLane(SUMOVehicleClass vclass) const;

    // the radius in which to register clicks for geometry nodes
    static const double SNAP_RADIUS;

    /// @brief clear current connections
    void clearGNEConnections();

    /// @brief obtain relative positions of RouteProbes
    int getRouteProbeRelativePosition(GNERouteProbe* routeProbe) const;

    /// @brief obtain relative positions of Vaporizer
    int getVaporizerRelativePosition(GNEVaporizer* vaporizer) const;

    /// @brief get GNECrossings vinculated with this Edge
    std::vector<GNECrossing*> getGNECrossings();

protected:
    /// @brief the underlying NBEdge
    NBEdge& myNBEdge;

    /// @brief pointer to GNEJunction source
    GNEJunction* myGNEJunctionSource;

    /// @brief pointer to GNEJunction destiny
    GNEJunction* myGNEJunctionDestiny;

    /// @brief restore point for undo
    PositionVector myOrigShape;

    /// @brief vectgor with the lanes of this edge
    LaneVector myLanes;

    /// @brief vector with the connections of this edge
    ConnectionVector myGNEConnections;

    /// @brief whether we are responsible for deleting myNBNode
    bool myAmResponsible;

    /// @brief whether this edge was created from a split
    bool myWasSplit;

    /// @brief modification status of the connections
    std::string myConnectionStatus;

    /// @brief list with the additionals vinculated with this edge
    AdditionalVector myAdditionals;

    /// @brief list of reroutes that has this edge as parameter
    std::vector<GNERerouter*> myReroutes;

private:
    /// @brief set attribute after validation
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /**@brief changes the number of lanes.
     * When reducing the number of lanes, higher-numbered lanes are removed first.
     * When increasing the number of lanes, the last known attributes for a lane
     * with this number are restored. If none are found the attributes for the
     * leftmost lane are copied
     */
    void setNumLanes(int numLanes, GNEUndoList* undoList);

    /// @brief@brief increase number of lanes by one use the given attributes and restore the GNELane
    void addLane(GNELane* lane, const NBEdge::Lane& laneAttrs);

    /// @briefdecrease the number of lanes by one. argument is only used to increase robustness (assertions)
    void removeLane(GNELane* lane);

    /// @brief adds a connection
    void addConnection(NBEdge::Connection nbCon, bool selectAfterCreation = false);

    /// @brief removes a connection
    void removeConnection(NBEdge::Connection nbCon);

    /// @brief remove crossing of junction
    void removeEdgeFromCrossings(GNEJunction* junction, GNEUndoList* undoList);

    /// @brief invalidated copy constructor
    GNEEdge(const GNEEdge& s) = delete;

    /// @brief invalidated assignment operator
    GNEEdge& operator=(const GNEEdge& s) = delete;
};


#endif

/****************************************************************************/

