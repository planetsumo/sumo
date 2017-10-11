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
/// @file    GNEPoly.h
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2017
/// @version $Id$
///
// A class for visualizing and editing POIS in netedit (adapted from
// GUIPolygon and NLHandler)
/****************************************************************************/
#ifndef GNEPoly_h
#define GNEPoly_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GNEShape.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GeoConvHelper;
class GNENetElement;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEPoly
 *
 * In the case the represented junction's shape is empty, the boundary
 *  is computed using the junction's position to which an offset of 1m to each
 *  side is added.
 */
class GNEPoly : public GUIPolygon, public GNEShape {

public:
    /// @brief needed to avoid diamond Problem between GUIPolygon and GNEShape
    using GNEShape::getID;

    /** @brief Constructor
     * @param[in] net net in which this polygon is placed
     * @param[in] id The name of the polygon
     * @param[in] type The (abstract) type of the polygon
     * @param[in] shape The shape of the polygon
     * @param[in] geo specifiy if shape was loaded as GEO
     * @param[in] color The color of the polygon
     * @param[in] layer The layer of the polygon
     * @param[in] angle The rotation of the polygon
     * @param[in] imgFile The raster image of the polygon
     * @param[in] fill Whether the polygon shall be filled
     * @param[in] movementBlocked if movement of POI is blocked
     * @param[in] shapeBlocked if shape of POI is blocked
     */
    GNEPoly(GNENet* net, const std::string& id, const std::string& type, const PositionVector& shape, bool geo, bool fill,
            const RGBColor& color, double layer, double angle, const std::string& imgFile, bool movementBlocked, bool shapeBlocked);

    /// @brief Destructor
    ~GNEPoly();

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

    /// @name inherited from GNEShape
    /// @{
    /**@brief writte shape element into a xml file
    * @param[in] device device in which write parameters of additional element
    */
    void writeShape(OutputDevice& device);

    /// @brief Returns position of additional in view
    Position getPositionInView() const;
    /// @}

    /// @name inherited from GUIGlObject
    /// @{
    /**@brief Returns the name of the parent object
     * @return This object's parent id
     */
    const std::string& getParentName() const;

    /**@brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent);

    /**@brief Returns an own parameter window
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     * @see GUIGlObject::getParameterWindow
     */
    GUIParameterTableWindow* getParameterWindow(GUIMainWindow& app, GUISUMOAbstractView& parent);

    /// @brief Returns the boundary to which the view shall be centered in order to show the object
    Boundary getCenteringBoundary() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    /// @}

    /// @name inherited from GNEAttributeCarrier
    /// @{
    /* @brief method for getting the Attribute of an XML key
     * @param[in] key The attribute key
     * @return string with the value associated to key
     */
    std::string getAttribute(SumoXMLAttr key) const;

    /**@brief method for setting the attribute and letting the object perform additional changes
     * @param[in] key The attribute key
     * @param[in] value The new value
     * @param[in] undoList The undoList on which to register changes
     */
    void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList);

    /**@brief method for checking if the key and their conrrespond attribute are valids
     * @param[in] key The attribute key
     * @param[in] value The value asociated to key key
     * @return true if the value is valid, false in other case
     */
    bool isValid(SumoXMLAttr key, const std::string& value);
    /// @}

    /**@brief return index of a vertex of shape, or of a new vertex if position is over an shape's edge
     * @param pos position of new/existent vertex
     * @param createIfNoExist enable or disable creation of new verte if there isn't another vertex in position
     * @return index of position vector
     */
    int getVertexIndex(const Position& pos, bool createIfNoExist = true);

    /// @brief delete the geometry point closest to the given pos
    void deleteGeometryPoint(const Position& pos, bool allowUndo = true);

    /// @brief check if polygon is closed
    bool isPolygonClosed() const;

    /// @brief retrieve the netElement of which the shape is being edited
    void setShapeEditedElement(GNENetElement *element);

    /// @brief retrieve the junction of which the shape is being edited
    GNENetElement* getShapeEditedElement() const;

    /// @brief open polygon
    void openPolygon(bool allowUndo = true);

    /// @brief close polygon
    void closePolygon(bool allowUndo = true);

    /// @brief change first geometry point
    void changeFirstGeometryPoint(int oldIndex, bool allowUndo = true);

    /// @brief replace the current shape with a rectangle
    void simplifyShape(bool allowUndo = true);

protected:
    /// @brief junction of which the shape is being edited (optional)
    GNENetElement* myNetElementShapeEdited;

    /// @brief Latitude of Polygon
    PositionVector myGeoShape;

    /// @brief flag to indicate if polygon is open or closed
    bool myClosedShape;

    /// @brief flag to indicate if polygon is simplified
    bool mySimplifiedShape;

    /// @brief index of vertex that is been moved (-1 means that none vertex is been moved)
    int myCurrentMovingVertexIndex;

private:
    /// @brief hint size of vertex
    static const double myHintSize;

    /// @brief set attribute after validation
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /// @brief Invalidated copy constructor.
    GNEPoly(const GNEPoly&) = delete;

    /// @brief Invalidated assignment operator.
    GNEPoly& operator=(const GNEPoly&) = delete;
};


#endif

/****************************************************************************/

