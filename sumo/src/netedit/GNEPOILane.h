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
/// @file    GNEPOI.h
/// @author  Pablo Alvarez Lopez
/// @date    Oct 2017
/// @version $Id: GNEPOI.h 26372 2017-10-07 12:33:09Z palcraft $
///
// A class for visualizing and editing POIS in netedit over lanes (adapted from
// GUIPointOfInterest and NLHandler)
/****************************************************************************/
#ifndef GNEPOILane_h
#define GNEPOILane_h


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
class GNELane;

// ===========================================================================
// class definitions
// ===========================================================================
/**
* @class GNEPOI
*
* In the case the represented junction's shape is empty, the boundary
*  is computed using the junction's position to which an offset of 1m to each
*  side is added.
*/
class GNEPOILane : public GUIPointOfInterest, public GNEShape {

public:
    /// @brief needed to avoid diamond Problem between GUIPolygon and GNEShape
    using GNEShape::getID;

    /** @brief Constructor
    * @param[in] net net in which this polygon is placed
    * @param[in] id The name of the POI
    * @param[in] type The (abstract) type of the POI
    * @param[in] color The color of the POI
    * @param[in] layer The layer of the POI
    * @param[in] angle The rotation of the POI
    * @param[in] imgFile The raster image of the shape
    * @param[in] lane lane in which tis POILane is placed
    * @param[in] posOverLane Position over lane in which this POILane is placed
    * @param[in] posLat Lateral position over lane
    * @param[in] width The width of the POI image
    * @param[in] height The height of the POI image
    * @param[in] movementBlocked if movement of POI is blocked
    */
    GNEPOILane(GNENet* net, const std::string& id, const std::string& type, const RGBColor& color, 
               double layer, double angle, const std::string& imgFile, GNELane *lane, double posOverLane, 
               double posLat, double width, double height, bool movementBlocked);

    /// @brief Destructor
    ~GNEPOILane();

    /**@brief change the position of the element geometry without saving in undoList
    * @param[in] newPosition new position of geometry
    * @note should't be called in drawGL(...) functions to avoid smoothness issues
    */
    void moveGeometry(const Position& oldPos, const Position &offset);

    /**@brief commit geometry changes in the attributes of an element after use of moveGeometry(...)
    * @param[in] oldPos the old position of additional
    * @param[in] undoList The undoList on which to register changes
    */
    void commitGeometryMoving(const Position& oldPos, GNEUndoList* undoList);

    /// @name inherited from GNEShape
    /// @{
    /// @brief update pre-computed geometry information
    void updateGeometry();

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
    /**@brief method for getting the Attribute of an XML key
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

    /**@brief method for checking if the key and their correspond attribute are valids
    * @param[in] key The attribute key
    * @param[in] value The value asociated to key key
    * @return true if the value is valid, false in other case
    */
    bool isValid(SumoXMLAttr key, const std::string& value);
    /// @}

protected:
    /// @brief GNElane in which this POILane is placed
    GNELane* myGNELane;

private:
    /// @brief set attribute after validation
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /// @brief Invalidated copy constructor.
    GNEPOILane(const GNEPOILane&) = delete;

    /// @brief Invalidated assignment operator.
    GNEPOILane& operator=(const GNEPOILane&) = delete;
};


#endif

/****************************************************************************/

