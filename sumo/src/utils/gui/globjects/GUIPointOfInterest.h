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
/// @file    GUIPointOfInterest.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    June 2006
/// @version $Id$
///
// missing_desc
/****************************************************************************/
#ifndef GUIPointOfInterest_h
#define GUIPointOfInterest_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/shapes/PointOfInterest.h>
#include <utils/gui/globjects/GUIGlObject_AbstractAdd.h>
#include <utils/gui/settings/GUIVisualizationSettings.h>


// ===========================================================================
// class declarations
// ===========================================================================


// ===========================================================================
// class definitions
// ===========================================================================
/*
 * @class GUIPointOfInterest
 * @brief The GUI-version of a point of interest
 */
class GUIPointOfInterest : public PointOfInterest, public GUIGlObject_AbstractAdd {
public:
    /** @brief Constructor
     * @param[in] id The name of the POI
     * @param[in] type The (abstract) type of the POI
     * @param[in] color The color of the POI
     * @param[in] pos The position of the POI
     * @param[in[ geo use GEO coordinates (lon/lat)
     * @param[in] lane The Lane in which this POI is placed
     * @param[in] posOverLane The position over Lane
     * @param[in] posLat The position lateral over Lane
     * @param[in] layer The layer of the POI
     * @param[in] angle The rotation of the POI
     * @param[in] imgFile The raster image of the shape
     * @param[in] width The width of the POI image
     * @param[in] height The height of the POI image
     */
    GUIPointOfInterest(const std::string& id, const std::string& type,
                       const RGBColor& color, const Position& pos, bool geo,
                       const std::string &lane, double posOverLane, double posLat,
                       double layer, double angle, const std::string& imgFile,
                       double width, double height);

    /// @brief Destructor
    virtual ~GUIPointOfInterest();



    /// @name inherited from GUIGlObject
    //@{

    /** @brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app,
                                       GUISUMOAbstractView& parent);


    /** @brief Returns an own parameter window
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     * @see GUIGlObject::getParameterWindow
     */
    GUIParameterTableWindow* getParameterWindow(GUIMainWindow& app,
            GUISUMOAbstractView& parent);


    /** @brief Returns the boundary to which the view shall be centered in order to show the object
     *
     * @return The boundary the object is within
     * @see GUIGlObject::getCenteringBoundary
     */
    Boundary getCenteringBoundary() const;


    /** @brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    //@}

};


#endif

/****************************************************************************/

