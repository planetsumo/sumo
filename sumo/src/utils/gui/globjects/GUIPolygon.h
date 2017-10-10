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
/// @file    GUIPolygon.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    June 2006
/// @version $Id$
///
// The GUI-version of a polygon
/****************************************************************************/
#ifndef GUIPolygon_h
#define GUIPolygon_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/foxtools/MFXMutex.h>
#include <utils/shapes/SUMOPolygon.h>
#include <utils/gui/globjects/GUIGlObject_AbstractAdd.h>
#include <utils/gui/globjects/GLIncludes.h>
#include <utils/gui/settings/GUIVisualizationSettings.h>


// ===========================================================================
// class definitions
// ===========================================================================
/*
 * @class GUIPolygon
 * @brief The GUI-version of a polygon
 */
class GUIPolygon : public SUMOPolygon, public GUIGlObject_AbstractAdd {
public:
    /** @brief Constructor
     * @param[in] id The name of the polygon
     * @param[in] type The (abstract) type of the polygon
     * @param[in] color The color of the polygon
     * @param[in] layer The layer of the polygon
     * @param[in] angle The rotation of the polygon
     * @param[in] imgFile The raster image of the polygon
     * @param[in] shape The shape of the polygon
     * @param[in] geo specifiy if shape was loaded as GEO
     * @param[in] fill Whether the polygon shall be filled
     */
    GUIPolygon(const std::string& id, const std::string& type,
               const RGBColor& color, const PositionVector& shape, bool geo, bool fill,
               double layer = 0, double angle = 0, const std::string& imgFile = "");

    /// @brief Destructor
    ~GUIPolygon();


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
    virtual void drawGL(const GUIVisualizationSettings& s) const;
    //@}


    /// @brief set a new shape and update the tesselation
    virtual void setShape(const PositionVector& shape);

    /// @brief set a new shape and update the tesselation
    void setLineWidth(double lineWidth) {
        myLineWidth = lineWidth;
    }

private:
    /// The mutex used to avoid concurrent updates of the shape
    mutable MFXMutex myLock;

    /// @brief id of the display list for the cached tesselation
    mutable GLuint myDisplayList;

    /// @brief the previous line width for deciding whether the display list must be refreshed
    mutable double myLineWidth;

    /// @brief store the drawing commands in a display list
    void storeTesselation(double lineWidth) const;

    // @brief perform the tesselation / drawing
    void performTesselation(double lineWidth) const;

};


#endif

/****************************************************************************/

