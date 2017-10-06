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
/// @file    GUIVisualizationSettings.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// Stores the information about how to visualize structures
/****************************************************************************/
#ifndef GUIVisualizationSettings_h
#define GUIVisualizationSettings_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <map>
#include <utils/common/RGBColor.h>
#include <utils/common/ToString.h>
#include "GUIPropertySchemeStorage.h"


// ===========================================================================
// class declarations
// ===========================================================================
class BaseSchemeInfoSource;
class OutputDevice;
class GUIVisualizationSettings;


// ===========================================================================
// class definitions
// ===========================================================================

// cannot declare this as inner class because it needs to be used in forward
// declaration (@todo fix inclusion order by removing references to guisim!)
struct GUIVisualizationTextSettings {
    GUIVisualizationTextSettings(bool _show, double _size, RGBColor _color) :
        show(_show), size(_size), color(_color) {}

    bool show;
    double size;
    RGBColor color;

    bool operator==(const GUIVisualizationTextSettings& other) {
        return show == other.show &&
               size == other.size &&
               color == other.color;
    }
    bool operator!=(const GUIVisualizationTextSettings& other) {
        return !((*this) == other);
    }

    void print(OutputDevice& dev, const std::string& name) const {
        dev.writeAttr(name + "_show", show);
        dev.writeAttr(name + "_size", size);
        dev.writeAttr(name + "_color", color);
    }
};


struct GUIVisualizationSizeSettings {
    GUIVisualizationSizeSettings(double _minSize, double _exaggeration = 1.0, bool _constantSize = false) :
        minSize(_minSize), exaggeration(_exaggeration), constantSize(_constantSize) {}

    /// @brief The minimum size to draw this object
    double minSize;
    /// @brief The size exaggeration (upscale)
    double exaggeration;
    // @brief whether the object shall be drawn with constant size regardless of zoom
    bool constantSize;

    bool operator==(const GUIVisualizationSizeSettings& other) {
        return constantSize == other.constantSize &&
               minSize == other.minSize &&
               exaggeration == other.exaggeration;
    }
    bool operator!=(const GUIVisualizationSizeSettings& other) {
        return !((*this) == other);
    }

    void print(OutputDevice& dev, const std::string& name) const {
        dev.writeAttr(name + "_minSize", minSize);
        dev.writeAttr(name + "_exaggeration", exaggeration);
        dev.writeAttr(name + "_constantSize", constantSize);
    }

    /// @brief return the drawing size including exaggeration and constantSize values
    double getExaggeration(const GUIVisualizationSettings& s, double factor = 20) const;
};


/**
 * @class GUIVisualizationSettings
 * @brief Stores the information about how to visualize structures
 */
class GUIVisualizationSettings {
public:

    /// @brief constructor
    GUIVisualizationSettings(bool _netedit = false);

    /// @brief The name of this setting
    std::string name;

    /// @brief Whether the settings are for Netedit
    bool netedit;

    /// @brief Information whether dithering shall be enabled
    bool dither;

    /// @name Background visualization settings
    //@{

    /// @brief The background color to use
    RGBColor backgroundColor;
    /// @brief Information whether a grid shall be shown
    bool showGrid;
    /// @brief Information about the grid spacings
    double gridXSize, gridYSize;
    //@}


    /// @name lane visualization settings
    //@{

    /// @brief The mesoscopic edge colorer
    GUIColorer edgeColorer;
    /// @brief The mesoscopic edge scaler
    GUIScaler edgeScaler;

    /// @brief this should be set at the same time as MSGlobals::gUseMesoSim
    static bool UseMesoSim;
    /// @brief The lane colorer
    GUIColorer laneColorer;
    /// @brief The lane scaler
    GUIScaler laneScaler;
    /// @brief Information whether lane borders shall be drawn
    bool laneShowBorders;
    /// @brief Information whether link textures (arrows) shall be drawn
    bool showLinkDecals;
    /// @brief Information whether link rules (colored bars) shall be drawn
    bool showLinkRules;
    /// @brief Information whether rails shall be drawn
    bool showRails;
    // Setting bundles for optional drawing names with size and color
    GUIVisualizationTextSettings edgeName, internalEdgeName, cwaEdgeName, streetName;

    bool hideConnectors;
    /// @brief The lane exaggeration (upscale thickness)
    double laneWidthExaggeration;
    /// @brief The minimum visual lane width for drawing
    double laneMinSize;
    /// @brief Whether to show direction indicators for lanes
    bool showLaneDirection;
    /// @brief Whether to show sublane boundaries
    bool showSublanes;
    //@}


    /// @name vehicle visualization settings
    //@{

    /// @brief The vehicle colorer
    GUIColorer vehicleColorer;
    /// @brief The quality of vehicle drawing
    int vehicleQuality;
    /// @brief Information whether vehicle blinkers shall be drawn
    bool showBlinker;
    /// @brief Information whether the lane change preference shall be drawn
    bool drawLaneChangePreference;
    /// @brief Information whether the minimum gap shall be drawn
    bool drawMinGap;
    /// @brief Information whether the communication range shall be drawn
    bool showBTRange;
    // Setting bundles for controling the size of the drawn vehicles
    GUIVisualizationSizeSettings vehicleSize;
    // Setting bundles for optional drawing vehicle names
    GUIVisualizationTextSettings vehicleName;
    //@}


    /// @name person visualization settings
    //@{

    /// @brief The person colorer
    GUIColorer personColorer;
    /// @brief The quality of person drawing
    int personQuality;
    // Setting bundles for controling the size of the drawn persons
    GUIVisualizationSizeSettings personSize;
    // Setting bundles for optional drawing person names
    GUIVisualizationTextSettings personName;
    //@}


    /// @name container visualization settings
    //@{

    /// @brief The container colorer
    GUIColorer containerColorer;
    /// @brief The quality of container drawing
    int containerQuality;
    // Setting bundles for controling the size of the drawn containers
    GUIVisualizationSizeSettings containerSize;
    // Setting bundles for optional drawing person names
    GUIVisualizationTextSettings containerName;
    //@}


    /// @name junction visualization settings
    //@{

    /// @brief The junction colorer
    GUIColorer junctionColorer;
    // Setting bundles for optional drawing junction names and indices
    GUIVisualizationTextSettings drawLinkTLIndex, drawLinkJunctionIndex, junctionName, internalJunctionName;
    /// @brief Information whether lane-to-lane arrows shall be drawn
    bool showLane2Lane;
    /// @brief whether the shape of the junction should be drawn
    bool drawJunctionShape;
    /// @brief whether crosings and walkingareas shall be drawn
    bool drawCrossingsAndWalkingareas;
    // Setting bundles for controling the size of the drawn junction
    GUIVisualizationSizeSettings junctionSize;
    //@}


    /// @name Additional structures visualization settings
    //@{

    /// @brief The additional structures visualization scheme
    // @todo decouple addExageration for POIs, Polygons, Triggers etc
    int addMode;
    // Setting bundles for controling the size of additional items
    GUIVisualizationSizeSettings addSize;
    // Setting bundles for optional drawing additional names
    GUIVisualizationTextSettings addName;
    // Setting bundles for optional drawing additional full names
    GUIVisualizationTextSettings addFullName;
    //@}


    /// @name shapes visualization settings
    //@{

    // Setting bundles for controling the size of the drawn POIs
    GUIVisualizationSizeSettings poiSize;
    // Setting bundles for optional drawing poi names
    GUIVisualizationTextSettings poiName;
    // Setting bundles for optional drawing poi types
    GUIVisualizationTextSettings poiType;

    // Setting bundles for controling the size of the drawn polygons
    GUIVisualizationSizeSettings polySize;
    // Setting bundles for optional drawing polygon names
    GUIVisualizationTextSettings polyName;
    // Setting bundles for optional drawing polygon types
    GUIVisualizationTextSettings polyType;
    //@}

    /// @brief Information whether the size legend shall be drawn
    bool showSizeLegend;

    /// @brief information about a lane's width (temporary, used for a single view)
    double scale;

    /// @brief whether the application is in gaming mode or not
    bool gaming;

    /// @brief the current NETEDIT mode (temporary)
    int editMode;

    /// @brief the current NETEDIT additional mode (temporary)
    int editAdditionalMode;

    /// @brief the current selection scaling in NETEDIT (temporary)
    double selectionScale;

    /// @brief whether drawing is performed for the purpose of selecting objects
    bool drawForSelecting;

    /// @brief init default settings
    void initNeteditDefaults();
    void initSumoGuiDefaults();

    /** @brief Writes the settings into an output device
     * @param[in] dev The device to write the settings into
     */
    void save(OutputDevice& dev) const;

    /** @brief Returns the number of the active lane (edge) coloring schme
     * @return number of the active scheme
     */
    int getLaneEdgeMode() const;

    /** @brief Returns the number of the active lane (edge) scaling schme
     * @return number of the active scheme
     */
    int getLaneEdgeScaleMode() const;

    /** @brief Returns the current lane (edge) coloring schme
     * @return current scheme
     */
    GUIColorScheme& getLaneEdgeScheme();

    /** @brief Returns the current lane (edge) scaling schme
     * @return current scheme
     */
    GUIScaleScheme& getLaneEdgeScaleScheme();

    /** @brief Comparison operator */
    bool operator==(const GUIVisualizationSettings& vs2);

    /* @brief map from LinkState to color constants  */
    static const RGBColor& getLinkColor(const LinkState& ls);

    static const RGBColor SUMO_color_E3Entry;
    static const RGBColor SUMO_color_E3Exit;


};


#endif

/****************************************************************************/

