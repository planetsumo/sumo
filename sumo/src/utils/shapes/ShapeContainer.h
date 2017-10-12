/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2005-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    ShapeContainer.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    2005-09-15
/// @version $Id$
///
// Storage for geometrical objects, sorted by the layers they are in
/****************************************************************************/
#ifndef ShapeContainer_h
#define ShapeContainer_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/common/NamedObjectCont.h>
#include "PointOfInterest.h"
#include "SUMOPolygon.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class ShapeContainer
 * @brief Storage for geometrical objects
 */
class ShapeContainer {
public:

    /// @brief containers
    typedef NamedObjectCont<SUMOPolygon*> Polygons;
    typedef NamedObjectCont<PointOfInterest*> POIs;

    /// @brief Constructor
    ShapeContainer();

    /// @brief Destructor
    virtual ~ShapeContainer();

    /** @brief Builds a polygon using the given values and adds it to the container
     * @param[in] id The name of the polygon
     * @param[in] type The (abstract) type of the polygon
     * @param[in] color The color of the polygon
     * @param[in] layer The layer of the polygon
     * @param[in] angle The rotation of the polygon
     * @param[in] imgFile The raster image of the polygon
     * @param[in] shape The shape of the polygon
     * @param[in] geo specify if shape was loaded as GEO coordinate
     * @param[in] fill Whether the polygon shall be filled
     * @return whether the polygon could be added
     */
    virtual bool addPolygon(const std::string& id, const std::string& type,
                            const RGBColor& color, double layer,
                            double angle, const std::string& imgFile,
                            const PositionVector& shape, bool geo, 
                            bool fill, bool ignorePruning = false);

    /** @brief Builds a POI using the given values and adds it to the container
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
     * @param[in] imgFile The raster image of the POI
     * @param[in] width The width of the POI image
     * @param[in] height The height of the POI image
     * @return whether the poi could be added
     */
    virtual bool addPOI(const std::string& id, const std::string& type, const RGBColor& color, const Position& pos, bool geo,
                        const std::string &lane, double posOverLane, double posLat, double layer, double angle, 
                        const std::string& imgFile, double width, double height, bool ignorePruning = false);

    /** @brief Removes a polygon from the container
     * @param[in] id The id of the polygon
     * @return Whether the polygon could be removed
     */
    virtual bool removePolygon(const std::string& id);

    /** @brief Removes a PoI from the container
     * @param[in] id The id of the PoI
     * @return Whether the poi could be removed
     */
    virtual bool removePOI(const std::string& id);

    /** @brief Assigns a new position to the named PoI
     * @param[in] id The id of the PoI to move
     * @param[in] pos The PoI's new position
     */
    virtual void movePOI(const std::string& id, const Position& pos);

    /** @brief Assigns a shape to the named polygon
     * @param[in] id The id of the polygon to reshape
     * @param[in] shape The polygon's new shape
     */
    virtual void reshapePolygon(const std::string& id, const PositionVector& shape);

    /// @brief Returns all polygons
    inline const Polygons& getPolygons() const {
        return myPolygons;
    }

    /// @brief Returns all pois
    inline const POIs& getPOIs() const {
        return myPOIs;
    }

protected:
    /// @brief add polygon
    virtual bool add(SUMOPolygon* poly, bool ignorePruning = false);

    /// @brief add poi
    virtual bool add(PointOfInterest* poi, bool ignorePruning = false);

protected:
    /// @brief stored Polygons
    Polygons myPolygons;

    /// @brief stored POIs
    POIs myPOIs;
};


#endif

/****************************************************************************/

