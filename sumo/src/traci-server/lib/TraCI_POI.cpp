/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2017-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    TraCI_POI.cpp
/// @author  Daniel Krajzewicz
/// @author  Mario Krumnow
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Robert Hilbrich
/// @date    30.05.2012
/// @version $Id$
///
// C++ TraCI client API implementation
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/shapes/PointOfInterest.h>
#include <utils/shapes/ShapeContainer.h>
#include <microsim/MSNet.h>
#include "TraCI_POI.h"
#include "TraCI.h"




// ===========================================================================
// member definitions
// ===========================================================================
std::vector<std::string>
TraCI_POI::getIDList() {
    std::vector<std::string> ids;
    ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();
    shapeCont.getPOIs().insertIDs(ids);
    return ids;
}

int
TraCI_POI::getIDCount() {
    return (int)getIDList().size();
}

std::string
TraCI_POI::getType(const std::string& poiID) {
    return getPoI(poiID)->getType();
}

TraCIColor
TraCI_POI::getColor(const std::string& poiID) {
    PointOfInterest* sumoPoi = getPoI(poiID);
    RGBColor col = sumoPoi->getColor();
    return TraCI::makeTraCIColor(col);
}

TraCIPosition
TraCI_POI::getPosition(const std::string& poiID) {
    TraCIPosition pos;
    PointOfInterest* p = getPoI(poiID);
    pos.x = p->x();
    pos.y = p->y();
    pos.z = p->z();
    return pos;
}

std::string
TraCI_POI::getParameter(const std::string& poiID, const std::string& param) {
    PointOfInterest* p = getPoI(poiID);
    return p->getParameter(param, "");
}

void
TraCI_POI::setType(const std::string& poiID, const std::string& type) {
    PointOfInterest* p = getPoI(poiID);
    p->setType(type);
}

void
TraCI_POI::setPosition(const std::string& poiID, const TraCIPosition& pos) {
    PointOfInterest* p = getPoI(poiID);
    p->set(TraCI::makePosition(pos));
}

void
TraCI_POI::setColor(const std::string& poiID, const TraCIColor& c) {
    PointOfInterest* p = getPoI(poiID);
    p->setColor(TraCI::makeRGBColor(c));
}

bool
TraCI_POI::add(const std::string& poiID, const TraCIPosition& pos, const TraCIColor& c, const std::string& type, int layer) {
    ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();
    return shapeCont.addPOI(poiID, type, TraCI::makeRGBColor(c), TraCI::makePosition(pos), false, "", 0, 0, (double) layer,
                            Shape::DEFAULT_ANGLE,
                            Shape::DEFAULT_IMG_FILE,
                            Shape::DEFAULT_IMG_WIDTH,
                            Shape::DEFAULT_IMG_HEIGHT);
}

bool
TraCI_POI::remove(const std::string& poiID, int /* layer */) {
    ShapeContainer& shapeCont = MSNet::getInstance()->getShapeContainer();
    return shapeCont.removePOI(poiID);
}

void
TraCI_POI::setParameter(const std::string& poiID, const std::string& param, const std::string& value) {
    PointOfInterest* p = getPoI(poiID);
    p->setParameter(param, value);
}


PointOfInterest*
TraCI_POI::getPoI(const std::string& id) {
    PointOfInterest* sumoPoi = MSNet::getInstance()->getShapeContainer().getPOIs().get(id);
    if (sumoPoi == 0) {
        throw TraCIException("POI '" + id + "' is not known");
    }
    return sumoPoi;
}


/****************************************************************************/
