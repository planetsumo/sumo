/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2008-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    PCLoaderOSM.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Christoph Sommer
/// @author  Michael Behrisch
/// @author  Melanie Knocke
/// @date    Wed, 19.11.2008
/// @version $Id$
///
// A reader of pois and polygons stored in OSM-format
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <map>
#include <fstream>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/common/StringUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/StdDefs.h>
#include <polyconvert/PCPolyContainer.h>
#include "PCLoaderOSM.h"
#include <utils/common/RGBColor.h>
#include <utils/geom/GeomHelper.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/common/FileHelpers.h>

// static members
// ---------------------------------------------------------------------------
const std::set<std::string> PCLoaderOSM::MyKeysToInclude(PCLoaderOSM::initMyKeysToInclude());

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static interface
// ---------------------------------------------------------------------------
std::set<std::string> PCLoaderOSM::initMyKeysToInclude() {
    std::set<std::string> result;
    result.insert("highway");
    result.insert("waterway");
    result.insert("aeroway");
    result.insert("aerialway");
    result.insert("power");
    result.insert("man_made");
    result.insert("building");
    result.insert("leisure");
    result.insert("amenity");
    result.insert("shop");
    result.insert("tourism");
    result.insert("historic");
    result.insert("landuse");
    result.insert("natural");
    result.insert("military");
    result.insert("boundary");
    result.insert("admin_level");
    result.insert("sport");
    result.insert("polygon");
    result.insert("place");
    result.insert("population");
    result.insert("openGeoDB:population");
    result.insert("openGeoDB:name");
    return result;
}

void
PCLoaderOSM::loadIfSet(OptionsCont& oc, PCPolyContainer& toFill,
                       PCTypeMap& tm) {
    if (!oc.isSet("osm-files")) {
        return;
    }
    // parse file(s)
    std::vector<std::string> files = oc.getStringVector("osm-files");
    // load nodes, first
    std::map<long long int, PCOSMNode*> nodes;
    bool withAttributes = oc.getBool("all-attributes");
    MsgHandler* m = OptionsCont::getOptions().getBool("ignore-errors") ? MsgHandler::getWarningInstance() : MsgHandler::getErrorInstance();
    NodesHandler nodesHandler(nodes, withAttributes, *m);
    for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file) {
        // nodes
        if (!FileHelpers::isReadable(*file)) {
            WRITE_ERROR("Could not open osm-file '" + *file + "'.");
            return;
        }
        PROGRESS_BEGIN_MESSAGE("Parsing nodes from osm-file '" + *file + "'");
        if (!XMLSubSys::runParser(nodesHandler, *file)) {
            for (std::map<long long int, PCOSMNode*>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
                delete(*i).second;
            }
            throw ProcessError();
        }
        PROGRESS_DONE_MESSAGE();
    }
    // load relations to see which additional ways may be relevant
    Relations relations;
    RelationsMap additionalWays;
    RelationsHandler relationsHandler(additionalWays, relations, withAttributes, *m);
    for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file) {
        // edges
        PROGRESS_BEGIN_MESSAGE("Parsing relations from osm-file '" + *file + "'");
        XMLSubSys::runParser(relationsHandler, *file);
        PROGRESS_DONE_MESSAGE();
    }

    // load ways
    EdgeMap edges;
    EdgesHandler edgesHandler(nodes, edges, additionalWays, withAttributes, *m);
    for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file) {
        // edges
        PROGRESS_BEGIN_MESSAGE("Parsing edges from osm-file '" + *file + "'");
        XMLSubSys::runParser(edgesHandler, *file);
        PROGRESS_DONE_MESSAGE();
    }

    // build all
    const bool useName = oc.getBool("osm.use-name");
    // instatiate polygons
    for (EdgeMap::iterator i = edges.begin(); i != edges.end(); ++i) {
        PCOSMEdge* e = (*i).second;
        if (e->myAttributes.size() == 0) {
            // cannot be relevant as a polygon
            continue;
        }
        if (e->myCurrentNodes.size() == 0) {
            WRITE_ERROR("Polygon '" + toString(e->id) + "' has no shape.");
            continue;
        }
        // compute shape
        PositionVector vec;
        for (std::vector<long long int>::iterator j = e->myCurrentNodes.begin(); j != e->myCurrentNodes.end(); ++j) {
            PCOSMNode* n = nodes.find(*j)->second;
            Position pos(n->lon, n->lat);
            if (!GeoConvHelper::getProcessing().x2cartesian(pos)) {
                WRITE_WARNING("Unable to project coordinates for polygon '" + toString(e->id) + "'.");
            }
            vec.push_back_noDoublePos(pos);
        }
        const bool ignorePruning = OptionsCont::getOptions().isInStringVector("prune.keep-list", toString(e->id));
        // add as many polygons as keys match defined types
        int index = 0;
        std::string unknownPolyType = "";
        for (std::map<std::string, std::string>::iterator it = e->myAttributes.begin(); it != e->myAttributes.end(); ++it) {
            const std::string& key = it->first;
            const std::string& value = it->second;
            const std::string fullType = key + "." + value;
            if (tm.has(key + "." + value)) {
                index = addPolygon(e, vec, tm.get(fullType), fullType, index, useName, toFill, ignorePruning, withAttributes);
            } else if (tm.has(key)) {
                index = addPolygon(e, vec, tm.get(key), fullType, index, useName, toFill, ignorePruning, withAttributes);
            } else if (MyKeysToInclude.count(key) > 0) {
                unknownPolyType = fullType;
            }
        }
        const PCTypeMap::TypeDef& def = tm.getDefault();
        if (index == 0 && !def.discard && unknownPolyType != "") {
            addPolygon(e, vec, def, unknownPolyType, index, useName, toFill, ignorePruning, withAttributes);
        }
    }


    // instantiate pois
    for (std::map<long long int, PCOSMNode*>::iterator i = nodes.begin(); i != nodes.end(); ++i) {
        PCOSMNode* n = (*i).second;
        if (n->myAttributes.size() == 0) {
            // cannot be relevant as a poi
            continue;
        }
        Position pos(n->lon, n->lat);
        if (!GeoConvHelper::getProcessing().x2cartesian(pos)) {
            WRITE_WARNING("Unable to project coordinates for POI '" + toString(n->id) + "'.");
        }
        const bool ignorePruning = OptionsCont::getOptions().isInStringVector("prune.keep-list", toString(n->id));
        // add as many POIs as keys match defined types
        int index = 0;
        std::string unKnownPOIType = "";
        for (std::map<std::string, std::string>::iterator it = n->myAttributes.begin(); it != n->myAttributes.end(); ++it) {
            const std::string& key = it->first;
            const std::string& value = it->second;
            const std::string fullType = key + "." + value;
            if (tm.has(key + "." + value)) {
                index = addPOI(n, pos, tm.get(fullType), fullType, index, toFill, ignorePruning, withAttributes);
            } else if (tm.has(key)) {
                index = addPOI(n, pos, tm.get(key), fullType, index, toFill, ignorePruning, withAttributes);
            } else if (MyKeysToInclude.count(key) > 0) {
                unKnownPOIType = fullType;
            }
        }
        const PCTypeMap::TypeDef& def = tm.getDefault();
        if (index == 0 && !def.discard && unKnownPOIType != "") {
            addPOI(n, pos, def, unKnownPOIType, index,  toFill, ignorePruning, withAttributes);
        }
    }
    // delete nodes
    for (std::map<long long int, PCOSMNode*>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
        delete(*i).second;
    }
    // delete edges
    for (EdgeMap::iterator i = edges.begin(); i != edges.end(); ++i) {
        delete(*i).second;
    }
    // delete relations
    for (Relations::iterator i = relations.begin(); i != relations.end(); ++i) {
        delete(*i);
    }
}


int
PCLoaderOSM::addPolygon(const PCOSMEdge* edge, const PositionVector& vec, const PCTypeMap::TypeDef& def, const std::string& fullType, int index, bool useName, PCPolyContainer& toFill, bool ignorePruning, bool withAttributes) {
    if (def.discard) {
        return index;
    } else {
        const bool closedShape = vec.front() == vec.back();
        const std::string idSuffix = (index == 0 ? "" : "#" + toString(index));
        const std::string id = def.prefix + (useName && edge->name != "" ? edge->name : toString(edge->id)) + idSuffix;
        SUMOPolygon* poly = new SUMOPolygon(
            StringUtils::escapeXML(id),
            StringUtils::escapeXML(OptionsCont::getOptions().getBool("osm.keep-full-type") ? fullType : def.id),
            def.color, vec, false, def.allowFill && closedShape, (double)def.layer);
        if (withAttributes) {
            poly->updateParameter(edge->myAttributes);
        }
        if (!toFill.add(poly, ignorePruning)) {
            return index;
        } else {
            return index + 1;
        }
    }
}

int
PCLoaderOSM::addPOI(const PCOSMNode* node, const Position& pos, const PCTypeMap::TypeDef& def, const std::string& fullType,
                    int index, PCPolyContainer& toFill, bool ignorePruning, bool withAttributes) {
    if (def.discard) {
        return index;
    } else {
        const std::string idSuffix = (index == 0 ? "" : "#" + toString(index));
        const std::string id = def.prefix + toString(node->id) + idSuffix;
        PointOfInterest* poi = new PointOfInterest(
            StringUtils::escapeXML(id),
            StringUtils::escapeXML(OptionsCont::getOptions().getBool("osm.keep-full-type") ? fullType : def.id),
            def.color, pos, (double)def.layer);
        if (withAttributes) {
            poi->updateParameter(node->myAttributes);
        }
        if (!toFill.add(poi, ignorePruning)) {
            return index;
        } else {
            return index + 1;
        }
    }
}


// ---------------------------------------------------------------------------
// definitions of PCLoaderOSM::NodesHandler-methods
// ---------------------------------------------------------------------------
PCLoaderOSM::NodesHandler::NodesHandler(std::map<long long int, PCOSMNode*>& toFill,
                                        bool withAttributes, MsgHandler& errorHandler) :
    SUMOSAXHandler("osm - file"), myWithAttributes(withAttributes), myErrorHandler(errorHandler),
    myToFill(toFill), myLastNodeID(-1) {}


PCLoaderOSM::NodesHandler::~NodesHandler() {}


void
PCLoaderOSM::NodesHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    myParentElements.push_back(element);
    if (element == SUMO_TAG_NODE) {
        bool ok = true;
        long long int id = attrs.get<long long int>(SUMO_ATTR_ID, 0, ok);
        if (!ok) {
            return;
        }
        myLastNodeID = -1;
        if (myToFill.find(id) == myToFill.end()) {
            myLastNodeID = id;
            // assume we are loading multiple files...
            //  ... so we won't report duplicate nodes
            PCOSMNode* toAdd = new PCOSMNode();
            toAdd->id = id;
            bool ok = true;
            toAdd->lon = attrs.get<double>(SUMO_ATTR_LON, toString(id).c_str(), ok);
            toAdd->lat = attrs.get<double>(SUMO_ATTR_LAT, toString(id).c_str(), ok);
            if (!ok) {
                delete toAdd;
                return;
            }
            myToFill[toAdd->id] = toAdd;
        }
    }
    if (element == SUMO_TAG_TAG && myParentElements.size() > 2 && myParentElements[myParentElements.size() - 2] == SUMO_TAG_NODE
            && myLastNodeID != -1) {
        bool ok = true;
        std::string key = attrs.getOpt<std::string>(SUMO_ATTR_K, toString(myLastNodeID).c_str(), ok, "", false);
        std::string value = attrs.getOpt<std::string>(SUMO_ATTR_V, toString(myLastNodeID).c_str(), ok, "", false);
        if (key == "") {
            myErrorHandler.inform("Empty key in a a tag while parsing node '" + toString(myLastNodeID) + "' occured.");
            ok = false;
        }
        if (!ok) {
            return;
        }
        myToFill[myLastNodeID]->myAttributes[key] = value;
    }
}


void
PCLoaderOSM::NodesHandler::myEndElement(int element) {
    if (element == SUMO_TAG_NODE) {
        myLastNodeID = -1;
    }
    myParentElements.pop_back();
}


// ---------------------------------------------------------------------------
// definitions of PCLoaderOSM::RelationsHandler-methods
// ---------------------------------------------------------------------------
PCLoaderOSM::RelationsHandler::RelationsHandler(RelationsMap& additionalWays,
        Relations& relations,
        bool withAttributes,
        MsgHandler& errorHandler) :
    SUMOSAXHandler("osm - file"),
    myAdditionalWays(additionalWays),
    myRelations(relations),
    myWithAttributes(withAttributes),
    myErrorHandler(errorHandler),
    myCurrentRelation(0) {
}


PCLoaderOSM::RelationsHandler::~RelationsHandler() {
}


void
PCLoaderOSM::RelationsHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    myParentElements.push_back(element);
    // parse "relation" elements
    if (element == SUMO_TAG_RELATION) {
        bool ok = true;
        myCurrentWays.clear();
        std::string action = attrs.hasAttribute("action") ? attrs.getStringSecure("action", "") : "";
        if (action == "delete" || !ok) {
            myCurrentRelation = 0;
        }
        myCurrentRelation = new PCOSMRelation();
        myCurrentRelation->id = attrs.get<long long int>(SUMO_ATTR_ID, 0, ok);
        myRelations.push_back(myCurrentRelation);
        return;
    } else if (myCurrentRelation == 0) {
        return;
    }
    // parse member elements
    if (element == SUMO_TAG_MEMBER) {
        bool ok = true;
        std::string role = attrs.hasAttribute("role") ? attrs.getStringSecure("role", "") : "";
        long long int ref = attrs.get<long long int>(SUMO_ATTR_REF, 0, ok);
        if (role == "outer" || role == "inner") {
            std::string memberType = attrs.get<std::string>(SUMO_ATTR_TYPE, 0, ok);
            if (memberType == "way") {
                myCurrentWays.push_back(ref);
            }
        }
        return;
    }
    // parse values
    if (element == SUMO_TAG_TAG && myParentElements.size() > 2 && myParentElements[myParentElements.size() - 2] == SUMO_TAG_RELATION
            && myCurrentRelation != 0) {
        bool ok = true;
        std::string key = attrs.getOpt<std::string>(SUMO_ATTR_K, toString(myCurrentRelation).c_str(), ok, "", false);
        std::string value = attrs.getOpt<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, "", false);
        if (key == "") {
            myErrorHandler.inform("Empty key in a a tag while parsing way '" + toString(myCurrentRelation) + "' occured.");
            ok = false;
        }
        if (!ok) {
            return;
        }
        if (key == "name") {
            myCurrentRelation->name = value;
        } else if (MyKeysToInclude.count(key) > 0) {
            for (std::vector<long long int>::iterator it = myCurrentWays.begin(); it != myCurrentWays.end(); ++it) {
                myAdditionalWays[*it] = myCurrentRelation;
            }
        }
        myCurrentRelation->myAttributes[key] = value;
    }
}


void
PCLoaderOSM::RelationsHandler::myEndElement(int element) {
    myParentElements.pop_back();
    if (element == SUMO_TAG_RELATION) {
        myCurrentRelation = 0;
        myCurrentWays.clear();
    }
}


// ---------------------------------------------------------------------------
// definitions of PCLoaderOSM::EdgesHandler-methods
// ---------------------------------------------------------------------------
PCLoaderOSM::EdgesHandler::EdgesHandler(const std::map<long long int, PCOSMNode*>& osmNodes,
                                        EdgeMap& toFill,
                                        const RelationsMap& additionalWays,
                                        bool withAttributes, MsgHandler& errorHandler) :
    SUMOSAXHandler("osm - file"),
    myWithAttributes(withAttributes),
    myErrorHandler(errorHandler),
    myOSMNodes(osmNodes),
    myEdgeMap(toFill),
    myAdditionalWays(additionalWays) {
}


PCLoaderOSM::EdgesHandler::~EdgesHandler() {
}


void
PCLoaderOSM::EdgesHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    myParentElements.push_back(element);
    // parse "way" elements
    if (element == SUMO_TAG_WAY) {
        bool ok = true;
        long long int id = attrs.get<long long int>(SUMO_ATTR_ID, 0, ok);
        if (!ok) {
            return;
        }
        myCurrentEdge = new PCOSMEdge();
        myCurrentEdge->id = id;
        myCurrentEdge->myIsClosed = false;
        myKeep = (myAdditionalWays.find(id) != myAdditionalWays.end());
    }
    // parse "nd" (node) elements
    if (element == SUMO_TAG_ND) {
        bool ok = true;
        long long int ref = attrs.get<long long int>(SUMO_ATTR_REF, 0, ok);
        if (ok) {
            if (myOSMNodes.find(ref) == myOSMNodes.end()) {
                WRITE_WARNING("The referenced geometry information (ref='" + toString(ref) + "') is not known");
                return;
            }
            myCurrentEdge->myCurrentNodes.push_back(ref);
        }
    }
    // parse values
    if (element == SUMO_TAG_TAG && myParentElements.size() > 2 && myParentElements[myParentElements.size() - 2] == SUMO_TAG_WAY
            && myCurrentEdge != 0) {
        bool ok = true;
        std::string key = attrs.getOpt<std::string>(SUMO_ATTR_K, toString(myCurrentEdge->id).c_str(), ok, "", false);
        std::string value = attrs.getOpt<std::string>(SUMO_ATTR_V, toString(myCurrentEdge->id).c_str(), ok, "", false);
        if (key == "") {
            myErrorHandler.inform("Empty key in a a tag while parsing way '" + toString(myCurrentEdge->id) + "' occured.");
            ok = false;
        }
        if (!ok) {
            return;
        }
        if (key == "name") {
            myCurrentEdge->name = value;
        } else if (MyKeysToInclude.count(key) > 0) {
            myKeep = true;
        }
        myCurrentEdge->myAttributes[key] = value;
    }
}


void
PCLoaderOSM::EdgesHandler::myEndElement(int element) {
    myParentElements.pop_back();
    if (element == SUMO_TAG_WAY) {
        if (myKeep) {
            RelationsMap::const_iterator it = myAdditionalWays.find(myCurrentEdge->id);
            if (it != myAdditionalWays.end()) {
                myCurrentEdge->myAttributes.insert((*it).second->myAttributes.begin(), (*it).second->myAttributes.end());
            }
            myEdgeMap[myCurrentEdge->id] = myCurrentEdge;
        } else {
            delete myCurrentEdge;
        }
        myCurrentEdge = 0;
    }
}


/****************************************************************************/

