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
/// @file    GNENet.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
/// @version $Id$
///
// A visual container for GNE-network-components such as GNEEdge and GNEJunction.
// GNE components wrap netbuild-components and supply visualisation and editing
// capabilities (adapted from GUINet)
//
// Workflow (rough draft)
//   use NILoader to fill
//   do netedit stuff
//   call compute to save results
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <map>
#include <set>
#include <vector>

#include <netbuild/NBAlgorithms.h>
#include <netwrite/NWFrame.h>
#include <netwrite/NWWriter_XML.h>
#include <utility>
#include <utils/common/MsgHandler.h>
#include <utils/common/RGBColor.h>
#include <utils/common/StringUtils.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/gui/globjects/GUIGlObject_AbstractAdd.h>
#include <utils/gui/images/GUITextureSubSys.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/shapes/ShapeContainer.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/xml/XMLSubSys.h>

#include "GNEAdditional.h"
#include "GNEAdditionalFrame.h"
#include "GNEApplicationWindow.h"
#include "GNEChange_Additional.h"
#include "GNEChange_Attribute.h"
#include "GNEChange_Connection.h"
#include "GNEChange_Crossing.h"
#include "GNEChange_Edge.h"
#include "GNEChange_Junction.h"
#include "GNEChange_Lane.h"
#include "GNEChange_Selection.h"
#include "GNEChange_Shape.h"
#include "GNEConnection.h"
#include "GNECrossing.h"
#include "GNEDetector.h"
#include "GNEDetectorE2.h"
#include "GNEEdge.h"
#include "GNEJunction.h"
#include "GNEPoly.h"
#include "GNEPOI.h"
#include "GNEPOILane.h"
#include "GNELane.h"
#include "GNENet.h"
#include "GNEStoppingPlace.h"
#include "GNEUndoList.h"
#include "GNEViewNet.h"
#include "GNEViewParent.h"
#include "GNERerouter.h"
#include "GNEAdditionalHandler.h"
#include "GNEDialog_FixAdditionalPositions.h"


FXIMPLEMENT_ABSTRACT(GNENet::GNEChange_ReplaceEdgeInTLS, GNEChange, NULL, 0)

// ===========================================================================
// static members
// ===========================================================================
const RGBColor GNENet::selectionColor(0, 0, 204, 255);
const RGBColor GNENet::selectedLaneColor(0, 0, 128, 255);
const RGBColor GNENet::selectedConnectionColor(0, 0, 100, 255);
const RGBColor GNENet::selectedAdditionalColor(0, 0, 150, 255);
const double GNENet::Z_INITIALIZED = 1;

// ===========================================================================
// member method definitions
// ===========================================================================
GNENet::GNENet(NBNetBuilder* netBuilder) :
    GUIGlObject(GLO_NETWORK, ""),
    ShapeContainer(),
    myViewNet(0),
    myNetBuilder(netBuilder),
    myJunctions(),
    myEdges(),
    myEdgeIDSupplier("gneE", netBuilder->getEdgeCont().getAllNames()),
    myJunctionIDSupplier("gneJ", netBuilder->getNodeCont().getAllNames()),
    myNeedRecompute(true),
    myAdditionalsSaved(true),
    myShapesSaved(true) {
    // set net in gIDStorage
    GUIGlObjectStorage::gIDStorage.setNetObject(this);

    // init junction and edges
    initJunctionsAndEdges();

    // check Z boundary
    if (myZBoundary.ymin() != Z_INITIALIZED) {
        myZBoundary.add(0, 0);
    }
}


GNENet::~GNENet() {
    // Drop Edges
    for (auto it : myEdges) {
        it.second->decRef("GNENet::~GNENet");
        // show extra information for tests
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Deleting unreferenced " + toString(it.second->getTag()) + " '" + it.second->getID() + "' in GNENet destructor");
        }
        delete it.second;
    }
    // Drop junctions
    for (auto it : myJunctions) {
        it.second->decRef("GNENet::~GNENet");
        // show extra information for tests
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Deleting unreferenced " + toString(it.second->getTag()) + " '" + it.second->getID() + "' in GNENet destructor");
        }
        delete it.second;
    }


    // show extra information for tests
    if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
        WRITE_WARNING("Deleting net builder in GNENet destructor");
    }
    delete myNetBuilder;
}


const Boundary&
GNENet::getBoundary() const {
    // SUMORTree is also a Boundary
    return myGrid;
}


GUIGLObjectPopupMenu*
GNENet::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
    buildPopupHeader(ret, app);
    buildCenterPopupEntry(ret);
    buildPositionCopyEntry(ret, false);
    return ret;
}


GUIParameterTableWindow*
GNENet::getParameterWindow(GUIMainWindow& app, GUISUMOAbstractView&) {
    // Nets lanes don't have attributes
    GUIParameterTableWindow* ret = new GUIParameterTableWindow(app, *this, 2);
    // close building
    ret->closeBuilding();
    return ret;
}


void
GNENet::drawGL(const GUIVisualizationSettings& /*s*/) const {
}


bool
GNENet::addPolygon(const std::string& id, const std::string& type, const RGBColor& color, double layer, double angle,
    const std::string& imgFile, const PositionVector& shape, bool geo, bool fill, bool /*ignorePruning*/) {
    // check if ID is duplicated
    if (myPolygons.get(id) == NULL) {
        // create poly
        GNEPoly* poly = new GNEPoly(this, id, type, shape, geo, fill, color, layer, angle, imgFile, false, false);
        if (myPolygons.add(poly->getID(), poly)) {
            myViewNet->getUndoList()->p_begin("add " + toString(poly->getTag()));
            myViewNet->getUndoList()->add(new GNEChange_Shape(this, poly, true), true);
            myViewNet->getUndoList()->p_end();
            return true;
        } else {
            throw ProcessError("Error adding GNEPOly into shapeContainer");
        }
    } else {
        return false;
    }
}


bool
GNENet::removePolygon(const std::string& id) {
    GNEPoly* p = dynamic_cast<GNEPoly*>(myPolygons.get(id));
    if (p == 0) {
        return false;
    } else {
        return myPolygons.remove(id);
    }
}


bool
GNENet::addPOI(const std::string& id, const std::string& type, const RGBColor& color, const Position& pos, bool geo,
               const std::string &lane, double posOverLane, double posLat, double layer, double angle, 
               const std::string& imgFile, double width, double height, bool /*ignorePruning*/) {
    // check if ID is duplicated
    if (myPOIs.get(id) == NULL) {
        // create poly
        GNEPOI* poi = new GNEPOI(this, id, type, color, pos, geo, layer, angle, imgFile, width, height, false);
        if (myPOIs.add(poi->getID(), poi)) {
            myViewNet->getUndoList()->p_begin("add " + toString(poi->getTag()));
            myViewNet->getUndoList()->add(new GNEChange_Shape(this, poi, true), true);
            myViewNet->getUndoList()->p_end();
            return true;
        } else {
            throw ProcessError("Error adding GNEPOI into shapeContainer");
        }
    } else {
        return false;
    }
}


bool
GNENet::removePOI(const std::string& id) {
    GNEPOI* p = dynamic_cast<GNEPOI*>(myPOIs.get(id));
    if (p == 0) {
        return false;
    } else {
        return myPOIs.remove(id);
    }
}


Boundary
GNENet::getCenteringBoundary() const {
    return getBoundary();
}


const Boundary&
GNENet::getZBoundary() const {
    return myZBoundary;
}


SUMORTree&
GNENet::getVisualisationSpeedUp() {
    return myGrid;
}


const SUMORTree&
GNENet::getVisualisationSpeedUp() const {
    return myGrid;
}


GNEJunction*
GNENet::createJunction(const Position& pos, GNEUndoList* undoList) {
    std::string id = myJunctionIDSupplier.getNext();
    NBNode* nbn = new NBNode(id, pos);
    GNEJunction* junction = new GNEJunction(*nbn, this);
    undoList->add(new GNEChange_Junction(junction, true), true);
    assert(myJunctions[id]);
    return junction;
}


GNEEdge*
GNENet::createEdge(
    GNEJunction* src, GNEJunction* dest, GNEEdge* tpl, GNEUndoList* undoList,
    const std::string& suggestedName,
    bool wasSplit,
    bool allowDuplicateGeom) {
    // prevent duplicate edge (same geometry)
    const EdgeVector& outgoing = src->getNBNode()->getOutgoingEdges();
    for (EdgeVector::const_iterator it = outgoing.begin(); it != outgoing.end(); it++) {
        if ((*it)->getToNode() == dest->getNBNode() && (*it)->getGeometry().size() == 2) {
            if (!allowDuplicateGeom) {
                return 0;
            }
        }
    }

    std::string id;
    if (suggestedName != "" && !retrieveEdge(suggestedName, false)) {
        id = suggestedName;
        reserveEdgeID(id);
    } else {
        id = myEdgeIDSupplier.getNext();
    }

    GNEEdge* edge;
    if (tpl) {
        NBEdge* nbeTpl = tpl->getNBEdge();
        NBEdge* nbe = new NBEdge(id, src->getNBNode(), dest->getNBNode(), nbeTpl);
        edge = new GNEEdge(*nbe, this, wasSplit);
    } else {
        // default if no template is given
        double defaultSpeed = 50 / 3.6;
        std::string defaultType = "";
        int defaultNrLanes = 1;
        int defaultPriority = 1;
        double defaultWidth = NBEdge::UNSPECIFIED_WIDTH;
        double defaultOffset = NBEdge::UNSPECIFIED_OFFSET;
        NBEdge* nbe = new NBEdge(id, src->getNBNode(), dest->getNBNode(),
                                 defaultType, defaultSpeed,
                                 defaultNrLanes, defaultPriority,
                                 defaultWidth,
                                 defaultOffset);
        edge = new GNEEdge(*nbe, this, wasSplit);
    }
    undoList->p_begin("create " + toString(SUMO_TAG_EDGE));
    undoList->add(new GNEChange_Edge(edge, true), true);
    src->setLogicValid(false, undoList);
    dest->setLogicValid(false, undoList);
    requireRecompute();
    undoList->p_end();
    assert(myEdges[id]);
    return edge;
}


void
GNENet::deleteJunction(GNEJunction* junction, GNEUndoList* undoList) {
    // we have to delete all incident edges because they cannot exist without that junction
    // all deletions must be undone/redone together so we start a new command group
    // @todo if any of those edges are dead-ends should we remove their orphan junctions as well?
    undoList->p_begin("delete " + toString(SUMO_TAG_JUNCTION));

    // delete all crossings vinculated with junction
    while (junction->getGNECrossings().size() > 0) {
        deleteCrossing(junction->getGNECrossings().front(), undoList);
    }

    // find all crossings of neightbour junctions that shares an edge of this junction
    std::vector<GNECrossing*> crossingsToRemove;
    std::vector<GNEJunction*> junctionNeighbours = junction->getJunctionNeighbours();
    for (auto i : junctionNeighbours) {
        // iterate over crossing of neighbour juntion
        for (auto j : i->getGNECrossings()) {
            // if at least one of the edges of junction to remove belongs to a crossing of the neighbour junction, delete it
            if (j->checkEdgeBelong(junction->getGNEEdges())) {
                crossingsToRemove.push_back(j);
            }
        }
    }

    // delete crossings top remove
    for (auto i : crossingsToRemove) {
        deleteCrossing(i, undoList);
    }

    // deleting edges changes in the underlying EdgeVector so we have to make a copy
    const EdgeVector incident = junction->getNBNode()->getEdges();
    for (EdgeVector::const_iterator it = incident.begin(); it != incident.end(); it++) {
        deleteEdge(myEdges[(*it)->getID()], undoList);
    }

    // remove any traffic lights from the traffic light container (avoids lots of warnings)
    junction->setAttribute(SUMO_ATTR_TYPE, toString(NODETYPE_PRIORITY), undoList);

    // save selection status
    if (gSelected.isSelected(GLO_JUNCTION, junction->getGlID())) {
        std::set<GUIGlID> deselected;
        deselected.insert(junction->getGlID());
        undoList->add(new GNEChange_Selection(this, std::set<GUIGlID>(), deselected, true), true);
    }

    // delete edge
    undoList->add(new GNEChange_Junction(junction, false), true);
    undoList->p_end();
}


void
GNENet::deleteEdge(GNEEdge* edge, GNEUndoList* undoList) {
    undoList->p_begin("delete " + toString(SUMO_TAG_EDGE));

    // obtain a copy of GNERerouters of edge
    std::vector<GNERerouter*> rerouters = edge->getGNERerouters();

    // delete additionals childs of edge
    std::vector<GNEAdditional*> copyOfEdgeAdditionals = edge->getAdditionalChilds();
    for (auto i : copyOfEdgeAdditionals) {
        undoList->add(new GNEChange_Additional(i, false), true);
    }

    // delete additionals childs of lane
    for (auto i : edge->getLanes()) {
        std::vector<GNEAdditional*> copyOfLaneAdditionals = i->getAdditionalChilds();
        for (auto j : copyOfLaneAdditionals) {
            undoList->add(new GNEChange_Additional(j, false), true);
        }
    }

    // remove edge from crossings related with this edge
    edge->getGNEJunctionSource()->removeEdgeFromCrossings(edge, undoList);
    edge->getGNEJunctionDestiny()->removeEdgeFromCrossings(edge, undoList);

    // invalidate junctions
    edge->getGNEJunctionSource()->setLogicValid(false, undoList);
    edge->getGNEJunctionDestiny()->setLogicValid(false, undoList);

    // save selection status
    if (gSelected.isSelected(GLO_EDGE, edge->getGlID())) {
        std::set<GUIGlID> deselected;
        deselected.insert(edge->getGlID());
        undoList->add(new GNEChange_Selection(this, std::set<GUIGlID>(), deselected, true), true);
    }

    // Delete edge
    undoList->add(new GNEChange_Edge(edge, false), true);

    // check if after removing there are Rerouters without edge Childs
    for (auto i : rerouters) {
        if (i->getEdgeChilds().size() == 0) {
            undoList->add(new GNEChange_Additional(i, false), true);
        }
    }
    // remove edge requieres always a recompute (due geometry and connections)
    requireRecompute();
    undoList->p_end();
}


void
GNENet::replaceIncomingEdge(GNEEdge* which, GNEEdge* by, GNEUndoList* undoList) {
    undoList->p_begin("replace " + toString(SUMO_TAG_EDGE));
    undoList->p_add(new GNEChange_Attribute(by, SUMO_ATTR_TO, which->getAttribute(SUMO_ATTR_TO)));

    // replace in additionals childs of edge
    std::vector<GNEAdditional*> copyOfEdgeAdditionals = which->getAdditionalChilds();
    for (auto i : copyOfEdgeAdditionals) {
        undoList->p_add(new GNEChange_Attribute(i, SUMO_ATTR_EDGE, by->getID()));
    }

    // replace in additionals childs of lane
    for (auto i : which->getLanes()) {
        std::vector<GNEAdditional*> copyOfLaneAdditionals = i->getAdditionalChilds();
        for (auto j : copyOfLaneAdditionals) {
            undoList->p_add(new GNEChange_Attribute(i, SUMO_ATTR_LANE, by->getNBEdge()->getLaneID(i->getIndex())));
        }
    }

    // replace in rerouters
    for (auto rerouter : which->getGNERerouters()) {
        replaceInListAttribute(rerouter, SUMO_ATTR_EDGES, which->getID(), by->getID(), undoList);
    }

    // replace in crossings
    for (auto crossing : which->getGNEJunctionDestiny()->getGNECrossings()) {
        // if at least one of the edges of junction to remove belongs to a crossing of the source junction, delete it
        replaceInListAttribute(crossing, SUMO_ATTR_EDGES, which->getID(), by->getID(), undoList);
    }

    // fix connections (make a copy because they will be modified
    std::vector<NBEdge::Connection> connections = which->getNBEdge()->getConnections();
    for (auto con : connections) {
        undoList->add(new GNEChange_Connection(which, con, false, false), true);
        undoList->add(new GNEChange_Connection(by, con, false, true), true);
    }

    // save selection status
    if (gSelected.isSelected(GLO_EDGE, which->getGlID())) {
        std::set<GUIGlID> deselected;
        deselected.insert(which->getGlID());
        undoList->add(new GNEChange_Selection(this, std::set<GUIGlID>(), deselected, true), true);
    }
    undoList->add(new GNEChange_ReplaceEdgeInTLS(getTLLogicCont(), which->getNBEdge(), by->getNBEdge()), true);

    // Delete edge
    undoList->add(new GNEChange_Edge(which, false), true);

    // finish replace edge
    undoList->p_end();
}


void
GNENet::deleteLane(GNELane* lane, GNEUndoList* undoList) {
    GNEEdge* edge = &lane->getParentEdge();
    if (edge->getNBEdge()->getNumLanes() == 1) {
        // remove the whole edge instead
        deleteEdge(edge, undoList);
    } else {
        undoList->p_begin("delete " + toString(SUMO_TAG_LANE));

        // delete additionals childs of lane
        std::vector<GNEAdditional*> copyOfAdditionals = lane->getAdditionalChilds();
        for (auto i : copyOfAdditionals) {
            undoList->add(new GNEChange_Additional(i, false), true);
        }

        // invalidate junctions (saving connections)
        edge->getGNEJunctionSource()->setLogicValid(false, undoList);
        edge->getGNEJunctionDestiny()->setLogicValid(false, undoList);

        // save selection status
        if (gSelected.isSelected(GLO_EDGE, edge->getGlID())) {
            std::set<GUIGlID> deselected;
            deselected.insert(edge->getGlID());
            undoList->add(new GNEChange_Selection(this, std::set<GUIGlID>(), deselected, true), true);
        }

        // delete lane
        const NBEdge::Lane& laneAttrs = edge->getNBEdge()->getLaneStruct(lane->getIndex());
        undoList->add(new GNEChange_Lane(edge, lane, laneAttrs, false), true);
        if (gSelected.isSelected(GLO_LANE, lane->getGlID())) {
            std::set<GUIGlID> deselected;
            deselected.insert(lane->getGlID());
            undoList->add(new GNEChange_Selection(this, std::set<GUIGlID>(), deselected, true), true);
        }

        // remove lane requieres always a recompute (due geometry and connections)
        requireRecompute();
        undoList->p_end();
    }
}


void
GNENet::deleteConnection(GNEConnection* connection, GNEUndoList* undoList) {
    undoList->p_begin("delete " + toString(SUMO_TAG_CONNECTION));
    // obtain NBConnection to remove
    NBConnection deleted = connection->getNBConnection();
    GNEJunction* junctionDestiny = connection->getEdgeFrom()->getGNEJunctionDestiny();
    junctionDestiny->markAsModified(undoList);
    // check if GNEConnection was previouslyselected, and if true, unselect it.
    bool selected = gSelected.isSelected(GLO_CONNECTION, connection->getGlID());
    if (selected) {
        gSelected.deselect(connection->getGlID());
    }
    undoList->add(new GNEChange_Connection(connection->getEdgeFrom(), connection->getNBEdgeConnection(), selected, false), true);
    junctionDestiny->invalidateTLS(myViewNet->getUndoList(), deleted);
    // remove connection requieres always a recompute (due geometry and connections)
    requireRecompute();
    undoList->p_end();
}


void
GNENet::deleteCrossing(GNECrossing* crossing, GNEUndoList* undoList) {
    undoList->p_begin("delete crossing");
    // check if GNECrossing was previouslyselected, and if true, unselect it.
    bool selected = gSelected.isSelected(GLO_CROSSING, crossing->getGlID());
    if (selected) {
        gSelected.deselect(crossing->getGlID());
    }
    undoList->add(new GNEChange_Crossing(crossing->getParentJunction(), crossing->getNBCrossing()->edges,
                                         crossing->getNBCrossing()->width, crossing->getNBCrossing()->priority, 
                                         crossing->getNBCrossing()->customTLIndex,
                                         crossing->getNBCrossing()->customShape, selected, false), true);
    // remove crossing requieres always a recompute (due geometry and connections)
    requireRecompute();
    undoList->p_end();
}


void
GNENet::deleteShape(GNEShape* shape, GNEUndoList* undoList) {
    myViewNet->getUndoList()->p_begin("delete " + toString(shape->getTag()));
    // save selection status
    if (gSelected.isSelected(GLO_POLYGON, dynamic_cast<GUIGlObject*>(shape)->getGlID())) {
        std::set<GUIGlID> deselected;
        deselected.insert(dynamic_cast<GUIGlObject*>(shape)->getGlID());
        undoList->add(new GNEChange_Selection(this, std::set<GUIGlID>(), deselected, true), true);
    }
    // delete shape
    myViewNet->getUndoList()->add(new GNEChange_Shape(myViewNet->getNet(), shape, false), true);
    myViewNet->getUndoList()->p_end();
}


void
GNENet::duplicateLane(GNELane* lane, GNEUndoList* undoList) {
    undoList->p_begin("duplicate " + toString(SUMO_TAG_LANE));
    GNEEdge* edge = &lane->getParentEdge();
    const NBEdge::Lane& laneAttrs = edge->getNBEdge()->getLaneStruct(lane->getIndex());
    GNELane* newLane = new GNELane(*edge, lane->getIndex());
    undoList->add(new GNEChange_Lane(edge, newLane, laneAttrs, true), true);
    requireRecompute();
    undoList->p_end();
}


bool
GNENet::restrictLane(SUMOVehicleClass vclass, GNELane* lane, GNEUndoList* undoList) {
    bool addRestriction = true;
    if (vclass == SVC_PEDESTRIAN) {
        GNEEdge& edge = lane->getParentEdge();
        for (auto i : edge.getLanes()) {
            if (i->isRestricted(SVC_PEDESTRIAN)) {
                // prevent adding a 2nd sidewalk
                addRestriction = false;
            } else {
                // ensure that the sidewalk is used exclusively
                const SVCPermissions allOldWithoutPeds = edge.getNBEdge()->getPermissions(i->getIndex()) & ~SVC_PEDESTRIAN;
                i->setAttribute(SUMO_ATTR_ALLOW, getVehicleClassNames(allOldWithoutPeds), undoList);
            }
        }
    }
    // restrict the lane
    if (addRestriction) {
        lane->setAttribute(SUMO_ATTR_ALLOW, toString(vclass), undoList);
        lane->setAttribute(SUMO_ATTR_WIDTH, toString(OptionsCont::getOptions().getFloat("default.sidewalk-width")), undoList);
        return true;
    } else {
        return false;
    }
}


bool
GNENet::addSRestrictedLane(SUMOVehicleClass vclass, GNEEdge& edge, GNEUndoList* undoList) {
    // First check that edge don't have a sidewalk
    for (auto i : edge.getLanes()) {
        if (i->isRestricted(vclass)) {
            return false;
        }
    }
    // duplicate last lane
    duplicateLane(edge.getLanes().at(0), undoList);
    // transform the created (last) lane to a sidewalk
    return restrictLane(vclass, edge.getLanes()[0], undoList);
}


bool
GNENet::removeRestrictedLane(SUMOVehicleClass vclass, GNEEdge& edge, GNEUndoList* undoList) {
    // iterate over lanes of edge
    for (auto i : edge.getLanes()) {
        if (i->isRestricted(vclass)) {
            // Delete lane
            deleteLane(i, undoList);
            return true;
        }
    }
    return false;
}


GNEJunction*
GNENet::splitEdge(GNEEdge* edge, const Position& pos, GNEUndoList* undoList, GNEJunction* newJunction) {
    undoList->p_begin("split " + toString(SUMO_TAG_EDGE));
    deleteEdge(edge, undoList); // still exists. we delete it so we can reuse the name in case of resplit
    // compute geometry
    const PositionVector& oldGeom = edge->getNBEdge()->getGeometry();
    const double linePos = oldGeom.nearest_offset_to_point2D(pos, false);
    std::pair<PositionVector, PositionVector> newGeoms = oldGeom.splitAt(linePos);
    // figure out the new name
    int posBase = 0;
    std::string baseName = edge->getMicrosimID();
    if (edge->wasSplit()) {
        const std::string::size_type sep_index = baseName.rfind('.');
        if (sep_index != std::string::npos) { // edge may have been renamed in between
            std::string posString = baseName.substr(sep_index + 1);
            try {
                posBase = GNEAttributeCarrier::parse<int>(posString.c_str());
                baseName = baseName.substr(0, sep_index); // includes the .
            } catch (NumberFormatException) {
            }
        }
    }
    baseName += '.';
    // create edges
    if (newJunction == 0) {
        newJunction = createJunction(pos, undoList);
    }
    GNEEdge* firstPart = createEdge(edge->getGNEJunctionSource(), newJunction, edge,
                                    undoList, baseName + toString(posBase), true);
    GNEEdge* secondPart = createEdge(newJunction, edge->getGNEJunctionDestiny(), edge,
                                     undoList, baseName + toString(posBase + (int)linePos), true);
    // fix first part of geometry
    firstPart->setAttribute(GNE_ATTR_SHAPE_START, toString(newGeoms.first[0]), undoList);
    firstPart->setAttribute(GNE_ATTR_SHAPE_END, toString(newGeoms.first[-1]), undoList);
    newGeoms.first.pop_back();
    newGeoms.first.erase(newGeoms.first.begin());
    firstPart->setAttribute(SUMO_ATTR_SHAPE, toString(newGeoms.first), undoList);
    // fix second part of geometry
    secondPart->setAttribute(GNE_ATTR_SHAPE_START, toString(newGeoms.second[0]), undoList);
    secondPart->setAttribute(GNE_ATTR_SHAPE_END, toString(newGeoms.second[-1]), undoList);
    newGeoms.second.pop_back();
    newGeoms.second.erase(newGeoms.second.begin());
    secondPart->setAttribute(SUMO_ATTR_SHAPE, toString(newGeoms.second), undoList);
    // fix connections
    for (auto con_it : edge->getNBEdge()->getConnections()) {
        undoList->add(new GNEChange_Connection(secondPart, con_it, false, true), true);
    }
    undoList->p_end();
    return newJunction;
}


void
GNENet::splitEdgesBidi(const std::set<GNEEdge*>& edges, const Position& pos, GNEUndoList* undoList) {
    GNEJunction* newJunction = 0;
    undoList->p_begin("split " + toString(SUMO_TAG_EDGE) + "s");
    for (auto it : edges) {
        newJunction = splitEdge(it, pos, undoList, newJunction);
    }
    undoList->p_end();
}


void
GNENet::reverseEdge(GNEEdge* edge, GNEUndoList* undoList) {
    undoList->p_begin("reverse " + toString(SUMO_TAG_EDGE));
    deleteEdge(edge, undoList); // still exists. we delete it so we can reuse the name in case of resplit
    GNEEdge* reversed = createEdge(edge->getGNEJunctionDestiny(), edge->getGNEJunctionSource(), edge, undoList, edge->getID(), false, true);
    assert(reversed != 0);
    reversed->setAttribute(SUMO_ATTR_SHAPE, toString(edge->getNBEdge()->getInnerGeometry().reverse()), undoList);
    undoList->p_end();
}


GNEEdge*
GNENet::addReversedEdge(GNEEdge* edge, GNEUndoList* undoList) {
    undoList->p_begin("add reversed " + toString(SUMO_TAG_EDGE));
    GNEEdge* reversed = NULL;
    if (edge->getNBEdge()->getLaneSpreadFunction() == LANESPREAD_RIGHT || isRailway(edge->getNBEdge()->getPermissions())) {
        // for rail edges, we assume bi-directional tracks are wanted
        reversed = createEdge(edge->getGNEJunctionDestiny(), edge->getGNEJunctionSource(), edge, undoList, "-" + edge->getID(), false, true);
        assert(reversed != 0);
        reversed->setAttribute(SUMO_ATTR_SHAPE, toString(edge->getNBEdge()->getInnerGeometry().reverse()), undoList);
    } else {
        // if the edge is centered it should probably connect somewhere else
        // make it easy to move and reconnect it
        PositionVector orig = edge->getNBEdge()->getGeometry();
        PositionVector origInner = edge->getNBEdge()->getInnerGeometry();
        const double tentativeShift = edge->getNBEdge()->getTotalWidth() + 2;
        orig.move2side(-tentativeShift);
        origInner.move2side(-tentativeShift);
        GNEJunction* src = createJunction(orig.back(), undoList);
        GNEJunction* dest = createJunction(orig.front(), undoList);
        reversed = createEdge(src, dest, edge, undoList, "-" + edge->getID(), false, true);
        assert(reversed != 0);
        reversed->setAttribute(SUMO_ATTR_SHAPE, toString(origInner.reverse()), undoList);
        // select the new edge and its nodes
        std::set<GUIGlID> toSelect;
        toSelect.insert(reversed->getGlID());
        toSelect.insert(src->getGlID());
        toSelect.insert(dest->getGlID());
        undoList->add(new GNEChange_Selection(this, toSelect, gSelected.getSelected(), true), true);
    }
    undoList->p_end();
    return reversed;
}


void
GNENet::mergeJunctions(GNEJunction* moved, GNEJunction* target, GNEUndoList* undoList) {
    undoList->p_begin("merge " + toString(SUMO_TAG_JUNCTION) + "s");
    // place moved junction in the same position of target junction
    moved->setAttribute(SUMO_ATTR_POSITION, target->getAttribute(SUMO_ATTR_POSITION), undoList);
    // deleting edges changes in the underlying EdgeVector so we have to make a copy
    const EdgeVector incoming = moved->getNBNode()->getIncomingEdges();
    for (EdgeVector::const_iterator it = incoming.begin(); it != incoming.end(); it++) {
        GNEEdge* oldEdge = myEdges[(*it)->getID()];
        remapEdge(oldEdge, oldEdge->getGNEJunctionSource(), target, undoList);
    }
    // deleting edges changes in the underlying EdgeVector so we have to make a copy
    const EdgeVector outgoing = moved->getNBNode()->getOutgoingEdges();
    for (EdgeVector::const_iterator it = outgoing.begin(); it != outgoing.end(); it++) {
        GNEEdge* oldEdge = myEdges[(*it)->getID()];
        remapEdge(oldEdge, target, oldEdge->getGNEJunctionDestiny(), undoList);
    }
    // deleted moved junction
    deleteJunction(moved, undoList);
    undoList->p_end();
}


void
GNENet::remapEdge(GNEEdge* oldEdge, GNEJunction* from, GNEJunction* to, GNEUndoList* undoList, bool keepEndpoints) {
    // remove all crossings asociated to this edge before remap
    std::vector<GNECrossing*> crossingsOfOldEdge = oldEdge->getGNECrossings();
    for (auto i : crossingsOfOldEdge) {
        deleteCrossing(i, undoList);
    }
    // delete first so we can reuse the name, reference stays valid
    deleteEdge(oldEdge, undoList);
    if (from != to) {
        GNEEdge* newEdge = createEdge(from, to, oldEdge, undoList, oldEdge->getMicrosimID(), false, true);
        newEdge->setAttribute(SUMO_ATTR_SHAPE, oldEdge->getAttribute(SUMO_ATTR_SHAPE), undoList);
        if (keepEndpoints) {
            // preserve endpoints even if they where not customized
            newEdge->setAttribute(GNE_ATTR_SHAPE_START, toString(oldEdge->getNBEdge()->getGeometry().front()), undoList);
            newEdge->setAttribute(GNE_ATTR_SHAPE_END, toString(oldEdge->getNBEdge()->getGeometry().back()), undoList);
        }
    }
    // @todo remap connectivity as well
}


bool
GNENet::checkJunctionPosition(const Position& pos) {
    // Check that there isn't another junction in the same position as Pos
    for (auto i : myJunctions) {
        if (i.second->getPositionInView() == pos) {
            return false;
        }
    }
    return true;
}


void
GNENet::save(OptionsCont& oc) {
    // compute without volatile options and update network
    computeAndUpdate(oc, false);
    // write network
    NWFrame::writeNetwork(oc, *myNetBuilder);
}


void
GNENet::saveAdditionals(const std::string& filename) {
    // obtain invalid stopping places and detectors
    std::vector<GNEStoppingPlace*> invalidStoppingPlaces;
    std::vector<GNEDetector*> invalidDetectors;
    for (auto i : myAdditionals) {
        GNEStoppingPlace* stoppingPlace = dynamic_cast<GNEStoppingPlace*>(i.second);
        GNEDetector* detector = dynamic_cast<GNEDetector*>(i.second);
        // check if has to be fixed
        if ((stoppingPlace != NULL) && (stoppingPlace->areStoppingPlacesPositionsFixed() == false)) {
            invalidStoppingPlaces.push_back(stoppingPlace);
        } else if ((detector != NULL) && (detector->isDetectorPositionFixed() == false)) {
            invalidDetectors.push_back(detector);
        }
    }
    // if there are invalid StoppingPlaces or detectors, open GNEDialog_FixAdditionalPositions
    if (invalidStoppingPlaces.size() > 0 || invalidDetectors.size() > 0) {
        // 0 -> Canceled Saving, with or whithout selecting invalid stopping places and E2
        // 1 -> Invalid stoppingPlaces and E2 fixed, friendlyPos enabled, or saved with invalid positions
        GNEDialog_FixAdditionalPositions fixAdditionalPositionsDialog(myViewNet, invalidStoppingPlaces, invalidDetectors);
        if (fixAdditionalPositionsDialog.execute() == 0) {
            // Here a console message
            ;
        } else {
            // save additionals
            OutputDevice& device = OutputDevice::getDevice(filename);
            device.openTag("additionals");
            for (auto i : myAdditionals) {
                i.second->writeAdditional(device);
            }
            device.close();
        }
        // set focus again in viewNet
        myViewNet->setFocus();
    } else {
        OutputDevice& device = OutputDevice::getDevice(filename);
        device.openTag("additionals");
        for (auto i : myAdditionals) {
            i.second->writeAdditional(device);
        }
        device.close();
    }
    // change value of flag
    myAdditionalsSaved = true;
    // show debug information
    if(OptionsCont::getOptions().getBool("gui-testing-debug")) {
        WRITE_WARNING("Additionals saved");
    }
}


void
GNENet::savePlain(OptionsCont& oc) {
    // compute without volatile options
    computeAndUpdate(oc, false);
    NWWriter_XML::writeNetwork(oc, *myNetBuilder);
}


void
GNENet::saveJoined(OptionsCont& oc) {
    // compute without volatile options
    computeAndUpdate(oc, false);
    NWWriter_XML::writeJoinedJunctions(oc, myNetBuilder->getNodeCont());
}


void
GNENet::setViewNet(GNEViewNet* viewNet) {
    myViewNet = viewNet;
}


GNEJunction*
GNENet::retrieveJunction(const std::string& id, bool failHard) {
    if (myJunctions.count(id)) {
        return myJunctions[id];
    } else if (failHard) {
        // If junction wasn't found, throw exception
        throw UnknownElement("Junction " + id);
    } else {
        return NULL;
    }
}


GNEEdge*
GNENet::retrieveEdge(const std::string& id, bool failHard) {
    auto i = myEdges.find(id);
    // If edge was fund
    if (i != myEdges.end()) {
        return i->second;
    } else if (failHard) {
        // If edge wasn't found, throw exception
        throw UnknownElement("Edge " + id);
    } else {
        return NULL;
    }
}


GNEEdge*
GNENet::retrieveEdge(GNEJunction* from, GNEJunction* to, bool failHard) {
    assert((from != NULL) && (to != NULL));
    // iterate over Junctions of net
    for (auto i : myEdges) {
        if ((i.second->getGNEJunctionSource() == from) && (i.second->getGNEJunctionDestiny() == to)) {
            return i.second;
        }
    }
    // if edge wasn' found, throw exception or return NULL
    if (failHard) {
        throw UnknownElement("Edge with from='" + from->getID() + "' and to='" + to->getID() + "'");
    } else {
        return NULL;
    }
}


GNEPoly*
GNENet::retrievePolygon(const std::string& id, bool failHard) const {
    if (myPolygons.get(id) != 0) {
        return reinterpret_cast<GNEPoly*>(myPolygons.get(id));
    } else if (failHard) {
        // If Polygon wasn't found, throw exception
        throw UnknownElement("Polygon " + id);
    } else {
        return NULL;
    }
}


GNEPOI*
GNENet::retrievePOI(const std::string& id, bool failHard) const {
    if (myPOIs.get(id) != 0) {
        return reinterpret_cast<GNEPOI*>(myPOIs.get(id));
    } else if (failHard) {
        // If POI wasn't found, throw exception
        throw UnknownElement("POI " + id);
    } else {
        return NULL;
    }
}


GNEPOILane*
GNENet::retrievePOILane(const std::string& id, bool failHard) const {
    if (myPOIs.get(id) != 0) {
        return reinterpret_cast<GNEPOILane*>(myPOIs.get(id));
    } else if (failHard) {
        // If POI wasn't found, throw exception
        throw UnknownElement("POILane " + id);
    } else {
        return NULL;
    }
}


std::vector<GNEEdge*>
GNENet::retrieveEdges(bool onlySelected) {
    std::vector<GNEEdge*> result;
    for (auto it : myEdges) {
        if (!onlySelected || gSelected.isSelected(GLO_EDGE, it.second->getGlID())) {
            result.push_back(it.second);
        }
    }
    return result;
}


std::vector<GNELane*>
GNENet::retrieveLanes(bool onlySelected) {
    std::vector<GNELane*> result;
    for (auto it : myEdges) {
        for (auto it_lane : it.second->getLanes()) {
            if (!onlySelected || gSelected.isSelected(GLO_LANE, it_lane->getGlID())) {
                result.push_back(it_lane);
            }
        }
    }
    return result;
}


GNELane*
GNENet::retrieveLane(const std::string& id, bool failHard, bool checkVolatileChange) {
    const std::string edge_id = SUMOXMLDefinitions::getEdgeIDFromLane(id);
    GNEEdge* edge = retrieveEdge(edge_id, failHard);
    if (edge != 0) {
        GNELane *lane = NULL;
        // search  lane in lane's edges
        for (auto it : edge->getLanes()) {
            if (it->getID() == id) {
                lane = it;
            }
        }
        // throw exception or return NULL if lane wasn't found
        if (lane == NULL) {
            if(failHard) {
                // Throw exception if failHard is enabled
                throw UnknownElement(toString(SUMO_TAG_LANE) + " " + id);
            }
        } else {
            // check if the recomputing with volatile option has changed the number of lanes (needed for additionals)
            if(checkVolatileChange && (myEdgesAndNumberOfLanes.count(edge_id) == 1) && myEdgesAndNumberOfLanes[edge_id] != edge->getLanes().size()) {
                return edge->getLanes().at(lane->getIndex() + 1);
            }
            return lane;
        }
    } else if (failHard) {
        // Throw exception if failHard is enabled
        throw UnknownElement(toString(SUMO_TAG_EDGE) + " " + edge_id);
    }
    return NULL;
}


std::vector<GNEJunction*>
GNENet::retrieveJunctions(bool onlySelected) {
    std::vector<GNEJunction*> result;
    for (auto it : myJunctions) {
        if (!onlySelected || gSelected.isSelected(GLO_JUNCTION, it.second->getGlID())) {
            result.push_back(it.second);
        }
    }
    return result;
}


std::vector<GNEShape*> 
GNENet::retrieveShapes(SumoXMLTag shapeTag, bool onlySelected) {
    std::vector<GNEShape*> result;
    // fill polygons
    if((shapeTag == SUMO_TAG_NOTHING) || (shapeTag == SUMO_TAG_POLY)) {
        for (auto it : getPolygons().getMyMap()) {
            GNEPoly* poly = dynamic_cast<GNEPoly*>(it.second);
            // only add visible polygons
            if (poly && poly->isShapeVisible() && (!onlySelected || gSelected.isSelected(GLO_POLYGON, poly->getGlID()))) {
                result.push_back(poly);
            }
        }
    }
    // fill POIs
    if((shapeTag == SUMO_TAG_NOTHING) || (shapeTag == SUMO_TAG_POI)) {
        for (auto it : getPOIs().getMyMap()) {
            GNEPOI* POI = dynamic_cast<GNEPOI*>(it.second);
            // only add visible POIs
            if (POI && POI->isShapeVisible() && (!onlySelected || gSelected.isSelected(GLO_POI, POI->getGlID()))) {
                result.push_back(POI);
            }
        }
    }
    // fill POILanes
    if((shapeTag == SUMO_TAG_NOTHING) || (shapeTag == SUMO_TAG_POILANE)) {
        for (auto it : getPOIs().getMyMap()) {
            GNEPOILane* POILane = dynamic_cast<GNEPOILane*>(it.second);
            // only add visible POILanes
            if (POILane && POILane->isShapeVisible() && (!onlySelected || gSelected.isSelected(GLO_POI, POILane->getGlID()))) {
                result.push_back(POILane);
            }
        }
    }
    return result;
}


void
GNENet::refreshElement(GUIGlObject* o) {
    myGrid.removeAdditionalGLObject(o);
    myGrid.addAdditionalGLObject(o);
    update();
}


void
GNENet::refreshAdditional(GNEAdditional* additional) {
    // we need a special case for Calibrators
    SumoXMLTag tag = (additional->getTag() == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : additional->getTag();
    GNEAdditionals::iterator positionToRemove = myAdditionals.find(std::pair<std::string, SumoXMLTag>(additional->getID(), tag));
    // Check if additional element exists before refresh
    if (positionToRemove != myAdditionals.end()) {
        myGrid.removeAdditionalGLObject(additional);
        myGrid.addAdditionalGLObject(additional);
        update();
    }
}


std::string
GNENet::generateVaporizerID() const {
    int counter = 0;
    while (myAdditionals.find(std::pair<std::string, SumoXMLTag>("vaporizer_" + toString(counter), SUMO_TAG_VAPORIZER)) != myAdditionals.end()) {
        counter++;
    }
    return "vaporizer_" + toString(counter);
}


std::vector<GNEAttributeCarrier*>
GNENet::retrieveAttributeCarriers(const std::set<GUIGlID>& ids, GUIGlObjectType type) {
    std::vector<GNEAttributeCarrier*> result;
    for (auto it : ids) {
        GUIGlObject* object = GUIGlObjectStorage::gIDStorage.getObjectBlocking(it);
        if (object != 0) {
            std::string id = object->getMicrosimID();
            GUIGlObjectStorage::gIDStorage.unblockObject(it);
            GNEAttributeCarrier* ac = 0;
            switch (type) {
                case GLO_JUNCTION:
                    ac = dynamic_cast<GNEJunction*>(object);
                    break;
                case GLO_EDGE:
                    ac = dynamic_cast<GNEEdge*>(object);
                    break;
                case GLO_LANE:
                    ac = dynamic_cast<GNELane*>(object);
                    break;
                case GLO_ADDITIONAL:
                    ac = dynamic_cast<GNEAdditional*>(object);
                    break;
                case GLO_CONNECTION:
                    ac = dynamic_cast<GNEConnection*>(object);
                    break;
                case GLO_CROSSING:
                    ac = dynamic_cast<GNECrossing*>(object);
                    break;
                case GLO_POLYGON:
                    ac = dynamic_cast<GNEPoly*>(object);
                    break;
                case GLO_POI:
                    ac = dynamic_cast<GNEPOI*>(object);
                    break;
                default:
                    break;
            }
            if (ac == 0) {
                throw ProcessError("GUIGlObject does not match the declared type");
            } else {
                result.push_back(ac);
            }
        } else {
            throw ProcessError("Attempted to retrieve non-existant GUIGlObject");
        }
    }
    return result;
}


std::set<GUIGlID>
GNENet::getGlIDs(GUIGlObjectType type) {
    std::set<GUIGlID> result;
    switch (type) {
        case GLO_MAX: {
            std::set<GUIGlObjectType> knownTypes;
            knownTypes.insert(GLO_JUNCTION);
            knownTypes.insert(GLO_EDGE);
            knownTypes.insert(GLO_LANE);
            // knownTypes.insert(GLO_TLLOGIC); makes no sense to include them
            knownTypes.insert(GLO_ADDITIONAL);
            knownTypes.insert(GLO_CONNECTION);
            knownTypes.insert(GLO_CROSSING);
            knownTypes.insert(GLO_POLYGON);
            knownTypes.insert(GLO_POI);
            // obtain all GLIDS calling getGlIDs(...) recursively
            for (auto it : knownTypes) {
                const std::set<GUIGlID> tmp = getGlIDs(it);
                result.insert(tmp.begin(), tmp.end());
            }
            break;
        }
        case GLO_JUNCTION:
            for (auto it : myJunctions) {
                result.insert(it.second->getGlID());
            }
            break;
        case GLO_EDGE:
            for (auto it : myEdges) {
                result.insert(it.second->getGlID());
            }
            break;
        case GLO_LANE: {
            for (auto i : myEdges) {
                // iterate over every edge's lane
                for (auto j : i.second->getLanes()) {
                    result.insert(j->getGlID());
                }
            }
            break;
        }
        case GLO_TLLOGIC: {
            // return all junctions which have a traffic light (we do not have a GUIGlObject for each traffic light)
            for (auto it : myJunctions) {
                if (it.second->getNBNode()->isTLControlled()) {
                    result.insert(it.second->getGlID());
                }
            }
            break;
        }
        case GLO_ADDITIONAL: {
            // Iterate over all additionals of net
            for (auto it : myAdditionals) {
                // Insert every additional in result
                result.insert(it.second->getGlID());
            }
            break;
        }
        case GLO_CONNECTION: {
            for (auto i : myEdges) {
                // Iterate over edge's connections
                for (auto j : i.second->getGNEConnections()) {
                    // Insert every connection of edge in result
                    result.insert(j->getGlID());
                }
            }
            break;
        }
        case GLO_CROSSING: {
            for (auto i : myJunctions) {
                // Iterate over junction's crossings
                for (auto j : i.second->getGNECrossings()) {
                    // Insert every crossing of junction in result
                    result.insert(j->getGlID());
                }
            }
            break;
        }
        case GLO_POLYGON: {
            for (auto i : myPolygons.getMyMap()) {
                result.insert(dynamic_cast<GNEPoly*>(i.second)->getGlID());
            }
            break;
        }
        case GLO_POI: {
            for (auto i : myPOIs.getMyMap()) {
                result.insert(dynamic_cast<GNEPOI*>(i.second)->getGlID());
            }
            break;
        }
        default: // add other types once we know them
            break;
    }
    return result;
}


void
GNENet::computeEverything(GNEApplicationWindow* window, bool force, bool volatileOptions, std::string additionalPath, std::string shapePath) {
    if (!myNeedRecompute) {
        if (force) {
            if (volatileOptions) {
                window->setStatusBarText("Forced computing junctions with volatile options ...");
            } else {
                window->setStatusBarText("Forced computing junctions ...");
            }
        } else {
            return;
        }
    } else {
        if (volatileOptions) {
            window->setStatusBarText("Computing junctions with volatile options ...");
        } else {
            window->setStatusBarText("Computing junctions  ...");
        }
    }
    // save current number of lanes for every edge if recomputing is with volatile options
    if(volatileOptions) {
        for (auto it : myEdges) {
            myEdgesAndNumberOfLanes[it.second->getID()] = (int)it.second->getLanes().size();
        }
    }

    // compute
    OptionsCont& oc = OptionsCont::getOptions();
    computeAndUpdate(oc, volatileOptions);

    // load additionals if was recomputed with volatile options
    if (additionalPath != "") {
        // Create additional handler
        GNEAdditionalHandler additionalHandler(additionalPath, myViewNet, false);
        // Run parser
        if (!XMLSubSys::runParser(additionalHandler, additionalPath, false)) {
            WRITE_MESSAGE("Loading of " + additionalPath + " failed.");
        } else {
            // reset last tag (needed to avoid invalid E3s)
            additionalHandler.resetLastTag();
            update();
        }
        // clear myEdgesAndNumberOfLanes after reload additionals
        myEdgesAndNumberOfLanes.clear();
    }
    // load shapes if was recomputed with volatile options
    if (shapePath != "") {
        GNEApplicationWindow::GNEShapeHandler handler(shapePath, this);
        if (!XMLSubSys::runParser(handler, shapePath, false)) {
            WRITE_MESSAGE("Loading of " + shapePath + " failed.");
        }
    }
    window->getApp()->endWaitCursor();
    window->setStatusBarText("Finished computing junctions.");
    update();
}


void
GNENet::computeJunction(GNEJunction* junction) {
    // recompute tl-logics
    OptionsCont& oc = OptionsCont::getOptions();
    NBTrafficLightLogicCont& tllCont = getTLLogicCont();
    // iterate over traffic lights definitions
    for (auto it : junction->getNBNode()->getControllingTLS()) {
        it->setParticipantsInformation();
        it->setTLControllingInformation();
        tllCont.computeSingleLogic(oc, it);
    }

    // @todo compute connections etc...
}


void
GNENet::requireRecompute() {
    myNeedRecompute = true;
}

void 
GNENet::requiereSaveAdditionals() {
    if((myAdditionalsSaved == true) && OptionsCont::getOptions().getBool("gui-testing-debug")) {
        WRITE_WARNING("Additionals has to be saved");
    }
    myAdditionalsSaved = false;
    myViewNet->getViewParent()->getGNEAppWindows()->enableSaveAdditionalsMenu();
}


void 
GNENet::requiereSaveShapes() {
    if((myShapesSaved == true) && OptionsCont::getOptions().getBool("gui-testing-debug")) {
        WRITE_WARNING("Shapes has to be saved");
    }
    myShapesSaved = false;
    myViewNet->getViewParent()->getGNEAppWindows()->enableSaveShapesMenu();
}


bool
GNENet::netHasGNECrossings() const {
    for (auto n : myJunctions) {
        if (n.second->getGNECrossings().size() > 0) {
            return true;
        }
    }
    return false;
}


FXApp*
GNENet::getApp() {
    return myViewNet->getApp();
}


NBNetBuilder*
GNENet::getNetBuilder() const {
    return myNetBuilder;
}


bool
GNENet::joinSelectedJunctions(GNEUndoList* undoList) {
    std::vector<GNEJunction*> selectedJunctions = retrieveJunctions(true);
    if (selectedJunctions.size() < 2) {
        return false;
    }
    EdgeVector allIncoming;
    EdgeVector allOutgoing;
    std::set<NBNode*> cluster;
    for (auto it : selectedJunctions) {
        cluster.insert(it->getNBNode());
        const EdgeVector& incoming = it->getNBNode()->getIncomingEdges();
        allIncoming.insert(allIncoming.end(), incoming.begin(), incoming.end());
        const EdgeVector& outgoing = it->getNBNode()->getOutgoingEdges();
        allOutgoing.insert(allOutgoing.end(), outgoing.begin(), outgoing.end());
    }
    // create new junction
    Position pos;
    Position oldPos;
    bool setTL;
    std::string id;
    TrafficLightType type;
    myNetBuilder->getNodeCont().analyzeCluster(cluster, id, pos, setTL, type);
    // save position
    oldPos = pos;

    // Check that there isn't another junction in the same position as Pos but doesn't belong to cluster
    for (auto i : myJunctions) {
        if ((i.second->getPositionInView() == pos) && (cluster.find(i.second->getNBNode()) == cluster.end())) {
            // show warning in gui testing debug mode
            if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
                WRITE_WARNING("Opening FXMessageBox 'Join non-selected junction'");
            }
            // Ask confirmation to user
            FXuint answer = FXMessageBox::question(getApp(), MBOX_YES_NO,
                                                   ("Position of joined " + toString(SUMO_TAG_JUNCTION)).c_str(), "%s",
                                                   ("There is another unselected " + toString(SUMO_TAG_JUNCTION) + " in the same position of joined " + toString(SUMO_TAG_JUNCTION) +
                                                    + ".\nIt will be joined with the other selected " + toString(SUMO_TAG_JUNCTION) + "s. Continue?").c_str());
            if (answer != 1) { // 1:yes, 2:no, 4:esc
                // write warning if netedit is running in testing mode
                if ((answer == 2) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                    WRITE_WARNING("Closed FXMessageBox 'Join non-selected junction' with 'No'");
                } else if ((answer == 4) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                    WRITE_WARNING("Closed FXMessageBox 'Join non-selected junction' with 'ESC'");
                }
                return false;
            } else {
                // write warning if netedit is running in testing mode
                if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
                    WRITE_WARNING("Closed FXMessageBox 'Join non-selected junction' with 'Yes'");
                }
                // select conflicted junction an join all again
                gSelected.select(i.second->getGlID());
                return joinSelectedJunctions(undoList);
            }
        }
    }

    // use checkJunctionPosition to avoid conflicts with junction in the same position as others
    while (checkJunctionPosition(pos) == false) {
        pos.setx(pos.x() + 0.1);
        pos.sety(pos.y() + 0.1);
    }

    // start with the join selected junctions
    undoList->p_begin("Join selected " + toString(SUMO_TAG_JUNCTION) + "s");

    // first remove all crossing of the involved junctions and edges
    for (auto i : selectedJunctions) {

        while (i->getGNECrossings().size() > 0) {
            deleteCrossing(i->getGNECrossings().front(), undoList);
        }
    }

    // #3128 this is not undone when calling 'undo'
    myNetBuilder->getNodeCont().registerJoinedCluster(cluster);
    GNEJunction* joined = createJunction(pos, undoList);
    if (setTL) {
        joined->setAttribute(SUMO_ATTR_TYPE, toString(NODETYPE_TRAFFIC_LIGHT), undoList);
        // XXX ticket831
        //joined-><getTrafficLight>->setAttribute(SUMO_ATTR_TYPE, toString(type), undoList);
    }
    // remap edges
    for (auto it : allIncoming) {
        GNEEdge* oldEdge = myEdges[it->getID()];
        remapEdge(oldEdge, oldEdge->getGNEJunctionSource(), joined, undoList, true);
    }
    for (auto it : allOutgoing) {
        GNEEdge* oldEdge = myEdges[it->getID()];
        remapEdge(oldEdge, joined, oldEdge->getGNEJunctionDestiny(), undoList, true);
    }
    // delete original junctions
    for (auto it : selectedJunctions) {
        deleteJunction(it, undoList);
    }
    joined->setAttribute(SUMO_ATTR_ID, id, undoList);

    // check if joined junction had to change their original position to avoid errors
    if (pos != oldPos) {
        joined->setAttribute(SUMO_ATTR_POSITION, toString(oldPos), undoList);
    }
    undoList->p_end();
    return true;
}


bool
GNENet::cleanInvalidCrossings(GNEUndoList* undoList) {
    // obtain current net's crossings
    std::vector<GNECrossing*> myNetCrossings;
    for (auto it : myJunctions) {
        myNetCrossings.reserve(myNetCrossings.size() + it.second->getGNECrossings().size());
        myNetCrossings.insert(myNetCrossings.end(), it.second->getGNECrossings().begin(), it.second->getGNECrossings().end());
    }
    // obtain invalid crossigns
    std::vector<GNECrossing*> myInvalidCrossings;
    for (auto i = myNetCrossings.begin(); i != myNetCrossings.end(); i++) {
        if ((*i)->getNBCrossing()->valid == false) {
            myInvalidCrossings.push_back(*i);
        }
    }

    if (myInvalidCrossings.empty()) {
        // show warning in gui testing debug mode
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Opening FXMessageBox 'No crossing to remove'");
        }
        // open a dialog informing that there isn't crossing to remove
        FXMessageBox::warning(getApp(), MBOX_OK,
                              ("Clear " + toString(SUMO_TAG_CROSSING) + "s").c_str(), "%s",
                              ("There is no invalid " + toString(SUMO_TAG_CROSSING) + "s to remove").c_str());
        // show warning in gui testing debug mode
        WRITE_WARNING("Closed FXMessageBox 'No crossing to remove' with 'OK'");
    } else {
        std::string plural = myInvalidCrossings.size() == 1 ? ("") : ("s");
        // show warning in gui testing debug mode
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Opening FXMessageBox 'clear crossings'");
        }
        // Ask confirmation to user
        FXuint answer = FXMessageBox::question(getApp(), MBOX_YES_NO,
                                               ("Clear " + toString(SUMO_TAG_CROSSING) + "s").c_str(), "%s",
                                               ("Clear " + toString(SUMO_TAG_CROSSING) + plural + " will be removed. Continue?").c_str());
        if (answer != 1) { // 1:yes, 2:no, 4:esc
            // write warning if netedit is running in testing mode
            if ((answer == 2) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                WRITE_WARNING("Closed FXMessageBox 'clear crossings' with 'No'");
            } else if ((answer == 4) && (OptionsCont::getOptions().getBool("gui-testing-debug"))) {
                WRITE_WARNING("Closed FXMessageBox 'clear crossings' with 'ESC'");
            }
        } else {
            undoList->p_begin("Clean " + toString(SUMO_TAG_CROSSING) + "s");
            for (auto i = myInvalidCrossings.begin(); i != myInvalidCrossings.end(); i++) {
                deleteCrossing((*i), undoList);
            }
            undoList->p_end();
        }
    }
    return 1;
}


void
GNENet::removeSolitaryJunctions(GNEUndoList* undoList) {
    undoList->p_begin("Clean " + toString(SUMO_TAG_JUNCTION) + "s");
    std::vector<GNEJunction*> toRemove;
    for (auto it : myJunctions) {
        GNEJunction* junction = it.second;
        if (junction->getNBNode()->getEdges().size() == 0) {
            toRemove.push_back(junction);
        }
    }
    for (auto it : toRemove) {
        deleteJunction(it, undoList);
    }
    undoList->p_end();
}


void
GNENet::replaceJunctionByGeometry(GNEJunction* junction, GNEUndoList* undoList) {
    undoList->p_begin("Replace junction by geometry");
    assert(junction->getNBNode()->checkIsRemovable());
    std::vector<std::pair<NBEdge*, NBEdge*> > toJoin = junction->getNBNode()->getEdgesToJoin();
    for (auto j : toJoin) {
        GNEEdge* begin = myEdges[j.first->getID()];
        GNEEdge* continuation = myEdges[j.second->getID()];
        // remove connections between the edges
        std::vector<NBEdge::Connection> connections = begin->getNBEdge()->getConnections();
        for (auto con : connections) {
            undoList->add(new GNEChange_Connection(begin, con, false, false), true);
        }
        // replace
        replaceIncomingEdge(continuation, begin, undoList);
        // fix shape
        PositionVector newShape = begin->getNBEdge()->getInnerGeometry();
        if (begin->getNBEdge()->hasDefaultGeometryEndpointAtNode(begin->getNBEdge()->getToNode())) {
            newShape.push_back(junction->getNBNode()->getPosition());
        } else {
            newShape.push_back(begin->getNBEdge()->getGeometry()[-1]);
        }
        if (continuation->getNBEdge()->hasDefaultGeometryEndpointAtNode(begin->getNBEdge()->getToNode())) {
            newShape.push_back_noDoublePos(junction->getNBNode()->getPosition());
        } else {
            newShape.push_back_noDoublePos(continuation->getNBEdge()->getGeometry()[0]);
        }
        newShape.append(continuation->getNBEdge()->getInnerGeometry());
        begin->setAttribute(GNE_ATTR_SHAPE_END, continuation->getAttribute(GNE_ATTR_SHAPE_END), undoList);
        begin->setAttribute(SUMO_ATTR_SHAPE, toString(newShape), undoList);
    }
    deleteJunction(junction, undoList);
    undoList->p_end();
}


void 
GNENet::clearJunctionConnections(GNEJunction* junction, GNEUndoList* undoList) {
    undoList->p_begin("clear junction connections");
    std::vector<GNEConnection*> connections = junction->getGNEConnections();
    // Iterate over all connections and clear it
    for(auto i : connections) {
        deleteConnection(i, undoList);
    }
    undoList->p_end();
}


void 
GNENet::resetJunctionConnections(GNEJunction* junction, GNEUndoList* undoList) {
    undoList->p_begin("reset junction connections");
    // first clear connections 
    clearJunctionConnections(junction, undoList);
    // invalidate logic to create new connections in the next recomputing
    junction->setLogicValid(false, undoList);
    undoList->p_end();
}


void
GNENet::renameEdge(GNEEdge* edge, const std::string& newID) {
    myEdges.erase(edge->getNBEdge()->getID());
    myNetBuilder->getEdgeCont().rename(edge->getNBEdge(), newID);
    edge->setMicrosimID(newID);
    myEdges[newID] = edge;
    // rename all connections related to this edge
    for (auto i : edge->getLanes()) {
        i->updateConnectionIDs();
    }
}


void
GNENet::changeEdgeEndpoints(GNEEdge* edge, const std::string& newSource, const std::string& newDest) {
    NBNode* from = retrieveJunction(newSource)->getNBNode();
    NBNode* to = retrieveJunction(newDest)->getNBNode();
    edge->getNBEdge()->reinitNodes(from, to);
    requireRecompute();
    update();
}


GNEViewNet*
GNENet::getViewNet() const {
    return myViewNet;
}


NBTrafficLightLogicCont&
GNENet::getTLLogicCont() {
    return myNetBuilder->getTLLogicCont();
}


void
GNENet::renameJunction(GNEJunction* junction, const std::string& newID) {
    myJunctions.erase(junction->getNBNode()->getID());
    myNetBuilder->getNodeCont().rename(junction->getNBNode(), newID);
    junction->setMicrosimID(newID);
    myJunctions[newID] = junction;
}


void
GNENet::addExplicitTurnaround(std::string id) {
    myExplicitTurnarounds.insert(id);
}


void
GNENet::removeExplicitTurnaround(std::string id) {
    myExplicitTurnarounds.erase(id);
}


void
GNENet::insertAdditional(GNEAdditional* additional, bool hardFail) {
    // we need a special case for Calibrators
    SumoXMLTag tag = (additional->getTag() == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : additional->getTag();
    // Check if additional element exists before insertion
    if (myAdditionals.find(std::pair<std::string, SumoXMLTag>(additional->getID(), tag)) != myAdditionals.end()) {
        // Throw exception only if hardFail is enabled
        if (hardFail) {
            throw ProcessError(toString(tag) + " with ID='" + additional->getID() + "' already exist");
        }
    } else {
        myAdditionals[std::pair<std::string, SumoXMLTag>(additional->getID(), tag)] = additional;
        myGrid.addAdditionalGLObject(additional);
        update();
        // additionals has to be saved
        requiereSaveAdditionals();
    }
}


void
GNENet::deleteAdditional(GNEAdditional* additional) {
    // we need a special case for Calibrators
    SumoXMLTag tag = (additional->getTag() == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : additional->getTag();
    // obtain iterator to additional to remove
    GNEAdditionals::iterator additionalToRemove = myAdditionals.find(std::pair<std::string, SumoXMLTag>(additional->getID(), tag));
    // Check if additional element exists before deletion
    if (additionalToRemove == myAdditionals.end()) {
        throw ProcessError(toString(additional->getTag()) + " with ID='" + additional->getID() + "' doesn't exist");
    } else {
        myAdditionals.erase(additionalToRemove);
        myGrid.removeAdditionalGLObject(additional);
        update();
        // additionals has to be saved
        requiereSaveAdditionals();
    }
}


void
GNENet::updateAdditionalID(const std::string& oldID, GNEAdditional* additional) {
    // we need a special case for Calibrators
    SumoXMLTag tag = (additional->getTag() == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : additional->getTag();
    GNEAdditionals::iterator additionalToUpdate = myAdditionals.find(std::pair<std::string, SumoXMLTag>(oldID, tag));
    if (additionalToUpdate == myAdditionals.end()) {
        throw ProcessError(toString(additional->getTag()) + "  with old ID='" + oldID + "' doesn't exist");
    } else {
        // remove an insert additional again into container
        myAdditionals.erase(additionalToUpdate);
        myAdditionals[std::pair<std::string, SumoXMLTag>(additional->getID(), tag)] = additional;
        // additionals has to be saved
        requiereSaveAdditionals();
    }
}


GNEAdditional*
GNENet::retrieveAdditional(const std::string& id, bool hardFail) const {
    for (auto i : myAdditionals) {
        if (i.second->getID() == id) {
            return i.second;
        }
    }
    if (hardFail) {
        throw ProcessError("Attempted to retrieve non-existant additional");
    } else {
        return NULL;
    }
}


std::vector<GNEAdditional*>
GNENet::retrieveAdditionals(bool onlySelected) {
    std::vector<GNEAdditional*> result;
    for (auto it : myAdditionals) {
        if (!onlySelected || gSelected.isSelected(GLO_ADDITIONAL, it.second->getGlID())) {
            result.push_back(it.second);
        }
    }
    return result;
}


GNEAdditional*
GNENet::getAdditional(SumoXMLTag type, const std::string& id) const {
    // we need a special case for Calibrators
    SumoXMLTag tag = (type == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : type;
    if (myAdditionals.empty()) {
        return NULL;
    } else if (myAdditionals.find(std::pair<std::string, SumoXMLTag>(id, tag)) != myAdditionals.end())  {
        return myAdditionals.at(std::pair<std::string, SumoXMLTag>(id, tag));
    } else {
        return NULL;
    }
}


std::string
GNENet::getAdditionalID(SumoXMLTag type, const GNELane* lane, const double pos) const {
    // we need a special case for Calibrators
    SumoXMLTag tag = (type == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : type;
    for (auto it : myAdditionals) {
        if ((it.second->getTag() == tag) && (it.second->getLane() != NULL) && (it.second->getLane() == lane) && (fabs(it.second->getPositionInView().x() - pos) < POSITION_EPS)) {
            return it.second->getID();
        }
    }
    return "";
}


std::vector<GNEAdditional*>
GNENet::getAdditionals(SumoXMLTag type) const {
    // we need a special case for Calibrators
    SumoXMLTag tag = (type == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : type;
    std::vector<GNEAdditional*> vectorOfAdditionals;
    for (auto i : myAdditionals) {
        if (tag == SUMO_TAG_NOTHING || tag == i.second->getTag()) {
            vectorOfAdditionals.push_back(i.second);
        }
    }
    return vectorOfAdditionals;
}


int
GNENet::getNumberOfAdditionals(SumoXMLTag type) const {
    // we need a special case for Calibrators
    SumoXMLTag tag = (type == SUMO_TAG_LANECALIBRATOR)? SUMO_TAG_CALIBRATOR : type;
    int counter = 0;
    for (auto i : myAdditionals) {
        if (tag == SUMO_TAG_NOTHING || tag == i.second->getTag()) {
            counter++;
        }
    }
    return counter;
}


const GNECalibratorRoute&
GNENet::getGNECalibratorRoute(const std::string& calibratorRouteID) const {
    std::vector<GNEAdditional*> calibrators = getAdditionals(SUMO_TAG_CALIBRATOR);
    for (auto i : calibrators) {
        GNECalibrator* calibrator = dynamic_cast<GNECalibrator*>(i);
        if (calibrator->routeExists(calibratorRouteID)) {
            return calibrator->getCalibratorRoute(calibratorRouteID);
        }
    }
    throw InvalidArgument("A " + toString(SUMO_TAG_VTYPE) + "'s " + toString(SUMO_TAG_VTYPE) + " with ID = '" + calibratorRouteID + "' doesn't exists");
}


const GNECalibratorVehicleType&
GNENet::getGNECalibratorVehicleType(const std::string& calibratorVehicleTypeID) const {
    std::vector<GNEAdditional*> calibrators = getAdditionals(SUMO_TAG_CALIBRATOR);
    for (auto i : calibrators) {
        GNECalibrator* calibrator = dynamic_cast<GNECalibrator*>(i);
        if (calibrator->vehicleTypeExists(calibratorVehicleTypeID)) {
            return calibrator->getCalibratorVehicleType(calibratorVehicleTypeID);
        }
    }
    throw InvalidArgument("A " + toString(SUMO_TAG_VTYPE) + "'s " + toString(SUMO_TAG_VTYPE) + " with ID = '" + calibratorVehicleTypeID + "' doesn't exists");
}


const GNECalibratorFlow&
GNENet::getGNECalibratorFlow(const std::string& calibratorFlowID) const {
    std::vector<GNEAdditional*> calibrators = getAdditionals(SUMO_TAG_CALIBRATOR);
    for (auto i : calibrators) {
        GNECalibrator* calibrator = dynamic_cast<GNECalibrator*>(i);
        if (calibrator->flowExists(calibratorFlowID)) {
            return calibrator->getCalibratorFlow(calibratorFlowID);
        }
    }
    throw InvalidArgument("A " + toString(SUMO_TAG_VTYPE) + "'s " + toString(SUMO_TAG_VTYPE) + " with ID = '" + calibratorFlowID + "' doesn't exists");
}


bool
GNENet::routeExists(const std::string& routeID) const {
    std::vector<GNEAdditional*> calibrators = getAdditionals(SUMO_TAG_CALIBRATOR);
    for (auto i : calibrators) {
        GNECalibrator* calibrator = dynamic_cast<GNECalibrator*>(i);
        if (calibrator->routeExists(routeID)) {
            return true;
        }
    }
    return false;
}


bool
GNENet::vehicleTypeExists(const std::string& vehicleTypeID) const {
    std::vector<GNEAdditional*> calibrators = getAdditionals(SUMO_TAG_CALIBRATOR);
    for (auto i : calibrators) {
        GNECalibrator* calibrator = dynamic_cast<GNECalibrator*>(i);
        if (calibrator->vehicleTypeExists(vehicleTypeID)) {
            return true;
        }
    }
    return false;
}


bool
GNENet::flowExists(const std::string& flowID) const {
    std::vector<GNEAdditional*> calibrators = getAdditionals(SUMO_TAG_CALIBRATOR);
    for (auto i : calibrators) {
        GNECalibrator* calibrator = dynamic_cast<GNECalibrator*>(i);
        if (calibrator->flowExists(flowID)) {
            return true;
        }
    }
    return false;
}


GNEPoly*
GNENet::addPolygonForEditShapes(GNENetElement* netElement, const PositionVector &shape, bool fill) {
    if(shape.size() > 0) {
        // generate a ID for polygon used for shape
        int counter = 0;
        std::string polyID = "edit_shape:" + toString(counter);
        while (myPolygons.get(polyID) != NULL) {
            counter++;
            polyID = "edit_shape:" + toString(counter);
        }
        // create poly for edit shapes
        GNEPoly* shapePoly = new GNEPoly(this, "edit_shape:" + toString(counter), "edit_shape", shape, false, true, RGBColor::GREEN, GLO_POLYGON, 0, "", false , false);
        shapePoly->setShapeEditedElement(netElement);
        shapePoly->setFill(fill);
        shapePoly->setLineWidth(0.3);
        insertShapeInView(shapePoly, true);
        myViewNet->update();
        return shapePoly;  
    } else {
        throw ProcessError("shape cannot be empty");
    }
}


void
GNENet::insertShapeInView(GNEShape* s, bool isShapeForEditShapes) {
    if (s->isShapeVisible() == false) {
        myGrid.addAdditionalGLObject(dynamic_cast<GUIGlObject*>(s));
        myViewNet->update();
        s->setShapeVisible(true);
        // shapes has to be saved if polygon isn't for edit shapes
        if(!isShapeForEditShapes) {
            requiereSaveShapes();
        }
    } else {
        throw ProcessError("Shape was already inserted in view");
    }
}


void
GNENet::removeShapeOfView(GNEShape* s, bool isShapeForEditShapes) {
    if (s->isShapeVisible()) {
        myGrid.removeAdditionalGLObject(dynamic_cast<GUIGlObject*>(s));
        myViewNet->update();
        s->setShapeVisible(false);
        // shapes has to be saved if polygon isn't for edit shapes
        if(!isShapeForEditShapes) {
            requiereSaveShapes();
        }
    } else {
        throw ProcessError("Shape wasn't already inserted in view");
    }
}


void
GNENet::refreshShape(GNEShape* s) {
    if (s->isShapeVisible()) {
        myGrid.removeAdditionalGLObject(dynamic_cast<GUIGlObject*>(s));
        myGrid.addAdditionalGLObject(dynamic_cast<GUIGlObject*>(s));
        myViewNet->update();
    } else {
        throw ProcessError("Shape wasn't inserted in view");
    }
}


std::string
GNENet::generateShapeID(SumoXMLTag shapeTag) const {
    // generate tag depending of type of shape
    if(shapeTag == SUMO_TAG_POLY) {
        int counter = 0;
        std::string newID = "poly_" + toString(counter);
        // generate new IDs to find a non-assigned ID
        while (myPolygons.get(newID) != NULL) {
            counter++;
            newID = "poly_" + toString(counter);
        }
        return newID;
    } else {
        int counter = 0;
        std::string newID = "POI_" + toString(counter);
        // generate new IDs to find a non-assigned ID
        while (myPOIs.get(newID) != NULL) {
            counter++;
            newID = "POI_" + toString(counter);
        }
        return newID;
    }
}


void
GNENet::changeShapeID(GNEShape* s, const std::string& OldID) {
    if(s->getTag() == SUMO_TAG_POLY) {
        if (myPolygons.get(OldID) == 0) {
            throw UnknownElement("Polygon " + OldID);
        } else {
            myPolygons.changeID(OldID, s->getID());
        }
    } else {
        if (myPOIs.get(OldID) == 0) {
            throw UnknownElement("POI " + OldID);
        } else {
            myPOIs.changeID(OldID, s->getID());
        }
    }
}


void GNENet::saveShapes(const std::string& filename) {
    // save Shapes
    OutputDevice& device = OutputDevice::getDevice(filename);
    device.openTag("additionals");
    // write only visible polygons
    for (auto i : myPolygons.getMyMap()) {
        GNEShape* shape = dynamic_cast<GNEShape*>(i.second);
        if (shape->isShapeVisible()) {
            shape->writeShape(device);
        }
    }
    // write only visible POIs
    for (auto i : myPOIs.getMyMap()) {
        GNEShape* shape = dynamic_cast<GNEShape*>(i.second);
        if (shape->isShapeVisible()) {
            shape->writeShape(device);
        }
    }
    device.close();
    // change flag to true
    myShapesSaved = true;
    // show debug information
    if(OptionsCont::getOptions().getBool("gui-testing-debug")) {
        WRITE_WARNING("Shapes saved");
    }
}


int
GNENet::getNumberOfShapes() const {
    return (int)(myPolygons.size() + myPOIs.size());
}


bool
GNENet::isShapeSelected(SumoXMLTag tag, const std::string& ID) const {
    if (tag == SUMO_TAG_POLY) {
        return gSelected.isSelected(GLO_POLYGON, retrievePolygon(ID)->getGlID());
    } else if (tag == SUMO_TAG_POI) {
        return gSelected.isSelected(GLO_POI, retrievePOI(ID)->getGlID());
    } else if (tag == SUMO_TAG_POILANE) {
        return gSelected.isSelected(GLO_POI, retrievePOILane(ID)->getGlID());
    } else {
        throw ProcessError("Invalid Shape");
    }
}

// ===========================================================================
// private
// ===========================================================================

void
GNENet::initJunctionsAndEdges() {
    // init junctions (by default Crossing and walking areas aren't created)
    NBNodeCont& nodeContainer = myNetBuilder->getNodeCont();
    for (auto name_it : nodeContainer.getAllNames()) {
        NBNode* nbn = nodeContainer.retrieve(name_it);
        registerJunction(new GNEJunction(*nbn, this, true));
    }

    // init edges
    NBEdgeCont& ec = myNetBuilder->getEdgeCont();
    for (auto name_it : ec.getAllNames()) {
        NBEdge* nbe = ec.retrieve(name_it);
        registerEdge(new GNEEdge(*nbe, this, false, true));
        if (myGrid.getWidth() > 10e16 || myGrid.getHeight() > 10e16) {
            throw ProcessError("Network size exceeds 1 Lightyear. Please reconsider your inputs.\n");
        }
    }

    // make sure myGrid is initialized even for an empty net
    if (myEdges.size() == 0) {
        myGrid.add(Boundary(0, 0, 100, 100));
    }

    // sort nodes edges so that arrows can be drawn correctly
    NBNodesEdgesSorter::sortNodesEdges(nodeContainer);
}


void
GNENet::insertJunction(GNEJunction* junction) {
    myNetBuilder->getNodeCont().insert(junction->getNBNode());
    registerJunction(junction);
}


void
GNENet::insertEdge(GNEEdge* edge) {
    NBEdge* nbe = edge->getNBEdge();
    myNetBuilder->getEdgeCont().insert(nbe); // should we ignore pruning double edges?
    // if this edge was previouls extracted from the edgeContainer we have to rewire the nodes
    nbe->getFromNode()->addOutgoingEdge(nbe);
    nbe->getToNode()->addIncomingEdge(nbe);
    registerEdge(edge);
}


GNEJunction*
GNENet::registerJunction(GNEJunction* junction) {
    junction->incRef("GNENet::registerJunction");
    junction->setResponsible(false);
    myJunctions[junction->getMicrosimID()] = junction;
    myGrid.add(junction->getBoundary());
    myGrid.addAdditionalGLObject(junction);
    // @todo let Boundary class track z-coordinate natively
    const double z = junction->getNBNode()->getPosition().z();
    if (z != 0) {
        myZBoundary.add(z, Z_INITIALIZED);
    }
    update();
    return junction;
}


GNEEdge*
GNENet::registerEdge(GNEEdge* edge) {
    edge->incRef("GNENet::registerEdge");
    edge->setResponsible(false);
    // add edge to internal container of GNENet
    myEdges[edge->getMicrosimID()] = edge;
    // add edge to grid
    myGrid.add(edge->getBoundary());
    myGrid.addAdditionalGLObject(edge);
    // Add references into GNEJunctions
    edge->getGNEJunctionSource()->addOutgoingGNEEdge(edge);
    edge->getGNEJunctionDestiny()->addIncomingGNEEdge(edge);
    // update view
    update();
    return edge;
}


void
GNENet::deleteSingleJunction(GNEJunction* junction) {
    myGrid.removeAdditionalGLObject(junction);
    myJunctions.erase(junction->getMicrosimID());
    myNetBuilder->getNodeCont().extract(junction->getNBNode());
    junction->decRef("GNENet::deleteSingleJunction");
    junction->setResponsible(true);
    // selection status is lost when removing junction via undo and the selection operation was not part of a command group
    gSelected.deselect(junction->getGlID());
    update();
}


void
GNENet::deleteSingleEdge(GNEEdge* edge) {
    // remove edge from visual grid and container
    myGrid.removeAdditionalGLObject(edge);
    myEdges.erase(edge->getMicrosimID());
    // extract edge of district container
    myNetBuilder->getEdgeCont().extract(myNetBuilder->getDistrictCont(), edge->getNBEdge());
    edge->decRef("GNENet::deleteSingleEdge");
    edge->setResponsible(true);
    // selection status is lost when removing edge via undo and the selection operation was not part of a command group
    gSelected.deselect(edge->getGlID());
    // Remove refrences from GNEJunctions
    edge->getGNEJunctionSource()->removeOutgoingGNEEdge(edge);
    edge->getGNEJunctionDestiny()->removeIncomingGNEEdge(edge);
    // invalidate junction logic
    update();
}


void
GNENet::update() {
    if (myViewNet) {
        myViewNet->update();
    }
}


void
GNENet::reserveEdgeID(const std::string& id) {
    myEdgeIDSupplier.avoid(id);
}


void
GNENet::reserveJunctionID(const std::string& id) {
    myJunctionIDSupplier.avoid(id);
}


void
GNENet::initGNEConnections() {
    for (auto it : myEdges) {
        it.second->remakeGNEConnections();
    }
    for (auto it : myEdges) {
        it.second->updateGeometry();
    }
}


void
GNENet::computeAndUpdate(OptionsCont& oc, bool volatileOptions) {
    // make sure we only add turn arounds to edges which currently exist within the network
    std::set<std::string> liveExplicitTurnarounds;
    for (auto it : myExplicitTurnarounds) {
        if (myEdges.count(it) > 0) {
            liveExplicitTurnarounds.insert(it);
        }
    }

    myNetBuilder->compute(oc, liveExplicitTurnarounds, volatileOptions);
    // update ids if necessary
    if (oc.getBool("numerical-ids") || oc.isSet("reserved-ids")) {
        GNEEdges newEdgeMap;
        GNEJunctions newJunctionMap;
        // fill newEdgeMap
        for (auto it : myEdges) {
            it.second->setMicrosimID(it.second->getNBEdge()->getID());
            newEdgeMap[it.second->getNBEdge()->getID()] = it.second;
        }
        for (auto it : myJunctions) {
            newJunctionMap[it.second->getNBNode()->getID()] = it.second;
            it.second->setMicrosimID(it.second->getNBNode()->getID());
        }
        myEdges = newEdgeMap;
        myJunctions = newJunctionMap;
    }
    // update rtree if necessary
    if (!oc.getBool("offset.disable-normalization")) {
        for (auto it : myEdges) {
            refreshElement(it.second);
        }
    }
    myGrid.reset();
    myGrid.add(GeoConvHelper::getFinal().getConvBoundary());
    // if volatile options are true
    if (volatileOptions) {
        // clear all Polys of grid
        for (auto i : myPolygons.getMyMap()) {
            GNEShape* shape = dynamic_cast<GNEShape*>(i.second);
            if (shape->isShapeVisible()) {
                removeShapeOfView(shape);
            }
        }

        // clear all POIs of grid
        for (auto i : myPOIs.getMyMap()) {
            GNEShape* shape = dynamic_cast<GNEShape*>(i.second);
            if (shape->isShapeVisible()) {
                removeShapeOfView(shape);
            }
        }

        // clear all additionals of grid
        GNEAdditionals copyOfAdditionals = myAdditionals;
        for (auto it : copyOfAdditionals) {
            myGrid.removeAdditionalGLObject(it.second);
        }

        // remove all edges of grid and net
        GNEEdges copyOfEdges = myEdges;
        for (auto it : copyOfEdges) {
            myGrid.removeAdditionalGLObject(it.second);
            myEdges.erase(it.second->getMicrosimID());
        }

        // removes all junctions of grid and net
        GNEJunctions copyOfJunctions = myJunctions;
        for (auto it : copyOfJunctions) {
            myGrid.removeAdditionalGLObject(it.second);
            myJunctions.erase(it.second->getMicrosimID());
        }

        // clear undo list
        myViewNet->getUndoList()->clear();

        // clear additionals (must be do it separated)
        myAdditionals.clear();

        // init again junction an edges
        initJunctionsAndEdges();
    }

    // update precomputed geometries
    initGNEConnections();

    for (auto it : myJunctions) {
        it.second->setLogicValid(true, myViewNet->getUndoList());
        // updated shape
        it.second->updateGeometry();
        refreshElement(it.second);
    }

    myNeedRecompute = false;
}


void
GNENet::replaceInListAttribute(GNEAttributeCarrier* ac, SumoXMLAttr key, const std::string& which, const std::string& by, GNEUndoList* undoList) {
    assert(GNEAttributeCarrier::isList(ac->getTag(), key));
    std::vector<std::string> values = GNEAttributeCarrier::parse<std::vector<std::string> >(ac->getAttribute(key));
    std::vector<std::string> newValues;
    for (auto v : values) {
        newValues.push_back(v == which ? by : v);
    }
    ac->setAttribute(key, toString(newValues), undoList);
}

/****************************************************************************/
