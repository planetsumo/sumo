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
/// @file    GUIEdge.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Sept 2002
/// @version $Id$
///
// A road/street connecting two junctions (gui-version)
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/geom/GeomHelper.h>
#include <utils/foxtools/MFXMutex.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/globjects/GLIncludes.h>
#include <microsim/MSBaseVehicle.h>
#include <microsim/MSEdge.h>
#include <microsim/MSJunction.h>
#include <microsim/MSLaneChanger.h>
#include <microsim/MSInsertionControl.h>
#include <microsim/MSGlobals.h>
#include <microsim/logging/CastingFunctionBinding.h>
#include <microsim/logging/FunctionBinding.h>
#include "GUITriggeredRerouter.h"
#include "GUIEdge.h"
#include "GUIVehicle.h"
#include "GUINet.h"
#include "GUILane.h"
#include "GUIPerson.h"
#include "GUIContainer.h"

#include <mesogui/GUIMEVehicleControl.h>
#include <mesogui/GUIMEVehicle.h>
#include <mesosim/MESegment.h>
#include <mesosim/MELoop.h>
#include <mesosim/MEVehicle.h>


// ===========================================================================
// included modules
// ===========================================================================
GUIEdge::GUIEdge(const std::string& id, int numericalID,
                 const SumoXMLEdgeFunc function,
                 const std::string& streetName, const std::string& edgeType, int priority)
    : MSEdge(id, numericalID, function, streetName, edgeType, priority),
      GUIGlObject(GLO_EDGE, id) {}


GUIEdge::~GUIEdge() {
    // just to quit cleanly on a failure
    if (myLock.locked()) {
        myLock.unlock();
    }
}


MSLane&
GUIEdge::getLane(int laneNo) {
    assert(laneNo < (int)myLanes->size());
    return *((*myLanes)[laneNo]);
}


std::vector<GUIGlID>
GUIEdge::getIDs(bool includeInternal) {
    std::vector<GUIGlID> ret;
    ret.reserve(MSEdge::myDict.size());
    for (MSEdge::DictType::const_iterator i = MSEdge::myDict.begin(); i != MSEdge::myDict.end(); ++i) {
        const GUIEdge* edge = dynamic_cast<const GUIEdge*>(i->second);
        assert(edge);
        if (includeInternal || !edge->isInternal()) {
            ret.push_back(edge->getGlID());
        }
    }
    return ret;
}


double
GUIEdge::getTotalLength(bool includeInternal, bool eachLane) {
    double result = 0;
    for (MSEdge::DictType::const_iterator i = MSEdge::myDict.begin(); i != MSEdge::myDict.end(); ++i) {
        const MSEdge* edge = i->second;
        if (includeInternal || !edge->isInternal()) {
            // @note needs to be change once lanes may have different length
            result += edge->getLength() * (eachLane ? edge->getLanes().size() : 1);
        }
    }
    return result;
}


Boundary
GUIEdge::getBoundary() const {
    Boundary ret;
    if (!isTazConnector()) {
        for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            ret.add((*i)->getShape().getBoxBoundary());
        }
    } else {
        // take the starting coordinates of all follower edges and the endpoints
        // of all successor edges
        for (MSEdgeVector::const_iterator it = mySuccessors.begin(); it != mySuccessors.end(); ++it) {
            const std::vector<MSLane*>& lanes = (*it)->getLanes();
            for (std::vector<MSLane*>::const_iterator it_lane = lanes.begin(); it_lane != lanes.end(); ++it_lane) {
                ret.add((*it_lane)->getShape().front());
            }
        }
        for (MSEdgeVector::const_iterator it = myPredecessors.begin(); it != myPredecessors.end(); ++it) {
            const std::vector<MSLane*>& lanes = (*it)->getLanes();
            for (std::vector<MSLane*>::const_iterator it_lane = lanes.begin(); it_lane != lanes.end(); ++it_lane) {
                ret.add((*it_lane)->getShape().back());
            }
        }
    }
    ret.grow(10);
    return ret;
}


GUIGLObjectPopupMenu*
GUIEdge::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
    buildPopupHeader(ret, app);
    buildCenterPopupEntry(ret);
    buildNameCopyPopupEntry(ret);
    buildSelectionPopupEntry(ret);
    if (MSGlobals::gUseMesoSim) {
        buildShowParamsPopupEntry(ret);
    }
    const double pos = getLanes()[0]->getShape().nearest_offset_to_point2D(parent.getPositionInformation());
    new FXMenuCommand(ret, ("pos: " + toString(pos)).c_str(), 0, 0, 0);
    buildPositionCopyEntry(ret, false);
    return ret;
}


GUIParameterTableWindow*
GUIEdge::getParameterWindow(GUIMainWindow& app,
                            GUISUMOAbstractView& parent) {
    GUIParameterTableWindow* ret = 0;
    ret = new GUIParameterTableWindow(app, *this, 18);
    // add edge items
    ret->mkItem("length [m]", false, (*myLanes)[0]->getLength());
    ret->mkItem("allowed speed [m/s]", false, getAllowedSpeed());
    ret->mkItem("brutto occupancy [%]", true, new FunctionBinding<GUIEdge, double>(this, &GUIEdge::getBruttoOccupancy, 100.));
    ret->mkItem("mean vehicle speed [m/s]", true, new FunctionBinding<GUIEdge, double>(this, &GUIEdge::getMeanSpeed));
    ret->mkItem("flow [veh/h/lane]", true, new FunctionBinding<GUIEdge, double>(this, &GUIEdge::getFlow));
    ret->mkItem("routing speed [m/s]", true, new FunctionBinding<MSEdge, double>(this, &MSEdge::getRoutingSpeed));
    ret->mkItem("#vehicles", true, new CastingFunctionBinding<GUIEdge, double, int>(this, &GUIEdge::getVehicleNo));
    ret->mkItem("vehicle ids", false, getVehicleIDs());
    // add segment items
    MESegment* segment = getSegmentAtPosition(parent.getPositionInformation());
    ret->mkItem("segment index", false, segment->getIndex());
    ret->mkItem("segment queues", false, segment->numQueues());
    ret->mkItem("segment length [m]", false, segment->getLength());
    ret->mkItem("segment allowed speed [m/s]", false, segment->getEdge().getSpeedLimit());
    ret->mkItem("segment jam threshold [%]", false, segment->getRelativeJamThreshold() * 100);
    ret->mkItem("segment brutto occupancy [%]", true, new FunctionBinding<MESegment, double>(segment, &MESegment::getRelativeOccupancy, 100));
    ret->mkItem("segment mean vehicle speed [m/s]", true, new FunctionBinding<MESegment, double>(segment, &MESegment::getMeanSpeed));
    ret->mkItem("segment flow [veh/h/lane]", true, new FunctionBinding<MESegment, double>(segment, &MESegment::getFlow));
    ret->mkItem("segment #vehicles", true, new CastingFunctionBinding<MESegment, double, int>(segment, &MESegment::getCarNumber));
    ret->mkItem("segment leader leave time", true, new FunctionBinding<MESegment, double>(segment, &MESegment::getEventTimeSeconds));
    ret->mkItem("segment headway [s]", true, new FunctionBinding<MESegment, double>(segment, &MESegment::getLastHeadwaySeconds));

    // close building
    ret->closeBuilding();
    return ret;
}


Boundary
GUIEdge::getCenteringBoundary() const {
    Boundary b = getBoundary();
    // ensure that vehicles and persons on the side are drawn even if the edge
    // is outside the view
    b.grow(10);
    return b;
}


void
GUIEdge::drawGL(const GUIVisualizationSettings& s) const {
    if (s.hideConnectors && myFunction == EDGEFUNC_CONNECTOR) {
        return;
    }
    glPushName(getGlID());
    // draw the lanes
    for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        if (MSGlobals::gUseMesoSim) {
            setColor(s);
        }
        GUILane* l = dynamic_cast<GUILane*>(*i);
        if (l != 0) {
            l->drawGL(s);
        }
    }
    if (MSGlobals::gUseMesoSim) {
        if (s.scale * s.vehicleSize.getExaggeration(s) > s.vehicleSize.minSize) {
            drawMesoVehicles(s);
        }
    }
    glPopName();
    // (optionally) draw the name and/or the street name
    const bool drawEdgeName = s.edgeName.show && myFunction == EDGEFUNC_NORMAL;
    const bool drawInternalEdgeName = s.internalEdgeName.show && myFunction == EDGEFUNC_INTERNAL;
    const bool drawCwaEdgeName = s.cwaEdgeName.show && (myFunction == EDGEFUNC_CROSSING || myFunction == EDGEFUNC_WALKINGAREA);
    const bool drawStreetName = s.streetName.show && myStreetName != "";
    if (drawEdgeName || drawInternalEdgeName || drawCwaEdgeName || drawStreetName) {
        GUILane* lane1 = dynamic_cast<GUILane*>((*myLanes)[0]);
        GUILane* lane2 = dynamic_cast<GUILane*>((*myLanes).back());
        if (lane1 != 0 && lane2 != 0) {
            Position p = lane1->getShape().positionAtOffset(lane1->getShape().length() / (double) 2.);
            p.add(lane2->getShape().positionAtOffset(lane2->getShape().length() / (double) 2.));
            p.mul(.5);
            double angle = lane1->getShape().rotationDegreeAtOffset(lane1->getShape().length() / (double) 2.);
            angle += 90;
            if (angle > 90 && angle < 270) {
                angle -= 180;
            }
            if (drawEdgeName) {
                drawName(p, s.scale, s.edgeName, angle);
            } else if (drawInternalEdgeName) {
                drawName(p, s.scale, s.internalEdgeName, angle);
            } else if (drawCwaEdgeName) {
                drawName(p, s.scale, s.cwaEdgeName, angle);
            }
            if (drawStreetName) {
                GLHelper::drawText(getStreetName(), p, GLO_MAX,
                                   s.streetName.size / s.scale, s.streetName.color, angle);
            }
        }
    }
    if (s.scale * s.personSize.getExaggeration(s) > s.personSize.minSize) {
        myLock.lock();
        for (std::set<MSTransportable*>::const_iterator i = myPersons.begin(); i != myPersons.end(); ++i) {
            GUIPerson* person = dynamic_cast<GUIPerson*>(*i);
            assert(person != 0);
            person->drawGL(s);
        }
        myLock.unlock();
    }
    if (s.scale * s.containerSize.getExaggeration(s) > s.containerSize.minSize) {
        myLock.lock();
        for (std::set<MSTransportable*>::const_iterator i = myContainers.begin(); i != myContainers.end(); ++i) {
            GUIContainer* container = dynamic_cast<GUIContainer*>(*i);
            assert(container != 0);
            container->drawGL(s);
        }
        myLock.unlock();
    }
}


void
GUIEdge::drawMesoVehicles(const GUIVisualizationSettings& s) const {
    GUIMEVehicleControl* vehicleControl = GUINet::getGUIInstance()->getGUIMEVehicleControl();
    if (vehicleControl != 0) {
        // draw the meso vehicles
        vehicleControl->secureVehicles();
        AbstractMutex::ScopedLocker locker(myLock);
        int laneIndex = 0;
        MESegment::Queue queue;
        for (std::vector<MSLane*>::const_iterator msl = myLanes->begin(); msl != myLanes->end(); ++msl, ++laneIndex) {
            GUILane* l = static_cast<GUILane*>(*msl);
            // go through the vehicles
            double segmentOffset = 0; // offset at start of current segment
            for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this);
                    segment != 0; segment = segment->getNextSegment()) {
                const double length = segment->getLength();
                if (laneIndex < segment->numQueues()) {
                    // make a copy so we don't have to worry about synchronization
                    queue = segment->getQueue(laneIndex);
                    const int queueSize = (int)queue.size();
                    double vehiclePosition = segmentOffset + length;
                    // draw vehicles beginning with the leader at the end of the segment
                    double xOff = 0;
                    for (int i = 0; i < queueSize; ++i) {
                        GUIMEVehicle* veh = static_cast<GUIMEVehicle*>(queue[queueSize - i - 1]);
                        const double vehLength = veh->getVehicleType().getLengthWithGap();
                        while (vehiclePosition < segmentOffset) {
                            // if there is only a single queue for a
                            // multi-lane edge shift vehicles and start
                            // drawing again from the end of the segment
                            vehiclePosition += length;
                            xOff += 2;
                        }
                        const Position p = l->geometryPositionAtOffset(vehiclePosition);
                        const double angle = l->getShape().rotationAtOffset(l->interpolateLanePosToGeometryPos(vehiclePosition));
                        veh->drawOnPos(s, p, angle);
                        vehiclePosition -= vehLength;
                    }
                }
                segmentOffset += length;
            }
            glPopMatrix();
        }
        vehicleControl->releaseVehicles();
    }
}



int
GUIEdge::getVehicleNo() const {
    int vehNo = 0;
    for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != 0; segment = segment->getNextSegment()) {
        vehNo += segment->getCarNumber();
    }
    return (int)vehNo;
}


std::string
GUIEdge::getVehicleIDs() const {
    std::string result = " ";
    std::vector<const MEVehicle*> vehs;
    for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != 0; segment = segment->getNextSegment()) {
        std::vector<const MEVehicle*> segmentVehs = segment->getVehicles();
        vehs.insert(vehs.end(), segmentVehs.begin(), segmentVehs.end());
    }
    for (std::vector<const MEVehicle*>::const_iterator it = vehs.begin(); it != vehs.end(); it++) {
        result += (*it)->getID() + " ";
    }
    return result;
}


double
GUIEdge::getFlow() const {
    double flow = 0;
    for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != 0; segment = segment->getNextSegment()) {
        flow += (double) segment->getCarNumber() * segment->getMeanSpeed();
    }
    return 3600 * flow / (*myLanes)[0]->getLength();
}


double
GUIEdge::getBruttoOccupancy() const {
    double occ = 0;
    for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != 0; segment = segment->getNextSegment()) {
        occ += segment->getBruttoOccupancy();
    }
    return occ / (*myLanes)[0]->getLength() / (double)(myLanes->size());
}


double
GUIEdge::getAllowedSpeed() const {
    return (*myLanes)[0]->getSpeedLimit();
}


double
GUIEdge::getRelativeSpeed() const {
    return getMeanSpeed() / getAllowedSpeed();
}


void
GUIEdge::setColor(const GUIVisualizationSettings& s) const {
    myMesoColor = RGBColor(0, 0, 0); // default background color when using multiColor
    const GUIColorer& c = s.edgeColorer;
    if (!setFunctionalColor(c.getActive()) && !setMultiColor(c)) {
        myMesoColor = c.getScheme().getColor(getColorValue(c.getActive()));
    }
}


bool
GUIEdge::setFunctionalColor(int activeScheme) const {
    switch (activeScheme) {
        case 9: {
            const PositionVector& shape = getLanes()[0]->getShape();
            double hue = GeomHelper::naviDegree(shape.beginEndAngle()); // [0-360]
            myMesoColor = RGBColor::fromHSV(hue, 1., 1.);
            return true;
        }
        default:
            return false;
    }
}


bool
GUIEdge::setMultiColor(const GUIColorer& c) const {
    const int activeScheme = c.getActive();
    mySegmentColors.clear();
    switch (activeScheme) {
        case 10: // alternating segments
            for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this);
                    segment != 0; segment = segment->getNextSegment()) {
                mySegmentColors.push_back(c.getScheme().getColor(segment->getIndex() % 2));
            }
            //std::cout << getID() << " scheme=" << c.getScheme().getName() << " schemeCols=" << c.getScheme().getColors().size() << " thresh=" << toString(c.getScheme().getThresholds()) << " segmentColors=" << mySegmentColors.size() << " [0]=" << mySegmentColors[0] << " [1]=" << mySegmentColors[1] <<  "\n";
            return true;
        case 11: // by segment jammed state
            for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this);
                    segment != 0; segment = segment->getNextSegment()) {
                mySegmentColors.push_back(c.getScheme().getColor(segment->free() ? 0 : 1));
            }
            return true;
        case 12: // by segment occupancy
            for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this);
                    segment != 0; segment = segment->getNextSegment()) {
                mySegmentColors.push_back(c.getScheme().getColor(segment->getRelativeOccupancy()));
            }
            return true;
        case 13: // by segment speed
            for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this);
                    segment != 0; segment = segment->getNextSegment()) {
                mySegmentColors.push_back(c.getScheme().getColor(segment->getMeanSpeed()));
            }
            return true;
        case 14: // by segment flow
            for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this);
                    segment != 0; segment = segment->getNextSegment()) {
                mySegmentColors.push_back(c.getScheme().getColor(3600 * segment->getCarNumber() * segment->getMeanSpeed() / segment->getLength()));
            }
            return true;
        case 15: // by segment relative speed
            for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this);
                    segment != 0; segment = segment->getNextSegment()) {
                mySegmentColors.push_back(c.getScheme().getColor(segment->getMeanSpeed() / getAllowedSpeed()));
            }
            return true;
        default:
            return false;
    }
}


double
GUIEdge::getColorValue(int activeScheme) const {
    switch (activeScheme) {
        case 1:
            return gSelected.isSelected(getType(), getGlID());
        case 2:
            return getFunction();
        case 3:
            return getAllowedSpeed();
        case 4:
            return getBruttoOccupancy();
        case 5:
            return getMeanSpeed();
        case 6:
            return getFlow();
        case 7:
            return getRelativeSpeed();
        case 8:
            return getRoutingSpeed();
        case 16:
            return MSNet::getInstance()->getInsertionControl().getPendingEmits(getLanes()[0]);
    }
    return 0;
}


double
GUIEdge::getScaleValue(int activeScheme) const {
    switch (activeScheme) {
        case 1:
            return gSelected.isSelected(getType(), getGlID());
        case 2:
            return getAllowedSpeed();
        case 3:
            return getBruttoOccupancy();
        case 4:
            return getMeanSpeed();
        case 5:
            return getFlow();
        case 6:
            return getRelativeSpeed();
        case 7:
            return MSNet::getInstance()->getInsertionControl().getPendingEmits(getLanes()[0]);
    }
    return 0;
}


MESegment*
GUIEdge::getSegmentAtPosition(const Position& pos) {
    const PositionVector& shape = getLanes()[0]->getShape();
    const double lanePos = shape.nearest_offset_to_point2D(pos);
    return MSGlobals::gMesoNet->getSegmentForEdge(*this, lanePos);
}



void
GUIEdge::closeTraffic(const GUILane* lane) {
    const std::vector<MSLane*>& lanes = getLanes();
    const bool isClosed = lane->isClosed();
    for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
        GUILane* l = dynamic_cast<GUILane*>(*i);
        if (l->isClosed() == isClosed) {
            l->closeTraffic(false);
        }
    }
    rebuildAllowedLanes();
}


void
GUIEdge::addRerouter() {
    MSEdgeVector edges;
    edges.push_back(this);
    GUITriggeredRerouter* rr = new GUITriggeredRerouter(getID() + "_dynamic_rerouter", edges, 1, "", false,
            GUINet::getGUIInstance()->getVisualisationSpeedUp());

    MSTriggeredRerouter::RerouteInterval ri;
    ri.begin = MSNet::getInstance()->getCurrentTimeStep();
    ri.end = SUMOTime_MAX;
    ri.edgeProbs.add(&MSTriggeredRerouter::mySpecialDest_keepDestination, 1.);
    rr->myIntervals.push_back(ri);

    // trigger rerouting for vehicles already on this edge
    const std::vector<MSLane*>& lanes = getLanes();
    for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
        const MSLane::VehCont& vehicles = (*i)->getVehiclesSecure();
        for (MSLane::VehCont::const_iterator v = vehicles.begin(); v != vehicles.end(); ++v) {
            if ((*v)->getLane() == (*i)) {
                rr->notifyEnter(**v, MSMoveReminder::NOTIFICATION_JUNCTION);
            } // else: this is the shadow during a continuous lane change
        }
        (*i)->releaseVehicles();
    }
}

/****************************************************************************/

