/****************************************************************************/
/// @file    NBAlgorithms.cpp
/// @author  Daniel Krajzewicz
/// @date    02. March 2012
/// @version $Id$
///
// Algorithms for network computation
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
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

#include <sstream>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <fstream>
#include <utils/common/MsgHandler.h>
#include <utils/geom/GeomHelper.h> 
#include <utils/options/OptionsCont.h> 
#include "NBEdge.h"
#include "NBNodeCont.h"
#include "NBTypeCont.h"
#include "NBNode.h"
#include "NBAlgorithms.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// NBTurningDirectionsComputer
// ---------------------------------------------------------------------------
void
NBTurningDirectionsComputer::computeTurnDirections(NBNodeCont& nc) {
    for (std::map<std::string, NBNode*>::const_iterator i = nc.begin(); i != nc.end(); ++i) {
        computeTurnDirectionsForNode(i->second);
    }
}

void
NBTurningDirectionsComputer::computeTurnDirectionsForNode(NBNode* node) {
    const std::vector<NBEdge*>& incoming = node->getIncomingEdges();
    const std::vector<NBEdge*>& outgoing = node->getOutgoingEdges();
    std::vector<Combination> combinations;
    for (std::vector<NBEdge*>::const_iterator j = outgoing.begin(); j != outgoing.end(); ++j) {
        NBEdge* outedge = *j;
        for (std::vector<NBEdge*>::const_iterator k = incoming.begin(); k != incoming.end(); ++k) {
            NBEdge* e = *k;
            if (e->getConnections().size() != 0 && !e->isConnectedTo(outedge)) {
                // has connections, but not to outedge; outedge will not be the turn direction
                //
                // @todo: this seems to be needed due to legacy issues; actually, we could regard
                //  such pairs, too, and it probably would increase the accuracy. But there is
                //  no mechanism implemented, yet, which would avoid adding them as turnarounds though
                //  no connection is specified.
                continue;
            }

            // @todo: check whether NBHelpers::relAngle is properly defined and whether it should really be used, here
            SUMOReal angle = fabs(NBHelpers::relAngle(e->getAngleAtNode(node), outedge->getAngleAtNode(node)));
            if (angle < 160) {
                continue;
            }
            if (e->getFromNode() == outedge->getToNode()) {
                // they connect the same nodes; should be the turnaround direction
                // we'll assign a maximum number
                //
                // @todo: indeed, we have observed some pathological intersections
                //  see "294831560" in OSM/adlershof. Here, several edges are connecting
                //  same nodes. We have to do the angle check before...
                //
                // @todo: and well, there are some other as well, see plain import
                //  of delphi_muenchen (elmar), intersection "59534191". Not that it would
                //  be realistic in any means; we will warn, here.
                angle += 360;
            }
            Combination c;
            c.from = e;
            c.to = outedge;
            c.angle = angle;
            combinations.push_back(c);
        }
    }
    // sort combinations so that the ones with the highest angle are at the begin
    std::sort(combinations.begin(), combinations.end(), combination_by_angle_sorter());
    std::set<NBEdge*> seen;
    bool haveWarned = false;
    for (std::vector<Combination>::const_iterator j = combinations.begin(); j != combinations.end(); ++j) {
        if (seen.find((*j).from) != seen.end() || seen.find((*j).to) != seen.end()) {
            // do not regard already set edges
            if ((*j).angle > 360 && !haveWarned) {
                WRITE_WARNING("Ambiguity in turnarounds computation at node '" + node->getID() + "'.");
                haveWarned = true;
            }
            continue;
        }
        // mark as seen
        seen.insert((*j).from);
        seen.insert((*j).to);
        // set turnaround information
        (*j).from->setTurningDestination((*j).to);
    }
}


// ---------------------------------------------------------------------------
// NBNodesEdgesSorter
// ---------------------------------------------------------------------------
void
NBNodesEdgesSorter::sortNodesEdges(NBNodeCont& nc, bool leftHand) {
    for (std::map<std::string, NBNode*>::const_iterator i = nc.begin(); i != nc.end(); ++i) {
        NBNode* n = (*i).second;
        if (n->myAllEdges.size() == 0) {
            continue;
        }
        std::vector<NBEdge*>& allEdges = (*i).second->myAllEdges;
        std::vector<NBEdge*>& incoming = (*i).second->myIncomingEdges;
        std::vector<NBEdge*>& outgoing = (*i).second->myOutgoingEdges;
        // sort the edges
        std::sort(allEdges.begin(), allEdges.end(), edge_by_junction_angle_sorter(n));
        std::sort(incoming.begin(), incoming.end(), edge_by_junction_angle_sorter(n));
        std::sort(outgoing.begin(), outgoing.end(), edge_by_junction_angle_sorter(n));
        std::vector<NBEdge*>::iterator j;
        for (j = allEdges.begin(); j != allEdges.end() - 1 && j != allEdges.end(); ++j) {
            swapWhenReversed(n, leftHand, j, j + 1);
        }
        if (allEdges.size() > 1 && j != allEdges.end()) {
            swapWhenReversed(n, leftHand, allEdges.end() - 1, allEdges.begin());
        }
    }
}


void
NBNodesEdgesSorter::swapWhenReversed(const NBNode* const n, bool leftHand,
                                     const std::vector<NBEdge*>::iterator& i1,
                                     const std::vector<NBEdge*>::iterator& i2) {
    NBEdge* e1 = *i1;
    NBEdge* e2 = *i2;
    if (leftHand) {
        // @todo: check this; shouldn't it be "swap(*e1, *e2)"?
        std::swap(e1, e2);
    }
    // @todo: The difference between "isTurningDirectionAt" and "isTurnaround"
    //  is not nice. Maybe we could get rid of it if we would always mark edges
    //  as turnarounds, even if they do not have to be added, as mentioned in
    //  notes on NBTurningDirectionsComputer::computeTurnDirectionsForNode
    if (e2->getToNode() == n && e2->isTurningDirectionAt(n, e1)) {
        std::swap(*i1, *i2);
    }
}


// ---------------------------------------------------------------------------
// NBNodeTypeComputer
// ---------------------------------------------------------------------------
void
NBNodeTypeComputer::computeNodeTypes(NBNodeCont& nc) {
    for (std::map<std::string, NBNode*>::const_iterator i = nc.begin(); i != nc.end(); ++i) {
        NBNode* n = (*i).second;
        // the type may already be set from the data
        if (n->myType != NODETYPE_UNKNOWN) {
            continue;
        }
        // check whether the junction is not a real junction
        if (n->myIncomingEdges.size() == 1) {
            n->myType = NODETYPE_PRIORITY_JUNCTION;
            continue;
        }
        // @todo "isSimpleContinuation" should be revalidated
        if (n->isSimpleContinuation()) {
            n->myType = NODETYPE_PRIORITY_JUNCTION;
            continue;
        }
        // determine the type
        SumoXMLNodeType type = NODETYPE_RIGHT_BEFORE_LEFT;
        for (EdgeVector::const_iterator i = n->myIncomingEdges.begin(); i != n->myIncomingEdges.end(); i++) {
            for (EdgeVector::const_iterator j = i + 1; j != n->myIncomingEdges.end(); j++) {
                // @todo "getOppositeIncoming" should probably be refactored into something the edge knows
                if (n->getOppositeIncoming(*j) == *i && n->myIncomingEdges.size() > 2) {
                    continue;
                }
                // @todo check against a legal document
                const SUMOReal s1 = (*i)->getSpeed() * (SUMOReal) 3.6;
                const SUMOReal s2 = (*j)->getSpeed() * (SUMOReal) 3.6;
                const int p1 = (*i)->getPriority();
                const int p2 = (*j)->getPriority();
                if (fabs(s1 - s2) > (SUMOReal) 9.5 || MAX2(s1, s2) >= (SUMOReal) 49. || p1 != p2) {
                    type = NODETYPE_PRIORITY_JUNCTION;
                    break;
                }
            }
        }
        // save type
        n->myType = type;
    }
}


// ---------------------------------------------------------------------------
// NBEdgePriorityComputer
// ---------------------------------------------------------------------------
void
NBEdgePriorityComputer::computeEdgePriorities(NBNodeCont& nc) {
    for (std::map<std::string, NBNode*>::const_iterator i = nc.begin(); i != nc.end(); ++i) {
        NBNode* n = (*i).second;
        // preset all junction's edge priorities to zero
        for (EdgeVector::iterator j = n->myAllEdges.begin(); j != n->myAllEdges.end(); ++j) {
            (*j)->setJunctionPriority(n, 0);
        }
        // check if the junction is not a real junction
        if (n->myIncomingEdges.size() == 1 && n->myOutgoingEdges.size() == 1) {
            continue;
        }
        // compute the priorities on junction when needed
        if (n->myType != NODETYPE_RIGHT_BEFORE_LEFT) {
            setPriorityJunctionPriorities(*n);
        }
    }
}


void
NBEdgePriorityComputer::setPriorityJunctionPriorities(NBNode& n) {
    if (n.myIncomingEdges.size() == 0 || n.myOutgoingEdges.size() == 0) {
        return;
    }
    EdgeVector incoming = n.myIncomingEdges;
    EdgeVector outgoing = n.myOutgoingEdges;
    // what we do want to have is to extract the pair of roads that are
    //  the major roads for this junction
    // let's get the list of incoming edges with the highest priority
    std::sort(incoming.begin(), incoming.end(), NBContHelper::edge_by_priority_sorter());
    EdgeVector bestIncoming;
    NBEdge* best = incoming[0];
    while (incoming.size() > 0 && samePriority(best, incoming[0])) {
        bestIncoming.push_back(*incoming.begin());
        incoming.erase(incoming.begin());
    }
    // now, let's get the list of best outgoing
    assert(outgoing.size() != 0);
    sort(outgoing.begin(), outgoing.end(), NBContHelper::edge_by_priority_sorter());
    EdgeVector bestOutgoing;
    best = outgoing[0];
    while (outgoing.size() > 0 && samePriority(best, outgoing[0])) { //->getPriority()==best->getPriority()) {
        bestOutgoing.push_back(*outgoing.begin());
        outgoing.erase(outgoing.begin());
    }
    // now, let's compute for each of the best incoming edges
    //  the incoming which is most opposite
    //  the outgoing which is most opposite
    EdgeVector::iterator i;
    std::map<NBEdge*, NBEdge*> counterIncomingEdges;
    std::map<NBEdge*, NBEdge*> counterOutgoingEdges;
    incoming = n.myIncomingEdges;
    outgoing = n.myOutgoingEdges;
    for (i = bestIncoming.begin(); i != bestIncoming.end(); ++i) {
        std::sort(incoming.begin(), incoming.end(), NBContHelper::edge_opposite_direction_sorter(*i, &n));
        counterIncomingEdges[*i] = *incoming.begin();
        std::sort(outgoing.begin(), outgoing.end(), NBContHelper::edge_opposite_direction_sorter(*i, &n));
        counterOutgoingEdges[*i] = *outgoing.begin();
    }
    // ok, let's try
    // 1) there is one best incoming road
    if (bestIncoming.size() == 1) {
        // let's mark this road as the best
        NBEdge* best1 = extractAndMarkFirst(n, bestIncoming);
        if (counterIncomingEdges.find(best1) != counterIncomingEdges.end()) {
            // ok, look, what we want is the opposit of the straight continuation edge
            // but, what if such an edge does not exist? By now, we'll determine it
            // geometrically
            NBEdge* s = counterIncomingEdges.find(best1)->second;
            if (GeomHelper::getMinAngleDiff(best1->getAngleAtNode(&n), s->getAngleAtNode(&n)) > 180 - 45) {
                s->setJunctionPriority(&n, 1);
            }
        }
        if (bestOutgoing.size() != 0) {
            // mark the best outgoing as the continuation
            sort(bestOutgoing.begin(), bestOutgoing.end(), NBContHelper::edge_similar_direction_sorter(best1));
            best1 = extractAndMarkFirst(n, bestOutgoing);
            if (counterOutgoingEdges.find(best1) != counterOutgoingEdges.end()) {
                NBEdge* s = counterOutgoingEdges.find(best1)->second;
                if (GeomHelper::getMinAngleDiff(best1->getAngleAtNode(&n), s->getAngleAtNode(&n)) > 180 - 45) {
                    s->setJunctionPriority(&n, 1);
                }
            }
        }
        return;
    }

    // ok, what we want to do in this case is to determine which incoming
    //  has the best continuation...
    // This means, when several incoming roads have the same priority,
    //  we want a (any) straight connection to be more priorised than a turning
    SUMOReal bestAngle = 0;
    NBEdge* bestFirst = 0;
    NBEdge* bestSecond = 0;
    bool hadBest = false;
    for (i = bestIncoming.begin(); i != bestIncoming.end(); ++i) {
        EdgeVector::iterator j;
        NBEdge* t1 = *i;
        SUMOReal angle1 = t1->getAngle() + 180;
        if (angle1 >= 360) {
            angle1 -= 360;
        }
        for (j = i + 1; j != bestIncoming.end(); ++j) {
            NBEdge* t2 = *j;
            SUMOReal angle2 = t2->getAngle() + 180;
            if (angle2 >= 360) {
                angle2 -= 360;
            }
            SUMOReal angle = GeomHelper::getMinAngleDiff(angle1, angle2);
            if (!hadBest || angle > bestAngle) {
                bestAngle = angle;
                bestFirst = *i;
                bestSecond = *j;
                hadBest = true;
            }
        }
    }
    bestFirst->setJunctionPriority(&n, 1);
    sort(bestOutgoing.begin(), bestOutgoing.end(), NBContHelper::edge_similar_direction_sorter(bestFirst));
    if (bestOutgoing.size() != 0) {
        extractAndMarkFirst(n, bestOutgoing);
    }
    bestSecond->setJunctionPriority(&n, 1);
    sort(bestOutgoing.begin(), bestOutgoing.end(), NBContHelper::edge_similar_direction_sorter(bestSecond));
    if (bestOutgoing.size() != 0) {
        extractAndMarkFirst(n, bestOutgoing);
    }
}


NBEdge*
NBEdgePriorityComputer::extractAndMarkFirst(NBNode& n, std::vector<NBEdge*>& s) {
    if (s.size() == 0) {
        return 0;
    }
    NBEdge* ret = s.front();
    s.erase(s.begin());
    ret->setJunctionPriority(&n, 1);
    return ret;
}


bool
NBEdgePriorityComputer::samePriority(const NBEdge* const e1, const NBEdge* const e2) {
    if (e1 == e2) {
        return true;
    }
    if (e1->getPriority() != e2->getPriority()) {
        return false;
    }
    if ((int) e1->getSpeed() != (int) e2->getSpeed()) {
        return false;
    }
    return (int) e1->getNumLanes() == (int) e2->getNumLanes();
}

 
// --------------------------------------------------------------------------- 
// NBEdgePriorityComputer 
// --------------------------------------------------------------------------- 

int 
ct(std::map<NBNode*, int> &typed, NBNode *n) {
    if(typed.find(n)==typed.end()) {
        typed[n] = 0;
        return 0;
    }
    typed[n] = typed[n] + 1;
    return typed[n];
}

void 
NBNodeTopologyTypeComputer::computeTopologyType(NBNodeCont &nc, OptionsCont &oc) { 
    //SUMOReal minHighwaySpeed = oc.getFloat("ramps.min-highway-speed");
    //SUMOReal maxRampSpeed = oc.getFloat("ramps.max-ramp-speed");
    bool wantsRightAccelerators = oc.getBool("right-accel.guess");

    std::map<NBNode*, int> typed;
    std::ofstream strm("d:\\types.xml");
    strm << "<pois>" << std::endl;

    std::set<NBNode*> inMultiJoin;

    for(std::map<std::string, NBNode*>::const_iterator i=nc.begin(); i!=nc.end(); ++i) {
        NBNode *c = (*i).second;

        //std::cout << c->getID() << std::endl;
        if(c->getID()=="54525266") {
            int bla = 0;
        }

        const std::vector<NBEdge*> &incoming = c->getIncomingEdges();
        const std::vector<NBEdge*> &outgoing = c->getOutgoingEdges();
        const std::vector<NBEdge*> &all = c->getEdges();
        // source (no incoming)
        if(incoming.size()==0) {
            c->setTopologyType(NBNode::NTT_SOURCE);
            strm << "   <poi id=\"" << c->getID() << '_'  << ct(typed, c) << "\" type=\"source\" x=\"" << c->getPosition().x() << "\" y=\"" << c->getPosition().y() << "\" color=\".5,.5,.5\" layer=\"10\"/>" << std::endl;
        }
        // dead end (no outgoing)
        if(outgoing.size()==0) {
            c->setTopologyType(NBNode::NTT_DEAD_END);
            strm << "   <poi id=\"" << c->getID() << '_'  << ct(typed, c) << "\" type=\"dead_end\" x=\"" << c->getPosition().x() << "\" y=\"" << c->getPosition().y() << "\" color=\".5,.5,.5\" layer=\"10\"/>" << std::endl;
        }

        // acceleration
        if(outgoing.size()>1 && wantsRightAccelerators) {
            checkRightAccelerators(c);
        }

        // to join
        if(outgoing.size()>1&&incoming.size()>1&&inMultiJoin.find(c)==inMultiJoin.end()) {
            for(std::vector<NBEdge*>::const_iterator j=all.begin(); j!=all.end(); ++j) {
                if((*j)->getFromNode()!=c) {
                    continue;
                }
                if((*j)->getLength()>20) {
                    continue;
                }
                NBEdge *first = *j;
                NBEdge *currEdge = *j;
                NBNode *currNode = c;
                bool loopContinue = true;
                std::set<NBNode*> seen;
                while(true) {
                    seen.insert(currNode);
                    currNode = currEdge->getToNode();
                    const std::vector<NBEdge*> &ce = currNode->getEdges();
                    std::vector<NBEdge*>::const_iterator k = std::find(ce.begin(), ce.end(), currEdge);
                    NBContHelper::nextCW(ce, k);
                    if((*k)->getFromNode()!=currNode) {
                        break;
                    }
                    currEdge = *k;
                    if(currEdge==first) {
                        break;
                    }
                    if(currEdge->getLength()>20) {
                        break;
                    }
                }
                if(currEdge!=first || seen.size()<2) {
                    continue;
                }
                for(std::set<NBNode*>::const_iterator k=seen.begin(); k!=seen.end(); ++k) {
                    inMultiJoin.insert(*k);
                    (*k)->setTopologyType(NBNode::NTT_MULTI_TO_JOIN);
                    strm << "   <poi id=\"" << (*k)->getID() << '_'  << ct(typed, (*k)) << "\" type=\"NTT_MULTI_TO_JOIN\" x=\"" << (*k)->getPosition().x() << "\" y=\"" << (*k)->getPosition().y() << "\" color=\"1,1,.5\" layer=\"10\"/>" << std::endl;
                }
            }
        }

        /*
        // highway on-ramp
        if(outgoing.size()==1 && incoming.size()==2 /*&& fulfillsRampConstraints(potHighway, potRamp, cont, minHighwaySpeed, maxRampSpeed)/) {
            c->setTopologyType(NBNode::NTT_HIGHWAY_ON_RAMP);
            strm << "   <poi id=\"" << c->getID() << "\" type=\"highway_on_ramp\" x=\"" << c->getPosition().x() << "\" y=\"" << c->getPosition().y() << "\" color=\"\" layer=\"10\"/>" << std::endl;
            continue;
        }
        // highway off-ramp
        if(outgoing.size()==2 && incoming.size()==1 /*&& fulfillsRampConstraints(potHighway, potRamp, cont, minHighwaySpeed, maxRampSpeed)/) {
            c->setTopologyType(NBNode::NTT_HIGHWAY_OFF_RAMP);
            strm << "   <poi id=\"" << c->getID() << "\" type=\"highway_off_ramp\" x=\"" << c->getPosition().x() << "\" y=\"" << c->getPosition().y() << "\" color=\"\" layer=\"10\"/>" << std::endl;
            continue;
        }
        */
    }
    strm << "</pois>" << std::endl;
} 


void 
NBNodeTopologyTypeComputer::checkRightAccelerators(NBNode *c) {
    const std::vector<NBEdge*> &all = c->getEdges();
    for(std::vector<NBEdge*>::const_iterator j=all.begin(); j!=all.end(); ++j) {
        if((*j)->getFromNode()!=c) {
            continue;
        }
        NBEdge *out = *j;
        std::vector<NBEdge*>::const_iterator k = j;
        NBContHelper::nextCW(all, k);
        if((*k)->getToNode()!=c) {
            continue;
        }
        NBEdge *in = *k;
        if(in->getFromNode()->getConnectionTo(out->getToNode())==0 || out->getToNode()->getConnectionTo(in->getFromNode())!=0) {
            continue;
        }
        if(out->getLength()>50 || (*k)->getLength()>50) {
            continue;
        }
        NBEdge *connection = in->getFromNode()->getConnectionTo(out->getToNode());
        if(in->getFromNode()->getIncomingEdges().size()>3) {
            continue;
        }
        if(out->getToNode()->getOutgoingEdges().size()>3) {
            continue;
        }

        /*
        c->setTopologyType(NBNode::NTT_ACCELERATED);
        strm << "   <poi id=\"" << c->getID() << '_'  << ct(typed, c)<< "\" type=\"NTT_ACCELERATED\" x=\"" << c->getPosition().x() << "\" y=\"" << c->getPosition().y() << "\" color=\"1,1,0\" layer=\"10\"/>" << std::endl;
        in->getFromNode()->setTopologyType(NBNode::NTT_ACCEL_BEGIN);
        strm << "   <poi id=\"" << in->getFromNode()->getID() << '_'  << ct(typed, in->getFromNode()) << "\" type=\"NTT_ACCEL_BEGIN\" x=\"" << in->getFromNode()->getPosition().x() << "\" y=\"" << in->getFromNode()->getPosition().y() << "\" color=\"1,.5,0\" layer=\"10\"/>" << std::endl;
        c->setTopologyType(NBNode::NTT_ACCEL_END);
        strm << "   <poi id=\"" << out->getToNode()->getID() << '_'  << ct(typed, out->getToNode()) << "\" type=\"NTT_ACCEL_END\" x=\"" << out->getToNode()->getPosition().x() << "\" y=\"" << out->getToNode()->getPosition().y() << "\" color=\".5,1,0\" layer=\"10\"/>" << std::endl;
        */  

        // set connections
        in->setAsUnconnected(out);
        NBNode *d = connection->getToNode();
        const std::vector<NBEdge*> &connEdges = d->getEdges();
        SUMOReal maxSpeed = connEdges.front()->getSpeed();
        int minPrio = connEdges.front()->getPriority();
        for(k=connEdges.begin(); k!=connEdges.end(); ++k) {
            maxSpeed = MAX2(maxSpeed, (*k)->getSpeed());
            minPrio = MIN2(minPrio, (*k)->getPriority());
        }
        connection->setPriority(minPrio-1);
        if(maxSpeed<=50./3.6) {
            break;
        }
        k = std::find(connEdges.begin(), connEdges.end(), connection);
        bool first = true;
        do {
            NBContHelper::nextCCW(connEdges, k);
            if((*k)->getFromNode()==d) {
                if(!first) {
                    connection->setAsUnconnected(*k);
                }
                first = false;
            }
        } while(connection!=*k);

                /*
                */
    }
}

/****************************************************************************/

