#!/usr/bin/env python
"""
A module for building road networks.

@file    network.py
@author  Daniel Krajzewicz
@date    2013-10-10
@version $Id: network.py 14731 2013-09-20 18:56:22Z behrisch $

SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
Copyright (C) 2013 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
import sumolib          
import os
import subprocess



class Node:
  def __init__(self, id, x, y, nodeType):
    self.id = id
    self.x = x
    self.y = y
    self.nodeType = nodeType

class Edge:
  def __init__(self, id, fromNode, toNode, numLanes, maxSpeed):
    self.id = id
    self.fromNode = fromNode
    self.toNode = toNode
    self.numLanes = numLanes
    self.maxSpeed = maxSpeed
    self.splits = []
  def addSplit(self, s):
    self.split.append(s)

class Connection:
  def __init__(self, fromEdge, fromLane, toEdge, toLane):
    self.fromEdge = fromEdge    
    self.fromLane = fromLane    
    self.toEdge = toEdge    
    self.toLane = toLane    
  

class Net:
  def __init__(self):
    self._nodes = {}
    self._edges = {}
    self._connections = []
    self._defaultEdge = Edge(None, None, None, 2, 13.89)

  def addNode(self, n):
    self._nodes[n.id] = n

  def getNode(self, id):
    if id in self._nodes:
      return self._nodes[id]
    return None 

  def addEdge(self, e):
    self._edges[e.id] = e

  def getEdge(self, id):
    if id in self._edges:
      return self._edges[id]
    return None 
    
  def buildEdge(self, id, node1, node2):
    numLanes = getValue(id, "numLanes", self._defaultEdge.numLanes)
    maxSpeed = getValue(id, "maxSpeed", self._defaultEdge.maxSpeed)    
    
    
  def connectNodes(self, node1, node2, bidi):
    self.addEdge(Edge(node1+"_to_"+node2, self._nodes[node1], self._nodes[node2], 2, 13.89))
    if bidi:
      self.addEdge(Edge(node2+"_to_"+node1, self._nodes[node2], self._nodes[node1], 2, 13.89))
    
  def hack_addConnection(self, fromEdge, tEdge, tEdgeLanes):
    for l in tEdgeLanes:
      self._connections.append(Connection(fromEdge, l[0], tEdge, l[1]))

  def hack_addConnections(self, fromEdge, rEdge, sEdge, lEdge, rEdgeLanes, sEdgeLanes, lEdgeLanes):
    self.hack_addConnection(fromEdge, rEdge, rEdgeLanes)
    self.hack_addConnection(fromEdge, sEdge, sEdgeLanes)
    self.hack_addConnection(fromEdge, lEdge, lEdgeLanes)
        

  def build(self):
    nodesFile = "nodes.nod.xml"
    fdo = open(nodesFile, "w")
    print >> fdo, "<nodes>"
    for nid in self._nodes:
      n = self._nodes[nid]
      print >> fdo, '    <node id="%s" x="%s" y="%s" type="%s"/>' % (n.id, n.x, n.y, n.nodeType)
    print >> fdo, "</nodes>"
    fdo.close()
    
    edgesFile = "edges.edg.xml"
    fdo = open(edgesFile, "w")
    print >> fdo, "<edges>"
    for eid in self._edges:
      e = self._edges[eid]
      print >> fdo, '    <edge id="%s" from="%s" to="%s" numLanes="%s" speed="%s">' % (e.id, e.fromNode.id, e.toNode.id, e.numLanes, e.maxSpeed)
      for s in e.splits:
        print >> fdo, '        <split pos="%s" lanes="%s">' % (s.pos, s.lanes)
      print >> fdo, '    </edge>'
    print >> fdo, "</edges>"
    fdo.close()
    
    connectionsFile = "connections.con.xml"
    fdo = open(connectionsFile, "w")
    print >> fdo, "<connections>"
    for c in self._connections:
      print >> fdo, '    <connection from="%s" to="%s" fromLane="%s" toLane="%s"/>' % (c.fromEdge.id, c.toEdge.id, c.fromLane, c.toLane)
    print >> fdo, "</connections>"
    fdo.close()
    
    netconvert = sumolib.checkBinary("netconvert")
    
    retCode = subprocess.call(" ".join([netconvert, "-v -n %s -e %s -x %s -o net.net.xml" % (nodesFile, edgesFile, connectionsFile)]))
    os.remove(nodesFile)
    os.remove(edgesFile)
    os.remove(connectionsFile)
    


    