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
  def __init__(self, nid, x, y, nodeType):
    self.nid = nid
    self.x = x
    self.y = y
    self.nodeType = nodeType
  def getNetworkCoordinates(self):
    t = self.nid.split("/")
    return [int(t[0]), int(t[1])]

class Lane:
  def __init__(self, dirs=None):
    self.dirs = dirs
    if self.dirs==None: 
      self.dirs = []

class Edge:
  def __init__(self, eid=None, fromNode=None, toNode=None, numLanes=None, maxSpeed=None, lanes=None, splits=None):
    self.eid = eid
    self.fromNode = fromNode
    self.toNode = toNode
    self.numLanes = numLanes
    self.maxSpeed = maxSpeed
    self.lanes = lanes
    if self.lanes==None: self.lanes = []
    self.splits = splits
    if self.splits==None: self.splits = []
  def addSplit(self, s):
    self.split.append(s)
  def getConnections(self, net):
    ret = []
    for i,l in enumerate(self.lanes):
      for d in l.dirs:
        c = net.dir2connection(d, self, i, i, i)
        if c!=None: ret.append(c)
    return ret
  def getDirection(self):
    n1c = self.fromNode.getNetworkCoordinates()
    n2c = self.toNode.getNetworkCoordinates()
    return [n2c[0]-n1c[0], n2c[1]-n1c[1]]

class Connection:
  def __init__(self, fromEdge, fromLane, toEdge, toLane):
    self.fromEdge = fromEdge    
    self.fromLane = fromLane    
    self.toEdge = toEdge    
    self.toLane = toLane    
  

class Net:
  def __init__(self, defaultNode, defaultEdge):
    self._nodes = {}
    self._edges = {}
    self._defaultEdge = defaultEdge
    if self._defaultEdge==None: self._defaultEdge = Edge(None, None, None, 2, 13.89)
    self._defaultNode = defaultNode
    if self._defaultNode==None: self._defaultNode = Node(None, None, None, "traffic_light")

  def addNode(self, n):
    self._nodes[n.nid] = n

  def getNode(self, id):
    if id in self._nodes:
      return self._nodes[id]
    return None 

  def addEdge(self, e):
    self._edges[e.eid] = e

  def getEdge(self, id):
    if id in self._edges:
      return self._edges[id]
    return None 
    
  def getDefaultEdge(self, n1, n2):
    return self._defaultEdge
    
  def buildEdge(self, n1, n2):
    numLanes = self.getDefaultEdge(n1, n2).numLanes
    maxSpeed = self.getDefaultEdge(n1, n2).maxSpeed 
    e = Edge(n1.nid+"_to_"+n2.nid, n1, n2, numLanes, maxSpeed)
    for s in self.getDefaultEdge(n1, n2).splits:
        e.splits.append(s)
    for l in self.getDefaultEdge(n1, n2).lanes:
        e.lanes.append(Lane(l.dirs))
    return e   

  def connectNodes(self, node1, node2, bidi):
    n1 = self._nodes[node1]
    n2 = self._nodes[node2] 
    self.addEdge(self.buildEdge(n1, n2))
    if bidi:
      self.addEdge(self.buildEdge(n2, n1))

  def getDirectionFromNode(self, n, dir):
    nc = n.getNetworkCoordinates()
    eid = n.nid + "_to_" + str(nc[0]+dir[0]) + "/" + str(nc[1]+dir[1])
    #print "%s %s %s" % (n.nid, dir, eid)
    if eid in self._edges:
      return self._edges[eid]
    return None

  def getMatchingOutgoing(self, edge, direction):
    edir = edge.getDirection()
    if direction=="s":
      return self.getDirectionFromNode(edge.toNode, edir)
    elif direction=="t":
      return self.getDirectionFromNode(edge.toNode, [-1*edir[0], -1*edir[1]])
    elif direction=="r":
      # look, the following is because SUMO's coordinates don't match:
      #  the y-direction starts at the bottom, while x on right 
      if edir[0]==0:
        return self.getDirectionFromNode(edge.toNode, [1*edir[1], 1*edir[0]])
      else:
        return self.getDirectionFromNode(edge.toNode, [-1*edir[1], -1*edir[0]])
    elif direction=="l":
      # the same as above
      if edir[0]!=0:
        return self.getDirectionFromNode(edge.toNode, [1*edir[1], 1*edir[0]])
      else:
        return self.getDirectionFromNode(edge.toNode, [-1*edir[1], -1*edir[0]])
    else:
      raise "Unrecognized direction '%s'" % direction
    
  def dir2connection(self, direction, edge, lane, seenRight, leftLeft):
    toEdge = self.getMatchingOutgoing(edge, direction)
    if toEdge!=None:    
      return Connection(edge, lane, toEdge, seenRight)
    return None


  def build(self, netName="net.net.xml"):
    connections = []
    nodesFile = "nodes.nod.xml"
    fdo = open(nodesFile, "w")
    print >> fdo, "<nodes>"
    for nid in self._nodes:
      n = self._nodes[nid]
      print >> fdo, '    <node id="%s" x="%s" y="%s" type="%s"/>' % (n.nid, n.x, n.y, n.nodeType)
    print >> fdo, "</nodes>"
    fdo.close()
    
    edgesFile = "edges.edg.xml"
    fdo = open(edgesFile, "w")
    print >> fdo, "<edges>"
    for eid in self._edges:
      e = self._edges[eid]
      print >> fdo, '    <edge id="%s" from="%s" to="%s" numLanes="%s" speed="%s">' % (e.eid, e.fromNode.nid, e.toNode.nid, e.numLanes, e.maxSpeed)
      for s in e.splits:
        print >> fdo, '        <split pos="%s" lanes="%s">' % (s.pos, s.lanes)
      connections.extend(e.getConnections(self))
      print >> fdo, '    </edge>'
    print >> fdo, "</edges>"
    fdo.close()
    
    connectionsFile = "connections.con.xml"
    fdo = open(connectionsFile, "w")
    print >> fdo, "<connections>"
    for c in connections:
      print >> fdo, '    <connection from="%s" to="%s" fromLane="%s" toLane="%s"/>' % (c.fromEdge.eid, c.toEdge.eid, c.fromLane, c.toLane)
    print >> fdo, "</connections>"
    fdo.close()
    
    netconvert = sumolib.checkBinary("netconvert")
    
    retCode = subprocess.call(" ".join([netconvert, "-v -n %s -e %s -x %s -o %s" % (nodesFile, edgesFile, connectionsFile, netName)]))
    os.remove(nodesFile)
    os.remove(edgesFile)
    os.remove(connectionsFile)
    


    