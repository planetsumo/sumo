#!/usr/bin/env python
"""
A script for building corridor networks.

@file    corridor.py
@author  Daniel Krajzewicz
@date    2013-10-10
@version $Id: corridor.py 14731 2013-09-20 18:56:22Z behrisch $

SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
Copyright (C) 2013 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
import network
import demand

def corridor():
  numIntersections = 10
  net = network.Net()
  net.addNode(network.Node("0/1", 0, 500, "priority"))
  for i in range(0, numIntersections):
    net.addNode(network.Node(str(i+1)+"/0", (i+1)*500, 0, "priority"))
    net.addNode(network.Node(str(i+1)+"/1", (i+1)*500, 500, "traffic_light"))
    net.addNode(network.Node(str(i+1)+"/2", (i+1)*500, 1000, "priority"))
    net.connectNodes(str(i)+"/1", str(i+1)+"/1", True)
    net.connectNodes(str(i+1)+"/0", str(i+1)+"/1", True)
    net.connectNodes(str(i+1)+"/1", str(i+1)+"/2", True)
  #net.addNode(network.Node(str(numIntersections)+"/1", (numIntersections)*500, 0, "priority"))
  net.addNode(network.Node(str(numIntersections+1)+"/1", (numIntersections+1)*500, 500, "priority"))
  net.connectNodes(str(numIntersections)+"/1", str(numIntersections+1)+"/1", True)
  net.build()
  
  d = demand.Demand()
  d.addStream(demand.Stream("1/0_to_1/2", 10, "1/0 1/2"))
  d.build(3600)

if __name__ == "__main__":
    corridor()         
