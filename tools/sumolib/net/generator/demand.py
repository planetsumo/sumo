#!/usr/bin/env python
"""
A module for building a demand.

@file    demand.py
@author  Daniel Krajzewicz
@date    2013-10-10
@version $Id: demand.py 14731 2013-09-20 18:56:22Z behrisch $

SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
Copyright (C) 2013 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
import random


class Vehicle:
  def __init__(self, id, depart, route):
    self.id = id
    self.depart = depart
    self.route = route

class Stream:
  def __init__(self, id, flow, route):
    self.id = id
    self.flow = flow
    self.route = route
  def toVehicles(self, t):
    vehicles = []
    numVehicles = t*self.flow/3600
    for j in range(0, numVehicles):
      vehicles.append(Vehicle(self.id+"#"+str(j), int(random.random()*t), self.route))
    return vehicles

class Demand:
  def __init__(self):
    self.streams = []

  def addStream(self, s):
    self.streams.append(s)
  
  def build(self, t):
    vehicles = []
    for s in self.streams:
      vehicles.extend(s.toVehicles(t))
    fdo = open("input_routes.rou.xml", "w")
    fdo.write("<routes>\n")
    for v in sorted(vehicles, key=lambda veh: veh.depart):
      fdo.write('    <vehicle id="%s" depart="%s" departSpeed="max"><route edges="%s"/></vehicle>\n' % (v.id, v.depart, v.route))  
    fdo.write("</routes>")
    fdo.close()
    