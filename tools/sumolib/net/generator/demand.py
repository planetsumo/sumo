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
import sumolib          
import os
import subprocess



class Vehicle:
  def __init__(self, id, depart, fromEdge, toEdge, vType):
    self.id = id
    self.depart = depart
    self.fromEdge = fromEdge
    self.toEdge = toEdge
    self.vType = vType

class Stream:
  def __init__(self, sid, numberModel, departEdgeModel, arrivalEdgeModel, vTypeModel, timeSpan=3600):
    self.sid = sid
    self._timeSpan = timeSpan
    self._numberModel = numberModel
    self._departEdgeModel = departEdgeModel
    self._arrivalEdgeModel = arrivalEdgeModel
    self._vTypeModel = vTypeModel

  def getVehicleNumer(self, b, e):
    number = int(self.getFrom(self._numberModel))
    return number * (e-b) / self._timeSpan

  def getFrom(self, what):
    if isinstance(what, str): return what
    if isinstance(what, int): return what
    if isinstance(what, float): return what
    if isinstance(what, dict):
      r = random.random()
      s = 0 
      for k in what:
        s = s + k
        if s>r: return what[k]
      return None
    return what.get()
          
  def toVehicles(self, b, e):
    vehicles = []
    number = self.getVehicleNumer(b, e)
    for i in range(0, number):
        fromEdge = self.getFrom(self._departEdgeModel)
        toEdge = self.getFrom(self._arrivalEdgeModel)
        vType = self.getFrom(self._vTypeModel)
        sid = self.sid   
        if sid==None:
          sid = fromEdge + "to" + toEdge
        vehicles.append(Vehicle(sid+"#"+str(i), int(random.random()*(e-b)+b), fromEdge, toEdge, vType))  
    return vehicles




class Demand:
  def __init__(self):
    self.streams = []

  def addStream(self, s):
    self.streams.append(s)
  
  def build(self, b, e, netName, routesName="input_routes.rou.xml"):
    vehicles = []
    for s in self.streams:
      vehicles.extend(s.toVehicles(b, e))
    fdo = open("input_trips.rou.xml", "w")
    fdo.write("<routes>\n")
    for v in sorted(vehicles, key=lambda veh: veh.depart):
      fdo.write('    <trip id="%s" depart="%s" from="%s" to="%s" type="%s"/>\n' % (v.id, v.depart, v.fromEdge, v.toEdge, v.vType))
    fdo.write("</routes>")
    fdo.close()

    duarouter = sumolib.checkBinary("duarouter")
    retCode = subprocess.call([duarouter, "-v", "-n", netName,  "-t", "input_trips.rou.xml", "-o", routesName])
    os.remove(nodesFile)
    os.remove(edgesFile)
    os.remove(connectionsFile)
