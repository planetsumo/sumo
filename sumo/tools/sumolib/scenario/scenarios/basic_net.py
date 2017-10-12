# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2012-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    basic_net.py
# @author  Daniel Krajzewicz
# @date    2014-09-01
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function


from . import *
import os
import math
import sumolib.net.generator.grid as netGenerator
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *


class Scenario_BasicNet(Scenario):
    NAME = "BasicNet"
    THIS_DIR = os.path.join(os.path.abspath(os.path.dirname(__file__)), NAME)
    TLS_FILE = os.path.join(THIS_DIR, "tls.add.xml")
    NET_FILE = os.path.join(THIS_DIR, "network_%s.net.xml")

    def __init__(self, rot, withDefaultDemand=True):
        Scenario.__init__(self, self.NAME)
        self.NET_FILE = self.NET_FILE % rot
        self.netName = self.fullPath(self.NET_FILE)
        self.demandName = self.fullPath("routes.rou.xml")
        # network
        if fileNeedsRebuild(self.netName, "netconvert"):
            print("Network in '%s' needs to be rebuild" % self.netName)
            defaultEdge = Edge(numLanes=1, maxSpeed=13.89)
            defaultEdge.addSplit(50, 1)
            defaultEdge.lanes = [Lane(dirs="rs"), Lane(dirs="l")]
            netGen = netGenerator.grid(5, 5, None, defaultEdge)
            m = rot / 3.14
            for y in range(1, 6):
                for x in range(1, 6):
                    sr = math.sin(rot * y / 2. * x / 2.)
                    cr = math.cos(rot * x / 2. * y / 2.)
                    # * abs(3-x)/3.
                    netGen._nodes[
                        "%s/%s" % (x, y)].x = netGen._nodes["%s/%s" % (x, y)].x + sr * m * 250
                    # * abs(3-y)/3.
                    netGen._nodes[
                        "%s/%s" % (x, y)].y = netGen._nodes["%s/%s" % (x, y)].y + cr * m * 250
            # not nice, the network name should be given/returned
            netGen.build(self.netName)
        # demand
        if withDefaultDemand:
            print("Demand in '%s' needs to be rebuild" % self.demandName)
            self.demand = demandGenerator.Demand()
            # why isn't it possible to get a network and return all possible
            # routes or whatever - to ease the process
            self.demand.addStream(demandGenerator.Stream(
                None, 0, 3600, 1000, "6/1_to_5/1", "1/1_to_0/1", {"hdv": .2, "passenger": .8}))
            if fileNeedsRebuild(self.demandName, "duarouter"):
                self.demand.build(0, 3600, self.netName, self.demandName)
