#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2007-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    detector.py
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @date    2007-06-28
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function
import sys
from collections import defaultdict
from xml.sax import make_parser, handler

MAX_POS_DEVIATION = 10


class DetectorGroupData:

    def __init__(self, pos, isValid, id=None):
        self.ids = []
        self.pos = pos
        self.isValid = isValid
        self.totalFlow = 0
        self.avgSpeed = 0
        self.entryCount = 0
        if id is not None:
            self.ids.append(id)

    def addDetFlow(self, flow, speed):
        oldFlow = self.totalFlow
        self.totalFlow += flow
        if flow > 0:
            self.avgSpeed = (
                self.avgSpeed * oldFlow + speed * flow) / self.totalFlow
        self.entryCount += 1

    def clearFlow(self):
        self.totalFlow = 0
        self.avgSpeed = 0
        self.entryCount = 0


class DetectorReader(handler.ContentHandler):

    def __init__(self, detFile=None, laneMap={}):
        self._edge2DetData = defaultdict(list)
        self._det2edge = {}
        self._currentGroup = None
        self._currentEdge = None
        self._laneMap = laneMap
        if detFile:
            parser = make_parser()
            parser.setContentHandler(self)
            parser.parse(detFile)

    def addDetector(self, id, pos, edge):
        if id in self._det2edge:
            print("Warning! Detector %s already known." % id, file=sys.stderr)
            return
        if edge is None:
            raise RuntimeError("Detector '%s' has no edge" % id)
        if self._currentGroup:
            self._currentGroup.ids.append(id)
        else:
            haveGroup = False
            for data in self._edge2DetData[edge]:
                if abs(data.pos - pos) <= MAX_POS_DEVIATION:
                    data.ids.append(id)
                    haveGroup = True
                    break
            if not haveGroup:
                self._edge2DetData[edge].append(
                    DetectorGroupData(pos, True, id))
        self._det2edge[id] = edge

    def getEdgeDetGroups(self, edge):
        return self._edge2DetData[edge]

    def startElement(self, name, attrs):
        if name == 'detectorDefinition' or name == 'e1Detector':
            self.addDetector(attrs['id'], float(attrs['pos']),
                             self._laneMap.get(attrs['lane'], self._currentEdge))
        elif name == 'group':
            self._currentGroup = DetectorGroupData(float(attrs['pos']),
                                                   attrs.get('valid', "1") == "1")
            edge = attrs['orig_edge']
            self._currentEdge = edge
            self._edge2DetData[edge].append(self._currentGroup)

    def endElement(self, name):
        if name == 'group':
            self._currentGroup = None

    def addFlow(self, det, flow, speed=0.0):
        if det in self._det2edge:
            edge = self._det2edge[det]
            for group in self._edge2DetData[edge]:
                if det in group.ids:
                    group.addDetFlow(flow, speed)
                    break

    def clearFlows(self):
        for groupList in self._edge2DetData.itervalues():
            for group in groupList:
                group.clearFlow()

    def readFlows(self, flowFile, det="Detector", flow="qPKW", speed=None, time=None, timeVal=None, timeMax=None):
        detIdx = -1
        flowIdx = -1
        speedIdx = -1
        timeIdx = -1
        hadFlow = False
        with open(flowFile) as f:
            for l in f:
                if ';' not in l:
                    continue
                flowDef = [e.strip() for e in l.split(';')]
                if detIdx == -1 and det in flowDef:
                    detIdx = flowDef.index(det)
                    if flow in flowDef:
                        flowIdx = flowDef.index(flow)
                    if speed in flowDef:
                        speedIdx = flowDef.index(speed)
                    if time in flowDef:
                        timeIdx = flowDef.index(time)
                elif flowIdx != -1:
                    if timeIdx == -1 or timeVal is None:
                        timeIsValid = True
                    else:
                        curTime = float(flowDef[timeIdx])
                        timeIsValid = (timeMax is None and curTime == timeVal) or (
                            curTime >= timeVal and curTime < timeMax)
                    if timeIsValid:
                        hadFlow = True
                        if speedIdx != -1:
                            self.addFlow(
                                flowDef[detIdx], float(flowDef[flowIdx]), float(flowDef[speedIdx]))
                        else:
                            self.addFlow(
                                flowDef[detIdx], float(flowDef[flowIdx]))
        return hadFlow

    def findTimes(self, flowFile, tMin, tMax, det="Detector", time="Time"):
        timeIdx = 1
        with open(flowFile) as f:
            for l in f:
                if ';' not in l:
                    continue
                flowDef = [e.strip() for e in l.split(';')]
                if det in flowDef:
                    if time in flowDef:
                        timeIdx = flowDef.index(time)
                elif len(flowDef) > timeIdx:
                    curTime = float(flowDef[timeIdx])
                    if tMin is None or tMin > curTime:
                        tMin = curTime
                    if tMax is None or tMax < curTime:
                        tMax = curTime
        return tMin, tMax
