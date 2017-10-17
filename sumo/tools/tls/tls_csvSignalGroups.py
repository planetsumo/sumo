#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    tls_csvSignalGroups.py
# @author  Mirko Barthauer (Technische Universitaet Braunschweig)
# @date    2017-10-17
# @version $Id: tls_csvSignalGroups.py 26301 2017-10-02 20:48:38Z namdre $
"""
This script helps with converting a CSV input file with green times per signal group into the SUMO format. Additionally, it supports creating template CSV input files 
from a SUMO network file. The input CSV file(s) contain input blocks divided by titles in brackets. The block [general] sets general information relating to the 
signal program like the TLS ID, the program ID, the cycle time [s] and the offset time [s]. Additional 0..n optional parameters can also be supplied.

[general]
cycle time;<CYCLE TIME [s]>
key;<TLS ID>
subkey;<PROGRAM ID>
offset;<OFFSET [s]>
param;<KEY>;<VALUE>
param;<KEY>;<VALUE>

The [links] block lists the relations between signal groups and junction connections in SUMO. The relation is build from the edges/lanes 
controlled by the respective signal group. The target edge/lane can be omitted.

[links]
<SIGNAL_GROUP_ID>;<FROM_LANE/EDGE_ID>
<SIGNAL_GROUP_ID>;<FROM_LANE/EDGE_ID>;<FROM_LANE/EDGE_ID>

The last block [signal groups] contains the table of green times and signal group properties. The table uses a header row with the following keywords and their meanings:
id = signal group id, see block [links]
on1 = start time (signal state green) [s] for the first green time in the cycle
off1 = end time (signal state green) [s] for the first green time in the cycle
on2 = optional start time (signal state green) [s] for the second green time in the cycle
off2 = optional end time (signal state green) [s] for the second green time in the cycle
transOn = duration [s] for special signal state before green (usually red-yellow)
transOff = duration [s] for special signal state after green (usually yellow)

The keywords may have a different order than shown above. The definition of a second green time per cycle (keywords "on2" and "off2") is optional, so are the keywords.
Call the script with --help to see all the available command line parameters.


"""
from __future__ import absolute_import
from __future__ import print_function

import sys
import os
import csv
import math
import xml
import xml.dom.minidom
import argparse

try:
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), "tools"))
    import sumolib.net
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

class TlLogic(sumolib.net.TLSProgram):

    def __init__(self, id, programID, cycleTime, offset = 0, parameters = {}, net = None, useTlIndex = True, debug = False):
        if(not isinstance( cycleTime, int ) or cycleTime < 1):
            print("Invalid cycle time = %s" % str(cycleTime))
                
        sumolib.net.TLSProgram.__init__(self, id, str(offset), "static")
        self._cycleTime = cycleTime # cycle time [s]
        self._programID = programID
        self._parameters = parameters
        self._useTlIndex = useTlIndex
        self.net = net
        self._signalGroups = {}
        self.__signalGroupOrder = []
        self._allTimes = [0]
        self._debug = debug
        self._tlIndexToSignalGroup = {}
        
    def addSignalGroups(self, signalGroups, signalGroupOrder = None):
        self._signalGroups = signalGroups
        for sgID in self._signalGroups:
            self._signalGroups[sgID].tlLogic = self
        if(signalGroupOrder is not None):
            self.__signalGroupOrder = signalGroupOrder
        else:
            self.__signalGroupOrder = list(self.signalGroups.keys())
            self.__signalGroupOrder.sort()
    
    def _timeToCycle(self, time):
        return time % self._cycleTime
    
    def setFreeTime(self):
        for sg in self._signalGroups.values():
            sg._times = {} # clear previous entries
            for fromTime, toTime in sg._freeTimes: # calculate times of signal state changes
                if(sg._transTimes[0] > 0):
                    sg._times[self._timeToCycle(fromTime-sg._transTimes[0])] = sg._start
                sg._times[fromTime] = sg._free
                if(sg._transTimes[1] > 0):
                    sg._times[self._timeToCycle(toTime)] = sg._stop
                    sg._times[self._timeToCycle(toTime + sg._transTimes[1])] = sg._red
                else:
                    sg._times[self._timeToCycle(toTime)] = sg._red
                for timeKey in sg._times:
                    if(timeKey not in self._allTimes):
                        self._allTimes.append(timeKey)
        if(self._debug):
            print("All switch times: %s" % str(self._allTimes))
        for sg in self._signalGroups.values(): 
            sg.calculateCompleteSignals(self._allTimes)

    def setSignalGroupRelations(self, sgToLinks):
        if(self.net is not None):
            tls = self.net._id2tls[self._id]
            connections = tls._connections
            
            for sgID in sgToLinks:
                for fromLink, toLink in sgToLinks[sgID]:
                    # check link validity (lane exists?)
                    for connIn, connOut, tlIndex in connections:
                        valid = True
                        if(fromLink.find('_') > 0):
                            valid = fromLink == connIn.getID()
                        else:
                            valid = fromLink == connIn.getEdge().getID()
                        if(toLink != '' and valid):
                            if(toLink.find('_') > 0):
                                valid = toLink == connOut.getID()
                            else:
                                valid = toLink == connOut.getEdge().getID()
                        if(valid):
                            if(self._debug):
                                print("Valid description from %s to %s (SG %s, tlIndex %d)" % (connIn.getID(), connOut.getID(), sgID, tlIndex))
                            self._signalGroups[sgID].addConnection(connIn, connOut, tlIndex)
                            self._tlIndexToSignalGroup[tlIndex] = sgID
                            
            for sgID in self._signalGroups:
                self._signalGroups[sgID].checkYielding()
        
    def xmlOutput(self, doc):
        # transform signal group based information to "phase" elements of constant signal states
        self._allTimes.sort()        
        if(self._useTlIndex and len(self._tlIndexToSignalGroup) > 0):
            tlIndices = list(self._tlIndexToSignalGroup.keys())
            tlIndices.sort()
            sgIndices = [self._tlIndexToSignalGroup[i] for i in tlIndices]
        elif(len(self.__signalGroupOrder) == len(self._signalGroups)):
            sgIndices = self.__signalGroupOrder
        else:
            sgIndices = list(self._signalGroups.keys())
            sgIndices.sort()
        
        tlEl = doc.createElement("tlLogic")
        tlEl.setAttribute("id", self._id)
        tlEl.setAttribute("type", "static")
        tlEl.setAttribute("programID", self._programID)
        tlEl.setAttribute("offset", self._offset)
        commentNode = doc.createComment(" Order of signal groups: %s " % " ".join(sgIndices))
        tlEl.appendChild(commentNode)
        
        # output custom parameters
        for key in self._parameters:
            parEl = doc.createElement("param")
            parEl.setAttribute("key", key)
            parEl.setAttribute("value", self._parameters[key])
            tlEl.appendChild(parEl)

        for i in range(0, len(self._allTimes)-1):
            # fill duration up to the cycle time
            if(i == len(self._allTimes)-2):
                duration =  self._cycleTime - self._allTimes[i]
            else:
                duration = self._allTimes[i+1] - self._allTimes[i]
            states = "".join([self._signalGroups[sg].completeSignals[self._allTimes[i]] for sg in sgIndices])
            phaseEl = doc.createElement("phase")
            phaseEl.setAttribute("duration", str(duration))
            phaseEl.setAttribute("state", states)
            tlEl.appendChild(phaseEl)
        return tlEl

class SignalGroup(object):
    
    def __init__(self, id, free = "g", transTimeOn = 0, transTimeOff = 0):
        self._id = id
        self._free = free
        self._red = "r"
        self._start = "u"
        self._stop = "y"
        self._transTimes = (max(0, transTimeOn), max(0, transTimeOff))
        self._times = {}
        self._freeTimes = []
        self.completeSignals = {}
        self.tlLogic = None
        self._indexPerJunction = {}
        self._yield = False
    
    def addFreeTime(self, fromTime, toTime):
        if(fromTime != toTime):
            self._freeTimes.append((fromTime, toTime))
    
    def addConnection(self, connIn, connOut, tlIndex):
        junction = connIn.getEdge().getToNode()
        # get junction index of the connection
        connections = junction.getConnections()
        junctionIndex = -1
        for conn in connections:
            if(conn.getFromLane() == connIn and conn.getToLane() == connOut):
                junctionIndex = conn.getJunctionIndex()
                break
        if(junctionIndex >= 0):
            if(junction not in self._indexPerJunction):
                self._indexPerJunction[junction] = [junctionIndex]
            elif(junctionIndex not in self._indexPerJunction[junction]):
                self._indexPerJunction[junction].append(junctionIndex)
    
    def checkYielding(self):
        wait = False
        for junction in self._indexPerJunction:
            connections = junction.getConnections()
            
            for conn in connections:
                if(conn.getTLLinkIndex() != ''):
                    otherJunctionIndex = conn.getJunctionIndex()
                    if(otherJunctionIndex not in self._indexPerJunction[junction]):
                        for ownIndex in self._indexPerJunction[junction]:
                            wait = junction._prohibits[ownIndex][-(otherJunctionIndex - 1)] == '1'
                            if(wait):
                                break
                if(wait):
                    break
        self._yield = wait
        if(not self._yield and self._free.lower() == 'g' and self._stop.lower() == 'y'):
            self._free = self._free.upper()
            self._stop = self._stop.upper()
    
    def calculateCompleteSignals(self, times):
        for time in times:
            self.completeSignals[time] = self.getStateAt(time)
    
    def getStateAt(self, time): 
        if(self._yield): # return off/blinking if no times are set
            result = "o"
        else:
            result = "O"
        if(len(self._times) > 0):
            timeKeys = list(self._times.keys())
            timeKeys.sort()
            relevantKey = None
            if(time < timeKeys[0] or time >= timeKeys[len(timeKeys)-1]):
                relevantKey = timeKeys[len(timeKeys)-1]
            else:
                for i in range(0, len(timeKeys)-1):
                    if(time >= timeKeys[i] and time < timeKeys[i+1]):
                        relevantKey = timeKeys[i]
                        break
            result = self._times[relevantKey]
        return result
    
    def __str__(self):
        return "SignalGroup %s (%ds %s %s %ds %s) has %d free times" % (self._id, self._transTimes[0], self._start, self._free, self._transTimes[1], self._stop, len(self._freeTimes))

def writeXmlOutput(tlList, outputFile):
    if(len(tlList) > 0):
        doc = xml.dom.minidom.Document()
        root = doc.createElement("add")
        doc.appendChild(root)
        
        for tlLogic in tlList:
            tlEl = tlLogic.xmlOutput(doc)
            root.appendChild(tlEl)
        doc.writexml(open(options.output, 'w'),indent = "", addindent = "    ", newl = "\n")

def writeInputTemplates(net, outputDir, delimiter):
    # identify tls-controlled junctions
    for tlsID in net._id2tls:
        # get all connections per junction from the incoming lanes
        connections = net._id2tls[tlsID].getConnections()

        # create the data
        data = [["[general]"], ["cycle time"], ["key", tlsID], ["subkey", "0"], ["offset", "0"], ["[links]"]]
        for connIn, connOut, tlIndex in connections:
            data.append(["SG_" + str(tlIndex), connIn.getID(), connOut.getID()])
        data.extend([["[signal groups]"], ["id", "on1", "off1", "on2", "off2", "transOn", "transOff"]])
        for connIn, connOut, tlIndex in connections:
            data.append(["SG_" + str(tlIndex)])
        
        # write the template file
        with open(os.path.join(outputDir, "%s.csv" % tlsID),'wb') as inputTemplate:
            csvWriter = csv.writer(inputTemplate, quoting=csv.QUOTE_NONE, delimiter=delimiter)
            csvWriter.writerows(data)

def getOptions():
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-o", "--output", action="store", default="tls.add.xml", help="File path to output file (SUMO additional file)")
    argParser.add_argument("-i", "--input", action="store", default="", help="File path to input csv file(s). Multiple file paths have to be separated by ','.")
    argParser.add_argument("--delimiter", action="store", default=";", help="CSV delimiter used for input and template files.")
    argParser.add_argument("-n", "--net", action="store", default="", help="File path to SUMO network file")
    argParser.add_argument("-m", "--make-input-dir", action="store", default="", help="Create input file template(s) from the SUMO network file in the given directory.")
    argParser.add_argument("-t", "--tl-index", action="store_true", default=True, help="Use tl indices from the SUMO network file for output")
    argParser.add_argument("-d", "--debug", action="store_true", default=False, help="Output debugging information")
    options = argParser.parse_args()
    
    return options
        
# this is the main entry point of this script
if __name__ == "__main__":
    options = getOptions()
    
    # read general and signal groub based information from input file(s)
    sections = ["general", "links", "signal groups"]
    signalColumns = ["id", "on1", "off1", "on2", "off2", "transOn", "transOff"]
    if(len(options.input) == 0):
        inputFiles = []
    else:
        inputFiles = options.input.split(',')
    tlList = []
    
    
    # read SUMO network
    net = None
    if(len(options.net) > 0):
        net = sumolib.net.readNet(options.net)
    
        if(len(options.make_input_dir) > 0): # check input template directory
            if(os.path.isdir(options.make_input_dir)):
                writeInputTemplates(net, options.make_input_dir, options.delimiter)
            else:
                sys.stderr.write("The input template directory %s does not exit.\n" % options.make_input_dir)
                sys.exit(-1)
    
    for inputFile in inputFiles: # one signal program per input file
        readSgHeader = False
        secondFreeTime = False
        colIndices = {}
        activeSection = None
        cycleTime = 90
        key = "SZP 1"
        subkey = "0"
        signalGroups = {}
        signalGroupOrder = []
        sgToLinks = {}
        parameters = {}
        
        with open(inputFile, 'r') as inputFile:
            inputReader = csv.reader(inputFile, delimiter = options.delimiter, quotechar='"')
            for i, line in enumerate(inputReader):
                cell0 = line[0].strip()
                
                # skip lines with empty first cell
                if(len(cell0) == 0):
                    continue
                
                # type of input announced by section title
                if(cell0.startswith("[") and cell0.endswith("]")):
                    if(cell0[1:-1] in sections):
                        activeSection = cell0[1:-1]
                        continue
                    else:
                        print("Unknown input section %s." % cell0)
                        sys.exit()
                
                # general TLS input
                if(activeSection == "general"):
                    if(cell0 == "cycle time"):
                        cycleTime = int(line[1])
                    elif(cell0 == "key"):
                        key = line[1].strip()
                    elif(cell0 == "subkey"):
                        subkey = line[1].strip()
                    elif(cell0 == "offset"):
                        offset = int(line[1])
                    elif(cell0 == "param"):
                        parameters[line[1].strip()] = line[2].strip()
                    
                # relation between signal groups and network connection elements
                elif(activeSection == "links"):
                    link = (line[1], line[2])
                    if(cell0 not in sgToLinks):
                        sgToLinks[cell0] = []
                    sgToLinks[cell0].append(link)
                
                # define green times (once or twice per cycle time) and special transitional signal states (yellow...)
                elif(activeSection == "signal groups"):
                    if(not readSgHeader): # remember relation between columns and their meanings
                        readSgHeader = True
                        for colIndex in range(0, len(line)): 
                            if(line[colIndex].strip() in signalColumns):
                                colIndices[line[colIndex].strip()] = colIndex
                        secondFreeTime = "on2" in colIndices.keys() and "off2" in colIndices.keys()
                    else:
                        sg = SignalGroup(line[colIndices["id"]], transTimeOn = int(line[colIndices["transOn"]]), transTimeOff = int(line[colIndices["transOff"]]))
                        sg.addFreeTime(int(line[colIndices["on1"]]), int(line[colIndices["off1"]]))
                        if(secondFreeTime):
                            if(line[colIndices["on2"]] != "" and line[colIndices["off2"]] != ""):
                                sg.addFreeTime(int(line[colIndices["on2"]]), int(line[colIndices["off2"]]))
                        signalGroups[sg._id] = sg
                        signalGroupOrder.append(sg._id)
        
        # build everything together
        tlLogic = TlLogic(key, subkey, cycleTime, parameters = parameters, net = net, useTlIndex = options.tl_index, debug = options.debug)
        tlLogic.addSignalGroups(signalGroups, signalGroupOrder)
        tlLogic.setSignalGroupRelations(sgToLinks)
        tlLogic.setFreeTime()
        tlList.append(tlLogic)
        
    writeXmlOutput(tlList, options.output)
