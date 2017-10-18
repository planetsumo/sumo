#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2010-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    ptlines2flows.py
# @author  Gregor Laemmel
# @author  Jakob Erdmann
# @author  Michael Behrisch
# @date    2017-06-23
# @version $Id$

import os
import sys
import subprocess
import collections

from optparse import OptionParser

import sumolib

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_options():
    optParser = OptionParser()
    optParser.add_option("-n", "--net-file", dest="netfile", help="network file")
    optParser.add_option("-l", "--ptlines-file", dest="ptlines", help="public transit lines file")
    optParser.add_option("-s", "--ptstops-file", dest="ptstops", help="public transit stops file")
    optParser.add_option("-o", "--output-file", dest="outfile", default="flows.rou.xml", help="output flows file")
    optParser.add_option("-i", "--stopinfos-file", dest="stopinfos",
                         default="stopinfos.xml", help="file from '--stop-output'")
    optParser.add_option(
        "-r", "--routes-file", dest="routes", default="vehroutes.xml", help="file from '--vehroute-output'")
    optParser.add_option("-t", "--trips-file", dest="trips", default="trips.trips.xml", help="output trips file")
    optParser.add_option("-p", "--period", type=float, default=600, help="period")
    optParser.add_option("-b", "--begin", type=float, default=0, help="start time")
    optParser.add_option("-e", "--end", type=float, default=3600, help="end time")
    optParser.add_option("--min-stops", type=int, default=2, help="end time")
    optParser.add_option("-f", "--flow-attributes", dest="flowattrs",
                         default="", help="additional flow attributes")
    optParser.add_option("--use-osm-routes", default=False, action="store_true", dest='osmRoutes', help="use osm routes")
    optParser.add_option("--no-vtypes", default=False, action="store_true", dest='novtypes', help="do not write vtypes for generated flows")
    (options, args) = optParser.parse_args()
    return options

def writeTypes(fout):
    fout.write('    <vType id="bus" vClass="bus"/>\n')
    fout.write('    <vType id="tram" vClass="tram"/>\n')
    fout.write('    <vType id="train" vClass="rail"/>\n')
    fout.write('    <vType id="subway" vClass="rail_urban"/>\n')
    fout.write('    <vType id="monorail" vClass="rail"/>\n')
    fout.write('    <vType id="trolleybus" vClass="bus"/>\n')
    fout.write('    <vType id="aerialway" vClass="bus"/>\n')
    fout.write('    <vType id="ferry" vClass="ship"/>\n')

def main():
    options = get_options()
    print("generating trips...")
    net = sumolib.net.readNet(options.netfile)
    stopsLanes = {}
    for stop in sumolib.output.parse_fast(options.ptstops, 'busStop', ['id', 'lane']):
        stopsLanes[stop.id] = stop.lane

    trpIDLineMap = {}
    with open(options.trips, 'w') as fouttrips:
        sumolib.writeXMLHeader(
            fouttrips, "$Id$", "routes")
        writeTypes(fouttrips)
        trp_nr = 0
        for line in sumolib.output.parse(options.ptlines, 'ptLine'):

            stops = line._child_dict['busStop']
            fr = None
            to = None
            stop_ids = []
            for stop in stops:
                if not stop.id in stopsLanes:
                    sys.stderr.write("Warning: skipping unknown stop '%s'\n" % stop.id)
                    continue
                laneId = stopsLanes[stop.id]
                edge_id, lane_index = laneId.rsplit("_", 1)
                if fr == None:
                    fr = edge_id
                    dep_lane = laneId

                to = edge_id
                edge = net.getEdge(edge_id)
                stop_ids.append(stop.id)

            if fr is None or len(stop_ids) < options.min_stops:
                sys.stderr.write("Warning: skipping line '%s' because it has too few stops\n" % line.id)
                trp_nr += 1
                continue

            if options.osmRoutes and 'route' in line._child_dict:
                route = line._child_dict['route']
                edges = route[0].edges.split(' ')
                lenE = len(edges)

                if (lenE > 3):
                    vias = ' '.join(edges[0:lenE])
                    fouttrips.write(
                        '    <trip id="%s" type="%s" depart="0" departLane="%s" from="%s" to="%s" via="%s">\n' % (
                            trp_nr, line.type, 'best', fr, to, vias))
                else:
                    fouttrips.write(
                        '    <trip id="%s" type="%s" depart="0" departLane="%s" from="%s" to="%s" >\n' % (
                            trp_nr, line.type, 'best', fr, to))
            else:
                fouttrips.write(
                    '    <trip id="%s" type="%s" depart="0" departLane="%s" from="%s" to="%s" >\n' % (
                        trp_nr, line.type, 'best', fr, to))

            trpIDLineMap[str(trp_nr)] = line.line
            trp_nr += 1
            for stop in stop_ids:
                fouttrips.write('        <stop busStop="%s" duration="30" />\n' % (stop))
            fouttrips.write('    </trip>\n')
        fouttrips.write("</routes>\n")
    print("done.")
    print("running SUMO to determine actual departure times...")
    subprocess.call([sumolib.checkBinary("sumo"), "-r", options.trips, "-n", options.netfile,
                     "--no-step-log",
                     "--ignore-route-errors",
                     "--error-log", options.trips + ".errorlog",
                     "-a", options.ptstops,
                     "--vehroute-output", options.routes,
                     "--stop-output", options.stopinfos, ])
    print("done.")

    print("creating routes...")
    stopsUntil = {}
    for stop in sumolib.output.parse_fast(options.stopinfos, 'stopinfo', ['id', 'ended', 'busStop']):
        stopsUntil[(stop.id, stop.busStop)] = stop.ended

    with open(options.outfile, 'w') as foutflows:
        flows = []
        sumolib.writeXMLHeader(
            foutflows, "$Id$", "routes")
        if not options.novtypes:
            writeTypes(foutflows)
        for vehicle in sumolib.output.parse(options.routes, 'vehicle'):
            id = vehicle.id
            flows.append((id, vehicle.type))
            edges = vehicle.routeDistribution[0]._child_dict['route'][1].edges
            stops = vehicle.stop
            foutflows.write(
                '    <route id="%s" edges="%s" >\n' % (id, edges))
            for stop in stops:
                if (id, stop.busStop) in stopsUntil:
                    foutflows.write(
                        '        <stop busStop="%s" duration="%s" until="%s" />\n' % (
                            stop.busStop, stop.duration, stopsUntil[(id, stop.busStop)]))
                else:
                    sys.stderr.write("Missing stop '%s' for flow '%s'\n" % (stop.busStop, id))
            foutflows.write('    </route>\n')
        lineCount = collections.defaultdict(int)
        for flow, type in flows:
            lineRef = "%s:%s" % (trpIDLineMap[flow], lineCount[trpIDLineMap[flow]])
            lineCount[trpIDLineMap[flow]] += 1
            foutflows.write('    <flow id="%s_%s" type="%s" route="%s" begin="%s" end="%s" period="%s" line="%s" %s/>\n' %
                            (type, lineRef, type, flow, options.begin, options.end, options.period, lineRef, options.flowattrs))
        foutflows.write('</routes>\n')

    print("done.")


if __name__ == "__main__":
    main()
