#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2010-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    generateParkingLots.py
# @author  Yun-Pang Floetteroed
# @date    2017-10-13
# @version $Id: generateParkingLots.py

"""
- generate The Webster's equation is used to optimize the cycle length
  and the green times of the traffic lights in a sumo network
  with a given route file.

"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import collections
import math

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    import sumolib  # noqa
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_options(args=None):
    optParser = optparse.OptionParser()
    optParser.add_option("-i", "--parking-id", dest="parkId",
                         default="parking lot", help="define the name/id of the parking lot")
    optParser.add_option("-x", "--x-axis", dest="xaxis", type="int",
                         help="define x-position of the parking lot")
    optParser.add_option("-y", "--y-axis", dest="yaxis", type="int",
                         help="define y-position of the parking lot")
    optParser.add_option("-b", "--bounding-box", dest="bbox",
                         help="define the xmin, ymin, xmax, ymax of the parking lot")
    optParser.add_option("-n", "--parking-spaces", dest="spaces", type= "int",
                         default= 5, help="define the number of the parking spaces")
    optParser.add_option("-c", "--connecting-edge", dest="connEdge",
                         help="define the connecting edge of the parking lot")
    optParser.add_option("-s", "--start-position", dest="start", type ="int",
                         default= 0, help="define the begin position of the enterance/exit of the parking lot")
    optParser.add_option("-e", "--end-position", dest="end", type ="int",
                         default= 2, help="define the end position of the enterance/exit of the parking lot")
    optParser.add_option("-l", "--space-length", dest="length", type ="int",
                         default= 5, help="define the length of each parking space")
    optParser.add_option("-a", "--space-angle", dest="angle", type="int",
                         default= 315, help="define the name of the output file")
    optParser.add_option("-d", "--space-distance", dest="dist", type="float",
                         default= 5, help="define the distance between the locations of two parking spaces")
    optParser.add_option("-r", "--rotation-degree", dest="rotation", type="int",
                         default= 0, help="define the rotation degree of the parking lot")
    optParser.add_option("--adjustrate-x", dest="factorX", type="float",
                         default= 0.28, help="define the modification rate of x-axis if the rotation exists")
    optParser.add_option("--adjustrate-y", dest="factorY", type="float",
                         default= 0.7, help="define the modification rate of y-axis if the rotation exists")
    optParser.add_option("--output-suffix", dest="suffix", help="output suffix", default="")
    optParser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                         default=False, help="tell me what you are doing")
    (options, args) = optParser.parse_args(args=args)

    if not options.bbox and (not options.xaxis or not options.yaxis) or not options.connEdge :
        optParser.print_help()
        sys.exit()

    return options

def main(options):
    movingX = options.dist
    movingY = options.dist
    factorX = 0.
    factorY = 1.
    row = 0

    if options.bbox:
        xys = options.bbox.split(',')
        x = int(xys[0])
        y = int(xys[1])
    else:
        x = options.xaxis
        y = options.yaxis

    if options.rotation != 0:
        movingX = options.dist*(math.cos(options.rotation*math.pi/180.))
        movingY = options.dist*(math.sin(options.rotation*math.pi/180.))
        factorX = options.factorX
        factorY = options.factorY

    if options.suffix:
        outfile = 'parking_%s.add.xml' %options.suffix
    else:
        outfile = 'parking_%s.add.xml' %options.parkId

    with open(outfile, 'w') as outf:
        outf.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        outf.write('<additional>\n')
        outf.write('    <parkingArea id="%s" lane="%s" startPos="%s" endPos="%s">\n' %(options.parkId, options.connEdge, options.start, options.end))
        for i in range(0,options.spaces):
            outf.write('        <space x="%s" y="%s" length="%s" angle="%s"/>\n' %(x,y,options.length, options.angle))
            if options.bbox:
                if x > float(xys[2]):
                    row += 1
                    if y < float(xys[3]):
                        y = float(xys[1]) + row * options.dist*factorY
                    else:
                        print ("*** The maximum y is reached. Some of the parking lots are overlapped.")
                    x = float(xys[0]) + row * movingX *factorX
                else:
                    x += movingX
                    if options.rotation != 0:
                        y += movingY 
            else:
                # No rotation degree is considered.
                x += options.dist
                y += options.dist
        outf.write('    </parkingArea>\n')
        outf.write('</additional>\n')

if __name__ == "__main__":
    options = get_options(sys.argv)
    main(options)
