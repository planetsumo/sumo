#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
from lxml import etree

parser = argparse.ArgumentParser(
    description='Create tls links from sumo net as needed by tls_csv2SUMO.py. You have to edit the link number field (preset with g). The comment gives the link number shown on demand in SUMO-GUI')
parser.add_argument('net', help='Input file name')

args = parser.parse_args()

doc = etree.parse(args.net)

connections = {}

for conn in doc.xpath('//connection'):
    if 'linkIndex' in conn.attrib:
        # use traffic light id and right adjusted number for sorting and as
        # comment
        numIndex = conn.attrib['linkIndex']
        index = conn.attrib['tl'] + ';' + numIndex.zfill(3)
        connections[index] = conn.attrib['from'] + '_' + conn.attrib['fromLane'] + \
            ';' + conn.attrib['to'] + '_' + conn.attrib['toLane']
        # print record
        # print conn.attrib['from'], conn.attrib['to'],
        # conn.attrib['linkIndex']

for conn in sorted(connections):
        # print conn, connections[conn]
    print "link;g;{};0".format(connections[conn]).ljust(50) + '#' + str(conn).rjust(3)
