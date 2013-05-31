#!/usr/bin/env python
# -*- coding: utf-8 -*-

# call jtrrouter twice and check that the output differs
import sys,os,subprocess,random
sys.path.append(os.path.join(os.path.dirname(sys.argv[0]), '..', '..', '..', '..', "tools"))
from sumolib import checkBinary

def buildFlows(simSteps, pWE, pEW, pNS, pSN):
    fd = open("routes.rou.xml", "w")
		#---routes---
		print >> fd, """<routes>
		
			<vType id="type1" accel="2.0" decel="5.0" sigma="0.0" length="6.5" maxSpeed="70"/>
		
			<route id="WE" edges="1i 3o 5o"/>
			<route id="NS" edges="2i 4o 6o"/>
			<route id="EW" edges="3i 1o 7o"/>
			<route id="SN" edges="4i 2o 8o"/>
			
		"""
		lastVeh = 0
		vehNr = 0
		for i in range(simSteps):
			if random.uniform(0,1) < pWE: # Poisson distribution
				print >> fd, '    <vehicle id="%i" type="type1" route="WE" depart="%i" departSpeed="13.89" />' % (vehNr, i)
				vehNr += 1
				lastVeh = i
			if random.uniform(0,1) < pNS:
				print >> fd, '    <vehicle id="%i" type="type1" route="NS" depart="%i" departSpeed="13.89" />' % (vehNr, i)
				vehNr += 1
				lastVeh = i
			if random.uniform(0,1) < pEW:
				print >> fd, '    <vehicle id="%i" type="type1" route="EW" depart="%i" departSpeed="13.89" />' % (vehNr, i)
				vehNr += 1
				lastVeh = i	
			if random.uniform(0,1) < pSN:
				print >> fd, '    <vehicle id="%i" type="type1" route="SN" depart="%i" departSpeed="13.89" />' % (vehNr, i)
				vehNr += 1
				lastVeh = i

		print >> fd, "</routes>"
		fd.close()  
  
def buildMeasures():
  pass  
  
sumo = checkBinary('sumo')
assert(sumo)
args = [sumo,
        '--net-file', 'net.net.xml',
        '--route-files', 'routes.rou.xml',
        '--tripinfo-output', 'tripinfos.xml',
        ]

flow1def = "0;2400;200".split(";")
flow2def = "0;2400;200".split(";")
steps = 100001

measures = []

for f1 in range(float(flow1def[0]), float(flow1def[1]), float(flow1def[2])):
  	pWE = float(f1)/3600 # [veh/s]
    pEW = pWE
    for f2 in range(float(flow2def[0]), float(flow2def[1]), float(flow2def[2])):
    		pNS = float(f2)/3600 # [veh/s]
    		pSN = pNS
        buildRoutes(steps, pWE, pEW, pNS, pSN)
        subprocess.call(args)
        buildMeasures(f1, f2)
         


def get_depart_lines(route_file):
    return [l for l in open(route_file) if 'depart' in l]

output_file1 = 'output1.rou.xml'
output_file2 = 'output2.rou.xml'

jtrrouter = checkBinary('jtrrouter')
assert(jtrrouter)

args = [jtrrouter,
        '--net-file', 'input_net.net.xml',
        '--flow-files', 'input_flows.flows.xml',
        '--turn-ratio-files', 'input_turns.turns.xml',
        '--output-file', output_file1,
        '--sinks=end',
        '--seed', None,
        '--no-step-log',
        '--randomize-flows']

args[11] = str(random.randint(0, 2 ** 31))
subprocess.call(args)
route_lines1 = get_depart_lines(output_file1)

args[8] = output_file2
args[11] = str(random.randint(0, 2 ** 31))
subprocess.call(args)
route_lines2 = get_depart_lines(output_file2)

if route_lines1 != route_lines2:
    print('test passed. output is random')
else:
    print('test failed. output is deterministic')
