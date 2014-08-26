import sumolib.net.generator.cross as netGenerator
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *
from pop import * 


RWS="""
1;0.5104;0.5828;0.5772;0.6332;0.748;2.8719;1.7177
2;0.3432;0.3078;0.3675;0.336;0.4566;1.9838;1.0125
3;0.2107;0.2523;0.2086;0.2895;0.2517;1.3241;0.8154
4;0.3703;0.1997;0.3053;0.2064;0.3579;0.9965;0.4875
5;0.9379;0.4054;0.5936;0.457;0.6685;0.6633;0.6375
6;2.5954;1.3955;1.9009;1.5343;2.2885;0.9947;1.2423
7;6.6675;2.9516;4.9363;3.5946;5.1519;1.0119;1.5891
8;8.9356;5.3546;7.2095;4.5774;7.6271;1.4289;2.7169
9;8.1931;6.0357;6.9139;5.2376;6.8091;2.4236;3.8612
10;6.3549;4.9486;6.0444;4.9067;5.7137;3.9569;5.7839
11;5.496;4.4953;5.3436;5.5661;5.2829;5.4762;6.406
12;5.0961;4.778;5.0059;5.955;5.2941;5.9344;7.0014
13;5.3599;5.2839;5.4039;6.853;5.9041;6.4891;7.3738
14;5.6462;5.9352;5.7807;7.2908;6.4795;7.2301;7.6242
15;5.7565;6.6796;6.1341;8.336;6.8031;7.8309;7.7892
16;6.0419;7.4557;7.0506;9.0655;7.0955;7.4463;7.3836
17;6.9183;9.3616;7.8898;8.6483;6.7089;7.7562;6.9353
18;6.6566;10.19;7.5263;7.7115;6.4494;8.2159;6.7839
19;5.8434;8.5196;6.9226;6.4153;5.942;7.5234;6.331
20;4.4669;5.8307;4.9389;4.2742;4.4143;6.1206;4.9072
21;3.3168;3.8433;3.4602;2.8543;3.5677;4.778;4.0775
22;2.0562;2.3324;2.6526;2.2575;2.7751;3.6782;2.9477
23;1.4711;1.9216;1.7354;1.8819;1.8463;2.3371;2.5049
24;0.7552;0.9392;1.0983;1.118;1.3643;1.5282;2.0705
"""

def merge(defaultParams, setParams):
  ret = {}
  for p in defaultParams:
    ret[p] = defaultParams[p]
    if p in setParams:
      ret[p] = setParams[p]
  return ret

class ScenarioSet:
  def __init__(self, name, params):
    self.name = name
    self.params = params
  def getNumRuns(self):
    raise "virtual ScenarioSet/getNumRuns"
  def getAverageDuration(self):
    raise "virtual ScenarioSet/getAverageDuration"
  def iterate(self):
    raise "virtual ScenarioSet/iterate"
  def getInt(self, name):
    return int(self.params[name])



class ScenarioSet_IterateFlowsNA(ScenarioSet):
  def __init__(self, params):
    ScenarioSet.__init__(self, "iterateFlowsNA", merge(
      {"f1from":"0", "f1to":"2400", "f1step":"400","f2from":"0", "f2to":"2400", "f2step":"400"},
      params))
  def getNumRuns(self):
    f1num = 1 + (self.getInt("f1to") - self.getInt("f1from")) / self.getInt("f1step")
    f2num = 1 + (self.getInt("f2to") - self.getInt("f2from")) / self.getInt("f2step")
    return f1num * f2num
  
  """
  Yields returning a built scenario and its description as key/value pairs
  """
  def iterateScenarios(self):
    desc = {"name":"iterateFlowsNA"}
    for f1 in range(self.getInt("f1from"), self.getInt("f1to"), self.getInt("f1step")):
        for f2 in range(self.getInt("f2from"), self.getInt("f2to"), self.getInt("f2step")):
            if f1==0 and f2==0:
              continue
            s = getScenario("BasicCross", demandGenerator.Demand())
            print "Computing for %s<->%s" % (f1, f2)
            #buildDemand(simSteps, pWE, pEW, pNS, pSN)
            s.demand.addStream(demandGenerator.Stream(None, 0, 3600, f1, "2/1_to_1/1", "1/1_to_0/1", { 1:"passenger"})) # why isn't it possible to get a network and return all possible routes or whatever - to ease the process
            s.demand.addStream(demandGenerator.Stream(None, 0, 3600, f1, "0/1_to_1/1", "1/1_to_2/1", { 1:"passenger"})) # why isn't it possible to get a network and return all possible routes or whatever - to ease the process
            s.demand.addStream(demandGenerator.Stream(None, 0, 3600, f2, "1/2_to_1/1", "1/1_to_1/0", { 1:"passenger"})) # why isn't it possible to get a network and return all possible routes or whatever - to ease the process
            s.demand.addStream(demandGenerator.Stream(None, 0, 3600, f2, "1/0_to_1/1", "1/1_to_1/2", { 1:"passenger"})) # why isn't it possible to get a network and return all possible routes or whatever - to ease the process
            s.demandName = "routes.rou.xml"
            s.demand.build(0, 3600, s.netName, s.demandName)
            desc = {"scenario":"BasicCross", "f1":str(f1), "f2":str(f2)}
            yield s, desc
  def getAverageDuration(self):
    return -1 # !!!
        
        
class ScenarioSet_RiLSA1LoadCurves(ScenarioSet):
  def __init__(self, params):
    ScenarioSet.__init__(self, "RiLSA1LoadCurves", merge(
      {"f1from":"0", "f1to":"2400", "f1step":"400","f2from":"0", "f2to":"2400", "f2step":"400"},
      params))
  def getNumRuns(self):
    return 3*3*3*3
  
  """
  Yields returning a built scenario and its description as key/value pairs
  """
  def iterateScenarios(self):
    desc = {"name":"RiLSA1LoadCurves"}
    RWScurves = [[]]*3
    for l in RWS.split("\n"):
      l = l.strip().split(";")
      if len(l)<2:
        continue
      RWScurves[0].append(float(l[1])/100.)
      RWScurves[1].append(float(l[2])/100.)
      RWScurves[2].append(float(l[3])/100.)
      
    for iWE,cWE in enumerate(RWScurves):
      for iNS,cNS in enumerate(RWScurves):
        for iEW,cEW in enumerate(RWScurves):
          for iSN,cSN in enumerate(RWScurves):
            s = getScenario("RiLSA1")
            print "Computing for %s %s %s %s" % (iWE, iNS, iEW, iSN)
            nStreams = []
            for j in range(0, 3):
              nStreams.extend(extrapolateDemand(s.demand.streams[j+0*3], 3600, cWE).streams) 
              nStreams.extend(extrapolateDemand(s.demand.streams[j+1*3], 3600, cNS).streams) 
              nStreams.extend(extrapolateDemand(s.demand.streams[j+2*3], 3600, cEW).streams) 
              nStreams.extend(extrapolateDemand(s.demand.streams[j+3*3], 3600, cSN).streams)
            s.demand.streams = nStreams 
            s.demand.build(0, 86400, s.netName, s.demandName)
            desc = {"scenario":"RiLSA1LoadCurves", "iWE":str(iWE), "iNS":str(iNS), "iEW":str(iEW), "iSN":str(iSN)}
            yield s, desc
  def getAverageDuration(self):
    return -1 # !!!        
      
def getScenarioSet(name, params):
  if name=="iterateFlowsNA":
    return ScenarioSet_IterateFlowsNA(params)  
  if name=="RiLSA1LoadCurves":
    return ScenarioSet_RiLSA1LoadCurves(params)  
  raise "unknown scenario '%s'" % name

def getAllScenarioSets():
  return ";".join(["iterateFlowsNA"])    