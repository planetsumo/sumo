import sumolib.net.generator.cross as netGenerator
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *
from pop import * 

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
        pWE = pEW = float(f1)/3600 # [veh/s]
        for f2 in range(self.getInt("f2from"), self.getInt("f2to"), self.getInt("f2step")):
            if f1==0 and f2==0:
              continue
            pNS = pSN = float(f2)/3600 # [veh/s]
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
        
      
def getScenarioSet(name, params):
  if name=="iterateFlowsNA":
    return ScenarioSet_IterateFlowsNA(params)  
  raise "unknown scenario '%s'" % name

def getAllScenarioSets():
  return ";".join(["iterateFlowsNA"])    