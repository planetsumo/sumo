import sumolib.net.generator.cross as netGenerator
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *


class Scenario:
  def __init__(self):
    self.net = None
    self.netName = None
    self.demand = None
    self.demandName = None
    self.additional = {}
    self.conn = None
    
  
    
  def extrapolateDemand(origDemand, freq, probs, pivot=demandGenerator.PIVOT__PEAK, tBeg=0):
    ret = demandGenerator.Demand()
    if pivot==Demand.PIVOT__PEAK:
      mmax = 0
      mpos = []
      for i,p in iterate(probs):
        if p>mmax:
          mpos = i
          mmax = p
      if count(probs, p)>1:
        raise "more than one maximum value"
      else:
        pivot = mpos
    t = tBeg
    self.demand = demandGenerator.Demand()
    for i,p in iterate(probs):
      for s in origDemand.streams:
        num = s._numberModel / 100. / probs[pivot] * p
        ret.addStream(Stream(None, t, t+freq, num, s._departEdgeModel, s._arrivalEdgeModel, s._vTypeModel))

  def addAdditionalFile(self, name):
    self.additional[name] = []

  def addAdditional(self, name, add):
    self.additional[name].append(add)
    
  def writeSUMOConfig(self, cfgName, addOptions={}):
    cfg = {}
    for a in addOptions:
      cfg[a] = addOptions[a]
    cfg["net-file"] = self.netName
    cfg["route-files"] = self.demandName
    for a in self.additional:
      fileName = a+".add.xml" 
      if len(self.additional[a])>0:
        sumolib.files.additional.write(fileName, self.additional[a])
      if "additional-files" not in cfg:
        cfg["additional-files"] = fileName
      else:
        cfg["additional-files"] = cfg["additional-files"] + "," + fileName
    fdo = open(cfgName, "w")
    fdo.write("<c>\n")
    for v in cfg:
      fdo.write('  <%s value="%s"/>\n' % (v, cfg[v]))
    fdo.write("</c>\n")
    fdo.close()
      
    
      
  def getNet(self):
    if self.net!=None:
      return self.net        
    if self.netName!=None:
      return sumolib.net.readNet(self.netName)
    raise "network was not build"   


        
class Scenario_BasicCross(Scenario):
  def __init__(self, demand=None):
    Scenario.__init__(self)
    # network
    defaultEdge = Edge(numLanes=1, maxSpeed=13.89)
    defaultEdge.addSplit(100, 1)
    defaultEdge.lanes = [Lane(dirs="rs"), Lane(dirs="l")]
    net = netGenerator.cross(None, defaultEdge)
    self.netName = "net.net.xml"
    net.build(self.netName) # not nice, the network name should be given/returned
    # demand
    self.demandName = "routes.rou.xml"
    if demand==None:
      self.demand = demandGenerator.Demand()
      self.demand.addStream(demandGenerator.Stream(None, 0, 3600, 1000, "2/1_to_1/1", "1/1_to_0/1", { .2:"hdv", .8:"passenger"})) # why isn't it possible to get a network and return all possible routes or whatever - to ease the process
      self.demand.build(0, 3600, self.netName, self.demandName)
    else:
      self.demand = demand
        
      
flowsRiLSA1 = [
    [ "1/2_to_1/1", [
       [ "1/1_to_1/0", 159, 9 ],  
       [ "1/1_to_2/1", 59, 9 ],  
       [ "1/1_to_0/1", 64, 12 ]  
   ] ],

   [ "0/1_to_1/1", [
       [ "1/1_to_2/1", 708, 10 ],  
       [ "1/1_to_1/2", 80, 14 ],  
       [ "1/1_to_1/0", 130, 2 ]  
   ] ],

   [ "2/1_to_1/1", [
       [ "1/1_to_0/1", 571, 10 ],  
       [ "1/1_to_1/2", 57, 9 ],  
       [ "1/1_to_1/0", 47, 3 ]  
   ] ],

   [ "1/0_to_1/1", [
       [ "1/1_to_1/2", 154, 2 ],  
       [ "1/1_to_2/1", 49, 2 ],  
       [ "1/1_to_0/1", 92, 2 ]  
   ] ]
 
]

class Scenario_RiLSA1(Scenario):
  def __init__(self):
    Scenario.__init__(self)
    # set up network
    defaultEdge = Edge(numLanes=1, maxSpeed=13.89)
    defaultEdge.addSplit(100, 1)
    defaultEdge.lanes = [Lane(dirs="rs"), Lane(dirs="l")]
    net = netGenerator.cross(None, defaultEdge)
    self.netName = "RiLSA1.net.xml"
    net.build(self.netName)
    # build the demand model (streams)
    self.demand = demandGenerator.Demand()
    for f in flowsRiLSA1:
      for rel in f[1]:
        prob = rel[2]/100.
        iprob = 1. - prob
        self.demand.addStream(demandGenerator.Stream(None, 0, 3600, rel[1], f[0], rel[0], { prob:"lkw", iprob:"pkw"}))
    self.demandName = "routes.rou.xml"
    self.demand.build(0, 3600, self.netName, self.demandName)


def getScenario(name, demand):
  if name=="RiLSA1":
    return Scenario_RiLSA1(demand)  
  elif name=="BasicCross":
    return Scenario_BasicCross(demand)  
  raise "unknown scenario '%s'" % name

    