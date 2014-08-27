import sumolib.net.generator.cross as netGenerator
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *
import sumolib
import os


SCENARIO_PATH = "D:\\projects\\x_EU_COLOMBO_318622\\svn_smartSVN\\trunk\\software\\TLSEvaluationSystem\\scenarios\\"


def fileNeedsRebuild(filePath, app):
  if not os.path.exists(filePath):
    return True
  genAppPath = sumolib.checkBinary(app)
  tf = os.path.getmtime(filePath)
  ta = os.path.getmtime(genAppPath)
  return tf<ta 

def extrapolateDemand(stream, freq, probs, pivot=demandGenerator.PIVOT__PEAK, tBeg=0):
  ret = demandGenerator.Demand()
  if pivot==demandGenerator.PIVOT__PEAK:
    mmax = 0
    mpos = []
    for i,p in enumerate(probs):
      if p>mmax:
        mpos = i
        mmax = p
    # !!! should be done
    #if count(probs, p)>1:
    #  raise "more than one maximum value"
    #else:
    #  pivot = mpos
    pivot = mpos
  t = tBeg
  for i,p in enumerate(probs):
    num = float(stream._numberModel) * p  / probs[pivot]# ok, this works just if _numberModel is a number
    ret.addStream(demandGenerator.Stream(stream.sid+"_"+str(i), t, t+freq, num, stream._departEdgeModel, stream._arrivalEdgeModel, stream._vTypeModel))
    t = t + freq
  return ret


class Scenario:
  def __init__(self, name):
    self.name = name
    self.net = None
    self.netName = None
    self.demand = None
    self.demandName = None
    self.additional = {}
    self.conn = None
    try: os.makedirs(os.path.join(SCENARIO_PATH, self.name))
    except: pass

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
      self.net = sumolib.net.readNet(self.netName)
      return self.net
    raise "network is unknown"   

  def fullPath(self, fileName):
    return os.path.join(SCENARIO_PATH, self.name, fileName)


        
class Scenario_BasicCross(Scenario):
  def __init__(self, withDefaultDemand=True):
    Scenario.__init__(self, "BasicCross")
    self.netName = self.fullPath("net.net.xml")
    self.demandName = self.fullPath("routes.rou.xml")
    # network
    if fileNeedsRebuild(self.netName, "netconvert"):
      print "Network in '%s' needs to be rebuild" % self.netName
      defaultEdge = Edge(numLanes=1, maxSpeed=13.89)
      defaultEdge.addSplit(100, 1)
      defaultEdge.lanes = [Lane(dirs="rs"), Lane(dirs="l")]
      netGen = netGenerator.cross(None, defaultEdge)
      netGen.build(self.netName) # not nice, the network name should be given/returned
    # demand
    if withDefaultDemand:
      print "Demand in '%s' needs to be rebuild" % self.demandName
      self.demand = demandGenerator.Demand()
      self.demand.addStream(demandGenerator.Stream(None, 0, 3600, 1000, "2/1_to_1/1", "1/1_to_0/1", { .2:"hdv", .8:"passenger"})) # why isn't it possible to get a network and return all possible routes or whatever - to ease the process
      if fileNeedsRebuild(self.demandName, "duarouter"):
        self.demand.build(0, 3600, self.netName, self.demandName)




       
      
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
  def __init__(self, withDefaultDemand=True):
    Scenario.__init__(self, "RiLSA1")
    self.netName = self.fullPath("net.net.xml")
    self.demandName = self.fullPath("routes.rou.xml")
    # network
    if fileNeedsRebuild(self.netName, "netconvert"):
      defaultEdge = Edge(numLanes=1, maxSpeed=13.89)
      defaultEdge.addSplit(100, 1)
      defaultEdge.lanes = [Lane(dirs="rs"), Lane(dirs="l")]
      net = netGenerator.cross(None, defaultEdge)
      net.build(self.netName)
    # build the demand model (streams)
    if withDefaultDemand:
      self.demand = demandGenerator.Demand()
      for f in flowsRiLSA1:
        for rel in f[1]:
          prob = rel[2]/100.
          iprob = 1. - prob
          self.demand.addStream(demandGenerator.Stream(f[0]+"__"+rel[0], 0, 3600, rel[1], f[0], rel[0], { prob:"lkw", iprob:"pkw"}))
      if fileNeedsRebuild(self.demandName, "duarouter"):
        self.demand.build(0, 3600, self.netName, self.demandName)


def getScenario(name, withDefaultDemand=True):
  if name=="RiLSA1":
    return Scenario_RiLSA1(withDefaultDemand)  
  elif name=="BasicCross":
    return Scenario_BasicCross(withDefaultDemand)  
  raise "unknown scenario '%s'" % name

    