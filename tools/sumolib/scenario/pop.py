import sumolib.net.generator.cross as netGenerator
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *
import sumolib
import os


SCENARIO_PATH = os.path.join(os.path.dirname(__file__), "sandbox")

def maxIndexValue(l):
    max_val = max(l)
    max_idx = l.index(max_val)
    return max_idx, max_val
    
def minIndexValue(l):
    min_val = min(l)
    min_idx = l.index(min_val)
    return min_idx, min_val
    
def maxIndexValue_unset(l, l2):
    i = 0
    max_val = None
    max_idx = -1
    while i<len(l):
      if l2[i]!=0:
        i = i + 1
        continue
      if max_val==None or max_val<l[i]:
        max_idx = i
        max_val = l[i]
      i = i + 1
    return max_idx, max_val
    
def minIndexValue_unset(l, l2):
    i = 0
    min_val = None
    min_idx = -1
    while i<len(l):
      if l2[i]!=0:
        i = i + 1
        continue
      if min_val==None or min_val>l[i]:
        min_idx = i
        min_val = l[i]
      i = i + 1
    return min_idx, min_val
    
def fileNeedsRebuild(filePath, app):
  if not os.path.exists(filePath):
    return True
  genAppPath = sumolib.checkBinary(app)
  tf = os.path.getmtime(filePath)
  ta = os.path.getmtime(genAppPath)
  return tf<ta 


def split_by_proportions(total, proportions, mininum_values):
    """splits the given total by the given proportions but ensures that each value in
    the result has at least the given minimum value"""
    assert(len(proportions) == len(mininum_values))
    assert(total >= sum(mininum_values))
    assert(min(proportions) > 0)
    num = len(proportions)
    sumProportions = float(sum(proportions))
    fractions = [p / sumProportions for p in proportions]
    result = [max(m, int(round(total * f))) for f,m in zip(fractions, mininum_values)]
    delta = sum(result) - total
    correct = -1 if delta > 0 else 1
    i = 0
    while delta != 0:
        if result[i] + correct >= mininum_values[i]:
            result[i] += correct
            delta += correct
        i = (i + 1) % num

    assert(sum(result) == total)
    return result

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


        




       
      
flowsRiLSA1 = [
   [ "nmp", [
       [ "ms", 359, 9 ],  
       [ "me", 59, 9 ],  
       [ "mw", 64, 12 ]  
   ] ],

   [ "wmp", [
       [ "me", 508, 10 ],  
       [ "mn", 80, 14 ],  
       [ "ms", 130, 2 ]  
   ] ],

   [ "emp", [
       [ "mw", 571, 10 ],  
       [ "mn", 57, 9 ],  
       [ "ms", 47, 3 ]  
   ] ],

   [ "smp", [
       [ "mn", 354, 2 ],  
       [ "me", 49, 2 ],  
       [ "mw", 92, 2 ]  
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
  def getOppositeFlows(self):
    fNS = [0]*24
    fWE = [0]*24
    for s in self.demand.streams:
      if s._departEdgeModel.startswith("em") or s._departEdgeModel.startswith("wm"):
        fWE[int(s._validFrom/3600)] = fWE[int(s._validFrom/3600)] + s._numberModel
      else:  
        fNS[int(s._validFrom/3600)] = fNS[int(s._validFrom/3600)] + s._numberModel 
    return (fNS, fWE)
  def buildWAUT(self, streamsNS, streamsWE):
    #
    rel = []
    ovr = []
    program = [0] * len(streamsNS)
    for i,x in enumerate(streamsNS):
      s = streamsNS[i]+streamsWE[i]
      if s!=0:
        rel.append(x/s)
      else:
        rel.append(0)
      ovr.append(s)
    #
    NIGHT = 1
    DAY = 2
    MORNING = 3
    AFTERNOON = 4
    NIGHT_MAX = 500
    
    MORNING_MIN = 500
    AFTERNOON_MIN = 500
    REL_THRESHOLD = .45
    IREL_THRESHOLD = 1. - REL_THRESHOLD
    
    
    i = 0
    while i<len(streamsNS) and streamsNS[i]+streamsWE[i]<NIGHT_MAX:#streamsNS[i]<NIGHT_MAX and streamsWE[i]<NIGHT_MAX:
      program[i] = NIGHT 
      i = i + 1 
    i = len(streamsNS)-1
    while i>0 and streamsNS[i]+streamsWE[i]<NIGHT_MAX:# and streamsNS[i]<NIGHT_MAX and streamsWE[i]<NIGHT_MAX:
      program[i] = NIGHT
      i = i - 1  
    i = 0
    maxIdx, maxVal = maxIndexValue_unset(rel, program)
    minIdx, minVal = minIndexValue_unset(rel, program)
    print program
    print "morning max %s %s" % (maxIdx, maxVal)
    print "morning min %s %s" % (minIdx, minVal)
    print "%s %s" % (maxVal, 1.-minVal)
    if maxVal>1.-minVal:
      i = maxIdx
      while i>0 and rel[i]>IREL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>MORNING_MIN:
        program[i] = MORNING
        i = i - 1    
      i = maxIdx + 1
      while i<len(rel) and rel[i]>IREL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>MORNING_MIN:
        program[i] = MORNING
        i = i + 1
    else:
      i = minIdx
      print "!"
      print "  %s %s %s %s" % (i, rel[i], program[i], streamsNS[i]+streamsWE[i])
      while i>0 and rel[i]<REL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>MORNING_MIN:
        program[i] = MORNING
        i = i - 1    
        print "  %s %s %s %s" % (i, rel[i], program[i], streamsNS[i]+streamsWE[i])
      i = minIdx + 1
      print "  %s %s %s %s" % (i, rel[i], program[i], streamsNS[i]+streamsWE[i])
      while i<len(rel) and rel[i]<REL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>MORNING_MIN:
        program[i] = MORNING
        i = i + 1
        print "  %s %s %s %s" % (i, rel[i], program[i], streamsNS[i]+streamsWE[i])
    
    
    maxIdx, maxVal = maxIndexValue_unset(rel, program)
    minIdx, minVal = minIndexValue_unset(rel, program)
    print "%s %s" % (maxIdx, maxVal)
    print "%s %s" % (minIdx, minVal)
    print "%s %s" % (maxVal, 1-minVal)
    if maxVal>1-minVal:
      i = maxIdx
      while i>0 and rel[i]>IREL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>AFTERNOON_MIN: # and rel[i]>.6 
        program[i] = AFTERNOON
        i = i - 1    
      i = maxIdx + 1
      while i<len(rel) and rel[i]>IREL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>AFTERNOON_MIN:
        program[i] = AFTERNOON
        i = i + 1
    else:
      i = minIdx
      while i>0 and rel[i]<REL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>AFTERNOON_MIN: # 
        program[i] = AFTERNOON
        i = i - 1    
      i = minIdx + 1
      while i<len(rel) and rel[i]<REL_THRESHOLD and program[i]==0 and streamsNS[i]+streamsWE[i]>AFTERNOON_MIN:
        program[i] = AFTERNOON
        i = i + 1
    for i,x in enumerate(program):
      if program[i]==0:
        program[i] = DAY
    fdo = open("waut.txt","w")
    for i,x in enumerate(program):
      fdo.write("%s %s %s %s %s\n" % (streamsNS[i], streamsWE[i], ovr[i], rel[i], program[i]))
    fdo.close()
    #
    ret1 = {}
    for i in range(1, 5):
      maxV = None
      maxI = 0
      for j,v in enumerate(rel):
        if program[j]!=i:
          continue
        if maxV==None:
          maxV = rel[j]
          maxI = j
        elif maxV<rel[j]:
          maxV = rel[j]
          maxI = j
        elif maxV<1.-rel[j]:
          maxV = 1.-rel[j]
          maxI = j
      print "%s %s %s" % (i, maxV, maxI)
      greens = split_by_proportions(72, (rel[maxI], 1.-rel[maxI]), (10, 10))
      ret1[i] = greens
      print greens
    # build programs
    ret2 = []
    last = -1
    for j,v in enumerate(program):
      if last!=v:
        ret2.append((j, v))
        last = v
    return (ret1, ret2)    
        
    
     

def getScenario(name, withDefaultDemand=True):
  if name=="RiLSA1":
    return Scenario_RiLSA1(withDefaultDemand)  
  elif name=="BasicCross":
    import BasicCross
    return BasicCross.Scenario_BasicCross(withDefaultDemand)  
  raise "unknown scenario '%s'" % name

    