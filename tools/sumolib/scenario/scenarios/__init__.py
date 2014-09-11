import sumolib.net.generator.cross as netGenerator
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *
import sumolib
import os


SANDBOX_PATH = os.path.join(os.path.dirname(__file__), "..", "sandbox")
REBUILD = False

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
  if REBUILD:
    return True
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
    try: os.makedirs(os.path.join(SANDBOX_PATH, self.name))
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
    return os.path.join(SANDBOX_PATH, self.name, fileName)


        




       
        
    
     

def getScenario(name, withDefaultDemand=True):
  if name=="RiLSA1":
    import RiLSA1
    return RiLSA1.Scenario_RiLSA1(withDefaultDemand)  
  elif name=="BasicCross":
    import BasicCross
    return BasicCross.Scenario_BasicCross(withDefaultDemand)  
  raise "unknown scenario '%s'" % name

    