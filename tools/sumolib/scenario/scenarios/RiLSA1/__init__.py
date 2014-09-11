"""
@author  Daniel.Krajzewicz@dlr.de
@date    2014-09-01
@version $Id: __init__.py 3774 2014-08-06 10:23:19Z erdm_ja $

Copyright (C) 2014 DLR/TS, Germany
All rights reserved
"""


import os
from .. import * 
import sumolib.net.generator.demand as demandGenerator
from sumolib.net.generator.network import *



      
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
  NAME = "RiLSA1"
  THIS_DIR = os.path.abspath(os.path.dirname(__file__))
  TLS_FILE = os.path.join(THIS_DIR, "tls.add.xml")
  NET_FILE = os.path.join(THIS_DIR, "rilsa1.net.xml") 

  def __init__(self, withDefaultDemand=True):
    Scenario.__init__(self, self.NAME)
    self.netName = self.fullPath(self.NET_FILE)
    self.demandName = self.fullPath("routes.rou.xml")
    # network
    if fileNeedsRebuild(self.netName, "netconvert"):
      netconvert = sumolib.checkBinary("netconvert")
      retCode = subprocess.call([netconvert, "-c", os.path.join(self.THIS_DIR, "build.netc.cfg")])
    # build the demand model (streams)
    if withDefaultDemand:
      self.demand = demandGenerator.Demand()
      for f in flowsRiLSA1:
        for rel in f[1]:
          prob = rel[2]/100.
          iprob = 1. - prob
          self.demand.addStream(demandGenerator.Stream(f[0]+"__"+rel[0], 0, 3600, rel[1], f[0], rel[0], { prob:"hdv", iprob:"passenger"}))
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
    
    print rel
    print program
    maxIdx, maxVal = maxIndexValue_unset(rel, program)
    minIdx, minVal = minIndexValue_unset(rel, program)
    if maxIdx!=-1 and minIdx!=-1:
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