import os, subprocess, sys, random
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'tools'))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(os.path.dirname(__file__), '..', '..')), 'tools'))
import sumolib.net
import sumolib.output

FILES = sys.argv[1].split(",")  
AGG = 3600.

departs = {}
minAggDepart = None
maxAggDepart = None 
departSums = {}

for f in FILES:
  print "Reading %s" % f
  departs[f] = {}
  departSums[f] = 0
  routes = sumolib.output.parse(f, "vehicle")
  for v in routes:
    es = v.route[0].edges.split(" ")
#    if "572247818" not in es:
#      continue
    departAgg = int(float(v.depart) / AGG)
    if departAgg not in departs[f]:
      departs[f][departAgg] = 0
    departs[f][departAgg] += 1
    if not minAggDepart or departAgg<minAggDepart: minAggDepart = departAgg
    if not maxAggDepart or departAgg>maxAggDepart: maxAggDepart = departAgg

for i in range(int(minAggDepart), int(maxAggDepart)+1):
  o = str(i)
  for f in FILES:
    if i not in departs[f]:
      departs[f][i] = 0
    o = o + ";%s" % (departs[f][i])
    departSums[f] += departs[f][i]
  print o

o = "Overall:"
for f in FILES:
  o = o + ";%s" % (departSums[f])
print o



