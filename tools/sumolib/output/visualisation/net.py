from matplotlib import rcParams
from pylab import *

import os, subprocess, sys, random
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'tools'))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(os.path.dirname(__file__), '..', '..')), 'tools'))


import sumolib.net
import sumolib.output



def toHex(val):
    """Converts the given value (0-255) into its hexadecimal representation"""
    hex = "0123456789abcdef"
    return hex[int(val/16)] + hex[int(val - int(val/16)*16)]

def toFloat(val):
    """Converts the given value (0-255) into its hexadecimal representation"""
    hex = "0123456789abcdef"
    return float(hex.find(val[0])*16 + hex.find(val[1]))


def toColor(val, colormap):
    #0-1
    #
    """Converts the given value (0-1) into a color definition parseable by matplotlib"""
    for i in range(0, len(colormap)-1):
        if colormap[i+1][0]>val:
            scale = (val - colormap[i][0]) / (colormap[i+1][0] - colormap[i][0])
            r = colormap[i][1][0] + (colormap[i+1][1][0] - colormap[i][1][0]) * scale 
            g = colormap[i][1][1] + (colormap[i+1][1][1] - colormap[i][1][1]) * scale 
            b = colormap[i][1][2] + (colormap[i+1][1][2] - colormap[i][1][2]) * scale 
            return "#" + toHex(r) + toHex(g) + toHex(b)
    return "#" + toHex(colormap[-1][1][0]) + toHex(colormap[-1][1][1]) + toHex(colormap[-1][1][2]) 


def parseColorMap(mapDef):
    ret = []
    defs = mapDef.split(",")
    for d in defs:
        (value, color) = d.split(":")
        r = color[1:3]
        g = color[3:5]
        b = color[5:7]
        ret.append( (float(value), ( toFloat(r), toFloat(g), toFloat(b) ) ) )
    return ret



def plotNet(net, colors, widths, **others):
  for e in net._edges:
    gx = []
    gy = []
    for s in e._shape:
      gx.append(s[0])
      gy.append(s[1])
    if e._id in colors:
      c = colors[str(e._id)]
      lw = widths[str(e._id)]
    else:
      c = "#808080"
      lw = .1
    plot(gx, gy, color=c, linewidth=lw)
  if others.get('showA', False):
    show()    
  xlim(6000, 22000)
  ylim(6000, 22000)
  savefig(sys.argv[3])
  show()    
  #self._withPhases = others.get('withPrograms', False)


print log(1)
print log(44428)
print log(1) / log(44428)
print log(222) / log(44428)
print log(2222) / log(44428)

colorMap = parseColorMap("0:#00ff00,.5:#ffff00,1:#ff0000")
print "reading net"
net = sumolib.net.readNet(sys.argv[1], showA=True)
edgeValuesC = {}
edgeValuesW = {}
edgeMaxValuesC = 0
try:
  fd = open(sys.argv[2] + ".joined")
  fd.close() 
except:
  print "pre-parsing data"
  mean = sumolib.output.parse(sys.argv[2], "interval")
  for v in mean:
    t = float(v.begin)
    print " " + str(t)
    for e in v.edge:
      id = str(e.id)
      if id in edgeValuesC:
        edgeValuesC[id] = edgeValuesC[id] + float(e.CO2_abs)
      else:
        edgeValuesC[id] = float(e.CO2_abs)
  fdo = open(sys.argv[2] + ".joined", "w")
  for e in edgeValuesC:
    fdo.write("%s;%s\n" % (e, edgeValuesC[e]))
  fdo.close()

print "parsing data"
fd = open(sys.argv[2] + ".joined")
for line in fd:
  edgeValuesC[line[:line.find(";")]] = float(line[line.find(";")+1:])
fd.close() 


edgeMaxValuesC = None
edgeMinValuesC = None
for e in edgeValuesC:
  if not edgeMaxValuesC or edgeMaxValuesC<edgeValuesC[e]:
    edgeMaxValuesC = edgeValuesC[e]    
  if not edgeMinValuesC or edgeMinValuesC>edgeValuesC[e]:
    edgeMinValuesC = edgeValuesC[e]    
print "Min: %s" % edgeMinValuesC
print "Max: %s" % edgeMaxValuesC

edgeMaxValuesC = 25391453301.7
print log(edgeMaxValuesC)
edgeMinValuesC = 4312.914424

emin = None
emax = None
for e in edgeValuesC:
  if edgeValuesC[e]!=0:
    edgeValuesC[e] = log(edgeValuesC[e]) / log(edgeMaxValuesC)
    if not emin or emin>edgeValuesC[e]:
      emin = edgeValuesC[e]
    if not emax or emax<edgeValuesC[e]:
      emax = edgeValuesC[e]

print emin
print emax

for e in edgeValuesC:
  edgeValuesC[e] = (edgeValuesC[e]-emin) / (emax-emin)
  #edgeValuesC[e] = pow(edgeValuesC[e], 1./5.)
  edgeValuesW[e] = edgeValuesC[e] + 1.
  edgeValuesC[e] = toColor(edgeValuesC[e], colorMap)


print edgeMaxValuesC
print "plotting"
plotNet(net, edgeValuesC, edgeValuesW)


  