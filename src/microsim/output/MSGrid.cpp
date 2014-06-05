/****************************************************************************/
/// @file    MSGrid.cpp
/// @author  Daniel Krajzewicz
/// @date    2011-09-08
/// @version $Id: MSGrid.cpp 11042 2011-07-20 10:53:55Z namdre $
///
// Realises dumping the complete network state
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2011 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cassert>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSGrid.h"
#include <utils/geom/GeoConvHelper.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSGrid::MSGrid(const std::string &id, SUMOReal xs, SUMOReal ys, const Position &offset, const Position &size, MSNet &n) 
    : MSDetectorFileOutput(id), myEdges(n.getEdgeControl()), myBoxSizes(xs, ys), mySize(size)
{
    //Boundary b = GeoConvHelper::getFinal().getConvBoundary();
    myOffset = offset;
    myXNumber = (unsigned int) (ceil(mySize.x()) / myBoxSizes.x());
    myYNumber = (unsigned int) (ceil(mySize.y()) / myBoxSizes.y());
    myValues = new SUMOReal[myXNumber*myYNumber];
    for(int i=myXNumber*myYNumber; --i>=0; ) {
        myValues[i] = 0;
    }
}

MSGrid::~MSGrid() {
}


void 
MSGrid::writeXMLOutput(OutputDevice &dev,SUMOTime startTime, SUMOTime stopTime) throw(IOError) {
    dev.openTag("timestep") << " time=\"" << time2string(stopTime) << "\">\n";
    for(unsigned int y=0; y<myYNumber; ++y) {
        for(unsigned int x=0; x<myXNumber; ++x) {
            dev << myValues[x+y*myXNumber] << ' ';
            myValues[x+y*myXNumber] = 0;
        }
        dev << '\n';
    }
    dev.closeTag();
}


void 
MSGrid::writeXMLDetectorProlog(OutputDevice &dev) const throw(IOError) {
    dev.writeXMLHeader("detector");
}


void 
MSGrid::detectorUpdate(const SUMOTime t) throw() {
    if(t==21600*1000) {
        for(unsigned int y=0; y<myYNumber; ++y) {
            for(unsigned int x=0; x<myXNumber; ++x) {
                myValues[x+y*myXNumber] = 0;
            }
        }
    }
    const std::vector<MSEdge*> &edges = myEdges.getEdges();
    for (std::vector<MSEdge*>::const_iterator e=edges.begin(); e!=edges.end(); ++e) {
        const std::vector<MSLane*> &lanes = (*e)->getLanes();
        for (std::vector<MSLane*>::const_iterator lane=lanes.begin(); lane!=lanes.end(); ++lane) {
            const std::deque< MSVehicle* > &vehs = (*lane)->getVehiclesSecure();
            for (MSLane::VehCont::const_iterator veh = vehs.begin(); veh != vehs.end(); ++veh) {
                Position p = (*veh)->getPosition();
                if(p.x()<myOffset.x()||p.y()<myOffset.y()||p.x()>=mySize.x()+myOffset.x()||p.y()>=mySize.y()+myOffset.y()) {
                    continue;
                }
                unsigned int idxX = (unsigned int) ((p.x() - myOffset.x()) / myBoxSizes.x());
                unsigned int idxY = (unsigned int) ((p.y() - myOffset.y()) / myBoxSizes.y());
                if(idxX>=myXNumber||idxY>=myYNumber) {
                    int bla = 0;
                }
                myValues[idxX+idxY*myXNumber] += (*veh)->getHBEFA_CO2Emissions();
            }
            (*lane)->releaseVehicles();
        }
    }
}




/****************************************************************************/

