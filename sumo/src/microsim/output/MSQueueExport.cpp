/****************************************************************************/
/// @file    MSQueueExport.cpp
/// @author  Mario Krumnow
/// @version $Id$
///
// Export the queueing length in front of a junction (very experimental!)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2013 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
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

#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSGlobals.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSQueueExport.h"
#include <microsim/MSNet.h>
#include <microsim/MSVehicle.h>

#ifdef HAVE_MESOSIM
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
void
MSQueueExport::write(OutputDevice& of, SUMOTime timestep) {

    of.openTag("data") << " timestep=\"" << time2string(timestep) << "\">\n";

    writeEdge(of);

    of.closeTag();
}

void
MSQueueExport::writeEdge(OutputDevice& of) {

    of.openTag("lanes") <<  ">\n";

    MSEdgeControl& ec = MSNet::getInstance()->getEdgeControl();

    //const std::vector<MSEdge*>& edges = ec.getEdges();
    MSEdge** edges = ec.getEdges();
    unsigned int esize = 0;
    PREBSIZE(edges,esize);
    //for (std::vector<MSEdge*>::const_iterator e = edges.begin(); e != edges.end(); ++e) {
    for(int i=0;i<esize;++i){
        MSEdge& edge = **(edges+i);
        /* speed fix */
        MSLane** lanes = edge.getLanes();
        unsigned int sizeLanes = 0;
        PREBSIZE(lanes,sizeLanes);
        for (int i=0;i<sizeLanes;i++) {
            writeLane(of, *(*(lanes+i)));
        }

    }

    of.closeTag();

}


void
MSQueueExport::writeLane(OutputDevice& of, const MSLane& lane) {

    //Fahrzeug mit der höchsten Wartezeit
    //Fahrzeug am Ende des Rückstaus

    double queueing_time = 0.0;
    double queueing_length = 0.0;
    double queueing_length2 = 0.0;


    if (lane.getVehicleNumber() != 0) {

        for (std::vector<MSVehicle*>::const_iterator veh = lane.myVehBuffer.begin(); veh != lane.myVehBuffer.end(); ++veh) {

            const MSVehicle& veh_tmp = **veh;
            if (veh_tmp.isOnRoad()) {

                if (veh_tmp.getWaitingSeconds() > 0) {

                    if (veh_tmp.getWaitingSeconds() > queueing_time) {
                        queueing_time = veh_tmp.getWaitingSeconds();
                    }

                    double tmp_length = (lane.getLength() -  veh_tmp.getPositionOnLane()) + veh_tmp.getVehicleType().getLengthWithGap();

                    if (tmp_length > queueing_length) {
                        queueing_length = tmp_length;
                    }


                }

            }

        }


        for (MSLane::VehCont::const_iterator veh = lane.myVehicles.begin(); veh != lane.myVehicles.end(); ++veh) {

            const MSVehicle& veh_tmp = **veh;
            if (veh_tmp.isOnRoad()) {

                if (veh_tmp.getWaitingSeconds() > 0) {

                    if (veh_tmp.getWaitingSeconds() > queueing_time) {
                        queueing_time = veh_tmp.getWaitingSeconds();
                    }

                    double tmp_length = (lane.getLength() - veh_tmp.getPositionOnLane()) + veh_tmp.getVehicleType().getLengthWithGap();

                    if (tmp_length > queueing_length) {
                        queueing_length = tmp_length;
                    }


                }

            }
        }


        //Experimental
        double tmp_length2 = 0.0;

        for (MSLane::VehCont::const_iterator veh = lane.myVehicles.begin(); veh != lane.myVehicles.end(); ++veh) {

            //wenn Fahrzeug langsamer als 5 km/h fährt = Rückstau
            double threshold_velocity = 5 / 3.6;
            const MSVehicle& veh_tmp = **veh;
            if (veh_tmp.isOnRoad()) {

                if (veh_tmp.getSpeed() < (threshold_velocity) && (veh_tmp.getPositionOnLane() > (veh_tmp.getLane()->getLength()) * 0.25))

                {
                    tmp_length2 = (lane.getLength() - veh_tmp.getPositionOnLane()) + veh_tmp.getVehicleType().getLengthWithGap();
                }
                if (tmp_length2 > queueing_length2) {
                    queueing_length2 = tmp_length2;
                }

            }
        }

    }

    //Output
    if (queueing_length > 1 || queueing_length2 > 1) {
        of.openTag("lane") << " id=\"" << lane.getID() << "\"";
        of << " queueing_time=\"" << queueing_time << "\" queueing_length=\"" << queueing_length << "\" queueing_length_experimental=\"" << queueing_length2 << "\"";
        of.closeTag();
    }

}

/****************************************************************************/
