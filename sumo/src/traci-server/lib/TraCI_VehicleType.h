/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2017-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    TraCI_VehicleType.h
/// @author  Gregor Laemmel
/// @date    04.04.2017
/// @version $Id$
///
// C++ TraCI client API implementation
/****************************************************************************/

#ifndef SUMO_TRACI_VEHICLETYPE_H
#define SUMO_TRACI_VEHICLETYPE_H


#include <string>
#include <traci-server/TraCIDefs.h>
#include <microsim/MSVehicleType.h>
class TraCI_VehicleType {

public:

    static std::vector<std::string> getIDList();
    static double getLength(const std::string& typeID);
    static double getMaxSpeed(const std::string& typeID);
    static double getActionStepLength(const std::string& typeID);
    static double getSpeedFactor(const std::string& typeID);
    static double getSpeedDeviation(const std::string& typeID);
    static double getAccel(const std::string& typeID);
    static double getDecel(const std::string& typeID);
    static double getEmergencyDecel(const std::string& typeID);
    static double getApparentDecel(const std::string& typeID);
    static double getImperfection(const std::string& typeID);
    static double getTau(const std::string& typeID);
    static std::string getVehicleClass(const std::string& typeID);
    static std::string getEmissionClass(const std::string& typeID);
    static std::string getShapeClass(const std::string& typeID);
    static double getMinGap(const std::string& typeID);
    static double getWidth(const std::string& typeID);
    static double getHeight(const std::string& typeID);
    static TraCIColor getColor(const std::string& typeID);
    static double getMinGapLat(const std::string& typeID);
    static double getMaxSpeedLat(const std::string& typeID);
    static std::string getLateralAlignment(const std::string& typeID);
    static std::string getParameter(const std::string& typeID, const std::string& key);

    static void setLength(const std::string& typeID, double length);
    static void setMaxSpeed(const std::string& typeID, double speed);
    static void setActionStepLength(const std::string& typeID, double actionStepLength, bool resetActionOffset);
    static void setVehicleClass(const std::string& typeID, const std::string& clazz);
    static void setSpeedFactor(const std::string& typeID, double factor);
    static void setSpeedDeviation(const std::string& typeID, double deviation);
    static void setEmissionClass(const std::string& typeID, const std::string& clazz);
    static void setShapeClass(const std::string& typeID, const std::string& shapeClass);
    static void setWidth(const std::string& typeID, double width);
    static void setHeight(const std::string& typeID, double height);
    static void setMinGap(const std::string& typeID, double minGap);
    static void setAccel(const std::string& typeID, double accel);
    static void setDecel(const std::string& typeID, double decel);
    static void setEmergencyDecel(const std::string& typeID, double decel);
    static void setApparentDecel(const std::string& typeID, double decel);
    static void setImperfection(const std::string& typeID, double imperfection);
    static void setTau(const std::string& typeID, double tau);
    static void setColor(const std::string& typeID, const TraCIColor& c);
    static void setMinGapLat(const std::string& typeID, double minGapLat);
    static void setMaxSpeedLat(const std::string& typeID, double speed);
    static void setLateralAlignment(const std::string& typeID, const std::string& latAlignment);
    static void copy(const std::string& origTypeID, const std::string& newTypeID);
    static MSVehicleType* getVType(std::string id);

    static void setParameter(const std::string& id, const std::string& name, const std::string& value);

private:
    /// @brief invalidated standard constructor
    TraCI_VehicleType();

    /// @brief invalidated copy constructor
    TraCI_VehicleType(const TraCI_VehicleType& src);

    /// @brief invalidated assignment operator
    TraCI_VehicleType& operator=(const TraCI_VehicleType& src);

};


#endif //SUMO_TRACI_VEHICLETYPE_H
