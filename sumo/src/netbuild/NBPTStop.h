/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    NBPTStop.h
/// @author  Gregor Laemmel
/// @date    Tue, 20 Mar 2017
/// @version $Id$
///
// The representation of a single pt stop
/****************************************************************************/
#ifndef SUMO_NBPTSTOP_H
#define SUMO_NBPTSTOP_H

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/geom/Position.h>
#include "utils/common/SUMOVehicleClass.h"
#include "NBPTPlatform.h"


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class NBEdgeCont;


// ===========================================================================
// class definitions
// ===========================================================================
/**
* @class NBPTStop
* @brief The representation of a single pt stop
*/
class NBPTStop {

public:
    /**@brief Constructor
    * @param[in] id The id of the pt stop
    * @param[in] position The position of the pt stop
    * @param[in] edgeId The edge id of the pt stop
    * @param[in] length The length of the pt stop
    */
    NBPTStop(std::string ptStopId, Position position, std::string edgeId, std::string origEdgeId, double length, std::string name, SVCPermissions svcPermissions);
    std::string getID() const;

    const std::string getEdgeId();
    const std::string getOrigEdgeId();
    const std::string getName();
    const Position& getPosition();
    SVCPermissions getPermissions();
    void computExtent(double center, double d);
    void write(OutputDevice& device);
    void reshiftPostion(const double offsetX, const double offsetY);

    std::vector<NBPTPlatform>& getPlatformCands();
    bool getIsMultipleStopPositions();
    void setIsMultipleStopPositions(bool multipleStopPositions);
    double getLength();
    bool setEdgeId(std::string edgeId, NBEdgeCont& ec);
    void registerAdditionalEdge(std::string wayId, std::string edgeId);
    void addPlatformCand(NBPTPlatform platform);
    bool findLaneAndComputeBusStopExtend(NBEdgeCont& ec);

private:
    const std::string myPTStopId;
    Position myPosition;
    std::string myEdgeId;
    std::map<std::string, std::string> myAdditionalEdgeCandidates;
public:
    const std::map<std::string, std::string>& getMyAdditionalEdgeCandidates() const;
private:
    std::string myOrigEdgeId;
public:
    void setMyOrigEdgeId(const std::string& myOrigEdgeId);
private:
    double myPTStopLength;
public:
    void setMyPTStopLength(double myPTStopLength);
private:
    const std::string myName;
    std::string myLaneId;
    const SVCPermissions myPermissions;

    double myStartPos;
    double myEndPos;


private:
    /// @brief Invalidated assignment operator.
    NBPTStop& operator=(const NBPTStop&);


    std::vector<NBPTPlatform> myPlatformCands;
    bool myIsMultipleStopPositions;
};

#endif //SUMO_NBPTSTOP_H
