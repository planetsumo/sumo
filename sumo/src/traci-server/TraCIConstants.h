/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2007-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    TraCIConstants.h
/// @author  Axel Wegener
/// @author  Friedemann Wesner
/// @author  Bjoern Hendriks
/// @author  Daniel Krajzewicz
/// @author  Thimor Bohn
/// @author  Tino Morenz
/// @author  Michael Behrisch
/// @author  Christoph Sommer
/// @author  Mario Krumnow
/// @author  Jakob Erdmann
/// @author  Laura Bieker
/// @date    2007/10/24
/// @version $Id$
///
// holds codes used for TraCI
/****************************************************************************/
#ifndef TRACICONSTANTS_H
#define TRACICONSTANTS_H

// ****************************************
// VERSION
// ****************************************
#define TRACI_VERSION 16

// ****************************************
// COMMANDS
// ****************************************
// command: get version
#define CMD_GETVERSION 0x00

// command: load
#define CMD_LOAD 0x01

// command: simulation step
#define CMD_SIMSTEP 0x02

// command: set connection priority (execution order)
#define CMD_SETORDER 0x03

// command: stop node
#define CMD_STOP 0x12

// command: Resume from parking
#define CMD_RESUME 0x19

// command: set lane
#define CMD_CHANGELANE 0x13

// command: slow down
#define CMD_SLOWDOWN 0x14

// command: set sublane (vehicle)
#define CMD_CHANGESUBLANE 0x15

// command: change target
#define CMD_CHANGETARGET 0x31

// command: close sumo
#define CMD_CLOSE 0x7F


// command: subscribe induction loop (e1) context
#define CMD_SUBSCRIBE_INDUCTIONLOOP_CONTEXT 0x80
// response: subscribe induction loop (e1) context
#define RESPONSE_SUBSCRIBE_INDUCTIONLOOP_CONTEXT 0x90
// command: get induction loop (e1) variable
#define CMD_GET_INDUCTIONLOOP_VARIABLE 0xa0
// response: get induction loop (e1) variable
#define RESPONSE_GET_INDUCTIONLOOP_VARIABLE 0xb0
// command: subscribe induction loop (e1) variable
#define CMD_SUBSCRIBE_INDUCTIONLOOP_VARIABLE 0xd0
// response: subscribe induction loop (e1) variable
#define RESPONSE_SUBSCRIBE_INDUCTIONLOOP_VARIABLE 0xe0

// command: subscribe multi-entry/multi-exit detector (e3) context
#define CMD_SUBSCRIBE_MULTIENTRYEXIT_CONTEXT 0x81
// response: subscribe multi-entry/multi-exit detector (e3) context
#define RESPONSE_SUBSCRIBE_MULTIENTRYEXIT_CONTEXT 0x91
// command: get multi-entry/multi-exit detector (e3) variable
#define CMD_GET_MULTIENTRYEXIT_VARIABLE 0xa1
// response: get multi-entry/multi-exit detector (e3) variable
#define RESPONSE_GET_MULTIENTRYEXIT_VARIABLE 0xb1
// command: subscribe multi-entry/multi-exit detector (e3) variable
#define CMD_SUBSCRIBE_MULTIENTRYEXIT_VARIABLE 0xd1
// response: subscribe multi-entry/multi-exit detector (e3) variable
#define RESPONSE_SUBSCRIBE_MULTIENTRYEXIT_VARIABLE 0xe1

// command: subscribe traffic lights context
#define CMD_SUBSCRIBE_TL_CONTEXT 0x82
// response: subscribe traffic lights context
#define RESPONSE_SUBSCRIBE_TL_CONTEXT 0x92
// command: get traffic lights variable
#define CMD_GET_TL_VARIABLE 0xa2
// response: get traffic lights variable
#define RESPONSE_GET_TL_VARIABLE 0xb2
// command: set traffic lights variable
#define CMD_SET_TL_VARIABLE 0xc2
// command: subscribe traffic lights variable
#define CMD_SUBSCRIBE_TL_VARIABLE 0xd2
// response: subscribe traffic lights variable
#define RESPONSE_SUBSCRIBE_TL_VARIABLE 0xe2

// command: subscribe lane context
#define CMD_SUBSCRIBE_LANE_CONTEXT 0x83
// response: subscribe lane context
#define RESPONSE_SUBSCRIBE_LANE_CONTEXT 0x93
// command: get lane variable
#define CMD_GET_LANE_VARIABLE 0xa3
// response: get lane variable
#define RESPONSE_GET_LANE_VARIABLE 0xb3
// command: set lane variable
#define CMD_SET_LANE_VARIABLE 0xc3
// command: subscribe lane variable
#define CMD_SUBSCRIBE_LANE_VARIABLE 0xd3
// response: subscribe lane variable
#define RESPONSE_SUBSCRIBE_LANE_VARIABLE 0xe3

// command: subscribe vehicle context
#define CMD_SUBSCRIBE_VEHICLE_CONTEXT 0x84
// response: subscribe vehicle context
#define RESPONSE_SUBSCRIBE_VEHICLE_CONTEXT 0x94
// command: get vehicle variable
#define CMD_GET_VEHICLE_VARIABLE 0xa4
// response: get vehicle variable
#define RESPONSE_GET_VEHICLE_VARIABLE 0xb4
// command: set vehicle variable
#define CMD_SET_VEHICLE_VARIABLE 0xc4
// command: subscribe vehicle variable
#define CMD_SUBSCRIBE_VEHICLE_VARIABLE 0xd4
// response: subscribe vehicle variable
#define RESPONSE_SUBSCRIBE_VEHICLE_VARIABLE 0xe4

// command: subscribe vehicle type context
#define CMD_SUBSCRIBE_VEHICLETYPE_CONTEXT 0x85
// response: subscribe vehicle type context
#define RESPONSE_SUBSCRIBE_VEHICLETYPE_CONTEXT 0x95
// command: get vehicle type variable
#define CMD_GET_VEHICLETYPE_VARIABLE 0xa5
// response: get vehicle type variable
#define RESPONSE_GET_VEHICLETYPE_VARIABLE 0xb5
// command: set vehicle type variable
#define CMD_SET_VEHICLETYPE_VARIABLE 0xc5
// command: subscribe vehicle type variable
#define CMD_SUBSCRIBE_VEHICLETYPE_VARIABLE 0xd5
// response: subscribe vehicle type variable
#define RESPONSE_SUBSCRIBE_VEHICLETYPE_VARIABLE 0xe5

// command: subscribe route context
#define CMD_SUBSCRIBE_ROUTE_CONTEXT 0x86
// response: subscribe route context
#define RESPONSE_SUBSCRIBE_ROUTE_CONTEXT 0x96
// command: get route variable
#define CMD_GET_ROUTE_VARIABLE 0xa6
// response: get route variable
#define RESPONSE_GET_ROUTE_VARIABLE 0xb6
// command: set route variable
#define CMD_SET_ROUTE_VARIABLE 0xc6
// command: subscribe route variable
#define CMD_SUBSCRIBE_ROUTE_VARIABLE 0xd6
// response: subscribe route variable
#define RESPONSE_SUBSCRIBE_ROUTE_VARIABLE 0xe6

// command: subscribe poi context
#define CMD_SUBSCRIBE_POI_CONTEXT 0x87
// response: subscribe poi context
#define RESPONSE_SUBSCRIBE_POI_CONTEXT 0x97
// command: get poi variable
#define CMD_GET_POI_VARIABLE 0xa7
// response: get poi variable
#define RESPONSE_GET_POI_VARIABLE 0xb7
// command: set poi variable
#define CMD_SET_POI_VARIABLE 0xc7
// command: subscribe poi variable
#define CMD_SUBSCRIBE_POI_VARIABLE 0xd7
// response: subscribe poi variable
#define RESPONSE_SUBSCRIBE_POI_VARIABLE 0xe7

// command: subscribe polygon context
#define CMD_SUBSCRIBE_POLYGON_CONTEXT 0x88
// response: subscribe polygon context
#define RESPONSE_SUBSCRIBE_POLYGON_CONTEXT 0x98
// command: get polygon variable
#define CMD_GET_POLYGON_VARIABLE 0xa8
// response: get polygon variable
#define RESPONSE_GET_POLYGON_VARIABLE 0xb8
// command: set polygon variable
#define CMD_SET_POLYGON_VARIABLE 0xc8
// command: subscribe polygon variable
#define CMD_SUBSCRIBE_POLYGON_VARIABLE 0xd8
// response: subscribe polygon variable
#define RESPONSE_SUBSCRIBE_POLYGON_VARIABLE 0xe8

// command: subscribe junction context
#define CMD_SUBSCRIBE_JUNCTION_CONTEXT 0x89
// response: subscribe junction context
#define RESPONSE_SUBSCRIBE_JUNCTION_CONTEXT 0x99
// command: get junction variable
#define CMD_GET_JUNCTION_VARIABLE 0xa9
// response: get junction variable
#define RESPONSE_GET_JUNCTION_VARIABLE 0xb9
// command: set junction variable
#define CMD_SET_JUNCTION_VARIABLE 0xc9
// command: subscribe junction variable
#define CMD_SUBSCRIBE_JUNCTION_VARIABLE 0xd9
// response: subscribe junction variable
#define RESPONSE_SUBSCRIBE_JUNCTION_VARIABLE 0xe9

// command: subscribe edge context
#define CMD_SUBSCRIBE_EDGE_CONTEXT 0x8a
// response: subscribe edge context
#define RESPONSE_SUBSCRIBE_EDGE_CONTEXT 0x9a
// command: get edge variable
#define CMD_GET_EDGE_VARIABLE 0xaa
// response: get edge variable
#define RESPONSE_GET_EDGE_VARIABLE 0xba
// command: set edge variable
#define CMD_SET_EDGE_VARIABLE 0xca
// command: subscribe edge variable
#define CMD_SUBSCRIBE_EDGE_VARIABLE 0xda
// response: subscribe edge variable
#define RESPONSE_SUBSCRIBE_EDGE_VARIABLE 0xea

// command: subscribe simulation context
#define CMD_SUBSCRIBE_SIM_CONTEXT 0x8b
// response: subscribe simulation context
#define RESPONSE_SUBSCRIBE_SIM_CONTEXT 0x9b
// command: get simulation variable
#define CMD_GET_SIM_VARIABLE 0xab
// response: get simulation variable
#define RESPONSE_GET_SIM_VARIABLE 0xbb
// command: set simulation variable
#define CMD_SET_SIM_VARIABLE 0xcb
// command: subscribe simulation variable
#define CMD_SUBSCRIBE_SIM_VARIABLE 0xdb
// response: subscribe simulation variable
#define RESPONSE_SUBSCRIBE_SIM_VARIABLE 0xeb

// command: subscribe GUI context
#define CMD_SUBSCRIBE_GUI_CONTEXT 0x8c
// response: subscribe GUI context
#define RESPONSE_SUBSCRIBE_GUI_CONTEXT 0x9c
// command: get GUI variable
#define CMD_GET_GUI_VARIABLE 0xac
// response: get GUI variable
#define RESPONSE_GET_GUI_VARIABLE 0xbc
// command: set GUI variable
#define CMD_SET_GUI_VARIABLE 0xcc
// command: subscribe GUI variable
#define CMD_SUBSCRIBE_GUI_VARIABLE 0xdc
// response: subscribe GUI variable
#define RESPONSE_SUBSCRIBE_GUI_VARIABLE 0xec

// command: subscribe areal detector (e2) context
#define CMD_SUBSCRIBE_LANEAREA_CONTEXT 0x8d
// response: subscribe areal detector (e2) context
#define RESPONSE_SUBSCRIBE_LANEAREA_CONTEXT 0x9d
// command: get areal detector (e2) variable
#define CMD_GET_LANEAREA_VARIABLE 0xad
// response: get areal detector (e2) variable
#define RESPONSE_GET_LANEAREA_VARIABLE 0xbd
// command: subscribe areal detector (e2) variable
#define CMD_SUBSCRIBE_LANEAREA_VARIABLE 0xdd
// response: subscribe areal detector (e2) variable
#define RESPONSE_SUBSCRIBE_LANEAREA_VARIABLE 0xed

// command: subscribe person context
#define CMD_SUBSCRIBE_PERSON_CONTEXT 0x8e
// response: subscribe person context
#define RESPONSE_SUBSCRIBE_PERSON_CONTEXT 0x9e
// command: get person variable
#define CMD_GET_PERSON_VARIABLE 0xae
// response: get person variable
#define RESPONSE_GET_PERSON_VARIABLE 0xbe
// command: set person variable
#define CMD_SET_PERSON_VARIABLE 0xce
// command: subscribe person variable
#define CMD_SUBSCRIBE_PERSON_VARIABLE 0xde
// response: subscribe person variable
#define RESPONSE_SUBSCRIBE_PERSON_VARIABLE 0xee


// ****************************************
// POSITION REPRESENTATIONS
// ****************************************
// Position in geo-coordinates
#define POSITION_LON_LAT 0x00
// 2D cartesian coordinates
#define POSITION_2D 0x01
// Position in geo-coordinates with altitude
#define POSITION_LON_LAT_ALT 0x02
// 3D cartesian coordinates
#define POSITION_3D 0x03
// Position on road map
#define POSITION_ROADMAP 0x04


// ****************************************
// DATA TYPES
// ****************************************
// Boundary Box (4 doubles)
#define TYPE_BOUNDINGBOX 0x05
// Polygon (2*n doubles)
#define TYPE_POLYGON 0x06
// unsigned byte
#define TYPE_UBYTE 0x07
// signed byte
#define TYPE_BYTE 0x08
// 32 bit signed integer
#define TYPE_INTEGER 0x09
// float
#define TYPE_FLOAT 0x0A
// double
#define TYPE_DOUBLE 0x0B
// 8 bit ASCII string
#define TYPE_STRING 0x0C
// list of traffic light phases
#define TYPE_TLPHASELIST 0x0D
// list of strings
#define TYPE_STRINGLIST 0x0E
// compound object
#define TYPE_COMPOUND 0x0F
// color (four ubytes)
#define TYPE_COLOR 0x11


// ****************************************
// RESULT TYPES
// ****************************************
// result type: Ok
#define RTYPE_OK 0x00
// result type: not implemented
#define RTYPE_NOTIMPLEMENTED 0x01
// result type: error
#define RTYPE_ERR 0xFF

// return value for invalid queries (especially vehicle is not on the road)
#define INVALID_DOUBLE_VALUE -1001.
// return value for invalid queries (especially vehicle is not on the road)
#define INVALID_INT_VALUE -1
// maximum value for client ordering (2 ^ 30 - 1)
#define MAX_ORDER 1073741823

// ****************************************
// TRAFFIC LIGHT PHASES
// ****************************************
// red phase
#define TLPHASE_RED 0x01
// yellow phase
#define TLPHASE_YELLOW 0x02
// green phase
#define TLPHASE_GREEN 0x03
// tl is blinking
#define TLPHASE_BLINKING 0x04
// tl is off and not blinking
#define TLPHASE_NOSIGNAL 0x05


// ****************************************
// DIFFERENT DISTANCE REQUESTS
// ****************************************
// air distance
#define REQUEST_AIRDIST 0x00
// driving distance
#define REQUEST_DRIVINGDIST 0x01


// ****************************************
// VEHICLE REMOVAL REASONS
// ****************************************
// vehicle started teleport
#define REMOVE_TELEPORT 0x00
// vehicle removed while parking
#define REMOVE_PARKING 0x01
// vehicle arrived
#define REMOVE_ARRIVED 0x02
// vehicle was vaporized
#define REMOVE_VAPORIZED 0x03
// vehicle finished route during teleport
#define REMOVE_TELEPORT_ARRIVED 0x04

// ****************************************
// PERSON/CONTAINER STAGES
// ****************************************
// person / container stopping
#define STAGE_WAITING_FOR_DEPART 0x00
// person / container stopping
#define STAGE_WAITING 0x01
// person walking / container transhiping
#define STAGE_WALKING 0x02
// person riding / container being transported
#define STAGE_DRIVING 0x03

// ****************************************
// Stop Flags
// ****************************************
#define STOP_DEFAULT 0x00
#define STOP_PARKING 0x01
#define STOP_TRIGGERED 0x02
#define STOP_CONTAINER_TRIGGERED 0x04
#define STOP_BUS_STOP 0x08
#define STOP_CONTAINER_STOP 0x10
#define STOP_CHARGING_STATION 0x20
#define STOP_PARKING_AREA 0x40

// ****************************************
// Departure Flags
// ****************************************
#define DEPARTFLAG_TRIGGERED -0x01
#define DEPARTFLAG_CONTAINER_TRIGGERED -0x02
#define DEPARTFLAG_NOW -0x03

#define DEPARTFLAG_SPEED_RANDOM -0x02
#define DEPARTFLAG_SPEED_MAX -0x03

#define DEPARTFLAG_LANE_RANDOM -0x02
#define DEPARTFLAG_LANE_FREE -0x03
#define DEPARTFLAG_LANE_ALLOWED_FREE -0x04
#define DEPARTFLAG_LANE_BEST_FREE -0x05
#define DEPARTFLAG_LANE_FIRST_ALLOWED -0x06

// ****************************************
// Routing modes
// ****************************************
// use custom weights if available, fall back to loaded weights and then to free-flow speed
#define ROUTING_MODE_DEFAULT 0x00
// use aggregated travel times from device.rerouting
#define ROUTING_MODE_AGGREGATED 0x01

// ****************************************
// VARIABLE TYPES (for CMD_GET_*_VARIABLE)
// ****************************************
// list of instances' ids (get: all)
#define ID_LIST 0x00

// count of instances (get: all)
#define ID_COUNT 0x01

// subscribe object variables (get: all)
#define AUTOMATIC_VARIABLES_SUBSCRIPTION 0x02

// subscribe context variables (get: all)
#define AUTOMATIC_CONTEXT_SUBSCRIPTION 0x03

// generic attributes (get/set: all)
#define GENERIC_ATTRIBUTE 0x03

// last step vehicle number (get: induction loops, multi-entry/multi-exit detector, lanes, edges)
#define LAST_STEP_VEHICLE_NUMBER 0x10

// last step vehicle number (get: induction loops, multi-entry/multi-exit detector, lanes, edges)
#define LAST_STEP_MEAN_SPEED 0x11

// last step vehicle list (get: induction loops, multi-entry/multi-exit detector, lanes, edges)
#define LAST_STEP_VEHICLE_ID_LIST 0x12

// last step occupancy (get: induction loops, lanes, edges)
#define LAST_STEP_OCCUPANCY 0x13

// last step vehicle halting number (get: multi-entry/multi-exit detector, lanes, edges)
#define LAST_STEP_VEHICLE_HALTING_NUMBER 0x14

// last step mean vehicle length (get: induction loops, lanes, edges)
#define LAST_STEP_LENGTH 0x15

// last step time since last detection (get: induction loops)
#define LAST_STEP_TIME_SINCE_DETECTION 0x16

// entry times
#define LAST_STEP_VEHICLE_DATA 0x17

// last step jam length in vehicles
#define JAM_LENGTH_VEHICLE 0x18

// last step jam length in meters
#define JAM_LENGTH_METERS 0x19

// last step person list (get: edges)
#define LAST_STEP_PERSON_ID_LIST 0x1a


// traffic light states, encoded as rRgGyYoO tuple (get: traffic lights)
#define TL_RED_YELLOW_GREEN_STATE 0x20

// index of the phase (set: traffic lights)
#define TL_PHASE_INDEX 0x22

// traffic light program (set: traffic lights)
#define TL_PROGRAM 0x23

// phase duration (set: traffic lights)
#define TL_PHASE_DURATION 0x24

// controlled lanes (get: traffic lights)
#define TL_CONTROLLED_LANES 0x26

// controlled links (get: traffic lights)
#define TL_CONTROLLED_LINKS 0x27

// index of the current phase (get: traffic lights)
#define TL_CURRENT_PHASE 0x28

// name of the current program (get: traffic lights)
#define TL_CURRENT_PROGRAM 0x29

// controlled junctions (get: traffic lights)
#define TL_CONTROLLED_JUNCTIONS 0x2a

// complete definition (get: traffic lights)
#define TL_COMPLETE_DEFINITION_RYG 0x2b

// complete program (set: traffic lights)
#define TL_COMPLETE_PROGRAM_RYG 0x2c

// assumed time to next switch (get: traffic lights)
#define TL_NEXT_SWITCH 0x2d

// current state, using external signal names (get: traffic lights)
#define TL_EXTERNAL_STATE 0x2e

// outgoing link number (get: lanes)
#define LANE_LINK_NUMBER 0x30

// id of parent edge (get: lanes)
#define LANE_EDGE_ID 0x31

// outgoing link definitions (get: lanes)
#define LANE_LINKS 0x33

// list of allowed vehicle classes (get&set: lanes)
#define LANE_ALLOWED 0x34

// list of not allowed vehicle classes (get&set: lanes)
#define LANE_DISALLOWED 0x35

// slope (get: edge, lane, vehicle, person)
#define VAR_SLOPE 0x36

// speed (get: vehicle)
#define VAR_SPEED 0x40

// maximum allowed/possible speed (get: vehicle types, lanes, set: edges, lanes)
#define VAR_MAXSPEED 0x41

// position (2D) (get: vehicle, poi, inductionloop, areadetector; set: poi)
#define VAR_POSITION 0x42

// position (3D) (get: vehicle, poi, set: poi)
#define VAR_POSITION3D 0x39

// angle (get: vehicle)
#define VAR_ANGLE 0x43

// angle (get: vehicle types, lanes, arealdetector, set: lanes)
#define VAR_LENGTH 0x44

// color (get: vehicles, vehicle types, polygons, pois)
#define VAR_COLOR 0x45

// max. acceleration (get: vehicles, vehicle types)
#define VAR_ACCEL 0x46

// max. comfortable deceleration (get: vehicles, vehicle types)
#define VAR_DECEL 0x47

// max. (physically possible) deceleration (get: vehicles, vehicle types)
#define VAR_EMERGENCY_DECEL 0x7b

// apparent deceleration (get: vehicles, vehicle types)
#define VAR_APPARENT_DECEL 0x7c

// action step length (get: vehicles, vehicle types)
#define VAR_ACTIONSTEPLENGTH 0x7d

// driver's desired headway (get: vehicle types)
#define VAR_TAU 0x48

// vehicle class (get: vehicle types)
#define VAR_VEHICLECLASS 0x49

// emission class (get: vehicle types)
#define VAR_EMISSIONCLASS 0x4a

// shape class (get: vehicle types)
#define VAR_SHAPECLASS 0x4b

// minimum gap (get: vehicle types)
#define VAR_MINGAP 0x4c

// width (get: vehicle types, lanes)
#define VAR_WIDTH 0x4d

// shape (get: polygons)
#define VAR_SHAPE 0x4e

// type id (get: vehicles, polygons, pois)
#define VAR_TYPE 0x4f

// road id (get: vehicles)
#define VAR_ROAD_ID 0x50

// lane id (get: vehicles, inductionloop, arealdetector)
#define VAR_LANE_ID 0x51

// lane index (get: vehicles)
#define VAR_LANE_INDEX 0x52

// route id (get & set: vehicles)
#define VAR_ROUTE_ID 0x53

// edges (get: routes, vehicles)
#define VAR_EDGES 0x54

// filled? (get: polygons)
#define VAR_FILL 0x55

// position (1D along lane) (get: vehicle)
#define VAR_LANEPOSITION 0x56

// route (set: vehicles)
#define VAR_ROUTE 0x57

// travel time information (get&set: vehicle)
#define VAR_EDGE_TRAVELTIME 0x58

// effort information (get&set: vehicle)
#define VAR_EDGE_EFFORT 0x59

// last step travel time (get: edge, lane)
#define VAR_CURRENT_TRAVELTIME 0x5a

// signals state (get/set: vehicle)
#define VAR_SIGNALS 0x5b

// new lane/position along (set: vehicle)
#define VAR_MOVE_TO 0x5c

// driver imperfection (set: vehicle)
#define VAR_IMPERFECTION 0x5d

// speed factor (set: vehicle)
#define VAR_SPEED_FACTOR 0x5e

// speed deviation (set: vehicle)
#define VAR_SPEED_DEVIATION 0x5f

// routing mode (get/set: vehicle)
#define VAR_ROUTING_MODE 0x89

// speed without TraCI influence (get: vehicle)
#define VAR_SPEED_WITHOUT_TRACI 0xb1

// best lanes (get: vehicle)
#define VAR_BEST_LANES 0xb2

// how speed is set (set: vehicle)
#define VAR_SPEEDSETMODE 0xb3

// move vehicle, VTD version (set: vehicle)
#define MOVE_TO_XY 0xb4

// is the vehicle stopped, and if so parked and/or triggered?
// value = stopped + 2 * parking + 4 * triggered
#define VAR_STOPSTATE 0xb5

// how lane changing is performed (get/set: vehicle)
#define VAR_LANECHANGE_MODE 0xb6

// maximum speed regarding max speed on the current lane and speed factor (get: vehicle)
#define VAR_ALLOWED_SPEED 0xb7

// position (1D lateral position relative to center of the current lane) (get: vehicle)
#define VAR_LANEPOSITION_LAT 0xb8

// get/set prefered lateral alignment within the lane (vehicle)
#define VAR_LATALIGNMENT 0xb9

// get/set maximum lateral speed (vehicle, vtypes)
#define VAR_MAXSPEED_LAT 0xba

// get/set minimum lateral gap (vehicle, vtypes)
#define VAR_MINGAP_LAT 0xbb

// get/set vehicle height (vehicle, vtypes)
#define VAR_HEIGHT 0xbc

// get/set vehicle line
#define VAR_LINE 0xbd

// get/set vehicle via
#define VAR_VIA 0xbe

// current CO2 emission of a node (get: vehicle, lane, edge)
#define VAR_CO2EMISSION 0x60

// current CO emission of a node (get: vehicle, lane, edge)
#define VAR_COEMISSION 0x61

// current HC emission of a node (get: vehicle, lane, edge)
#define VAR_HCEMISSION 0x62

// current PMx emission of a node (get: vehicle, lane, edge)
#define VAR_PMXEMISSION 0x63

// current NOx emission of a node (get: vehicle, lane, edge)
#define VAR_NOXEMISSION 0x64

// current fuel consumption of a node (get: vehicle, lane, edge)
#define VAR_FUELCONSUMPTION 0x65

// current noise emission of a node (get: vehicle, lane, edge)
#define VAR_NOISEEMISSION 0x66

// current person number (get: vehicle)
#define VAR_PERSON_NUMBER 0x67

// number of persons waiting at a defined bus stop (get: simulation)
#define VAR_BUS_STOP_WAITING 0x67

// current leader together with gap (get: vehicle)
#define VAR_LEADER 0x68

// edge index in current route (get: vehicle)
#define VAR_ROUTE_INDEX 0x69

// current waiting time (get: vehicle, lane)
#define VAR_WAITING_TIME 0x7a

// current waiting time (get: vehicle)
#define VAR_ACCUMULATED_WAITING_TIME 0x87

// upcoming traffic lights (get: vehicle)
#define VAR_NEXT_TLS 0x70

// current time step (get: simulation)
#define VAR_TIME_STEP 0x70

// current electricity consumption of a node (get: vehicle, lane, edge)
#define VAR_ELECTRICITYCONSUMPTION 0x71

// number of loaded vehicles (get: simulation)
#define VAR_LOADED_VEHICLES_NUMBER 0x71

// loaded vehicle ids (get: simulation)
#define VAR_LOADED_VEHICLES_IDS 0x72

// number of departed vehicle (get: simulation)
#define VAR_DEPARTED_VEHICLES_NUMBER 0x73

// departed vehicle ids (get: simulation)
#define VAR_DEPARTED_VEHICLES_IDS 0x74

// number of vehicles starting to teleport (get: simulation)
#define VAR_TELEPORT_STARTING_VEHICLES_NUMBER 0x75

// ids of vehicles starting to teleport (get: simulation)
#define VAR_TELEPORT_STARTING_VEHICLES_IDS 0x76

// number of vehicles ending to teleport (get: simulation)
#define VAR_TELEPORT_ENDING_VEHICLES_NUMBER 0x77

// ids of vehicles ending to teleport (get: simulation)
#define VAR_TELEPORT_ENDING_VEHICLES_IDS 0x78

// number of arrived vehicles (get: simulation)
#define VAR_ARRIVED_VEHICLES_NUMBER 0x79

// ids of arrived vehicles (get: simulation)
#define VAR_ARRIVED_VEHICLES_IDS 0x7a

// delta t (get: simulation)
#define VAR_DELTA_T 0x7b

// bounding box (get: simulation)
#define VAR_NET_BOUNDING_BOX 0x7c

// minimum number of expected vehicles (get: simulation)
#define VAR_MIN_EXPECTED_VEHICLES 0x7d

// number of vehicles starting to park (get: simulation)
#define VAR_STOP_STARTING_VEHICLES_NUMBER 0x68

// ids of vehicles starting to park (get: simulation)
#define VAR_STOP_STARTING_VEHICLES_IDS 0x69

// number of vehicles ending to park (get: simulation)
#define VAR_STOP_ENDING_VEHICLES_NUMBER 0x6a

// ids of vehicles ending to park (get: simulation)
#define VAR_STOP_ENDING_VEHICLES_IDS 0x6b

// number of vehicles starting to park (get: simulation)
#define VAR_PARKING_STARTING_VEHICLES_NUMBER 0x6c

// ids of vehicles starting to park (get: simulation)
#define VAR_PARKING_STARTING_VEHICLES_IDS 0x6d

// number of vehicles ending to park (get: simulation)
#define VAR_PARKING_ENDING_VEHICLES_NUMBER 0x6e

// ids of vehicles ending to park (get: simulation)
#define VAR_PARKING_ENDING_VEHICLES_IDS 0x6f

// clears the simulation of all not inserted vehicles (set: simulation)
#define CMD_CLEAR_PENDING_VEHICLES 0x94

// triggers saving simulation state (set: simulation)
#define CMD_SAVE_SIMSTATE 0x95

// sets/retrieves abstract parameter
#define VAR_PARAMETER 0x7e


// add an instance (poi, polygon, vehicle, person, route)
#define ADD 0x80

// remove an instance (poi, polygon, vehicle, person)
#define REMOVE 0x81

// copy an instance (vehicle type, other TBD.)
#define COPY 0x88

// convert coordinates
#define POSITION_CONVERSION 0x82

// distance between points or vehicles
#define DISTANCE_REQUEST 0x83

// the current driving distance
#define VAR_DISTANCE 0x84

// add a fully specified instance (vehicle)
#define ADD_FULL 0x85

// force rerouting based on travel time (vehicles)
#define CMD_REROUTE_TRAVELTIME 0x90

// force rerouting based on effort (vehicles)
#define CMD_REROUTE_EFFORT 0x91

// validates current route (vehicles)
#define VAR_ROUTE_VALID 0x92

// retrieve information regarding the current person/container stage
#define VAR_STAGE 0xc0

// retrieve information regarding the next edge including crossings and walkingAreas (pedestrians only)
#define VAR_NEXT_EDGE 0xc1

// retrieve information regarding the number of remaining stages
#define VAR_STAGES_REMAINING 0xc2

// retrieve the current vehicle id for the driving stage (person, container)
#define VAR_VEHICLE 0xc3

// append a person stage (person)
#define APPEND_STAGE 0xc4

// append a person stage (person)
#define REMOVE_STAGE 0xc5

// zoom
#define VAR_VIEW_ZOOM 0xa0

// view position
#define VAR_VIEW_OFFSET 0xa1

// view schema
#define VAR_VIEW_SCHEMA 0xa2

// view by boundary
#define VAR_VIEW_BOUNDARY 0xa3

// screenshot
#define VAR_SCREENSHOT 0xa5

// track vehicle
#define VAR_TRACK_VEHICLE 0xa6


#endif
