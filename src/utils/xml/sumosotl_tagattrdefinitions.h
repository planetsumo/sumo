/****************************************************************************/
/// @file    SUMOSOTL_TagAttrDefinitions.h
/// @author  Gianfilippo Slager
/// @date    Feb 2010
/// @version $Id: MSPhasedTrafficLightLogic.cpp 1 2010-02-21 18:07:00Z gslager $
///
// The container of tag and attributes for SUMO XML files relative to SOTL declarations
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2009 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef SUMOSOTL_TagAttrDefinitions_h
#define SUMOSOTL_TagAttrDefinitions_h

//These extend the XML phase declaration
#define SOTL_ATTR_TYPE "type"
#define SOTL_ATTR_TYPE_TRANSIENT "transient"
#define SOTL_ATTR_TYPE_DECISIONAL "decisional"
#define SOTL_ATTR_TYPE_TARGET "target"
#define SOTL_ATTR_TYPE_COMMIT "commit"
#define SOTL_ATTR_TARGETLANES "targetLanes"
#define SOTL_ATTR_TYPE_DELIMITERS " ,;"

#endif

/****************************************************************************/