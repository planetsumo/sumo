/****************************************************************************/
/// @file    MSEdgeContinuations.cpp
/// @author  Daniel Krajzewicz
/// @date    2005-11-09
/// @version $Id$
///
// »missingDescription«
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// copyright : (C) 2001-2007
//  by DLR (http://www.dlr.de/) and ZAIK (http://www.zaik.uni-koeln.de/AFS)
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

#include <map>
#include <vector>
#include "MSEdgeContinuations.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// used namespaces
// ===========================================================================

using namespace std;

MSEdgeContinuations::MSEdgeContinuations()
{}


MSEdgeContinuations::~MSEdgeContinuations()
{}


void
MSEdgeContinuations::add(MSEdge *to, MSEdge *from)
{
    if (myContinuations.find(to)==myContinuations.end()) {
        myContinuations[to] = vector<MSEdge*>();
    }
    myContinuations[to].push_back(from);
}


const std::vector<MSEdge*> &
MSEdgeContinuations::getInFrontOfEdge(const MSEdge &toEdge) const
{
    return myContinuations.find(static_cast<MSEdge*>(& ((MSEdge&) toEdge)))->second;
}


bool
MSEdgeContinuations::hasFurther(const MSEdge &toEdge) const
{
    return myContinuations.find(static_cast<MSEdge*>(& ((MSEdge&) toEdge)))!=myContinuations.end();
}



/****************************************************************************/

