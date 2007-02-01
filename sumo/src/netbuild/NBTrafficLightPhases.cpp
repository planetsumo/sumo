/****************************************************************************/
/// @file    NBTrafficLightPhases.cpp
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id: $
///
// A container for traffic light phases
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
// compiler pragmas
// ===========================================================================
#ifdef _MSC_VER
#pragma warning(disable: 4786)
#endif


// ===========================================================================
// included modules
// ===========================================================================
#ifdef WIN32
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cassert>
#include <vector>
#include <algorithm>
#include <iostream>
#include <bitset>
#include <utils/options/OptionsSubSys.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include "NBTrafficLightLogic.h"
#include "NBTrafficLightLogicVector.h"
#include "NBRequestEdgeLinkIterator.h"
#include "NBTrafficLightPhases.h"
#include "NBLinkCliqueContainer.h"

#ifdef _DEBUG
#include <utils/dev/debug_new.h>
#endif // _DEBUG


// ===========================================================================
// used namespaces
// ===========================================================================
using namespace std;


// ===========================================================================
// method definitions
// ===========================================================================
NBTrafficLightPhases::NBTrafficLightPhases(
    const NBLinkCliqueContainer &cliques, size_t noCliques)
        : _phasesVectorsByLength(noCliques, PhasesVector()),
        _cliques(cliques), _noPhaseVectors(0)
{}


NBTrafficLightPhases::~NBTrafficLightPhases()
{}


void
NBTrafficLightPhases::add(const PhaseIndexVector &phase)
{
    // check whether the given phasevector contains one of the already
    //  added phases in full
    //  do this for smaller vectors only
    size_t size = phase.size();
    size_t i;
    for (i=0; i<size&&i<_phasesVectorsByLength.size(); i++) {
        assert(i<_phasesVectorsByLength.size());
        PhasesVector::iterator j = find_if(
                                       _phasesVectorsByLength[i].begin(),
                                       _phasesVectorsByLength[i].end(),
                                       shorter_included_finder(phase));
        // shorter fully included by current was found;
        //  return without adding
        if (j!=_phasesVectorsByLength[i].end()) {
            return;
        }
    }
    // check whether the given phasevector is inside one of the larger
    //  already added phases
    for (i=size+1; i<_phasesVectorsByLength.size(); i++) {
        assert(i<_phasesVectorsByLength.size());
        PhasesVector::iterator j = find_if(
                                       _phasesVectorsByLength[i].begin(),
                                       _phasesVectorsByLength[i].end(),
                                       larger_included_finder(phase));
        if (j!=_phasesVectorsByLength[i].end()) {
            _phasesVectorsByLength[i].erase(j);
        }
    }
    // append empty vectors when needed
    while (_phasesVectorsByLength.size()<phase.size()+1)  {
        _phasesVectorsByLength.push_back(PhasesVector());
    }
    // add the current phase to the list
    assert(phase.size()<_phasesVectorsByLength.size());
    _phasesVectorsByLength[phase.size()].push_back(phase);
    _noPhaseVectors += 1;
}


void
NBTrafficLightPhases::add(const NBTrafficLightPhases &phases,
                              bool skipLarger)
{
    size_t size = _phasesVectorsByLength.size();
    size_t i;
    for (i=0; i<size&&i<phases._phasesVectorsByLength.size(); i++) {
        for (size_t j=0; j<phases._phasesVectorsByLength[i].size(); j++) {
            add(phases._phasesVectorsByLength[i][j]);
        }
    }
    if (skipLarger) {
        return;
    }
    for (; i<phases._phasesVectorsByLength.size(); i++) {
        for (size_t j=0; j<phases._phasesVectorsByLength[i].size(); j++) {
            add(phases._phasesVectorsByLength[i][j]);
        }
    }
}


std::ostream &operator<<(std::ostream &os, const NBTrafficLightPhases &p)
{
    os << "Folgen:" << endl;
    for (NBTrafficLightPhases::PhasesVectorVector::const_iterator
            i=p._phasesVectorsByLength.begin();
            i!=p._phasesVectorsByLength.end(); i++) {
        for (NBTrafficLightPhases::PhasesVector::const_iterator
                j=(*i).begin();
                j!=(*i).end();
                j++) {
            PhaseIndexVector tmp = (*j);
            for (PhaseIndexVector::const_iterator k=tmp.begin(); k!=tmp.end(); k++) {
                os << (*k) << ", ";
            }
            os << endl;
        }
    }
    os << "--------------------------------" << endl;
    return os;
}


NBTrafficLightLogicVector *
NBTrafficLightPhases::computeLogics(const std::string &key,
                                    std::string type,
                                    size_t noLinks,
                                    const NBRequestEdgeLinkIterator &cei1,
                                    const NBConnectionVector &inLinks,
                                    size_t breakingTime) const
{
    NBTrafficLightLogicVector *ret =
        new NBTrafficLightLogicVector(inLinks, type);
    for (size_t i=0; i<_phasesVectorsByLength.size(); i++) {
        for (size_t j=0; j<_phasesVectorsByLength[i].size(); j++) {
            ret->add(
                buildTrafficLightsLogic(
                    key, noLinks, _phasesVectorsByLength[i][j], cei1,
                    breakingTime));
        }
    }
    return ret;
}


NBTrafficLightLogic *
NBTrafficLightPhases::buildTrafficLightsLogic(const std::string &key,
        size_t noLinks,
        const PhaseIndexVector &phaseList,
        const NBRequestEdgeLinkIterator &cei1,
        size_t breakingTime) const
{
    NBTrafficLightLogic *ret =
        new NBTrafficLightLogic(key, noLinks);
    for (size_t i=0; i<phaseList.size(); i++) {
        // build and add the complete phase
        std::bitset<64> driveMask;
        std::bitset<64> brakeMask;
        size_t pos = 0;
        // go through the regarded links (in dependence whether
        //  left mover etc. were joined with te current)
        size_t j = 0;
        for (; j<cei1.getNoValidLinks(); j++) {
            // check how many real links are assigned to it
            size_t noEdges = cei1.getNumberOfAssignedLinks(j);
            // go through these links
            for (size_t k=0; k<noEdges; k++) {
                // set information for this link
                assert(i<phaseList.size());
                driveMask.set(pos, _cliques.test(phaseList[i], j));
                pos++;
            }
        }
        pos = 0;
        j = 0;
        for (; j<cei1.getNoValidLinks(); j++) {
            // check how many real links are assigned to it
            size_t noEdges = cei1.getNumberOfAssignedLinks(j);
            // go through these links
            for (size_t k=0; k<noEdges; k++) {
                // set information for this link
                assert(i<phaseList.size());
                if (driveMask.test(pos)) {
                    // the vehicle is allowed to drive, but may
                    //  have to brake due to a higher priorised
                    //  stream
                    brakeMask.set(pos,
                                  cei1.testBrakeMask(pos, driveMask));
                } else {
                    // the vehicle is not allowed to drive anyway
                    brakeMask.set(pos, true);
                }
                pos++;
            }
        }
        // add phase
        size_t duration = 31;
        if (OptionsSubSys::getOptions().isSet("traffic-light-green")) {
            duration = OptionsSubSys::getOptions().getInt("traffic-light-green");
        }
        ret->addStep(duration, driveMask, brakeMask, std::bitset<64>());
        // add possible additional left-turn phase when existing
        std::bitset<64> yellow = driveMask;
        std::bitset<64> oldBrakeMask = brakeMask;
        cei1.resetNonLeftMovers(driveMask, brakeMask, yellow);
        std::bitset<64> inv;
        inv.flip();
        if (driveMask.any()) {
            // let the junction be clear from any vehicles before alowing turn left
            //  left movers may drive but have to wait if another vehicle is passing
            if (breakingTime!=0) {
                ret->addStep(breakingTime, driveMask, oldBrakeMask|yellow, yellow);
            }
            // let vehicles moving left pass secure
            if (breakingTime!=0) { // !! something else
                ret->addStep(6, driveMask, brakeMask, std::bitset<64>());
            }
            yellow = driveMask;
        }
        // add the dead phase for this junction
        if (breakingTime!=0) {
            ret->addStep(breakingTime, std::bitset<64>(), inv, yellow);
        }
    }
#ifdef TL_DEBUG
    DEBUG_OUT << "Phasenfolge (Ende):" << endl;
    ret->_debugWritePhases();
    DEBUG_OUT << "----------------------------------" << endl;
#endif
    return ret;
}



/****************************************************************************/

