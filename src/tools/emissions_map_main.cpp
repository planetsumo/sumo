/****************************************************************************/
/// @file    emissionsMap_main.cpp
/// @author  Daniel Krajzewicz
/// @date    Wed, 21.08.2013
/// @version $Id$
///
// Main for an emissions map writer
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
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

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include <utils/common/TplConvert.h>
#include <iostream>
#include <string>
#include <ctime>
#include <utils/common/MsgHandler.h>
#include <utils/options/Option.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/OptionsIO.h>
#include <utils/common/UtilExceptions.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/common/SystemFrame.h>
#include <utils/common/ToString.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/common/FileHelpers.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// functions
// ===========================================================================


/* -------------------------------------------------------------------------
 * main
 * ----------------------------------------------------------------------- */
int
main(int argc, char **argv) {
    // build options
    OptionsCont &oc = OptionsCont::getOptions();
    //  give some application descriptions
    oc.setApplicationDescription("Builds and writes an emissions map.");
    oc.setApplicationName("emissionsMap", "SUMO emissionsMap Version " + (std::string)VERSION_STRING);
    //  add options
    oc.addOptionSubTopic("Emissions");
    oc.doRegister("phemlight-path", new Option_FileName("./PHEMlight/"));
    oc.addDescription("phemlight-path", "Emissions", "Determines where to load PHEMlight definitions from.");
    // run
    int ret = 0;
    try {
        // initialise the application system (messaging, xml, options)
        XMLSubSys::init();
        OptionsIO::getOptions(true, argc, argv);

		SUMOReal vmin = 0.;
		SUMOReal vmax = 50.;
		SUMOReal vstep = 2.;
		SUMOReal amin = -5.;
		SUMOReal amax = 3.;
		SUMOReal astep = .2;
		SUMOReal smin = -10.;
		SUMOReal smax = 10.;
		SUMOReal sstep = 1;
		SUMOEmissionClass c = SVE_LSZ_D_EU3;
		std::ofstream o("D:\\SVE_LSZ_D_EU3.txt");
		for(SUMOReal v=vmin; v<vmax; v+=vstep) {
			for(SUMOReal a=amin; a<amax; a+=astep) {
    			for(SUMOReal s=smin; s<smax; s+=sstep) {
                    o << v << ";" << a << ";" << s << ";" << "CO" << ";" << PollutantsInterface::computeCO(c, v, a, s) << std::endl;
                    o << v << ";" << a << ";" << s << ";" << "CO2" << ";" << PollutantsInterface::computeCO2(c, v, a, s) << std::endl;
                    o << v << ";" << a << ";" << s << ";" << "HC" << ";" << PollutantsInterface::computeHC(c, v, a, s) << std::endl;
                    o << v << ";" << a << ";" << s << ";" << "PMx" << ";" << PollutantsInterface::computePMx(c, v, a, s) << std::endl;
                    o << v << ";" << a << ";" << s << ";" << "NOx" << ";" << PollutantsInterface::computeNOx(c, v, a, s) << std::endl;
                    o << v << ";" << a << ";" << s << ";" << "fuel" << ";" << PollutantsInterface::computeFuel(c, v, a, s) << std::endl;
                }
			}
		}

    } catch (ProcessError &e) {
        if (std::string(e.what())!=std::string("Process Error") && std::string(e.what())!=std::string("")) {
            MsgHandler::getErrorInstance()->inform(e.what());
        }
        MsgHandler::getErrorInstance()->inform("Quitting (on error).", false);
        ret = 1;
#ifndef _DEBUG
    } catch (...) {
        MsgHandler::getErrorInstance()->inform("Quitting (on unknown error).", false);
        ret = 1;
#endif
    }
    SystemFrame::close();
    if (ret==0) {
        std::cout << "Success." << std::endl;
    }
    return ret;
}



/****************************************************************************/

