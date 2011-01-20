/****************************************************************************/
/// @file    dfrouter_main.cpp
/// @author  Daniel Krajzewicz
/// @date    Thu, 16.03.2006
/// @version $Id: dfrouter_main.cpp 8743 2010-05-07 14:54:01Z bieker $
///
// Main for the DFROUTER
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2010 DLR (http://www.dlr.de/) and contributors
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

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include <xercesc/sax/SAXException.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <utils/common/TplConvert.h>
#include <iostream>
#include <string>
#include <limits.h>
#include <ctime>
#include <router/ROFrame.h>
#include <utils/common/MsgHandler.h>
#include <utils/options/Option.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/OptionsIO.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/HelpersHBEFA.h>
#include <utils/common/SystemFrame.h>
#include <utils/common/ToString.h>
#include <utils/xml/XMLSubSys.h>
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
    OptionsCont &oc = OptionsCont::getOptions();
    // give some application descriptions
    oc.setApplicationDescription("Builds vehicle routes for SUMO using detector values.");
    oc.setApplicationName("dfrouter", "SUMO dfrouter Version " + (std::string)VERSION_STRING);
    int ret = 0;
    try {
        // initialise the application system (messaging, xml, options)
        XMLSubSys::init(false);
//        RODFFrame::fillOptions();
        OptionsIO::getOptions(true, argc, argv);
		/*
        if (oc.processMetaOptions(argc < 2)) {
            SystemFrame::close();
            return 0;
        }
		*/

		SUMOReal vmin = 0.;
		SUMOReal vmax = 56.;
		SUMOReal vstep = 2.;
		SUMOReal amin = -1.;
		SUMOReal amax = 5.;
		SUMOReal astep = .2;
		SUMOEmissionClass c = SVE_UNKNOWN;
		std::ofstream o("D:\\peter\\nox.txt");
		for(SUMOReal v=vmin; v<vmax; v+=vstep) {
			for(SUMOReal a=amin; a<amax; a+=astep) {
				if(a!=amin) {
					o << ";";
				}
				SUMOReal value = HelpersHBEFA::computeNOx(c, v, a);
				o << value;
			}
			o << std::endl;
		}

//        MsgHandler::initOutputOptions();
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
    OutputDevice::closeAll();
    SystemFrame::close();
    if (ret==0) {
        std::cout << "Success." << std::endl;
    }
    return ret;
}



/****************************************************************************/

