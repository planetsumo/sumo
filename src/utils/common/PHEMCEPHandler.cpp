/****************************************************************************/
/// @file    PHEMCEPHandler.cpp
/// @author  Nikolaus Furian
/// @date    Thu, 13.06.2013
/// @version $$
///
// Helper class for PHEM Light, holds CEP data for emission computation
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

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "UtilExceptions.h"
#include "PHEMCEPHandler.h"
#include "PHEMConstants.h"
#include <utils/options/OptionsCont.h>

// ===========================================================================
// method definitions
// ===========================================================================

PHEMCEPHandler::PHEMCEPHandler() {
}

PHEMCEPHandler::~PHEMCEPHandler() {
	std::map<SUMOEmissionClass,PHEMCEP*>::iterator iter = _ceps.begin();
	while (iter != _ceps.end()) {
		delete(iter->second);
		iter++;
	} // end while
	_ceps.clear();
}


PHEMCEPHandler& 
PHEMCEPHandler::getHandlerInstance() {
	static PHEMCEPHandler instance;
	return instance;
}


bool 
PHEMCEPHandler::Load(SUMOEmissionClass emissionClass) {	
	// get string identifier for PHEM emission class
	std::string emissionClassIdentifier = SumoEmissionClassStrings.getString(emissionClass);

	// to hold everything.
	std::vector< std::vector<double> > matrix; 
	std::vector<std::string> header;

	double vehicleMass;
	double vehicleLoading;
	double vehicleMassRot;
	double crosssectionalArea;
	double cwValue;
	double f0;
	double f1;
	double f2;
	double f3;
	double f4;
	double ratedPower;

    OptionsCont &oc = OptionsCont::getOptions();
    std::string phemPath = oc.getString("phemlight-path") + "/";
	if(!ReadVehicleFile(phemPath, emissionClassIdentifier, vehicleMass, vehicleLoading, vehicleMassRot, crosssectionalArea, cwValue, f0, f1, f2, f3, f4, ratedPower))
		return false;

	if(!ReadEmissionData(phemPath, emissionClassIdentifier, header, matrix))
		return false;

	_ceps[emissionClass] = new PHEMCEP(emissionClass, 
									   vehicleMass,
									   vehicleLoading,
									   vehicleMassRot,
									   crosssectionalArea,
									   cwValue,
									   f0,
									   f1,
									   f2,
									   f3,
									   f4,
									   ratedPower, 
									   header,
									   matrix);

	return true;
} // end of Load()


PHEMCEP* 
PHEMCEPHandler::GetCep(SUMOEmissionClass emissionClass) {
	// check if Cep has been loaded
	if(_ceps.find(emissionClass) == _ceps.end()) {
		if(!PHEMCEPHandler::Load(emissionClass))
			throw InvalidArgument("File for PHEM emission class " + SumoEmissionClassStrings.getString(emissionClass) + " not found.");
	} // end if

	return _ceps[emissionClass];
	
} // end of GetCep


bool 
PHEMCEPHandler::ReadVehicleFile(const std::string &path, const std::string &emissionClass,
                                double &vehicleMass, double &vehicleLoading, double &vehicleMassRot,
                                double &crossArea, double &cWValue,
                                double &f0, double &f1, double &f2, double &f3, double &f4, double &ratedPower) {
    std::ifstream fileVehicle (path + emissionClass + ".veh");

	if(!fileVehicle.good())
		return false;

	std::string line;
	std::string cell;
	std::string commentPrefix = "c";
	int dataCount = 0;

	// skip header
	std::getline(fileVehicle,line);

	while(std::getline(fileVehicle,line) && dataCount<=18)
    {
        std::stringstream  lineStream(line);

		if(line.substr(0,1) == commentPrefix)
			continue;
		else
			dataCount++;

		std::getline(lineStream,cell,',');

		// reading Mass
		if(dataCount==1)
			std::istringstream(cell) >> vehicleMass;

		// reading vehicle loading
		if(dataCount==2)
			std::istringstream(cell) >> vehicleLoading;

		// reading cWValue
		if(dataCount==3)
			std::istringstream(cell) >> cWValue;

		// reading crossectional area
		if(dataCount==4)
			std::istringstream(cell) >> crossArea;

		// reading vehicle mass rotational
		if(dataCount==7)
			std::istringstream(cell) >> vehicleMassRot;

		// reading rated power
		if(dataCount==10)
			std::istringstream(cell) >> ratedPower;

		// reading f0
		if(dataCount==14)
			std::istringstream(cell) >> f0;

		// reading f1
		if(dataCount==15)
			std::istringstream(cell) >> f1;

		// reading f2
		if(dataCount==16)
			std::istringstream(cell) >> f2;

		// reading f3
		if(dataCount==17)
			std::istringstream(cell) >> f3;

		// reading f4
		if(dataCount==18)
			std::istringstream(cell) >> f4;

    } // end while

	fileVehicle.close();

	return true;

} // end of ReadVehicleFile


bool 
PHEMCEPHandler::ReadEmissionData(const std::string &path, const std::string &emissionClass, 
                                 std::vector<std::string> &header, std::vector<std::vector<double> > &matrix) {
	// declare file stream
    std::ifstream fileEmission (path + emissionClass + ".csv");

	if(!fileEmission.good())
		return false;

	
	std::string line;
	std::string cell;

	// read header line for pollutant identifiers
	if(std::getline(fileEmission,line))
	{
		std::stringstream  lineStream(line);

		// skip first entry "Pe"
		std::getline(lineStream,cell,',');

		while(std::getline(lineStream,cell,','))
        {
			header.push_back(cell);
		} // end while

	} // end if

	// skip units
	std::getline(fileEmission,line);

	while(std::getline(fileEmission,line))
    {
        std::stringstream  lineStream(line);
        std::string cell;
		std::vector <double> vi;
        while(std::getline(lineStream,cell,','))
        {
			double entry;
			std::istringstream(cell) >> entry;
			vi.push_back(entry);

        } // end while
		matrix.push_back(vi);
    } // end while

	fileEmission.close();

	return true;
} // end of ReadEmissionData


/****************************************************************************/
