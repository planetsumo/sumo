/****************************************************************************/
/// @file    PHEMCEP.cpp
/// @author  Nikolaus Furian
/// @date    Thu, 13.06.2013
/// @version $$
///
// Helper class for PHEM Light, holds a specific CEP for a PHEM emission class
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

#include <string>
#include "StringBijection.h"
#include "PHEMCEP.h"
#include "UtilExceptions.h"

using std::string;

PHEMCEP::PHEMCEP(SUMOEmissionClass emissionClass,
		 double vehicleMass,
		 double vehicleLoading,
		 double vehicleMassRot,
		 double crossArea,
		 double cWValue,
		 double f0,
		 double f1,
		 double f2,
		 double f3,
		 double f4,
		 double ratedPower,
		 std::vector<std::string> headerLine,
		 std::vector< std::vector<double> > matrix)
{
	_emissionClass = emissionClass;
	_resistanceF0 = f0;
	_resistanceF1 = f1;
	_resistanceF2 = f2;
	_resistanceF3 = f3;
	_resistanceF4 = f4;
	_cdValue = cWValue;
	_crossSectionalArea = crossArea;
	_massVehicle = vehicleMass;
	_vehicleLoading = vehicleLoading;
	_massRot = vehicleMassRot;
	_ratedPower = ratedPower;

	_cepCurves = StringBijection<std::vector<double> >();

	std::vector<std::string> pollutantIdentifier;
	std::vector<std::vector<double> > pollutantMeasures;

	// get number pollutant Measures
	_numberPollutants = headerLine.size();

	// init pollutant identifiers
	for(int i=0; i<_numberPollutants; i++)
	{
		pollutantIdentifier.push_back(headerLine[i]);
	} // end for

	// get size of powerPattern
	_sizeOfPattern = matrix.size();

	// initialize measures
	for(int i=0; i<_numberPollutants; i++)
	{
		pollutantMeasures.push_back(std::vector<double>());
	} // end for


	// looping through matrix and assigning values
	for(int i=0; i<matrix.size(); i++)
	{
		for(int j=0; j<matrix[i].size(); j++)
		{
			if(matrix[i].size() != _numberPollutants + 1)
				return;

			if(j==0)
			{
				_powerPattern.push_back(matrix[i][j] * _ratedPower);
			}
			else
			{
				pollutantMeasures[j-1].push_back(matrix[i][j] * matrix[i][0] * _ratedPower);
			} // end if


		} // end for
	} // end for


	_cepCurves = StringBijection<std::vector<double> >();  

	for(int i=0; i<_numberPollutants; i++)
	{
		_cepCurves.insert(pollutantIdentifier[i], pollutantMeasures[i]);
	} // end for

} // end of Cep



PHEMCEP::~PHEMCEP(void)
{
	// free power pattern
	_powerPattern.clear();

} // end of ~Cep

double PHEMCEP::GetEmission(std::string pollutant, double power)
{
	if(!_cepCurves.hasString(pollutant))
		throw InvalidArgument("Emission pollutant " + pollutant + " not found!");

	std::vector<double> emissionCurve = _cepCurves.get(pollutant);

	if(emissionCurve.size()==0)
		throw InvalidArgument("Empty emission curve for " + pollutant + " found!");

	if(emissionCurve.size()==1)
		return emissionCurve[0];

	// in case that the demanded power is smaller than the first entry (smallest) in the power pattern the first two entries are extrapolated
	if(power <= _powerPattern.front())
	{ 
		double calcEmission =  PHEMCEP::Interpolate(power, _powerPattern[0], _powerPattern[1], emissionCurve[0], emissionCurve[1]);
	
		if(calcEmission < 0)
			return 0;
		else
			return calcEmission;
			
	} // end if

	// if power bigger than all entries in power pattern the last two values are linearly extrapolated
	if(power >= _powerPattern.back())
	{
		return PHEMCEP::Interpolate(power, _powerPattern[_powerPattern.size()-2], _powerPattern.back(), emissionCurve[emissionCurve.size()-2], emissionCurve.back());
	} // end if

	// bisection search to find correct position in power pattern	
	int middleIndex = (_powerPattern.size() - 1) / 2;
	int upperIndex = _powerPattern.size() - 1;
	int lowerIndex = 0;

	while(upperIndex - lowerIndex > 1)
	{
		if(_powerPattern[middleIndex] == power)
		{
			return emissionCurve[middleIndex];
		}
		else if (_powerPattern[middleIndex] < power)
		{
			lowerIndex = middleIndex;
			middleIndex = (upperIndex - lowerIndex) / 2 + lowerIndex;
		}
		else
		{
			upperIndex = middleIndex;
			middleIndex = (upperIndex - lowerIndex) / 2 + lowerIndex;
		} // end if
	} // end while

	if(_powerPattern[lowerIndex]<= power && power < _powerPattern[upperIndex])
		return PHEMCEP::Interpolate(power, _powerPattern[lowerIndex], _powerPattern[upperIndex], emissionCurve[lowerIndex], emissionCurve[upperIndex]);
	else
		throw ProcessError("Error during calculation of emission for power!");

} // end of GetEmission

double PHEMCEP::Interpolate(double px, double p1, double p2, double e1, double e2)
{
	if(p2==p1)
		return e1;

	return e1 + (px - p1)/(p2 - p1)*(e2 - e1);
} // end of Interpolate 

double PHEMCEP::GetResistanceF0()
{
	return PHEMCEP::_resistanceF0;
} // end of GetResistanceF0

double PHEMCEP::GetResistanceF1()
{
	return PHEMCEP::_resistanceF1;
} // end if GetResistanceF1

double PHEMCEP::GetResistanceF2()
{
	return PHEMCEP::_resistanceF2;
} // end of GetResistanceF2

double PHEMCEP::GetResistanceF3()
{
	return PHEMCEP::_resistanceF3;
} // end of GetResistanceF3

double PHEMCEP::GetResistanceF4()
{
	return PHEMCEP::_resistanceF4;
} // end of GetResistanceF4

double PHEMCEP::GetCdValue()
{
	return PHEMCEP::_cdValue;
} // end of GetCdValue

double PHEMCEP::GetCrossSectionalArea()
{
	return PHEMCEP::_crossSectionalArea;
} // end of GetCrossSectionalArea

double PHEMCEP::GetMassVehicle()
{
	return PHEMCEP::_massVehicle;
} // end of GetMassVehicle

double PHEMCEP::GetVehicleLoading()
{
	return PHEMCEP::_vehicleLoading;
} // end of GetMassVehicle

double PHEMCEP::GetMassRot()
{
	return PHEMCEP::_massRot;
} // end of GetMassRot

double PHEMCEP::GetRatedPower()
{
	return PHEMCEP::_ratedPower;
} // end of GetRatedPower

SUMOEmissionClass PHEMCEP::GetEmissionClass()
{
	return _emissionClass;
} // end of GetEmissionClass
