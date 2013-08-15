/****************************************************************************/
/// @file    PHEMCEP.h
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
#ifndef PHEMCEP_h
#define PHEMCEP_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include "PHEMCEP.h"
#include "SUMOVehicleClass.h"
#include "StringBijection.h"

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class PHEMCEP
 * @brief Data Handler for a single CEP emission data set
 */
class PHEMCEP
{	

public:
	PHEMCEP(SUMOEmissionClass emissionClass,
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
		     std::vector< std::vector<double> > matrix);

	PHEMCEP() {};

	~PHEMCEP(void);

	/** @brief Getter function to recieve vehicle data from CEP
     * @return PHEM emission class of vehicle
     */
	SUMOEmissionClass GetEmissionClass();

	/** @brief Returns a emission measure for power[kW] level
	 * @param[in] pollutantIdentifier Desired pollutant, e.g. NOx
	 * @param[in] power in [kW]
     * @return emission in [g/h]
     */
	double GetEmission(std::string pollutantIdentifier, double power);

	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f0
     */
	double GetResistanceF0();
	
	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f1
     */
	double GetResistanceF1();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f2
     */
	double GetResistanceF2();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f3
     */
	double GetResistanceF3();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f4
     */
	double GetResistanceF4();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return Cw value
     */
	double GetCdValue();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return crosssectional area of vehicle
     */
	double GetCrossSectionalArea();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return vehicle mass
     */
	double GetMassVehicle();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return vehicle loading
     */
	double GetVehicleLoading();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return rotational mass of vehicle
     */
	double GetMassRot();

	/** @brief Getter function to recieve vehicle data from CEP
     * @return rated power of vehicle
     */
	double GetRatedPower();

private:

	/** @brief Interpolates emission linearly between two known power-emission pairs
	 * @param[in] px power-value to interpolate
	 * @param[in] p1 first known power value
	 * @param[in] p2 second known power value
	 * @param[in] e1 emission value for p1
	 * @param[in] e2 emission value for p2
     * @return emission value for px
     */
	double Interpolate(double px, double p1, double p2, double e1, double e2);

// ===========================================================================
// member declarations
// ===========================================================================

	std::string _vehicleType;
	SUMOEmissionClass _emissionClass;
	double _resistanceF0;
	double _resistanceF1;
	double _resistanceF2;
	double _resistanceF3;
	double _resistanceF4;
	double _cdValue;
	double _crossSectionalArea;
	double _massVehicle;
	double _vehicleLoading;
	double _massRot;
	double _ratedPower;
	int _sizeOfPattern;
	int _numberPollutants;
	std::vector<double> _powerPattern;

	StringBijection<std::vector<double> > _cepCurves;
};

#endif
