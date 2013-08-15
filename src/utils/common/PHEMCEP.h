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
class PHEMCEP {	
public:
    /*** @brief Constructor
     * @param[in] emissionClass PHEM emission class of vehicle
     * @param[in] vehicleMass vehicle mass
     * @param[in] vehicleLoading vehicle loading
     * @param[in] vehicleMassRot rotational mass of vehicle
     * @param[in] crossArea crosssectional area of vehicle
     * @param[in] cWValue cw-value
     * @param[in] f0 Rolling resistance f0
     * @param[in] f1 Rolling resistance f1
     * @param[in] f2 Rolling resistance f2
     * @param[in] f3 Rolling resistance f3
     * @param[in] f4 Rolling resistance f4
     * @param[in] ratedPower rated power of vehicle
     * @param[in] headerLine Definition of covered pollutants
     * @param[in] matrix Coefficients of the pollutants
     */
	PHEMCEP(SUMOEmissionClass emissionClass,
		     double vehicleMass, double vehicleLoading, double vehicleMassRot, 
             double crossArea, double cWValue,
		     double f0, double f1, double f2, double f3, double f4,
			 double ratedPower, const std::vector<std::string> &headerLine, 
             const std::vector< std::vector<double> > &matrix);


    /// @brief Destructor
	~PHEMCEP();


	/** @brief Returns a emission measure for power[kW] level
	 * @param[in] pollutantIdentifier Desired pollutant, e.g. NOx
	 * @param[in] power in [kW]
     * @return emission in [g/h]
     */
	double GetEmission(const std::string &pollutantIdentifier, double power) const;


	/** @brief Getter function to recieve vehicle data from CEP
     * @return PHEM emission class of vehicle
     */
	SUMOEmissionClass GetEmissionClass() const {
        return _emissionClass;
    }


    /** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f0
     */
	double GetResistanceF0() const {
        return _resistanceF0;
    }
	

	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f1
     */
	double GetResistanceF1() const {
        return _resistanceF1;
    }


	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f2
     */
	double GetResistanceF2() const {
        return _resistanceF2;
    }


	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f3
     */
	double GetResistanceF3() const {
        return _resistanceF3;
    }


	/** @brief Getter function to recieve vehicle data from CEP
     * @return Rolling resistance f4
     */
	double GetResistanceF4() const {
        return _resistanceF4;
    }


	/** @brief Getter function to recieve vehicle data from CEP
     * @return Cw value
     * @todo: Why is it named "cdValue", here?
     */
	double GetCdValue() const {
        return _cwValue;
    }

	/** @brief Getter function to recieve vehicle data from CEP
     * @return crosssectional area of vehicle
     */
	double GetCrossSectionalArea() const {
        return _crossSectionalArea;
    }


	/** @brief Getter function to recieve vehicle data from CEP
     * @return vehicle mass
     */
	double GetMassVehicle() const {
        return _massVehicle;
    }

	/** @brief Getter function to recieve vehicle data from CEP
     * @return vehicle loading
     */
	double GetVehicleLoading() const {
        return _vehicleLoading;
    }


	/** @brief Getter function to recieve vehicle data from CEP
     * @return rotational mass of vehicle
     */
	double GetMassRot() const {
        return _massRot;
    }


	/** @brief Getter function to recieve vehicle data from CEP
     * @return rated power of vehicle
     */
	double GetRatedPower() const {
        return _ratedPower;
    }



private:
	/** @brief Interpolates emission linearly between two known power-emission pairs
	 * @param[in] px power-value to interpolate
	 * @param[in] p1 first known power value
	 * @param[in] p2 second known power value
	 * @param[in] e1 emission value for p1
	 * @param[in] e2 emission value for p2
     * @return emission value for px
     */
	double Interpolate(double px, double p1, double p2, double e1, double e2) const;


private:
    /// @brief PHEM emission class of vehicle
	SUMOEmissionClass _emissionClass;
    /// @brief Rolling resistance f0
	double _resistanceF0;
    /// @brief Rolling resistance f1
	double _resistanceF1;
    /// @brief Rolling resistance f2
	double _resistanceF2;
    /// @brief Rolling resistance f3
	double _resistanceF3;
    /// @brief Rolling resistance f4
	double _resistanceF4;
    /// @brief Cw value
	double _cwValue;
    /// @brief crosssectional area of vehicle
	double _crossSectionalArea;
    /// @brief vehicle mass
	double _massVehicle;
    /// @brief vehicle loading
	double _vehicleLoading;
    /// @brief rotational mass of vehicle
	double _massRot;
    /// @brief rated power of vehicle
	double _ratedPower;
    /// @todo describe
	int _sizeOfPattern;
    /// @todo describe
	int _numberPollutants;
    /// @todo describe
	std::vector<double> _powerPattern;
    /// @todo describe
	StringBijection<std::vector<double> > _cepCurves;

};

#endif

/****************************************************************************/
