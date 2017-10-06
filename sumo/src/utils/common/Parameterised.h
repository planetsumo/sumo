/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    Parameterised.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Melanie Knocke
/// @date    Sept 2002
/// @version $Id$
///
// A super class for objects with additional parameters
/****************************************************************************/
#ifndef Parameterised_h
#define Parameterised_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <map>
#include <string>

// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Parameterised
 * @brief An upper class for objects with additional parameters
 */
class Parameterised {
public:
    /** @brief Constructor
     */
    Parameterised();


    /** @brief Constructor
     * @param[in] mapArg Pre-given parameter
     */
    Parameterised(const std::map<std::string, std::string>& mapArg);


    /// @brief Destructor
    ~Parameterised();



    /** @brief Sets a parameter
     * @param[in] key The parameter's name
     * @param[in] value The parameter's value
     */
    void setParameter(const std::string& key, const std::string& value);

    /** @brief Removes a parameter
     * @param[in] key The parameter's name
     */
    void unsetParameter(const std::string& key);


    /** @brief Adds or updates all given parameters from the map
     * @param[in] mapArg The keys/values to insert
     */
    void updateParameter(const std::map<std::string, std::string>& mapArg);


    /** @brief Returns whether the parameter is known
     * @param[in] key The key to ask for
     * @return Whether the key is known
     */
    bool knowsParameter(const std::string& key) const;


    /** @brief Returns the value for a given key
     * @param[in] key The key to ask for
     * @param[in] defaultValue The default value to return if no value is stored under the key
     * @return The value stored under the key
     */
    const std::string& getParameter(const std::string& key, const std::string& defaultValue = "") const;


    /** @brief Returns the value for a given key converted to a double
     * @param[in] key The key to ask for
     * @param[in] defaultValue The default value to return if no value is stored under the key
     * @return The value stored under the key
     */
    double getDouble(const std::string& key, const double defaultValue) const;


    /** @brief Clears the parameter map
     */
    void clearParameter();


    /** @brief Returns the inner key/value map
     * @return the inner map
     */
    const std::map<std::string, std::string>& getMap() const {
        return myMap;
    }

    void writeParams(OutputDevice& out) const;


private:
    /// @brief The key->value map
    std::map<std::string, std::string> myMap;

};


#endif

/****************************************************************************/

