/****************************************************************************/
/// @file    MSGrid.h
/// @author  Daniel Krajzewicz
/// @date    2011-09-08
/// @version $Id: MSGrid.h 11042 2011-07-20 10:53:55Z namdre $
///
// Realises dumping the complete network state
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2011 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSGrid_h
#define MSGrid_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSDetectorFileOutput.h"
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class MSNet;
class MSEdgeControl;
class MSEdge;
class MSLane;
class Position;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSGrid
 * @brief Realises dumping the complete network state
 *
 * The class offers a static method, which writes the complete dump of
 *  the given network into the given OutputDevice.
 *
 * @todo consider error-handling on write (using IOError)
 */
class MSGrid : public MSDetectorFileOutput {
public:
    MSGrid(const std::string &id, SUMOReal xs, SUMOReal ys, const Position &offset, const Position &size, MSNet &n);

    ~MSGrid();


    
    /// @name Inherited from MSDetectorFileOutput
    /// @{

    /** @brief Write the generated output to the given device
     * @param[in] dev The output device to write the data into
     * @param[in] startTime First time step the data were gathered
     * @param[in] stopTime Last time step the data were gathered
     * @exception IOError If an error on writing occurs
     */
    void writeXMLOutput(OutputDevice &dev,
                                SUMOTime startTime, SUMOTime stopTime) throw(IOError);


    /** @brief Open the XML-output
     *
     * The implementing function should open an xml element using
     *  OutputDevice::writeXMLHeader.
     *
     * @param[in] dev The output device to write the root into
     * @exception IOError If an error on writing occurs
     */
    void writeXMLDetectorProlog(OutputDevice &dev) const throw(IOError);


    /** @brief Updates the detector (computes values)
     *
     * @param[in] step The current time step
     */
    virtual void detectorUpdate(const SUMOTime step) throw();
    /// @}

    Position &getOffset() {
        return myOffset;
    }

    Position &getSize() {
        return mySize;
    }
    Position &getBoxSize() {
        return myBoxSizes;
    }

    const unsigned int getXNumber() {
        return myXNumber;
    }

    const unsigned int getYNumber() {
        return myYNumber;
    }

    SUMOReal *getValues() {
        return myValues;
    }

protected:
    MSEdgeControl &myEdges;
    Position myOffset;
    Position mySize;
    Position myBoxSizes;
    unsigned int myXNumber, myYNumber;
    SUMOReal *myValues;
  

private:
    /// @brief Invalidated copy constructor.
    MSGrid(const MSGrid&);

    /// @brief Invalidated assignment operator.
    MSGrid& operator=(const MSGrid&);


};


#endif

/****************************************************************************/

