/****************************************************************************/
/// @file    SUMOTime.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Fri, 29.04.2005
/// @version $Id: SUMOTime.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Variables, methods, and tools for internal time representation
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
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
#include "TraCIConfig.h"
#endif

#include <sstream>
#include "SUMOTime.h"
#include "TplConvert.h"


// ===========================================================================
// type definitions
// ===========================================================================
#ifdef HAVE_SUBSECOND_TIMESTEPS
SUMOTime DELTA_T = 1000;
#endif


// ===========================================================================
// method definitions
// ===========================================================================
SUMOTime
string2time(const std::string& r) throw(EmptyData, NumberFormatException, ProcessError) {
    double time;
    std::istringstream buf(r);
    buf >> time;
    if (buf.fail()) {
        throw ProcessError("Input string '" + r + "' cannot be parsed as a time");
    } else {
        return TIME2STEPS(time);
    }
}


std::string
time2string(SUMOTime t) {
    // 123456 -> "12.34"
    std::ostringstream oss;
    oss.setf(oss.fixed);
    oss.precision(OUTPUT_ACCURACY);
    oss << STEPS2TIME(t);
    return oss.str();
}


/****************************************************************************/

