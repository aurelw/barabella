/*
 *   (C) 2012, Aurel Wildfellner
 *
 *   This file is part of Barabella.
 *
 *   Barabella is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Barabella is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Barabella.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tracker.h"

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>


void Trace::writeToCSV(std::string fpath) {
    std::ofstream csvfile;
    csvfile.open(fpath.c_str());

    /* write the coordinates of the trace */
    //FIXME timestamps
    BOOST_FOREACH( Eigen::Affine3f trans, transforms ) {
        Eigen::Vector3f pos = trans.translation();
        csvfile << pos[0] << "," << pos[1] << "," << pos[2] << std::endl;
    }

    csvfile.close();
}


void Tracker::setClip(Clip* cl) {
    clip = cl;
    clip->load();
    clip->begin();
}


void Tracker::setCoordinateFrame(Eigen::Affine3f t) {
    coordinateFrame = t;
}


Trace::Ptr Tracker::getTrace() {
    return trace;
}
