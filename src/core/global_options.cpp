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

#include "global_options.h"

#include <boost/filesystem.hpp>


GlobalOptions::GlobalOptions(int argc, char** argv) {
    doRecording = pcl::console::find_switch(argc, argv, "-r");
    doPlayBack =  pcl::console::find_switch(argc, argv, "-p");

    clipDirectory = "/tmp/foobar/";
    selectionCubeSettingsPath = "/tmp/barabella_scube";

    validSCubePath = boost::filesystem::is_regular_file(selectionCubeSettingsPath);

    traceCSVPath = "/tmp/bb_trace.csv";
}


