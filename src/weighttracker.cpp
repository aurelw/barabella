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

#include <iostream>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include "barabella_app.h"


int main (int argc, char** argv) {

#ifdef BB_INFO
    std::cout << "My name isn't pretty, pretty, it's Barabella v" << 
        BARABELLA_VERSION << std::endl;
#endif

    /* global options */
    GlobalOptions options(argc, argv);

    /* the app */
    BarabellaApp app(&options);

    /* clip recording and playback */
    /*
    Clip clip;
    clip.setDirectory(options.clipDirectory);
    if (options.doRecording) {
        clip.startRecording(&kinIface);
    }
    if (options.doPlayBack) {
        clip.load();
        clip.begin();
    }
    */

    while (true) {
        app.spinOnce();
/*
        if (view3d.flagExtractTemplate) {
            view3d.flagExtractTemplate = false;
            templateCloud = sCube.filterCloud(*cloudptr);
            view3d.addTemplate(templateCloud);
            view3d.setDrawMode(View3D::TEMPLATE);
            displayTemplate = true;
        }
    */
    }
    



    
    return (0);
}
