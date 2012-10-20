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

#include "barabella_app.h"


void BarabellaApp::initView3d() {
    view3d.setCube(&sCube);
    mainCloud = kinIface.getLastCloud();
    view3d.addCloud(mainCloud);
}


void BarabellaApp::spinOnce() {

    mainCloud = kinIface.getLastCloud();
    
    if (!displayTemplate) {
        view3d.updateCloud(mainCloud);
    }

    view3d.spinOnce();

    if (view3d.flagCaptureFloor) {
        view3d.flagCaptureFloor = false;
        updateFloor();
    }

    if (view3d.flagExtractTemplate) {
        view3d.flagExtractTemplate = false;
        if (!displayTemplate) {
            extractTemplate();
        } else {
            view3d.setDrawMode(View3D::NORMAL);
        }
        displayTemplate = !displayTemplate;
    }
}


void BarabellaApp::updateFloor() {
    floorEx.setInputCloud(mainCloud);
    floorEx.extract(floorCoefficients);
    floorTrans = affineFromPlane(floorCoefficients);
    // set the transformation of the selection cube
    sCube.setCoordinateFrame(floorTrans);
    // display floor in view3d
    view3d.setFloor(floorCoefficients);
}


void BarabellaApp::extractTemplate() {
    templateCloud = sCube.filterCloud(*mainCloud);
    templateCloud = sCube.filterCloud(*mainCloud);
    view3d.addTemplate(templateCloud);
    view3d.setDrawMode(View3D::TEMPLATE);
}

