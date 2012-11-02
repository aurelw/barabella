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
    view3d.setCube(sCube);
    view3d.setMainCloud(mainCloud);
    view3d.setDrawMode(View3D::NORMAL);
}


void BarabellaApp::initTemplates() {
    if (gOptions->validSCubePath) {
        sCube->loadFromFile(gOptions->selectionCubeSettingsPath);
    }
}


void BarabellaApp::startTracker() {
    tracker.setClip(clip);
    tracker.setCoordinateFrame(floorTrans.inverse()); 
    tracker.setTemplate(cloudTemplate);
    tracker.setInitialSearchWindow(sCube);
    tracker.initTracker();
}


void BarabellaApp::setOperationMode(OperationMode mode) {
    operationMode = mode;
    switch (operationMode) {

        case STREAMING:
            view3d.setDrawMode(View3D::NORMAL);
            break;

        case CLIPPLAYBACK:
            view3d.setDrawMode(View3D::NORMAL);
            break;

        case TRACKING:
            view3d.setDrawMode(View3D::TRACKING);
            startTracker();
            break;
    }
}


void BarabellaApp::spinOnce() {

    switch (operationMode) {
        
        case STREAMING:
            spinStreaming();
            break;

        case CLIPPLAYBACK:
            spinClipPlayBack();
            break;

        case TRACKING:
            spinTracking();
            break;
    }

}


void BarabellaApp::spinStreaming() {

    mainCloud = kinIface.getLastCloud();

    if (!displayTemplate) {
        view3d.setMainCloud(mainCloud);
    }

    view3d.spinOnce();

    /* View3D events */
    if (view3d.flagCaptureFloor) {
        view3d.flagCaptureFloor = false;
        updateFloor();
    }

    if (view3d.flagExtractTemplate) {
        view3d.flagExtractTemplate = false;
        if (!displayTemplate) {
            extractTemplate();
            saveTemplateSettings();
        } else {
            view3d.setDrawMode(View3D::NORMAL);
        }
        displayTemplate = !displayTemplate;
    }

    if (view3d.flagTrack) {
        view3d.flagTrack = false;
        setOperationMode(TRACKING);
    }
}


void BarabellaApp::spinClipPlayBack() {
    mainCloud = clipPlayer.getLastCloud();
    view3d.setMainCloud(mainCloud);
    view3d.spinOnce();
}


void BarabellaApp::spinTracking() {
    if (!tracker.finished()) {
        tracker.processFrame();

        /* display results of current frame */
        mainCloud = tracker.getCurrentCloud();
        view3d.setMainCloud(mainCloud);
        view3d.setCube(tracker.getSearchWindow());
        view3d.setTrackedCenter(
        //tracker.getSearchWindow()->getGlobalPosition());
        (tracker.getTrace()->coordinateFrame.inverse() *
         tracker.getTrace()->transforms.back() ).translation());
        view3d.spinOnce();

        //FIXME
#ifdef BB_VERBOSE
        std::cout << "translation: " << std::endl;
        std::cout << tracker.getTrace()->transforms.back().translation() << std::endl;
        std::cout << "-------------" << std::endl;
#endif
    } else { // finished tracking
        tracker.getTrace()->writeToCSV(gOptions->traceCSVPath);
        setOperationMode(STREAMING);
    }

}


void BarabellaApp::updateFloor() {
    floorEx.setInputCloud(mainCloud);
    floorEx.extract(floorCoefficients);
    floorTrans = affineFromPlane(floorCoefficients);
    // set the transformation of the selection cube
    sCube->setCoordinateFrame(floorTrans);
    // display floor in view3d
    view3d.setFloor(floorCoefficients);
}


void BarabellaApp::extractTemplate() {
    cloudTemplate->setPointCloud(sCube->filterCloud(*mainCloud));
    cloudTemplate->center = sCube->getGlobalPosition();
    view3d.setTemplateCloud(cloudTemplate->getPointCloud());
    view3d.setDrawMode(View3D::TEMPLATE);
}


void BarabellaApp::saveTemplateSettings() {
    sCube->saveToFile(gOptions->selectionCubeSettingsPath);
}

