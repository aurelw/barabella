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

#include "kinect_interface.h"

#include <boost/foreach.hpp>


void KinectInterface::cloud_callback(const CloudConstPtr &cld) {
    cloud = cld;
    if (cloud != NULL) {
        isStreaming = true;
        frameEvent(cloud);
    }
}


KinectInterface::CloudConstPtr KinectInterface::getLastCloud() {
    return cloud;
}


void KinectInterface::setupGrabber() {

    // modes can be specified
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    try {
        grabber = new pcl::OpenNIGrabber();
    } catch (pcl::PCLIOException& exc) {
        return;
    }
    
    // setup callback
    boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&KinectInterface::cloud_callback, this, _1);
    grabber->registerCallback(cloud_cb);

    // start 
    grabber->start();
    isConnected = true;
}


bool KinectInterface::init() {
    setupGrabber();
    return isConnected;
}


void KinectInterface::waitForFirstFrame() {
    while (!isStreaming) {
        usleep(10);
    }
}

