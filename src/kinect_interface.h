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

#include <stdio.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>


#ifndef __KINECT_INTERFACE_H__
#define __KINECT_INTERFACE_H__



class KinectInterface {

    public:

        typedef pcl::PointXYZRGBA PointType;
        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename Cloud::ConstPtr CloudConstPtr;


        KinectInterface() {
            setupGrabber();
        }


        CloudConstPtr getLastCloud();

    private:
        void setupGrabber();
        void cloud_callback(const CloudConstPtr& cld);

        pcl::OpenNIGrabber *grabber;
        CloudConstPtr cloud;

};

#endif

