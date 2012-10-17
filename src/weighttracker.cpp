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


#include "kinect_interface.h"
#include "view3d.h"
#include "utils.h"
#include "floor_extractor.h"
#include "barabella_config.h"



typedef pcl::PointXYZRGBA PointType;


int main (int argc, char** argv) {

#ifdef BB_INFO
    std::cout << "My name isn't pretty, pretty, it's Barabella v" << 
        BARABELLA_VERSION << std::endl;
#endif

    KinectInterface kinIface;
    View3D view3d;

    pcl::PointCloud<PointType>::ConstPtr cloudptr;
    pcl::PointCloud<PointType>::Ptr transCloud (new pcl::PointCloud<PointType>);
    cloudptr = kinIface.getLastCloud();
    view3d.addCloud(cloudptr);

    FloorExtractor floorEx;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    Eigen::Affine3f cloudTransform;

#ifdef BB_LOG 
    std::cout << "Enter Main Loop" << std::endl;
#endif
    while (true) {
        cloudptr = kinIface.getLastCloud();
        //pcl::transformPointCloud<PointType>(*cloudptr, *transCloud, cloudTransform);
             

        view3d.spinOnce();
        view3d.updateCloud(cloudptr);

        if (view3d.flagCaptureFloor) {
            view3d.flagCaptureFloor = false;
            floorEx.setInputCloud(cloudptr);
            floorEx.extract(coefficients);
            view3d.setFloor(coefficients);
            cloudTransform = affineFromPlane(coefficients).inverse();
        }
    }
    



    
    return (0);
}
