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

#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image_planar.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>


#ifndef __VIEWED_H__
#define __VIEWED_H__


void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* view3d_void);


class View3D {

    public:

        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;


        View3D() {
            visualizer.addCoordinateSystem(1.0);
            registerCallbacks();

            /* init the selection cube */
            /*
            cubeCoe.values.resize(9);
            // translation
            cubeCoe.values[0] = 0;
            cubeCoe.values[1] = 0;
            cubeCoe.values[2] = 0;
            // rotation
            cubeCoe.values[3] = 0;
            cubeCoe.values[4] = 0;
            cubeCoe.values[5] = 0;
            // width, height, depth
            cubeCoe.values[6] = 1;
            cubeCoe.values[7] = 1;
            cubeCoe.values[8] = 0;
            */

            //visualizer.addCube(cubeCoe);
            visualizer.addCube(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1, 0, 0);

        }

        void spinOnce();
        void updateCloud(PointCloudConstPtr cloud);
        void addCloud(PointCloudConstPtr cloud);
        void setFloor(pcl::ModelCoefficients::Ptr coefficients);

        bool flagCaptureFloor;

    private:

        pcl::visualization::PCLVisualizer visualizer;
        PointCloudConstPtr mainCloud;
        pcl::ModelCoefficients cubeCoe;

        void registerCallbacks();

};


#endif


