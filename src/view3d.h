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
        
        friend void keyboardCallback(const pcl::visualization::KeyboardEvent&, void*);

        enum InteractionState {
            START,
            GRAB,
            GRABX,
            GRABY,
            GRABZ,
            SCALE,
            SCALEX,
            SCALEY,
            SCALEZ,
        };


        View3D() {
            visualizer.addCoordinateSystem(1.0);
            registerCallbacks();

            cloudTransform.setIdentity();
            //cloudTransform.rotate(Eigen::AngleAxisf(1.5707963267948966,
            //            Eigen::Vector3f(1.0, 0.0, 0.0)));

            //visualizer.addCube(cubeCoe);
            cube_x = cube_y = cube_z = 0.0;
            cube_sx = cube_sy = cube_sz = 0.5;
            updateSelectionCube();

            // interaction and flags
            state = START;
            flagCaptureFloor = false;
        }

        void spinOnce();
        void updateCloud(PointCloudConstPtr cloud);
        void addCloud(PointCloudConstPtr cloud);
        void setFloor(pcl::ModelCoefficients::Ptr coefficients);

        bool flagCaptureFloor;

    private:

        pcl::visualization::PCLVisualizer visualizer;
        void registerCallbacks();

        PointCloudConstPtr mainCloud;
        Eigen::Affine3f cloudTransform;
        pcl::visualization::PointCloudGeometryHandler<PointT>::ConstPtr gemHandl;

        float cube_x, cube_y, cube_z, cube_sx, cube_sy, cube_sz;
        void updateSelectionCube();
        float edit_stepsize = 0.025;


        InteractionState state;

};


#endif


