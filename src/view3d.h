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

#include "selection_cube.h"


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

        enum DrawMode {
            NORMAL,
            TEMPLATE,
        };


        View3D() {
            visualizer.addCoordinateSystem(1.0);
            visualizer.setBackgroundColor(0.2, 0.2, 0.2);
            registerCallbacks();

            cloudTransform.setIdentity();
            //cloudTransform.rotate(Eigen::AngleAxisf(1.5707963267948966,
            //            Eigen::Vector3f(1.0, 0.0, 0.0)));

            /* interaction and flags */
            state = START;
            flagCaptureFloor = false;
            flagExtractTemplate = false;

        }

        void spinOnce();
        void updateCloud(PointCloudConstPtr cloud);
        void addCloud(PointCloudConstPtr cloud);
        void setFloor(pcl::ModelCoefficients::Ptr coefficients);
        void setCube(SelectionCube* cube);

        void addTemplate(PointCloudConstPtr cloud);

        void setDrawMode(DrawMode mode);

        bool flagCaptureFloor;
        bool flagExtractTemplate;
        bool flagTrack;

    private:

        /* pcl visualizer */
        pcl::visualization::PCLVisualizer visualizer;
        void registerCallbacks();

        /* the raw main cloud */
        PointCloudConstPtr mainCloud;
        Eigen::Affine3f cloudTransform;
        pcl::visualization::PointCloudGeometryHandler<PointT>::ConstPtr gemHandl;

        /* a selection cube */
        SelectionCube* sCube;
        void moveCube(float dx, float dy, float dz);
        void updateSelectionCube();
        float edit_stepsize = 0.025;

        /* template */
        PointCloudConstPtr templateCloud;

        /* user input and draw states */
        InteractionState state;
};


#endif

