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
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;
        
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
            TRACKING
        };


        View3D() :
                mainCloud(new PointCloud),
                templateCloud(new PointCloud) 
        {
            /* basic rendering */
            visualizer.addCoordinateSystem(1.0);
            visualizer.setBackgroundColor(0.2, 0.2, 0.2);

            registerCallbacks();

            /* init render items */
            cloudTransform.setIdentity();
            //cloudTransform.rotate(Eigen::AngleAxisf(1.5707963267948966,
            //            Eigen::Vector3f(1.0, 0.0, 0.0)));
            

            /* item render options */
            mainCloudAdded = false;
            templateCloudAdded = false;

            /* interaction and flags */
            state = START;
            flagCaptureFloor = false;
            flagExtractTemplate = false;
            flagTrack = false;

        }

        void spinOnce();

        /* set render items */
        void setMainCloud(PointCloudConstPtr cloud);
        void setFloor(pcl::ModelCoefficients::Ptr coefficients);
        void setCube(SelectionCube::Ptr cube);
        void setTemplateCloud(PointCloudConstPtr cloud);
        void setTrackedCenter(const Eigen::Vector3f& v);

        /* set render parameters */
        void setDrawMode(DrawMode mode);

        /* interaction flags */
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
        void updateMainCloud();
        bool drawMainCloud;
        bool mainCloudAdded;

        /* the floor */
        pcl::ModelCoefficients::Ptr floorCoefficients;
        void updateFloor();
        bool drawFloor;

        /* a selection cube */
        SelectionCube::Ptr sCube;
        void moveCube(float dx, float dy, float dz);
        float edit_stepsize = 0.025;
        void updateSelectionCube();
        bool drawSelectionCube;

        /* template */
        PointCloudConstPtr templateCloud;
        void updateTemplateCloud();
        bool drawTemplateCloud;
        bool templateCloudAdded;

        /* tracking */
        Eigen::Vector3f trackedCenter;
        void updateTrackedCenter();
        bool drawTrackedCenter;

        /* user input and draw states */
        InteractionState state;

};


#endif

