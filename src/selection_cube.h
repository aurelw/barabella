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

#include <pcl/common/common_headers.h>

#include "update_signal.h"
#include "barabella_config.h"


#ifndef __SELECTION_CUBE_H__
#define __SELECTION_CUBE_H__


class SelectionCube : public UpdateSignal {

    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

        SelectionCube() : 
            position(0,0,0),
            scaleX(1.0), scaleY(1.0), scaleZ(1.0)    
        {
        }

        /* local transformations */
        Eigen::Vector3f getPosition();
        float getSx();
        float getSy();
        float getSz();

        void setPosition(const Eigen::Vector3f& pos);
        void setSx(const float sx);
        void setSy(const float sy);
        void setSz(const float sz);

        /* transform in different coordinate frame */
        void setCoordinateFrame(const Eigen::Affine3f& t);
        Eigen::Affine3f getCoordinateFrame();
        Eigen::Vector3f getGlobalPosition();
        Eigen::Quaternionf getGlobalRotation();

        /* filtering */
        PointCloudPtr filterCloud(const PointCloud& cloud);


    private:

        Eigen::Vector3f position;
        float scaleX, scaleY, scaleZ;
        //additional transformation for the floor
        Eigen::Affine3f coordinateFrame;

};

#endif
