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
#include <fstream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "boost/smart_ptr.hpp"

#include <pcl/common/common_headers.h>

#include "update_signal.h"
#include "barabella_config.h"


#ifndef __SELECTION_CUBE_H__
#define __SELECTION_CUBE_H__


class SelectionCube : public UpdateSignal {

    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<SelectionCube> Ptr;

    public:

        SelectionCube() : 
            position(0,0,0),
            scaleX(1.0), scaleY(1.0), scaleZ(1.0)    
        {
        }

        SelectionCube(const SelectionCube& oCube);

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
        Eigen::Affine3f getGlobalTransformation();
        Eigen::Vector3f getGlobalPosition();
        Eigen::Quaternionf getGlobalRotation();

        /* filtering */
        PointCloudPtr filterCloud(const PointCloud& cloud);

        /* load/save */
        void loadFromFile(std::string path);
        void saveToFile(std::string path);


    private:

        Eigen::Vector3f position;
        float scaleX, scaleY, scaleZ;
        //additional transformation for the floor
        Eigen::Affine3f coordinateFrame;

    /* serialization */
    private:
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
            ar & position[0];
            ar & position[1];
            ar & position[2];
            ar & scaleX;
            ar & scaleY;
            ar & scaleZ;
            //ar & coordinateFrame;
        }

};

#endif
