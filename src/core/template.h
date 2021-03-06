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

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/smart_ptr.hpp>

#include <pcl/common/common_headers.h>

#ifndef __TEMPLATE_H__
#define __TEMPLATE_H__

class CloudTemplate {

    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<CloudTemplate> Ptr;

    public:
        Eigen::Vector3f center;

        virtual void setPointCloud(PointCloudConstPtr cloud);
        virtual PointCloudConstPtr getPointCloud();

        virtual void saveToFile(std::string path);
        virtual void loadFromFile(std::string path);

    protected:

        PointCloudConstPtr templateCloud;

    /* serialization */
    protected:
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
            ar & center[0];
            ar & center[1];
            ar & center[2];
        }

};



class FilteredCloudTemplate : public CloudTemplate {

    public:

        FilteredCloudTemplate(float vSize) :
            CloudTemplate(),
            voxelSize(vSize)
        {
        }

        FilteredCloudTemplate() :
            CloudTemplate(),
            voxelSize(0.01)
        {
        }

        void setPointCloud(PointCloudConstPtr cloud);

    protected:

        float voxelSize;

};

#endif

