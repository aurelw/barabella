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

#include "boost/smart_ptr.hpp"

#include <pcl/common/common_headers.h>

#ifndef __TEMPLATE_H__
#define __TEMPLATE_H__

class CloudTemplate {

    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<CloudTemplate> Ptr;

    public:
        Eigen::Vector3f center;

        virtual void setPointCloud(PointCloudConstPtr cloud);
        virtual PointCloudConstPtr getPointCloud();

    protected:

        PointCloudConstPtr templateCloud;
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

