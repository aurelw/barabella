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

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


class FloorExtractor {

    public:

        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        FloorExtractor() {

        }


        void setInputCloud(PointCloudConstPtr cloud);
        void extract(pcl::ModelCoefficients::Ptr coefficients);
        std::vector<pcl::ModelCoefficients> extract(const int maxPlanes=10, 
                const int iterations=100);
        pcl::ModelCoefficients refine(const pcl::ModelCoefficients& coefficients);

        /* just a helper which returns extracts the closest plane to a point */
        pcl::ModelCoefficients closestPlane(const PointT point,
                const std::vector<pcl::ModelCoefficients> planes);


    private:

        void printCoefficients(pcl::ModelCoefficients::Ptr coe);
        bool checkFloorPlane(pcl::ModelCoefficients::Ptr coe);

        PointCloudConstPtr inputCloud;

};

