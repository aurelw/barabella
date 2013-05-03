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


#include "floor_extractor.h"



void FloorExtractor::printCoefficients(pcl::ModelCoefficients::Ptr coe) {
    printf("x=%f; y=%f; z=%f; d=%f;\n", 
            coe->values[0], coe->values[1], coe->values[2], coe->values[3]);
}


void FloorExtractor::setInputCloud(PointCloudConstPtr cloud) {
    inputCloud = cloud;
}


void FloorExtractor::extract(pcl::ModelCoefficients::Ptr coefficients) {
    
    // create temporary clouds
    PointCloudPtr cloud_filtered (new PointCloud(*inputCloud));

    // the model
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // create seg object
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.005);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    int nr_points = (int) inputCloud->size();
    int maxi = 4;
    int i = 0;

    while (cloud_filtered->points.size () > 0.3 * nr_points &&
            i++ < maxi) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        printCoefficients(coefficients);
        
        // end search if the plane seems to be the floor
        if (checkFloorPlane(coefficients)) {
            return;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
    }

}


std::vector<pcl::ModelCoefficients> FloorExtractor::extract(
        const int maxPlanes, const int iterations)
{
    // result set of planes
    std::vector<pcl::ModelCoefficients> planes;

    /* additional parameters */
    const float distanceThres = 0.01;

    // create temporary clouds
    PointCloudPtr cloud_filtered (new PointCloud(*inputCloud));

    /* preapre ransac segmentation */
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (iterations);
    seg.setDistanceThreshold (distanceThres);

    /* Create the filtering object */
    pcl::ExtractIndices<PointT> extract;

    for (int i=0; i<maxPlanes; i++) {

        /* coeffitients for the current plane */
        pcl::ModelCoefficients coefficients;

        /* segment the largest plane */
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, coefficients);

        /* no plane left to segment */
        if (inliers->indices.size() == 0) {
            break;
        }

        /* add plane to result set */
        planes.push_back(coefficients);

        /* extract inliers */
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
    }

    return planes;
}


pcl::ModelCoefficients FloorExtractor::refine(
        pcl::ModelCoefficients::Ptr coefficients)
{
    //TODO refine plane by better fitting to input cloud
    return *coefficients;
}


bool FloorExtractor::checkFloorPlane(pcl::ModelCoefficients::Ptr coe) {
    float x = coe->values[0];
    float y = coe->values[1];
    float z = coe->values[2];
    float d = coe->values[3];

    /* floor can't be above the sensor */
    if (y > 0) {
        return false;
    }

    /* check if the floor is tilted more than 45deg from the y axis */
    Eigen::Vector3d v(-x, -y, -z);
    Eigen::Vector3d yaxis(0, 1.0, 0);
    
    float angle = acos(yaxis.dot(v.normalized()));
    if (angle > 0.785) { // 45 deg
        return false;
    }

    return true;
} 

