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

    // create sef object
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

