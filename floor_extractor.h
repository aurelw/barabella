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
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        FloorExtractor() {

        }


        void setInputCloud(PointCloudConstPtr cloud);
        void extract(pcl::ModelCoefficients::Ptr coefficients);


    private:

        void printCoefficients(pcl::ModelCoefficients::Ptr coe);
        bool checkFloorPlane(pcl::ModelCoefficients::Ptr coe);

        PointCloudConstPtr inputCloud;

};

