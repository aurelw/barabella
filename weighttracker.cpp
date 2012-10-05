#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>


#include "kinect_interface.h"
#include "view3d.h"
#include "floor_extractor.h"



typedef pcl::PointXYZRGBA PointType;


int main (int argc, char** argv) {

    KinectInterface kinIface;
    View3D view3d;

    pcl::PointCloud<PointType>::ConstPtr cloudptr;
    cloudptr = kinIface.getLastCloud();
    view3d.addCloud(cloudptr);

    FloorExtractor floorEx;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    
    std::cout << "Enter Main Loop" << std::endl;
    while (true) {
        view3d.spinOnce();
        view3d.updateCloud(kinIface.getLastCloud());
        cloudptr = kinIface.getLastCloud();

        if (view3d.flagCaptureFloor) {
            view3d.flagCaptureFloor = false;
            floorEx.setInputCloud(cloudptr);
            floorEx.extract(coefficients);
            view3d.setFloor(coefficients);
        }
    }
    



    
    return (0);
}
