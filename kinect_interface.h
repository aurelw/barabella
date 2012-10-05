#include <stdio.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>


#ifndef __KINECT_INTERFACE_H__
#define __KINECT_INTERFACE_H__



class KinectInterface {

    public:

        typedef pcl::PointXYZRGBA PointType;
        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename Cloud::ConstPtr CloudConstPtr;


        KinectInterface() {
            setupGrabber();
        }


        CloudConstPtr getLastCloud();

    private:
        void setupGrabber();
        void cloud_callback(const CloudConstPtr& cld);

        pcl::OpenNIGrabber *grabber;
        CloudConstPtr cloud;

};

#endif

