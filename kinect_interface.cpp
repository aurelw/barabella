#include "kinect_interface.h"



void KinectInterface::cloud_callback(const CloudConstPtr &cld) {
    cloud = cld;
}


KinectInterface::CloudConstPtr KinectInterface::getLastCloud() {
    return cloud;
}


void KinectInterface::setupGrabber() {

    // modes can be specified
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    grabber = new pcl::OpenNIGrabber();
    
    // setup callback
    boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&KinectInterface::cloud_callback, this, _1);
    grabber->registerCallback(cloud_cb);

    // start stream
    grabber->start();
}
