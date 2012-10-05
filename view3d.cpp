#include "view3d.h"


void View3D::spinOnce() {
    visualizer.spinOnce(10, false);
}


void View3D::updateCloud(PointCloudConstPtr cloud) {
    visualizer.updatePointCloud(cloud, "mainCloud");
    mainCloud = cloud;
}


void View3D::addCloud(PointCloudConstPtr cloud) {
    visualizer.addPointCloud(cloud, "mainCloud");
}


void View3D::setFloor(pcl::ModelCoefficients::Ptr coefficients) {
    visualizer.removeShape("floor");
    visualizer.addPlane(*coefficients, "floor");

    /* draw an arrow */
    pcl::PointXYZ p0, p1;
    p0.x = p0.y = p0.z = 0.0;
    p1.x = -coefficients->values[0];
    p1.y = -coefficients->values[1];
    p1.z = -coefficients->values[2];

    visualizer.removeShape("floor_line");
    visualizer.addLine(p0, p1, "floor_line");
}


void View3D::registerCallbacks() {
    visualizer.registerKeyboardCallback(keyboardCallback, (void*) this);
}


void keyboardCallback(const pcl::visualization::KeyboardEvent &event, 
        void* view3d_void) 
{
    View3D* view3d = (View3D*) view3d_void;
    if (event.keyDown()) {
        if (event.getKeySym() == "c") {
            view3d->flagCaptureFloor = true;
        }
    }
}
