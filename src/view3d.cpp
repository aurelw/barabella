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
