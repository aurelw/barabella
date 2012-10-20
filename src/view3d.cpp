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

#include "utils.h"


void View3D::spinOnce() {
    visualizer.spinOnce(10, false);
}


void View3D::updateCloud(PointCloudConstPtr cloud) {
    mainCloud = cloud;
    visualizer.updatePointCloud(mainCloud, "mainCloud");
}


void View3D::addCloud(PointCloudConstPtr cloud) {
    mainCloud = cloud;
    visualizer.addPointCloud(mainCloud, "mainCloud");
}


void View3D::addTemplate(PointCloudConstPtr cloud) {
    visualizer.removePointCloud("templateCloud");
    templateCloud = cloud;
}


void View3D::setDrawMode(DrawMode mode) {
    visualizer.removePointCloud("templateCloud");
    visualizer.removePointCloud("mainCloud");

    switch (mode) {
        case NORMAL:
            visualizer.addPointCloud(mainCloud, "mainCloud");
            break;
        case TEMPLATE:
            visualizer.addPointCloud(templateCloud, "templateCloud");
            break;
    }

}


void View3D::setFloor(pcl::ModelCoefficients::Ptr coefficients) {
    visualizer.removeShape("floor");
    visualizer.addPlane(*coefficients, "floor");

    cloudTransform = affineFromPlane(coefficients);
    visualizer.addCoordinateSystem(0.5, cloudTransform);

    /* draw an arrow */
    pcl::PointXYZ p0, p1;
    p0.x = p0.y = p0.z = 0.0;
    p1.x = -coefficients->values[0];
    p1.y = -coefficients->values[1];
    p1.z = -coefficients->values[2];

    visualizer.removeShape("floor_line");
    visualizer.addLine(p0, p1, "floor_line");

    /* transform the main cloud */
    // doesn't work as expected, only for PointCloud2 Foo 
    /*
    pcl::visualization::PointCloudGeometryHandler<PointT>::Ptr handl
        (new pcl::visualization::PointCloudGeometryHandlerXYZ<PointT>(mainCloud));
    gemHandl = handl;

    Eigen::Vector4f translation(1.0, 1.0, 1.0, 1.0);
    Eigen::Quaternionf rotation(cloudTransform.rotation());

    visualizer.removePointCloud("mainCloud");
    visualizer.addPointCloud(mainCloud, *gemHandl, 
            translation,
            rotation,
            "mainCloud");
    */
}


void View3D::updateSelectionCube() {
#ifdef BB_FLOOD
    std::cout << "View3D::updateSelectionCube" << std::endl;
#endif
    visualizer.removeShape("selection_cube");
    visualizer.addCube(sCube->getGlobalPosition(),
            sCube->getGlobalRotation(),
            sCube->getSx(),
            sCube->getSy(),
            sCube->getSz(),
            "selection_cube");
}


void View3D::registerCallbacks() {
    visualizer.registerKeyboardCallback(keyboardCallback, (void*) this);
}


void keyboardCallback(const pcl::visualization::KeyboardEvent &event, 
        void* view3d_void) 
{
    View3D* view3d = (View3D*) view3d_void;
    std::string keysym = event.getKeySym();

    if (event.keyDown()) {

        /* flags which are checked externaly */
        if (keysym == "c") {
            view3d->flagCaptureFloor = true;
        } else if (keysym == "t") {
            view3d->flagExtractTemplate = true;
        }

#ifdef BB_VERBOSE
        std::cout << "event: " << view3d->state << " " << keysym << std::endl;
#endif
        /* interaction state machine */
        switch (view3d->state) {

            case View3D::START:  // start state
                if (keysym == "v") {
                    view3d->state = View3D::GRAB;
                } else if (keysym == "b") {
                    view3d->state = View3D::SCALE;
                }
                break;

            case View3D::GRAB:  // move the cube
                if (keysym == "x") {
                    view3d->state = View3D::GRABX;
                } else if (keysym == "y") {
                    view3d->state = View3D::GRABY;
                } else if (keysym == "z") {
                    view3d->state = View3D::GRABZ;
                }
                break;

            case View3D::GRABX:
                if (keysym == "Up") {
                    view3d->moveCube(view3d->edit_stepsize, 0, 0);
                } else if (keysym == "Down") {
                    view3d->moveCube(-view3d->edit_stepsize, 0, 0);
                }
                break;

            case View3D::GRABY:
                if (keysym == "Up") {
                    view3d->moveCube(0, view3d->edit_stepsize, 0);
                } else if (keysym == "Down") {
                    view3d->moveCube(0, -view3d->edit_stepsize, 0);
                }
                break;

            case View3D::GRABZ:
                if (keysym == "Up") {
                    view3d->moveCube(0, 0, view3d->edit_stepsize);
                } else if (keysym == "Down") {
                    view3d->moveCube(0, 0, -view3d->edit_stepsize);
                }
                break;

            case View3D::SCALE:
                if (keysym == "x") {
                    view3d->state = View3D::SCALEX;
                } else if (keysym == "y") {
                    view3d->state = View3D::SCALEY;
                } else if (keysym == "z") {
                    view3d->state = View3D::SCALEZ;
                }
                break;

            case View3D::SCALEX:
                if (keysym == "Up") {
                    view3d->sCube->setSx( 
                            view3d->sCube->getSx() +
                            view3d->edit_stepsize);
                } else if (keysym == "Down") {
                    view3d->sCube->setSx( 
                            view3d->sCube->getSx() -
                            +view3d->edit_stepsize);
                }
                break;

            case View3D::SCALEY:
                if (keysym == "Up") {
                    view3d->sCube->setSy( 
                            view3d->sCube->getSy() +
                            view3d->edit_stepsize);
                } else if (keysym == "Down") {
                    view3d->sCube->setSy( 
                            view3d->sCube->getSy() -
                            view3d->edit_stepsize);
                }
                break;

            case View3D::SCALEZ:
                if (keysym == "Up") {
                    view3d->sCube->setSz( 
                            view3d->sCube->getSz() +
                            view3d->edit_stepsize);
                } else if (keysym == "Down") {
                    view3d->sCube->setSz( 
                            view3d->sCube->getSz() -
                            view3d->edit_stepsize);
                }
                break;

            defaul:  // if state is not handled
                break;
        }
        /* for all states */
        if (keysym == "Escape" ||
            keysym == "Return")
        {
            view3d->state = View3D::START;
        }
        /**********************/
    }

}


void View3D::setCube(SelectionCube* cube) {
    sCube = cube;
    updateSelectionCube();
    // connect signal to slot
    sCube->connect(boost::bind(&View3D::updateSelectionCube, this));
}


void View3D::moveCube(float dx, float dy, float dz) {
    Eigen::Vector3f pos = sCube->getPosition();
    pos[0] += dx;
    pos[1] += dy;
    pos[2] += dz;
    sCube->setPosition(pos);
}

