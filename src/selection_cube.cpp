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

#include "selection_cube.h"

Eigen::Vector3f SelectionCube::getPosition() {
    return position;
}


float SelectionCube::getSx() {
    return scaleX;
}


float SelectionCube::getSy() {
    return scaleY;
}


float SelectionCube::getSz() {
    return scaleZ;
}


void SelectionCube::setPosition(const Eigen::Vector3f& pos) {
    position = pos;
    update();
}


void SelectionCube::setSx(const float sx) {
    scaleX = sx;
    update();
}


void SelectionCube::setSy(const float sy) {
    scaleY = sy;
    update();
}


void SelectionCube::setSz(const float sz) {
    scaleZ = sz;
    update();
}


void SelectionCube::setCoordinateFrame(const Eigen::Affine3f& t) {
    coordinateFrame = t;
    update();
}


Eigen::Affine3f SelectionCube::getCoordinateFrame() {
    return coordinateFrame;
}


Eigen::Vector3f SelectionCube::getGlobalPosition() {
    Eigen::Affine3f pos = coordinateFrame; 
    pos.translate(position);
    Eigen::Vector3f vec(pos.translation());
    return vec;
}


Eigen::Quaternionf SelectionCube::getGlobalRotation() {
    Eigen::Quaternionf quat(coordinateFrame.rotation());
    return quat;
}

