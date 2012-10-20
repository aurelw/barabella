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

#include "utils.h"

Eigen::Affine3f affineFromPlane(pcl::ModelCoefficients::Ptr coe) {
    //FIXME might not be correct. tiny rotation offest!
    float x = -coe->values[0];    
    float y = -coe->values[1];    
    float z = -coe->values[2];    
    float d = coe->values[3];    

    Eigen::Vector3f normal(x,y,z);
    //Eigen::Vector3f normal(0,1,1);
    normal.normalize();
    Eigen::Vector3f axis(0.0,1.0,0.0);
    Eigen::Vector3f avec = normal.cross(axis);
    // DON'T rotate with a non unit vector!!!!
    avec.normalize();
    Eigen::Vector3f translation = normal * d * -1.0;

    float angle = acos(normal.dot(axis));
    std::cout << "angle: " << angle << std::endl;
    std::cout << "avec: " << avec << std::endl;

    Eigen::Affine3f m;
    m.setIdentity(); 
    m.rotate(Eigen::AngleAxisf(angle, avec));
    m.translate(translation);
    
    m = m.inverse();

    return m;
}


Eigen::Quaternionf rotationFromAffine(Eigen::Affine3f aff) {
    Eigen::Quaternionf quat(aff.rotation());
    return quat; 
}
