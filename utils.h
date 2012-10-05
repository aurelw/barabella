#include <stdio.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>


Eigen::Affine3f affineFromPlane(pcl::ModelCoefficients::Ptr coe);
