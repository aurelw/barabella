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

#include "icp_framed_tracker.h"


void IcpFramedTracker::setTemplate(CloudTemplate::Ptr cTemp) {
    cloudTemplate = cTemp;
    // seperate variable, because this template will be transformed
    templateCloud = cloudTemplate->cloud;
}


void IcpFramedTracker::setInitialSearchWindow(SelectionCube::Ptr sCube) {
    searchWindow = sCube;
}


void IcpFramedTracker::setWindowBorder(float bSize) {
    boarderSize = bSize;
}


void IcpFramedTracker::initTracker() {
    /* copy and extend the search window */
    SelectionCube::Ptr newCube(new SelectionCube(*searchWindow));
    newCube->setSx( newCube->getSx() + boarderSize );
    newCube->setSy( newCube->getSy() + boarderSize );
    newCube->setSz( newCube->getSz() + boarderSize );
    searchWindow = newCube;

    /* init trace */
    trace->clip = clip;
    trace->coordinateFrame = coordinateFrame;

    /* init position */
    currentTransform.setIdentity();
    currentTransform.translate(cloudTemplate->center);
}


void IcpFramedTracker::processFrame() {
    /* check if there are still frames in the clip to process */
    if (clip->end()) {
        isFinished = true;
        return;
    }

    /* get the target cloud */
    clipCloud = clip->next();
    PointCloudPtr targetCloud = searchWindow->filterCloud(*clipCloud);

    /* ICP setup */
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputCloud(templateCloud);
    icp.setInputTarget(targetCloud);
    PointCloud finalCloud;

    /* do alignment */
    icp.align(finalCloud);
#ifdef BB_VERBOSE
    std::cout << "has converged:" 
              << icp.hasConverged() << " score: " 
              << icp.getFitnessScore() << std::endl;
#endif
    Eigen::Matrix4f mat;
    mat = icp.getFinalTransformation();
    Eigen::Affine3f trans(mat);

    /* move the search window to this frame */
    Eigen::Affine3f sWindowTrans = searchWindow->getCoordinateFrame();
    sWindowTrans = trans * sWindowTrans;
    searchWindow->setCoordinateFrame(sWindowTrans);

    /* also move the template */
    PointCloudPtr transformedTemplate( new PointCloud);
    pcl::transformPointCloud<PointT>(*templateCloud, 
            *transformedTemplate, trans);
    templateCloud = transformedTemplate;

    /* results */
    currentTransform = trans * currentTransform;
    // Save the transform of the object in the 
    // LOCAL coordinate frame of the trace!
    // This will give coordinates in respect to 
    // the floor.
    trace->transforms.push_back(trace->coordinateFrame * currentTransform);
    //trace->transforms.push_back(currentTransform);
}


bool IcpFramedTracker::finished() {
    return isFinished;
}


void IcpFramedTracker::processAllFrames() {
    while (!isFinished) {
        processFrame();
    }
}


SelectionCube::Ptr IcpFramedTracker::getSearchWindow() {
    // copy so it is safe from manipulation
    SelectionCube::Ptr r(new SelectionCube(*searchWindow));
    return r;
}


IcpFramedTracker::PointCloudConstPtr IcpFramedTracker::getCurrentCloud() {
    return clipCloud;
}

