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


void IcpFramedTracker::setTemplate(PointCloudConstPtr tCloud) {
    templateCloud = tCloud;
}


void IcpFramedTracker::setInitialSearchWindow(SelectionCube* sCube) {
    searchWindow = sCube;
}


void IcpFramedTracker::setWindowBorder(float bSize) {
    boarderSize = bSize;
}


void IcpFramedTracker::initTracker() {
    /* copy and extend the search window */
    SelectionCube* newCube = new SelectionCube(*searchWindow);
    newCube->setSx( newCube->getSx() + boarderSize );
    newCube->setSy( newCube->getSy() + boarderSize );
    newCube->setSz( newCube->getSz() + boarderSize );
    searchWindow = newCube;
}


void IcpFramedTracker::processFrame() {
    std::cout << "process frame" << std::endl;
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
    //trans = trans.inverse();
    trace.push_back(trans);

    /* move the search window to this frame */
    Eigen::Affine3f sWindowTrans = searchWindow->getCoordinateFrame();
    sWindowTrans = trans * sWindowTrans;
    searchWindow->setCoordinateFrame(sWindowTrans);

    /* move the template */
    PointCloudPtr transformedTemplate( new PointCloud);
    pcl::transformPointCloud<PointT>(*templateCloud, 
            *transformedTemplate, trans);
    templateCloud = transformedTemplate;
}


void IcpFramedTracker::processAllFrames() {
    while (!clip->end()) {
        processFrame();
    }
}


SelectionCube* IcpFramedTracker::getSearchWindow() {
    return searchWindow;
}


IcpFramedTracker::PointCloudConstPtr IcpFramedTracker::getCurrentCloud() {
    return clipCloud;
}

