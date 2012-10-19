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
 *   along with Barabella.  If not, see <http://www.gnu.org/licenses/>. */

#include "clip.h"

#include <iterator>
#include <vector>
#include <algorithm>

#include <pcl/common/time.h>

#include "barabella_config.h"

using namespace std;
using namespace boost::filesystem;

void Clip::setDirectory(string dir) {
    directory = dir;
}


void Clip::load() {
    frameCounter = frameFiles.size();
    frameFiles.clear();
    copy(directory_iterator(directory), 
            directory_iterator(), back_inserter(frameFiles));
    //boost directory operator is not sorting, so do it by hand
    sort(frameFiles.begin(), frameFiles.end());
}


void Clip::save() {

}


void Clip::startRecording(KinectInterface* kif) {
    isRecording = true;
    frameCounter = 0;
    kinectIf = kif;

    kinectIf->registerObserver(this);
}


void Clip::endRecording() {
    isRecording = false;
}


void Clip::frameEvent(PointCloudConstPtr cloud) {
    if (isRecording) {
        //copy the cloud
        //FIXME buffer
        //PointCloudPtr frame ( new PointCloud(*cloud));
        
        /* the filename */
        stringstream ss;
        //FIXME time stamp from cloud header
        ss << directory << "frame_" 
            << boost::posix_time::to_iso_string (
                    boost::posix_time::microsec_clock::local_time())
            << ".pcd";

        saveCloud(cloud, ss.str());
        frameCounter++;
    }
}


void Clip::saveCloud(PointCloudConstPtr cloud, const string& fname) {
    writer.writeBinaryCompressed<PointT>(fname, *cloud);
}


void Clip::begin() {
    frameIt = frameFiles.begin();
}


Clip::PointCloudPtr Clip::next() {
    if (frameIt != frameFiles.end()) {
        /* get current path to file and convert to filename */
        path frameFile = *frameIt;
        frameIt++;
        string filename = frameFile.string();

#ifdef BB_VERBOSE
        std::cout << "Loading pcd: " << filename << std::endl;
#endif
        
        /* load pcd file */
        PointCloudPtr cloud (new PointCloud);
        pcl::io::loadPCDFile<PointT>(filename, *cloud);
        lastCloud = cloud;
        return cloud;
    }

    return lastCloud;
}


bool Clip::end() {
    return frameIt == frameFiles.end();
}

