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

#include "boost/smart_ptr.hpp"

#include "barabella_config.h"

using namespace std;
using namespace boost::filesystem;

void Clip::setDirectory(string dir) {
    directory = dir;
}


void Clip::load() {
    frameFiles.clear();
    copy(directory_iterator(directory), 
            directory_iterator(), back_inserter(frameFiles));
    //boost directory operator is not sorting, so do it by hand
    sort(frameFiles.begin(), frameFiles.end());
    frameCounter = frameFiles.size();
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

#ifdef BB_VERBOSE
        std::cout << "recorded frame - stamp/seq/id: " << 
            cloud->header.stamp    << "/" << 
            cloud->header.seq      << "/" << 
            cloud->header.frame_id << std::endl;
#endif
        
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
    /* reset and init queue and producer thread */
    if (loadThreadRunning) {
        loadThread->interrupt();
        loadThread->join();
    }
    loadQueue.reset();

    /* start producer thread */
    loadThread = new boost::thread(boost::bind(&Clip::loadFilesToQueue, this));
    loadThreadRunning = true;
}


void Clip::loadFilesToQueue() {
    frameIt = frameFiles.begin();
    while (frameIt != frameFiles.end()) {

        /* load the pcd file */
        path frameFile = *frameIt;
        string filename = frameFile.string();
        PointCloudPtr cloud (new PointCloud);
        pcl::io::loadPCDFile<PointT>(filename, *cloud);

        /* insert into queue */
        if (frameIt + 1 != frameFiles.end()) {
            loadQueue.push(cloud);
        } else { // last cloud
            loadQueue.push(cloud, true);
        }

        /* to next file */
        frameIt++;
    }
    loadThreadRunning = false;
}


Clip::PointCloudPtr Clip::next() {
    if (! loadQueue.isDone()) {
        lastCloud = loadQueue.pop();
    }

    return lastCloud;
}


bool Clip::end() {
    return loadQueue.isDone();
}

