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

#include <string>
#include <iostream>
#include <stdio.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>

#include "frame_observer.h"
#include "kinect_interface.h"
#include "producer_consumer_queue.h"

#ifndef __CLIP_H__
#define __CLIP_H__

class Clip : public FrameObserver {

    public:

        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<Clip> Ptr;

    public:

        Clip() :
            loadThread(NULL),
            isRecording(false),
            loadThreadRunning(false),
            frameCounter(0)
        {
        };

        ~Clip() {
            if (loadThreadRunning) {
                loadThread->interrupt();
                loadThread->join();
            }
        }

        void setDirectory(std::string dir);
        void load();
        void save();
        void startRecording(KinectInterface *kif);
        void endRecording();
        int numFrames();

        void begin();
        PointCloudPtr next();
        bool end();

        /* addition clip info */
        long frameTimestamp();
        

        void frameEvent(PointCloudConstPtr cloud);

    private:
        /* recording */
        KinectInterface* kinectIf;
        bool isRecording;
        pcl::PCDWriter writer;
        int frameCounter;

        /* loading */
        std::string directory;
        std::vector<boost::filesystem::path> frameFiles;
        std::vector<boost::filesystem::path>::iterator frameIt;
        PointCloudPtr lastCloud;
        ProducerConsumerQueue<PointCloudPtr> loadQueue;
        boost::thread* loadThread;
        bool loadThreadRunning;
        void loadFilesToQueue();

        /* saving */
        void saveCloud(PointCloudConstPtr cloud, const std::string& fname);

};
#endif

