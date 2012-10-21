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

#include <iostream>

#include "frame_observer.h"
#include "clip.h"


#include <boost/thread.hpp>

#ifndef __CLIP_PLAYER_H__
#define __CLIP_PLAYER_H__


class ClipPlayer : public FrameProvider {

    public:

        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

        ClipPlayer() :
            isClipInit(false),
            isPlay(false),
            threadRunning(false),
            stopThread(false)
        {
        }

        PointCloudConstPtr getLastCloud();

        void setClip(Clip* c);
        void start();
        void stop();
        void pause();
        void reset();

    private:
        Clip* clip;
        PointCloudConstPtr lastCloud;

        /* thread */
        boost::thread* thread;
        void runPlayer();

        /* state flags */
        bool isClipInit;
        bool isPlay;
        bool threadRunning;
        bool stopThread;

};

#endif
