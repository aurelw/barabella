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

#include "boost/smart_ptr.hpp"

#include "clip.h"

#ifndef __TRACKER_H__
#define __TRACKER_H__

class Trace {

    public:
        typedef boost::shared_ptr<Trace> Ptr;

    public:

        Eigen::Affine3f coordinateFrame;
        std::vector<Eigen::Affine3f> transforms;
        Clip* clip;

        void writeToCSV(std::string fpath);

    protected:
};


class Tracker {

    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

    public:

        Tracker() :
            trace(new Trace)
        {
        }

        /* tracking options */
        virtual void setClip(Clip* cl);
        virtual void setCoordinateFrame(Eigen::Affine3f t);

        /* processing */
        virtual void initTracker() = 0;
        virtual void processFrame() = 0;
        virtual void processAllFrames() = 0;
        virtual bool finished() = 0;

        /* results */
        virtual Trace::Ptr getTrace();

    protected:
        Clip* clip;
        Eigen::Affine3f coordinateFrame;

        Trace::Ptr trace;
};


#endif

