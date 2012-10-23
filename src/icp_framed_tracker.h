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

#include <pcl/registration/icp.h>

#include "tracker.h"
#include "selection_cube.h"


#ifndef __ICP_FRAMED_TRACKER__
#define __ICP_FRAMED_TRACKER__

class IcpFramedTracker : public Tracker {

    public:

        IcpFramedTracker() :
            boarderSize(0.3)
        {
        }

        /* tracking options */
        //mandatory
        virtual void setTemplate(PointCloudConstPtr tCloud);
        virtual void setInitialSearchWindow(SelectionCube* sCube);
        //optional
        virtual void setWindowBorder(float bSize);

        /* processing */
        virtual void initTracker();
        virtual void processFrame();
        virtual void processAllFrames();

        /* results for current frame */
        SelectionCube* getSearchWindow();
        PointCloudConstPtr getCurrentCloud();

    protected:
        PointCloudConstPtr templateCloud;
        PointCloudConstPtr clipCloud;
        SelectionCube* searchWindow;
        float boarderSize;

        /* tracking results */
        std::vector<Eigen::Affine3f> trace;
};

#endif

