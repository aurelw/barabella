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

#include "clip_player.h"


void ClipPlayer::setClip(Clip *c) {
    clip = c;
    isClipInit = false;
}


ClipPlayer::PointCloudConstPtr ClipPlayer::getLastCloud() {
    return lastCloud;
}


void ClipPlayer::start() {
    if (!isClipInit) {
        isClipInit = true;
        clip->load();
        clip->begin();
    }

    if (!threadRunning) {
        threadRunning = true;
        thread = new boost::thread(boost::bind(&ClipPlayer::runPlayer, this));
    }

    isPlay = true;
}


void ClipPlayer::stop() {
    /* terminate the thread */
    stopThread = true; 
    thread->join();
    delete thread;
    stopThread = false;
    
    /* set flafs */
    isPlay = false;
    threadRunning = false;
}


void ClipPlayer::pause() {
    isPlay = false;
}


void ClipPlayer::reset() {
    /* restart */
    stop();
    clip->begin();
    start();
}


void ClipPlayer::runPlayer() {
    /* variables for proper timing in playback */
    bool first_frame = true;
    long lstamp = 0;
    long cstamp = 0;
    boost::system_time ltime;
    boost::system_time ctime;


    while (!stopThread) {
        if (isPlay) {
            /* get next frame */
            lastCloud = clip->next();

            /* get time stamp and time difference */
            lstamp = cstamp;
            //FIXME get timestamp from frame
            cstamp = cstamp + 33; // ~30 fps;

            /* first frame - no sleep */
            if (first_frame) {
                ctime = boost::get_system_time();
                first_frame = false;
            } else {
                /* sleep time duration till next frame */
                ltime = ctime;
                ctime = boost::get_system_time();
                boost::posix_time::milliseconds dstamp(cstamp - lstamp);
                boost::thread::sleep(ltime + dstamp);
            }

            /* frame evenet */
            frameEvent(lastCloud);
        } else {
            usleep(50);
        }
    }
}

