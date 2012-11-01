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
    //FIXME proper timing and load buffer
    while (!stopThread) {
        if (isPlay) {
            lastCloud = clip->next();
            frameEvent(lastCloud);
        } else {
            usleep(50);
        }
    }
}

