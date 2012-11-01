
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

#include "frame_observer.h"

#include <boost/foreach.hpp>


void FrameProvider::registerObserver(FrameObserver* observer) {
    observers.push_back(observer);
}


void FrameProvider::frameEvent(CloudConstPtr cloud) {
    BOOST_FOREACH(FrameObserver* obs, observers) {
        obs->frameEvent(cloud);
    }
}

