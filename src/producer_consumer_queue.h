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

#include <queue>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#ifndef __PRODUCER_CONSUMER_QUEUE_H__
#define __PRODUCER_CONSUMER_QUEUE_H__

template <class T>
class ProducerConsumerQueue {

    public:

        ProducerConsumerQueue(int size=5, int delayMillis=10) :
            maxSize(size),
            sleepDelay(delayMillis),
            noMoreInput(false),
            noMoreElements(false)
        {
        }

        void push(T element, bool isLast);
        T pop(); 

        bool isDone();
        void reset();

    private:

        std::queue<T> queue;
        int maxSize;
        int sleepDelay;

        boost::mutex mutex;
        bool noMoreInput;
        bool noMoreElements;

};

#endif

