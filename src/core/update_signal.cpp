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


#include "update_signal.h"

UpdateSignal::connection_t UpdateSignal::connect(signal_t::slot_function_type subscriber) {
#ifdef BB_FLOOD
   std::cout << "slot connected" << std::endl;
#endif
    return m_sig.connect(subscriber);
}


void UpdateSignal::disconnect(connection_t subscriber) {
    subscriber.disconnect();
}


void UpdateSignal::update() {
   m_sig(); 
#ifdef BB_FLOOD
   std::cout << "update()" << std::endl;
#endif
}

