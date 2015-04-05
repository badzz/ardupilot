// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AP_FRSKY_DPORT_H__
#define __AP_FRSKY_DPORT_H__

#include <AP_Frsky_Backend.h>
#include <AP_Frsky.h>

class AP_Frsky_dport : public AP_Frsky_Backend 
{
public:
    AP_Frsky_dport (AP_Frsky_Telem &frontend);

    // init - performs any required initialisation for this instance
    void init(const AP_SerialManager& serial_manager);
    
     // send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
    //  should be called by main program at 50hz to allow poll for serial bytes
    //  coming from the receiver for the SPort protocol
    void send_frames(uint8_t control_mode);

private:
    // send_hub_frame - main transmission function when protocol is FrSkyDPORT
    void send_hub_frame();

   // methods related to the nuts-and-bolts of sending data
    void frsky_send_byte(uint8_t value);
    void frsky_send_hub_startstop();
    void frsky_send_data(uint8_t id, int16_t data);

    uint32_t _last_frame1_ms;
    uint32_t _last_frame2_ms;

};
#endif
