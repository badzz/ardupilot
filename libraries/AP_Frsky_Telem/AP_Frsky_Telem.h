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

#ifndef __AP_FRSKY_TELEM_H__
#define __AP_FRSKY_TELEM_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Baro.h>
#include <AP_BattMonitor.h>
#include <AP_SerialManager.h>

class AP_Frsky_Backend;
class AP_Frsky_sport;
class AP_Frsky_dport;


class AP_Frsky_Telem
{

    friend class AP_Frsky_Backend;
    friend class AP_Frsky_sport;
    friend class AP_Frsky_dport;

public: 
    //constructor
    AP_Frsky_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery);

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager& serial_manager);

    // send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
    //  should be called by main program at 50hz to allow poll for serial bytes
    //  coming from the receiver for the SPort protocol
    void send_frames(uint8_t control_mode);

    AP_AHRS &_ahrs;                         // reference to attitude estimate
    AP_BattMonitor &_battery;               // reference to battery monitor object

    AP_Frsky_Backend *_backend;             // reference to active backend 
};
#endif
