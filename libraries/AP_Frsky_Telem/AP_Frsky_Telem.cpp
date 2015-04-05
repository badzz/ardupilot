// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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

/* 
   FRSKY Telemetry library
*/
#include <AP_Frsky_Telem.h>
#include <AP_Frsky_Backend.h>
#include <AP_Frsky_sport.h>
#include <AP_Frsky_dport.h>
extern const AP_HAL::HAL& hal;

//constructor
AP_Frsky_Telem::AP_Frsky_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery) :
    _ahrs(ahrs),
    _battery(battery),
    _backend (NULL)
    {}

// init - perform require initialisation including detecting which protocol to use
void AP_Frsky_Telem::init(const AP_SerialManager& serial_manager)
{
    // check for FRSky_DPort
    if ((serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_DPort, 0))) {
        // initialize dport backend
        _backend = new AP_Frsky_dport (*this);
    } else if ((serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_SPort, 0))) {
        // initialize sport backend
        _backend = new AP_Frsky_sport (*this);
    }
    if (_backend != NULL){
        _backend->init (serial_manager);
    }
}

/*
  send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
  should be called by main program at 50hz to allow poll for serial bytes
  coming from the receiver for the SPort protocol
*/
void AP_Frsky_Telem::send_frames(uint8_t control_mode)
{
    // return immediately if not initialised
    if (_backend == NULL) {
        return;
    }
    _backend->send_frames (control_mode);
}

