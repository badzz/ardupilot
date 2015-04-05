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

#ifndef __AP_FRSKY_SPORT_H__
#define __AP_FRSKY_SPORT_H__

#include <AP_Frsky_Backend.h>
#include <AP_Frsky.h>

#define SPORT_START_FRAME 0x7E

class AP_Frsky_sport : public AP_Frsky_Backend
{    
public:
    AP_Frsky_sport (AP_Frsky_Telem &frontend);

    //init - performs any required initialisation for this instance
    void init(const AP_SerialManager& serial_manager);
    
    // send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
    //  should be called by main program at 50hz to allow poll for serial bytes
    //  coming from the receiver for the SPort protocol
    void send_frames(uint8_t control_mode);

private:
    // init_uart_for_sport - initialise uart for use by sport
    void init_uart_for_sport();

    // sport_tick - main call to send updates to transmitter when protocol is FrSkySPORT
    //  called by scheduler at a high rate
    void sport_tick();

    // methods related to the nuts-and-bolts of sending data
    void calc_crc(uint8_t byte);
    void send_crc();
    void frsky_send_byte(uint8_t value);
    void frsky_send_sport_prim();
    void frsky_send_data16(uint16_t id, int32_t data);
    void frsky_send_data(uint8_t id, int16_t data);

    uint8_t _fas_call;
    uint8_t _gps_call;
    uint8_t _vario_call;
    uint8_t _various_call;

    bool _battery_data_ready;
    bool _sats_data_ready;
    bool _gps_data_ready;
    bool _baro_data_ready;
    bool _mode_data_ready;

    uint8_t _sport_status;
    uint16_t _crc;

};
#endif
