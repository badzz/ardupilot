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

#ifndef __AP_FRSKY_BACKEND_H__
#define __AP_FRSKY_BACKEND_H__

#include <AP_Frsky_Telem.h>
#include <AP_Frsky.h>

class AP_Frsky_Backend
{
public:
    AP_Frsky_Backend (AP_Frsky_Telem &frontend);
    virtual void init (const AP_SerialManager& serial_manager) = 0;
    
    // send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
    //  should be called by main program at 50hz to allow poll for serial bytes
    //  coming from the receiver for the SPort protocol
    virtual void send_frames(uint8_t control_mode) = 0 ;
    virtual void frsky_send_data(uint8_t id, int16_t data) = 0;

protected:
    AP_Frsky_Telem &_frontend ; // reference to the front end which holds parameters

    bool _initialised_uart;                 // true when we have detected the protocol and UART has been initialised

    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver

    // methods to convert flight controller data to frsky telemetry format
    void calc_baro_alt();
    float frsky_format_gps(float dec);
    void calc_gps_position();
    void calc_gps_position_sport();
    void calc_battery();
    void calc_gps_sats();

    // methods to send individual pieces of data down telemetry link
    void send_gps_sats(void);
    void send_mode(void);
    void send_baro_alt_m(void);
    void send_baro_alt_cm(void);
    void send_batt_remain(void);
    void send_batt_volts(void);
    void send_current(void);
    void send_prearm_error(void);
    void send_heading(void);
    void send_gps_lat_dd(void);
    void send_gps_lat_mm(void);
    void send_gps_lat_ns(void);
    void send_gps_lon_dd(void);
    void send_gps_lon_mm(void);
    void send_gps_lon_ew(void);
    void send_gps_speed_meter(void);
    void send_gps_speed_cm(void);
    void send_gps_alt_meter(void);
    void send_gps_alt_cm(void);

    uint16_t _batt_remaining;
    uint16_t _batt_volts;
    uint16_t _batt_amps;

    uint16_t gps_sats;

    bool _pos_gps_ok;
    uint16_t _course_in_degrees;
    //GPS information in Dport format
    char _lat_ns, _lon_ew;
    uint16_t _latdddmm;
    uint16_t _latmmmm;
    uint16_t _londddmm;
    uint16_t _lonmmmm;	
    uint16_t _alt_gps_meters;
    uint16_t _alt_gps_cm;
    int16_t _speed_in_meter;
    uint16_t _speed_in_centimeter;

    int16_t _baro_alt_meters;
    uint16_t _baro_alt_cm;

    uint8_t _mode; 
};
#endif
