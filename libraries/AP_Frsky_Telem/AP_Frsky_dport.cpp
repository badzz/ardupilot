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

#include <AP_Frsky_dport.h>
extern const AP_HAL::HAL& hal;

AP_Frsky_dport::AP_Frsky_dport (AP_Frsky_Telem &frontend) :
    AP_Frsky_Backend(frontend),
    _last_frame1_ms(0),
    _last_frame2_ms(0)
{}

void AP_Frsky_dport::init (const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_DPort, 0);
    if (_port != NULL){
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _initialised_uart = true; 
    }
}

/*
  send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
  should be called by main program at 50hz to allow poll for serial bytes
  coming from the receiver for the SPort protocol
*/
void AP_Frsky_dport::send_frames(uint8_t control_mode)
{
    // return immediately if not initialised
    if (!_initialised_uart)
        return;
    _mode=control_mode;
    send_hub_frame();
}

/*
  send_hub_frame - send frame1 and frame2 when protocol is FrSkyDPORT
  frame 1 is sent every 200ms with baro alt, nb sats, batt volts and amp, control_mode
  frame 2 is sent every second with gps position data
*/
void AP_Frsky_dport::send_hub_frame()
{    
    uint32_t now = hal.scheduler->millis();

    // send frame1 every 200ms
    if (now - _last_frame1_ms > 200) {
        _last_frame1_ms = now;
        calc_gps_sats();
        send_gps_sats();
        send_mode();

        calc_battery();
        send_batt_remain();
        send_batt_volts();
        send_current();

        calc_baro_alt();
        send_baro_alt_m();
        send_baro_alt_cm();
    }
    // send frame2 every second
    if (now - _last_frame2_ms > 1000) {
        _last_frame2_ms = now;
        send_heading();
        calc_gps_position();
        if (_pos_gps_ok) {
            send_gps_lat_dd();
            send_gps_lat_mm();
            send_gps_lat_ns();
            send_gps_lon_dd();
            send_gps_lon_mm();
            send_gps_lon_ew();
            send_gps_speed_meter();
            send_gps_speed_cm();
            send_gps_alt_meter();
            send_gps_alt_cm();
        }
    }
}


/*
  send 1 byte and do the byte stuffing Frsky stuff 
  This can send more than 1 byte eventually
*/
void AP_Frsky_dport::frsky_send_byte(uint8_t value)
{
    const uint8_t x5E[] = { 0x5D, 0x3E };
    const uint8_t x5D[] = { 0x5D, 0x3D };
    switch (value) {
    case 0x5E:
        _port->write( x5E, sizeof(x5E));
        break;
            
    case 0x5D:
        _port->write( x5D, sizeof(x5D));
        break;
            
    default:
        _port->write(&value, sizeof(value));
        break;
    }
}

/**
 * Sends a 0x5E start/stop byte.
 */
void AP_Frsky_dport::frsky_send_hub_startstop()
{
    static const uint8_t c = 0x5E;
    _port->write(&c, sizeof(c));
}

/**
 * Sends one data id/value pair.
 */
void  AP_Frsky_dport::frsky_send_data(uint8_t id, int16_t data)
{
    /* Cast data to unsigned, because signed shift might behave incorrectly */
    uint16_t udata = data;

    frsky_send_hub_startstop();
    frsky_send_byte(id);

    frsky_send_byte(udata); /* LSB */
    frsky_send_byte(udata >> 8); /* MSB */
}

