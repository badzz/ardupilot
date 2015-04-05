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

#include <AP_Frsky_sport.h>
extern const AP_HAL::HAL& hal;


AP_Frsky_sport::AP_Frsky_sport (AP_Frsky_Telem &frontend):
    AP_Frsky_Backend(frontend),
    _fas_call(0),
    _gps_call(0),
    _vario_call(0),
    _various_call(0),
    _battery_data_ready(false),
    _sats_data_ready(false),
    _gps_data_ready(false),
    _baro_data_ready(false),
    _mode_data_ready(false),
    _sport_status(0),
    _crc(0)
{}

void AP_Frsky_sport::init (const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_SPort, 0);
    if (_port != NULL){
        _gps_call = 0;
        _fas_call = 0;
        _vario_call = 0 ;
        _various_call = 0 ;
        _gps_data_ready = false;
        _battery_data_ready = false;
        _baro_data_ready = false;
        _mode_data_ready = false;
        _sats_data_ready = false;
        _sport_status = 0;
        hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&AP_Frsky_sport::sport_tick));
    }
}


    


/*
  send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
  should be called by main program at 50hz to allow poll for serial bytes
  coming from the receiver for the SPort protocol
*/
void AP_Frsky_sport::send_frames(uint8_t control_mode)
{
    // return immediately if not initialised
    if (!_initialised_uart) {
        return;
    }
    if (!_mode_data_ready) {
        _mode=control_mode;
        _mode_data_ready = true;
    }
    if (!_baro_data_ready) {
        calc_baro_alt();
        _baro_data_ready = true;
    }
    if (!_gps_data_ready) {
        calc_gps_position();
        _gps_data_ready = true;
    }
    if (!_sats_data_ready) {
        calc_gps_sats();
        _sats_data_ready = true;
    }
    if (!_battery_data_ready) {
        calc_battery();
        _battery_data_ready = true;
    }
}


/*
  init_uart_for_sport - initialise uart for use by sport
  this must be called from sport_tick which is called from the 1khz scheduler
  because the UART begin must be called from the same thread as it is used from
 */
void AP_Frsky_sport::init_uart_for_sport()
{
    // initialise uart
    _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
    _initialised_uart = true;
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

}

/*
  sport_tick - main call to send updates to transmitter when protocol is FrSkySPORT
  called by scheduler at a high rate
*/
void AP_Frsky_sport::sport_tick(void)
{

    // check UART has been initialised
    if (!_initialised_uart) {
        init_uart_for_sport();
    }
    
    int16_t numc;
    numc = _port->available();

    // check if available is negative
    if (numc < 0) {
        return;
    }

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }

    for (int16_t i = 0; i < numc; i++) {
        int16_t readbyte = _port->read();
        if (_sport_status == 0) {
            if  (readbyte == SPORT_START_FRAME) {
                _sport_status = 1;
            }
        } else {
            switch (readbyte) {
            case DATA_ID_FAS:
                if (_battery_data_ready) {
                    switch (_fas_call) {
                    case 0:
                        send_batt_volts();
                        break;
                    case 1:
                        send_current();
                        break;
                    }
                    _fas_call++;
                    if (_fas_call > 1) {
                        _fas_call = 0;
                    }
                    _battery_data_ready = false;
                }
                break;
            case DATA_ID_GPS:
                if (_gps_data_ready) {
                    switch (_gps_call) {
                    case 0:
                        send_gps_lat_dd();
                        break;
                    case 1:
                        send_gps_lat_mm();
                        break;
                    case 2:
                        send_gps_lat_ns();
                        break;
                    case 3:
                        send_gps_lon_dd();
                        break;
                    case 4:
                        send_gps_lon_mm();
                        break;
                    case 5:
                        send_gps_lon_ew();
                        break;
                    case 6:
                        send_gps_speed_meter();
                        break;
                    case 7:
                        send_gps_speed_cm();
                        break;
                    case 8:
                        send_gps_alt_meter();
                        break;
                    case 9:
                        send_gps_alt_cm();
                        break;
                    case 10:
                        send_heading();
                        break;
                    }

                    _gps_call++;
                    if (_gps_call > 10) {
                        _gps_call = 0;
                        _gps_data_ready = false;
                    }
                }
                break;
            case DATA_ID_VARIO:
                if (_baro_data_ready) {
                    switch (_vario_call) {
                    case 0 :
                        send_baro_alt_m();
                        break;
                    case 1:
                        send_baro_alt_cm();
                        break;
                    }
                    _vario_call ++;
                    if (_vario_call > 1) {
                        _vario_call = 0;
                        _baro_data_ready = false;
                    }
                }
                break;
            case DATA_ID_SP2UR:
                switch (_various_call) {
                case 0 :
                    if ( _sats_data_ready ) {
                        send_gps_sats();
                        _sats_data_ready = false;
                    }
                    break;
                case 1:
                    if ( _mode_data_ready ) {
                        send_mode();
                        _mode_data_ready = false;
                    }
                    break;
                }
                _various_call++;
                if (_various_call > 1) {
                    _various_call = 0;
                }
                break;
            }
            _sport_status = 0;
        }
    }
}

/* 
   simple crc implementation for FRSKY telem S-PORT
*/
void AP_Frsky_sport::calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0x00ff;
    _crc += _crc >> 8; //0-0FF
    _crc &= 0x00ff;
}

/*
 * send the crc at the end of the S-PORT frame
 */
void AP_Frsky_sport::send_crc() 
{
    frsky_send_byte(0x00ff-_crc);
    _crc = 0;
}

void AP_Frsky_sport::frsky_send_byte(uint8_t value)
{
    calc_crc(value);
    const uint8_t x7E[] = { 0x7D, 0x5E };
    const uint8_t x7D[] = { 0x7D, 0x5D };
    switch (value) {
    case 0x7E:
        _port->write( x7E, sizeof(x7E));
        break;
        
    case 0x7D:
        _port->write( x7D, sizeof(x7D));
        break;
        
    default:
        _port->write(&value, sizeof(value));
        break;
    }
}

/*
  add sport protocol for frsky tx module 
*/
void AP_Frsky_sport::frsky_send_sport_prim()
{
    static const uint8_t c = 0x10;
    frsky_send_byte(c);
}


/**
 * Sends one data id/value pair.
 */
void  AP_Frsky_sport::frsky_send_data16(uint16_t id, int32_t data)
{
    /* Cast data to unsigned, because signed shift might behave incorrectly */
    uint32_t udata = data;

    frsky_send_sport_prim();
    frsky_send_byte(id);
    frsky_send_byte(id >> 8); 
    
    frsky_send_byte(udata); /* LSB */
    frsky_send_byte(udata >> 8); /* MSB */
    frsky_send_byte(udata >> 16); /* MSB */
    frsky_send_byte(udata >> 24); /* MSB */

    send_crc();
}

/**
 * Sends one data id/value pair.
 */
void  AP_Frsky_sport::frsky_send_data(uint8_t id, int16_t data)
{
    frsky_send_data16((uint16_t) id , (int32_t) data);
}
