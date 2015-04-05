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

#include <AP_Frsky_Backend.h>


AP_Frsky_Backend::AP_Frsky_Backend (AP_Frsky_Telem &frontend) :
    _frontend (frontend),
    _initialised_uart(false),
    _batt_remaining(0),
    _batt_volts(0),
    _batt_amps(0),
    gps_sats(0),
    _pos_gps_ok(false),
    _course_in_degrees(0),
    _lat_ns(0),
    _lon_ew(0),
    _latdddmm(0),
    _latmmmm(0),
    _londddmm(0),
    _lonmmmm(0),
    _alt_gps_meters(0),
    _alt_gps_cm(0),
    _speed_in_meter(0),
    _speed_in_centimeter(0),
    _baro_alt_meters(0),
    _baro_alt_cm(0),
    _mode(0)
{}



/*
 * calc_baro_alt : send altitude in Meters based on ahrs estimate
 */
void AP_Frsky_Backend::calc_baro_alt()
{
    struct Location loc;
    float baro_alt = 0; // in meters
    bool posok = _frontend._ahrs.get_position(loc);
    if  (posok) {
        baro_alt = loc.alt * 0.01f; // convert to meters
        if (!loc.flags.relative_alt) {
            baro_alt -= _frontend._ahrs.get_home().alt * 0.01f; // subtract home if set
        }
    }
    /*
      Note that this isn't actually barometric altitude, it is the
      inertial nav estimate of altitude above home.
    */
    _baro_alt_meters = (int16_t)baro_alt;
    _baro_alt_cm = (baro_alt - abs(_baro_alt_meters)) * 100;
}

/**
 * Formats the decimal latitude/longitude to the required degrees/minutes.
 */
float  AP_Frsky_Backend::frsky_format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare latitude and longitude information stored in member variables
 */

void AP_Frsky_Backend::calc_gps_position()
{
    _course_in_degrees = (_frontend._ahrs.yaw_sensor / 100) % 360;

    const AP_GPS &gps = _frontend._ahrs.get_gps();
    float lat;
    float lon ;
    float alt ;
    float speed;
    _pos_gps_ok = (gps.status() >= 3);
    if (_pos_gps_ok) {
        Location loc = gps.location();//get gps instance 0
    
        lat = frsky_format_gps(fabsf(loc.lat/10000000.0));
        _latdddmm = lat;
        _latmmmm = (lat - _latdddmm) * 10000;
        _lat_ns = (loc.lat < 0) ? 'S' : 'N';
        
        lon = frsky_format_gps(fabsf(loc.lng/10000000.0));
        _londddmm = lon;
        _lonmmmm = (lon - _londddmm) * 10000;
        _lon_ew = (loc.lng < 0) ? 'W' : 'E';
        
        alt = loc.alt * 0.01f;
        _alt_gps_meters = (int16_t)alt;
        _alt_gps_cm = (alt - abs(_alt_gps_meters)) * 100;
        
        speed = gps.ground_speed();
        _speed_in_meter = speed;
        _speed_in_centimeter = (speed - _speed_in_meter) * 100;
    } else {
        _latdddmm = 0;
        _latmmmm = 0;
        _lat_ns = 0;
        _londddmm = 0;
        _lonmmmm = 0;
        _alt_gps_meters = 0;
        _alt_gps_cm = 0;
        _speed_in_meter = 0;
        _speed_in_centimeter = 0;
    }
}

/*
 * prepare battery information stored in member variables
 */
void AP_Frsky_Backend::calc_battery()
{
    _batt_remaining = roundf(_frontend._battery.capacity_remaining_pct());
    _batt_volts = roundf(_frontend._battery.voltage() * 10.0f);
    _batt_amps = roundf(_frontend._battery.current_amps() * 10.0f);
}

/*
 * prepare sats information stored in member variables
 */
void AP_Frsky_Backend::calc_gps_sats()
{
    // GPS status is sent as num_sats*10 + status, to fit into a uint8_t
    const AP_GPS &gps = _frontend._ahrs.get_gps();
    gps_sats = gps.num_sats() * 10 + gps.status();
}

/*
 * send number of gps satellite and gps status eg: 73 means 7 satellite and 3d lock
 */
void AP_Frsky_Backend::send_gps_sats()
{
    frsky_send_data(FRSKY_ID_TEMP2, gps_sats);
}

/*
 * send control_mode as Temperature 1 (TEMP1)
 */
void AP_Frsky_Backend::send_mode(void)
{
    frsky_send_data(FRSKY_ID_TEMP1, _mode);
}

/*
 * send barometer altitude integer part . Initialize baro altitude
 */
void AP_Frsky_Backend::send_baro_alt_m(void)
{
    frsky_send_data(FRSKY_ID_BARO_ALT_BP, _baro_alt_meters);
}

/*
 * send barometer altitude decimal part
 */
void AP_Frsky_Backend::send_baro_alt_cm(void)
{
    frsky_send_data(FRSKY_ID_BARO_ALT_AP, _baro_alt_cm);
}

/*
 * send battery remaining
 */
void AP_Frsky_Backend::send_batt_remain(void)
{
    frsky_send_data(FRSKY_ID_FUEL, _batt_remaining);
}

/*
 * send battery voltage 
 */
void AP_Frsky_Backend::send_batt_volts(void)
{
    frsky_send_data(FRSKY_ID_VFAS, _batt_volts);
}

/*
 * send current consumptiom 
 */
void AP_Frsky_Backend::send_current(void)
{
    frsky_send_data(FRSKY_ID_CURRENT, _batt_amps);
}

/*
 * send heading in degree based on AHRS and not GPS 
 */
void AP_Frsky_Backend::send_heading(void)
{
    frsky_send_data(FRSKY_ID_GPS_COURS_BP, _course_in_degrees);
}

/*
 * send gps lattitude degree and minute integer part; Initialize gps info
 */
void AP_Frsky_Backend::send_gps_lat_dd(void)
{
    frsky_send_data(FRSKY_ID_GPS_LAT_BP, _latdddmm);
}

/*
 * send gps lattitude minutes decimal part 
 */
void AP_Frsky_Backend::send_gps_lat_mm(void)
{
    frsky_send_data(FRSKY_ID_GPS_LAT_AP, _latmmmm);
}

/*
 * send gps North / South information 
 */
void AP_Frsky_Backend::send_gps_lat_ns(void)
{
    frsky_send_data(FRSKY_ID_GPS_LAT_NS, _lat_ns);
}

/*
 * send gps longitude degree and minute integer part 
 */
void AP_Frsky_Backend::send_gps_lon_dd(void)
{
    frsky_send_data(FRSKY_ID_GPS_LONG_BP, _londddmm);
}

/*
 * send gps longitude minutes decimal part 
 */
void AP_Frsky_Backend::send_gps_lon_mm(void)
{
    frsky_send_data(FRSKY_ID_GPS_LONG_AP, _lonmmmm);
}

/*
 * send gps East / West information 
 */
void AP_Frsky_Backend::send_gps_lon_ew(void)
{
    frsky_send_data(FRSKY_ID_GPS_LONG_EW, _lon_ew);
}

/*
 * send gps speed integer part
 */
void AP_Frsky_Backend::send_gps_speed_meter(void)
{
    frsky_send_data(FRSKY_ID_GPS_SPEED_BP, _speed_in_meter);
}

/*
 * send gps speed decimal part
 */
void AP_Frsky_Backend::send_gps_speed_cm(void)
{
    frsky_send_data(FRSKY_ID_GPS_SPEED_AP, _speed_in_centimeter);
}

/*
 * send gps altitude integer part
 */
void AP_Frsky_Backend::send_gps_alt_meter(void)
{
    frsky_send_data(FRSKY_ID_GPS_ALT_BP, _alt_gps_meters);
}

/*
 * send gps altitude decimals
 */
void AP_Frsky_Backend::send_gps_alt_cm(void)
{
    frsky_send_data(FRSKY_ID_GPS_ALT_AP, _alt_gps_cm);
}
