#pragma once
// MESSAGE RTK_SENSORS_DATA PACKING

#define TTALINK_MSG_ID_RTK_SENSORS_DATA 2176

TTAPACKED(
typedef struct __ttalink_rtk_sensors_data_t {
 uint8_t stas_number; /*<  .*/
 uint8_t stas_number_tracked; /*<  .*/
 uint8_t stas_number_usedinsolution; /*<  .*/
 uint16_t gps_time_week; /*<  .*/
 uint32_t gps_time_ms; /*<  .*/
 uint8_t sol_status; /*<  .*/
 uint8_t pos_type; /*<  .*/
 double rtk_lat; /*<  .*/
 double rtk_lon; /*<  .*/
 float rtk_alt; /*<  .*/
 float undulation; /*<  .*/
 float lat_std_deviation; /*<  .*/
 float lon_std_deviation; /*<  .*/
 float alt_std_deviation; /*<  .*/
 float hor_speed; /*<  .*/
 float trk_gnd; /*<  .*/
 float vert_speed; /*<  .*/
 float length; /*<  .*/
 float heading; /*<  .*/
 float pitch; /*<  .*/
 float hdg_std_deviation; /*<  .*/
 float ptch_std_deviation; /*<  .*/
 float GDOP; /*<  .*/
 float PDOP; /*<  .*/
 float HDOP; /*<  .*/
 uint32_t updateTime; /*<  .*/
 uint32_t vel_updateTime; /*<  .*/
 uint32_t pos_updateTime; /*<  .*/
 uint32_t dop_updateTime; /*<  .*/
 uint32_t heading_updateTime; /*<  .*/
 uint8_t heading_sol_status; /*<  .*/
 uint8_t heading_pos_type; /*<  .*/
 uint32_t utc_year; /*<  .*/
 uint8_t utc_month; /*<  .*/
 uint8_t utc_day; /*<  .*/
 uint8_t utc_hour; /*<  .*/
 uint8_t utc_min; /*<  .*/
 uint32_t utc_ms; /*<  .*/
 uint32_t utc_status; /*<  .*/
 uint8_t ant_status; /*<  .*/
}) ttalink_rtk_sensors_data_t;

#define TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN 130
#define TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN 130
#define TTALINK_MSG_ID_2176_LEN 130
#define TTALINK_MSG_ID_2176_MIN_LEN 130

#define TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC 161
#define TTALINK_MSG_ID_2176_CRC 161



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_RTK_SENSORS_DATA { \
    2176, \
    "RTK_SENSORS_DATA", \
    40, \
    {  { "stas_number", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rtk_sensors_data_t, stas_number) }, \
         { "stas_number_tracked", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rtk_sensors_data_t, stas_number_tracked) }, \
         { "stas_number_usedinsolution", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_rtk_sensors_data_t, stas_number_usedinsolution) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_rtk_sensors_data_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 5, offsetof(ttalink_rtk_sensors_data_t, gps_time_ms) }, \
         { "sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_rtk_sensors_data_t, sol_status) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_rtk_sensors_data_t, pos_type) }, \
         { "rtk_lat", NULL, TTALINK_TYPE_DOUBLE, 0, 11, offsetof(ttalink_rtk_sensors_data_t, rtk_lat) }, \
         { "rtk_lon", NULL, TTALINK_TYPE_DOUBLE, 0, 19, offsetof(ttalink_rtk_sensors_data_t, rtk_lon) }, \
         { "rtk_alt", NULL, TTALINK_TYPE_FLOAT, 0, 27, offsetof(ttalink_rtk_sensors_data_t, rtk_alt) }, \
         { "undulation", NULL, TTALINK_TYPE_FLOAT, 0, 31, offsetof(ttalink_rtk_sensors_data_t, undulation) }, \
         { "lat_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_rtk_sensors_data_t, lat_std_deviation) }, \
         { "lon_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 39, offsetof(ttalink_rtk_sensors_data_t, lon_std_deviation) }, \
         { "alt_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_rtk_sensors_data_t, alt_std_deviation) }, \
         { "hor_speed", NULL, TTALINK_TYPE_FLOAT, 0, 47, offsetof(ttalink_rtk_sensors_data_t, hor_speed) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 51, offsetof(ttalink_rtk_sensors_data_t, trk_gnd) }, \
         { "vert_speed", NULL, TTALINK_TYPE_FLOAT, 0, 55, offsetof(ttalink_rtk_sensors_data_t, vert_speed) }, \
         { "length", NULL, TTALINK_TYPE_FLOAT, 0, 59, offsetof(ttalink_rtk_sensors_data_t, length) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 63, offsetof(ttalink_rtk_sensors_data_t, heading) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 67, offsetof(ttalink_rtk_sensors_data_t, pitch) }, \
         { "hdg_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 71, offsetof(ttalink_rtk_sensors_data_t, hdg_std_deviation) }, \
         { "ptch_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 75, offsetof(ttalink_rtk_sensors_data_t, ptch_std_deviation) }, \
         { "GDOP", NULL, TTALINK_TYPE_FLOAT, 0, 79, offsetof(ttalink_rtk_sensors_data_t, GDOP) }, \
         { "PDOP", NULL, TTALINK_TYPE_FLOAT, 0, 83, offsetof(ttalink_rtk_sensors_data_t, PDOP) }, \
         { "HDOP", NULL, TTALINK_TYPE_FLOAT, 0, 87, offsetof(ttalink_rtk_sensors_data_t, HDOP) }, \
         { "updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 91, offsetof(ttalink_rtk_sensors_data_t, updateTime) }, \
         { "vel_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 95, offsetof(ttalink_rtk_sensors_data_t, vel_updateTime) }, \
         { "pos_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 99, offsetof(ttalink_rtk_sensors_data_t, pos_updateTime) }, \
         { "dop_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 103, offsetof(ttalink_rtk_sensors_data_t, dop_updateTime) }, \
         { "heading_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 107, offsetof(ttalink_rtk_sensors_data_t, heading_updateTime) }, \
         { "heading_sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 111, offsetof(ttalink_rtk_sensors_data_t, heading_sol_status) }, \
         { "heading_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 112, offsetof(ttalink_rtk_sensors_data_t, heading_pos_type) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 113, offsetof(ttalink_rtk_sensors_data_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 117, offsetof(ttalink_rtk_sensors_data_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 118, offsetof(ttalink_rtk_sensors_data_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 119, offsetof(ttalink_rtk_sensors_data_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 120, offsetof(ttalink_rtk_sensors_data_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 121, offsetof(ttalink_rtk_sensors_data_t, utc_ms) }, \
         { "utc_status", NULL, TTALINK_TYPE_UINT32_T, 0, 125, offsetof(ttalink_rtk_sensors_data_t, utc_status) }, \
         { "ant_status", NULL, TTALINK_TYPE_UINT8_T, 0, 129, offsetof(ttalink_rtk_sensors_data_t, ant_status) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_RTK_SENSORS_DATA { \
    "RTK_SENSORS_DATA", \
    40, \
    {  { "stas_number", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rtk_sensors_data_t, stas_number) }, \
         { "stas_number_tracked", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rtk_sensors_data_t, stas_number_tracked) }, \
         { "stas_number_usedinsolution", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_rtk_sensors_data_t, stas_number_usedinsolution) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_rtk_sensors_data_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 5, offsetof(ttalink_rtk_sensors_data_t, gps_time_ms) }, \
         { "sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_rtk_sensors_data_t, sol_status) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_rtk_sensors_data_t, pos_type) }, \
         { "rtk_lat", NULL, TTALINK_TYPE_DOUBLE, 0, 11, offsetof(ttalink_rtk_sensors_data_t, rtk_lat) }, \
         { "rtk_lon", NULL, TTALINK_TYPE_DOUBLE, 0, 19, offsetof(ttalink_rtk_sensors_data_t, rtk_lon) }, \
         { "rtk_alt", NULL, TTALINK_TYPE_FLOAT, 0, 27, offsetof(ttalink_rtk_sensors_data_t, rtk_alt) }, \
         { "undulation", NULL, TTALINK_TYPE_FLOAT, 0, 31, offsetof(ttalink_rtk_sensors_data_t, undulation) }, \
         { "lat_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_rtk_sensors_data_t, lat_std_deviation) }, \
         { "lon_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 39, offsetof(ttalink_rtk_sensors_data_t, lon_std_deviation) }, \
         { "alt_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_rtk_sensors_data_t, alt_std_deviation) }, \
         { "hor_speed", NULL, TTALINK_TYPE_FLOAT, 0, 47, offsetof(ttalink_rtk_sensors_data_t, hor_speed) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 51, offsetof(ttalink_rtk_sensors_data_t, trk_gnd) }, \
         { "vert_speed", NULL, TTALINK_TYPE_FLOAT, 0, 55, offsetof(ttalink_rtk_sensors_data_t, vert_speed) }, \
         { "length", NULL, TTALINK_TYPE_FLOAT, 0, 59, offsetof(ttalink_rtk_sensors_data_t, length) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 63, offsetof(ttalink_rtk_sensors_data_t, heading) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 67, offsetof(ttalink_rtk_sensors_data_t, pitch) }, \
         { "hdg_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 71, offsetof(ttalink_rtk_sensors_data_t, hdg_std_deviation) }, \
         { "ptch_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 75, offsetof(ttalink_rtk_sensors_data_t, ptch_std_deviation) }, \
         { "GDOP", NULL, TTALINK_TYPE_FLOAT, 0, 79, offsetof(ttalink_rtk_sensors_data_t, GDOP) }, \
         { "PDOP", NULL, TTALINK_TYPE_FLOAT, 0, 83, offsetof(ttalink_rtk_sensors_data_t, PDOP) }, \
         { "HDOP", NULL, TTALINK_TYPE_FLOAT, 0, 87, offsetof(ttalink_rtk_sensors_data_t, HDOP) }, \
         { "updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 91, offsetof(ttalink_rtk_sensors_data_t, updateTime) }, \
         { "vel_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 95, offsetof(ttalink_rtk_sensors_data_t, vel_updateTime) }, \
         { "pos_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 99, offsetof(ttalink_rtk_sensors_data_t, pos_updateTime) }, \
         { "dop_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 103, offsetof(ttalink_rtk_sensors_data_t, dop_updateTime) }, \
         { "heading_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 107, offsetof(ttalink_rtk_sensors_data_t, heading_updateTime) }, \
         { "heading_sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 111, offsetof(ttalink_rtk_sensors_data_t, heading_sol_status) }, \
         { "heading_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 112, offsetof(ttalink_rtk_sensors_data_t, heading_pos_type) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 113, offsetof(ttalink_rtk_sensors_data_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 117, offsetof(ttalink_rtk_sensors_data_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 118, offsetof(ttalink_rtk_sensors_data_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 119, offsetof(ttalink_rtk_sensors_data_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 120, offsetof(ttalink_rtk_sensors_data_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 121, offsetof(ttalink_rtk_sensors_data_t, utc_ms) }, \
         { "utc_status", NULL, TTALINK_TYPE_UINT32_T, 0, 125, offsetof(ttalink_rtk_sensors_data_t, utc_status) }, \
         { "ant_status", NULL, TTALINK_TYPE_UINT8_T, 0, 129, offsetof(ttalink_rtk_sensors_data_t, ant_status) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rtk_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, stas_number);
    _tta_put_uint8_t(buf, 1, stas_number_tracked);
    _tta_put_uint8_t(buf, 2, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 3, gps_time_week);
    _tta_put_uint32_t(buf, 5, gps_time_ms);
    _tta_put_uint8_t(buf, 9, sol_status);
    _tta_put_uint8_t(buf, 10, pos_type);
    _tta_put_double(buf, 11, rtk_lat);
    _tta_put_double(buf, 19, rtk_lon);
    _tta_put_float(buf, 27, rtk_alt);
    _tta_put_float(buf, 31, undulation);
    _tta_put_float(buf, 35, lat_std_deviation);
    _tta_put_float(buf, 39, lon_std_deviation);
    _tta_put_float(buf, 43, alt_std_deviation);
    _tta_put_float(buf, 47, hor_speed);
    _tta_put_float(buf, 51, trk_gnd);
    _tta_put_float(buf, 55, vert_speed);
    _tta_put_float(buf, 59, length);
    _tta_put_float(buf, 63, heading);
    _tta_put_float(buf, 67, pitch);
    _tta_put_float(buf, 71, hdg_std_deviation);
    _tta_put_float(buf, 75, ptch_std_deviation);
    _tta_put_float(buf, 79, GDOP);
    _tta_put_float(buf, 83, PDOP);
    _tta_put_float(buf, 87, HDOP);
    _tta_put_uint32_t(buf, 91, updateTime);
    _tta_put_uint32_t(buf, 95, vel_updateTime);
    _tta_put_uint32_t(buf, 99, pos_updateTime);
    _tta_put_uint32_t(buf, 103, dop_updateTime);
    _tta_put_uint32_t(buf, 107, heading_updateTime);
    _tta_put_uint8_t(buf, 111, heading_sol_status);
    _tta_put_uint8_t(buf, 112, heading_pos_type);
    _tta_put_uint32_t(buf, 113, utc_year);
    _tta_put_uint8_t(buf, 117, utc_month);
    _tta_put_uint8_t(buf, 118, utc_day);
    _tta_put_uint8_t(buf, 119, utc_hour);
    _tta_put_uint8_t(buf, 120, utc_min);
    _tta_put_uint32_t(buf, 121, utc_ms);
    _tta_put_uint32_t(buf, 125, utc_status);
    _tta_put_uint8_t(buf, 129, ant_status);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN);
#else
    ttalink_rtk_sensors_data_t packet;
    packet.stas_number = stas_number;
    packet.stas_number_tracked = stas_number_tracked;
    packet.stas_number_usedinsolution = stas_number_usedinsolution;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.sol_status = sol_status;
    packet.pos_type = pos_type;
    packet.rtk_lat = rtk_lat;
    packet.rtk_lon = rtk_lon;
    packet.rtk_alt = rtk_alt;
    packet.undulation = undulation;
    packet.lat_std_deviation = lat_std_deviation;
    packet.lon_std_deviation = lon_std_deviation;
    packet.alt_std_deviation = alt_std_deviation;
    packet.hor_speed = hor_speed;
    packet.trk_gnd = trk_gnd;
    packet.vert_speed = vert_speed;
    packet.length = length;
    packet.heading = heading;
    packet.pitch = pitch;
    packet.hdg_std_deviation = hdg_std_deviation;
    packet.ptch_std_deviation = ptch_std_deviation;
    packet.GDOP = GDOP;
    packet.PDOP = PDOP;
    packet.HDOP = HDOP;
    packet.updateTime = updateTime;
    packet.vel_updateTime = vel_updateTime;
    packet.pos_updateTime = pos_updateTime;
    packet.dop_updateTime = dop_updateTime;
    packet.heading_updateTime = heading_updateTime;
    packet.heading_sol_status = heading_sol_status;
    packet.heading_pos_type = heading_pos_type;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.utc_status = utc_status;
    packet.ant_status = ant_status;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RTK_SENSORS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rtk_sensors_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param stas_number  .
 * @param stas_number_tracked  .
 * @param stas_number_usedinsolution  .
 * @param gps_time_week  .
 * @param gps_time_ms  .
 * @param sol_status  .
 * @param pos_type  .
 * @param rtk_lat  .
 * @param rtk_lon  .
 * @param rtk_alt  .
 * @param undulation  .
 * @param lat_std_deviation  .
 * @param lon_std_deviation  .
 * @param alt_std_deviation  .
 * @param hor_speed  .
 * @param trk_gnd  .
 * @param vert_speed  .
 * @param length  .
 * @param heading  .
 * @param pitch  .
 * @param hdg_std_deviation  .
 * @param ptch_std_deviation  .
 * @param GDOP  .
 * @param PDOP  .
 * @param HDOP  .
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param utc_year  .
 * @param utc_month  .
 * @param utc_day  .
 * @param utc_hour  .
 * @param utc_min  .
 * @param utc_ms  .
 * @param utc_status  .
 * @param ant_status  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    return _ttalink_rtk_sensors_data_pack(dst_addr, src_addr, msg,  stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/**
 * @brief Pack a rtk_sensors_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param stas_number  .
 * @param stas_number_tracked  .
 * @param stas_number_usedinsolution  .
 * @param gps_time_week  .
 * @param gps_time_ms  .
 * @param sol_status  .
 * @param pos_type  .
 * @param rtk_lat  .
 * @param rtk_lon  .
 * @param rtk_alt  .
 * @param undulation  .
 * @param lat_std_deviation  .
 * @param lon_std_deviation  .
 * @param alt_std_deviation  .
 * @param hor_speed  .
 * @param trk_gnd  .
 * @param vert_speed  .
 * @param length  .
 * @param heading  .
 * @param pitch  .
 * @param hdg_std_deviation  .
 * @param ptch_std_deviation  .
 * @param GDOP  .
 * @param PDOP  .
 * @param HDOP  .
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param utc_year  .
 * @param utc_month  .
 * @param utc_day  .
 * @param utc_hour  .
 * @param utc_min  .
 * @param utc_ms  .
 * @param utc_status  .
 * @param ant_status  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    return _ttalink_rtk_sensors_data_pack(dst_addr, src_addr, msg,  stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}


static inline uint16_t _ttalink_rtk_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stas_number,uint8_t stas_number_tracked,uint8_t stas_number_usedinsolution,uint16_t gps_time_week,uint32_t gps_time_ms,uint8_t sol_status,uint8_t pos_type,double rtk_lat,double rtk_lon,float rtk_alt,float undulation,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float hor_speed,float trk_gnd,float vert_speed,float length,float heading,float pitch,float hdg_std_deviation,float ptch_std_deviation,float GDOP,float PDOP,float HDOP,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t utc_status,uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, stas_number);
    _tta_put_uint8_t(buf, 1, stas_number_tracked);
    _tta_put_uint8_t(buf, 2, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 3, gps_time_week);
    _tta_put_uint32_t(buf, 5, gps_time_ms);
    _tta_put_uint8_t(buf, 9, sol_status);
    _tta_put_uint8_t(buf, 10, pos_type);
    _tta_put_double(buf, 11, rtk_lat);
    _tta_put_double(buf, 19, rtk_lon);
    _tta_put_float(buf, 27, rtk_alt);
    _tta_put_float(buf, 31, undulation);
    _tta_put_float(buf, 35, lat_std_deviation);
    _tta_put_float(buf, 39, lon_std_deviation);
    _tta_put_float(buf, 43, alt_std_deviation);
    _tta_put_float(buf, 47, hor_speed);
    _tta_put_float(buf, 51, trk_gnd);
    _tta_put_float(buf, 55, vert_speed);
    _tta_put_float(buf, 59, length);
    _tta_put_float(buf, 63, heading);
    _tta_put_float(buf, 67, pitch);
    _tta_put_float(buf, 71, hdg_std_deviation);
    _tta_put_float(buf, 75, ptch_std_deviation);
    _tta_put_float(buf, 79, GDOP);
    _tta_put_float(buf, 83, PDOP);
    _tta_put_float(buf, 87, HDOP);
    _tta_put_uint32_t(buf, 91, updateTime);
    _tta_put_uint32_t(buf, 95, vel_updateTime);
    _tta_put_uint32_t(buf, 99, pos_updateTime);
    _tta_put_uint32_t(buf, 103, dop_updateTime);
    _tta_put_uint32_t(buf, 107, heading_updateTime);
    _tta_put_uint8_t(buf, 111, heading_sol_status);
    _tta_put_uint8_t(buf, 112, heading_pos_type);
    _tta_put_uint32_t(buf, 113, utc_year);
    _tta_put_uint8_t(buf, 117, utc_month);
    _tta_put_uint8_t(buf, 118, utc_day);
    _tta_put_uint8_t(buf, 119, utc_hour);
    _tta_put_uint8_t(buf, 120, utc_min);
    _tta_put_uint32_t(buf, 121, utc_ms);
    _tta_put_uint32_t(buf, 125, utc_status);
    _tta_put_uint8_t(buf, 129, ant_status);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN);
#else
    ttalink_rtk_sensors_data_t packet;
    packet.stas_number = stas_number;
    packet.stas_number_tracked = stas_number_tracked;
    packet.stas_number_usedinsolution = stas_number_usedinsolution;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.sol_status = sol_status;
    packet.pos_type = pos_type;
    packet.rtk_lat = rtk_lat;
    packet.rtk_lon = rtk_lon;
    packet.rtk_alt = rtk_alt;
    packet.undulation = undulation;
    packet.lat_std_deviation = lat_std_deviation;
    packet.lon_std_deviation = lon_std_deviation;
    packet.alt_std_deviation = alt_std_deviation;
    packet.hor_speed = hor_speed;
    packet.trk_gnd = trk_gnd;
    packet.vert_speed = vert_speed;
    packet.length = length;
    packet.heading = heading;
    packet.pitch = pitch;
    packet.hdg_std_deviation = hdg_std_deviation;
    packet.ptch_std_deviation = ptch_std_deviation;
    packet.GDOP = GDOP;
    packet.PDOP = PDOP;
    packet.HDOP = HDOP;
    packet.updateTime = updateTime;
    packet.vel_updateTime = vel_updateTime;
    packet.pos_updateTime = pos_updateTime;
    packet.dop_updateTime = dop_updateTime;
    packet.heading_updateTime = heading_updateTime;
    packet.heading_sol_status = heading_sol_status;
    packet.heading_pos_type = heading_pos_type;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.utc_status = utc_status;
    packet.ant_status = ant_status;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RTK_SENSORS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rtk_sensors_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stas_number  .
 * @param stas_number_tracked  .
 * @param stas_number_usedinsolution  .
 * @param gps_time_week  .
 * @param gps_time_ms  .
 * @param sol_status  .
 * @param pos_type  .
 * @param rtk_lat  .
 * @param rtk_lon  .
 * @param rtk_alt  .
 * @param undulation  .
 * @param lat_std_deviation  .
 * @param lon_std_deviation  .
 * @param alt_std_deviation  .
 * @param hor_speed  .
 * @param trk_gnd  .
 * @param vert_speed  .
 * @param length  .
 * @param heading  .
 * @param pitch  .
 * @param hdg_std_deviation  .
 * @param ptch_std_deviation  .
 * @param GDOP  .
 * @param PDOP  .
 * @param HDOP  .
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param utc_year  .
 * @param utc_month  .
 * @param utc_day  .
 * @param utc_hour  .
 * @param utc_min  .
 * @param utc_ms  .
 * @param utc_status  .
 * @param ant_status  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stas_number,uint8_t stas_number_tracked,uint8_t stas_number_usedinsolution,uint16_t gps_time_week,uint32_t gps_time_ms,uint8_t sol_status,uint8_t pos_type,double rtk_lat,double rtk_lon,float rtk_alt,float undulation,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float hor_speed,float trk_gnd,float vert_speed,float length,float heading,float pitch,float hdg_std_deviation,float ptch_std_deviation,float GDOP,float PDOP,float HDOP,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t utc_status,uint8_t ant_status)
{
    return _ttalink_rtk_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/**
 * @brief Pack a rtk_sensors_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stas_number  .
 * @param stas_number_tracked  .
 * @param stas_number_usedinsolution  .
 * @param gps_time_week  .
 * @param gps_time_ms  .
 * @param sol_status  .
 * @param pos_type  .
 * @param rtk_lat  .
 * @param rtk_lon  .
 * @param rtk_alt  .
 * @param undulation  .
 * @param lat_std_deviation  .
 * @param lon_std_deviation  .
 * @param alt_std_deviation  .
 * @param hor_speed  .
 * @param trk_gnd  .
 * @param vert_speed  .
 * @param length  .
 * @param heading  .
 * @param pitch  .
 * @param hdg_std_deviation  .
 * @param ptch_std_deviation  .
 * @param GDOP  .
 * @param PDOP  .
 * @param HDOP  .
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param utc_year  .
 * @param utc_month  .
 * @param utc_day  .
 * @param utc_hour  .
 * @param utc_min  .
 * @param utc_ms  .
 * @param utc_status  .
 * @param ant_status  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stas_number,uint8_t stas_number_tracked,uint8_t stas_number_usedinsolution,uint16_t gps_time_week,uint32_t gps_time_ms,uint8_t sol_status,uint8_t pos_type,double rtk_lat,double rtk_lon,float rtk_alt,float undulation,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float hor_speed,float trk_gnd,float vert_speed,float length,float heading,float pitch,float hdg_std_deviation,float ptch_std_deviation,float GDOP,float PDOP,float HDOP,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t utc_status,uint8_t ant_status)
{
    return _ttalink_rtk_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}


static inline uint16_t _ttalink_rtk_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_data_t* rtk_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rtk_sensors_data_pack_nocrc(dst_addr, src_addr, msg, rtk_sensors_data->stas_number, rtk_sensors_data->stas_number_tracked, rtk_sensors_data->stas_number_usedinsolution, rtk_sensors_data->gps_time_week, rtk_sensors_data->gps_time_ms, rtk_sensors_data->sol_status, rtk_sensors_data->pos_type, rtk_sensors_data->rtk_lat, rtk_sensors_data->rtk_lon, rtk_sensors_data->rtk_alt, rtk_sensors_data->undulation, rtk_sensors_data->lat_std_deviation, rtk_sensors_data->lon_std_deviation, rtk_sensors_data->alt_std_deviation, rtk_sensors_data->hor_speed, rtk_sensors_data->trk_gnd, rtk_sensors_data->vert_speed, rtk_sensors_data->length, rtk_sensors_data->heading, rtk_sensors_data->pitch, rtk_sensors_data->hdg_std_deviation, rtk_sensors_data->ptch_std_deviation, rtk_sensors_data->GDOP, rtk_sensors_data->PDOP, rtk_sensors_data->HDOP, rtk_sensors_data->updateTime, rtk_sensors_data->vel_updateTime, rtk_sensors_data->pos_updateTime, rtk_sensors_data->dop_updateTime, rtk_sensors_data->heading_updateTime, rtk_sensors_data->heading_sol_status, rtk_sensors_data->heading_pos_type, rtk_sensors_data->utc_year, rtk_sensors_data->utc_month, rtk_sensors_data->utc_day, rtk_sensors_data->utc_hour, rtk_sensors_data->utc_min, rtk_sensors_data->utc_ms, rtk_sensors_data->utc_status, rtk_sensors_data->ant_status);
    }else{
        return ttalink_rtk_sensors_data_pack(dst_addr, src_addr, msg, rtk_sensors_data->stas_number, rtk_sensors_data->stas_number_tracked, rtk_sensors_data->stas_number_usedinsolution, rtk_sensors_data->gps_time_week, rtk_sensors_data->gps_time_ms, rtk_sensors_data->sol_status, rtk_sensors_data->pos_type, rtk_sensors_data->rtk_lat, rtk_sensors_data->rtk_lon, rtk_sensors_data->rtk_alt, rtk_sensors_data->undulation, rtk_sensors_data->lat_std_deviation, rtk_sensors_data->lon_std_deviation, rtk_sensors_data->alt_std_deviation, rtk_sensors_data->hor_speed, rtk_sensors_data->trk_gnd, rtk_sensors_data->vert_speed, rtk_sensors_data->length, rtk_sensors_data->heading, rtk_sensors_data->pitch, rtk_sensors_data->hdg_std_deviation, rtk_sensors_data->ptch_std_deviation, rtk_sensors_data->GDOP, rtk_sensors_data->PDOP, rtk_sensors_data->HDOP, rtk_sensors_data->updateTime, rtk_sensors_data->vel_updateTime, rtk_sensors_data->pos_updateTime, rtk_sensors_data->dop_updateTime, rtk_sensors_data->heading_updateTime, rtk_sensors_data->heading_sol_status, rtk_sensors_data->heading_pos_type, rtk_sensors_data->utc_year, rtk_sensors_data->utc_month, rtk_sensors_data->utc_day, rtk_sensors_data->utc_hour, rtk_sensors_data->utc_min, rtk_sensors_data->utc_ms, rtk_sensors_data->utc_status, rtk_sensors_data->ant_status);
    }
    
}

/**
 * @brief Encode a rtk_sensors_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_data_t* rtk_sensors_data)
{
    return _ttalink_rtk_sensors_data_encode(dst_addr, src_addr, msg, rtk_sensors_data, false);
}

/**
 * @brief Encode a rtk_sensors_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_data_t* rtk_sensors_data)
{
    return _ttalink_rtk_sensors_data_encode(dst_addr, src_addr, msg, rtk_sensors_data, true);
}


static inline uint16_t _ttalink_rtk_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_data_t* rtk_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rtk_sensors_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rtk_sensors_data->stas_number, rtk_sensors_data->stas_number_tracked, rtk_sensors_data->stas_number_usedinsolution, rtk_sensors_data->gps_time_week, rtk_sensors_data->gps_time_ms, rtk_sensors_data->sol_status, rtk_sensors_data->pos_type, rtk_sensors_data->rtk_lat, rtk_sensors_data->rtk_lon, rtk_sensors_data->rtk_alt, rtk_sensors_data->undulation, rtk_sensors_data->lat_std_deviation, rtk_sensors_data->lon_std_deviation, rtk_sensors_data->alt_std_deviation, rtk_sensors_data->hor_speed, rtk_sensors_data->trk_gnd, rtk_sensors_data->vert_speed, rtk_sensors_data->length, rtk_sensors_data->heading, rtk_sensors_data->pitch, rtk_sensors_data->hdg_std_deviation, rtk_sensors_data->ptch_std_deviation, rtk_sensors_data->GDOP, rtk_sensors_data->PDOP, rtk_sensors_data->HDOP, rtk_sensors_data->updateTime, rtk_sensors_data->vel_updateTime, rtk_sensors_data->pos_updateTime, rtk_sensors_data->dop_updateTime, rtk_sensors_data->heading_updateTime, rtk_sensors_data->heading_sol_status, rtk_sensors_data->heading_pos_type, rtk_sensors_data->utc_year, rtk_sensors_data->utc_month, rtk_sensors_data->utc_day, rtk_sensors_data->utc_hour, rtk_sensors_data->utc_min, rtk_sensors_data->utc_ms, rtk_sensors_data->utc_status, rtk_sensors_data->ant_status);
    }else{
        return ttalink_rtk_sensors_data_pack_chan(dst_addr, src_addr, chan, msg, rtk_sensors_data->stas_number, rtk_sensors_data->stas_number_tracked, rtk_sensors_data->stas_number_usedinsolution, rtk_sensors_data->gps_time_week, rtk_sensors_data->gps_time_ms, rtk_sensors_data->sol_status, rtk_sensors_data->pos_type, rtk_sensors_data->rtk_lat, rtk_sensors_data->rtk_lon, rtk_sensors_data->rtk_alt, rtk_sensors_data->undulation, rtk_sensors_data->lat_std_deviation, rtk_sensors_data->lon_std_deviation, rtk_sensors_data->alt_std_deviation, rtk_sensors_data->hor_speed, rtk_sensors_data->trk_gnd, rtk_sensors_data->vert_speed, rtk_sensors_data->length, rtk_sensors_data->heading, rtk_sensors_data->pitch, rtk_sensors_data->hdg_std_deviation, rtk_sensors_data->ptch_std_deviation, rtk_sensors_data->GDOP, rtk_sensors_data->PDOP, rtk_sensors_data->HDOP, rtk_sensors_data->updateTime, rtk_sensors_data->vel_updateTime, rtk_sensors_data->pos_updateTime, rtk_sensors_data->dop_updateTime, rtk_sensors_data->heading_updateTime, rtk_sensors_data->heading_sol_status, rtk_sensors_data->heading_pos_type, rtk_sensors_data->utc_year, rtk_sensors_data->utc_month, rtk_sensors_data->utc_day, rtk_sensors_data->utc_hour, rtk_sensors_data->utc_min, rtk_sensors_data->utc_ms, rtk_sensors_data->utc_status, rtk_sensors_data->ant_status);
    }
}

/**
 * @brief Encode a rtk_sensors_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_data_t* rtk_sensors_data)
{
    return _ttalink_rtk_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, rtk_sensors_data, false);
}

/**
 * @brief Encode a rtk_sensors_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_data_t* rtk_sensors_data)
{
    return _ttalink_rtk_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, rtk_sensors_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rtk_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, stas_number);
    _tta_put_uint8_t(buf, 1, stas_number_tracked);
    _tta_put_uint8_t(buf, 2, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 3, gps_time_week);
    _tta_put_uint32_t(buf, 5, gps_time_ms);
    _tta_put_uint8_t(buf, 9, sol_status);
    _tta_put_uint8_t(buf, 10, pos_type);
    _tta_put_double(buf, 11, rtk_lat);
    _tta_put_double(buf, 19, rtk_lon);
    _tta_put_float(buf, 27, rtk_alt);
    _tta_put_float(buf, 31, undulation);
    _tta_put_float(buf, 35, lat_std_deviation);
    _tta_put_float(buf, 39, lon_std_deviation);
    _tta_put_float(buf, 43, alt_std_deviation);
    _tta_put_float(buf, 47, hor_speed);
    _tta_put_float(buf, 51, trk_gnd);
    _tta_put_float(buf, 55, vert_speed);
    _tta_put_float(buf, 59, length);
    _tta_put_float(buf, 63, heading);
    _tta_put_float(buf, 67, pitch);
    _tta_put_float(buf, 71, hdg_std_deviation);
    _tta_put_float(buf, 75, ptch_std_deviation);
    _tta_put_float(buf, 79, GDOP);
    _tta_put_float(buf, 83, PDOP);
    _tta_put_float(buf, 87, HDOP);
    _tta_put_uint32_t(buf, 91, updateTime);
    _tta_put_uint32_t(buf, 95, vel_updateTime);
    _tta_put_uint32_t(buf, 99, pos_updateTime);
    _tta_put_uint32_t(buf, 103, dop_updateTime);
    _tta_put_uint32_t(buf, 107, heading_updateTime);
    _tta_put_uint8_t(buf, 111, heading_sol_status);
    _tta_put_uint8_t(buf, 112, heading_pos_type);
    _tta_put_uint32_t(buf, 113, utc_year);
    _tta_put_uint8_t(buf, 117, utc_month);
    _tta_put_uint8_t(buf, 118, utc_day);
    _tta_put_uint8_t(buf, 119, utc_hour);
    _tta_put_uint8_t(buf, 120, utc_min);
    _tta_put_uint32_t(buf, 121, utc_ms);
    _tta_put_uint32_t(buf, 125, utc_status);
    _tta_put_uint8_t(buf, 129, ant_status);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_DATA, buf, TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_rtk_sensors_data_t packet;
    packet.stas_number = stas_number;
    packet.stas_number_tracked = stas_number_tracked;
    packet.stas_number_usedinsolution = stas_number_usedinsolution;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.sol_status = sol_status;
    packet.pos_type = pos_type;
    packet.rtk_lat = rtk_lat;
    packet.rtk_lon = rtk_lon;
    packet.rtk_alt = rtk_alt;
    packet.undulation = undulation;
    packet.lat_std_deviation = lat_std_deviation;
    packet.lon_std_deviation = lon_std_deviation;
    packet.alt_std_deviation = alt_std_deviation;
    packet.hor_speed = hor_speed;
    packet.trk_gnd = trk_gnd;
    packet.vert_speed = vert_speed;
    packet.length = length;
    packet.heading = heading;
    packet.pitch = pitch;
    packet.hdg_std_deviation = hdg_std_deviation;
    packet.ptch_std_deviation = ptch_std_deviation;
    packet.GDOP = GDOP;
    packet.PDOP = PDOP;
    packet.HDOP = HDOP;
    packet.updateTime = updateTime;
    packet.vel_updateTime = vel_updateTime;
    packet.pos_updateTime = pos_updateTime;
    packet.dop_updateTime = dop_updateTime;
    packet.heading_updateTime = heading_updateTime;
    packet.heading_sol_status = heading_sol_status;
    packet.heading_pos_type = heading_pos_type;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.utc_status = utc_status;
    packet.ant_status = ant_status;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_DATA, (const char *)&packet, TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rtk_sensors_data message
 * @param chan TTAlink channel to send the message
 *
 * @param stas_number  .
 * @param stas_number_tracked  .
 * @param stas_number_usedinsolution  .
 * @param gps_time_week  .
 * @param gps_time_ms  .
 * @param sol_status  .
 * @param pos_type  .
 * @param rtk_lat  .
 * @param rtk_lon  .
 * @param rtk_alt  .
 * @param undulation  .
 * @param lat_std_deviation  .
 * @param lon_std_deviation  .
 * @param alt_std_deviation  .
 * @param hor_speed  .
 * @param trk_gnd  .
 * @param vert_speed  .
 * @param length  .
 * @param heading  .
 * @param pitch  .
 * @param hdg_std_deviation  .
 * @param ptch_std_deviation  .
 * @param GDOP  .
 * @param PDOP  .
 * @param HDOP  .
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param utc_year  .
 * @param utc_month  .
 * @param utc_day  .
 * @param utc_hour  .
 * @param utc_min  .
 * @param utc_ms  .
 * @param utc_status  .
 * @param ant_status  .
 */
static inline void ttalink_rtk_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_rtk_sensors_data_send(dst_addr, src_addr, chan, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/**
 * @brief Send a rtk_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param stas_number  .
 * @param stas_number_tracked  .
 * @param stas_number_usedinsolution  .
 * @param gps_time_week  .
 * @param gps_time_ms  .
 * @param sol_status  .
 * @param pos_type  .
 * @param rtk_lat  .
 * @param rtk_lon  .
 * @param rtk_alt  .
 * @param undulation  .
 * @param lat_std_deviation  .
 * @param lon_std_deviation  .
 * @param alt_std_deviation  .
 * @param hor_speed  .
 * @param trk_gnd  .
 * @param vert_speed  .
 * @param length  .
 * @param heading  .
 * @param pitch  .
 * @param hdg_std_deviation  .
 * @param ptch_std_deviation  .
 * @param GDOP  .
 * @param PDOP  .
 * @param HDOP  .
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param utc_year  .
 * @param utc_month  .
 * @param utc_day  .
 * @param utc_hour  .
 * @param utc_min  .
 * @param utc_ms  .
 * @param utc_status  .
 * @param ant_status  .
 */
static inline void ttalink_rtk_sensors_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_rtk_sensors_data_send(dst_addr, src_addr, chan, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}


static inline void _ttalink_rtk_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_data_t* rtk_sensors_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rtk_sensors_data_send_nocrc(dst_addr, src_addr, chan, rtk_sensors_data->stas_number, rtk_sensors_data->stas_number_tracked, rtk_sensors_data->stas_number_usedinsolution, rtk_sensors_data->gps_time_week, rtk_sensors_data->gps_time_ms, rtk_sensors_data->sol_status, rtk_sensors_data->pos_type, rtk_sensors_data->rtk_lat, rtk_sensors_data->rtk_lon, rtk_sensors_data->rtk_alt, rtk_sensors_data->undulation, rtk_sensors_data->lat_std_deviation, rtk_sensors_data->lon_std_deviation, rtk_sensors_data->alt_std_deviation, rtk_sensors_data->hor_speed, rtk_sensors_data->trk_gnd, rtk_sensors_data->vert_speed, rtk_sensors_data->length, rtk_sensors_data->heading, rtk_sensors_data->pitch, rtk_sensors_data->hdg_std_deviation, rtk_sensors_data->ptch_std_deviation, rtk_sensors_data->GDOP, rtk_sensors_data->PDOP, rtk_sensors_data->HDOP, rtk_sensors_data->updateTime, rtk_sensors_data->vel_updateTime, rtk_sensors_data->pos_updateTime, rtk_sensors_data->dop_updateTime, rtk_sensors_data->heading_updateTime, rtk_sensors_data->heading_sol_status, rtk_sensors_data->heading_pos_type, rtk_sensors_data->utc_year, rtk_sensors_data->utc_month, rtk_sensors_data->utc_day, rtk_sensors_data->utc_hour, rtk_sensors_data->utc_min, rtk_sensors_data->utc_ms, rtk_sensors_data->utc_status, rtk_sensors_data->ant_status);
    }else{
        ttalink_rtk_sensors_data_send(dst_addr, src_addr, chan, rtk_sensors_data->stas_number, rtk_sensors_data->stas_number_tracked, rtk_sensors_data->stas_number_usedinsolution, rtk_sensors_data->gps_time_week, rtk_sensors_data->gps_time_ms, rtk_sensors_data->sol_status, rtk_sensors_data->pos_type, rtk_sensors_data->rtk_lat, rtk_sensors_data->rtk_lon, rtk_sensors_data->rtk_alt, rtk_sensors_data->undulation, rtk_sensors_data->lat_std_deviation, rtk_sensors_data->lon_std_deviation, rtk_sensors_data->alt_std_deviation, rtk_sensors_data->hor_speed, rtk_sensors_data->trk_gnd, rtk_sensors_data->vert_speed, rtk_sensors_data->length, rtk_sensors_data->heading, rtk_sensors_data->pitch, rtk_sensors_data->hdg_std_deviation, rtk_sensors_data->ptch_std_deviation, rtk_sensors_data->GDOP, rtk_sensors_data->PDOP, rtk_sensors_data->HDOP, rtk_sensors_data->updateTime, rtk_sensors_data->vel_updateTime, rtk_sensors_data->pos_updateTime, rtk_sensors_data->dop_updateTime, rtk_sensors_data->heading_updateTime, rtk_sensors_data->heading_sol_status, rtk_sensors_data->heading_pos_type, rtk_sensors_data->utc_year, rtk_sensors_data->utc_month, rtk_sensors_data->utc_day, rtk_sensors_data->utc_hour, rtk_sensors_data->utc_min, rtk_sensors_data->utc_ms, rtk_sensors_data->utc_status, rtk_sensors_data->ant_status);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_DATA, (const char *)rtk_sensors_data, TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rtk_sensors_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rtk_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_data_t* rtk_sensors_data)
{
    _ttalink_rtk_sensors_data_send_struct(dst_addr, src_addr, chan, rtk_sensors_data, false);
}

/**
 * @brief Send a rtk_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rtk_sensors_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_data_t* rtk_sensors_data)
{
    _ttalink_rtk_sensors_data_send_struct(dst_addr, src_addr, chan, rtk_sensors_data, true);
}

#if TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rtk_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, stas_number);
    _tta_put_uint8_t(buf, 1, stas_number_tracked);
    _tta_put_uint8_t(buf, 2, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 3, gps_time_week);
    _tta_put_uint32_t(buf, 5, gps_time_ms);
    _tta_put_uint8_t(buf, 9, sol_status);
    _tta_put_uint8_t(buf, 10, pos_type);
    _tta_put_double(buf, 11, rtk_lat);
    _tta_put_double(buf, 19, rtk_lon);
    _tta_put_float(buf, 27, rtk_alt);
    _tta_put_float(buf, 31, undulation);
    _tta_put_float(buf, 35, lat_std_deviation);
    _tta_put_float(buf, 39, lon_std_deviation);
    _tta_put_float(buf, 43, alt_std_deviation);
    _tta_put_float(buf, 47, hor_speed);
    _tta_put_float(buf, 51, trk_gnd);
    _tta_put_float(buf, 55, vert_speed);
    _tta_put_float(buf, 59, length);
    _tta_put_float(buf, 63, heading);
    _tta_put_float(buf, 67, pitch);
    _tta_put_float(buf, 71, hdg_std_deviation);
    _tta_put_float(buf, 75, ptch_std_deviation);
    _tta_put_float(buf, 79, GDOP);
    _tta_put_float(buf, 83, PDOP);
    _tta_put_float(buf, 87, HDOP);
    _tta_put_uint32_t(buf, 91, updateTime);
    _tta_put_uint32_t(buf, 95, vel_updateTime);
    _tta_put_uint32_t(buf, 99, pos_updateTime);
    _tta_put_uint32_t(buf, 103, dop_updateTime);
    _tta_put_uint32_t(buf, 107, heading_updateTime);
    _tta_put_uint8_t(buf, 111, heading_sol_status);
    _tta_put_uint8_t(buf, 112, heading_pos_type);
    _tta_put_uint32_t(buf, 113, utc_year);
    _tta_put_uint8_t(buf, 117, utc_month);
    _tta_put_uint8_t(buf, 118, utc_day);
    _tta_put_uint8_t(buf, 119, utc_hour);
    _tta_put_uint8_t(buf, 120, utc_min);
    _tta_put_uint32_t(buf, 121, utc_ms);
    _tta_put_uint32_t(buf, 125, utc_status);
    _tta_put_uint8_t(buf, 129, ant_status);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_DATA, buf, TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_rtk_sensors_data_t *packet = (ttalink_rtk_sensors_data_t *)msgbuf;
    packet->stas_number = stas_number;
    packet->stas_number_tracked = stas_number_tracked;
    packet->stas_number_usedinsolution = stas_number_usedinsolution;
    packet->gps_time_week = gps_time_week;
    packet->gps_time_ms = gps_time_ms;
    packet->sol_status = sol_status;
    packet->pos_type = pos_type;
    packet->rtk_lat = rtk_lat;
    packet->rtk_lon = rtk_lon;
    packet->rtk_alt = rtk_alt;
    packet->undulation = undulation;
    packet->lat_std_deviation = lat_std_deviation;
    packet->lon_std_deviation = lon_std_deviation;
    packet->alt_std_deviation = alt_std_deviation;
    packet->hor_speed = hor_speed;
    packet->trk_gnd = trk_gnd;
    packet->vert_speed = vert_speed;
    packet->length = length;
    packet->heading = heading;
    packet->pitch = pitch;
    packet->hdg_std_deviation = hdg_std_deviation;
    packet->ptch_std_deviation = ptch_std_deviation;
    packet->GDOP = GDOP;
    packet->PDOP = PDOP;
    packet->HDOP = HDOP;
    packet->updateTime = updateTime;
    packet->vel_updateTime = vel_updateTime;
    packet->pos_updateTime = pos_updateTime;
    packet->dop_updateTime = dop_updateTime;
    packet->heading_updateTime = heading_updateTime;
    packet->heading_sol_status = heading_sol_status;
    packet->heading_pos_type = heading_pos_type;
    packet->utc_year = utc_year;
    packet->utc_month = utc_month;
    packet->utc_day = utc_day;
    packet->utc_hour = utc_hour;
    packet->utc_min = utc_min;
    packet->utc_ms = utc_ms;
    packet->utc_status = utc_status;
    packet->ant_status = ant_status;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_DATA, (const char *)packet, TTALINK_MSG_ID_RTK_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rtk_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_rtk_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rtk_sensors_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_rtk_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}
#endif

#endif

// MESSAGE RTK_SENSORS_DATA UNPACKING


/**
 * @brief Get field stas_number from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_stas_number(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field stas_number_tracked from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_stas_number_tracked(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field stas_number_usedinsolution from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_stas_number_usedinsolution(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field gps_time_week from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_rtk_sensors_data_get_gps_time_week(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field gps_time_ms from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_gps_time_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  5);
}

/**
 * @brief Get field sol_status from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_sol_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field pos_type from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field rtk_lat from rtk_sensors_data message
 *
 * @return  .
 */
static inline double ttalink_rtk_sensors_data_get_rtk_lat(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  11);
}

/**
 * @brief Get field rtk_lon from rtk_sensors_data message
 *
 * @return  .
 */
static inline double ttalink_rtk_sensors_data_get_rtk_lon(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  19);
}

/**
 * @brief Get field rtk_alt from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_rtk_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  27);
}

/**
 * @brief Get field undulation from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_undulation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  31);
}

/**
 * @brief Get field lat_std_deviation from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_lat_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  35);
}

/**
 * @brief Get field lon_std_deviation from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_lon_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  39);
}

/**
 * @brief Get field alt_std_deviation from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_alt_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  43);
}

/**
 * @brief Get field hor_speed from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_hor_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  47);
}

/**
 * @brief Get field trk_gnd from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_trk_gnd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  51);
}

/**
 * @brief Get field vert_speed from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_vert_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  55);
}

/**
 * @brief Get field length from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_length(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  59);
}

/**
 * @brief Get field heading from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_heading(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  63);
}

/**
 * @brief Get field pitch from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  67);
}

/**
 * @brief Get field hdg_std_deviation from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_hdg_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  71);
}

/**
 * @brief Get field ptch_std_deviation from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_ptch_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  75);
}

/**
 * @brief Get field GDOP from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_GDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  79);
}

/**
 * @brief Get field PDOP from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_PDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  83);
}

/**
 * @brief Get field HDOP from rtk_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_data_get_HDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  87);
}

/**
 * @brief Get field updateTime from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  91);
}

/**
 * @brief Get field vel_updateTime from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_vel_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  95);
}

/**
 * @brief Get field pos_updateTime from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_pos_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  99);
}

/**
 * @brief Get field dop_updateTime from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_dop_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  103);
}

/**
 * @brief Get field heading_updateTime from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_heading_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  107);
}

/**
 * @brief Get field heading_sol_status from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_heading_sol_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  111);
}

/**
 * @brief Get field heading_pos_type from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_heading_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  112);
}

/**
 * @brief Get field utc_year from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_utc_year(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  113);
}

/**
 * @brief Get field utc_month from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_utc_month(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  117);
}

/**
 * @brief Get field utc_day from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_utc_day(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  118);
}

/**
 * @brief Get field utc_hour from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_utc_hour(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  119);
}

/**
 * @brief Get field utc_min from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_utc_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  120);
}

/**
 * @brief Get field utc_ms from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_utc_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  121);
}

/**
 * @brief Get field utc_status from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_data_get_utc_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  125);
}

/**
 * @brief Get field ant_status from rtk_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_data_get_ant_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  129);
}

/**
 * @brief Decode a rtk_sensors_data message into a struct
 *
 * @param msg The message to decode
 * @param rtk_sensors_data C-struct to decode the message contents into
 */
static inline void ttalink_rtk_sensors_data_decode(const ttalink_message_t* msg, ttalink_rtk_sensors_data_t* rtk_sensors_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rtk_sensors_data->stas_number = ttalink_rtk_sensors_data_get_stas_number(msg);
    rtk_sensors_data->stas_number_tracked = ttalink_rtk_sensors_data_get_stas_number_tracked(msg);
    rtk_sensors_data->stas_number_usedinsolution = ttalink_rtk_sensors_data_get_stas_number_usedinsolution(msg);
    rtk_sensors_data->gps_time_week = ttalink_rtk_sensors_data_get_gps_time_week(msg);
    rtk_sensors_data->gps_time_ms = ttalink_rtk_sensors_data_get_gps_time_ms(msg);
    rtk_sensors_data->sol_status = ttalink_rtk_sensors_data_get_sol_status(msg);
    rtk_sensors_data->pos_type = ttalink_rtk_sensors_data_get_pos_type(msg);
    rtk_sensors_data->rtk_lat = ttalink_rtk_sensors_data_get_rtk_lat(msg);
    rtk_sensors_data->rtk_lon = ttalink_rtk_sensors_data_get_rtk_lon(msg);
    rtk_sensors_data->rtk_alt = ttalink_rtk_sensors_data_get_rtk_alt(msg);
    rtk_sensors_data->undulation = ttalink_rtk_sensors_data_get_undulation(msg);
    rtk_sensors_data->lat_std_deviation = ttalink_rtk_sensors_data_get_lat_std_deviation(msg);
    rtk_sensors_data->lon_std_deviation = ttalink_rtk_sensors_data_get_lon_std_deviation(msg);
    rtk_sensors_data->alt_std_deviation = ttalink_rtk_sensors_data_get_alt_std_deviation(msg);
    rtk_sensors_data->hor_speed = ttalink_rtk_sensors_data_get_hor_speed(msg);
    rtk_sensors_data->trk_gnd = ttalink_rtk_sensors_data_get_trk_gnd(msg);
    rtk_sensors_data->vert_speed = ttalink_rtk_sensors_data_get_vert_speed(msg);
    rtk_sensors_data->length = ttalink_rtk_sensors_data_get_length(msg);
    rtk_sensors_data->heading = ttalink_rtk_sensors_data_get_heading(msg);
    rtk_sensors_data->pitch = ttalink_rtk_sensors_data_get_pitch(msg);
    rtk_sensors_data->hdg_std_deviation = ttalink_rtk_sensors_data_get_hdg_std_deviation(msg);
    rtk_sensors_data->ptch_std_deviation = ttalink_rtk_sensors_data_get_ptch_std_deviation(msg);
    rtk_sensors_data->GDOP = ttalink_rtk_sensors_data_get_GDOP(msg);
    rtk_sensors_data->PDOP = ttalink_rtk_sensors_data_get_PDOP(msg);
    rtk_sensors_data->HDOP = ttalink_rtk_sensors_data_get_HDOP(msg);
    rtk_sensors_data->updateTime = ttalink_rtk_sensors_data_get_updateTime(msg);
    rtk_sensors_data->vel_updateTime = ttalink_rtk_sensors_data_get_vel_updateTime(msg);
    rtk_sensors_data->pos_updateTime = ttalink_rtk_sensors_data_get_pos_updateTime(msg);
    rtk_sensors_data->dop_updateTime = ttalink_rtk_sensors_data_get_dop_updateTime(msg);
    rtk_sensors_data->heading_updateTime = ttalink_rtk_sensors_data_get_heading_updateTime(msg);
    rtk_sensors_data->heading_sol_status = ttalink_rtk_sensors_data_get_heading_sol_status(msg);
    rtk_sensors_data->heading_pos_type = ttalink_rtk_sensors_data_get_heading_pos_type(msg);
    rtk_sensors_data->utc_year = ttalink_rtk_sensors_data_get_utc_year(msg);
    rtk_sensors_data->utc_month = ttalink_rtk_sensors_data_get_utc_month(msg);
    rtk_sensors_data->utc_day = ttalink_rtk_sensors_data_get_utc_day(msg);
    rtk_sensors_data->utc_hour = ttalink_rtk_sensors_data_get_utc_hour(msg);
    rtk_sensors_data->utc_min = ttalink_rtk_sensors_data_get_utc_min(msg);
    rtk_sensors_data->utc_ms = ttalink_rtk_sensors_data_get_utc_ms(msg);
    rtk_sensors_data->utc_status = ttalink_rtk_sensors_data_get_utc_status(msg);
    rtk_sensors_data->ant_status = ttalink_rtk_sensors_data_get_ant_status(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN? msg->len : TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN;
        memset(rtk_sensors_data, 0, TTALINK_MSG_ID_RTK_SENSORS_DATA_LEN);
    memcpy(rtk_sensors_data, _TTA_PAYLOAD(msg), len);
#endif
}
