#pragma once
// MESSAGE POD_PTZ_9013_RTK PACKING

#define TTALINK_MSG_ID_POD_PTZ_9013_RTK 9013

TTAPACKED(
typedef struct __ttalink_pod_ptz_9013_rtk_t {
 uint32_t update_time; /*<  .*/
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
}) ttalink_pod_ptz_9013_rtk_t;

#define TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN 134
#define TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN 134
#define TTALINK_MSG_ID_9013_LEN 134
#define TTALINK_MSG_ID_9013_MIN_LEN 134

#define TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC 114
#define TTALINK_MSG_ID_9013_CRC 114



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_PTZ_9013_RTK { \
    9013, \
    "POD_PTZ_9013_RTK", \
    41, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_ptz_9013_rtk_t, update_time) }, \
         { "stas_number", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_ptz_9013_rtk_t, stas_number) }, \
         { "stas_number_tracked", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_ptz_9013_rtk_t, stas_number_tracked) }, \
         { "stas_number_usedinsolution", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_pod_ptz_9013_rtk_t, stas_number_usedinsolution) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 7, offsetof(ttalink_pod_ptz_9013_rtk_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_pod_ptz_9013_rtk_t, gps_time_ms) }, \
         { "sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_pod_ptz_9013_rtk_t, sol_status) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_pod_ptz_9013_rtk_t, pos_type) }, \
         { "rtk_lat", NULL, TTALINK_TYPE_DOUBLE, 0, 15, offsetof(ttalink_pod_ptz_9013_rtk_t, rtk_lat) }, \
         { "rtk_lon", NULL, TTALINK_TYPE_DOUBLE, 0, 23, offsetof(ttalink_pod_ptz_9013_rtk_t, rtk_lon) }, \
         { "rtk_alt", NULL, TTALINK_TYPE_FLOAT, 0, 31, offsetof(ttalink_pod_ptz_9013_rtk_t, rtk_alt) }, \
         { "undulation", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_pod_ptz_9013_rtk_t, undulation) }, \
         { "lat_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 39, offsetof(ttalink_pod_ptz_9013_rtk_t, lat_std_deviation) }, \
         { "lon_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_pod_ptz_9013_rtk_t, lon_std_deviation) }, \
         { "alt_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 47, offsetof(ttalink_pod_ptz_9013_rtk_t, alt_std_deviation) }, \
         { "hor_speed", NULL, TTALINK_TYPE_FLOAT, 0, 51, offsetof(ttalink_pod_ptz_9013_rtk_t, hor_speed) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 55, offsetof(ttalink_pod_ptz_9013_rtk_t, trk_gnd) }, \
         { "vert_speed", NULL, TTALINK_TYPE_FLOAT, 0, 59, offsetof(ttalink_pod_ptz_9013_rtk_t, vert_speed) }, \
         { "length", NULL, TTALINK_TYPE_FLOAT, 0, 63, offsetof(ttalink_pod_ptz_9013_rtk_t, length) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 67, offsetof(ttalink_pod_ptz_9013_rtk_t, heading) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 71, offsetof(ttalink_pod_ptz_9013_rtk_t, pitch) }, \
         { "hdg_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 75, offsetof(ttalink_pod_ptz_9013_rtk_t, hdg_std_deviation) }, \
         { "ptch_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 79, offsetof(ttalink_pod_ptz_9013_rtk_t, ptch_std_deviation) }, \
         { "GDOP", NULL, TTALINK_TYPE_FLOAT, 0, 83, offsetof(ttalink_pod_ptz_9013_rtk_t, GDOP) }, \
         { "PDOP", NULL, TTALINK_TYPE_FLOAT, 0, 87, offsetof(ttalink_pod_ptz_9013_rtk_t, PDOP) }, \
         { "HDOP", NULL, TTALINK_TYPE_FLOAT, 0, 91, offsetof(ttalink_pod_ptz_9013_rtk_t, HDOP) }, \
         { "updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 95, offsetof(ttalink_pod_ptz_9013_rtk_t, updateTime) }, \
         { "vel_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 99, offsetof(ttalink_pod_ptz_9013_rtk_t, vel_updateTime) }, \
         { "pos_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 103, offsetof(ttalink_pod_ptz_9013_rtk_t, pos_updateTime) }, \
         { "dop_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 107, offsetof(ttalink_pod_ptz_9013_rtk_t, dop_updateTime) }, \
         { "heading_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 111, offsetof(ttalink_pod_ptz_9013_rtk_t, heading_updateTime) }, \
         { "heading_sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 115, offsetof(ttalink_pod_ptz_9013_rtk_t, heading_sol_status) }, \
         { "heading_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 116, offsetof(ttalink_pod_ptz_9013_rtk_t, heading_pos_type) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 117, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 121, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 122, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 123, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 124, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 125, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_ms) }, \
         { "utc_status", NULL, TTALINK_TYPE_UINT32_T, 0, 129, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_status) }, \
         { "ant_status", NULL, TTALINK_TYPE_UINT8_T, 0, 133, offsetof(ttalink_pod_ptz_9013_rtk_t, ant_status) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_PTZ_9013_RTK { \
    "POD_PTZ_9013_RTK", \
    41, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_ptz_9013_rtk_t, update_time) }, \
         { "stas_number", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_ptz_9013_rtk_t, stas_number) }, \
         { "stas_number_tracked", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_ptz_9013_rtk_t, stas_number_tracked) }, \
         { "stas_number_usedinsolution", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_pod_ptz_9013_rtk_t, stas_number_usedinsolution) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 7, offsetof(ttalink_pod_ptz_9013_rtk_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_pod_ptz_9013_rtk_t, gps_time_ms) }, \
         { "sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_pod_ptz_9013_rtk_t, sol_status) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_pod_ptz_9013_rtk_t, pos_type) }, \
         { "rtk_lat", NULL, TTALINK_TYPE_DOUBLE, 0, 15, offsetof(ttalink_pod_ptz_9013_rtk_t, rtk_lat) }, \
         { "rtk_lon", NULL, TTALINK_TYPE_DOUBLE, 0, 23, offsetof(ttalink_pod_ptz_9013_rtk_t, rtk_lon) }, \
         { "rtk_alt", NULL, TTALINK_TYPE_FLOAT, 0, 31, offsetof(ttalink_pod_ptz_9013_rtk_t, rtk_alt) }, \
         { "undulation", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_pod_ptz_9013_rtk_t, undulation) }, \
         { "lat_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 39, offsetof(ttalink_pod_ptz_9013_rtk_t, lat_std_deviation) }, \
         { "lon_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_pod_ptz_9013_rtk_t, lon_std_deviation) }, \
         { "alt_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 47, offsetof(ttalink_pod_ptz_9013_rtk_t, alt_std_deviation) }, \
         { "hor_speed", NULL, TTALINK_TYPE_FLOAT, 0, 51, offsetof(ttalink_pod_ptz_9013_rtk_t, hor_speed) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 55, offsetof(ttalink_pod_ptz_9013_rtk_t, trk_gnd) }, \
         { "vert_speed", NULL, TTALINK_TYPE_FLOAT, 0, 59, offsetof(ttalink_pod_ptz_9013_rtk_t, vert_speed) }, \
         { "length", NULL, TTALINK_TYPE_FLOAT, 0, 63, offsetof(ttalink_pod_ptz_9013_rtk_t, length) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 67, offsetof(ttalink_pod_ptz_9013_rtk_t, heading) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 71, offsetof(ttalink_pod_ptz_9013_rtk_t, pitch) }, \
         { "hdg_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 75, offsetof(ttalink_pod_ptz_9013_rtk_t, hdg_std_deviation) }, \
         { "ptch_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 79, offsetof(ttalink_pod_ptz_9013_rtk_t, ptch_std_deviation) }, \
         { "GDOP", NULL, TTALINK_TYPE_FLOAT, 0, 83, offsetof(ttalink_pod_ptz_9013_rtk_t, GDOP) }, \
         { "PDOP", NULL, TTALINK_TYPE_FLOAT, 0, 87, offsetof(ttalink_pod_ptz_9013_rtk_t, PDOP) }, \
         { "HDOP", NULL, TTALINK_TYPE_FLOAT, 0, 91, offsetof(ttalink_pod_ptz_9013_rtk_t, HDOP) }, \
         { "updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 95, offsetof(ttalink_pod_ptz_9013_rtk_t, updateTime) }, \
         { "vel_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 99, offsetof(ttalink_pod_ptz_9013_rtk_t, vel_updateTime) }, \
         { "pos_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 103, offsetof(ttalink_pod_ptz_9013_rtk_t, pos_updateTime) }, \
         { "dop_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 107, offsetof(ttalink_pod_ptz_9013_rtk_t, dop_updateTime) }, \
         { "heading_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 111, offsetof(ttalink_pod_ptz_9013_rtk_t, heading_updateTime) }, \
         { "heading_sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 115, offsetof(ttalink_pod_ptz_9013_rtk_t, heading_sol_status) }, \
         { "heading_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 116, offsetof(ttalink_pod_ptz_9013_rtk_t, heading_pos_type) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 117, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 121, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 122, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 123, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 124, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 125, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_ms) }, \
         { "utc_status", NULL, TTALINK_TYPE_UINT32_T, 0, 129, offsetof(ttalink_pod_ptz_9013_rtk_t, utc_status) }, \
         { "ant_status", NULL, TTALINK_TYPE_UINT8_T, 0, 133, offsetof(ttalink_pod_ptz_9013_rtk_t, ant_status) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_ptz_9013_rtk_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, stas_number);
    _tta_put_uint8_t(buf, 5, stas_number_tracked);
    _tta_put_uint8_t(buf, 6, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 7, gps_time_week);
    _tta_put_uint32_t(buf, 9, gps_time_ms);
    _tta_put_uint8_t(buf, 13, sol_status);
    _tta_put_uint8_t(buf, 14, pos_type);
    _tta_put_double(buf, 15, rtk_lat);
    _tta_put_double(buf, 23, rtk_lon);
    _tta_put_float(buf, 31, rtk_alt);
    _tta_put_float(buf, 35, undulation);
    _tta_put_float(buf, 39, lat_std_deviation);
    _tta_put_float(buf, 43, lon_std_deviation);
    _tta_put_float(buf, 47, alt_std_deviation);
    _tta_put_float(buf, 51, hor_speed);
    _tta_put_float(buf, 55, trk_gnd);
    _tta_put_float(buf, 59, vert_speed);
    _tta_put_float(buf, 63, length);
    _tta_put_float(buf, 67, heading);
    _tta_put_float(buf, 71, pitch);
    _tta_put_float(buf, 75, hdg_std_deviation);
    _tta_put_float(buf, 79, ptch_std_deviation);
    _tta_put_float(buf, 83, GDOP);
    _tta_put_float(buf, 87, PDOP);
    _tta_put_float(buf, 91, HDOP);
    _tta_put_uint32_t(buf, 95, updateTime);
    _tta_put_uint32_t(buf, 99, vel_updateTime);
    _tta_put_uint32_t(buf, 103, pos_updateTime);
    _tta_put_uint32_t(buf, 107, dop_updateTime);
    _tta_put_uint32_t(buf, 111, heading_updateTime);
    _tta_put_uint8_t(buf, 115, heading_sol_status);
    _tta_put_uint8_t(buf, 116, heading_pos_type);
    _tta_put_uint32_t(buf, 117, utc_year);
    _tta_put_uint8_t(buf, 121, utc_month);
    _tta_put_uint8_t(buf, 122, utc_day);
    _tta_put_uint8_t(buf, 123, utc_hour);
    _tta_put_uint8_t(buf, 124, utc_min);
    _tta_put_uint32_t(buf, 125, utc_ms);
    _tta_put_uint32_t(buf, 129, utc_status);
    _tta_put_uint8_t(buf, 133, ant_status);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN);
#else
    ttalink_pod_ptz_9013_rtk_t packet;
    packet.update_time = update_time;
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

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_PTZ_9013_RTK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC, nocrc);
}

/**
 * @brief Pack a pod_ptz_9013_rtk message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
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
static inline uint16_t ttalink_pod_ptz_9013_rtk_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    return _ttalink_pod_ptz_9013_rtk_pack(dst_addr, src_addr, msg,  update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/**
 * @brief Pack a pod_ptz_9013_rtk message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
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
static inline uint16_t ttalink_pod_ptz_9013_rtk_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    return _ttalink_pod_ptz_9013_rtk_pack(dst_addr, src_addr, msg,  update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}


static inline uint16_t _ttalink_pod_ptz_9013_rtk_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t stas_number,uint8_t stas_number_tracked,uint8_t stas_number_usedinsolution,uint16_t gps_time_week,uint32_t gps_time_ms,uint8_t sol_status,uint8_t pos_type,double rtk_lat,double rtk_lon,float rtk_alt,float undulation,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float hor_speed,float trk_gnd,float vert_speed,float length,float heading,float pitch,float hdg_std_deviation,float ptch_std_deviation,float GDOP,float PDOP,float HDOP,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t utc_status,uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, stas_number);
    _tta_put_uint8_t(buf, 5, stas_number_tracked);
    _tta_put_uint8_t(buf, 6, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 7, gps_time_week);
    _tta_put_uint32_t(buf, 9, gps_time_ms);
    _tta_put_uint8_t(buf, 13, sol_status);
    _tta_put_uint8_t(buf, 14, pos_type);
    _tta_put_double(buf, 15, rtk_lat);
    _tta_put_double(buf, 23, rtk_lon);
    _tta_put_float(buf, 31, rtk_alt);
    _tta_put_float(buf, 35, undulation);
    _tta_put_float(buf, 39, lat_std_deviation);
    _tta_put_float(buf, 43, lon_std_deviation);
    _tta_put_float(buf, 47, alt_std_deviation);
    _tta_put_float(buf, 51, hor_speed);
    _tta_put_float(buf, 55, trk_gnd);
    _tta_put_float(buf, 59, vert_speed);
    _tta_put_float(buf, 63, length);
    _tta_put_float(buf, 67, heading);
    _tta_put_float(buf, 71, pitch);
    _tta_put_float(buf, 75, hdg_std_deviation);
    _tta_put_float(buf, 79, ptch_std_deviation);
    _tta_put_float(buf, 83, GDOP);
    _tta_put_float(buf, 87, PDOP);
    _tta_put_float(buf, 91, HDOP);
    _tta_put_uint32_t(buf, 95, updateTime);
    _tta_put_uint32_t(buf, 99, vel_updateTime);
    _tta_put_uint32_t(buf, 103, pos_updateTime);
    _tta_put_uint32_t(buf, 107, dop_updateTime);
    _tta_put_uint32_t(buf, 111, heading_updateTime);
    _tta_put_uint8_t(buf, 115, heading_sol_status);
    _tta_put_uint8_t(buf, 116, heading_pos_type);
    _tta_put_uint32_t(buf, 117, utc_year);
    _tta_put_uint8_t(buf, 121, utc_month);
    _tta_put_uint8_t(buf, 122, utc_day);
    _tta_put_uint8_t(buf, 123, utc_hour);
    _tta_put_uint8_t(buf, 124, utc_min);
    _tta_put_uint32_t(buf, 125, utc_ms);
    _tta_put_uint32_t(buf, 129, utc_status);
    _tta_put_uint8_t(buf, 133, ant_status);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN);
#else
    ttalink_pod_ptz_9013_rtk_t packet;
    packet.update_time = update_time;
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

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_PTZ_9013_RTK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC, nocrc);
}

/**
 * @brief Pack a pod_ptz_9013_rtk message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
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
static inline uint16_t ttalink_pod_ptz_9013_rtk_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t stas_number,uint8_t stas_number_tracked,uint8_t stas_number_usedinsolution,uint16_t gps_time_week,uint32_t gps_time_ms,uint8_t sol_status,uint8_t pos_type,double rtk_lat,double rtk_lon,float rtk_alt,float undulation,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float hor_speed,float trk_gnd,float vert_speed,float length,float heading,float pitch,float hdg_std_deviation,float ptch_std_deviation,float GDOP,float PDOP,float HDOP,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t utc_status,uint8_t ant_status)
{
    return _ttalink_pod_ptz_9013_rtk_pack_chan(dst_addr, src_addr, chan, msg,  update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/**
 * @brief Pack a pod_ptz_9013_rtk message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
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
static inline uint16_t ttalink_pod_ptz_9013_rtk_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t stas_number,uint8_t stas_number_tracked,uint8_t stas_number_usedinsolution,uint16_t gps_time_week,uint32_t gps_time_ms,uint8_t sol_status,uint8_t pos_type,double rtk_lat,double rtk_lon,float rtk_alt,float undulation,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float hor_speed,float trk_gnd,float vert_speed,float length,float heading,float pitch,float hdg_std_deviation,float ptch_std_deviation,float GDOP,float PDOP,float HDOP,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t utc_status,uint8_t ant_status)
{
    return _ttalink_pod_ptz_9013_rtk_pack_chan(dst_addr, src_addr, chan, msg,  update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}


static inline uint16_t _ttalink_pod_ptz_9013_rtk_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_ptz_9013_rtk_pack_nocrc(dst_addr, src_addr, msg, pod_ptz_9013_rtk->update_time, pod_ptz_9013_rtk->stas_number, pod_ptz_9013_rtk->stas_number_tracked, pod_ptz_9013_rtk->stas_number_usedinsolution, pod_ptz_9013_rtk->gps_time_week, pod_ptz_9013_rtk->gps_time_ms, pod_ptz_9013_rtk->sol_status, pod_ptz_9013_rtk->pos_type, pod_ptz_9013_rtk->rtk_lat, pod_ptz_9013_rtk->rtk_lon, pod_ptz_9013_rtk->rtk_alt, pod_ptz_9013_rtk->undulation, pod_ptz_9013_rtk->lat_std_deviation, pod_ptz_9013_rtk->lon_std_deviation, pod_ptz_9013_rtk->alt_std_deviation, pod_ptz_9013_rtk->hor_speed, pod_ptz_9013_rtk->trk_gnd, pod_ptz_9013_rtk->vert_speed, pod_ptz_9013_rtk->length, pod_ptz_9013_rtk->heading, pod_ptz_9013_rtk->pitch, pod_ptz_9013_rtk->hdg_std_deviation, pod_ptz_9013_rtk->ptch_std_deviation, pod_ptz_9013_rtk->GDOP, pod_ptz_9013_rtk->PDOP, pod_ptz_9013_rtk->HDOP, pod_ptz_9013_rtk->updateTime, pod_ptz_9013_rtk->vel_updateTime, pod_ptz_9013_rtk->pos_updateTime, pod_ptz_9013_rtk->dop_updateTime, pod_ptz_9013_rtk->heading_updateTime, pod_ptz_9013_rtk->heading_sol_status, pod_ptz_9013_rtk->heading_pos_type, pod_ptz_9013_rtk->utc_year, pod_ptz_9013_rtk->utc_month, pod_ptz_9013_rtk->utc_day, pod_ptz_9013_rtk->utc_hour, pod_ptz_9013_rtk->utc_min, pod_ptz_9013_rtk->utc_ms, pod_ptz_9013_rtk->utc_status, pod_ptz_9013_rtk->ant_status);
    }else{
        return ttalink_pod_ptz_9013_rtk_pack(dst_addr, src_addr, msg, pod_ptz_9013_rtk->update_time, pod_ptz_9013_rtk->stas_number, pod_ptz_9013_rtk->stas_number_tracked, pod_ptz_9013_rtk->stas_number_usedinsolution, pod_ptz_9013_rtk->gps_time_week, pod_ptz_9013_rtk->gps_time_ms, pod_ptz_9013_rtk->sol_status, pod_ptz_9013_rtk->pos_type, pod_ptz_9013_rtk->rtk_lat, pod_ptz_9013_rtk->rtk_lon, pod_ptz_9013_rtk->rtk_alt, pod_ptz_9013_rtk->undulation, pod_ptz_9013_rtk->lat_std_deviation, pod_ptz_9013_rtk->lon_std_deviation, pod_ptz_9013_rtk->alt_std_deviation, pod_ptz_9013_rtk->hor_speed, pod_ptz_9013_rtk->trk_gnd, pod_ptz_9013_rtk->vert_speed, pod_ptz_9013_rtk->length, pod_ptz_9013_rtk->heading, pod_ptz_9013_rtk->pitch, pod_ptz_9013_rtk->hdg_std_deviation, pod_ptz_9013_rtk->ptch_std_deviation, pod_ptz_9013_rtk->GDOP, pod_ptz_9013_rtk->PDOP, pod_ptz_9013_rtk->HDOP, pod_ptz_9013_rtk->updateTime, pod_ptz_9013_rtk->vel_updateTime, pod_ptz_9013_rtk->pos_updateTime, pod_ptz_9013_rtk->dop_updateTime, pod_ptz_9013_rtk->heading_updateTime, pod_ptz_9013_rtk->heading_sol_status, pod_ptz_9013_rtk->heading_pos_type, pod_ptz_9013_rtk->utc_year, pod_ptz_9013_rtk->utc_month, pod_ptz_9013_rtk->utc_day, pod_ptz_9013_rtk->utc_hour, pod_ptz_9013_rtk->utc_min, pod_ptz_9013_rtk->utc_ms, pod_ptz_9013_rtk->utc_status, pod_ptz_9013_rtk->ant_status);
    }
    
}

/**
 * @brief Encode a pod_ptz_9013_rtk struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9013_rtk C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9013_rtk_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk)
{
    return _ttalink_pod_ptz_9013_rtk_encode(dst_addr, src_addr, msg, pod_ptz_9013_rtk, false);
}

/**
 * @brief Encode a pod_ptz_9013_rtk struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9013_rtk C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9013_rtk_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk)
{
    return _ttalink_pod_ptz_9013_rtk_encode(dst_addr, src_addr, msg, pod_ptz_9013_rtk, true);
}


static inline uint16_t _ttalink_pod_ptz_9013_rtk_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_ptz_9013_rtk_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_ptz_9013_rtk->update_time, pod_ptz_9013_rtk->stas_number, pod_ptz_9013_rtk->stas_number_tracked, pod_ptz_9013_rtk->stas_number_usedinsolution, pod_ptz_9013_rtk->gps_time_week, pod_ptz_9013_rtk->gps_time_ms, pod_ptz_9013_rtk->sol_status, pod_ptz_9013_rtk->pos_type, pod_ptz_9013_rtk->rtk_lat, pod_ptz_9013_rtk->rtk_lon, pod_ptz_9013_rtk->rtk_alt, pod_ptz_9013_rtk->undulation, pod_ptz_9013_rtk->lat_std_deviation, pod_ptz_9013_rtk->lon_std_deviation, pod_ptz_9013_rtk->alt_std_deviation, pod_ptz_9013_rtk->hor_speed, pod_ptz_9013_rtk->trk_gnd, pod_ptz_9013_rtk->vert_speed, pod_ptz_9013_rtk->length, pod_ptz_9013_rtk->heading, pod_ptz_9013_rtk->pitch, pod_ptz_9013_rtk->hdg_std_deviation, pod_ptz_9013_rtk->ptch_std_deviation, pod_ptz_9013_rtk->GDOP, pod_ptz_9013_rtk->PDOP, pod_ptz_9013_rtk->HDOP, pod_ptz_9013_rtk->updateTime, pod_ptz_9013_rtk->vel_updateTime, pod_ptz_9013_rtk->pos_updateTime, pod_ptz_9013_rtk->dop_updateTime, pod_ptz_9013_rtk->heading_updateTime, pod_ptz_9013_rtk->heading_sol_status, pod_ptz_9013_rtk->heading_pos_type, pod_ptz_9013_rtk->utc_year, pod_ptz_9013_rtk->utc_month, pod_ptz_9013_rtk->utc_day, pod_ptz_9013_rtk->utc_hour, pod_ptz_9013_rtk->utc_min, pod_ptz_9013_rtk->utc_ms, pod_ptz_9013_rtk->utc_status, pod_ptz_9013_rtk->ant_status);
    }else{
        return ttalink_pod_ptz_9013_rtk_pack_chan(dst_addr, src_addr, chan, msg, pod_ptz_9013_rtk->update_time, pod_ptz_9013_rtk->stas_number, pod_ptz_9013_rtk->stas_number_tracked, pod_ptz_9013_rtk->stas_number_usedinsolution, pod_ptz_9013_rtk->gps_time_week, pod_ptz_9013_rtk->gps_time_ms, pod_ptz_9013_rtk->sol_status, pod_ptz_9013_rtk->pos_type, pod_ptz_9013_rtk->rtk_lat, pod_ptz_9013_rtk->rtk_lon, pod_ptz_9013_rtk->rtk_alt, pod_ptz_9013_rtk->undulation, pod_ptz_9013_rtk->lat_std_deviation, pod_ptz_9013_rtk->lon_std_deviation, pod_ptz_9013_rtk->alt_std_deviation, pod_ptz_9013_rtk->hor_speed, pod_ptz_9013_rtk->trk_gnd, pod_ptz_9013_rtk->vert_speed, pod_ptz_9013_rtk->length, pod_ptz_9013_rtk->heading, pod_ptz_9013_rtk->pitch, pod_ptz_9013_rtk->hdg_std_deviation, pod_ptz_9013_rtk->ptch_std_deviation, pod_ptz_9013_rtk->GDOP, pod_ptz_9013_rtk->PDOP, pod_ptz_9013_rtk->HDOP, pod_ptz_9013_rtk->updateTime, pod_ptz_9013_rtk->vel_updateTime, pod_ptz_9013_rtk->pos_updateTime, pod_ptz_9013_rtk->dop_updateTime, pod_ptz_9013_rtk->heading_updateTime, pod_ptz_9013_rtk->heading_sol_status, pod_ptz_9013_rtk->heading_pos_type, pod_ptz_9013_rtk->utc_year, pod_ptz_9013_rtk->utc_month, pod_ptz_9013_rtk->utc_day, pod_ptz_9013_rtk->utc_hour, pod_ptz_9013_rtk->utc_min, pod_ptz_9013_rtk->utc_ms, pod_ptz_9013_rtk->utc_status, pod_ptz_9013_rtk->ant_status);
    }
}

/**
 * @brief Encode a pod_ptz_9013_rtk struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9013_rtk C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9013_rtk_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk)
{
    return _ttalink_pod_ptz_9013_rtk_encode_chan(dst_addr, src_addr, chan, msg, pod_ptz_9013_rtk, false);
}

/**
 * @brief Encode a pod_ptz_9013_rtk struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9013_rtk C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9013_rtk_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk)
{
    return _ttalink_pod_ptz_9013_rtk_encode_chan(dst_addr, src_addr, chan, msg, pod_ptz_9013_rtk, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_ptz_9013_rtk_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, stas_number);
    _tta_put_uint8_t(buf, 5, stas_number_tracked);
    _tta_put_uint8_t(buf, 6, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 7, gps_time_week);
    _tta_put_uint32_t(buf, 9, gps_time_ms);
    _tta_put_uint8_t(buf, 13, sol_status);
    _tta_put_uint8_t(buf, 14, pos_type);
    _tta_put_double(buf, 15, rtk_lat);
    _tta_put_double(buf, 23, rtk_lon);
    _tta_put_float(buf, 31, rtk_alt);
    _tta_put_float(buf, 35, undulation);
    _tta_put_float(buf, 39, lat_std_deviation);
    _tta_put_float(buf, 43, lon_std_deviation);
    _tta_put_float(buf, 47, alt_std_deviation);
    _tta_put_float(buf, 51, hor_speed);
    _tta_put_float(buf, 55, trk_gnd);
    _tta_put_float(buf, 59, vert_speed);
    _tta_put_float(buf, 63, length);
    _tta_put_float(buf, 67, heading);
    _tta_put_float(buf, 71, pitch);
    _tta_put_float(buf, 75, hdg_std_deviation);
    _tta_put_float(buf, 79, ptch_std_deviation);
    _tta_put_float(buf, 83, GDOP);
    _tta_put_float(buf, 87, PDOP);
    _tta_put_float(buf, 91, HDOP);
    _tta_put_uint32_t(buf, 95, updateTime);
    _tta_put_uint32_t(buf, 99, vel_updateTime);
    _tta_put_uint32_t(buf, 103, pos_updateTime);
    _tta_put_uint32_t(buf, 107, dop_updateTime);
    _tta_put_uint32_t(buf, 111, heading_updateTime);
    _tta_put_uint8_t(buf, 115, heading_sol_status);
    _tta_put_uint8_t(buf, 116, heading_pos_type);
    _tta_put_uint32_t(buf, 117, utc_year);
    _tta_put_uint8_t(buf, 121, utc_month);
    _tta_put_uint8_t(buf, 122, utc_day);
    _tta_put_uint8_t(buf, 123, utc_hour);
    _tta_put_uint8_t(buf, 124, utc_min);
    _tta_put_uint32_t(buf, 125, utc_ms);
    _tta_put_uint32_t(buf, 129, utc_status);
    _tta_put_uint8_t(buf, 133, ant_status);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9013_RTK, buf, TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC, nocrc);
#else
    ttalink_pod_ptz_9013_rtk_t packet;
    packet.update_time = update_time;
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

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9013_RTK, (const char *)&packet, TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_ptz_9013_rtk message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
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
static inline void ttalink_pod_ptz_9013_rtk_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_pod_ptz_9013_rtk_send(dst_addr, src_addr, chan, update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/**
 * @brief Send a pod_ptz_9013_rtk message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
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
static inline void ttalink_pod_ptz_9013_rtk_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_pod_ptz_9013_rtk_send(dst_addr, src_addr, chan, update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}


static inline void _ttalink_pod_ptz_9013_rtk_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_ptz_9013_rtk_send_nocrc(dst_addr, src_addr, chan, pod_ptz_9013_rtk->update_time, pod_ptz_9013_rtk->stas_number, pod_ptz_9013_rtk->stas_number_tracked, pod_ptz_9013_rtk->stas_number_usedinsolution, pod_ptz_9013_rtk->gps_time_week, pod_ptz_9013_rtk->gps_time_ms, pod_ptz_9013_rtk->sol_status, pod_ptz_9013_rtk->pos_type, pod_ptz_9013_rtk->rtk_lat, pod_ptz_9013_rtk->rtk_lon, pod_ptz_9013_rtk->rtk_alt, pod_ptz_9013_rtk->undulation, pod_ptz_9013_rtk->lat_std_deviation, pod_ptz_9013_rtk->lon_std_deviation, pod_ptz_9013_rtk->alt_std_deviation, pod_ptz_9013_rtk->hor_speed, pod_ptz_9013_rtk->trk_gnd, pod_ptz_9013_rtk->vert_speed, pod_ptz_9013_rtk->length, pod_ptz_9013_rtk->heading, pod_ptz_9013_rtk->pitch, pod_ptz_9013_rtk->hdg_std_deviation, pod_ptz_9013_rtk->ptch_std_deviation, pod_ptz_9013_rtk->GDOP, pod_ptz_9013_rtk->PDOP, pod_ptz_9013_rtk->HDOP, pod_ptz_9013_rtk->updateTime, pod_ptz_9013_rtk->vel_updateTime, pod_ptz_9013_rtk->pos_updateTime, pod_ptz_9013_rtk->dop_updateTime, pod_ptz_9013_rtk->heading_updateTime, pod_ptz_9013_rtk->heading_sol_status, pod_ptz_9013_rtk->heading_pos_type, pod_ptz_9013_rtk->utc_year, pod_ptz_9013_rtk->utc_month, pod_ptz_9013_rtk->utc_day, pod_ptz_9013_rtk->utc_hour, pod_ptz_9013_rtk->utc_min, pod_ptz_9013_rtk->utc_ms, pod_ptz_9013_rtk->utc_status, pod_ptz_9013_rtk->ant_status);
    }else{
        ttalink_pod_ptz_9013_rtk_send(dst_addr, src_addr, chan, pod_ptz_9013_rtk->update_time, pod_ptz_9013_rtk->stas_number, pod_ptz_9013_rtk->stas_number_tracked, pod_ptz_9013_rtk->stas_number_usedinsolution, pod_ptz_9013_rtk->gps_time_week, pod_ptz_9013_rtk->gps_time_ms, pod_ptz_9013_rtk->sol_status, pod_ptz_9013_rtk->pos_type, pod_ptz_9013_rtk->rtk_lat, pod_ptz_9013_rtk->rtk_lon, pod_ptz_9013_rtk->rtk_alt, pod_ptz_9013_rtk->undulation, pod_ptz_9013_rtk->lat_std_deviation, pod_ptz_9013_rtk->lon_std_deviation, pod_ptz_9013_rtk->alt_std_deviation, pod_ptz_9013_rtk->hor_speed, pod_ptz_9013_rtk->trk_gnd, pod_ptz_9013_rtk->vert_speed, pod_ptz_9013_rtk->length, pod_ptz_9013_rtk->heading, pod_ptz_9013_rtk->pitch, pod_ptz_9013_rtk->hdg_std_deviation, pod_ptz_9013_rtk->ptch_std_deviation, pod_ptz_9013_rtk->GDOP, pod_ptz_9013_rtk->PDOP, pod_ptz_9013_rtk->HDOP, pod_ptz_9013_rtk->updateTime, pod_ptz_9013_rtk->vel_updateTime, pod_ptz_9013_rtk->pos_updateTime, pod_ptz_9013_rtk->dop_updateTime, pod_ptz_9013_rtk->heading_updateTime, pod_ptz_9013_rtk->heading_sol_status, pod_ptz_9013_rtk->heading_pos_type, pod_ptz_9013_rtk->utc_year, pod_ptz_9013_rtk->utc_month, pod_ptz_9013_rtk->utc_day, pod_ptz_9013_rtk->utc_hour, pod_ptz_9013_rtk->utc_min, pod_ptz_9013_rtk->utc_ms, pod_ptz_9013_rtk->utc_status, pod_ptz_9013_rtk->ant_status);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9013_RTK, (const char *)pod_ptz_9013_rtk, TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_ptz_9013_rtk message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_ptz_9013_rtk_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk)
{
    _ttalink_pod_ptz_9013_rtk_send_struct(dst_addr, src_addr, chan, pod_ptz_9013_rtk, false);
}

/**
 * @brief Send a pod_ptz_9013_rtk message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_ptz_9013_rtk_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk)
{
    _ttalink_pod_ptz_9013_rtk_send_struct(dst_addr, src_addr, chan, pod_ptz_9013_rtk, true);
}

#if TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_ptz_9013_rtk_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, stas_number);
    _tta_put_uint8_t(buf, 5, stas_number_tracked);
    _tta_put_uint8_t(buf, 6, stas_number_usedinsolution);
    _tta_put_uint16_t(buf, 7, gps_time_week);
    _tta_put_uint32_t(buf, 9, gps_time_ms);
    _tta_put_uint8_t(buf, 13, sol_status);
    _tta_put_uint8_t(buf, 14, pos_type);
    _tta_put_double(buf, 15, rtk_lat);
    _tta_put_double(buf, 23, rtk_lon);
    _tta_put_float(buf, 31, rtk_alt);
    _tta_put_float(buf, 35, undulation);
    _tta_put_float(buf, 39, lat_std_deviation);
    _tta_put_float(buf, 43, lon_std_deviation);
    _tta_put_float(buf, 47, alt_std_deviation);
    _tta_put_float(buf, 51, hor_speed);
    _tta_put_float(buf, 55, trk_gnd);
    _tta_put_float(buf, 59, vert_speed);
    _tta_put_float(buf, 63, length);
    _tta_put_float(buf, 67, heading);
    _tta_put_float(buf, 71, pitch);
    _tta_put_float(buf, 75, hdg_std_deviation);
    _tta_put_float(buf, 79, ptch_std_deviation);
    _tta_put_float(buf, 83, GDOP);
    _tta_put_float(buf, 87, PDOP);
    _tta_put_float(buf, 91, HDOP);
    _tta_put_uint32_t(buf, 95, updateTime);
    _tta_put_uint32_t(buf, 99, vel_updateTime);
    _tta_put_uint32_t(buf, 103, pos_updateTime);
    _tta_put_uint32_t(buf, 107, dop_updateTime);
    _tta_put_uint32_t(buf, 111, heading_updateTime);
    _tta_put_uint8_t(buf, 115, heading_sol_status);
    _tta_put_uint8_t(buf, 116, heading_pos_type);
    _tta_put_uint32_t(buf, 117, utc_year);
    _tta_put_uint8_t(buf, 121, utc_month);
    _tta_put_uint8_t(buf, 122, utc_day);
    _tta_put_uint8_t(buf, 123, utc_hour);
    _tta_put_uint8_t(buf, 124, utc_min);
    _tta_put_uint32_t(buf, 125, utc_ms);
    _tta_put_uint32_t(buf, 129, utc_status);
    _tta_put_uint8_t(buf, 133, ant_status);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9013_RTK, buf, TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC, nocrc);
#else
    ttalink_pod_ptz_9013_rtk_t *packet = (ttalink_pod_ptz_9013_rtk_t *)msgbuf;
    packet->update_time = update_time;
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

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9013_RTK, (const char *)packet, TTALINK_MSG_ID_POD_PTZ_9013_RTK_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN, TTALINK_MSG_ID_POD_PTZ_9013_RTK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_ptz_9013_rtk_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_pod_ptz_9013_rtk_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_ptz_9013_rtk_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t stas_number, uint8_t stas_number_tracked, uint8_t stas_number_usedinsolution, uint16_t gps_time_week, uint32_t gps_time_ms, uint8_t sol_status, uint8_t pos_type, double rtk_lat, double rtk_lon, float rtk_alt, float undulation, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float hor_speed, float trk_gnd, float vert_speed, float length, float heading, float pitch, float hdg_std_deviation, float ptch_std_deviation, float GDOP, float PDOP, float HDOP, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t utc_status, uint8_t ant_status)
{
    _ttalink_pod_ptz_9013_rtk_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, stas_number, stas_number_tracked, stas_number_usedinsolution, gps_time_week, gps_time_ms, sol_status, pos_type, rtk_lat, rtk_lon, rtk_alt, undulation, lat_std_deviation, lon_std_deviation, alt_std_deviation, hor_speed, trk_gnd, vert_speed, length, heading, pitch, hdg_std_deviation, ptch_std_deviation, GDOP, PDOP, HDOP, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, utc_status, ant_status, true);
}
#endif

#endif

// MESSAGE POD_PTZ_9013_RTK UNPACKING


/**
 * @brief Get field update_time from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field stas_number from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_stas_number(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field stas_number_tracked from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_stas_number_tracked(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field stas_number_usedinsolution from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_stas_number_usedinsolution(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field gps_time_week from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint16_t ttalink_pod_ptz_9013_rtk_get_gps_time_week(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  7);
}

/**
 * @brief Get field gps_time_ms from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_gps_time_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Get field sol_status from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_sol_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field pos_type from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field rtk_lat from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline double ttalink_pod_ptz_9013_rtk_get_rtk_lat(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  15);
}

/**
 * @brief Get field rtk_lon from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline double ttalink_pod_ptz_9013_rtk_get_rtk_lon(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  23);
}

/**
 * @brief Get field rtk_alt from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_rtk_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  31);
}

/**
 * @brief Get field undulation from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_undulation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  35);
}

/**
 * @brief Get field lat_std_deviation from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_lat_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  39);
}

/**
 * @brief Get field lon_std_deviation from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_lon_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  43);
}

/**
 * @brief Get field alt_std_deviation from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_alt_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  47);
}

/**
 * @brief Get field hor_speed from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_hor_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  51);
}

/**
 * @brief Get field trk_gnd from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_trk_gnd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  55);
}

/**
 * @brief Get field vert_speed from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_vert_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  59);
}

/**
 * @brief Get field length from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_length(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  63);
}

/**
 * @brief Get field heading from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_heading(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  67);
}

/**
 * @brief Get field pitch from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  71);
}

/**
 * @brief Get field hdg_std_deviation from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_hdg_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  75);
}

/**
 * @brief Get field ptch_std_deviation from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_ptch_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  79);
}

/**
 * @brief Get field GDOP from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_GDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  83);
}

/**
 * @brief Get field PDOP from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_PDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  87);
}

/**
 * @brief Get field HDOP from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9013_rtk_get_HDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  91);
}

/**
 * @brief Get field updateTime from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  95);
}

/**
 * @brief Get field vel_updateTime from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_vel_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  99);
}

/**
 * @brief Get field pos_updateTime from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_pos_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  103);
}

/**
 * @brief Get field dop_updateTime from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_dop_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  107);
}

/**
 * @brief Get field heading_updateTime from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_heading_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  111);
}

/**
 * @brief Get field heading_sol_status from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_heading_sol_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  115);
}

/**
 * @brief Get field heading_pos_type from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_heading_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  116);
}

/**
 * @brief Get field utc_year from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_utc_year(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  117);
}

/**
 * @brief Get field utc_month from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_utc_month(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  121);
}

/**
 * @brief Get field utc_day from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_utc_day(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  122);
}

/**
 * @brief Get field utc_hour from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_utc_hour(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  123);
}

/**
 * @brief Get field utc_min from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_utc_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  124);
}

/**
 * @brief Get field utc_ms from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_utc_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  125);
}

/**
 * @brief Get field utc_status from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9013_rtk_get_utc_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  129);
}

/**
 * @brief Get field ant_status from pod_ptz_9013_rtk message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9013_rtk_get_ant_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  133);
}

/**
 * @brief Decode a pod_ptz_9013_rtk message into a struct
 *
 * @param msg The message to decode
 * @param pod_ptz_9013_rtk C-struct to decode the message contents into
 */
static inline void ttalink_pod_ptz_9013_rtk_decode(const ttalink_message_t* msg, ttalink_pod_ptz_9013_rtk_t* pod_ptz_9013_rtk)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_ptz_9013_rtk->update_time = ttalink_pod_ptz_9013_rtk_get_update_time(msg);
    pod_ptz_9013_rtk->stas_number = ttalink_pod_ptz_9013_rtk_get_stas_number(msg);
    pod_ptz_9013_rtk->stas_number_tracked = ttalink_pod_ptz_9013_rtk_get_stas_number_tracked(msg);
    pod_ptz_9013_rtk->stas_number_usedinsolution = ttalink_pod_ptz_9013_rtk_get_stas_number_usedinsolution(msg);
    pod_ptz_9013_rtk->gps_time_week = ttalink_pod_ptz_9013_rtk_get_gps_time_week(msg);
    pod_ptz_9013_rtk->gps_time_ms = ttalink_pod_ptz_9013_rtk_get_gps_time_ms(msg);
    pod_ptz_9013_rtk->sol_status = ttalink_pod_ptz_9013_rtk_get_sol_status(msg);
    pod_ptz_9013_rtk->pos_type = ttalink_pod_ptz_9013_rtk_get_pos_type(msg);
    pod_ptz_9013_rtk->rtk_lat = ttalink_pod_ptz_9013_rtk_get_rtk_lat(msg);
    pod_ptz_9013_rtk->rtk_lon = ttalink_pod_ptz_9013_rtk_get_rtk_lon(msg);
    pod_ptz_9013_rtk->rtk_alt = ttalink_pod_ptz_9013_rtk_get_rtk_alt(msg);
    pod_ptz_9013_rtk->undulation = ttalink_pod_ptz_9013_rtk_get_undulation(msg);
    pod_ptz_9013_rtk->lat_std_deviation = ttalink_pod_ptz_9013_rtk_get_lat_std_deviation(msg);
    pod_ptz_9013_rtk->lon_std_deviation = ttalink_pod_ptz_9013_rtk_get_lon_std_deviation(msg);
    pod_ptz_9013_rtk->alt_std_deviation = ttalink_pod_ptz_9013_rtk_get_alt_std_deviation(msg);
    pod_ptz_9013_rtk->hor_speed = ttalink_pod_ptz_9013_rtk_get_hor_speed(msg);
    pod_ptz_9013_rtk->trk_gnd = ttalink_pod_ptz_9013_rtk_get_trk_gnd(msg);
    pod_ptz_9013_rtk->vert_speed = ttalink_pod_ptz_9013_rtk_get_vert_speed(msg);
    pod_ptz_9013_rtk->length = ttalink_pod_ptz_9013_rtk_get_length(msg);
    pod_ptz_9013_rtk->heading = ttalink_pod_ptz_9013_rtk_get_heading(msg);
    pod_ptz_9013_rtk->pitch = ttalink_pod_ptz_9013_rtk_get_pitch(msg);
    pod_ptz_9013_rtk->hdg_std_deviation = ttalink_pod_ptz_9013_rtk_get_hdg_std_deviation(msg);
    pod_ptz_9013_rtk->ptch_std_deviation = ttalink_pod_ptz_9013_rtk_get_ptch_std_deviation(msg);
    pod_ptz_9013_rtk->GDOP = ttalink_pod_ptz_9013_rtk_get_GDOP(msg);
    pod_ptz_9013_rtk->PDOP = ttalink_pod_ptz_9013_rtk_get_PDOP(msg);
    pod_ptz_9013_rtk->HDOP = ttalink_pod_ptz_9013_rtk_get_HDOP(msg);
    pod_ptz_9013_rtk->updateTime = ttalink_pod_ptz_9013_rtk_get_updateTime(msg);
    pod_ptz_9013_rtk->vel_updateTime = ttalink_pod_ptz_9013_rtk_get_vel_updateTime(msg);
    pod_ptz_9013_rtk->pos_updateTime = ttalink_pod_ptz_9013_rtk_get_pos_updateTime(msg);
    pod_ptz_9013_rtk->dop_updateTime = ttalink_pod_ptz_9013_rtk_get_dop_updateTime(msg);
    pod_ptz_9013_rtk->heading_updateTime = ttalink_pod_ptz_9013_rtk_get_heading_updateTime(msg);
    pod_ptz_9013_rtk->heading_sol_status = ttalink_pod_ptz_9013_rtk_get_heading_sol_status(msg);
    pod_ptz_9013_rtk->heading_pos_type = ttalink_pod_ptz_9013_rtk_get_heading_pos_type(msg);
    pod_ptz_9013_rtk->utc_year = ttalink_pod_ptz_9013_rtk_get_utc_year(msg);
    pod_ptz_9013_rtk->utc_month = ttalink_pod_ptz_9013_rtk_get_utc_month(msg);
    pod_ptz_9013_rtk->utc_day = ttalink_pod_ptz_9013_rtk_get_utc_day(msg);
    pod_ptz_9013_rtk->utc_hour = ttalink_pod_ptz_9013_rtk_get_utc_hour(msg);
    pod_ptz_9013_rtk->utc_min = ttalink_pod_ptz_9013_rtk_get_utc_min(msg);
    pod_ptz_9013_rtk->utc_ms = ttalink_pod_ptz_9013_rtk_get_utc_ms(msg);
    pod_ptz_9013_rtk->utc_status = ttalink_pod_ptz_9013_rtk_get_utc_status(msg);
    pod_ptz_9013_rtk->ant_status = ttalink_pod_ptz_9013_rtk_get_ant_status(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN? msg->len : TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN;
        memset(pod_ptz_9013_rtk, 0, TTALINK_MSG_ID_POD_PTZ_9013_RTK_LEN);
    memcpy(pod_ptz_9013_rtk, _TTA_PAYLOAD(msg), len);
#endif
}
