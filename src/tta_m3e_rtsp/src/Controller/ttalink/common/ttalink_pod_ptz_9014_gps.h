#pragma once
// MESSAGE POD_PTZ_9014_GPS PACKING

#define TTALINK_MSG_ID_POD_PTZ_9014_GPS 9014

TTAPACKED(
typedef struct __ttalink_pod_ptz_9014_gps_t {
 uint32_t update_time; /*<  .*/
 uint8_t gpsFix; /*<  .*/
 uint8_t carrSoln; /*<  .*/
 double hp_longitude; /*<  .*/
 double hp_latitude; /*<  .*/
 float hp_hMSL; /*<  .*/
 int16_t temperature; /*<  .*/
 int32_t longitude; /*<  .*/
 int32_t latitude; /*<  .*/
 float altitude; /*<  .*/
 float vel_n; /*<  .*/
 float vel_e; /*<  .*/
 float vel_d; /*<  .*/
 float diff_vel[3]; /*<  .*/
 float ground_vel; /*<  .*/
 float heading; /*<  .*/
 float pos_acc; /*<  .*/
 float speed_acc; /*<  .*/
 float hor_acc; /*<  .*/
 float ver_acc; /*<  .*/
 float course_acc; /*<  .*/
 uint8_t sate_num; /*<  .*/
 uint16_t year; /*<  .*/
 uint8_t month; /*<  .*/
 uint8_t date; /*<  .*/
 uint8_t hour; /*<  .*/
 uint8_t minuter; /*<  .*/
 uint8_t second; /*<  .*/
}) ttalink_pod_ptz_9014_gps_t;

#define TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN 100
#define TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN 100
#define TTALINK_MSG_ID_9014_LEN 100
#define TTALINK_MSG_ID_9014_MIN_LEN 100

#define TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC 94
#define TTALINK_MSG_ID_9014_CRC 94

#define TTALINK_MSG_POD_PTZ_9014_GPS_FIELD_DIFF_VEL_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_PTZ_9014_GPS { \
    9014, \
    "POD_PTZ_9014_GPS", \
    28, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_ptz_9014_gps_t, update_time) }, \
         { "gpsFix", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_ptz_9014_gps_t, gpsFix) }, \
         { "carrSoln", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_ptz_9014_gps_t, carrSoln) }, \
         { "hp_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 6, offsetof(ttalink_pod_ptz_9014_gps_t, hp_longitude) }, \
         { "hp_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 14, offsetof(ttalink_pod_ptz_9014_gps_t, hp_latitude) }, \
         { "hp_hMSL", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_pod_ptz_9014_gps_t, hp_hMSL) }, \
         { "temperature", NULL, TTALINK_TYPE_INT16_T, 0, 26, offsetof(ttalink_pod_ptz_9014_gps_t, temperature) }, \
         { "longitude", NULL, TTALINK_TYPE_INT32_T, 0, 28, offsetof(ttalink_pod_ptz_9014_gps_t, longitude) }, \
         { "latitude", NULL, TTALINK_TYPE_INT32_T, 0, 32, offsetof(ttalink_pod_ptz_9014_gps_t, latitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_pod_ptz_9014_gps_t, altitude) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_pod_ptz_9014_gps_t, vel_n) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_pod_ptz_9014_gps_t, vel_e) }, \
         { "vel_d", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_pod_ptz_9014_gps_t, vel_d) }, \
         { "diff_vel", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_pod_ptz_9014_gps_t, diff_vel) }, \
         { "ground_vel", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_pod_ptz_9014_gps_t, ground_vel) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 68, offsetof(ttalink_pod_ptz_9014_gps_t, heading) }, \
         { "pos_acc", NULL, TTALINK_TYPE_FLOAT, 0, 72, offsetof(ttalink_pod_ptz_9014_gps_t, pos_acc) }, \
         { "speed_acc", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_pod_ptz_9014_gps_t, speed_acc) }, \
         { "hor_acc", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_pod_ptz_9014_gps_t, hor_acc) }, \
         { "ver_acc", NULL, TTALINK_TYPE_FLOAT, 0, 84, offsetof(ttalink_pod_ptz_9014_gps_t, ver_acc) }, \
         { "course_acc", NULL, TTALINK_TYPE_FLOAT, 0, 88, offsetof(ttalink_pod_ptz_9014_gps_t, course_acc) }, \
         { "sate_num", NULL, TTALINK_TYPE_UINT8_T, 0, 92, offsetof(ttalink_pod_ptz_9014_gps_t, sate_num) }, \
         { "year", NULL, TTALINK_TYPE_UINT16_T, 0, 93, offsetof(ttalink_pod_ptz_9014_gps_t, year) }, \
         { "month", NULL, TTALINK_TYPE_UINT8_T, 0, 95, offsetof(ttalink_pod_ptz_9014_gps_t, month) }, \
         { "date", NULL, TTALINK_TYPE_UINT8_T, 0, 96, offsetof(ttalink_pod_ptz_9014_gps_t, date) }, \
         { "hour", NULL, TTALINK_TYPE_UINT8_T, 0, 97, offsetof(ttalink_pod_ptz_9014_gps_t, hour) }, \
         { "minuter", NULL, TTALINK_TYPE_UINT8_T, 0, 98, offsetof(ttalink_pod_ptz_9014_gps_t, minuter) }, \
         { "second", NULL, TTALINK_TYPE_UINT8_T, 0, 99, offsetof(ttalink_pod_ptz_9014_gps_t, second) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_PTZ_9014_GPS { \
    "POD_PTZ_9014_GPS", \
    28, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_ptz_9014_gps_t, update_time) }, \
         { "gpsFix", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_ptz_9014_gps_t, gpsFix) }, \
         { "carrSoln", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_pod_ptz_9014_gps_t, carrSoln) }, \
         { "hp_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 6, offsetof(ttalink_pod_ptz_9014_gps_t, hp_longitude) }, \
         { "hp_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 14, offsetof(ttalink_pod_ptz_9014_gps_t, hp_latitude) }, \
         { "hp_hMSL", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_pod_ptz_9014_gps_t, hp_hMSL) }, \
         { "temperature", NULL, TTALINK_TYPE_INT16_T, 0, 26, offsetof(ttalink_pod_ptz_9014_gps_t, temperature) }, \
         { "longitude", NULL, TTALINK_TYPE_INT32_T, 0, 28, offsetof(ttalink_pod_ptz_9014_gps_t, longitude) }, \
         { "latitude", NULL, TTALINK_TYPE_INT32_T, 0, 32, offsetof(ttalink_pod_ptz_9014_gps_t, latitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_pod_ptz_9014_gps_t, altitude) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_pod_ptz_9014_gps_t, vel_n) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_pod_ptz_9014_gps_t, vel_e) }, \
         { "vel_d", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_pod_ptz_9014_gps_t, vel_d) }, \
         { "diff_vel", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_pod_ptz_9014_gps_t, diff_vel) }, \
         { "ground_vel", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_pod_ptz_9014_gps_t, ground_vel) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 68, offsetof(ttalink_pod_ptz_9014_gps_t, heading) }, \
         { "pos_acc", NULL, TTALINK_TYPE_FLOAT, 0, 72, offsetof(ttalink_pod_ptz_9014_gps_t, pos_acc) }, \
         { "speed_acc", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_pod_ptz_9014_gps_t, speed_acc) }, \
         { "hor_acc", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_pod_ptz_9014_gps_t, hor_acc) }, \
         { "ver_acc", NULL, TTALINK_TYPE_FLOAT, 0, 84, offsetof(ttalink_pod_ptz_9014_gps_t, ver_acc) }, \
         { "course_acc", NULL, TTALINK_TYPE_FLOAT, 0, 88, offsetof(ttalink_pod_ptz_9014_gps_t, course_acc) }, \
         { "sate_num", NULL, TTALINK_TYPE_UINT8_T, 0, 92, offsetof(ttalink_pod_ptz_9014_gps_t, sate_num) }, \
         { "year", NULL, TTALINK_TYPE_UINT16_T, 0, 93, offsetof(ttalink_pod_ptz_9014_gps_t, year) }, \
         { "month", NULL, TTALINK_TYPE_UINT8_T, 0, 95, offsetof(ttalink_pod_ptz_9014_gps_t, month) }, \
         { "date", NULL, TTALINK_TYPE_UINT8_T, 0, 96, offsetof(ttalink_pod_ptz_9014_gps_t, date) }, \
         { "hour", NULL, TTALINK_TYPE_UINT8_T, 0, 97, offsetof(ttalink_pod_ptz_9014_gps_t, hour) }, \
         { "minuter", NULL, TTALINK_TYPE_UINT8_T, 0, 98, offsetof(ttalink_pod_ptz_9014_gps_t, minuter) }, \
         { "second", NULL, TTALINK_TYPE_UINT8_T, 0, 99, offsetof(ttalink_pod_ptz_9014_gps_t, second) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_ptz_9014_gps_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, gpsFix);
    _tta_put_uint8_t(buf, 5, carrSoln);
    _tta_put_double(buf, 6, hp_longitude);
    _tta_put_double(buf, 14, hp_latitude);
    _tta_put_float(buf, 22, hp_hMSL);
    _tta_put_int16_t(buf, 26, temperature);
    _tta_put_int32_t(buf, 28, longitude);
    _tta_put_int32_t(buf, 32, latitude);
    _tta_put_float(buf, 36, altitude);
    _tta_put_float(buf, 40, vel_n);
    _tta_put_float(buf, 44, vel_e);
    _tta_put_float(buf, 48, vel_d);
    _tta_put_float(buf, 64, ground_vel);
    _tta_put_float(buf, 68, heading);
    _tta_put_float(buf, 72, pos_acc);
    _tta_put_float(buf, 76, speed_acc);
    _tta_put_float(buf, 80, hor_acc);
    _tta_put_float(buf, 84, ver_acc);
    _tta_put_float(buf, 88, course_acc);
    _tta_put_uint8_t(buf, 92, sate_num);
    _tta_put_uint16_t(buf, 93, year);
    _tta_put_uint8_t(buf, 95, month);
    _tta_put_uint8_t(buf, 96, date);
    _tta_put_uint8_t(buf, 97, hour);
    _tta_put_uint8_t(buf, 98, minuter);
    _tta_put_uint8_t(buf, 99, second);
    _tta_put_float_array(buf, 52, diff_vel, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN);
#else
    ttalink_pod_ptz_9014_gps_t packet;
    packet.update_time = update_time;
    packet.gpsFix = gpsFix;
    packet.carrSoln = carrSoln;
    packet.hp_longitude = hp_longitude;
    packet.hp_latitude = hp_latitude;
    packet.hp_hMSL = hp_hMSL;
    packet.temperature = temperature;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.ground_vel = ground_vel;
    packet.heading = heading;
    packet.pos_acc = pos_acc;
    packet.speed_acc = speed_acc;
    packet.hor_acc = hor_acc;
    packet.ver_acc = ver_acc;
    packet.course_acc = course_acc;
    packet.sate_num = sate_num;
    packet.year = year;
    packet.month = month;
    packet.date = date;
    packet.hour = hour;
    packet.minuter = minuter;
    packet.second = second;
    tta_array_memcpy(packet.diff_vel, diff_vel, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_PTZ_9014_GPS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC, nocrc);
}

/**
 * @brief Pack a pod_ptz_9014_gps message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param gpsFix  .
 * @param carrSoln  .
 * @param hp_longitude  .
 * @param hp_latitude  .
 * @param hp_hMSL  .
 * @param temperature  .
 * @param longitude  .
 * @param latitude  .
 * @param altitude  .
 * @param vel_n  .
 * @param vel_e  .
 * @param vel_d  .
 * @param diff_vel  .
 * @param ground_vel  .
 * @param heading  .
 * @param pos_acc  .
 * @param speed_acc  .
 * @param hor_acc  .
 * @param ver_acc  .
 * @param course_acc  .
 * @param sate_num  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    return _ttalink_pod_ptz_9014_gps_pack(dst_addr, src_addr, msg,  update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/**
 * @brief Pack a pod_ptz_9014_gps message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param gpsFix  .
 * @param carrSoln  .
 * @param hp_longitude  .
 * @param hp_latitude  .
 * @param hp_hMSL  .
 * @param temperature  .
 * @param longitude  .
 * @param latitude  .
 * @param altitude  .
 * @param vel_n  .
 * @param vel_e  .
 * @param vel_d  .
 * @param diff_vel  .
 * @param ground_vel  .
 * @param heading  .
 * @param pos_acc  .
 * @param speed_acc  .
 * @param hor_acc  .
 * @param ver_acc  .
 * @param course_acc  .
 * @param sate_num  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    return _ttalink_pod_ptz_9014_gps_pack(dst_addr, src_addr, msg,  update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}


static inline uint16_t _ttalink_pod_ptz_9014_gps_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t gpsFix,uint8_t carrSoln,double hp_longitude,double hp_latitude,float hp_hMSL,int16_t temperature,int32_t longitude,int32_t latitude,float altitude,float vel_n,float vel_e,float vel_d,const float *diff_vel,float ground_vel,float heading,float pos_acc,float speed_acc,float hor_acc,float ver_acc,float course_acc,uint8_t sate_num,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, gpsFix);
    _tta_put_uint8_t(buf, 5, carrSoln);
    _tta_put_double(buf, 6, hp_longitude);
    _tta_put_double(buf, 14, hp_latitude);
    _tta_put_float(buf, 22, hp_hMSL);
    _tta_put_int16_t(buf, 26, temperature);
    _tta_put_int32_t(buf, 28, longitude);
    _tta_put_int32_t(buf, 32, latitude);
    _tta_put_float(buf, 36, altitude);
    _tta_put_float(buf, 40, vel_n);
    _tta_put_float(buf, 44, vel_e);
    _tta_put_float(buf, 48, vel_d);
    _tta_put_float(buf, 64, ground_vel);
    _tta_put_float(buf, 68, heading);
    _tta_put_float(buf, 72, pos_acc);
    _tta_put_float(buf, 76, speed_acc);
    _tta_put_float(buf, 80, hor_acc);
    _tta_put_float(buf, 84, ver_acc);
    _tta_put_float(buf, 88, course_acc);
    _tta_put_uint8_t(buf, 92, sate_num);
    _tta_put_uint16_t(buf, 93, year);
    _tta_put_uint8_t(buf, 95, month);
    _tta_put_uint8_t(buf, 96, date);
    _tta_put_uint8_t(buf, 97, hour);
    _tta_put_uint8_t(buf, 98, minuter);
    _tta_put_uint8_t(buf, 99, second);
    _tta_put_float_array(buf, 52, diff_vel, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN);
#else
    ttalink_pod_ptz_9014_gps_t packet;
    packet.update_time = update_time;
    packet.gpsFix = gpsFix;
    packet.carrSoln = carrSoln;
    packet.hp_longitude = hp_longitude;
    packet.hp_latitude = hp_latitude;
    packet.hp_hMSL = hp_hMSL;
    packet.temperature = temperature;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.ground_vel = ground_vel;
    packet.heading = heading;
    packet.pos_acc = pos_acc;
    packet.speed_acc = speed_acc;
    packet.hor_acc = hor_acc;
    packet.ver_acc = ver_acc;
    packet.course_acc = course_acc;
    packet.sate_num = sate_num;
    packet.year = year;
    packet.month = month;
    packet.date = date;
    packet.hour = hour;
    packet.minuter = minuter;
    packet.second = second;
    tta_array_memcpy(packet.diff_vel, diff_vel, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_PTZ_9014_GPS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC, nocrc);
}

/**
 * @brief Pack a pod_ptz_9014_gps message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param gpsFix  .
 * @param carrSoln  .
 * @param hp_longitude  .
 * @param hp_latitude  .
 * @param hp_hMSL  .
 * @param temperature  .
 * @param longitude  .
 * @param latitude  .
 * @param altitude  .
 * @param vel_n  .
 * @param vel_e  .
 * @param vel_d  .
 * @param diff_vel  .
 * @param ground_vel  .
 * @param heading  .
 * @param pos_acc  .
 * @param speed_acc  .
 * @param hor_acc  .
 * @param ver_acc  .
 * @param course_acc  .
 * @param sate_num  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t gpsFix,uint8_t carrSoln,double hp_longitude,double hp_latitude,float hp_hMSL,int16_t temperature,int32_t longitude,int32_t latitude,float altitude,float vel_n,float vel_e,float vel_d,const float *diff_vel,float ground_vel,float heading,float pos_acc,float speed_acc,float hor_acc,float ver_acc,float course_acc,uint8_t sate_num,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second)
{
    return _ttalink_pod_ptz_9014_gps_pack_chan(dst_addr, src_addr, chan, msg,  update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/**
 * @brief Pack a pod_ptz_9014_gps message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param gpsFix  .
 * @param carrSoln  .
 * @param hp_longitude  .
 * @param hp_latitude  .
 * @param hp_hMSL  .
 * @param temperature  .
 * @param longitude  .
 * @param latitude  .
 * @param altitude  .
 * @param vel_n  .
 * @param vel_e  .
 * @param vel_d  .
 * @param diff_vel  .
 * @param ground_vel  .
 * @param heading  .
 * @param pos_acc  .
 * @param speed_acc  .
 * @param hor_acc  .
 * @param ver_acc  .
 * @param course_acc  .
 * @param sate_num  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t gpsFix,uint8_t carrSoln,double hp_longitude,double hp_latitude,float hp_hMSL,int16_t temperature,int32_t longitude,int32_t latitude,float altitude,float vel_n,float vel_e,float vel_d,const float *diff_vel,float ground_vel,float heading,float pos_acc,float speed_acc,float hor_acc,float ver_acc,float course_acc,uint8_t sate_num,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second)
{
    return _ttalink_pod_ptz_9014_gps_pack_chan(dst_addr, src_addr, chan, msg,  update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}


static inline uint16_t _ttalink_pod_ptz_9014_gps_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_ptz_9014_gps_pack_nocrc(dst_addr, src_addr, msg, pod_ptz_9014_gps->update_time, pod_ptz_9014_gps->gpsFix, pod_ptz_9014_gps->carrSoln, pod_ptz_9014_gps->hp_longitude, pod_ptz_9014_gps->hp_latitude, pod_ptz_9014_gps->hp_hMSL, pod_ptz_9014_gps->temperature, pod_ptz_9014_gps->longitude, pod_ptz_9014_gps->latitude, pod_ptz_9014_gps->altitude, pod_ptz_9014_gps->vel_n, pod_ptz_9014_gps->vel_e, pod_ptz_9014_gps->vel_d, pod_ptz_9014_gps->diff_vel, pod_ptz_9014_gps->ground_vel, pod_ptz_9014_gps->heading, pod_ptz_9014_gps->pos_acc, pod_ptz_9014_gps->speed_acc, pod_ptz_9014_gps->hor_acc, pod_ptz_9014_gps->ver_acc, pod_ptz_9014_gps->course_acc, pod_ptz_9014_gps->sate_num, pod_ptz_9014_gps->year, pod_ptz_9014_gps->month, pod_ptz_9014_gps->date, pod_ptz_9014_gps->hour, pod_ptz_9014_gps->minuter, pod_ptz_9014_gps->second);
    }else{
        return ttalink_pod_ptz_9014_gps_pack(dst_addr, src_addr, msg, pod_ptz_9014_gps->update_time, pod_ptz_9014_gps->gpsFix, pod_ptz_9014_gps->carrSoln, pod_ptz_9014_gps->hp_longitude, pod_ptz_9014_gps->hp_latitude, pod_ptz_9014_gps->hp_hMSL, pod_ptz_9014_gps->temperature, pod_ptz_9014_gps->longitude, pod_ptz_9014_gps->latitude, pod_ptz_9014_gps->altitude, pod_ptz_9014_gps->vel_n, pod_ptz_9014_gps->vel_e, pod_ptz_9014_gps->vel_d, pod_ptz_9014_gps->diff_vel, pod_ptz_9014_gps->ground_vel, pod_ptz_9014_gps->heading, pod_ptz_9014_gps->pos_acc, pod_ptz_9014_gps->speed_acc, pod_ptz_9014_gps->hor_acc, pod_ptz_9014_gps->ver_acc, pod_ptz_9014_gps->course_acc, pod_ptz_9014_gps->sate_num, pod_ptz_9014_gps->year, pod_ptz_9014_gps->month, pod_ptz_9014_gps->date, pod_ptz_9014_gps->hour, pod_ptz_9014_gps->minuter, pod_ptz_9014_gps->second);
    }
    
}

/**
 * @brief Encode a pod_ptz_9014_gps struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9014_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps)
{
    return _ttalink_pod_ptz_9014_gps_encode(dst_addr, src_addr, msg, pod_ptz_9014_gps, false);
}

/**
 * @brief Encode a pod_ptz_9014_gps struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9014_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps)
{
    return _ttalink_pod_ptz_9014_gps_encode(dst_addr, src_addr, msg, pod_ptz_9014_gps, true);
}


static inline uint16_t _ttalink_pod_ptz_9014_gps_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_ptz_9014_gps_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_ptz_9014_gps->update_time, pod_ptz_9014_gps->gpsFix, pod_ptz_9014_gps->carrSoln, pod_ptz_9014_gps->hp_longitude, pod_ptz_9014_gps->hp_latitude, pod_ptz_9014_gps->hp_hMSL, pod_ptz_9014_gps->temperature, pod_ptz_9014_gps->longitude, pod_ptz_9014_gps->latitude, pod_ptz_9014_gps->altitude, pod_ptz_9014_gps->vel_n, pod_ptz_9014_gps->vel_e, pod_ptz_9014_gps->vel_d, pod_ptz_9014_gps->diff_vel, pod_ptz_9014_gps->ground_vel, pod_ptz_9014_gps->heading, pod_ptz_9014_gps->pos_acc, pod_ptz_9014_gps->speed_acc, pod_ptz_9014_gps->hor_acc, pod_ptz_9014_gps->ver_acc, pod_ptz_9014_gps->course_acc, pod_ptz_9014_gps->sate_num, pod_ptz_9014_gps->year, pod_ptz_9014_gps->month, pod_ptz_9014_gps->date, pod_ptz_9014_gps->hour, pod_ptz_9014_gps->minuter, pod_ptz_9014_gps->second);
    }else{
        return ttalink_pod_ptz_9014_gps_pack_chan(dst_addr, src_addr, chan, msg, pod_ptz_9014_gps->update_time, pod_ptz_9014_gps->gpsFix, pod_ptz_9014_gps->carrSoln, pod_ptz_9014_gps->hp_longitude, pod_ptz_9014_gps->hp_latitude, pod_ptz_9014_gps->hp_hMSL, pod_ptz_9014_gps->temperature, pod_ptz_9014_gps->longitude, pod_ptz_9014_gps->latitude, pod_ptz_9014_gps->altitude, pod_ptz_9014_gps->vel_n, pod_ptz_9014_gps->vel_e, pod_ptz_9014_gps->vel_d, pod_ptz_9014_gps->diff_vel, pod_ptz_9014_gps->ground_vel, pod_ptz_9014_gps->heading, pod_ptz_9014_gps->pos_acc, pod_ptz_9014_gps->speed_acc, pod_ptz_9014_gps->hor_acc, pod_ptz_9014_gps->ver_acc, pod_ptz_9014_gps->course_acc, pod_ptz_9014_gps->sate_num, pod_ptz_9014_gps->year, pod_ptz_9014_gps->month, pod_ptz_9014_gps->date, pod_ptz_9014_gps->hour, pod_ptz_9014_gps->minuter, pod_ptz_9014_gps->second);
    }
}

/**
 * @brief Encode a pod_ptz_9014_gps struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9014_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps)
{
    return _ttalink_pod_ptz_9014_gps_encode_chan(dst_addr, src_addr, chan, msg, pod_ptz_9014_gps, false);
}

/**
 * @brief Encode a pod_ptz_9014_gps struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9014_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps)
{
    return _ttalink_pod_ptz_9014_gps_encode_chan(dst_addr, src_addr, chan, msg, pod_ptz_9014_gps, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_ptz_9014_gps_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, gpsFix);
    _tta_put_uint8_t(buf, 5, carrSoln);
    _tta_put_double(buf, 6, hp_longitude);
    _tta_put_double(buf, 14, hp_latitude);
    _tta_put_float(buf, 22, hp_hMSL);
    _tta_put_int16_t(buf, 26, temperature);
    _tta_put_int32_t(buf, 28, longitude);
    _tta_put_int32_t(buf, 32, latitude);
    _tta_put_float(buf, 36, altitude);
    _tta_put_float(buf, 40, vel_n);
    _tta_put_float(buf, 44, vel_e);
    _tta_put_float(buf, 48, vel_d);
    _tta_put_float(buf, 64, ground_vel);
    _tta_put_float(buf, 68, heading);
    _tta_put_float(buf, 72, pos_acc);
    _tta_put_float(buf, 76, speed_acc);
    _tta_put_float(buf, 80, hor_acc);
    _tta_put_float(buf, 84, ver_acc);
    _tta_put_float(buf, 88, course_acc);
    _tta_put_uint8_t(buf, 92, sate_num);
    _tta_put_uint16_t(buf, 93, year);
    _tta_put_uint8_t(buf, 95, month);
    _tta_put_uint8_t(buf, 96, date);
    _tta_put_uint8_t(buf, 97, hour);
    _tta_put_uint8_t(buf, 98, minuter);
    _tta_put_uint8_t(buf, 99, second);
    _tta_put_float_array(buf, 52, diff_vel, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9014_GPS, buf, TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC, nocrc);
#else
    ttalink_pod_ptz_9014_gps_t packet;
    packet.update_time = update_time;
    packet.gpsFix = gpsFix;
    packet.carrSoln = carrSoln;
    packet.hp_longitude = hp_longitude;
    packet.hp_latitude = hp_latitude;
    packet.hp_hMSL = hp_hMSL;
    packet.temperature = temperature;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.ground_vel = ground_vel;
    packet.heading = heading;
    packet.pos_acc = pos_acc;
    packet.speed_acc = speed_acc;
    packet.hor_acc = hor_acc;
    packet.ver_acc = ver_acc;
    packet.course_acc = course_acc;
    packet.sate_num = sate_num;
    packet.year = year;
    packet.month = month;
    packet.date = date;
    packet.hour = hour;
    packet.minuter = minuter;
    packet.second = second;
    tta_array_memcpy(packet.diff_vel, diff_vel, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9014_GPS, (const char *)&packet, TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_ptz_9014_gps message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param gpsFix  .
 * @param carrSoln  .
 * @param hp_longitude  .
 * @param hp_latitude  .
 * @param hp_hMSL  .
 * @param temperature  .
 * @param longitude  .
 * @param latitude  .
 * @param altitude  .
 * @param vel_n  .
 * @param vel_e  .
 * @param vel_d  .
 * @param diff_vel  .
 * @param ground_vel  .
 * @param heading  .
 * @param pos_acc  .
 * @param speed_acc  .
 * @param hor_acc  .
 * @param ver_acc  .
 * @param course_acc  .
 * @param sate_num  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 */
static inline void ttalink_pod_ptz_9014_gps_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_pod_ptz_9014_gps_send(dst_addr, src_addr, chan, update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/**
 * @brief Send a pod_ptz_9014_gps message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param gpsFix  .
 * @param carrSoln  .
 * @param hp_longitude  .
 * @param hp_latitude  .
 * @param hp_hMSL  .
 * @param temperature  .
 * @param longitude  .
 * @param latitude  .
 * @param altitude  .
 * @param vel_n  .
 * @param vel_e  .
 * @param vel_d  .
 * @param diff_vel  .
 * @param ground_vel  .
 * @param heading  .
 * @param pos_acc  .
 * @param speed_acc  .
 * @param hor_acc  .
 * @param ver_acc  .
 * @param course_acc  .
 * @param sate_num  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 */
static inline void ttalink_pod_ptz_9014_gps_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_pod_ptz_9014_gps_send(dst_addr, src_addr, chan, update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}


static inline void _ttalink_pod_ptz_9014_gps_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_ptz_9014_gps_send_nocrc(dst_addr, src_addr, chan, pod_ptz_9014_gps->update_time, pod_ptz_9014_gps->gpsFix, pod_ptz_9014_gps->carrSoln, pod_ptz_9014_gps->hp_longitude, pod_ptz_9014_gps->hp_latitude, pod_ptz_9014_gps->hp_hMSL, pod_ptz_9014_gps->temperature, pod_ptz_9014_gps->longitude, pod_ptz_9014_gps->latitude, pod_ptz_9014_gps->altitude, pod_ptz_9014_gps->vel_n, pod_ptz_9014_gps->vel_e, pod_ptz_9014_gps->vel_d, pod_ptz_9014_gps->diff_vel, pod_ptz_9014_gps->ground_vel, pod_ptz_9014_gps->heading, pod_ptz_9014_gps->pos_acc, pod_ptz_9014_gps->speed_acc, pod_ptz_9014_gps->hor_acc, pod_ptz_9014_gps->ver_acc, pod_ptz_9014_gps->course_acc, pod_ptz_9014_gps->sate_num, pod_ptz_9014_gps->year, pod_ptz_9014_gps->month, pod_ptz_9014_gps->date, pod_ptz_9014_gps->hour, pod_ptz_9014_gps->minuter, pod_ptz_9014_gps->second);
    }else{
        ttalink_pod_ptz_9014_gps_send(dst_addr, src_addr, chan, pod_ptz_9014_gps->update_time, pod_ptz_9014_gps->gpsFix, pod_ptz_9014_gps->carrSoln, pod_ptz_9014_gps->hp_longitude, pod_ptz_9014_gps->hp_latitude, pod_ptz_9014_gps->hp_hMSL, pod_ptz_9014_gps->temperature, pod_ptz_9014_gps->longitude, pod_ptz_9014_gps->latitude, pod_ptz_9014_gps->altitude, pod_ptz_9014_gps->vel_n, pod_ptz_9014_gps->vel_e, pod_ptz_9014_gps->vel_d, pod_ptz_9014_gps->diff_vel, pod_ptz_9014_gps->ground_vel, pod_ptz_9014_gps->heading, pod_ptz_9014_gps->pos_acc, pod_ptz_9014_gps->speed_acc, pod_ptz_9014_gps->hor_acc, pod_ptz_9014_gps->ver_acc, pod_ptz_9014_gps->course_acc, pod_ptz_9014_gps->sate_num, pod_ptz_9014_gps->year, pod_ptz_9014_gps->month, pod_ptz_9014_gps->date, pod_ptz_9014_gps->hour, pod_ptz_9014_gps->minuter, pod_ptz_9014_gps->second);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9014_GPS, (const char *)pod_ptz_9014_gps, TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_ptz_9014_gps message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_ptz_9014_gps_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps)
{
    _ttalink_pod_ptz_9014_gps_send_struct(dst_addr, src_addr, chan, pod_ptz_9014_gps, false);
}

/**
 * @brief Send a pod_ptz_9014_gps message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_ptz_9014_gps_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps)
{
    _ttalink_pod_ptz_9014_gps_send_struct(dst_addr, src_addr, chan, pod_ptz_9014_gps, true);
}

#if TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_ptz_9014_gps_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, gpsFix);
    _tta_put_uint8_t(buf, 5, carrSoln);
    _tta_put_double(buf, 6, hp_longitude);
    _tta_put_double(buf, 14, hp_latitude);
    _tta_put_float(buf, 22, hp_hMSL);
    _tta_put_int16_t(buf, 26, temperature);
    _tta_put_int32_t(buf, 28, longitude);
    _tta_put_int32_t(buf, 32, latitude);
    _tta_put_float(buf, 36, altitude);
    _tta_put_float(buf, 40, vel_n);
    _tta_put_float(buf, 44, vel_e);
    _tta_put_float(buf, 48, vel_d);
    _tta_put_float(buf, 64, ground_vel);
    _tta_put_float(buf, 68, heading);
    _tta_put_float(buf, 72, pos_acc);
    _tta_put_float(buf, 76, speed_acc);
    _tta_put_float(buf, 80, hor_acc);
    _tta_put_float(buf, 84, ver_acc);
    _tta_put_float(buf, 88, course_acc);
    _tta_put_uint8_t(buf, 92, sate_num);
    _tta_put_uint16_t(buf, 93, year);
    _tta_put_uint8_t(buf, 95, month);
    _tta_put_uint8_t(buf, 96, date);
    _tta_put_uint8_t(buf, 97, hour);
    _tta_put_uint8_t(buf, 98, minuter);
    _tta_put_uint8_t(buf, 99, second);
    _tta_put_float_array(buf, 52, diff_vel, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9014_GPS, buf, TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC, nocrc);
#else
    ttalink_pod_ptz_9014_gps_t *packet = (ttalink_pod_ptz_9014_gps_t *)msgbuf;
    packet->update_time = update_time;
    packet->gpsFix = gpsFix;
    packet->carrSoln = carrSoln;
    packet->hp_longitude = hp_longitude;
    packet->hp_latitude = hp_latitude;
    packet->hp_hMSL = hp_hMSL;
    packet->temperature = temperature;
    packet->longitude = longitude;
    packet->latitude = latitude;
    packet->altitude = altitude;
    packet->vel_n = vel_n;
    packet->vel_e = vel_e;
    packet->vel_d = vel_d;
    packet->ground_vel = ground_vel;
    packet->heading = heading;
    packet->pos_acc = pos_acc;
    packet->speed_acc = speed_acc;
    packet->hor_acc = hor_acc;
    packet->ver_acc = ver_acc;
    packet->course_acc = course_acc;
    packet->sate_num = sate_num;
    packet->year = year;
    packet->month = month;
    packet->date = date;
    packet->hour = hour;
    packet->minuter = minuter;
    packet->second = second;
    tta_array_memcpy(packet->diff_vel, diff_vel, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9014_GPS, (const char *)packet, TTALINK_MSG_ID_POD_PTZ_9014_GPS_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN, TTALINK_MSG_ID_POD_PTZ_9014_GPS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_ptz_9014_gps_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_pod_ptz_9014_gps_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_ptz_9014_gps_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_pod_ptz_9014_gps_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}
#endif

#endif

// MESSAGE POD_PTZ_9014_GPS UNPACKING


/**
 * @brief Get field update_time from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint32_t ttalink_pod_ptz_9014_gps_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field gpsFix from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_gpsFix(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field carrSoln from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_carrSoln(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field hp_longitude from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline double ttalink_pod_ptz_9014_gps_get_hp_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  6);
}

/**
 * @brief Get field hp_latitude from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline double ttalink_pod_ptz_9014_gps_get_hp_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  14);
}

/**
 * @brief Get field hp_hMSL from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_hp_hMSL(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  22);
}

/**
 * @brief Get field temperature from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline int16_t ttalink_pod_ptz_9014_gps_get_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field longitude from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline int32_t ttalink_pod_ptz_9014_gps_get_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field latitude from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline int32_t ttalink_pod_ptz_9014_gps_get_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field altitude from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field vel_n from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_vel_n(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field vel_e from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_vel_e(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field vel_d from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_vel_d(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field diff_vel from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_get_diff_vel(const ttalink_message_t* msg, float *diff_vel)
{
    return _TTA_RETURN_float_array(msg, diff_vel, 3,  52);
}

/**
 * @brief Get field ground_vel from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_ground_vel(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  64);
}

/**
 * @brief Get field heading from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_heading(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  68);
}

/**
 * @brief Get field pos_acc from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_pos_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  72);
}

/**
 * @brief Get field speed_acc from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_speed_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  76);
}

/**
 * @brief Get field hor_acc from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_hor_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  80);
}

/**
 * @brief Get field ver_acc from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_ver_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  84);
}

/**
 * @brief Get field course_acc from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline float ttalink_pod_ptz_9014_gps_get_course_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  88);
}

/**
 * @brief Get field sate_num from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_sate_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  92);
}

/**
 * @brief Get field year from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint16_t ttalink_pod_ptz_9014_gps_get_year(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  93);
}

/**
 * @brief Get field month from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_month(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  95);
}

/**
 * @brief Get field date from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_date(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  96);
}

/**
 * @brief Get field hour from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_hour(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  97);
}

/**
 * @brief Get field minuter from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_minuter(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  98);
}

/**
 * @brief Get field second from pod_ptz_9014_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_ptz_9014_gps_get_second(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  99);
}

/**
 * @brief Decode a pod_ptz_9014_gps message into a struct
 *
 * @param msg The message to decode
 * @param pod_ptz_9014_gps C-struct to decode the message contents into
 */
static inline void ttalink_pod_ptz_9014_gps_decode(const ttalink_message_t* msg, ttalink_pod_ptz_9014_gps_t* pod_ptz_9014_gps)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_ptz_9014_gps->update_time = ttalink_pod_ptz_9014_gps_get_update_time(msg);
    pod_ptz_9014_gps->gpsFix = ttalink_pod_ptz_9014_gps_get_gpsFix(msg);
    pod_ptz_9014_gps->carrSoln = ttalink_pod_ptz_9014_gps_get_carrSoln(msg);
    pod_ptz_9014_gps->hp_longitude = ttalink_pod_ptz_9014_gps_get_hp_longitude(msg);
    pod_ptz_9014_gps->hp_latitude = ttalink_pod_ptz_9014_gps_get_hp_latitude(msg);
    pod_ptz_9014_gps->hp_hMSL = ttalink_pod_ptz_9014_gps_get_hp_hMSL(msg);
    pod_ptz_9014_gps->temperature = ttalink_pod_ptz_9014_gps_get_temperature(msg);
    pod_ptz_9014_gps->longitude = ttalink_pod_ptz_9014_gps_get_longitude(msg);
    pod_ptz_9014_gps->latitude = ttalink_pod_ptz_9014_gps_get_latitude(msg);
    pod_ptz_9014_gps->altitude = ttalink_pod_ptz_9014_gps_get_altitude(msg);
    pod_ptz_9014_gps->vel_n = ttalink_pod_ptz_9014_gps_get_vel_n(msg);
    pod_ptz_9014_gps->vel_e = ttalink_pod_ptz_9014_gps_get_vel_e(msg);
    pod_ptz_9014_gps->vel_d = ttalink_pod_ptz_9014_gps_get_vel_d(msg);
    ttalink_pod_ptz_9014_gps_get_diff_vel(msg, pod_ptz_9014_gps->diff_vel);
    pod_ptz_9014_gps->ground_vel = ttalink_pod_ptz_9014_gps_get_ground_vel(msg);
    pod_ptz_9014_gps->heading = ttalink_pod_ptz_9014_gps_get_heading(msg);
    pod_ptz_9014_gps->pos_acc = ttalink_pod_ptz_9014_gps_get_pos_acc(msg);
    pod_ptz_9014_gps->speed_acc = ttalink_pod_ptz_9014_gps_get_speed_acc(msg);
    pod_ptz_9014_gps->hor_acc = ttalink_pod_ptz_9014_gps_get_hor_acc(msg);
    pod_ptz_9014_gps->ver_acc = ttalink_pod_ptz_9014_gps_get_ver_acc(msg);
    pod_ptz_9014_gps->course_acc = ttalink_pod_ptz_9014_gps_get_course_acc(msg);
    pod_ptz_9014_gps->sate_num = ttalink_pod_ptz_9014_gps_get_sate_num(msg);
    pod_ptz_9014_gps->year = ttalink_pod_ptz_9014_gps_get_year(msg);
    pod_ptz_9014_gps->month = ttalink_pod_ptz_9014_gps_get_month(msg);
    pod_ptz_9014_gps->date = ttalink_pod_ptz_9014_gps_get_date(msg);
    pod_ptz_9014_gps->hour = ttalink_pod_ptz_9014_gps_get_hour(msg);
    pod_ptz_9014_gps->minuter = ttalink_pod_ptz_9014_gps_get_minuter(msg);
    pod_ptz_9014_gps->second = ttalink_pod_ptz_9014_gps_get_second(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN? msg->len : TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN;
        memset(pod_ptz_9014_gps, 0, TTALINK_MSG_ID_POD_PTZ_9014_GPS_LEN);
    memcpy(pod_ptz_9014_gps, _TTA_PAYLOAD(msg), len);
#endif
}
