#pragma once
// MESSAGE GPS_SENSORS_DATA PACKING

#define TTALINK_MSG_ID_GPS_SENSORS_DATA 2171

TTAPACKED(
typedef struct __ttalink_gps_sensors_data_t {
 uint8_t sensor_index; /*< */
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
 uint32_t update_time; /*<  .*/
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
}) ttalink_gps_sensors_data_t;

#define TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN 101
#define TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN 101
#define TTALINK_MSG_ID_2171_LEN 101
#define TTALINK_MSG_ID_2171_MIN_LEN 101

#define TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC 42
#define TTALINK_MSG_ID_2171_CRC 42

#define TTALINK_MSG_GPS_SENSORS_DATA_FIELD_DIFF_VEL_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GPS_SENSORS_DATA { \
    2171, \
    "GPS_SENSORS_DATA", \
    29, \
    {  { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_gps_sensors_data_t, sensor_index) }, \
         { "gpsFix", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_gps_sensors_data_t, gpsFix) }, \
         { "carrSoln", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_gps_sensors_data_t, carrSoln) }, \
         { "hp_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 3, offsetof(ttalink_gps_sensors_data_t, hp_longitude) }, \
         { "hp_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 11, offsetof(ttalink_gps_sensors_data_t, hp_latitude) }, \
         { "hp_hMSL", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_gps_sensors_data_t, hp_hMSL) }, \
         { "temperature", NULL, TTALINK_TYPE_INT16_T, 0, 23, offsetof(ttalink_gps_sensors_data_t, temperature) }, \
         { "longitude", NULL, TTALINK_TYPE_INT32_T, 0, 25, offsetof(ttalink_gps_sensors_data_t, longitude) }, \
         { "latitude", NULL, TTALINK_TYPE_INT32_T, 0, 29, offsetof(ttalink_gps_sensors_data_t, latitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_gps_sensors_data_t, altitude) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_gps_sensors_data_t, vel_n) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_gps_sensors_data_t, vel_e) }, \
         { "vel_d", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_gps_sensors_data_t, vel_d) }, \
         { "diff_vel", NULL, TTALINK_TYPE_FLOAT, 3, 49, offsetof(ttalink_gps_sensors_data_t, diff_vel) }, \
         { "ground_vel", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_gps_sensors_data_t, ground_vel) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 65, offsetof(ttalink_gps_sensors_data_t, heading) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 69, offsetof(ttalink_gps_sensors_data_t, update_time) }, \
         { "pos_acc", NULL, TTALINK_TYPE_FLOAT, 0, 73, offsetof(ttalink_gps_sensors_data_t, pos_acc) }, \
         { "speed_acc", NULL, TTALINK_TYPE_FLOAT, 0, 77, offsetof(ttalink_gps_sensors_data_t, speed_acc) }, \
         { "hor_acc", NULL, TTALINK_TYPE_FLOAT, 0, 81, offsetof(ttalink_gps_sensors_data_t, hor_acc) }, \
         { "ver_acc", NULL, TTALINK_TYPE_FLOAT, 0, 85, offsetof(ttalink_gps_sensors_data_t, ver_acc) }, \
         { "course_acc", NULL, TTALINK_TYPE_FLOAT, 0, 89, offsetof(ttalink_gps_sensors_data_t, course_acc) }, \
         { "sate_num", NULL, TTALINK_TYPE_UINT8_T, 0, 93, offsetof(ttalink_gps_sensors_data_t, sate_num) }, \
         { "year", NULL, TTALINK_TYPE_UINT16_T, 0, 94, offsetof(ttalink_gps_sensors_data_t, year) }, \
         { "month", NULL, TTALINK_TYPE_UINT8_T, 0, 96, offsetof(ttalink_gps_sensors_data_t, month) }, \
         { "date", NULL, TTALINK_TYPE_UINT8_T, 0, 97, offsetof(ttalink_gps_sensors_data_t, date) }, \
         { "hour", NULL, TTALINK_TYPE_UINT8_T, 0, 98, offsetof(ttalink_gps_sensors_data_t, hour) }, \
         { "minuter", NULL, TTALINK_TYPE_UINT8_T, 0, 99, offsetof(ttalink_gps_sensors_data_t, minuter) }, \
         { "second", NULL, TTALINK_TYPE_UINT8_T, 0, 100, offsetof(ttalink_gps_sensors_data_t, second) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GPS_SENSORS_DATA { \
    "GPS_SENSORS_DATA", \
    29, \
    {  { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_gps_sensors_data_t, sensor_index) }, \
         { "gpsFix", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_gps_sensors_data_t, gpsFix) }, \
         { "carrSoln", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_gps_sensors_data_t, carrSoln) }, \
         { "hp_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 3, offsetof(ttalink_gps_sensors_data_t, hp_longitude) }, \
         { "hp_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 11, offsetof(ttalink_gps_sensors_data_t, hp_latitude) }, \
         { "hp_hMSL", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_gps_sensors_data_t, hp_hMSL) }, \
         { "temperature", NULL, TTALINK_TYPE_INT16_T, 0, 23, offsetof(ttalink_gps_sensors_data_t, temperature) }, \
         { "longitude", NULL, TTALINK_TYPE_INT32_T, 0, 25, offsetof(ttalink_gps_sensors_data_t, longitude) }, \
         { "latitude", NULL, TTALINK_TYPE_INT32_T, 0, 29, offsetof(ttalink_gps_sensors_data_t, latitude) }, \
         { "altitude", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_gps_sensors_data_t, altitude) }, \
         { "vel_n", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_gps_sensors_data_t, vel_n) }, \
         { "vel_e", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_gps_sensors_data_t, vel_e) }, \
         { "vel_d", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_gps_sensors_data_t, vel_d) }, \
         { "diff_vel", NULL, TTALINK_TYPE_FLOAT, 3, 49, offsetof(ttalink_gps_sensors_data_t, diff_vel) }, \
         { "ground_vel", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_gps_sensors_data_t, ground_vel) }, \
         { "heading", NULL, TTALINK_TYPE_FLOAT, 0, 65, offsetof(ttalink_gps_sensors_data_t, heading) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 69, offsetof(ttalink_gps_sensors_data_t, update_time) }, \
         { "pos_acc", NULL, TTALINK_TYPE_FLOAT, 0, 73, offsetof(ttalink_gps_sensors_data_t, pos_acc) }, \
         { "speed_acc", NULL, TTALINK_TYPE_FLOAT, 0, 77, offsetof(ttalink_gps_sensors_data_t, speed_acc) }, \
         { "hor_acc", NULL, TTALINK_TYPE_FLOAT, 0, 81, offsetof(ttalink_gps_sensors_data_t, hor_acc) }, \
         { "ver_acc", NULL, TTALINK_TYPE_FLOAT, 0, 85, offsetof(ttalink_gps_sensors_data_t, ver_acc) }, \
         { "course_acc", NULL, TTALINK_TYPE_FLOAT, 0, 89, offsetof(ttalink_gps_sensors_data_t, course_acc) }, \
         { "sate_num", NULL, TTALINK_TYPE_UINT8_T, 0, 93, offsetof(ttalink_gps_sensors_data_t, sate_num) }, \
         { "year", NULL, TTALINK_TYPE_UINT16_T, 0, 94, offsetof(ttalink_gps_sensors_data_t, year) }, \
         { "month", NULL, TTALINK_TYPE_UINT8_T, 0, 96, offsetof(ttalink_gps_sensors_data_t, month) }, \
         { "date", NULL, TTALINK_TYPE_UINT8_T, 0, 97, offsetof(ttalink_gps_sensors_data_t, date) }, \
         { "hour", NULL, TTALINK_TYPE_UINT8_T, 0, 98, offsetof(ttalink_gps_sensors_data_t, hour) }, \
         { "minuter", NULL, TTALINK_TYPE_UINT8_T, 0, 99, offsetof(ttalink_gps_sensors_data_t, minuter) }, \
         { "second", NULL, TTALINK_TYPE_UINT8_T, 0, 100, offsetof(ttalink_gps_sensors_data_t, second) }, \
         } \
}
#endif


static inline uint16_t _ttalink_gps_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint8_t(buf, 1, gpsFix);
    _tta_put_uint8_t(buf, 2, carrSoln);
    _tta_put_double(buf, 3, hp_longitude);
    _tta_put_double(buf, 11, hp_latitude);
    _tta_put_float(buf, 19, hp_hMSL);
    _tta_put_int16_t(buf, 23, temperature);
    _tta_put_int32_t(buf, 25, longitude);
    _tta_put_int32_t(buf, 29, latitude);
    _tta_put_float(buf, 33, altitude);
    _tta_put_float(buf, 37, vel_n);
    _tta_put_float(buf, 41, vel_e);
    _tta_put_float(buf, 45, vel_d);
    _tta_put_float(buf, 61, ground_vel);
    _tta_put_float(buf, 65, heading);
    _tta_put_uint32_t(buf, 69, update_time);
    _tta_put_float(buf, 73, pos_acc);
    _tta_put_float(buf, 77, speed_acc);
    _tta_put_float(buf, 81, hor_acc);
    _tta_put_float(buf, 85, ver_acc);
    _tta_put_float(buf, 89, course_acc);
    _tta_put_uint8_t(buf, 93, sate_num);
    _tta_put_uint16_t(buf, 94, year);
    _tta_put_uint8_t(buf, 96, month);
    _tta_put_uint8_t(buf, 97, date);
    _tta_put_uint8_t(buf, 98, hour);
    _tta_put_uint8_t(buf, 99, minuter);
    _tta_put_uint8_t(buf, 100, second);
    _tta_put_float_array(buf, 49, diff_vel, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN);
#else
    ttalink_gps_sensors_data_t packet;
    packet.sensor_index = sensor_index;
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
    packet.update_time = update_time;
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
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GPS_SENSORS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a gps_sensors_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor_index 
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
 * @param update_time  .
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
static inline uint16_t ttalink_gps_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    return _ttalink_gps_sensors_data_pack(dst_addr, src_addr, msg,  sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/**
 * @brief Pack a gps_sensors_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor_index 
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
 * @param update_time  .
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
static inline uint16_t ttalink_gps_sensors_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    return _ttalink_gps_sensors_data_pack(dst_addr, src_addr, msg,  sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}


static inline uint16_t _ttalink_gps_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor_index,uint8_t gpsFix,uint8_t carrSoln,double hp_longitude,double hp_latitude,float hp_hMSL,int16_t temperature,int32_t longitude,int32_t latitude,float altitude,float vel_n,float vel_e,float vel_d,const float *diff_vel,float ground_vel,float heading,uint32_t update_time,float pos_acc,float speed_acc,float hor_acc,float ver_acc,float course_acc,uint8_t sate_num,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint8_t(buf, 1, gpsFix);
    _tta_put_uint8_t(buf, 2, carrSoln);
    _tta_put_double(buf, 3, hp_longitude);
    _tta_put_double(buf, 11, hp_latitude);
    _tta_put_float(buf, 19, hp_hMSL);
    _tta_put_int16_t(buf, 23, temperature);
    _tta_put_int32_t(buf, 25, longitude);
    _tta_put_int32_t(buf, 29, latitude);
    _tta_put_float(buf, 33, altitude);
    _tta_put_float(buf, 37, vel_n);
    _tta_put_float(buf, 41, vel_e);
    _tta_put_float(buf, 45, vel_d);
    _tta_put_float(buf, 61, ground_vel);
    _tta_put_float(buf, 65, heading);
    _tta_put_uint32_t(buf, 69, update_time);
    _tta_put_float(buf, 73, pos_acc);
    _tta_put_float(buf, 77, speed_acc);
    _tta_put_float(buf, 81, hor_acc);
    _tta_put_float(buf, 85, ver_acc);
    _tta_put_float(buf, 89, course_acc);
    _tta_put_uint8_t(buf, 93, sate_num);
    _tta_put_uint16_t(buf, 94, year);
    _tta_put_uint8_t(buf, 96, month);
    _tta_put_uint8_t(buf, 97, date);
    _tta_put_uint8_t(buf, 98, hour);
    _tta_put_uint8_t(buf, 99, minuter);
    _tta_put_uint8_t(buf, 100, second);
    _tta_put_float_array(buf, 49, diff_vel, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN);
#else
    ttalink_gps_sensors_data_t packet;
    packet.sensor_index = sensor_index;
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
    packet.update_time = update_time;
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
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GPS_SENSORS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a gps_sensors_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_index 
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
 * @param update_time  .
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
static inline uint16_t ttalink_gps_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor_index,uint8_t gpsFix,uint8_t carrSoln,double hp_longitude,double hp_latitude,float hp_hMSL,int16_t temperature,int32_t longitude,int32_t latitude,float altitude,float vel_n,float vel_e,float vel_d,const float *diff_vel,float ground_vel,float heading,uint32_t update_time,float pos_acc,float speed_acc,float hor_acc,float ver_acc,float course_acc,uint8_t sate_num,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second)
{
    return _ttalink_gps_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/**
 * @brief Pack a gps_sensors_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_index 
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
 * @param update_time  .
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
static inline uint16_t ttalink_gps_sensors_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor_index,uint8_t gpsFix,uint8_t carrSoln,double hp_longitude,double hp_latitude,float hp_hMSL,int16_t temperature,int32_t longitude,int32_t latitude,float altitude,float vel_n,float vel_e,float vel_d,const float *diff_vel,float ground_vel,float heading,uint32_t update_time,float pos_acc,float speed_acc,float hor_acc,float ver_acc,float course_acc,uint8_t sate_num,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second)
{
    return _ttalink_gps_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}


static inline uint16_t _ttalink_gps_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_gps_sensors_data_t* gps_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_gps_sensors_data_pack_nocrc(dst_addr, src_addr, msg, gps_sensors_data->sensor_index, gps_sensors_data->gpsFix, gps_sensors_data->carrSoln, gps_sensors_data->hp_longitude, gps_sensors_data->hp_latitude, gps_sensors_data->hp_hMSL, gps_sensors_data->temperature, gps_sensors_data->longitude, gps_sensors_data->latitude, gps_sensors_data->altitude, gps_sensors_data->vel_n, gps_sensors_data->vel_e, gps_sensors_data->vel_d, gps_sensors_data->diff_vel, gps_sensors_data->ground_vel, gps_sensors_data->heading, gps_sensors_data->update_time, gps_sensors_data->pos_acc, gps_sensors_data->speed_acc, gps_sensors_data->hor_acc, gps_sensors_data->ver_acc, gps_sensors_data->course_acc, gps_sensors_data->sate_num, gps_sensors_data->year, gps_sensors_data->month, gps_sensors_data->date, gps_sensors_data->hour, gps_sensors_data->minuter, gps_sensors_data->second);
    }else{
        return ttalink_gps_sensors_data_pack(dst_addr, src_addr, msg, gps_sensors_data->sensor_index, gps_sensors_data->gpsFix, gps_sensors_data->carrSoln, gps_sensors_data->hp_longitude, gps_sensors_data->hp_latitude, gps_sensors_data->hp_hMSL, gps_sensors_data->temperature, gps_sensors_data->longitude, gps_sensors_data->latitude, gps_sensors_data->altitude, gps_sensors_data->vel_n, gps_sensors_data->vel_e, gps_sensors_data->vel_d, gps_sensors_data->diff_vel, gps_sensors_data->ground_vel, gps_sensors_data->heading, gps_sensors_data->update_time, gps_sensors_data->pos_acc, gps_sensors_data->speed_acc, gps_sensors_data->hor_acc, gps_sensors_data->ver_acc, gps_sensors_data->course_acc, gps_sensors_data->sate_num, gps_sensors_data->year, gps_sensors_data->month, gps_sensors_data->date, gps_sensors_data->hour, gps_sensors_data->minuter, gps_sensors_data->second);
    }
    
}

/**
 * @brief Encode a gps_sensors_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param gps_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_gps_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_gps_sensors_data_t* gps_sensors_data)
{
    return _ttalink_gps_sensors_data_encode(dst_addr, src_addr, msg, gps_sensors_data, false);
}

/**
 * @brief Encode a gps_sensors_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param gps_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_gps_sensors_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_gps_sensors_data_t* gps_sensors_data)
{
    return _ttalink_gps_sensors_data_encode(dst_addr, src_addr, msg, gps_sensors_data, true);
}


static inline uint16_t _ttalink_gps_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_gps_sensors_data_t* gps_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_gps_sensors_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, gps_sensors_data->sensor_index, gps_sensors_data->gpsFix, gps_sensors_data->carrSoln, gps_sensors_data->hp_longitude, gps_sensors_data->hp_latitude, gps_sensors_data->hp_hMSL, gps_sensors_data->temperature, gps_sensors_data->longitude, gps_sensors_data->latitude, gps_sensors_data->altitude, gps_sensors_data->vel_n, gps_sensors_data->vel_e, gps_sensors_data->vel_d, gps_sensors_data->diff_vel, gps_sensors_data->ground_vel, gps_sensors_data->heading, gps_sensors_data->update_time, gps_sensors_data->pos_acc, gps_sensors_data->speed_acc, gps_sensors_data->hor_acc, gps_sensors_data->ver_acc, gps_sensors_data->course_acc, gps_sensors_data->sate_num, gps_sensors_data->year, gps_sensors_data->month, gps_sensors_data->date, gps_sensors_data->hour, gps_sensors_data->minuter, gps_sensors_data->second);
    }else{
        return ttalink_gps_sensors_data_pack_chan(dst_addr, src_addr, chan, msg, gps_sensors_data->sensor_index, gps_sensors_data->gpsFix, gps_sensors_data->carrSoln, gps_sensors_data->hp_longitude, gps_sensors_data->hp_latitude, gps_sensors_data->hp_hMSL, gps_sensors_data->temperature, gps_sensors_data->longitude, gps_sensors_data->latitude, gps_sensors_data->altitude, gps_sensors_data->vel_n, gps_sensors_data->vel_e, gps_sensors_data->vel_d, gps_sensors_data->diff_vel, gps_sensors_data->ground_vel, gps_sensors_data->heading, gps_sensors_data->update_time, gps_sensors_data->pos_acc, gps_sensors_data->speed_acc, gps_sensors_data->hor_acc, gps_sensors_data->ver_acc, gps_sensors_data->course_acc, gps_sensors_data->sate_num, gps_sensors_data->year, gps_sensors_data->month, gps_sensors_data->date, gps_sensors_data->hour, gps_sensors_data->minuter, gps_sensors_data->second);
    }
}

/**
 * @brief Encode a gps_sensors_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param gps_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_gps_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_gps_sensors_data_t* gps_sensors_data)
{
    return _ttalink_gps_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, gps_sensors_data, false);
}

/**
 * @brief Encode a gps_sensors_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param gps_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_gps_sensors_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_gps_sensors_data_t* gps_sensors_data)
{
    return _ttalink_gps_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, gps_sensors_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_gps_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN];
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint8_t(buf, 1, gpsFix);
    _tta_put_uint8_t(buf, 2, carrSoln);
    _tta_put_double(buf, 3, hp_longitude);
    _tta_put_double(buf, 11, hp_latitude);
    _tta_put_float(buf, 19, hp_hMSL);
    _tta_put_int16_t(buf, 23, temperature);
    _tta_put_int32_t(buf, 25, longitude);
    _tta_put_int32_t(buf, 29, latitude);
    _tta_put_float(buf, 33, altitude);
    _tta_put_float(buf, 37, vel_n);
    _tta_put_float(buf, 41, vel_e);
    _tta_put_float(buf, 45, vel_d);
    _tta_put_float(buf, 61, ground_vel);
    _tta_put_float(buf, 65, heading);
    _tta_put_uint32_t(buf, 69, update_time);
    _tta_put_float(buf, 73, pos_acc);
    _tta_put_float(buf, 77, speed_acc);
    _tta_put_float(buf, 81, hor_acc);
    _tta_put_float(buf, 85, ver_acc);
    _tta_put_float(buf, 89, course_acc);
    _tta_put_uint8_t(buf, 93, sate_num);
    _tta_put_uint16_t(buf, 94, year);
    _tta_put_uint8_t(buf, 96, month);
    _tta_put_uint8_t(buf, 97, date);
    _tta_put_uint8_t(buf, 98, hour);
    _tta_put_uint8_t(buf, 99, minuter);
    _tta_put_uint8_t(buf, 100, second);
    _tta_put_float_array(buf, 49, diff_vel, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GPS_SENSORS_DATA, buf, TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_gps_sensors_data_t packet;
    packet.sensor_index = sensor_index;
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
    packet.update_time = update_time;
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
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GPS_SENSORS_DATA, (const char *)&packet, TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a gps_sensors_data message
 * @param chan TTAlink channel to send the message
 *
 * @param sensor_index 
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
 * @param update_time  .
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
static inline void ttalink_gps_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_gps_sensors_data_send(dst_addr, src_addr, chan, sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/**
 * @brief Send a gps_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sensor_index 
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
 * @param update_time  .
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
static inline void ttalink_gps_sensors_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_gps_sensors_data_send(dst_addr, src_addr, chan, sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}


static inline void _ttalink_gps_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_gps_sensors_data_t* gps_sensors_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_gps_sensors_data_send_nocrc(dst_addr, src_addr, chan, gps_sensors_data->sensor_index, gps_sensors_data->gpsFix, gps_sensors_data->carrSoln, gps_sensors_data->hp_longitude, gps_sensors_data->hp_latitude, gps_sensors_data->hp_hMSL, gps_sensors_data->temperature, gps_sensors_data->longitude, gps_sensors_data->latitude, gps_sensors_data->altitude, gps_sensors_data->vel_n, gps_sensors_data->vel_e, gps_sensors_data->vel_d, gps_sensors_data->diff_vel, gps_sensors_data->ground_vel, gps_sensors_data->heading, gps_sensors_data->update_time, gps_sensors_data->pos_acc, gps_sensors_data->speed_acc, gps_sensors_data->hor_acc, gps_sensors_data->ver_acc, gps_sensors_data->course_acc, gps_sensors_data->sate_num, gps_sensors_data->year, gps_sensors_data->month, gps_sensors_data->date, gps_sensors_data->hour, gps_sensors_data->minuter, gps_sensors_data->second);
    }else{
        ttalink_gps_sensors_data_send(dst_addr, src_addr, chan, gps_sensors_data->sensor_index, gps_sensors_data->gpsFix, gps_sensors_data->carrSoln, gps_sensors_data->hp_longitude, gps_sensors_data->hp_latitude, gps_sensors_data->hp_hMSL, gps_sensors_data->temperature, gps_sensors_data->longitude, gps_sensors_data->latitude, gps_sensors_data->altitude, gps_sensors_data->vel_n, gps_sensors_data->vel_e, gps_sensors_data->vel_d, gps_sensors_data->diff_vel, gps_sensors_data->ground_vel, gps_sensors_data->heading, gps_sensors_data->update_time, gps_sensors_data->pos_acc, gps_sensors_data->speed_acc, gps_sensors_data->hor_acc, gps_sensors_data->ver_acc, gps_sensors_data->course_acc, gps_sensors_data->sate_num, gps_sensors_data->year, gps_sensors_data->month, gps_sensors_data->date, gps_sensors_data->hour, gps_sensors_data->minuter, gps_sensors_data->second);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GPS_SENSORS_DATA, (const char *)gps_sensors_data, TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a gps_sensors_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_gps_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_gps_sensors_data_t* gps_sensors_data)
{
    _ttalink_gps_sensors_data_send_struct(dst_addr, src_addr, chan, gps_sensors_data, false);
}

/**
 * @brief Send a gps_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_gps_sensors_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_gps_sensors_data_t* gps_sensors_data)
{
    _ttalink_gps_sensors_data_send_struct(dst_addr, src_addr, chan, gps_sensors_data, true);
}

#if TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_gps_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, sensor_index);
    _tta_put_uint8_t(buf, 1, gpsFix);
    _tta_put_uint8_t(buf, 2, carrSoln);
    _tta_put_double(buf, 3, hp_longitude);
    _tta_put_double(buf, 11, hp_latitude);
    _tta_put_float(buf, 19, hp_hMSL);
    _tta_put_int16_t(buf, 23, temperature);
    _tta_put_int32_t(buf, 25, longitude);
    _tta_put_int32_t(buf, 29, latitude);
    _tta_put_float(buf, 33, altitude);
    _tta_put_float(buf, 37, vel_n);
    _tta_put_float(buf, 41, vel_e);
    _tta_put_float(buf, 45, vel_d);
    _tta_put_float(buf, 61, ground_vel);
    _tta_put_float(buf, 65, heading);
    _tta_put_uint32_t(buf, 69, update_time);
    _tta_put_float(buf, 73, pos_acc);
    _tta_put_float(buf, 77, speed_acc);
    _tta_put_float(buf, 81, hor_acc);
    _tta_put_float(buf, 85, ver_acc);
    _tta_put_float(buf, 89, course_acc);
    _tta_put_uint8_t(buf, 93, sate_num);
    _tta_put_uint16_t(buf, 94, year);
    _tta_put_uint8_t(buf, 96, month);
    _tta_put_uint8_t(buf, 97, date);
    _tta_put_uint8_t(buf, 98, hour);
    _tta_put_uint8_t(buf, 99, minuter);
    _tta_put_uint8_t(buf, 100, second);
    _tta_put_float_array(buf, 49, diff_vel, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GPS_SENSORS_DATA, buf, TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_gps_sensors_data_t *packet = (ttalink_gps_sensors_data_t *)msgbuf;
    packet->sensor_index = sensor_index;
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
    packet->update_time = update_time;
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
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GPS_SENSORS_DATA, (const char *)packet, TTALINK_MSG_ID_GPS_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN, TTALINK_MSG_ID_GPS_SENSORS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_gps_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_gps_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_gps_sensors_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor_index, uint8_t gpsFix, uint8_t carrSoln, double hp_longitude, double hp_latitude, float hp_hMSL, int16_t temperature, int32_t longitude, int32_t latitude, float altitude, float vel_n, float vel_e, float vel_d, const float *diff_vel, float ground_vel, float heading, uint32_t update_time, float pos_acc, float speed_acc, float hor_acc, float ver_acc, float course_acc, uint8_t sate_num, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second)
{
    _ttalink_gps_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, sensor_index, gpsFix, carrSoln, hp_longitude, hp_latitude, hp_hMSL, temperature, longitude, latitude, altitude, vel_n, vel_e, vel_d, diff_vel, ground_vel, heading, update_time, pos_acc, speed_acc, hor_acc, ver_acc, course_acc, sate_num, year, month, date, hour, minuter, second, true);
}
#endif

#endif

// MESSAGE GPS_SENSORS_DATA UNPACKING


/**
 * @brief Get field sensor_index from gps_sensors_data message
 *
 * @return 
 */
static inline uint8_t ttalink_gps_sensors_data_get_sensor_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field gpsFix from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_gpsFix(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field carrSoln from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_carrSoln(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field hp_longitude from gps_sensors_data message
 *
 * @return  .
 */
static inline double ttalink_gps_sensors_data_get_hp_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  3);
}

/**
 * @brief Get field hp_latitude from gps_sensors_data message
 *
 * @return  .
 */
static inline double ttalink_gps_sensors_data_get_hp_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  11);
}

/**
 * @brief Get field hp_hMSL from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_hp_hMSL(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  19);
}

/**
 * @brief Get field temperature from gps_sensors_data message
 *
 * @return  .
 */
static inline int16_t ttalink_gps_sensors_data_get_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  23);
}

/**
 * @brief Get field longitude from gps_sensors_data message
 *
 * @return  .
 */
static inline int32_t ttalink_gps_sensors_data_get_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  25);
}

/**
 * @brief Get field latitude from gps_sensors_data message
 *
 * @return  .
 */
static inline int32_t ttalink_gps_sensors_data_get_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  29);
}

/**
 * @brief Get field altitude from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  33);
}

/**
 * @brief Get field vel_n from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_vel_n(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  37);
}

/**
 * @brief Get field vel_e from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_vel_e(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  41);
}

/**
 * @brief Get field vel_d from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_vel_d(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  45);
}

/**
 * @brief Get field diff_vel from gps_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_gps_sensors_data_get_diff_vel(const ttalink_message_t* msg, float *diff_vel)
{
    return _TTA_RETURN_float_array(msg, diff_vel, 3,  49);
}

/**
 * @brief Get field ground_vel from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_ground_vel(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  61);
}

/**
 * @brief Get field heading from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_heading(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  65);
}

/**
 * @brief Get field update_time from gps_sensors_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_gps_sensors_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  69);
}

/**
 * @brief Get field pos_acc from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_pos_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  73);
}

/**
 * @brief Get field speed_acc from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_speed_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  77);
}

/**
 * @brief Get field hor_acc from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_hor_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  81);
}

/**
 * @brief Get field ver_acc from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_ver_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  85);
}

/**
 * @brief Get field course_acc from gps_sensors_data message
 *
 * @return  .
 */
static inline float ttalink_gps_sensors_data_get_course_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  89);
}

/**
 * @brief Get field sate_num from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_sate_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  93);
}

/**
 * @brief Get field year from gps_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_gps_sensors_data_get_year(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  94);
}

/**
 * @brief Get field month from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_month(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  96);
}

/**
 * @brief Get field date from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_date(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  97);
}

/**
 * @brief Get field hour from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_hour(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  98);
}

/**
 * @brief Get field minuter from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_minuter(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  99);
}

/**
 * @brief Get field second from gps_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_gps_sensors_data_get_second(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  100);
}

/**
 * @brief Decode a gps_sensors_data message into a struct
 *
 * @param msg The message to decode
 * @param gps_sensors_data C-struct to decode the message contents into
 */
static inline void ttalink_gps_sensors_data_decode(const ttalink_message_t* msg, ttalink_gps_sensors_data_t* gps_sensors_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    gps_sensors_data->sensor_index = ttalink_gps_sensors_data_get_sensor_index(msg);
    gps_sensors_data->gpsFix = ttalink_gps_sensors_data_get_gpsFix(msg);
    gps_sensors_data->carrSoln = ttalink_gps_sensors_data_get_carrSoln(msg);
    gps_sensors_data->hp_longitude = ttalink_gps_sensors_data_get_hp_longitude(msg);
    gps_sensors_data->hp_latitude = ttalink_gps_sensors_data_get_hp_latitude(msg);
    gps_sensors_data->hp_hMSL = ttalink_gps_sensors_data_get_hp_hMSL(msg);
    gps_sensors_data->temperature = ttalink_gps_sensors_data_get_temperature(msg);
    gps_sensors_data->longitude = ttalink_gps_sensors_data_get_longitude(msg);
    gps_sensors_data->latitude = ttalink_gps_sensors_data_get_latitude(msg);
    gps_sensors_data->altitude = ttalink_gps_sensors_data_get_altitude(msg);
    gps_sensors_data->vel_n = ttalink_gps_sensors_data_get_vel_n(msg);
    gps_sensors_data->vel_e = ttalink_gps_sensors_data_get_vel_e(msg);
    gps_sensors_data->vel_d = ttalink_gps_sensors_data_get_vel_d(msg);
    ttalink_gps_sensors_data_get_diff_vel(msg, gps_sensors_data->diff_vel);
    gps_sensors_data->ground_vel = ttalink_gps_sensors_data_get_ground_vel(msg);
    gps_sensors_data->heading = ttalink_gps_sensors_data_get_heading(msg);
    gps_sensors_data->update_time = ttalink_gps_sensors_data_get_update_time(msg);
    gps_sensors_data->pos_acc = ttalink_gps_sensors_data_get_pos_acc(msg);
    gps_sensors_data->speed_acc = ttalink_gps_sensors_data_get_speed_acc(msg);
    gps_sensors_data->hor_acc = ttalink_gps_sensors_data_get_hor_acc(msg);
    gps_sensors_data->ver_acc = ttalink_gps_sensors_data_get_ver_acc(msg);
    gps_sensors_data->course_acc = ttalink_gps_sensors_data_get_course_acc(msg);
    gps_sensors_data->sate_num = ttalink_gps_sensors_data_get_sate_num(msg);
    gps_sensors_data->year = ttalink_gps_sensors_data_get_year(msg);
    gps_sensors_data->month = ttalink_gps_sensors_data_get_month(msg);
    gps_sensors_data->date = ttalink_gps_sensors_data_get_date(msg);
    gps_sensors_data->hour = ttalink_gps_sensors_data_get_hour(msg);
    gps_sensors_data->minuter = ttalink_gps_sensors_data_get_minuter(msg);
    gps_sensors_data->second = ttalink_gps_sensors_data_get_second(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN? msg->len : TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN;
        memset(gps_sensors_data, 0, TTALINK_MSG_ID_GPS_SENSORS_DATA_LEN);
    memcpy(gps_sensors_data, _TTA_PAYLOAD(msg), len);
#endif
}
