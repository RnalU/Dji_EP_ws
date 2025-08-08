#pragma once
// MESSAGE STREAM_GPS PACKING

#define TTALINK_MSG_ID_STREAM_GPS 2144

TTAPACKED(
typedef struct __ttalink_stream_gps_t {
 uint8_t stream_type; /*<  .*/
 int32_t gps_lat; /*<  .*/
 int32_t gps_lon; /*<  .*/
 float gps_alt; /*<  .*/
 float gps_veln; /*<  .*/
 float gps_vele; /*<  .*/
 float gps_veld; /*<  .*/
 float gps_velu; /*<  .*/
 float gps_speed; /*<  .*/
 uint8_t nstart; /*<  .*/
 float yaw; /*<  .*/
 float p_dop; /*<  合精度.*/
 float v_dop; /*<  垂向精度.*/
 float h_dop; /*<  水平精度.*/
 float course_acc; /*<  航向精度.*/
 float speed_acc; /*<  速度精度.*/
 float rtk_root; /*<  .*/
 uint8_t pos_type; /*<  .*/
 uint8_t gps_state; /*<  .*/
 uint16_t year; /*<  .*/
 uint8_t month; /*<  .*/
 uint8_t date; /*<  .*/
 uint8_t hour; /*<  .*/
 uint8_t minuter; /*<  .*/
 uint8_t second; /*<  .*/
 uint64_t unix_timestamp_ms; /*<  .*/
 float gps_param[4]; /*<  .*/
}) ttalink_stream_gps_t;

#define TTALINK_MSG_ID_STREAM_GPS_LEN 95
#define TTALINK_MSG_ID_STREAM_GPS_MIN_LEN 95
#define TTALINK_MSG_ID_2144_LEN 95
#define TTALINK_MSG_ID_2144_MIN_LEN 95

#define TTALINK_MSG_ID_STREAM_GPS_CRC 85
#define TTALINK_MSG_ID_2144_CRC 85

#define TTALINK_MSG_STREAM_GPS_FIELD_GPS_PARAM_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_STREAM_GPS { \
    2144, \
    "STREAM_GPS", \
    27, \
    {  { "stream_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_stream_gps_t, stream_type) }, \
         { "gps_lat", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_stream_gps_t, gps_lat) }, \
         { "gps_lon", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_stream_gps_t, gps_lon) }, \
         { "gps_alt", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_stream_gps_t, gps_alt) }, \
         { "gps_veln", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_stream_gps_t, gps_veln) }, \
         { "gps_vele", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_stream_gps_t, gps_vele) }, \
         { "gps_veld", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_stream_gps_t, gps_veld) }, \
         { "gps_velu", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_stream_gps_t, gps_velu) }, \
         { "gps_speed", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_stream_gps_t, gps_speed) }, \
         { "nstart", NULL, TTALINK_TYPE_UINT8_T, 0, 33, offsetof(ttalink_stream_gps_t, nstart) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_stream_gps_t, yaw) }, \
         { "p_dop", NULL, TTALINK_TYPE_FLOAT, 0, 38, offsetof(ttalink_stream_gps_t, p_dop) }, \
         { "v_dop", NULL, TTALINK_TYPE_FLOAT, 0, 42, offsetof(ttalink_stream_gps_t, v_dop) }, \
         { "h_dop", NULL, TTALINK_TYPE_FLOAT, 0, 46, offsetof(ttalink_stream_gps_t, h_dop) }, \
         { "course_acc", NULL, TTALINK_TYPE_FLOAT, 0, 50, offsetof(ttalink_stream_gps_t, course_acc) }, \
         { "speed_acc", NULL, TTALINK_TYPE_FLOAT, 0, 54, offsetof(ttalink_stream_gps_t, speed_acc) }, \
         { "rtk_root", NULL, TTALINK_TYPE_FLOAT, 0, 58, offsetof(ttalink_stream_gps_t, rtk_root) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 62, offsetof(ttalink_stream_gps_t, pos_type) }, \
         { "gps_state", NULL, TTALINK_TYPE_UINT8_T, 0, 63, offsetof(ttalink_stream_gps_t, gps_state) }, \
         { "year", NULL, TTALINK_TYPE_UINT16_T, 0, 64, offsetof(ttalink_stream_gps_t, year) }, \
         { "month", NULL, TTALINK_TYPE_UINT8_T, 0, 66, offsetof(ttalink_stream_gps_t, month) }, \
         { "date", NULL, TTALINK_TYPE_UINT8_T, 0, 67, offsetof(ttalink_stream_gps_t, date) }, \
         { "hour", NULL, TTALINK_TYPE_UINT8_T, 0, 68, offsetof(ttalink_stream_gps_t, hour) }, \
         { "minuter", NULL, TTALINK_TYPE_UINT8_T, 0, 69, offsetof(ttalink_stream_gps_t, minuter) }, \
         { "second", NULL, TTALINK_TYPE_UINT8_T, 0, 70, offsetof(ttalink_stream_gps_t, second) }, \
         { "unix_timestamp_ms", NULL, TTALINK_TYPE_UINT64_T, 0, 71, offsetof(ttalink_stream_gps_t, unix_timestamp_ms) }, \
         { "gps_param", NULL, TTALINK_TYPE_FLOAT, 4, 79, offsetof(ttalink_stream_gps_t, gps_param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_STREAM_GPS { \
    "STREAM_GPS", \
    27, \
    {  { "stream_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_stream_gps_t, stream_type) }, \
         { "gps_lat", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_stream_gps_t, gps_lat) }, \
         { "gps_lon", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_stream_gps_t, gps_lon) }, \
         { "gps_alt", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_stream_gps_t, gps_alt) }, \
         { "gps_veln", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_stream_gps_t, gps_veln) }, \
         { "gps_vele", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_stream_gps_t, gps_vele) }, \
         { "gps_veld", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_stream_gps_t, gps_veld) }, \
         { "gps_velu", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_stream_gps_t, gps_velu) }, \
         { "gps_speed", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_stream_gps_t, gps_speed) }, \
         { "nstart", NULL, TTALINK_TYPE_UINT8_T, 0, 33, offsetof(ttalink_stream_gps_t, nstart) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_stream_gps_t, yaw) }, \
         { "p_dop", NULL, TTALINK_TYPE_FLOAT, 0, 38, offsetof(ttalink_stream_gps_t, p_dop) }, \
         { "v_dop", NULL, TTALINK_TYPE_FLOAT, 0, 42, offsetof(ttalink_stream_gps_t, v_dop) }, \
         { "h_dop", NULL, TTALINK_TYPE_FLOAT, 0, 46, offsetof(ttalink_stream_gps_t, h_dop) }, \
         { "course_acc", NULL, TTALINK_TYPE_FLOAT, 0, 50, offsetof(ttalink_stream_gps_t, course_acc) }, \
         { "speed_acc", NULL, TTALINK_TYPE_FLOAT, 0, 54, offsetof(ttalink_stream_gps_t, speed_acc) }, \
         { "rtk_root", NULL, TTALINK_TYPE_FLOAT, 0, 58, offsetof(ttalink_stream_gps_t, rtk_root) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 62, offsetof(ttalink_stream_gps_t, pos_type) }, \
         { "gps_state", NULL, TTALINK_TYPE_UINT8_T, 0, 63, offsetof(ttalink_stream_gps_t, gps_state) }, \
         { "year", NULL, TTALINK_TYPE_UINT16_T, 0, 64, offsetof(ttalink_stream_gps_t, year) }, \
         { "month", NULL, TTALINK_TYPE_UINT8_T, 0, 66, offsetof(ttalink_stream_gps_t, month) }, \
         { "date", NULL, TTALINK_TYPE_UINT8_T, 0, 67, offsetof(ttalink_stream_gps_t, date) }, \
         { "hour", NULL, TTALINK_TYPE_UINT8_T, 0, 68, offsetof(ttalink_stream_gps_t, hour) }, \
         { "minuter", NULL, TTALINK_TYPE_UINT8_T, 0, 69, offsetof(ttalink_stream_gps_t, minuter) }, \
         { "second", NULL, TTALINK_TYPE_UINT8_T, 0, 70, offsetof(ttalink_stream_gps_t, second) }, \
         { "unix_timestamp_ms", NULL, TTALINK_TYPE_UINT64_T, 0, 71, offsetof(ttalink_stream_gps_t, unix_timestamp_ms) }, \
         { "gps_param", NULL, TTALINK_TYPE_FLOAT, 4, 79, offsetof(ttalink_stream_gps_t, gps_param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_stream_gps_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_GPS_LEN];
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, gps_alt);
    _tta_put_float(buf, 13, gps_veln);
    _tta_put_float(buf, 17, gps_vele);
    _tta_put_float(buf, 21, gps_veld);
    _tta_put_float(buf, 25, gps_velu);
    _tta_put_float(buf, 29, gps_speed);
    _tta_put_uint8_t(buf, 33, nstart);
    _tta_put_float(buf, 34, yaw);
    _tta_put_float(buf, 38, p_dop);
    _tta_put_float(buf, 42, v_dop);
    _tta_put_float(buf, 46, h_dop);
    _tta_put_float(buf, 50, course_acc);
    _tta_put_float(buf, 54, speed_acc);
    _tta_put_float(buf, 58, rtk_root);
    _tta_put_uint8_t(buf, 62, pos_type);
    _tta_put_uint8_t(buf, 63, gps_state);
    _tta_put_uint16_t(buf, 64, year);
    _tta_put_uint8_t(buf, 66, month);
    _tta_put_uint8_t(buf, 67, date);
    _tta_put_uint8_t(buf, 68, hour);
    _tta_put_uint8_t(buf, 69, minuter);
    _tta_put_uint8_t(buf, 70, second);
    _tta_put_uint64_t(buf, 71, unix_timestamp_ms);
    _tta_put_float_array(buf, 79, gps_param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_GPS_LEN);
#else
    ttalink_stream_gps_t packet;
    packet.stream_type = stream_type;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.gps_veln = gps_veln;
    packet.gps_vele = gps_vele;
    packet.gps_veld = gps_veld;
    packet.gps_velu = gps_velu;
    packet.gps_speed = gps_speed;
    packet.nstart = nstart;
    packet.yaw = yaw;
    packet.p_dop = p_dop;
    packet.v_dop = v_dop;
    packet.h_dop = h_dop;
    packet.course_acc = course_acc;
    packet.speed_acc = speed_acc;
    packet.rtk_root = rtk_root;
    packet.pos_type = pos_type;
    packet.gps_state = gps_state;
    packet.year = year;
    packet.month = month;
    packet.date = date;
    packet.hour = hour;
    packet.minuter = minuter;
    packet.second = second;
    packet.unix_timestamp_ms = unix_timestamp_ms;
    tta_array_memcpy(packet.gps_param, gps_param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_GPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_GPS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_STREAM_GPS_MIN_LEN, TTALINK_MSG_ID_STREAM_GPS_LEN, TTALINK_MSG_ID_STREAM_GPS_CRC, nocrc);
}

/**
 * @brief Pack a stream_gps message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param stream_type  .
 * @param gps_lat  .
 * @param gps_lon  .
 * @param gps_alt  .
 * @param gps_veln  .
 * @param gps_vele  .
 * @param gps_veld  .
 * @param gps_velu  .
 * @param gps_speed  .
 * @param nstart  .
 * @param yaw  .
 * @param p_dop  合精度.
 * @param v_dop  垂向精度.
 * @param h_dop  水平精度.
 * @param course_acc  航向精度.
 * @param speed_acc  速度精度.
 * @param rtk_root  .
 * @param pos_type  .
 * @param gps_state  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @param unix_timestamp_ms  .
 * @param gps_param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_gps_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param)
{
    return _ttalink_stream_gps_pack(dst_addr, src_addr, msg,  stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, false);
}

/**
 * @brief Pack a stream_gps message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param stream_type  .
 * @param gps_lat  .
 * @param gps_lon  .
 * @param gps_alt  .
 * @param gps_veln  .
 * @param gps_vele  .
 * @param gps_veld  .
 * @param gps_velu  .
 * @param gps_speed  .
 * @param nstart  .
 * @param yaw  .
 * @param p_dop  合精度.
 * @param v_dop  垂向精度.
 * @param h_dop  水平精度.
 * @param course_acc  航向精度.
 * @param speed_acc  速度精度.
 * @param rtk_root  .
 * @param pos_type  .
 * @param gps_state  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @param unix_timestamp_ms  .
 * @param gps_param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_gps_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param)
{
    return _ttalink_stream_gps_pack(dst_addr, src_addr, msg,  stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, true);
}


static inline uint16_t _ttalink_stream_gps_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stream_type,int32_t gps_lat,int32_t gps_lon,float gps_alt,float gps_veln,float gps_vele,float gps_veld,float gps_velu,float gps_speed,uint8_t nstart,float yaw,float p_dop,float v_dop,float h_dop,float course_acc,float speed_acc,float rtk_root,uint8_t pos_type,uint8_t gps_state,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second,uint64_t unix_timestamp_ms,const float *gps_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_GPS_LEN];
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, gps_alt);
    _tta_put_float(buf, 13, gps_veln);
    _tta_put_float(buf, 17, gps_vele);
    _tta_put_float(buf, 21, gps_veld);
    _tta_put_float(buf, 25, gps_velu);
    _tta_put_float(buf, 29, gps_speed);
    _tta_put_uint8_t(buf, 33, nstart);
    _tta_put_float(buf, 34, yaw);
    _tta_put_float(buf, 38, p_dop);
    _tta_put_float(buf, 42, v_dop);
    _tta_put_float(buf, 46, h_dop);
    _tta_put_float(buf, 50, course_acc);
    _tta_put_float(buf, 54, speed_acc);
    _tta_put_float(buf, 58, rtk_root);
    _tta_put_uint8_t(buf, 62, pos_type);
    _tta_put_uint8_t(buf, 63, gps_state);
    _tta_put_uint16_t(buf, 64, year);
    _tta_put_uint8_t(buf, 66, month);
    _tta_put_uint8_t(buf, 67, date);
    _tta_put_uint8_t(buf, 68, hour);
    _tta_put_uint8_t(buf, 69, minuter);
    _tta_put_uint8_t(buf, 70, second);
    _tta_put_uint64_t(buf, 71, unix_timestamp_ms);
    _tta_put_float_array(buf, 79, gps_param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_GPS_LEN);
#else
    ttalink_stream_gps_t packet;
    packet.stream_type = stream_type;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.gps_veln = gps_veln;
    packet.gps_vele = gps_vele;
    packet.gps_veld = gps_veld;
    packet.gps_velu = gps_velu;
    packet.gps_speed = gps_speed;
    packet.nstart = nstart;
    packet.yaw = yaw;
    packet.p_dop = p_dop;
    packet.v_dop = v_dop;
    packet.h_dop = h_dop;
    packet.course_acc = course_acc;
    packet.speed_acc = speed_acc;
    packet.rtk_root = rtk_root;
    packet.pos_type = pos_type;
    packet.gps_state = gps_state;
    packet.year = year;
    packet.month = month;
    packet.date = date;
    packet.hour = hour;
    packet.minuter = minuter;
    packet.second = second;
    packet.unix_timestamp_ms = unix_timestamp_ms;
    tta_array_memcpy(packet.gps_param, gps_param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_GPS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_GPS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_GPS_MIN_LEN, TTALINK_MSG_ID_STREAM_GPS_LEN, TTALINK_MSG_ID_STREAM_GPS_CRC, nocrc);
}

/**
 * @brief Pack a stream_gps message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_type  .
 * @param gps_lat  .
 * @param gps_lon  .
 * @param gps_alt  .
 * @param gps_veln  .
 * @param gps_vele  .
 * @param gps_veld  .
 * @param gps_velu  .
 * @param gps_speed  .
 * @param nstart  .
 * @param yaw  .
 * @param p_dop  合精度.
 * @param v_dop  垂向精度.
 * @param h_dop  水平精度.
 * @param course_acc  航向精度.
 * @param speed_acc  速度精度.
 * @param rtk_root  .
 * @param pos_type  .
 * @param gps_state  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @param unix_timestamp_ms  .
 * @param gps_param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_gps_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stream_type,int32_t gps_lat,int32_t gps_lon,float gps_alt,float gps_veln,float gps_vele,float gps_veld,float gps_velu,float gps_speed,uint8_t nstart,float yaw,float p_dop,float v_dop,float h_dop,float course_acc,float speed_acc,float rtk_root,uint8_t pos_type,uint8_t gps_state,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second,uint64_t unix_timestamp_ms,const float *gps_param)
{
    return _ttalink_stream_gps_pack_chan(dst_addr, src_addr, chan, msg,  stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, false);
}

/**
 * @brief Pack a stream_gps message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_type  .
 * @param gps_lat  .
 * @param gps_lon  .
 * @param gps_alt  .
 * @param gps_veln  .
 * @param gps_vele  .
 * @param gps_veld  .
 * @param gps_velu  .
 * @param gps_speed  .
 * @param nstart  .
 * @param yaw  .
 * @param p_dop  合精度.
 * @param v_dop  垂向精度.
 * @param h_dop  水平精度.
 * @param course_acc  航向精度.
 * @param speed_acc  速度精度.
 * @param rtk_root  .
 * @param pos_type  .
 * @param gps_state  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @param unix_timestamp_ms  .
 * @param gps_param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_gps_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stream_type,int32_t gps_lat,int32_t gps_lon,float gps_alt,float gps_veln,float gps_vele,float gps_veld,float gps_velu,float gps_speed,uint8_t nstart,float yaw,float p_dop,float v_dop,float h_dop,float course_acc,float speed_acc,float rtk_root,uint8_t pos_type,uint8_t gps_state,uint16_t year,uint8_t month,uint8_t date,uint8_t hour,uint8_t minuter,uint8_t second,uint64_t unix_timestamp_ms,const float *gps_param)
{
    return _ttalink_stream_gps_pack_chan(dst_addr, src_addr, chan, msg,  stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, true);
}


static inline uint16_t _ttalink_stream_gps_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_gps_t* stream_gps, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_gps_pack_nocrc(dst_addr, src_addr, msg, stream_gps->stream_type, stream_gps->gps_lat, stream_gps->gps_lon, stream_gps->gps_alt, stream_gps->gps_veln, stream_gps->gps_vele, stream_gps->gps_veld, stream_gps->gps_velu, stream_gps->gps_speed, stream_gps->nstart, stream_gps->yaw, stream_gps->p_dop, stream_gps->v_dop, stream_gps->h_dop, stream_gps->course_acc, stream_gps->speed_acc, stream_gps->rtk_root, stream_gps->pos_type, stream_gps->gps_state, stream_gps->year, stream_gps->month, stream_gps->date, stream_gps->hour, stream_gps->minuter, stream_gps->second, stream_gps->unix_timestamp_ms, stream_gps->gps_param);
    }else{
        return ttalink_stream_gps_pack(dst_addr, src_addr, msg, stream_gps->stream_type, stream_gps->gps_lat, stream_gps->gps_lon, stream_gps->gps_alt, stream_gps->gps_veln, stream_gps->gps_vele, stream_gps->gps_veld, stream_gps->gps_velu, stream_gps->gps_speed, stream_gps->nstart, stream_gps->yaw, stream_gps->p_dop, stream_gps->v_dop, stream_gps->h_dop, stream_gps->course_acc, stream_gps->speed_acc, stream_gps->rtk_root, stream_gps->pos_type, stream_gps->gps_state, stream_gps->year, stream_gps->month, stream_gps->date, stream_gps->hour, stream_gps->minuter, stream_gps->second, stream_gps->unix_timestamp_ms, stream_gps->gps_param);
    }
    
}

/**
 * @brief Encode a stream_gps struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_gps_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_gps_t* stream_gps)
{
    return _ttalink_stream_gps_encode(dst_addr, src_addr, msg, stream_gps, false);
}

/**
 * @brief Encode a stream_gps struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_gps_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_gps_t* stream_gps)
{
    return _ttalink_stream_gps_encode(dst_addr, src_addr, msg, stream_gps, true);
}


static inline uint16_t _ttalink_stream_gps_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_gps_t* stream_gps, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_gps_pack_chan_nocrc(dst_addr, src_addr, chan, msg, stream_gps->stream_type, stream_gps->gps_lat, stream_gps->gps_lon, stream_gps->gps_alt, stream_gps->gps_veln, stream_gps->gps_vele, stream_gps->gps_veld, stream_gps->gps_velu, stream_gps->gps_speed, stream_gps->nstart, stream_gps->yaw, stream_gps->p_dop, stream_gps->v_dop, stream_gps->h_dop, stream_gps->course_acc, stream_gps->speed_acc, stream_gps->rtk_root, stream_gps->pos_type, stream_gps->gps_state, stream_gps->year, stream_gps->month, stream_gps->date, stream_gps->hour, stream_gps->minuter, stream_gps->second, stream_gps->unix_timestamp_ms, stream_gps->gps_param);
    }else{
        return ttalink_stream_gps_pack_chan(dst_addr, src_addr, chan, msg, stream_gps->stream_type, stream_gps->gps_lat, stream_gps->gps_lon, stream_gps->gps_alt, stream_gps->gps_veln, stream_gps->gps_vele, stream_gps->gps_veld, stream_gps->gps_velu, stream_gps->gps_speed, stream_gps->nstart, stream_gps->yaw, stream_gps->p_dop, stream_gps->v_dop, stream_gps->h_dop, stream_gps->course_acc, stream_gps->speed_acc, stream_gps->rtk_root, stream_gps->pos_type, stream_gps->gps_state, stream_gps->year, stream_gps->month, stream_gps->date, stream_gps->hour, stream_gps->minuter, stream_gps->second, stream_gps->unix_timestamp_ms, stream_gps->gps_param);
    }
}

/**
 * @brief Encode a stream_gps struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_gps_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_gps_t* stream_gps)
{
    return _ttalink_stream_gps_encode_chan(dst_addr, src_addr, chan, msg, stream_gps, false);
}

/**
 * @brief Encode a stream_gps struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_gps C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_gps_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_gps_t* stream_gps)
{
    return _ttalink_stream_gps_encode_chan(dst_addr, src_addr, chan, msg, stream_gps, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_stream_gps_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_GPS_LEN];
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, gps_alt);
    _tta_put_float(buf, 13, gps_veln);
    _tta_put_float(buf, 17, gps_vele);
    _tta_put_float(buf, 21, gps_veld);
    _tta_put_float(buf, 25, gps_velu);
    _tta_put_float(buf, 29, gps_speed);
    _tta_put_uint8_t(buf, 33, nstart);
    _tta_put_float(buf, 34, yaw);
    _tta_put_float(buf, 38, p_dop);
    _tta_put_float(buf, 42, v_dop);
    _tta_put_float(buf, 46, h_dop);
    _tta_put_float(buf, 50, course_acc);
    _tta_put_float(buf, 54, speed_acc);
    _tta_put_float(buf, 58, rtk_root);
    _tta_put_uint8_t(buf, 62, pos_type);
    _tta_put_uint8_t(buf, 63, gps_state);
    _tta_put_uint16_t(buf, 64, year);
    _tta_put_uint8_t(buf, 66, month);
    _tta_put_uint8_t(buf, 67, date);
    _tta_put_uint8_t(buf, 68, hour);
    _tta_put_uint8_t(buf, 69, minuter);
    _tta_put_uint8_t(buf, 70, second);
    _tta_put_uint64_t(buf, 71, unix_timestamp_ms);
    _tta_put_float_array(buf, 79, gps_param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_GPS, buf, TTALINK_MSG_ID_STREAM_GPS_MIN_LEN, TTALINK_MSG_ID_STREAM_GPS_LEN, TTALINK_MSG_ID_STREAM_GPS_CRC, nocrc);
#else
    ttalink_stream_gps_t packet;
    packet.stream_type = stream_type;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.gps_veln = gps_veln;
    packet.gps_vele = gps_vele;
    packet.gps_veld = gps_veld;
    packet.gps_velu = gps_velu;
    packet.gps_speed = gps_speed;
    packet.nstart = nstart;
    packet.yaw = yaw;
    packet.p_dop = p_dop;
    packet.v_dop = v_dop;
    packet.h_dop = h_dop;
    packet.course_acc = course_acc;
    packet.speed_acc = speed_acc;
    packet.rtk_root = rtk_root;
    packet.pos_type = pos_type;
    packet.gps_state = gps_state;
    packet.year = year;
    packet.month = month;
    packet.date = date;
    packet.hour = hour;
    packet.minuter = minuter;
    packet.second = second;
    packet.unix_timestamp_ms = unix_timestamp_ms;
    tta_array_memcpy(packet.gps_param, gps_param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_GPS, (const char *)&packet, TTALINK_MSG_ID_STREAM_GPS_MIN_LEN, TTALINK_MSG_ID_STREAM_GPS_LEN, TTALINK_MSG_ID_STREAM_GPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_gps message
 * @param chan TTAlink channel to send the message
 *
 * @param stream_type  .
 * @param gps_lat  .
 * @param gps_lon  .
 * @param gps_alt  .
 * @param gps_veln  .
 * @param gps_vele  .
 * @param gps_veld  .
 * @param gps_velu  .
 * @param gps_speed  .
 * @param nstart  .
 * @param yaw  .
 * @param p_dop  合精度.
 * @param v_dop  垂向精度.
 * @param h_dop  水平精度.
 * @param course_acc  航向精度.
 * @param speed_acc  速度精度.
 * @param rtk_root  .
 * @param pos_type  .
 * @param gps_state  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @param unix_timestamp_ms  .
 * @param gps_param  .
 */
static inline void ttalink_stream_gps_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param)
{
    _ttalink_stream_gps_send(dst_addr, src_addr, chan, stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, false);
}

/**
 * @brief Send a stream_gps message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param stream_type  .
 * @param gps_lat  .
 * @param gps_lon  .
 * @param gps_alt  .
 * @param gps_veln  .
 * @param gps_vele  .
 * @param gps_veld  .
 * @param gps_velu  .
 * @param gps_speed  .
 * @param nstart  .
 * @param yaw  .
 * @param p_dop  合精度.
 * @param v_dop  垂向精度.
 * @param h_dop  水平精度.
 * @param course_acc  航向精度.
 * @param speed_acc  速度精度.
 * @param rtk_root  .
 * @param pos_type  .
 * @param gps_state  .
 * @param year  .
 * @param month  .
 * @param date  .
 * @param hour  .
 * @param minuter  .
 * @param second  .
 * @param unix_timestamp_ms  .
 * @param gps_param  .
 */
static inline void ttalink_stream_gps_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param)
{
    _ttalink_stream_gps_send(dst_addr, src_addr, chan, stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, true);
}


static inline void _ttalink_stream_gps_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_gps_t* stream_gps, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_stream_gps_send_nocrc(dst_addr, src_addr, chan, stream_gps->stream_type, stream_gps->gps_lat, stream_gps->gps_lon, stream_gps->gps_alt, stream_gps->gps_veln, stream_gps->gps_vele, stream_gps->gps_veld, stream_gps->gps_velu, stream_gps->gps_speed, stream_gps->nstart, stream_gps->yaw, stream_gps->p_dop, stream_gps->v_dop, stream_gps->h_dop, stream_gps->course_acc, stream_gps->speed_acc, stream_gps->rtk_root, stream_gps->pos_type, stream_gps->gps_state, stream_gps->year, stream_gps->month, stream_gps->date, stream_gps->hour, stream_gps->minuter, stream_gps->second, stream_gps->unix_timestamp_ms, stream_gps->gps_param);
    }else{
        ttalink_stream_gps_send(dst_addr, src_addr, chan, stream_gps->stream_type, stream_gps->gps_lat, stream_gps->gps_lon, stream_gps->gps_alt, stream_gps->gps_veln, stream_gps->gps_vele, stream_gps->gps_veld, stream_gps->gps_velu, stream_gps->gps_speed, stream_gps->nstart, stream_gps->yaw, stream_gps->p_dop, stream_gps->v_dop, stream_gps->h_dop, stream_gps->course_acc, stream_gps->speed_acc, stream_gps->rtk_root, stream_gps->pos_type, stream_gps->gps_state, stream_gps->year, stream_gps->month, stream_gps->date, stream_gps->hour, stream_gps->minuter, stream_gps->second, stream_gps->unix_timestamp_ms, stream_gps->gps_param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_GPS, (const char *)stream_gps, TTALINK_MSG_ID_STREAM_GPS_MIN_LEN, TTALINK_MSG_ID_STREAM_GPS_LEN, TTALINK_MSG_ID_STREAM_GPS_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_gps message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_gps_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_gps_t* stream_gps)
{
    _ttalink_stream_gps_send_struct(dst_addr, src_addr, chan, stream_gps, false);
}

/**
 * @brief Send a stream_gps message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_gps_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_gps_t* stream_gps)
{
    _ttalink_stream_gps_send_struct(dst_addr, src_addr, chan, stream_gps, true);
}

#if TTALINK_MSG_ID_STREAM_GPS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_stream_gps_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_int32_t(buf, 1, gps_lat);
    _tta_put_int32_t(buf, 5, gps_lon);
    _tta_put_float(buf, 9, gps_alt);
    _tta_put_float(buf, 13, gps_veln);
    _tta_put_float(buf, 17, gps_vele);
    _tta_put_float(buf, 21, gps_veld);
    _tta_put_float(buf, 25, gps_velu);
    _tta_put_float(buf, 29, gps_speed);
    _tta_put_uint8_t(buf, 33, nstart);
    _tta_put_float(buf, 34, yaw);
    _tta_put_float(buf, 38, p_dop);
    _tta_put_float(buf, 42, v_dop);
    _tta_put_float(buf, 46, h_dop);
    _tta_put_float(buf, 50, course_acc);
    _tta_put_float(buf, 54, speed_acc);
    _tta_put_float(buf, 58, rtk_root);
    _tta_put_uint8_t(buf, 62, pos_type);
    _tta_put_uint8_t(buf, 63, gps_state);
    _tta_put_uint16_t(buf, 64, year);
    _tta_put_uint8_t(buf, 66, month);
    _tta_put_uint8_t(buf, 67, date);
    _tta_put_uint8_t(buf, 68, hour);
    _tta_put_uint8_t(buf, 69, minuter);
    _tta_put_uint8_t(buf, 70, second);
    _tta_put_uint64_t(buf, 71, unix_timestamp_ms);
    _tta_put_float_array(buf, 79, gps_param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_GPS, buf, TTALINK_MSG_ID_STREAM_GPS_MIN_LEN, TTALINK_MSG_ID_STREAM_GPS_LEN, TTALINK_MSG_ID_STREAM_GPS_CRC, nocrc);
#else
    ttalink_stream_gps_t *packet = (ttalink_stream_gps_t *)msgbuf;
    packet->stream_type = stream_type;
    packet->gps_lat = gps_lat;
    packet->gps_lon = gps_lon;
    packet->gps_alt = gps_alt;
    packet->gps_veln = gps_veln;
    packet->gps_vele = gps_vele;
    packet->gps_veld = gps_veld;
    packet->gps_velu = gps_velu;
    packet->gps_speed = gps_speed;
    packet->nstart = nstart;
    packet->yaw = yaw;
    packet->p_dop = p_dop;
    packet->v_dop = v_dop;
    packet->h_dop = h_dop;
    packet->course_acc = course_acc;
    packet->speed_acc = speed_acc;
    packet->rtk_root = rtk_root;
    packet->pos_type = pos_type;
    packet->gps_state = gps_state;
    packet->year = year;
    packet->month = month;
    packet->date = date;
    packet->hour = hour;
    packet->minuter = minuter;
    packet->second = second;
    packet->unix_timestamp_ms = unix_timestamp_ms;
    tta_array_memcpy(packet->gps_param, gps_param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_GPS, (const char *)packet, TTALINK_MSG_ID_STREAM_GPS_MIN_LEN, TTALINK_MSG_ID_STREAM_GPS_LEN, TTALINK_MSG_ID_STREAM_GPS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_stream_gps_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param)
{
    _ttalink_stream_gps_send_buf(dst_addr, src_addr, msgbuf, chan, stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_stream_gps_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stream_type, int32_t gps_lat, int32_t gps_lon, float gps_alt, float gps_veln, float gps_vele, float gps_veld, float gps_velu, float gps_speed, uint8_t nstart, float yaw, float p_dop, float v_dop, float h_dop, float course_acc, float speed_acc, float rtk_root, uint8_t pos_type, uint8_t gps_state, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minuter, uint8_t second, uint64_t unix_timestamp_ms, const float *gps_param)
{
    _ttalink_stream_gps_send_buf(dst_addr, src_addr, msgbuf, chan, stream_type, gps_lat, gps_lon, gps_alt, gps_veln, gps_vele, gps_veld, gps_velu, gps_speed, nstart, yaw, p_dop, v_dop, h_dop, course_acc, speed_acc, rtk_root, pos_type, gps_state, year, month, date, hour, minuter, second, unix_timestamp_ms, gps_param, true);
}
#endif

#endif

// MESSAGE STREAM_GPS UNPACKING


/**
 * @brief Get field stream_type from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_stream_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field gps_lat from stream_gps message
 *
 * @return  .
 */
static inline int32_t ttalink_stream_gps_get_gps_lat(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  1);
}

/**
 * @brief Get field gps_lon from stream_gps message
 *
 * @return  .
 */
static inline int32_t ttalink_stream_gps_get_gps_lon(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  5);
}

/**
 * @brief Get field gps_alt from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_gps_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field gps_veln from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_gps_veln(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field gps_vele from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_gps_vele(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Get field gps_veld from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_gps_veld(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field gps_velu from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_gps_velu(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field gps_speed from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_gps_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Get field nstart from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_nstart(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field yaw from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  34);
}

/**
 * @brief Get field p_dop from stream_gps message
 *
 * @return  合精度.
 */
static inline float ttalink_stream_gps_get_p_dop(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  38);
}

/**
 * @brief Get field v_dop from stream_gps message
 *
 * @return  垂向精度.
 */
static inline float ttalink_stream_gps_get_v_dop(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  42);
}

/**
 * @brief Get field h_dop from stream_gps message
 *
 * @return  水平精度.
 */
static inline float ttalink_stream_gps_get_h_dop(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  46);
}

/**
 * @brief Get field course_acc from stream_gps message
 *
 * @return  航向精度.
 */
static inline float ttalink_stream_gps_get_course_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  50);
}

/**
 * @brief Get field speed_acc from stream_gps message
 *
 * @return  速度精度.
 */
static inline float ttalink_stream_gps_get_speed_acc(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  54);
}

/**
 * @brief Get field rtk_root from stream_gps message
 *
 * @return  .
 */
static inline float ttalink_stream_gps_get_rtk_root(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  58);
}

/**
 * @brief Get field pos_type from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Get field gps_state from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_gps_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  63);
}

/**
 * @brief Get field year from stream_gps message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_gps_get_year(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  64);
}

/**
 * @brief Get field month from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_month(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field date from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_date(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  67);
}

/**
 * @brief Get field hour from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_hour(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Get field minuter from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_minuter(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  69);
}

/**
 * @brief Get field second from stream_gps message
 *
 * @return  .
 */
static inline uint8_t ttalink_stream_gps_get_second(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field unix_timestamp_ms from stream_gps message
 *
 * @return  .
 */
static inline uint64_t ttalink_stream_gps_get_unix_timestamp_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  71);
}

/**
 * @brief Get field gps_param from stream_gps message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_gps_get_gps_param(const ttalink_message_t* msg, float *gps_param)
{
    return _TTA_RETURN_float_array(msg, gps_param, 4,  79);
}

/**
 * @brief Decode a stream_gps message into a struct
 *
 * @param msg The message to decode
 * @param stream_gps C-struct to decode the message contents into
 */
static inline void ttalink_stream_gps_decode(const ttalink_message_t* msg, ttalink_stream_gps_t* stream_gps)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    stream_gps->stream_type = ttalink_stream_gps_get_stream_type(msg);
    stream_gps->gps_lat = ttalink_stream_gps_get_gps_lat(msg);
    stream_gps->gps_lon = ttalink_stream_gps_get_gps_lon(msg);
    stream_gps->gps_alt = ttalink_stream_gps_get_gps_alt(msg);
    stream_gps->gps_veln = ttalink_stream_gps_get_gps_veln(msg);
    stream_gps->gps_vele = ttalink_stream_gps_get_gps_vele(msg);
    stream_gps->gps_veld = ttalink_stream_gps_get_gps_veld(msg);
    stream_gps->gps_velu = ttalink_stream_gps_get_gps_velu(msg);
    stream_gps->gps_speed = ttalink_stream_gps_get_gps_speed(msg);
    stream_gps->nstart = ttalink_stream_gps_get_nstart(msg);
    stream_gps->yaw = ttalink_stream_gps_get_yaw(msg);
    stream_gps->p_dop = ttalink_stream_gps_get_p_dop(msg);
    stream_gps->v_dop = ttalink_stream_gps_get_v_dop(msg);
    stream_gps->h_dop = ttalink_stream_gps_get_h_dop(msg);
    stream_gps->course_acc = ttalink_stream_gps_get_course_acc(msg);
    stream_gps->speed_acc = ttalink_stream_gps_get_speed_acc(msg);
    stream_gps->rtk_root = ttalink_stream_gps_get_rtk_root(msg);
    stream_gps->pos_type = ttalink_stream_gps_get_pos_type(msg);
    stream_gps->gps_state = ttalink_stream_gps_get_gps_state(msg);
    stream_gps->year = ttalink_stream_gps_get_year(msg);
    stream_gps->month = ttalink_stream_gps_get_month(msg);
    stream_gps->date = ttalink_stream_gps_get_date(msg);
    stream_gps->hour = ttalink_stream_gps_get_hour(msg);
    stream_gps->minuter = ttalink_stream_gps_get_minuter(msg);
    stream_gps->second = ttalink_stream_gps_get_second(msg);
    stream_gps->unix_timestamp_ms = ttalink_stream_gps_get_unix_timestamp_ms(msg);
    ttalink_stream_gps_get_gps_param(msg, stream_gps->gps_param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_STREAM_GPS_LEN? msg->len : TTALINK_MSG_ID_STREAM_GPS_LEN;
        memset(stream_gps, 0, TTALINK_MSG_ID_STREAM_GPS_LEN);
    memcpy(stream_gps, _TTA_PAYLOAD(msg), len);
#endif
}
