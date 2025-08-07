#pragma once
// MESSAGE TC_9030_UTC_TIME_DATA PACKING

#define TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA 9030

TTAPACKED(
typedef struct __ttalink_tc_9030_utc_time_data_t {
 uint32_t update_time; /*<  update_time*/
 uint16_t gps_time_week; /*<  整周.*/
 uint32_t gps_time_ms; /*<  周内秒.*/
 uint32_t utc_year; /*<  UTC年.*/
 uint8_t utc_month; /*<  UTC月.*/
 uint8_t utc_day; /*<  UTC日.*/
 uint8_t utc_hour; /*<  UTC时.*/
 uint8_t utc_min; /*<  UTC分.*/
 uint32_t utc_ms; /*<  UTC毫秒.*/
 uint32_t unix_timestamp; /*<  Unix时间戳.*/
 float param[4]; /*<  .*/
}) ttalink_tc_9030_utc_time_data_t;

#define TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN 42
#define TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN 42
#define TTALINK_MSG_ID_9030_LEN 42
#define TTALINK_MSG_ID_9030_MIN_LEN 42

#define TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC 240
#define TTALINK_MSG_ID_9030_CRC 240

#define TTALINK_MSG_TC_9030_UTC_TIME_DATA_FIELD_PARAM_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9030_UTC_TIME_DATA { \
    9030, \
    "TC_9030_UTC_TIME_DATA", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9030_utc_time_data_t, update_time) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 4, offsetof(ttalink_tc_9030_utc_time_data_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 6, offsetof(ttalink_tc_9030_utc_time_data_t, gps_time_ms) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 10, offsetof(ttalink_tc_9030_utc_time_data_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_tc_9030_utc_time_data_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 15, offsetof(ttalink_tc_9030_utc_time_data_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_tc_9030_utc_time_data_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 17, offsetof(ttalink_tc_9030_utc_time_data_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 18, offsetof(ttalink_tc_9030_utc_time_data_t, utc_ms) }, \
         { "unix_timestamp", NULL, TTALINK_TYPE_UINT32_T, 0, 22, offsetof(ttalink_tc_9030_utc_time_data_t, unix_timestamp) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 26, offsetof(ttalink_tc_9030_utc_time_data_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9030_UTC_TIME_DATA { \
    "TC_9030_UTC_TIME_DATA", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9030_utc_time_data_t, update_time) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 4, offsetof(ttalink_tc_9030_utc_time_data_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 6, offsetof(ttalink_tc_9030_utc_time_data_t, gps_time_ms) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 10, offsetof(ttalink_tc_9030_utc_time_data_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_tc_9030_utc_time_data_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 15, offsetof(ttalink_tc_9030_utc_time_data_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_tc_9030_utc_time_data_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 17, offsetof(ttalink_tc_9030_utc_time_data_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 18, offsetof(ttalink_tc_9030_utc_time_data_t, utc_ms) }, \
         { "unix_timestamp", NULL, TTALINK_TYPE_UINT32_T, 0, 22, offsetof(ttalink_tc_9030_utc_time_data_t, unix_timestamp) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 26, offsetof(ttalink_tc_9030_utc_time_data_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9030_utc_time_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint16_t(buf, 4, gps_time_week);
    _tta_put_uint32_t(buf, 6, gps_time_ms);
    _tta_put_uint32_t(buf, 10, utc_year);
    _tta_put_uint8_t(buf, 14, utc_month);
    _tta_put_uint8_t(buf, 15, utc_day);
    _tta_put_uint8_t(buf, 16, utc_hour);
    _tta_put_uint8_t(buf, 17, utc_min);
    _tta_put_uint32_t(buf, 18, utc_ms);
    _tta_put_uint32_t(buf, 22, unix_timestamp);
    _tta_put_float_array(buf, 26, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN);
#else
    ttalink_tc_9030_utc_time_data_t packet;
    packet.update_time = update_time;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.unix_timestamp = unix_timestamp;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9030_utc_time_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param unix_timestamp  Unix时间戳.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param)
{
    return _ttalink_tc_9030_utc_time_data_pack(dst_addr, src_addr, msg,  update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, false);
}

/**
 * @brief Pack a tc_9030_utc_time_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param unix_timestamp  Unix时间戳.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param)
{
    return _ttalink_tc_9030_utc_time_data_pack(dst_addr, src_addr, msg,  update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, true);
}


static inline uint16_t _ttalink_tc_9030_utc_time_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint16_t gps_time_week,uint32_t gps_time_ms,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t unix_timestamp,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint16_t(buf, 4, gps_time_week);
    _tta_put_uint32_t(buf, 6, gps_time_ms);
    _tta_put_uint32_t(buf, 10, utc_year);
    _tta_put_uint8_t(buf, 14, utc_month);
    _tta_put_uint8_t(buf, 15, utc_day);
    _tta_put_uint8_t(buf, 16, utc_hour);
    _tta_put_uint8_t(buf, 17, utc_min);
    _tta_put_uint32_t(buf, 18, utc_ms);
    _tta_put_uint32_t(buf, 22, unix_timestamp);
    _tta_put_float_array(buf, 26, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN);
#else
    ttalink_tc_9030_utc_time_data_t packet;
    packet.update_time = update_time;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.unix_timestamp = unix_timestamp;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9030_utc_time_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param unix_timestamp  Unix时间戳.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint16_t gps_time_week,uint32_t gps_time_ms,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t unix_timestamp,const float *param)
{
    return _ttalink_tc_9030_utc_time_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, false);
}

/**
 * @brief Pack a tc_9030_utc_time_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param unix_timestamp  Unix时间戳.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint16_t gps_time_week,uint32_t gps_time_ms,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t unix_timestamp,const float *param)
{
    return _ttalink_tc_9030_utc_time_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, true);
}


static inline uint16_t _ttalink_tc_9030_utc_time_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9030_utc_time_data_pack_nocrc(dst_addr, src_addr, msg, tc_9030_utc_time_data->update_time, tc_9030_utc_time_data->gps_time_week, tc_9030_utc_time_data->gps_time_ms, tc_9030_utc_time_data->utc_year, tc_9030_utc_time_data->utc_month, tc_9030_utc_time_data->utc_day, tc_9030_utc_time_data->utc_hour, tc_9030_utc_time_data->utc_min, tc_9030_utc_time_data->utc_ms, tc_9030_utc_time_data->unix_timestamp, tc_9030_utc_time_data->param);
    }else{
        return ttalink_tc_9030_utc_time_data_pack(dst_addr, src_addr, msg, tc_9030_utc_time_data->update_time, tc_9030_utc_time_data->gps_time_week, tc_9030_utc_time_data->gps_time_ms, tc_9030_utc_time_data->utc_year, tc_9030_utc_time_data->utc_month, tc_9030_utc_time_data->utc_day, tc_9030_utc_time_data->utc_hour, tc_9030_utc_time_data->utc_min, tc_9030_utc_time_data->utc_ms, tc_9030_utc_time_data->unix_timestamp, tc_9030_utc_time_data->param);
    }
    
}

/**
 * @brief Encode a tc_9030_utc_time_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9030_utc_time_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data)
{
    return _ttalink_tc_9030_utc_time_data_encode(dst_addr, src_addr, msg, tc_9030_utc_time_data, false);
}

/**
 * @brief Encode a tc_9030_utc_time_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9030_utc_time_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data)
{
    return _ttalink_tc_9030_utc_time_data_encode(dst_addr, src_addr, msg, tc_9030_utc_time_data, true);
}


static inline uint16_t _ttalink_tc_9030_utc_time_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9030_utc_time_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9030_utc_time_data->update_time, tc_9030_utc_time_data->gps_time_week, tc_9030_utc_time_data->gps_time_ms, tc_9030_utc_time_data->utc_year, tc_9030_utc_time_data->utc_month, tc_9030_utc_time_data->utc_day, tc_9030_utc_time_data->utc_hour, tc_9030_utc_time_data->utc_min, tc_9030_utc_time_data->utc_ms, tc_9030_utc_time_data->unix_timestamp, tc_9030_utc_time_data->param);
    }else{
        return ttalink_tc_9030_utc_time_data_pack_chan(dst_addr, src_addr, chan, msg, tc_9030_utc_time_data->update_time, tc_9030_utc_time_data->gps_time_week, tc_9030_utc_time_data->gps_time_ms, tc_9030_utc_time_data->utc_year, tc_9030_utc_time_data->utc_month, tc_9030_utc_time_data->utc_day, tc_9030_utc_time_data->utc_hour, tc_9030_utc_time_data->utc_min, tc_9030_utc_time_data->utc_ms, tc_9030_utc_time_data->unix_timestamp, tc_9030_utc_time_data->param);
    }
}

/**
 * @brief Encode a tc_9030_utc_time_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9030_utc_time_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data)
{
    return _ttalink_tc_9030_utc_time_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9030_utc_time_data, false);
}

/**
 * @brief Encode a tc_9030_utc_time_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9030_utc_time_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data)
{
    return _ttalink_tc_9030_utc_time_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9030_utc_time_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9030_utc_time_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint16_t(buf, 4, gps_time_week);
    _tta_put_uint32_t(buf, 6, gps_time_ms);
    _tta_put_uint32_t(buf, 10, utc_year);
    _tta_put_uint8_t(buf, 14, utc_month);
    _tta_put_uint8_t(buf, 15, utc_day);
    _tta_put_uint8_t(buf, 16, utc_hour);
    _tta_put_uint8_t(buf, 17, utc_min);
    _tta_put_uint32_t(buf, 18, utc_ms);
    _tta_put_uint32_t(buf, 22, unix_timestamp);
    _tta_put_float_array(buf, 26, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA, buf, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC, nocrc);
#else
    ttalink_tc_9030_utc_time_data_t packet;
    packet.update_time = update_time;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.unix_timestamp = unix_timestamp;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA, (const char *)&packet, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9030_utc_time_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param unix_timestamp  Unix时间戳.
 * @param param  .
 */
static inline void ttalink_tc_9030_utc_time_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param)
{
    _ttalink_tc_9030_utc_time_data_send(dst_addr, src_addr, chan, update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, false);
}

/**
 * @brief Send a tc_9030_utc_time_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param unix_timestamp  Unix时间戳.
 * @param param  .
 */
static inline void ttalink_tc_9030_utc_time_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param)
{
    _ttalink_tc_9030_utc_time_data_send(dst_addr, src_addr, chan, update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, true);
}


static inline void _ttalink_tc_9030_utc_time_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9030_utc_time_data_send_nocrc(dst_addr, src_addr, chan, tc_9030_utc_time_data->update_time, tc_9030_utc_time_data->gps_time_week, tc_9030_utc_time_data->gps_time_ms, tc_9030_utc_time_data->utc_year, tc_9030_utc_time_data->utc_month, tc_9030_utc_time_data->utc_day, tc_9030_utc_time_data->utc_hour, tc_9030_utc_time_data->utc_min, tc_9030_utc_time_data->utc_ms, tc_9030_utc_time_data->unix_timestamp, tc_9030_utc_time_data->param);
    }else{
        ttalink_tc_9030_utc_time_data_send(dst_addr, src_addr, chan, tc_9030_utc_time_data->update_time, tc_9030_utc_time_data->gps_time_week, tc_9030_utc_time_data->gps_time_ms, tc_9030_utc_time_data->utc_year, tc_9030_utc_time_data->utc_month, tc_9030_utc_time_data->utc_day, tc_9030_utc_time_data->utc_hour, tc_9030_utc_time_data->utc_min, tc_9030_utc_time_data->utc_ms, tc_9030_utc_time_data->unix_timestamp, tc_9030_utc_time_data->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA, (const char *)tc_9030_utc_time_data, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9030_utc_time_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9030_utc_time_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data)
{
    _ttalink_tc_9030_utc_time_data_send_struct(dst_addr, src_addr, chan, tc_9030_utc_time_data, false);
}

/**
 * @brief Send a tc_9030_utc_time_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9030_utc_time_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data)
{
    _ttalink_tc_9030_utc_time_data_send_struct(dst_addr, src_addr, chan, tc_9030_utc_time_data, true);
}

#if TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9030_utc_time_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint16_t(buf, 4, gps_time_week);
    _tta_put_uint32_t(buf, 6, gps_time_ms);
    _tta_put_uint32_t(buf, 10, utc_year);
    _tta_put_uint8_t(buf, 14, utc_month);
    _tta_put_uint8_t(buf, 15, utc_day);
    _tta_put_uint8_t(buf, 16, utc_hour);
    _tta_put_uint8_t(buf, 17, utc_min);
    _tta_put_uint32_t(buf, 18, utc_ms);
    _tta_put_uint32_t(buf, 22, unix_timestamp);
    _tta_put_float_array(buf, 26, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA, buf, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC, nocrc);
#else
    ttalink_tc_9030_utc_time_data_t *packet = (ttalink_tc_9030_utc_time_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->gps_time_week = gps_time_week;
    packet->gps_time_ms = gps_time_ms;
    packet->utc_year = utc_year;
    packet->utc_month = utc_month;
    packet->utc_day = utc_day;
    packet->utc_hour = utc_hour;
    packet->utc_min = utc_min;
    packet->utc_ms = utc_ms;
    packet->unix_timestamp = unix_timestamp;
    tta_array_memcpy(packet->param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA, (const char *)packet, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9030_utc_time_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param)
{
    _ttalink_tc_9030_utc_time_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9030_utc_time_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t unix_timestamp, const float *param)
{
    _ttalink_tc_9030_utc_time_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, unix_timestamp, param, true);
}
#endif

#endif

// MESSAGE TC_9030_UTC_TIME_DATA UNPACKING


/**
 * @brief Get field update_time from tc_9030_utc_time_data message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9030_utc_time_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field gps_time_week from tc_9030_utc_time_data message
 *
 * @return  整周.
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_get_gps_time_week(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field gps_time_ms from tc_9030_utc_time_data message
 *
 * @return  周内秒.
 */
static inline uint32_t ttalink_tc_9030_utc_time_data_get_gps_time_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  6);
}

/**
 * @brief Get field utc_year from tc_9030_utc_time_data message
 *
 * @return  UTC年.
 */
static inline uint32_t ttalink_tc_9030_utc_time_data_get_utc_year(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  10);
}

/**
 * @brief Get field utc_month from tc_9030_utc_time_data message
 *
 * @return  UTC月.
 */
static inline uint8_t ttalink_tc_9030_utc_time_data_get_utc_month(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field utc_day from tc_9030_utc_time_data message
 *
 * @return  UTC日.
 */
static inline uint8_t ttalink_tc_9030_utc_time_data_get_utc_day(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field utc_hour from tc_9030_utc_time_data message
 *
 * @return  UTC时.
 */
static inline uint8_t ttalink_tc_9030_utc_time_data_get_utc_hour(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field utc_min from tc_9030_utc_time_data message
 *
 * @return  UTC分.
 */
static inline uint8_t ttalink_tc_9030_utc_time_data_get_utc_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field utc_ms from tc_9030_utc_time_data message
 *
 * @return  UTC毫秒.
 */
static inline uint32_t ttalink_tc_9030_utc_time_data_get_utc_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  18);
}

/**
 * @brief Get field unix_timestamp from tc_9030_utc_time_data message
 *
 * @return  Unix时间戳.
 */
static inline uint32_t ttalink_tc_9030_utc_time_data_get_unix_timestamp(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  22);
}

/**
 * @brief Get field param from tc_9030_utc_time_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_tc_9030_utc_time_data_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 4,  26);
}

/**
 * @brief Decode a tc_9030_utc_time_data message into a struct
 *
 * @param msg The message to decode
 * @param tc_9030_utc_time_data C-struct to decode the message contents into
 */
static inline void ttalink_tc_9030_utc_time_data_decode(const ttalink_message_t* msg, ttalink_tc_9030_utc_time_data_t* tc_9030_utc_time_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9030_utc_time_data->update_time = ttalink_tc_9030_utc_time_data_get_update_time(msg);
    tc_9030_utc_time_data->gps_time_week = ttalink_tc_9030_utc_time_data_get_gps_time_week(msg);
    tc_9030_utc_time_data->gps_time_ms = ttalink_tc_9030_utc_time_data_get_gps_time_ms(msg);
    tc_9030_utc_time_data->utc_year = ttalink_tc_9030_utc_time_data_get_utc_year(msg);
    tc_9030_utc_time_data->utc_month = ttalink_tc_9030_utc_time_data_get_utc_month(msg);
    tc_9030_utc_time_data->utc_day = ttalink_tc_9030_utc_time_data_get_utc_day(msg);
    tc_9030_utc_time_data->utc_hour = ttalink_tc_9030_utc_time_data_get_utc_hour(msg);
    tc_9030_utc_time_data->utc_min = ttalink_tc_9030_utc_time_data_get_utc_min(msg);
    tc_9030_utc_time_data->utc_ms = ttalink_tc_9030_utc_time_data_get_utc_ms(msg);
    tc_9030_utc_time_data->unix_timestamp = ttalink_tc_9030_utc_time_data_get_unix_timestamp(msg);
    ttalink_tc_9030_utc_time_data_get_param(msg, tc_9030_utc_time_data->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN? msg->len : TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN;
        memset(tc_9030_utc_time_data, 0, TTALINK_MSG_ID_TC_9030_UTC_TIME_DATA_LEN);
    memcpy(tc_9030_utc_time_data, _TTA_PAYLOAD(msg), len);
#endif
}
