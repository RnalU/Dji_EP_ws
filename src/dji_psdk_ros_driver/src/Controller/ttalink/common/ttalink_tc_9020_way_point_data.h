#pragma once
// MESSAGE TC_9020_WAY_POINT_DATA PACKING

#define TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA 9020

TTAPACKED(
typedef struct __ttalink_tc_9020_way_point_data_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t effect_num; /*<  有效点数*/
 int32_t wp_latit[6]; /*<  .*/
 int32_t wp_longi[6]; /*<  .*/
 float wp_altit[6]; /*<  .*/
}) ttalink_tc_9020_way_point_data_t;

#define TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN 77
#define TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN 77
#define TTALINK_MSG_ID_9020_LEN 77
#define TTALINK_MSG_ID_9020_MIN_LEN 77

#define TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC 69
#define TTALINK_MSG_ID_9020_CRC 69

#define TTALINK_MSG_TC_9020_WAY_POINT_DATA_FIELD_WP_LATIT_LEN 6
#define TTALINK_MSG_TC_9020_WAY_POINT_DATA_FIELD_WP_LONGI_LEN 6
#define TTALINK_MSG_TC_9020_WAY_POINT_DATA_FIELD_WP_ALTIT_LEN 6

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9020_WAY_POINT_DATA { \
    9020, \
    "TC_9020_WAY_POINT_DATA", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9020_way_point_data_t, update_time) }, \
         { "effect_num", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9020_way_point_data_t, effect_num) }, \
         { "wp_latit", NULL, TTALINK_TYPE_INT32_T, 6, 5, offsetof(ttalink_tc_9020_way_point_data_t, wp_latit) }, \
         { "wp_longi", NULL, TTALINK_TYPE_INT32_T, 6, 29, offsetof(ttalink_tc_9020_way_point_data_t, wp_longi) }, \
         { "wp_altit", NULL, TTALINK_TYPE_FLOAT, 6, 53, offsetof(ttalink_tc_9020_way_point_data_t, wp_altit) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9020_WAY_POINT_DATA { \
    "TC_9020_WAY_POINT_DATA", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9020_way_point_data_t, update_time) }, \
         { "effect_num", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_tc_9020_way_point_data_t, effect_num) }, \
         { "wp_latit", NULL, TTALINK_TYPE_INT32_T, 6, 5, offsetof(ttalink_tc_9020_way_point_data_t, wp_latit) }, \
         { "wp_longi", NULL, TTALINK_TYPE_INT32_T, 6, 29, offsetof(ttalink_tc_9020_way_point_data_t, wp_longi) }, \
         { "wp_altit", NULL, TTALINK_TYPE_FLOAT, 6, 53, offsetof(ttalink_tc_9020_way_point_data_t, wp_altit) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9020_way_point_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, effect_num);
    _tta_put_int32_t_array(buf, 5, wp_latit, 6);
    _tta_put_int32_t_array(buf, 29, wp_longi, 6);
    _tta_put_float_array(buf, 53, wp_altit, 6);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN);
#else
    ttalink_tc_9020_way_point_data_t packet;
    packet.update_time = update_time;
    packet.effect_num = effect_num;
    tta_array_memcpy(packet.wp_latit, wp_latit, sizeof(int32_t)*6);
    tta_array_memcpy(packet.wp_longi, wp_longi, sizeof(int32_t)*6);
    tta_array_memcpy(packet.wp_altit, wp_altit, sizeof(float)*6);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9020_way_point_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param effect_num  有效点数
 * @param wp_latit  .
 * @param wp_longi  .
 * @param wp_altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9020_way_point_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit)
{
    return _ttalink_tc_9020_way_point_data_pack(dst_addr, src_addr, msg,  update_time, effect_num, wp_latit, wp_longi, wp_altit, false);
}

/**
 * @brief Pack a tc_9020_way_point_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param effect_num  有效点数
 * @param wp_latit  .
 * @param wp_longi  .
 * @param wp_altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9020_way_point_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit)
{
    return _ttalink_tc_9020_way_point_data_pack(dst_addr, src_addr, msg,  update_time, effect_num, wp_latit, wp_longi, wp_altit, true);
}


static inline uint16_t _ttalink_tc_9020_way_point_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t effect_num,const int32_t *wp_latit,const int32_t *wp_longi,const float *wp_altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, effect_num);
    _tta_put_int32_t_array(buf, 5, wp_latit, 6);
    _tta_put_int32_t_array(buf, 29, wp_longi, 6);
    _tta_put_float_array(buf, 53, wp_altit, 6);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN);
#else
    ttalink_tc_9020_way_point_data_t packet;
    packet.update_time = update_time;
    packet.effect_num = effect_num;
    tta_array_memcpy(packet.wp_latit, wp_latit, sizeof(int32_t)*6);
    tta_array_memcpy(packet.wp_longi, wp_longi, sizeof(int32_t)*6);
    tta_array_memcpy(packet.wp_altit, wp_altit, sizeof(float)*6);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9020_way_point_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param effect_num  有效点数
 * @param wp_latit  .
 * @param wp_longi  .
 * @param wp_altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9020_way_point_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t effect_num,const int32_t *wp_latit,const int32_t *wp_longi,const float *wp_altit)
{
    return _ttalink_tc_9020_way_point_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, effect_num, wp_latit, wp_longi, wp_altit, false);
}

/**
 * @brief Pack a tc_9020_way_point_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param effect_num  有效点数
 * @param wp_latit  .
 * @param wp_longi  .
 * @param wp_altit  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9020_way_point_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t effect_num,const int32_t *wp_latit,const int32_t *wp_longi,const float *wp_altit)
{
    return _ttalink_tc_9020_way_point_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, effect_num, wp_latit, wp_longi, wp_altit, true);
}


static inline uint16_t _ttalink_tc_9020_way_point_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9020_way_point_data_pack_nocrc(dst_addr, src_addr, msg, tc_9020_way_point_data->update_time, tc_9020_way_point_data->effect_num, tc_9020_way_point_data->wp_latit, tc_9020_way_point_data->wp_longi, tc_9020_way_point_data->wp_altit);
    }else{
        return ttalink_tc_9020_way_point_data_pack(dst_addr, src_addr, msg, tc_9020_way_point_data->update_time, tc_9020_way_point_data->effect_num, tc_9020_way_point_data->wp_latit, tc_9020_way_point_data->wp_longi, tc_9020_way_point_data->wp_altit);
    }
    
}

/**
 * @brief Encode a tc_9020_way_point_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9020_way_point_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9020_way_point_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data)
{
    return _ttalink_tc_9020_way_point_data_encode(dst_addr, src_addr, msg, tc_9020_way_point_data, false);
}

/**
 * @brief Encode a tc_9020_way_point_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9020_way_point_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9020_way_point_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data)
{
    return _ttalink_tc_9020_way_point_data_encode(dst_addr, src_addr, msg, tc_9020_way_point_data, true);
}


static inline uint16_t _ttalink_tc_9020_way_point_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9020_way_point_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9020_way_point_data->update_time, tc_9020_way_point_data->effect_num, tc_9020_way_point_data->wp_latit, tc_9020_way_point_data->wp_longi, tc_9020_way_point_data->wp_altit);
    }else{
        return ttalink_tc_9020_way_point_data_pack_chan(dst_addr, src_addr, chan, msg, tc_9020_way_point_data->update_time, tc_9020_way_point_data->effect_num, tc_9020_way_point_data->wp_latit, tc_9020_way_point_data->wp_longi, tc_9020_way_point_data->wp_altit);
    }
}

/**
 * @brief Encode a tc_9020_way_point_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9020_way_point_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9020_way_point_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data)
{
    return _ttalink_tc_9020_way_point_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9020_way_point_data, false);
}

/**
 * @brief Encode a tc_9020_way_point_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9020_way_point_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9020_way_point_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data)
{
    return _ttalink_tc_9020_way_point_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9020_way_point_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9020_way_point_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, effect_num);
    _tta_put_int32_t_array(buf, 5, wp_latit, 6);
    _tta_put_int32_t_array(buf, 29, wp_longi, 6);
    _tta_put_float_array(buf, 53, wp_altit, 6);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA, buf, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC, nocrc);
#else
    ttalink_tc_9020_way_point_data_t packet;
    packet.update_time = update_time;
    packet.effect_num = effect_num;
    tta_array_memcpy(packet.wp_latit, wp_latit, sizeof(int32_t)*6);
    tta_array_memcpy(packet.wp_longi, wp_longi, sizeof(int32_t)*6);
    tta_array_memcpy(packet.wp_altit, wp_altit, sizeof(float)*6);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA, (const char *)&packet, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9020_way_point_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param effect_num  有效点数
 * @param wp_latit  .
 * @param wp_longi  .
 * @param wp_altit  .
 */
static inline void ttalink_tc_9020_way_point_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit)
{
    _ttalink_tc_9020_way_point_data_send(dst_addr, src_addr, chan, update_time, effect_num, wp_latit, wp_longi, wp_altit, false);
}

/**
 * @brief Send a tc_9020_way_point_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param effect_num  有效点数
 * @param wp_latit  .
 * @param wp_longi  .
 * @param wp_altit  .
 */
static inline void ttalink_tc_9020_way_point_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit)
{
    _ttalink_tc_9020_way_point_data_send(dst_addr, src_addr, chan, update_time, effect_num, wp_latit, wp_longi, wp_altit, true);
}


static inline void _ttalink_tc_9020_way_point_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9020_way_point_data_send_nocrc(dst_addr, src_addr, chan, tc_9020_way_point_data->update_time, tc_9020_way_point_data->effect_num, tc_9020_way_point_data->wp_latit, tc_9020_way_point_data->wp_longi, tc_9020_way_point_data->wp_altit);
    }else{
        ttalink_tc_9020_way_point_data_send(dst_addr, src_addr, chan, tc_9020_way_point_data->update_time, tc_9020_way_point_data->effect_num, tc_9020_way_point_data->wp_latit, tc_9020_way_point_data->wp_longi, tc_9020_way_point_data->wp_altit);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA, (const char *)tc_9020_way_point_data, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9020_way_point_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9020_way_point_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data)
{
    _ttalink_tc_9020_way_point_data_send_struct(dst_addr, src_addr, chan, tc_9020_way_point_data, false);
}

/**
 * @brief Send a tc_9020_way_point_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9020_way_point_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data)
{
    _ttalink_tc_9020_way_point_data_send_struct(dst_addr, src_addr, chan, tc_9020_way_point_data, true);
}

#if TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9020_way_point_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, effect_num);
    _tta_put_int32_t_array(buf, 5, wp_latit, 6);
    _tta_put_int32_t_array(buf, 29, wp_longi, 6);
    _tta_put_float_array(buf, 53, wp_altit, 6);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA, buf, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC, nocrc);
#else
    ttalink_tc_9020_way_point_data_t *packet = (ttalink_tc_9020_way_point_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->effect_num = effect_num;
    tta_array_memcpy(packet->wp_latit, wp_latit, sizeof(int32_t)*6);
    tta_array_memcpy(packet->wp_longi, wp_longi, sizeof(int32_t)*6);
    tta_array_memcpy(packet->wp_altit, wp_altit, sizeof(float)*6);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA, (const char *)packet, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9020_way_point_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit)
{
    _ttalink_tc_9020_way_point_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, effect_num, wp_latit, wp_longi, wp_altit, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9020_way_point_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t effect_num, const int32_t *wp_latit, const int32_t *wp_longi, const float *wp_altit)
{
    _ttalink_tc_9020_way_point_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, effect_num, wp_latit, wp_longi, wp_altit, true);
}
#endif

#endif

// MESSAGE TC_9020_WAY_POINT_DATA UNPACKING


/**
 * @brief Get field update_time from tc_9020_way_point_data message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9020_way_point_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field effect_num from tc_9020_way_point_data message
 *
 * @return  有效点数
 */
static inline uint8_t ttalink_tc_9020_way_point_data_get_effect_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field wp_latit from tc_9020_way_point_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_tc_9020_way_point_data_get_wp_latit(const ttalink_message_t* msg, int32_t *wp_latit)
{
    return _TTA_RETURN_int32_t_array(msg, wp_latit, 6,  5);
}

/**
 * @brief Get field wp_longi from tc_9020_way_point_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_tc_9020_way_point_data_get_wp_longi(const ttalink_message_t* msg, int32_t *wp_longi)
{
    return _TTA_RETURN_int32_t_array(msg, wp_longi, 6,  29);
}

/**
 * @brief Get field wp_altit from tc_9020_way_point_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_tc_9020_way_point_data_get_wp_altit(const ttalink_message_t* msg, float *wp_altit)
{
    return _TTA_RETURN_float_array(msg, wp_altit, 6,  53);
}

/**
 * @brief Decode a tc_9020_way_point_data message into a struct
 *
 * @param msg The message to decode
 * @param tc_9020_way_point_data C-struct to decode the message contents into
 */
static inline void ttalink_tc_9020_way_point_data_decode(const ttalink_message_t* msg, ttalink_tc_9020_way_point_data_t* tc_9020_way_point_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9020_way_point_data->update_time = ttalink_tc_9020_way_point_data_get_update_time(msg);
    tc_9020_way_point_data->effect_num = ttalink_tc_9020_way_point_data_get_effect_num(msg);
    ttalink_tc_9020_way_point_data_get_wp_latit(msg, tc_9020_way_point_data->wp_latit);
    ttalink_tc_9020_way_point_data_get_wp_longi(msg, tc_9020_way_point_data->wp_longi);
    ttalink_tc_9020_way_point_data_get_wp_altit(msg, tc_9020_way_point_data->wp_altit);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN? msg->len : TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN;
        memset(tc_9020_way_point_data, 0, TTALINK_MSG_ID_TC_9020_WAY_POINT_DATA_LEN);
    memcpy(tc_9020_way_point_data, _TTA_PAYLOAD(msg), len);
#endif
}
