#pragma once
// MESSAGE TC_9022_MASS_SENSOR_DATA PACKING

#define TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA 9022

TTAPACKED(
typedef struct __ttalink_tc_9022_mass_sensor_data_t {
 uint32_t update_time; /*<  update_time*/
 float weigh_left; /*<  左重量.*/
 float weigh_right; /*<  右重量.*/
 float param[2]; /*<  .*/
}) ttalink_tc_9022_mass_sensor_data_t;

#define TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN 20
#define TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN 20
#define TTALINK_MSG_ID_9022_LEN 20
#define TTALINK_MSG_ID_9022_MIN_LEN 20

#define TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC 55
#define TTALINK_MSG_ID_9022_CRC 55

#define TTALINK_MSG_TC_9022_MASS_SENSOR_DATA_FIELD_PARAM_LEN 2

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9022_MASS_SENSOR_DATA { \
    9022, \
    "TC_9022_MASS_SENSOR_DATA", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9022_mass_sensor_data_t, update_time) }, \
         { "weigh_left", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_tc_9022_mass_sensor_data_t, weigh_left) }, \
         { "weigh_right", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_tc_9022_mass_sensor_data_t, weigh_right) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 2, 12, offsetof(ttalink_tc_9022_mass_sensor_data_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9022_MASS_SENSOR_DATA { \
    "TC_9022_MASS_SENSOR_DATA", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9022_mass_sensor_data_t, update_time) }, \
         { "weigh_left", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_tc_9022_mass_sensor_data_t, weigh_left) }, \
         { "weigh_right", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_tc_9022_mass_sensor_data_t, weigh_right) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 2, 12, offsetof(ttalink_tc_9022_mass_sensor_data_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9022_mass_sensor_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float weigh_left, float weigh_right, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, weigh_left);
    _tta_put_float(buf, 8, weigh_right);
    _tta_put_float_array(buf, 12, param, 2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN);
#else
    ttalink_tc_9022_mass_sensor_data_t packet;
    packet.update_time = update_time;
    packet.weigh_left = weigh_left;
    packet.weigh_right = weigh_right;
    tta_array_memcpy(packet.param, param, sizeof(float)*2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9022_mass_sensor_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param weigh_left  左重量.
 * @param weigh_right  右重量.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float weigh_left, float weigh_right, const float *param)
{
    return _ttalink_tc_9022_mass_sensor_data_pack(dst_addr, src_addr, msg,  update_time, weigh_left, weigh_right, param, false);
}

/**
 * @brief Pack a tc_9022_mass_sensor_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param weigh_left  左重量.
 * @param weigh_right  右重量.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float weigh_left, float weigh_right, const float *param)
{
    return _ttalink_tc_9022_mass_sensor_data_pack(dst_addr, src_addr, msg,  update_time, weigh_left, weigh_right, param, true);
}


static inline uint16_t _ttalink_tc_9022_mass_sensor_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float weigh_left,float weigh_right,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, weigh_left);
    _tta_put_float(buf, 8, weigh_right);
    _tta_put_float_array(buf, 12, param, 2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN);
#else
    ttalink_tc_9022_mass_sensor_data_t packet;
    packet.update_time = update_time;
    packet.weigh_left = weigh_left;
    packet.weigh_right = weigh_right;
    tta_array_memcpy(packet.param, param, sizeof(float)*2);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9022_mass_sensor_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param weigh_left  左重量.
 * @param weigh_right  右重量.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float weigh_left,float weigh_right,const float *param)
{
    return _ttalink_tc_9022_mass_sensor_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, weigh_left, weigh_right, param, false);
}

/**
 * @brief Pack a tc_9022_mass_sensor_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param weigh_left  左重量.
 * @param weigh_right  右重量.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float weigh_left,float weigh_right,const float *param)
{
    return _ttalink_tc_9022_mass_sensor_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, weigh_left, weigh_right, param, true);
}


static inline uint16_t _ttalink_tc_9022_mass_sensor_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9022_mass_sensor_data_pack_nocrc(dst_addr, src_addr, msg, tc_9022_mass_sensor_data->update_time, tc_9022_mass_sensor_data->weigh_left, tc_9022_mass_sensor_data->weigh_right, tc_9022_mass_sensor_data->param);
    }else{
        return ttalink_tc_9022_mass_sensor_data_pack(dst_addr, src_addr, msg, tc_9022_mass_sensor_data->update_time, tc_9022_mass_sensor_data->weigh_left, tc_9022_mass_sensor_data->weigh_right, tc_9022_mass_sensor_data->param);
    }
    
}

/**
 * @brief Encode a tc_9022_mass_sensor_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9022_mass_sensor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data)
{
    return _ttalink_tc_9022_mass_sensor_data_encode(dst_addr, src_addr, msg, tc_9022_mass_sensor_data, false);
}

/**
 * @brief Encode a tc_9022_mass_sensor_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9022_mass_sensor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data)
{
    return _ttalink_tc_9022_mass_sensor_data_encode(dst_addr, src_addr, msg, tc_9022_mass_sensor_data, true);
}


static inline uint16_t _ttalink_tc_9022_mass_sensor_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9022_mass_sensor_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9022_mass_sensor_data->update_time, tc_9022_mass_sensor_data->weigh_left, tc_9022_mass_sensor_data->weigh_right, tc_9022_mass_sensor_data->param);
    }else{
        return ttalink_tc_9022_mass_sensor_data_pack_chan(dst_addr, src_addr, chan, msg, tc_9022_mass_sensor_data->update_time, tc_9022_mass_sensor_data->weigh_left, tc_9022_mass_sensor_data->weigh_right, tc_9022_mass_sensor_data->param);
    }
}

/**
 * @brief Encode a tc_9022_mass_sensor_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9022_mass_sensor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data)
{
    return _ttalink_tc_9022_mass_sensor_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9022_mass_sensor_data, false);
}

/**
 * @brief Encode a tc_9022_mass_sensor_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9022_mass_sensor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data)
{
    return _ttalink_tc_9022_mass_sensor_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9022_mass_sensor_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9022_mass_sensor_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float weigh_left, float weigh_right, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, weigh_left);
    _tta_put_float(buf, 8, weigh_right);
    _tta_put_float_array(buf, 12, param, 2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA, buf, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC, nocrc);
#else
    ttalink_tc_9022_mass_sensor_data_t packet;
    packet.update_time = update_time;
    packet.weigh_left = weigh_left;
    packet.weigh_right = weigh_right;
    tta_array_memcpy(packet.param, param, sizeof(float)*2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA, (const char *)&packet, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9022_mass_sensor_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param weigh_left  左重量.
 * @param weigh_right  右重量.
 * @param param  .
 */
static inline void ttalink_tc_9022_mass_sensor_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float weigh_left, float weigh_right, const float *param)
{
    _ttalink_tc_9022_mass_sensor_data_send(dst_addr, src_addr, chan, update_time, weigh_left, weigh_right, param, false);
}

/**
 * @brief Send a tc_9022_mass_sensor_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param weigh_left  左重量.
 * @param weigh_right  右重量.
 * @param param  .
 */
static inline void ttalink_tc_9022_mass_sensor_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float weigh_left, float weigh_right, const float *param)
{
    _ttalink_tc_9022_mass_sensor_data_send(dst_addr, src_addr, chan, update_time, weigh_left, weigh_right, param, true);
}


static inline void _ttalink_tc_9022_mass_sensor_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9022_mass_sensor_data_send_nocrc(dst_addr, src_addr, chan, tc_9022_mass_sensor_data->update_time, tc_9022_mass_sensor_data->weigh_left, tc_9022_mass_sensor_data->weigh_right, tc_9022_mass_sensor_data->param);
    }else{
        ttalink_tc_9022_mass_sensor_data_send(dst_addr, src_addr, chan, tc_9022_mass_sensor_data->update_time, tc_9022_mass_sensor_data->weigh_left, tc_9022_mass_sensor_data->weigh_right, tc_9022_mass_sensor_data->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA, (const char *)tc_9022_mass_sensor_data, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9022_mass_sensor_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9022_mass_sensor_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data)
{
    _ttalink_tc_9022_mass_sensor_data_send_struct(dst_addr, src_addr, chan, tc_9022_mass_sensor_data, false);
}

/**
 * @brief Send a tc_9022_mass_sensor_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9022_mass_sensor_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data)
{
    _ttalink_tc_9022_mass_sensor_data_send_struct(dst_addr, src_addr, chan, tc_9022_mass_sensor_data, true);
}

#if TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9022_mass_sensor_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float weigh_left, float weigh_right, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, weigh_left);
    _tta_put_float(buf, 8, weigh_right);
    _tta_put_float_array(buf, 12, param, 2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA, buf, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC, nocrc);
#else
    ttalink_tc_9022_mass_sensor_data_t *packet = (ttalink_tc_9022_mass_sensor_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->weigh_left = weigh_left;
    packet->weigh_right = weigh_right;
    tta_array_memcpy(packet->param, param, sizeof(float)*2);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA, (const char *)packet, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9022_mass_sensor_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float weigh_left, float weigh_right, const float *param)
{
    _ttalink_tc_9022_mass_sensor_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, weigh_left, weigh_right, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9022_mass_sensor_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float weigh_left, float weigh_right, const float *param)
{
    _ttalink_tc_9022_mass_sensor_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, weigh_left, weigh_right, param, true);
}
#endif

#endif

// MESSAGE TC_9022_MASS_SENSOR_DATA UNPACKING


/**
 * @brief Get field update_time from tc_9022_mass_sensor_data message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9022_mass_sensor_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field weigh_left from tc_9022_mass_sensor_data message
 *
 * @return  左重量.
 */
static inline float ttalink_tc_9022_mass_sensor_data_get_weigh_left(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field weigh_right from tc_9022_mass_sensor_data message
 *
 * @return  右重量.
 */
static inline float ttalink_tc_9022_mass_sensor_data_get_weigh_right(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field param from tc_9022_mass_sensor_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_tc_9022_mass_sensor_data_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 2,  12);
}

/**
 * @brief Decode a tc_9022_mass_sensor_data message into a struct
 *
 * @param msg The message to decode
 * @param tc_9022_mass_sensor_data C-struct to decode the message contents into
 */
static inline void ttalink_tc_9022_mass_sensor_data_decode(const ttalink_message_t* msg, ttalink_tc_9022_mass_sensor_data_t* tc_9022_mass_sensor_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9022_mass_sensor_data->update_time = ttalink_tc_9022_mass_sensor_data_get_update_time(msg);
    tc_9022_mass_sensor_data->weigh_left = ttalink_tc_9022_mass_sensor_data_get_weigh_left(msg);
    tc_9022_mass_sensor_data->weigh_right = ttalink_tc_9022_mass_sensor_data_get_weigh_right(msg);
    ttalink_tc_9022_mass_sensor_data_get_param(msg, tc_9022_mass_sensor_data->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN? msg->len : TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN;
        memset(tc_9022_mass_sensor_data, 0, TTALINK_MSG_ID_TC_9022_MASS_SENSOR_DATA_LEN);
    memcpy(tc_9022_mass_sensor_data, _TTA_PAYLOAD(msg), len);
#endif
}
