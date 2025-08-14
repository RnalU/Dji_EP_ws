#pragma once
// MESSAGE SYNC_CALI_HOR PACKING

#define TTALINK_MSG_ID_SYNC_CALI_HOR 2123

TTAPACKED(
typedef struct __ttalink_sync_cali_hor_t {
 uint32_t simple_time; /*<  .*/
 float param[3]; /*<  .*/
}) ttalink_sync_cali_hor_t;

#define TTALINK_MSG_ID_SYNC_CALI_HOR_LEN 16
#define TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN 16
#define TTALINK_MSG_ID_2123_LEN 16
#define TTALINK_MSG_ID_2123_MIN_LEN 16

#define TTALINK_MSG_ID_SYNC_CALI_HOR_CRC 239
#define TTALINK_MSG_ID_2123_CRC 239

#define TTALINK_MSG_SYNC_CALI_HOR_FIELD_PARAM_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CALI_HOR { \
    2123, \
    "SYNC_CALI_HOR", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_hor_t, simple_time) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_sync_cali_hor_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CALI_HOR { \
    "SYNC_CALI_HOR", \
    2, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_hor_t, simple_time) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_sync_cali_hor_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_cali_hor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_HOR_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, param, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN);
#else
    ttalink_sync_cali_hor_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.param, param, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_HOR;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_hor message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_hor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *param)
{
    return _ttalink_sync_cali_hor_pack(dst_addr, src_addr, msg,  simple_time, param, false);
}

/**
 * @brief Pack a sync_cali_hor message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_hor_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *param)
{
    return _ttalink_sync_cali_hor_pack(dst_addr, src_addr, msg,  simple_time, param, true);
}


static inline uint16_t _ttalink_sync_cali_hor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_HOR_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, param, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN);
#else
    ttalink_sync_cali_hor_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.param, param, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_HOR;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_hor message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_hor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *param)
{
    return _ttalink_sync_cali_hor_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, param, false);
}

/**
 * @brief Pack a sync_cali_hor message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_hor_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *param)
{
    return _ttalink_sync_cali_hor_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, param, true);
}


static inline uint16_t _ttalink_sync_cali_hor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_hor_t* sync_cali_hor, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_hor_pack_nocrc(dst_addr, src_addr, msg, sync_cali_hor->simple_time, sync_cali_hor->param);
    }else{
        return ttalink_sync_cali_hor_pack(dst_addr, src_addr, msg, sync_cali_hor->simple_time, sync_cali_hor->param);
    }
    
}

/**
 * @brief Encode a sync_cali_hor struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_hor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_hor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_hor_t* sync_cali_hor)
{
    return _ttalink_sync_cali_hor_encode(dst_addr, src_addr, msg, sync_cali_hor, false);
}

/**
 * @brief Encode a sync_cali_hor struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_hor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_hor_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_hor_t* sync_cali_hor)
{
    return _ttalink_sync_cali_hor_encode(dst_addr, src_addr, msg, sync_cali_hor, true);
}


static inline uint16_t _ttalink_sync_cali_hor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_hor_t* sync_cali_hor, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_hor_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_cali_hor->simple_time, sync_cali_hor->param);
    }else{
        return ttalink_sync_cali_hor_pack_chan(dst_addr, src_addr, chan, msg, sync_cali_hor->simple_time, sync_cali_hor->param);
    }
}

/**
 * @brief Encode a sync_cali_hor struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_hor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_hor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_hor_t* sync_cali_hor)
{
    return _ttalink_sync_cali_hor_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_hor, false);
}

/**
 * @brief Encode a sync_cali_hor struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_hor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_hor_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_hor_t* sync_cali_hor)
{
    return _ttalink_sync_cali_hor_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_hor, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_cali_hor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_HOR_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, param, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_HOR, buf, TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_CRC, nocrc);
#else
    ttalink_sync_cali_hor_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.param, param, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_HOR, (const char *)&packet, TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_hor message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param param  .
 */
static inline void ttalink_sync_cali_hor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *param)
{
    _ttalink_sync_cali_hor_send(dst_addr, src_addr, chan, simple_time, param, false);
}

/**
 * @brief Send a sync_cali_hor message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param param  .
 */
static inline void ttalink_sync_cali_hor_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *param)
{
    _ttalink_sync_cali_hor_send(dst_addr, src_addr, chan, simple_time, param, true);
}


static inline void _ttalink_sync_cali_hor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_hor_t* sync_cali_hor, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_cali_hor_send_nocrc(dst_addr, src_addr, chan, sync_cali_hor->simple_time, sync_cali_hor->param);
    }else{
        ttalink_sync_cali_hor_send(dst_addr, src_addr, chan, sync_cali_hor->simple_time, sync_cali_hor->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_HOR, (const char *)sync_cali_hor, TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_hor message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_hor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_hor_t* sync_cali_hor)
{
    _ttalink_sync_cali_hor_send_struct(dst_addr, src_addr, chan, sync_cali_hor, false);
}

/**
 * @brief Send a sync_cali_hor message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_hor_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_hor_t* sync_cali_hor)
{
    _ttalink_sync_cali_hor_send_struct(dst_addr, src_addr, chan, sync_cali_hor, true);
}

#if TTALINK_MSG_ID_SYNC_CALI_HOR_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_cali_hor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, param, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_HOR, buf, TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_CRC, nocrc);
#else
    ttalink_sync_cali_hor_t *packet = (ttalink_sync_cali_hor_t *)msgbuf;
    packet->simple_time = simple_time;
    tta_array_memcpy(packet->param, param, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_HOR, (const char *)packet, TTALINK_MSG_ID_SYNC_CALI_HOR_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN, TTALINK_MSG_ID_SYNC_CALI_HOR_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_cali_hor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *param)
{
    _ttalink_sync_cali_hor_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_cali_hor_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *param)
{
    _ttalink_sync_cali_hor_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, param, true);
}
#endif

#endif

// MESSAGE SYNC_CALI_HOR UNPACKING


/**
 * @brief Get field simple_time from sync_cali_hor message
 *
 * @return  .
 */
static inline uint32_t ttalink_sync_cali_hor_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field param from sync_cali_hor message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_hor_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 3,  4);
}

/**
 * @brief Decode a sync_cali_hor message into a struct
 *
 * @param msg The message to decode
 * @param sync_cali_hor C-struct to decode the message contents into
 */
static inline void ttalink_sync_cali_hor_decode(const ttalink_message_t* msg, ttalink_sync_cali_hor_t* sync_cali_hor)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_cali_hor->simple_time = ttalink_sync_cali_hor_get_simple_time(msg);
    ttalink_sync_cali_hor_get_param(msg, sync_cali_hor->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CALI_HOR_LEN? msg->len : TTALINK_MSG_ID_SYNC_CALI_HOR_LEN;
        memset(sync_cali_hor, 0, TTALINK_MSG_ID_SYNC_CALI_HOR_LEN);
    memcpy(sync_cali_hor, _TTA_PAYLOAD(msg), len);
#endif
}
