#pragma once
// MESSAGE GENERAL_VOICE_BROADCAST PACKING

#define TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST 4115

TTAPACKED(
typedef struct __ttalink_general_voice_broadcast_t {
 uint8_t mode; /*< 0:播放字符串 1：列表查询 .*/
 uint8_t index; /*< 播放列表.*/
 float param; /*<  .*/
}) ttalink_general_voice_broadcast_t;

#define TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN 6
#define TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN 6
#define TTALINK_MSG_ID_4115_LEN 6
#define TTALINK_MSG_ID_4115_MIN_LEN 6

#define TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC 159
#define TTALINK_MSG_ID_4115_CRC 159



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_VOICE_BROADCAST { \
    4115, \
    "GENERAL_VOICE_BROADCAST", \
    3, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_voice_broadcast_t, mode) }, \
         { "index", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_general_voice_broadcast_t, index) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 2, offsetof(ttalink_general_voice_broadcast_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_VOICE_BROADCAST { \
    "GENERAL_VOICE_BROADCAST", \
    3, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_voice_broadcast_t, mode) }, \
         { "index", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_general_voice_broadcast_t, index) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 0, 2, offsetof(ttalink_general_voice_broadcast_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_voice_broadcast_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t index, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, index);
    _tta_put_float(buf, 2, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN);
#else
    ttalink_general_voice_broadcast_t packet;
    packet.mode = mode;
    packet.index = index;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC, nocrc);
}

/**
 * @brief Pack a general_voice_broadcast message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode 0:播放字符串 1：列表查询 .
 * @param index 播放列表.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_voice_broadcast_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t index, float param)
{
    return _ttalink_general_voice_broadcast_pack(dst_addr, src_addr, msg,  mode, index, param, false);
}

/**
 * @brief Pack a general_voice_broadcast message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode 0:播放字符串 1：列表查询 .
 * @param index 播放列表.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_voice_broadcast_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t index, float param)
{
    return _ttalink_general_voice_broadcast_pack(dst_addr, src_addr, msg,  mode, index, param, true);
}


static inline uint16_t _ttalink_general_voice_broadcast_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t index,float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, index);
    _tta_put_float(buf, 2, param);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN);
#else
    ttalink_general_voice_broadcast_t packet;
    packet.mode = mode;
    packet.index = index;
    packet.param = param;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC, nocrc);
}

/**
 * @brief Pack a general_voice_broadcast message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode 0:播放字符串 1：列表查询 .
 * @param index 播放列表.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_voice_broadcast_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t index,float param)
{
    return _ttalink_general_voice_broadcast_pack_chan(dst_addr, src_addr, chan, msg,  mode, index, param, false);
}

/**
 * @brief Pack a general_voice_broadcast message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode 0:播放字符串 1：列表查询 .
 * @param index 播放列表.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_voice_broadcast_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t index,float param)
{
    return _ttalink_general_voice_broadcast_pack_chan(dst_addr, src_addr, chan, msg,  mode, index, param, true);
}


static inline uint16_t _ttalink_general_voice_broadcast_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_voice_broadcast_t* general_voice_broadcast, bool nocrc)
{
    if(nocrc){
        return ttalink_general_voice_broadcast_pack_nocrc(dst_addr, src_addr, msg, general_voice_broadcast->mode, general_voice_broadcast->index, general_voice_broadcast->param);
    }else{
        return ttalink_general_voice_broadcast_pack(dst_addr, src_addr, msg, general_voice_broadcast->mode, general_voice_broadcast->index, general_voice_broadcast->param);
    }
    
}

/**
 * @brief Encode a general_voice_broadcast struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_voice_broadcast C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_voice_broadcast_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_voice_broadcast_t* general_voice_broadcast)
{
    return _ttalink_general_voice_broadcast_encode(dst_addr, src_addr, msg, general_voice_broadcast, false);
}

/**
 * @brief Encode a general_voice_broadcast struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_voice_broadcast C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_voice_broadcast_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_voice_broadcast_t* general_voice_broadcast)
{
    return _ttalink_general_voice_broadcast_encode(dst_addr, src_addr, msg, general_voice_broadcast, true);
}


static inline uint16_t _ttalink_general_voice_broadcast_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_voice_broadcast_t* general_voice_broadcast, bool nocrc)
{
    if(nocrc){
        return ttalink_general_voice_broadcast_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_voice_broadcast->mode, general_voice_broadcast->index, general_voice_broadcast->param);
    }else{
        return ttalink_general_voice_broadcast_pack_chan(dst_addr, src_addr, chan, msg, general_voice_broadcast->mode, general_voice_broadcast->index, general_voice_broadcast->param);
    }
}

/**
 * @brief Encode a general_voice_broadcast struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_voice_broadcast C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_voice_broadcast_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_voice_broadcast_t* general_voice_broadcast)
{
    return _ttalink_general_voice_broadcast_encode_chan(dst_addr, src_addr, chan, msg, general_voice_broadcast, false);
}

/**
 * @brief Encode a general_voice_broadcast struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_voice_broadcast C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_voice_broadcast_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_voice_broadcast_t* general_voice_broadcast)
{
    return _ttalink_general_voice_broadcast_encode_chan(dst_addr, src_addr, chan, msg, general_voice_broadcast, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_voice_broadcast_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t index, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, index);
    _tta_put_float(buf, 2, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST, buf, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC, nocrc);
#else
    ttalink_general_voice_broadcast_t packet;
    packet.mode = mode;
    packet.index = index;
    packet.param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST, (const char *)&packet, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_voice_broadcast message
 * @param chan TTAlink channel to send the message
 *
 * @param mode 0:播放字符串 1：列表查询 .
 * @param index 播放列表.
 * @param param  .
 */
static inline void ttalink_general_voice_broadcast_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t index, float param)
{
    _ttalink_general_voice_broadcast_send(dst_addr, src_addr, chan, mode, index, param, false);
}

/**
 * @brief Send a general_voice_broadcast message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mode 0:播放字符串 1：列表查询 .
 * @param index 播放列表.
 * @param param  .
 */
static inline void ttalink_general_voice_broadcast_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t index, float param)
{
    _ttalink_general_voice_broadcast_send(dst_addr, src_addr, chan, mode, index, param, true);
}


static inline void _ttalink_general_voice_broadcast_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_voice_broadcast_t* general_voice_broadcast, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_voice_broadcast_send_nocrc(dst_addr, src_addr, chan, general_voice_broadcast->mode, general_voice_broadcast->index, general_voice_broadcast->param);
    }else{
        ttalink_general_voice_broadcast_send(dst_addr, src_addr, chan, general_voice_broadcast->mode, general_voice_broadcast->index, general_voice_broadcast->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST, (const char *)general_voice_broadcast, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_voice_broadcast message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_voice_broadcast_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_voice_broadcast_t* general_voice_broadcast)
{
    _ttalink_general_voice_broadcast_send_struct(dst_addr, src_addr, chan, general_voice_broadcast, false);
}

/**
 * @brief Send a general_voice_broadcast message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_voice_broadcast_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_voice_broadcast_t* general_voice_broadcast)
{
    _ttalink_general_voice_broadcast_send_struct(dst_addr, src_addr, chan, general_voice_broadcast, true);
}

#if TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_voice_broadcast_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t index, float param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, index);
    _tta_put_float(buf, 2, param);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST, buf, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC, nocrc);
#else
    ttalink_general_voice_broadcast_t *packet = (ttalink_general_voice_broadcast_t *)msgbuf;
    packet->mode = mode;
    packet->index = index;
    packet->param = param;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST, (const char *)packet, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_MIN_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_voice_broadcast_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t index, float param)
{
    _ttalink_general_voice_broadcast_send_buf(dst_addr, src_addr, msgbuf, chan, mode, index, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_voice_broadcast_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t index, float param)
{
    _ttalink_general_voice_broadcast_send_buf(dst_addr, src_addr, msgbuf, chan, mode, index, param, true);
}
#endif

#endif

// MESSAGE GENERAL_VOICE_BROADCAST UNPACKING


/**
 * @brief Get field mode from general_voice_broadcast message
 *
 * @return 0:播放字符串 1：列表查询 .
 */
static inline uint8_t ttalink_general_voice_broadcast_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field index from general_voice_broadcast message
 *
 * @return 播放列表.
 */
static inline uint8_t ttalink_general_voice_broadcast_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field param from general_voice_broadcast message
 *
 * @return  .
 */
static inline float ttalink_general_voice_broadcast_get_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  2);
}

/**
 * @brief Decode a general_voice_broadcast message into a struct
 *
 * @param msg The message to decode
 * @param general_voice_broadcast C-struct to decode the message contents into
 */
static inline void ttalink_general_voice_broadcast_decode(const ttalink_message_t* msg, ttalink_general_voice_broadcast_t* general_voice_broadcast)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_voice_broadcast->mode = ttalink_general_voice_broadcast_get_mode(msg);
    general_voice_broadcast->index = ttalink_general_voice_broadcast_get_index(msg);
    general_voice_broadcast->param = ttalink_general_voice_broadcast_get_param(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN? msg->len : TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN;
        memset(general_voice_broadcast, 0, TTALINK_MSG_ID_GENERAL_VOICE_BROADCAST_LEN);
    memcpy(general_voice_broadcast, _TTA_PAYLOAD(msg), len);
#endif
}
