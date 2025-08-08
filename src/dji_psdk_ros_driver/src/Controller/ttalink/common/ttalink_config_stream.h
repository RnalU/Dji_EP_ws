#pragma once
// MESSAGE CONFIG_STREAM PACKING

#define TTALINK_MSG_ID_CONFIG_STREAM 2140

TTAPACKED(
typedef struct __ttalink_config_stream_t {
 uint8_t stream_type; /*<  .*/
 uint8_t flags; /*<  .*/
 uint16_t delay_time; /*<  .*/
}) ttalink_config_stream_t;

#define TTALINK_MSG_ID_CONFIG_STREAM_LEN 4
#define TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN 4
#define TTALINK_MSG_ID_2140_LEN 4
#define TTALINK_MSG_ID_2140_MIN_LEN 4

#define TTALINK_MSG_ID_CONFIG_STREAM_CRC 251
#define TTALINK_MSG_ID_2140_CRC 251



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_CONFIG_STREAM { \
    2140, \
    "CONFIG_STREAM", \
    3, \
    {  { "stream_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_config_stream_t, stream_type) }, \
         { "flags", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_config_stream_t, flags) }, \
         { "delay_time", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_config_stream_t, delay_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_CONFIG_STREAM { \
    "CONFIG_STREAM", \
    3, \
    {  { "stream_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_config_stream_t, stream_type) }, \
         { "flags", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_config_stream_t, flags) }, \
         { "delay_time", NULL, TTALINK_TYPE_UINT16_T, 0, 2, offsetof(ttalink_config_stream_t, delay_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_config_stream_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stream_type, uint8_t flags, uint16_t delay_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CONFIG_STREAM_LEN];
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_uint8_t(buf, 1, flags);
    _tta_put_uint16_t(buf, 2, delay_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CONFIG_STREAM_LEN);
#else
    ttalink_config_stream_t packet;
    packet.stream_type = stream_type;
    packet.flags = flags;
    packet.delay_time = delay_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CONFIG_STREAM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CONFIG_STREAM;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN, TTALINK_MSG_ID_CONFIG_STREAM_LEN, TTALINK_MSG_ID_CONFIG_STREAM_CRC, nocrc);
}

/**
 * @brief Pack a config_stream message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param stream_type  .
 * @param flags  .
 * @param delay_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_config_stream_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stream_type, uint8_t flags, uint16_t delay_time)
{
    return _ttalink_config_stream_pack(dst_addr, src_addr, msg,  stream_type, flags, delay_time, false);
}

/**
 * @brief Pack a config_stream message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param stream_type  .
 * @param flags  .
 * @param delay_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_config_stream_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t stream_type, uint8_t flags, uint16_t delay_time)
{
    return _ttalink_config_stream_pack(dst_addr, src_addr, msg,  stream_type, flags, delay_time, true);
}


static inline uint16_t _ttalink_config_stream_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stream_type,uint8_t flags,uint16_t delay_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CONFIG_STREAM_LEN];
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_uint8_t(buf, 1, flags);
    _tta_put_uint16_t(buf, 2, delay_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CONFIG_STREAM_LEN);
#else
    ttalink_config_stream_t packet;
    packet.stream_type = stream_type;
    packet.flags = flags;
    packet.delay_time = delay_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CONFIG_STREAM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CONFIG_STREAM;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN, TTALINK_MSG_ID_CONFIG_STREAM_LEN, TTALINK_MSG_ID_CONFIG_STREAM_CRC, nocrc);
}

/**
 * @brief Pack a config_stream message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_type  .
 * @param flags  .
 * @param delay_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_config_stream_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stream_type,uint8_t flags,uint16_t delay_time)
{
    return _ttalink_config_stream_pack_chan(dst_addr, src_addr, chan, msg,  stream_type, flags, delay_time, false);
}

/**
 * @brief Pack a config_stream message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_type  .
 * @param flags  .
 * @param delay_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_config_stream_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t stream_type,uint8_t flags,uint16_t delay_time)
{
    return _ttalink_config_stream_pack_chan(dst_addr, src_addr, chan, msg,  stream_type, flags, delay_time, true);
}


static inline uint16_t _ttalink_config_stream_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_config_stream_t* config_stream, bool nocrc)
{
    if(nocrc){
        return ttalink_config_stream_pack_nocrc(dst_addr, src_addr, msg, config_stream->stream_type, config_stream->flags, config_stream->delay_time);
    }else{
        return ttalink_config_stream_pack(dst_addr, src_addr, msg, config_stream->stream_type, config_stream->flags, config_stream->delay_time);
    }
    
}

/**
 * @brief Encode a config_stream struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param config_stream C-struct to read the message contents from
 */
static inline uint16_t ttalink_config_stream_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_config_stream_t* config_stream)
{
    return _ttalink_config_stream_encode(dst_addr, src_addr, msg, config_stream, false);
}

/**
 * @brief Encode a config_stream struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param config_stream C-struct to read the message contents from
 */
static inline uint16_t ttalink_config_stream_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_config_stream_t* config_stream)
{
    return _ttalink_config_stream_encode(dst_addr, src_addr, msg, config_stream, true);
}


static inline uint16_t _ttalink_config_stream_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_config_stream_t* config_stream, bool nocrc)
{
    if(nocrc){
        return ttalink_config_stream_pack_chan_nocrc(dst_addr, src_addr, chan, msg, config_stream->stream_type, config_stream->flags, config_stream->delay_time);
    }else{
        return ttalink_config_stream_pack_chan(dst_addr, src_addr, chan, msg, config_stream->stream_type, config_stream->flags, config_stream->delay_time);
    }
}

/**
 * @brief Encode a config_stream struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param config_stream C-struct to read the message contents from
 */
static inline uint16_t ttalink_config_stream_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_config_stream_t* config_stream)
{
    return _ttalink_config_stream_encode_chan(dst_addr, src_addr, chan, msg, config_stream, false);
}

/**
 * @brief Encode a config_stream struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param config_stream C-struct to read the message contents from
 */
static inline uint16_t ttalink_config_stream_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_config_stream_t* config_stream)
{
    return _ttalink_config_stream_encode_chan(dst_addr, src_addr, chan, msg, config_stream, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_config_stream_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stream_type, uint8_t flags, uint16_t delay_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CONFIG_STREAM_LEN];
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_uint8_t(buf, 1, flags);
    _tta_put_uint16_t(buf, 2, delay_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CONFIG_STREAM, buf, TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN, TTALINK_MSG_ID_CONFIG_STREAM_LEN, TTALINK_MSG_ID_CONFIG_STREAM_CRC, nocrc);
#else
    ttalink_config_stream_t packet;
    packet.stream_type = stream_type;
    packet.flags = flags;
    packet.delay_time = delay_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CONFIG_STREAM, (const char *)&packet, TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN, TTALINK_MSG_ID_CONFIG_STREAM_LEN, TTALINK_MSG_ID_CONFIG_STREAM_CRC, nocrc);
#endif
}

/**
 * @brief Send a config_stream message
 * @param chan TTAlink channel to send the message
 *
 * @param stream_type  .
 * @param flags  .
 * @param delay_time  .
 */
static inline void ttalink_config_stream_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stream_type, uint8_t flags, uint16_t delay_time)
{
    _ttalink_config_stream_send(dst_addr, src_addr, chan, stream_type, flags, delay_time, false);
}

/**
 * @brief Send a config_stream message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param stream_type  .
 * @param flags  .
 * @param delay_time  .
 */
static inline void ttalink_config_stream_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t stream_type, uint8_t flags, uint16_t delay_time)
{
    _ttalink_config_stream_send(dst_addr, src_addr, chan, stream_type, flags, delay_time, true);
}


static inline void _ttalink_config_stream_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_config_stream_t* config_stream, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_config_stream_send_nocrc(dst_addr, src_addr, chan, config_stream->stream_type, config_stream->flags, config_stream->delay_time);
    }else{
        ttalink_config_stream_send(dst_addr, src_addr, chan, config_stream->stream_type, config_stream->flags, config_stream->delay_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CONFIG_STREAM, (const char *)config_stream, TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN, TTALINK_MSG_ID_CONFIG_STREAM_LEN, TTALINK_MSG_ID_CONFIG_STREAM_CRC, nocrc);
#endif
}

/**
 * @brief Send a config_stream message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_config_stream_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_config_stream_t* config_stream)
{
    _ttalink_config_stream_send_struct(dst_addr, src_addr, chan, config_stream, false);
}

/**
 * @brief Send a config_stream message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_config_stream_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_config_stream_t* config_stream)
{
    _ttalink_config_stream_send_struct(dst_addr, src_addr, chan, config_stream, true);
}

#if TTALINK_MSG_ID_CONFIG_STREAM_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_config_stream_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stream_type, uint8_t flags, uint16_t delay_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, stream_type);
    _tta_put_uint8_t(buf, 1, flags);
    _tta_put_uint16_t(buf, 2, delay_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CONFIG_STREAM, buf, TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN, TTALINK_MSG_ID_CONFIG_STREAM_LEN, TTALINK_MSG_ID_CONFIG_STREAM_CRC, nocrc);
#else
    ttalink_config_stream_t *packet = (ttalink_config_stream_t *)msgbuf;
    packet->stream_type = stream_type;
    packet->flags = flags;
    packet->delay_time = delay_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CONFIG_STREAM, (const char *)packet, TTALINK_MSG_ID_CONFIG_STREAM_MIN_LEN, TTALINK_MSG_ID_CONFIG_STREAM_LEN, TTALINK_MSG_ID_CONFIG_STREAM_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_config_stream_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stream_type, uint8_t flags, uint16_t delay_time)
{
    _ttalink_config_stream_send_buf(dst_addr, src_addr, msgbuf, chan, stream_type, flags, delay_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_config_stream_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t stream_type, uint8_t flags, uint16_t delay_time)
{
    _ttalink_config_stream_send_buf(dst_addr, src_addr, msgbuf, chan, stream_type, flags, delay_time, true);
}
#endif

#endif

// MESSAGE CONFIG_STREAM UNPACKING


/**
 * @brief Get field stream_type from config_stream message
 *
 * @return  .
 */
static inline uint8_t ttalink_config_stream_get_stream_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field flags from config_stream message
 *
 * @return  .
 */
static inline uint8_t ttalink_config_stream_get_flags(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field delay_time from config_stream message
 *
 * @return  .
 */
static inline uint16_t ttalink_config_stream_get_delay_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a config_stream message into a struct
 *
 * @param msg The message to decode
 * @param config_stream C-struct to decode the message contents into
 */
static inline void ttalink_config_stream_decode(const ttalink_message_t* msg, ttalink_config_stream_t* config_stream)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    config_stream->stream_type = ttalink_config_stream_get_stream_type(msg);
    config_stream->flags = ttalink_config_stream_get_flags(msg);
    config_stream->delay_time = ttalink_config_stream_get_delay_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_CONFIG_STREAM_LEN? msg->len : TTALINK_MSG_ID_CONFIG_STREAM_LEN;
        memset(config_stream, 0, TTALINK_MSG_ID_CONFIG_STREAM_LEN);
    memcpy(config_stream, _TTA_PAYLOAD(msg), len);
#endif
}
