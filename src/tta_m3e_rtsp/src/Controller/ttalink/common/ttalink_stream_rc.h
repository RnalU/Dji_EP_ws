#pragma once
// MESSAGE STREAM_RC PACKING

#define TTALINK_MSG_ID_STREAM_RC 2230

TTAPACKED(
typedef struct __ttalink_stream_rc_t {
 uint16_t rc_input[20]; /*< SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能*/
}) ttalink_stream_rc_t;

#define TTALINK_MSG_ID_STREAM_RC_LEN 40
#define TTALINK_MSG_ID_STREAM_RC_MIN_LEN 40
#define TTALINK_MSG_ID_2230_LEN 40
#define TTALINK_MSG_ID_2230_MIN_LEN 40

#define TTALINK_MSG_ID_STREAM_RC_CRC 0
#define TTALINK_MSG_ID_2230_CRC 0

#define TTALINK_MSG_STREAM_RC_FIELD_RC_INPUT_LEN 20

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_STREAM_RC { \
    2230, \
    "STREAM_RC", \
    1, \
    {  { "rc_input", NULL, TTALINK_TYPE_UINT16_T, 20, 0, offsetof(ttalink_stream_rc_t, rc_input) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_STREAM_RC { \
    "STREAM_RC", \
    1, \
    {  { "rc_input", NULL, TTALINK_TYPE_UINT16_T, 20, 0, offsetof(ttalink_stream_rc_t, rc_input) }, \
         } \
}
#endif


static inline uint16_t _ttalink_stream_rc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_RC_LEN];

    _tta_put_uint16_t_array(buf, 0, rc_input, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_RC_LEN);
#else
    ttalink_stream_rc_t packet;

    tta_array_memcpy(packet.rc_input, rc_input, sizeof(uint16_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_RC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_RC;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_STREAM_RC_MIN_LEN, TTALINK_MSG_ID_STREAM_RC_LEN, TTALINK_MSG_ID_STREAM_RC_CRC, nocrc);
}

/**
 * @brief Pack a stream_rc message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_rc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint16_t *rc_input)
{
    return _ttalink_stream_rc_pack(dst_addr, src_addr, msg,  rc_input, false);
}

/**
 * @brief Pack a stream_rc message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_rc_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint16_t *rc_input)
{
    return _ttalink_stream_rc_pack(dst_addr, src_addr, msg,  rc_input, true);
}


static inline uint16_t _ttalink_stream_rc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_RC_LEN];

    _tta_put_uint16_t_array(buf, 0, rc_input, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_RC_LEN);
#else
    ttalink_stream_rc_t packet;

    tta_array_memcpy(packet.rc_input, rc_input, sizeof(uint16_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_RC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_RC;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_RC_MIN_LEN, TTALINK_MSG_ID_STREAM_RC_LEN, TTALINK_MSG_ID_STREAM_RC_CRC, nocrc);
}

/**
 * @brief Pack a stream_rc message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_rc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint16_t *rc_input)
{
    return _ttalink_stream_rc_pack_chan(dst_addr, src_addr, chan, msg,  rc_input, false);
}

/**
 * @brief Pack a stream_rc message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_rc_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint16_t *rc_input)
{
    return _ttalink_stream_rc_pack_chan(dst_addr, src_addr, chan, msg,  rc_input, true);
}


static inline uint16_t _ttalink_stream_rc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_rc_t* stream_rc, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_rc_pack_nocrc(dst_addr, src_addr, msg, stream_rc->rc_input);
    }else{
        return ttalink_stream_rc_pack(dst_addr, src_addr, msg, stream_rc->rc_input);
    }
    
}

/**
 * @brief Encode a stream_rc struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_rc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_rc_t* stream_rc)
{
    return _ttalink_stream_rc_encode(dst_addr, src_addr, msg, stream_rc, false);
}

/**
 * @brief Encode a stream_rc struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_rc_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_rc_t* stream_rc)
{
    return _ttalink_stream_rc_encode(dst_addr, src_addr, msg, stream_rc, true);
}


static inline uint16_t _ttalink_stream_rc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_rc_t* stream_rc, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_rc_pack_chan_nocrc(dst_addr, src_addr, chan, msg, stream_rc->rc_input);
    }else{
        return ttalink_stream_rc_pack_chan(dst_addr, src_addr, chan, msg, stream_rc->rc_input);
    }
}

/**
 * @brief Encode a stream_rc struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_rc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_rc_t* stream_rc)
{
    return _ttalink_stream_rc_encode_chan(dst_addr, src_addr, chan, msg, stream_rc, false);
}

/**
 * @brief Encode a stream_rc struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_rc_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_rc_t* stream_rc)
{
    return _ttalink_stream_rc_encode_chan(dst_addr, src_addr, chan, msg, stream_rc, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_stream_rc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_RC_LEN];

    _tta_put_uint16_t_array(buf, 0, rc_input, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_RC, buf, TTALINK_MSG_ID_STREAM_RC_MIN_LEN, TTALINK_MSG_ID_STREAM_RC_LEN, TTALINK_MSG_ID_STREAM_RC_CRC, nocrc);
#else
    ttalink_stream_rc_t packet;

    tta_array_memcpy(packet.rc_input, rc_input, sizeof(uint16_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_RC, (const char *)&packet, TTALINK_MSG_ID_STREAM_RC_MIN_LEN, TTALINK_MSG_ID_STREAM_RC_LEN, TTALINK_MSG_ID_STREAM_RC_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_rc message
 * @param chan TTAlink channel to send the message
 *
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能
 */
static inline void ttalink_stream_rc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint16_t *rc_input)
{
    _ttalink_stream_rc_send(dst_addr, src_addr, chan, rc_input, false);
}

/**
 * @brief Send a stream_rc message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能
 */
static inline void ttalink_stream_rc_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint16_t *rc_input)
{
    _ttalink_stream_rc_send(dst_addr, src_addr, chan, rc_input, true);
}


static inline void _ttalink_stream_rc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_rc_t* stream_rc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_stream_rc_send_nocrc(dst_addr, src_addr, chan, stream_rc->rc_input);
    }else{
        ttalink_stream_rc_send(dst_addr, src_addr, chan, stream_rc->rc_input);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_RC, (const char *)stream_rc, TTALINK_MSG_ID_STREAM_RC_MIN_LEN, TTALINK_MSG_ID_STREAM_RC_LEN, TTALINK_MSG_ID_STREAM_RC_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_rc message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_rc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_rc_t* stream_rc)
{
    _ttalink_stream_rc_send_struct(dst_addr, src_addr, chan, stream_rc, false);
}

/**
 * @brief Send a stream_rc message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_rc_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_rc_t* stream_rc)
{
    _ttalink_stream_rc_send_struct(dst_addr, src_addr, chan, stream_rc, true);
}

#if TTALINK_MSG_ID_STREAM_RC_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_stream_rc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_uint16_t_array(buf, 0, rc_input, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_RC, buf, TTALINK_MSG_ID_STREAM_RC_MIN_LEN, TTALINK_MSG_ID_STREAM_RC_LEN, TTALINK_MSG_ID_STREAM_RC_CRC, nocrc);
#else
    ttalink_stream_rc_t *packet = (ttalink_stream_rc_t *)msgbuf;

    tta_array_memcpy(packet->rc_input, rc_input, sizeof(uint16_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_RC, (const char *)packet, TTALINK_MSG_ID_STREAM_RC_MIN_LEN, TTALINK_MSG_ID_STREAM_RC_LEN, TTALINK_MSG_ID_STREAM_RC_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_stream_rc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint16_t *rc_input)
{
    _ttalink_stream_rc_send_buf(dst_addr, src_addr, msgbuf, chan, rc_input, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_stream_rc_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint16_t *rc_input)
{
    _ttalink_stream_rc_send_buf(dst_addr, src_addr, msgbuf, chan, rc_input, true);
}
#endif

#endif

// MESSAGE STREAM_RC UNPACKING


/**
 * @brief Get field rc_input from stream_rc message
 *
 * @return SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记，与（19）激活故障保护功能
 */
static inline uint16_t ttalink_stream_rc_get_rc_input(const ttalink_message_t* msg, uint16_t *rc_input)
{
    return _TTA_RETURN_uint16_t_array(msg, rc_input, 20,  0);
}

/**
 * @brief Decode a stream_rc message into a struct
 *
 * @param msg The message to decode
 * @param stream_rc C-struct to decode the message contents into
 */
static inline void ttalink_stream_rc_decode(const ttalink_message_t* msg, ttalink_stream_rc_t* stream_rc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_stream_rc_get_rc_input(msg, stream_rc->rc_input);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_STREAM_RC_LEN? msg->len : TTALINK_MSG_ID_STREAM_RC_LEN;
        memset(stream_rc, 0, TTALINK_MSG_ID_STREAM_RC_LEN);
    memcpy(stream_rc, _TTA_PAYLOAD(msg), len);
#endif
}
