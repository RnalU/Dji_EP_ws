#pragma once
// MESSAGE EXTERN_MESSAGE PACKING

#define TTALINK_MSG_ID_EXTERN_MESSAGE 2253

TTAPACKED(
typedef struct __ttalink_extern_message_t {
 float param[10]; /*< extern message.*/
}) ttalink_extern_message_t;

#define TTALINK_MSG_ID_EXTERN_MESSAGE_LEN 40
#define TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN 40
#define TTALINK_MSG_ID_2253_LEN 40
#define TTALINK_MSG_ID_2253_MIN_LEN 40

#define TTALINK_MSG_ID_EXTERN_MESSAGE_CRC 185
#define TTALINK_MSG_ID_2253_CRC 185

#define TTALINK_MSG_EXTERN_MESSAGE_FIELD_PARAM_LEN 10

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_EXTERN_MESSAGE { \
    2253, \
    "EXTERN_MESSAGE", \
    1, \
    {  { "param", NULL, TTALINK_TYPE_FLOAT, 10, 0, offsetof(ttalink_extern_message_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_EXTERN_MESSAGE { \
    "EXTERN_MESSAGE", \
    1, \
    {  { "param", NULL, TTALINK_TYPE_FLOAT, 10, 0, offsetof(ttalink_extern_message_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_extern_message_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXTERN_MESSAGE_LEN];

    _tta_put_float_array(buf, 0, param, 10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN);
#else
    ttalink_extern_message_t packet;

    tta_array_memcpy(packet.param, param, sizeof(float)*10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EXTERN_MESSAGE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_CRC, nocrc);
}

/**
 * @brief Pack a extern_message message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param param extern message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_extern_message_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *param)
{
    return _ttalink_extern_message_pack(dst_addr, src_addr, msg,  param, false);
}

/**
 * @brief Pack a extern_message message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param param extern message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_extern_message_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *param)
{
    return _ttalink_extern_message_pack(dst_addr, src_addr, msg,  param, true);
}


static inline uint16_t _ttalink_extern_message_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXTERN_MESSAGE_LEN];

    _tta_put_float_array(buf, 0, param, 10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN);
#else
    ttalink_extern_message_t packet;

    tta_array_memcpy(packet.param, param, sizeof(float)*10);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EXTERN_MESSAGE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_CRC, nocrc);
}

/**
 * @brief Pack a extern_message message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param param extern message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_extern_message_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *param)
{
    return _ttalink_extern_message_pack_chan(dst_addr, src_addr, chan, msg,  param, false);
}

/**
 * @brief Pack a extern_message message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param param extern message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_extern_message_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *param)
{
    return _ttalink_extern_message_pack_chan(dst_addr, src_addr, chan, msg,  param, true);
}


static inline uint16_t _ttalink_extern_message_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_extern_message_t* extern_message, bool nocrc)
{
    if(nocrc){
        return ttalink_extern_message_pack_nocrc(dst_addr, src_addr, msg, extern_message->param);
    }else{
        return ttalink_extern_message_pack(dst_addr, src_addr, msg, extern_message->param);
    }
    
}

/**
 * @brief Encode a extern_message struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param extern_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_extern_message_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_extern_message_t* extern_message)
{
    return _ttalink_extern_message_encode(dst_addr, src_addr, msg, extern_message, false);
}

/**
 * @brief Encode a extern_message struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param extern_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_extern_message_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_extern_message_t* extern_message)
{
    return _ttalink_extern_message_encode(dst_addr, src_addr, msg, extern_message, true);
}


static inline uint16_t _ttalink_extern_message_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_extern_message_t* extern_message, bool nocrc)
{
    if(nocrc){
        return ttalink_extern_message_pack_chan_nocrc(dst_addr, src_addr, chan, msg, extern_message->param);
    }else{
        return ttalink_extern_message_pack_chan(dst_addr, src_addr, chan, msg, extern_message->param);
    }
}

/**
 * @brief Encode a extern_message struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param extern_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_extern_message_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_extern_message_t* extern_message)
{
    return _ttalink_extern_message_encode_chan(dst_addr, src_addr, chan, msg, extern_message, false);
}

/**
 * @brief Encode a extern_message struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param extern_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_extern_message_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_extern_message_t* extern_message)
{
    return _ttalink_extern_message_encode_chan(dst_addr, src_addr, chan, msg, extern_message, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_extern_message_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXTERN_MESSAGE_LEN];

    _tta_put_float_array(buf, 0, param, 10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXTERN_MESSAGE, buf, TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_CRC, nocrc);
#else
    ttalink_extern_message_t packet;

    tta_array_memcpy(packet.param, param, sizeof(float)*10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXTERN_MESSAGE, (const char *)&packet, TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_CRC, nocrc);
#endif
}

/**
 * @brief Send a extern_message message
 * @param chan TTAlink channel to send the message
 *
 * @param param extern message.
 */
static inline void ttalink_extern_message_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *param)
{
    _ttalink_extern_message_send(dst_addr, src_addr, chan, param, false);
}

/**
 * @brief Send a extern_message message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param param extern message.
 */
static inline void ttalink_extern_message_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *param)
{
    _ttalink_extern_message_send(dst_addr, src_addr, chan, param, true);
}


static inline void _ttalink_extern_message_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_extern_message_t* extern_message, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_extern_message_send_nocrc(dst_addr, src_addr, chan, extern_message->param);
    }else{
        ttalink_extern_message_send(dst_addr, src_addr, chan, extern_message->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXTERN_MESSAGE, (const char *)extern_message, TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_CRC, nocrc);
#endif
}

/**
 * @brief Send a extern_message message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_extern_message_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_extern_message_t* extern_message)
{
    _ttalink_extern_message_send_struct(dst_addr, src_addr, chan, extern_message, false);
}

/**
 * @brief Send a extern_message message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_extern_message_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_extern_message_t* extern_message)
{
    _ttalink_extern_message_send_struct(dst_addr, src_addr, chan, extern_message, true);
}

#if TTALINK_MSG_ID_EXTERN_MESSAGE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_extern_message_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_float_array(buf, 0, param, 10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXTERN_MESSAGE, buf, TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_CRC, nocrc);
#else
    ttalink_extern_message_t *packet = (ttalink_extern_message_t *)msgbuf;

    tta_array_memcpy(packet->param, param, sizeof(float)*10);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXTERN_MESSAGE, (const char *)packet, TTALINK_MSG_ID_EXTERN_MESSAGE_MIN_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN, TTALINK_MSG_ID_EXTERN_MESSAGE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_extern_message_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *param)
{
    _ttalink_extern_message_send_buf(dst_addr, src_addr, msgbuf, chan, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_extern_message_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *param)
{
    _ttalink_extern_message_send_buf(dst_addr, src_addr, msgbuf, chan, param, true);
}
#endif

#endif

// MESSAGE EXTERN_MESSAGE UNPACKING


/**
 * @brief Get field param from extern_message message
 *
 * @return extern message.
 */
static inline uint16_t ttalink_extern_message_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 10,  0);
}

/**
 * @brief Decode a extern_message message into a struct
 *
 * @param msg The message to decode
 * @param extern_message C-struct to decode the message contents into
 */
static inline void ttalink_extern_message_decode(const ttalink_message_t* msg, ttalink_extern_message_t* extern_message)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_extern_message_get_param(msg, extern_message->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_EXTERN_MESSAGE_LEN? msg->len : TTALINK_MSG_ID_EXTERN_MESSAGE_LEN;
        memset(extern_message, 0, TTALINK_MSG_ID_EXTERN_MESSAGE_LEN);
    memcpy(extern_message, _TTA_PAYLOAD(msg), len);
#endif
}
