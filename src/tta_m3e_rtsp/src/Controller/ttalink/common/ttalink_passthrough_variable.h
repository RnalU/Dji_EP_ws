#pragma once
// MESSAGE PASSTHROUGH_VARIABLE PACKING

#define TTALINK_MSG_ID_PASSTHROUGH_VARIABLE 2244

TTAPACKED(
typedef struct __ttalink_passthrough_variable_t {
 uint16_t length; /*<  当前数据长度.*/
 int8_t payload[512]; /*<  payload .*/
}) ttalink_passthrough_variable_t;

#define TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN 514
#define TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN 514
#define TTALINK_MSG_ID_2244_LEN 514
#define TTALINK_MSG_ID_2244_MIN_LEN 514

#define TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC 4
#define TTALINK_MSG_ID_2244_CRC 4

#define TTALINK_MSG_PASSTHROUGH_VARIABLE_FIELD_PAYLOAD_LEN 512

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PASSTHROUGH_VARIABLE { \
    2244, \
    "PASSTHROUGH_VARIABLE", \
    2, \
    {  { "length", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_passthrough_variable_t, length) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 512, 2, offsetof(ttalink_passthrough_variable_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PASSTHROUGH_VARIABLE { \
    "PASSTHROUGH_VARIABLE", \
    2, \
    {  { "length", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_passthrough_variable_t, length) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 512, 2, offsetof(ttalink_passthrough_variable_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_passthrough_variable_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t length, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN];
    _tta_put_uint16_t(buf, 0, length);
    _tta_put_int8_t_array(buf, 2, payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN);
#else
    ttalink_passthrough_variable_t packet;
    packet.length = length;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASSTHROUGH_VARIABLE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC, nocrc);
}

/**
 * @brief Pack a passthrough_variable message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param length  当前数据长度.
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_variable_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t length, const int8_t *payload)
{
    return _ttalink_passthrough_variable_pack(dst_addr, src_addr, msg,  length, payload, false);
}

/**
 * @brief Pack a passthrough_variable message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param length  当前数据长度.
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_variable_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t length, const int8_t *payload)
{
    return _ttalink_passthrough_variable_pack(dst_addr, src_addr, msg,  length, payload, true);
}


static inline uint16_t _ttalink_passthrough_variable_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t length,const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN];
    _tta_put_uint16_t(buf, 0, length);
    _tta_put_int8_t_array(buf, 2, payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN);
#else
    ttalink_passthrough_variable_t packet;
    packet.length = length;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASSTHROUGH_VARIABLE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC, nocrc);
}

/**
 * @brief Pack a passthrough_variable message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param length  当前数据长度.
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_variable_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t length,const int8_t *payload)
{
    return _ttalink_passthrough_variable_pack_chan(dst_addr, src_addr, chan, msg,  length, payload, false);
}

/**
 * @brief Pack a passthrough_variable message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param length  当前数据长度.
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_variable_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t length,const int8_t *payload)
{
    return _ttalink_passthrough_variable_pack_chan(dst_addr, src_addr, chan, msg,  length, payload, true);
}


static inline uint16_t _ttalink_passthrough_variable_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_passthrough_variable_t* passthrough_variable, bool nocrc)
{
    if(nocrc){
        return ttalink_passthrough_variable_pack_nocrc(dst_addr, src_addr, msg, passthrough_variable->length, passthrough_variable->payload);
    }else{
        return ttalink_passthrough_variable_pack(dst_addr, src_addr, msg, passthrough_variable->length, passthrough_variable->payload);
    }
    
}

/**
 * @brief Encode a passthrough_variable struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param passthrough_variable C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_variable_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_passthrough_variable_t* passthrough_variable)
{
    return _ttalink_passthrough_variable_encode(dst_addr, src_addr, msg, passthrough_variable, false);
}

/**
 * @brief Encode a passthrough_variable struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param passthrough_variable C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_variable_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_passthrough_variable_t* passthrough_variable)
{
    return _ttalink_passthrough_variable_encode(dst_addr, src_addr, msg, passthrough_variable, true);
}


static inline uint16_t _ttalink_passthrough_variable_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_passthrough_variable_t* passthrough_variable, bool nocrc)
{
    if(nocrc){
        return ttalink_passthrough_variable_pack_chan_nocrc(dst_addr, src_addr, chan, msg, passthrough_variable->length, passthrough_variable->payload);
    }else{
        return ttalink_passthrough_variable_pack_chan(dst_addr, src_addr, chan, msg, passthrough_variable->length, passthrough_variable->payload);
    }
}

/**
 * @brief Encode a passthrough_variable struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param passthrough_variable C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_variable_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_passthrough_variable_t* passthrough_variable)
{
    return _ttalink_passthrough_variable_encode_chan(dst_addr, src_addr, chan, msg, passthrough_variable, false);
}

/**
 * @brief Encode a passthrough_variable struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param passthrough_variable C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_variable_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_passthrough_variable_t* passthrough_variable)
{
    return _ttalink_passthrough_variable_encode_chan(dst_addr, src_addr, chan, msg, passthrough_variable, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_passthrough_variable_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t length, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN];
    _tta_put_uint16_t(buf, 0, length);
    _tta_put_int8_t_array(buf, 2, payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE, buf, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC, nocrc);
#else
    ttalink_passthrough_variable_t packet;
    packet.length = length;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE, (const char *)&packet, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC, nocrc);
#endif
}

/**
 * @brief Send a passthrough_variable message
 * @param chan TTAlink channel to send the message
 *
 * @param length  当前数据长度.
 * @param payload  payload .
 */
static inline void ttalink_passthrough_variable_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t length, const int8_t *payload)
{
    _ttalink_passthrough_variable_send(dst_addr, src_addr, chan, length, payload, false);
}

/**
 * @brief Send a passthrough_variable message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param length  当前数据长度.
 * @param payload  payload .
 */
static inline void ttalink_passthrough_variable_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t length, const int8_t *payload)
{
    _ttalink_passthrough_variable_send(dst_addr, src_addr, chan, length, payload, true);
}


static inline void _ttalink_passthrough_variable_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_passthrough_variable_t* passthrough_variable, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_passthrough_variable_send_nocrc(dst_addr, src_addr, chan, passthrough_variable->length, passthrough_variable->payload);
    }else{
        ttalink_passthrough_variable_send(dst_addr, src_addr, chan, passthrough_variable->length, passthrough_variable->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE, (const char *)passthrough_variable, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC, nocrc);
#endif
}

/**
 * @brief Send a passthrough_variable message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_passthrough_variable_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_passthrough_variable_t* passthrough_variable)
{
    _ttalink_passthrough_variable_send_struct(dst_addr, src_addr, chan, passthrough_variable, false);
}

/**
 * @brief Send a passthrough_variable message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_passthrough_variable_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_passthrough_variable_t* passthrough_variable)
{
    _ttalink_passthrough_variable_send_struct(dst_addr, src_addr, chan, passthrough_variable, true);
}

#if TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_passthrough_variable_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t length, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, length);
    _tta_put_int8_t_array(buf, 2, payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE, buf, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC, nocrc);
#else
    ttalink_passthrough_variable_t *packet = (ttalink_passthrough_variable_t *)msgbuf;
    packet->length = length;
    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE, (const char *)packet, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_passthrough_variable_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t length, const int8_t *payload)
{
    _ttalink_passthrough_variable_send_buf(dst_addr, src_addr, msgbuf, chan, length, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_passthrough_variable_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t length, const int8_t *payload)
{
    _ttalink_passthrough_variable_send_buf(dst_addr, src_addr, msgbuf, chan, length, payload, true);
}
#endif

#endif

// MESSAGE PASSTHROUGH_VARIABLE UNPACKING


/**
 * @brief Get field length from passthrough_variable message
 *
 * @return  当前数据长度.
 */
static inline uint16_t ttalink_passthrough_variable_get_length(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field payload from passthrough_variable message
 *
 * @return  payload .
 */
static inline uint16_t ttalink_passthrough_variable_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 512,  2);
}

/**
 * @brief Decode a passthrough_variable message into a struct
 *
 * @param msg The message to decode
 * @param passthrough_variable C-struct to decode the message contents into
 */
static inline void ttalink_passthrough_variable_decode(const ttalink_message_t* msg, ttalink_passthrough_variable_t* passthrough_variable)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    passthrough_variable->length = ttalink_passthrough_variable_get_length(msg);
    ttalink_passthrough_variable_get_payload(msg, passthrough_variable->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN? msg->len : TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN;
        memset(passthrough_variable, 0, TTALINK_MSG_ID_PASSTHROUGH_VARIABLE_LEN);
    memcpy(passthrough_variable, _TTA_PAYLOAD(msg), len);
#endif
}
