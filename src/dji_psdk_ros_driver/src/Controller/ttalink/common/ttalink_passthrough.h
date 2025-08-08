#pragma once
// MESSAGE PASSTHROUGH PACKING

#define TTALINK_MSG_ID_PASSTHROUGH 2250

TTAPACKED(
typedef struct __ttalink_passthrough_t {
 int8_t payload[255]; /*<  .*/
}) ttalink_passthrough_t;

#define TTALINK_MSG_ID_PASSTHROUGH_LEN 255
#define TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN 255
#define TTALINK_MSG_ID_2250_LEN 255
#define TTALINK_MSG_ID_2250_MIN_LEN 255

#define TTALINK_MSG_ID_PASSTHROUGH_CRC 38
#define TTALINK_MSG_ID_2250_CRC 38

#define TTALINK_MSG_PASSTHROUGH_FIELD_PAYLOAD_LEN 255

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PASSTHROUGH { \
    2250, \
    "PASSTHROUGH", \
    1, \
    {  { "payload", NULL, TTALINK_TYPE_INT8_T, 255, 0, offsetof(ttalink_passthrough_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PASSTHROUGH { \
    "PASSTHROUGH", \
    1, \
    {  { "payload", NULL, TTALINK_TYPE_INT8_T, 255, 0, offsetof(ttalink_passthrough_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_passthrough_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASSTHROUGH_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASSTHROUGH_LEN);
#else
    ttalink_passthrough_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASSTHROUGH_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASSTHROUGH;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_LEN, TTALINK_MSG_ID_PASSTHROUGH_CRC, nocrc);
}

/**
 * @brief Pack a passthrough message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload)
{
    return _ttalink_passthrough_pack(dst_addr, src_addr, msg,  payload, false);
}

/**
 * @brief Pack a passthrough message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload)
{
    return _ttalink_passthrough_pack(dst_addr, src_addr, msg,  payload, true);
}


static inline uint16_t _ttalink_passthrough_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASSTHROUGH_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASSTHROUGH_LEN);
#else
    ttalink_passthrough_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASSTHROUGH_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASSTHROUGH;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_LEN, TTALINK_MSG_ID_PASSTHROUGH_CRC, nocrc);
}

/**
 * @brief Pack a passthrough message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload)
{
    return _ttalink_passthrough_pack_chan(dst_addr, src_addr, chan, msg,  payload, false);
}

/**
 * @brief Pack a passthrough message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_passthrough_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload)
{
    return _ttalink_passthrough_pack_chan(dst_addr, src_addr, chan, msg,  payload, true);
}


static inline uint16_t _ttalink_passthrough_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_passthrough_t* passthrough, bool nocrc)
{
    if(nocrc){
        return ttalink_passthrough_pack_nocrc(dst_addr, src_addr, msg, passthrough->payload);
    }else{
        return ttalink_passthrough_pack(dst_addr, src_addr, msg, passthrough->payload);
    }
    
}

/**
 * @brief Encode a passthrough struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param passthrough C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_passthrough_t* passthrough)
{
    return _ttalink_passthrough_encode(dst_addr, src_addr, msg, passthrough, false);
}

/**
 * @brief Encode a passthrough struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param passthrough C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_passthrough_t* passthrough)
{
    return _ttalink_passthrough_encode(dst_addr, src_addr, msg, passthrough, true);
}


static inline uint16_t _ttalink_passthrough_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_passthrough_t* passthrough, bool nocrc)
{
    if(nocrc){
        return ttalink_passthrough_pack_chan_nocrc(dst_addr, src_addr, chan, msg, passthrough->payload);
    }else{
        return ttalink_passthrough_pack_chan(dst_addr, src_addr, chan, msg, passthrough->payload);
    }
}

/**
 * @brief Encode a passthrough struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param passthrough C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_passthrough_t* passthrough)
{
    return _ttalink_passthrough_encode_chan(dst_addr, src_addr, chan, msg, passthrough, false);
}

/**
 * @brief Encode a passthrough struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param passthrough C-struct to read the message contents from
 */
static inline uint16_t ttalink_passthrough_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_passthrough_t* passthrough)
{
    return _ttalink_passthrough_encode_chan(dst_addr, src_addr, chan, msg, passthrough, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_passthrough_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASSTHROUGH_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH, buf, TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_LEN, TTALINK_MSG_ID_PASSTHROUGH_CRC, nocrc);
#else
    ttalink_passthrough_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH, (const char *)&packet, TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_LEN, TTALINK_MSG_ID_PASSTHROUGH_CRC, nocrc);
#endif
}

/**
 * @brief Send a passthrough message
 * @param chan TTAlink channel to send the message
 *
 * @param payload  .
 */
static inline void ttalink_passthrough_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload)
{
    _ttalink_passthrough_send(dst_addr, src_addr, chan, payload, false);
}

/**
 * @brief Send a passthrough message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param payload  .
 */
static inline void ttalink_passthrough_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload)
{
    _ttalink_passthrough_send(dst_addr, src_addr, chan, payload, true);
}


static inline void _ttalink_passthrough_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_passthrough_t* passthrough, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_passthrough_send_nocrc(dst_addr, src_addr, chan, passthrough->payload);
    }else{
        ttalink_passthrough_send(dst_addr, src_addr, chan, passthrough->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH, (const char *)passthrough, TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_LEN, TTALINK_MSG_ID_PASSTHROUGH_CRC, nocrc);
#endif
}

/**
 * @brief Send a passthrough message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_passthrough_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_passthrough_t* passthrough)
{
    _ttalink_passthrough_send_struct(dst_addr, src_addr, chan, passthrough, false);
}

/**
 * @brief Send a passthrough message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_passthrough_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_passthrough_t* passthrough)
{
    _ttalink_passthrough_send_struct(dst_addr, src_addr, chan, passthrough, true);
}

#if TTALINK_MSG_ID_PASSTHROUGH_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_passthrough_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_int8_t_array(buf, 0, payload, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH, buf, TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_LEN, TTALINK_MSG_ID_PASSTHROUGH_CRC, nocrc);
#else
    ttalink_passthrough_t *packet = (ttalink_passthrough_t *)msgbuf;

    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASSTHROUGH, (const char *)packet, TTALINK_MSG_ID_PASSTHROUGH_MIN_LEN, TTALINK_MSG_ID_PASSTHROUGH_LEN, TTALINK_MSG_ID_PASSTHROUGH_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_passthrough_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload)
{
    _ttalink_passthrough_send_buf(dst_addr, src_addr, msgbuf, chan, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_passthrough_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload)
{
    _ttalink_passthrough_send_buf(dst_addr, src_addr, msgbuf, chan, payload, true);
}
#endif

#endif

// MESSAGE PASSTHROUGH UNPACKING


/**
 * @brief Get field payload from passthrough message
 *
 * @return  .
 */
static inline uint16_t ttalink_passthrough_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 255,  0);
}

/**
 * @brief Decode a passthrough message into a struct
 *
 * @param msg The message to decode
 * @param passthrough C-struct to decode the message contents into
 */
static inline void ttalink_passthrough_decode(const ttalink_message_t* msg, ttalink_passthrough_t* passthrough)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_passthrough_get_payload(msg, passthrough->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PASSTHROUGH_LEN? msg->len : TTALINK_MSG_ID_PASSTHROUGH_LEN;
        memset(passthrough, 0, TTALINK_MSG_ID_PASSTHROUGH_LEN);
    memcpy(passthrough, _TTA_PAYLOAD(msg), len);
#endif
}
