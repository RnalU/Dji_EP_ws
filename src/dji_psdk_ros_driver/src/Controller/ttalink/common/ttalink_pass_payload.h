#pragma once
// MESSAGE PASS_PAYLOAD PACKING

#define TTALINK_MSG_ID_PASS_PAYLOAD 2249

TTAPACKED(
typedef struct __ttalink_pass_payload_t {
 uint16_t size; /*<  .*/
 int8_t payload[255]; /*<  .*/
}) ttalink_pass_payload_t;

#define TTALINK_MSG_ID_PASS_PAYLOAD_LEN 257
#define TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN 257
#define TTALINK_MSG_ID_2249_LEN 257
#define TTALINK_MSG_ID_2249_MIN_LEN 257

#define TTALINK_MSG_ID_PASS_PAYLOAD_CRC 155
#define TTALINK_MSG_ID_2249_CRC 155

#define TTALINK_MSG_PASS_PAYLOAD_FIELD_PAYLOAD_LEN 255

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PASS_PAYLOAD { \
    2249, \
    "PASS_PAYLOAD", \
    2, \
    {  { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pass_payload_t, size) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 255, 2, offsetof(ttalink_pass_payload_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PASS_PAYLOAD { \
    "PASS_PAYLOAD", \
    2, \
    {  { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pass_payload_t, size) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 255, 2, offsetof(ttalink_pass_payload_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pass_payload_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_PAYLOAD_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_int8_t_array(buf, 2, payload, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_PAYLOAD_LEN);
#else
    ttalink_pass_payload_t packet;
    packet.size = size;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_PAYLOAD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_PAYLOAD;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_CRC, nocrc);
}

/**
 * @brief Pack a pass_payload message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_payload_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, const int8_t *payload)
{
    return _ttalink_pass_payload_pack(dst_addr, src_addr, msg,  size, payload, false);
}

/**
 * @brief Pack a pass_payload message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_payload_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, const int8_t *payload)
{
    return _ttalink_pass_payload_pack(dst_addr, src_addr, msg,  size, payload, true);
}


static inline uint16_t _ttalink_pass_payload_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_PAYLOAD_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_int8_t_array(buf, 2, payload, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_PAYLOAD_LEN);
#else
    ttalink_pass_payload_t packet;
    packet.size = size;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_PAYLOAD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_PAYLOAD;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_CRC, nocrc);
}

/**
 * @brief Pack a pass_payload message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_payload_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,const int8_t *payload)
{
    return _ttalink_pass_payload_pack_chan(dst_addr, src_addr, chan, msg,  size, payload, false);
}

/**
 * @brief Pack a pass_payload message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  .
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_payload_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,const int8_t *payload)
{
    return _ttalink_pass_payload_pack_chan(dst_addr, src_addr, chan, msg,  size, payload, true);
}


static inline uint16_t _ttalink_pass_payload_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_payload_t* pass_payload, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_payload_pack_nocrc(dst_addr, src_addr, msg, pass_payload->size, pass_payload->payload);
    }else{
        return ttalink_pass_payload_pack(dst_addr, src_addr, msg, pass_payload->size, pass_payload->payload);
    }
    
}

/**
 * @brief Encode a pass_payload struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_payload_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_payload_t* pass_payload)
{
    return _ttalink_pass_payload_encode(dst_addr, src_addr, msg, pass_payload, false);
}

/**
 * @brief Encode a pass_payload struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_payload_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_payload_t* pass_payload)
{
    return _ttalink_pass_payload_encode(dst_addr, src_addr, msg, pass_payload, true);
}


static inline uint16_t _ttalink_pass_payload_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_payload_t* pass_payload, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_payload_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pass_payload->size, pass_payload->payload);
    }else{
        return ttalink_pass_payload_pack_chan(dst_addr, src_addr, chan, msg, pass_payload->size, pass_payload->payload);
    }
}

/**
 * @brief Encode a pass_payload struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_payload_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_payload_t* pass_payload)
{
    return _ttalink_pass_payload_encode_chan(dst_addr, src_addr, chan, msg, pass_payload, false);
}

/**
 * @brief Encode a pass_payload struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_payload_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_payload_t* pass_payload)
{
    return _ttalink_pass_payload_encode_chan(dst_addr, src_addr, chan, msg, pass_payload, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pass_payload_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_PAYLOAD_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_int8_t_array(buf, 2, payload, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_PAYLOAD, buf, TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_CRC, nocrc);
#else
    ttalink_pass_payload_t packet;
    packet.size = size;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_PAYLOAD, (const char *)&packet, TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_payload message
 * @param chan TTAlink channel to send the message
 *
 * @param size  .
 * @param payload  .
 */
static inline void ttalink_pass_payload_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, const int8_t *payload)
{
    _ttalink_pass_payload_send(dst_addr, src_addr, chan, size, payload, false);
}

/**
 * @brief Send a pass_payload message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param size  .
 * @param payload  .
 */
static inline void ttalink_pass_payload_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, const int8_t *payload)
{
    _ttalink_pass_payload_send(dst_addr, src_addr, chan, size, payload, true);
}


static inline void _ttalink_pass_payload_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_payload_t* pass_payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pass_payload_send_nocrc(dst_addr, src_addr, chan, pass_payload->size, pass_payload->payload);
    }else{
        ttalink_pass_payload_send(dst_addr, src_addr, chan, pass_payload->size, pass_payload->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_PAYLOAD, (const char *)pass_payload, TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_payload message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_payload_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_payload_t* pass_payload)
{
    _ttalink_pass_payload_send_struct(dst_addr, src_addr, chan, pass_payload, false);
}

/**
 * @brief Send a pass_payload message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_payload_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_payload_t* pass_payload)
{
    _ttalink_pass_payload_send_struct(dst_addr, src_addr, chan, pass_payload, true);
}

#if TTALINK_MSG_ID_PASS_PAYLOAD_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pass_payload_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_int8_t_array(buf, 2, payload, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_PAYLOAD, buf, TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_CRC, nocrc);
#else
    ttalink_pass_payload_t *packet = (ttalink_pass_payload_t *)msgbuf;
    packet->size = size;
    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_PAYLOAD, (const char *)packet, TTALINK_MSG_ID_PASS_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_LEN, TTALINK_MSG_ID_PASS_PAYLOAD_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pass_payload_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, const int8_t *payload)
{
    _ttalink_pass_payload_send_buf(dst_addr, src_addr, msgbuf, chan, size, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pass_payload_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, const int8_t *payload)
{
    _ttalink_pass_payload_send_buf(dst_addr, src_addr, msgbuf, chan, size, payload, true);
}
#endif

#endif

// MESSAGE PASS_PAYLOAD UNPACKING


/**
 * @brief Get field size from pass_payload message
 *
 * @return  .
 */
static inline uint16_t ttalink_pass_payload_get_size(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field payload from pass_payload message
 *
 * @return  .
 */
static inline uint16_t ttalink_pass_payload_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 255,  2);
}

/**
 * @brief Decode a pass_payload message into a struct
 *
 * @param msg The message to decode
 * @param pass_payload C-struct to decode the message contents into
 */
static inline void ttalink_pass_payload_decode(const ttalink_message_t* msg, ttalink_pass_payload_t* pass_payload)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pass_payload->size = ttalink_pass_payload_get_size(msg);
    ttalink_pass_payload_get_payload(msg, pass_payload->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PASS_PAYLOAD_LEN? msg->len : TTALINK_MSG_ID_PASS_PAYLOAD_LEN;
        memset(pass_payload, 0, TTALINK_MSG_ID_PASS_PAYLOAD_LEN);
    memcpy(pass_payload, _TTA_PAYLOAD(msg), len);
#endif
}
