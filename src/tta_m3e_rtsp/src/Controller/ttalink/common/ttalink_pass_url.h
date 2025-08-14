#pragma once
// MESSAGE PASS_URL PACKING

#define TTALINK_MSG_ID_PASS_URL 2257

TTAPACKED(
typedef struct __ttalink_pass_url_t {
 int8_t payload[512]; /*<  .*/
}) ttalink_pass_url_t;

#define TTALINK_MSG_ID_PASS_URL_LEN 512
#define TTALINK_MSG_ID_PASS_URL_MIN_LEN 512
#define TTALINK_MSG_ID_2257_LEN 512
#define TTALINK_MSG_ID_2257_MIN_LEN 512

#define TTALINK_MSG_ID_PASS_URL_CRC 255
#define TTALINK_MSG_ID_2257_CRC 255

#define TTALINK_MSG_PASS_URL_FIELD_PAYLOAD_LEN 512

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PASS_URL { \
    2257, \
    "PASS_URL", \
    1, \
    {  { "payload", NULL, TTALINK_TYPE_INT8_T, 512, 0, offsetof(ttalink_pass_url_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PASS_URL { \
    "PASS_URL", \
    1, \
    {  { "payload", NULL, TTALINK_TYPE_INT8_T, 512, 0, offsetof(ttalink_pass_url_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pass_url_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_URL_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_URL_LEN);
#else
    ttalink_pass_url_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_URL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_URL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PASS_URL_MIN_LEN, TTALINK_MSG_ID_PASS_URL_LEN, TTALINK_MSG_ID_PASS_URL_CRC, nocrc);
}

/**
 * @brief Pack a pass_url message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_url_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload)
{
    return _ttalink_pass_url_pack(dst_addr, src_addr, msg,  payload, false);
}

/**
 * @brief Pack a pass_url message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_url_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const int8_t *payload)
{
    return _ttalink_pass_url_pack(dst_addr, src_addr, msg,  payload, true);
}


static inline uint16_t _ttalink_pass_url_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_URL_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_URL_LEN);
#else
    ttalink_pass_url_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_URL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_URL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_URL_MIN_LEN, TTALINK_MSG_ID_PASS_URL_LEN, TTALINK_MSG_ID_PASS_URL_CRC, nocrc);
}

/**
 * @brief Pack a pass_url message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_url_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload)
{
    return _ttalink_pass_url_pack_chan(dst_addr, src_addr, chan, msg,  payload, false);
}

/**
 * @brief Pack a pass_url message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_url_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const int8_t *payload)
{
    return _ttalink_pass_url_pack_chan(dst_addr, src_addr, chan, msg,  payload, true);
}


static inline uint16_t _ttalink_pass_url_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_url_t* pass_url, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_url_pack_nocrc(dst_addr, src_addr, msg, pass_url->payload);
    }else{
        return ttalink_pass_url_pack(dst_addr, src_addr, msg, pass_url->payload);
    }
    
}

/**
 * @brief Encode a pass_url struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_url C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_url_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_url_t* pass_url)
{
    return _ttalink_pass_url_encode(dst_addr, src_addr, msg, pass_url, false);
}

/**
 * @brief Encode a pass_url struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_url C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_url_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_url_t* pass_url)
{
    return _ttalink_pass_url_encode(dst_addr, src_addr, msg, pass_url, true);
}


static inline uint16_t _ttalink_pass_url_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_url_t* pass_url, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_url_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pass_url->payload);
    }else{
        return ttalink_pass_url_pack_chan(dst_addr, src_addr, chan, msg, pass_url->payload);
    }
}

/**
 * @brief Encode a pass_url struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_url C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_url_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_url_t* pass_url)
{
    return _ttalink_pass_url_encode_chan(dst_addr, src_addr, chan, msg, pass_url, false);
}

/**
 * @brief Encode a pass_url struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_url C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_url_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_url_t* pass_url)
{
    return _ttalink_pass_url_encode_chan(dst_addr, src_addr, chan, msg, pass_url, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pass_url_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_URL_LEN];

    _tta_put_int8_t_array(buf, 0, payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_URL, buf, TTALINK_MSG_ID_PASS_URL_MIN_LEN, TTALINK_MSG_ID_PASS_URL_LEN, TTALINK_MSG_ID_PASS_URL_CRC, nocrc);
#else
    ttalink_pass_url_t packet;

    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_URL, (const char *)&packet, TTALINK_MSG_ID_PASS_URL_MIN_LEN, TTALINK_MSG_ID_PASS_URL_LEN, TTALINK_MSG_ID_PASS_URL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_url message
 * @param chan TTAlink channel to send the message
 *
 * @param payload  .
 */
static inline void ttalink_pass_url_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload)
{
    _ttalink_pass_url_send(dst_addr, src_addr, chan, payload, false);
}

/**
 * @brief Send a pass_url message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param payload  .
 */
static inline void ttalink_pass_url_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const int8_t *payload)
{
    _ttalink_pass_url_send(dst_addr, src_addr, chan, payload, true);
}


static inline void _ttalink_pass_url_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_url_t* pass_url, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pass_url_send_nocrc(dst_addr, src_addr, chan, pass_url->payload);
    }else{
        ttalink_pass_url_send(dst_addr, src_addr, chan, pass_url->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_URL, (const char *)pass_url, TTALINK_MSG_ID_PASS_URL_MIN_LEN, TTALINK_MSG_ID_PASS_URL_LEN, TTALINK_MSG_ID_PASS_URL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_url message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_url_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_url_t* pass_url)
{
    _ttalink_pass_url_send_struct(dst_addr, src_addr, chan, pass_url, false);
}

/**
 * @brief Send a pass_url message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_url_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_url_t* pass_url)
{
    _ttalink_pass_url_send_struct(dst_addr, src_addr, chan, pass_url, true);
}

#if TTALINK_MSG_ID_PASS_URL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pass_url_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_int8_t_array(buf, 0, payload, 512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_URL, buf, TTALINK_MSG_ID_PASS_URL_MIN_LEN, TTALINK_MSG_ID_PASS_URL_LEN, TTALINK_MSG_ID_PASS_URL_CRC, nocrc);
#else
    ttalink_pass_url_t *packet = (ttalink_pass_url_t *)msgbuf;

    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*512);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_URL, (const char *)packet, TTALINK_MSG_ID_PASS_URL_MIN_LEN, TTALINK_MSG_ID_PASS_URL_LEN, TTALINK_MSG_ID_PASS_URL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pass_url_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload)
{
    _ttalink_pass_url_send_buf(dst_addr, src_addr, msgbuf, chan, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pass_url_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const int8_t *payload)
{
    _ttalink_pass_url_send_buf(dst_addr, src_addr, msgbuf, chan, payload, true);
}
#endif

#endif

// MESSAGE PASS_URL UNPACKING


/**
 * @brief Get field payload from pass_url message
 *
 * @return  .
 */
static inline uint16_t ttalink_pass_url_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 512,  0);
}

/**
 * @brief Decode a pass_url message into a struct
 *
 * @param msg The message to decode
 * @param pass_url C-struct to decode the message contents into
 */
static inline void ttalink_pass_url_decode(const ttalink_message_t* msg, ttalink_pass_url_t* pass_url)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_pass_url_get_payload(msg, pass_url->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PASS_URL_LEN? msg->len : TTALINK_MSG_ID_PASS_URL_LEN;
        memset(pass_url, 0, TTALINK_MSG_ID_PASS_URL_LEN);
    memcpy(pass_url, _TTA_PAYLOAD(msg), len);
#endif
}
