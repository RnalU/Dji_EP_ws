#pragma once
// MESSAGE POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT PACKING

#define TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT 11538

TTAPACKED(
typedef struct __ttalink_pod_1538_pass_payload_to_development_t {
 uint8_t data[255]; /*< .*/
}) ttalink_pod_1538_pass_payload_to_development_t;

#define TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN 255
#define TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN 255
#define TTALINK_MSG_ID_11538_LEN 255
#define TTALINK_MSG_ID_11538_MIN_LEN 255

#define TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC 105
#define TTALINK_MSG_ID_11538_CRC 105

#define TTALINK_MSG_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_FIELD_DATA_LEN 255

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT { \
    11538, \
    "POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT", \
    1, \
    {  { "data", NULL, TTALINK_TYPE_UINT8_T, 255, 0, offsetof(ttalink_pod_1538_pass_payload_to_development_t, data) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT { \
    "POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT", \
    1, \
    {  { "data", NULL, TTALINK_TYPE_UINT8_T, 255, 0, offsetof(ttalink_pod_1538_pass_payload_to_development_t, data) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1538_pass_payload_to_development_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN];

    _tta_put_uint8_t_array(buf, 0, data, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN);
#else
    ttalink_pod_1538_pass_payload_to_development_t packet;

    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC, nocrc);
}

/**
 * @brief Pack a pod_1538_pass_payload_to_development message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *data)
{
    return _ttalink_pod_1538_pass_payload_to_development_pack(dst_addr, src_addr, msg,  data, false);
}

/**
 * @brief Pack a pod_1538_pass_payload_to_development message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *data)
{
    return _ttalink_pod_1538_pass_payload_to_development_pack(dst_addr, src_addr, msg,  data, true);
}


static inline uint16_t _ttalink_pod_1538_pass_payload_to_development_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN];

    _tta_put_uint8_t_array(buf, 0, data, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN);
#else
    ttalink_pod_1538_pass_payload_to_development_t packet;

    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC, nocrc);
}

/**
 * @brief Pack a pod_1538_pass_payload_to_development message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *data)
{
    return _ttalink_pod_1538_pass_payload_to_development_pack_chan(dst_addr, src_addr, chan, msg,  data, false);
}

/**
 * @brief Pack a pod_1538_pass_payload_to_development message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *data)
{
    return _ttalink_pod_1538_pass_payload_to_development_pack_chan(dst_addr, src_addr, chan, msg,  data, true);
}


static inline uint16_t _ttalink_pod_1538_pass_payload_to_development_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1538_pass_payload_to_development_pack_nocrc(dst_addr, src_addr, msg, pod_1538_pass_payload_to_development->data);
    }else{
        return ttalink_pod_1538_pass_payload_to_development_pack(dst_addr, src_addr, msg, pod_1538_pass_payload_to_development->data);
    }
    
}

/**
 * @brief Encode a pod_1538_pass_payload_to_development struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1538_pass_payload_to_development C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development)
{
    return _ttalink_pod_1538_pass_payload_to_development_encode(dst_addr, src_addr, msg, pod_1538_pass_payload_to_development, false);
}

/**
 * @brief Encode a pod_1538_pass_payload_to_development struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1538_pass_payload_to_development C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development)
{
    return _ttalink_pod_1538_pass_payload_to_development_encode(dst_addr, src_addr, msg, pod_1538_pass_payload_to_development, true);
}


static inline uint16_t _ttalink_pod_1538_pass_payload_to_development_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1538_pass_payload_to_development_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1538_pass_payload_to_development->data);
    }else{
        return ttalink_pod_1538_pass_payload_to_development_pack_chan(dst_addr, src_addr, chan, msg, pod_1538_pass_payload_to_development->data);
    }
}

/**
 * @brief Encode a pod_1538_pass_payload_to_development struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1538_pass_payload_to_development C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development)
{
    return _ttalink_pod_1538_pass_payload_to_development_encode_chan(dst_addr, src_addr, chan, msg, pod_1538_pass_payload_to_development, false);
}

/**
 * @brief Encode a pod_1538_pass_payload_to_development struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1538_pass_payload_to_development C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development)
{
    return _ttalink_pod_1538_pass_payload_to_development_encode_chan(dst_addr, src_addr, chan, msg, pod_1538_pass_payload_to_development, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1538_pass_payload_to_development_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN];

    _tta_put_uint8_t_array(buf, 0, data, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT, buf, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC, nocrc);
#else
    ttalink_pod_1538_pass_payload_to_development_t packet;

    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT, (const char *)&packet, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1538_pass_payload_to_development message
 * @param chan TTAlink channel to send the message
 *
 * @param data .
 */
static inline void ttalink_pod_1538_pass_payload_to_development_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *data)
{
    _ttalink_pod_1538_pass_payload_to_development_send(dst_addr, src_addr, chan, data, false);
}

/**
 * @brief Send a pod_1538_pass_payload_to_development message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param data .
 */
static inline void ttalink_pod_1538_pass_payload_to_development_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *data)
{
    _ttalink_pod_1538_pass_payload_to_development_send(dst_addr, src_addr, chan, data, true);
}


static inline void _ttalink_pod_1538_pass_payload_to_development_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1538_pass_payload_to_development_send_nocrc(dst_addr, src_addr, chan, pod_1538_pass_payload_to_development->data);
    }else{
        ttalink_pod_1538_pass_payload_to_development_send(dst_addr, src_addr, chan, pod_1538_pass_payload_to_development->data);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT, (const char *)pod_1538_pass_payload_to_development, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1538_pass_payload_to_development message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1538_pass_payload_to_development_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development)
{
    _ttalink_pod_1538_pass_payload_to_development_send_struct(dst_addr, src_addr, chan, pod_1538_pass_payload_to_development, false);
}

/**
 * @brief Send a pod_1538_pass_payload_to_development message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1538_pass_payload_to_development_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development)
{
    _ttalink_pod_1538_pass_payload_to_development_send_struct(dst_addr, src_addr, chan, pod_1538_pass_payload_to_development, true);
}

#if TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1538_pass_payload_to_development_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_uint8_t_array(buf, 0, data, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT, buf, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC, nocrc);
#else
    ttalink_pod_1538_pass_payload_to_development_t *packet = (ttalink_pod_1538_pass_payload_to_development_t *)msgbuf;

    tta_array_memcpy(packet->data, data, sizeof(uint8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT, (const char *)packet, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_MIN_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1538_pass_payload_to_development_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *data)
{
    _ttalink_pod_1538_pass_payload_to_development_send_buf(dst_addr, src_addr, msgbuf, chan, data, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1538_pass_payload_to_development_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *data)
{
    _ttalink_pod_1538_pass_payload_to_development_send_buf(dst_addr, src_addr, msgbuf, chan, data, true);
}
#endif

#endif

// MESSAGE POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT UNPACKING


/**
 * @brief Get field data from pod_1538_pass_payload_to_development message
 *
 * @return .
 */
static inline uint16_t ttalink_pod_1538_pass_payload_to_development_get_data(const ttalink_message_t* msg, uint8_t *data)
{
    return _TTA_RETURN_uint8_t_array(msg, data, 255,  0);
}

/**
 * @brief Decode a pod_1538_pass_payload_to_development message into a struct
 *
 * @param msg The message to decode
 * @param pod_1538_pass_payload_to_development C-struct to decode the message contents into
 */
static inline void ttalink_pod_1538_pass_payload_to_development_decode(const ttalink_message_t* msg, ttalink_pod_1538_pass_payload_to_development_t* pod_1538_pass_payload_to_development)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_pod_1538_pass_payload_to_development_get_data(msg, pod_1538_pass_payload_to_development->data);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN? msg->len : TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN;
        memset(pod_1538_pass_payload_to_development, 0, TTALINK_MSG_ID_POD_1538_PASS_PAYLOAD_TO_DEVELOPMENT_LEN);
    memcpy(pod_1538_pass_payload_to_development, _TTA_PAYLOAD(msg), len);
#endif
}
