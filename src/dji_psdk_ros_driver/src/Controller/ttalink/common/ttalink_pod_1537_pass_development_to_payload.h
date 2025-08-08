#pragma once
// MESSAGE POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD PACKING

#define TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD 11537

TTAPACKED(
typedef struct __ttalink_pod_1537_pass_development_to_payload_t {
 uint8_t data[255]; /*< .*/
}) ttalink_pod_1537_pass_development_to_payload_t;

#define TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN 255
#define TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN 255
#define TTALINK_MSG_ID_11537_LEN 255
#define TTALINK_MSG_ID_11537_MIN_LEN 255

#define TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC 228
#define TTALINK_MSG_ID_11537_CRC 228

#define TTALINK_MSG_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_FIELD_DATA_LEN 255

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD { \
    11537, \
    "POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD", \
    1, \
    {  { "data", NULL, TTALINK_TYPE_UINT8_T, 255, 0, offsetof(ttalink_pod_1537_pass_development_to_payload_t, data) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD { \
    "POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD", \
    1, \
    {  { "data", NULL, TTALINK_TYPE_UINT8_T, 255, 0, offsetof(ttalink_pod_1537_pass_development_to_payload_t, data) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1537_pass_development_to_payload_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN];

    _tta_put_uint8_t_array(buf, 0, data, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN);
#else
    ttalink_pod_1537_pass_development_to_payload_t packet;

    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC, nocrc);
}

/**
 * @brief Pack a pod_1537_pass_development_to_payload message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *data)
{
    return _ttalink_pod_1537_pass_development_to_payload_pack(dst_addr, src_addr, msg,  data, false);
}

/**
 * @brief Pack a pod_1537_pass_development_to_payload message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const uint8_t *data)
{
    return _ttalink_pod_1537_pass_development_to_payload_pack(dst_addr, src_addr, msg,  data, true);
}


static inline uint16_t _ttalink_pod_1537_pass_development_to_payload_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN];

    _tta_put_uint8_t_array(buf, 0, data, 255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN);
#else
    ttalink_pod_1537_pass_development_to_payload_t packet;

    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*255);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC, nocrc);
}

/**
 * @brief Pack a pod_1537_pass_development_to_payload message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *data)
{
    return _ttalink_pod_1537_pass_development_to_payload_pack_chan(dst_addr, src_addr, chan, msg,  data, false);
}

/**
 * @brief Pack a pod_1537_pass_development_to_payload message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param data .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const uint8_t *data)
{
    return _ttalink_pod_1537_pass_development_to_payload_pack_chan(dst_addr, src_addr, chan, msg,  data, true);
}


static inline uint16_t _ttalink_pod_1537_pass_development_to_payload_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1537_pass_development_to_payload_pack_nocrc(dst_addr, src_addr, msg, pod_1537_pass_development_to_payload->data);
    }else{
        return ttalink_pod_1537_pass_development_to_payload_pack(dst_addr, src_addr, msg, pod_1537_pass_development_to_payload->data);
    }
    
}

/**
 * @brief Encode a pod_1537_pass_development_to_payload struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1537_pass_development_to_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload)
{
    return _ttalink_pod_1537_pass_development_to_payload_encode(dst_addr, src_addr, msg, pod_1537_pass_development_to_payload, false);
}

/**
 * @brief Encode a pod_1537_pass_development_to_payload struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1537_pass_development_to_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload)
{
    return _ttalink_pod_1537_pass_development_to_payload_encode(dst_addr, src_addr, msg, pod_1537_pass_development_to_payload, true);
}


static inline uint16_t _ttalink_pod_1537_pass_development_to_payload_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1537_pass_development_to_payload_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1537_pass_development_to_payload->data);
    }else{
        return ttalink_pod_1537_pass_development_to_payload_pack_chan(dst_addr, src_addr, chan, msg, pod_1537_pass_development_to_payload->data);
    }
}

/**
 * @brief Encode a pod_1537_pass_development_to_payload struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1537_pass_development_to_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload)
{
    return _ttalink_pod_1537_pass_development_to_payload_encode_chan(dst_addr, src_addr, chan, msg, pod_1537_pass_development_to_payload, false);
}

/**
 * @brief Encode a pod_1537_pass_development_to_payload struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1537_pass_development_to_payload C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload)
{
    return _ttalink_pod_1537_pass_development_to_payload_encode_chan(dst_addr, src_addr, chan, msg, pod_1537_pass_development_to_payload, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1537_pass_development_to_payload_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN];

    _tta_put_uint8_t_array(buf, 0, data, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD, buf, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC, nocrc);
#else
    ttalink_pod_1537_pass_development_to_payload_t packet;

    tta_array_memcpy(packet.data, data, sizeof(uint8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD, (const char *)&packet, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1537_pass_development_to_payload message
 * @param chan TTAlink channel to send the message
 *
 * @param data .
 */
static inline void ttalink_pod_1537_pass_development_to_payload_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *data)
{
    _ttalink_pod_1537_pass_development_to_payload_send(dst_addr, src_addr, chan, data, false);
}

/**
 * @brief Send a pod_1537_pass_development_to_payload message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param data .
 */
static inline void ttalink_pod_1537_pass_development_to_payload_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const uint8_t *data)
{
    _ttalink_pod_1537_pass_development_to_payload_send(dst_addr, src_addr, chan, data, true);
}


static inline void _ttalink_pod_1537_pass_development_to_payload_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1537_pass_development_to_payload_send_nocrc(dst_addr, src_addr, chan, pod_1537_pass_development_to_payload->data);
    }else{
        ttalink_pod_1537_pass_development_to_payload_send(dst_addr, src_addr, chan, pod_1537_pass_development_to_payload->data);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD, (const char *)pod_1537_pass_development_to_payload, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1537_pass_development_to_payload message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1537_pass_development_to_payload_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload)
{
    _ttalink_pod_1537_pass_development_to_payload_send_struct(dst_addr, src_addr, chan, pod_1537_pass_development_to_payload, false);
}

/**
 * @brief Send a pod_1537_pass_development_to_payload message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1537_pass_development_to_payload_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload)
{
    _ttalink_pod_1537_pass_development_to_payload_send_struct(dst_addr, src_addr, chan, pod_1537_pass_development_to_payload, true);
}

#if TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1537_pass_development_to_payload_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_uint8_t_array(buf, 0, data, 255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD, buf, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC, nocrc);
#else
    ttalink_pod_1537_pass_development_to_payload_t *packet = (ttalink_pod_1537_pass_development_to_payload_t *)msgbuf;

    tta_array_memcpy(packet->data, data, sizeof(uint8_t)*255);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD, (const char *)packet, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_MIN_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1537_pass_development_to_payload_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *data)
{
    _ttalink_pod_1537_pass_development_to_payload_send_buf(dst_addr, src_addr, msgbuf, chan, data, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1537_pass_development_to_payload_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const uint8_t *data)
{
    _ttalink_pod_1537_pass_development_to_payload_send_buf(dst_addr, src_addr, msgbuf, chan, data, true);
}
#endif

#endif

// MESSAGE POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD UNPACKING


/**
 * @brief Get field data from pod_1537_pass_development_to_payload message
 *
 * @return .
 */
static inline uint16_t ttalink_pod_1537_pass_development_to_payload_get_data(const ttalink_message_t* msg, uint8_t *data)
{
    return _TTA_RETURN_uint8_t_array(msg, data, 255,  0);
}

/**
 * @brief Decode a pod_1537_pass_development_to_payload message into a struct
 *
 * @param msg The message to decode
 * @param pod_1537_pass_development_to_payload C-struct to decode the message contents into
 */
static inline void ttalink_pod_1537_pass_development_to_payload_decode(const ttalink_message_t* msg, ttalink_pod_1537_pass_development_to_payload_t* pod_1537_pass_development_to_payload)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_pod_1537_pass_development_to_payload_get_data(msg, pod_1537_pass_development_to_payload->data);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN? msg->len : TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN;
        memset(pod_1537_pass_development_to_payload, 0, TTALINK_MSG_ID_POD_1537_PASS_DEVELOPMENT_TO_PAYLOAD_LEN);
    memcpy(pod_1537_pass_development_to_payload, _TTA_PAYLOAD(msg), len);
#endif
}
