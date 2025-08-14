#pragma once
// MESSAGE POD_1076_SET_DIGITAL_ZOOM_FACTOR PACKING

#define TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR 11076

TTAPACKED(
typedef struct __ttalink_pod_1076_set_digital_zoom_factor_t {
 uint32_t factor; /*<  The valid range is from 1 to the maximum digital scaling factor..*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1076_set_digital_zoom_factor_t;

#define TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN 5
#define TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN 5
#define TTALINK_MSG_ID_11076_LEN 5
#define TTALINK_MSG_ID_11076_MIN_LEN 5

#define TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC 159
#define TTALINK_MSG_ID_11076_CRC 159



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1076_SET_DIGITAL_ZOOM_FACTOR { \
    11076, \
    "POD_1076_SET_DIGITAL_ZOOM_FACTOR", \
    2, \
    {  { "factor", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_1076_set_digital_zoom_factor_t, factor) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1076_set_digital_zoom_factor_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1076_SET_DIGITAL_ZOOM_FACTOR { \
    "POD_1076_SET_DIGITAL_ZOOM_FACTOR", \
    2, \
    {  { "factor", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_1076_set_digital_zoom_factor_t, factor) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_pod_1076_set_digital_zoom_factor_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1076_set_digital_zoom_factor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t factor, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN];
    _tta_put_uint32_t(buf, 0, factor);
    _tta_put_uint8_t(buf, 4, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN);
#else
    ttalink_pod_1076_set_digital_zoom_factor_t packet;
    packet.factor = factor;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC, nocrc);
}

/**
 * @brief Pack a pod_1076_set_digital_zoom_factor message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param factor  The valid range is from 1 to the maximum digital scaling factor..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t factor, uint8_t ack)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_pack(dst_addr, src_addr, msg,  factor, ack, false);
}

/**
 * @brief Pack a pod_1076_set_digital_zoom_factor message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param factor  The valid range is from 1 to the maximum digital scaling factor..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t factor, uint8_t ack)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_pack(dst_addr, src_addr, msg,  factor, ack, true);
}


static inline uint16_t _ttalink_pod_1076_set_digital_zoom_factor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t factor,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN];
    _tta_put_uint32_t(buf, 0, factor);
    _tta_put_uint8_t(buf, 4, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN);
#else
    ttalink_pod_1076_set_digital_zoom_factor_t packet;
    packet.factor = factor;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC, nocrc);
}

/**
 * @brief Pack a pod_1076_set_digital_zoom_factor message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param factor  The valid range is from 1 to the maximum digital scaling factor..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t factor,uint8_t ack)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_pack_chan(dst_addr, src_addr, chan, msg,  factor, ack, false);
}

/**
 * @brief Pack a pod_1076_set_digital_zoom_factor message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param factor  The valid range is from 1 to the maximum digital scaling factor..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t factor,uint8_t ack)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_pack_chan(dst_addr, src_addr, chan, msg,  factor, ack, true);
}


static inline uint16_t _ttalink_pod_1076_set_digital_zoom_factor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1076_set_digital_zoom_factor_pack_nocrc(dst_addr, src_addr, msg, pod_1076_set_digital_zoom_factor->factor, pod_1076_set_digital_zoom_factor->ack);
    }else{
        return ttalink_pod_1076_set_digital_zoom_factor_pack(dst_addr, src_addr, msg, pod_1076_set_digital_zoom_factor->factor, pod_1076_set_digital_zoom_factor->ack);
    }
    
}

/**
 * @brief Encode a pod_1076_set_digital_zoom_factor struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1076_set_digital_zoom_factor C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_encode(dst_addr, src_addr, msg, pod_1076_set_digital_zoom_factor, false);
}

/**
 * @brief Encode a pod_1076_set_digital_zoom_factor struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1076_set_digital_zoom_factor C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_encode(dst_addr, src_addr, msg, pod_1076_set_digital_zoom_factor, true);
}


static inline uint16_t _ttalink_pod_1076_set_digital_zoom_factor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1076_set_digital_zoom_factor_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1076_set_digital_zoom_factor->factor, pod_1076_set_digital_zoom_factor->ack);
    }else{
        return ttalink_pod_1076_set_digital_zoom_factor_pack_chan(dst_addr, src_addr, chan, msg, pod_1076_set_digital_zoom_factor->factor, pod_1076_set_digital_zoom_factor->ack);
    }
}

/**
 * @brief Encode a pod_1076_set_digital_zoom_factor struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1076_set_digital_zoom_factor C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_encode_chan(dst_addr, src_addr, chan, msg, pod_1076_set_digital_zoom_factor, false);
}

/**
 * @brief Encode a pod_1076_set_digital_zoom_factor struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1076_set_digital_zoom_factor C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1076_set_digital_zoom_factor_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor)
{
    return _ttalink_pod_1076_set_digital_zoom_factor_encode_chan(dst_addr, src_addr, chan, msg, pod_1076_set_digital_zoom_factor, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1076_set_digital_zoom_factor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t factor, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN];
    _tta_put_uint32_t(buf, 0, factor);
    _tta_put_uint8_t(buf, 4, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR, buf, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC, nocrc);
#else
    ttalink_pod_1076_set_digital_zoom_factor_t packet;
    packet.factor = factor;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR, (const char *)&packet, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1076_set_digital_zoom_factor message
 * @param chan TTAlink channel to send the message
 *
 * @param factor  The valid range is from 1 to the maximum digital scaling factor..
 * @param ack .
 */
static inline void ttalink_pod_1076_set_digital_zoom_factor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t factor, uint8_t ack)
{
    _ttalink_pod_1076_set_digital_zoom_factor_send(dst_addr, src_addr, chan, factor, ack, false);
}

/**
 * @brief Send a pod_1076_set_digital_zoom_factor message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param factor  The valid range is from 1 to the maximum digital scaling factor..
 * @param ack .
 */
static inline void ttalink_pod_1076_set_digital_zoom_factor_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t factor, uint8_t ack)
{
    _ttalink_pod_1076_set_digital_zoom_factor_send(dst_addr, src_addr, chan, factor, ack, true);
}


static inline void _ttalink_pod_1076_set_digital_zoom_factor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1076_set_digital_zoom_factor_send_nocrc(dst_addr, src_addr, chan, pod_1076_set_digital_zoom_factor->factor, pod_1076_set_digital_zoom_factor->ack);
    }else{
        ttalink_pod_1076_set_digital_zoom_factor_send(dst_addr, src_addr, chan, pod_1076_set_digital_zoom_factor->factor, pod_1076_set_digital_zoom_factor->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR, (const char *)pod_1076_set_digital_zoom_factor, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1076_set_digital_zoom_factor message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1076_set_digital_zoom_factor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor)
{
    _ttalink_pod_1076_set_digital_zoom_factor_send_struct(dst_addr, src_addr, chan, pod_1076_set_digital_zoom_factor, false);
}

/**
 * @brief Send a pod_1076_set_digital_zoom_factor message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1076_set_digital_zoom_factor_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor)
{
    _ttalink_pod_1076_set_digital_zoom_factor_send_struct(dst_addr, src_addr, chan, pod_1076_set_digital_zoom_factor, true);
}

#if TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1076_set_digital_zoom_factor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t factor, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, factor);
    _tta_put_uint8_t(buf, 4, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR, buf, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC, nocrc);
#else
    ttalink_pod_1076_set_digital_zoom_factor_t *packet = (ttalink_pod_1076_set_digital_zoom_factor_t *)msgbuf;
    packet->factor = factor;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR, (const char *)packet, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_MIN_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1076_set_digital_zoom_factor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t factor, uint8_t ack)
{
    _ttalink_pod_1076_set_digital_zoom_factor_send_buf(dst_addr, src_addr, msgbuf, chan, factor, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1076_set_digital_zoom_factor_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t factor, uint8_t ack)
{
    _ttalink_pod_1076_set_digital_zoom_factor_send_buf(dst_addr, src_addr, msgbuf, chan, factor, ack, true);
}
#endif

#endif

// MESSAGE POD_1076_SET_DIGITAL_ZOOM_FACTOR UNPACKING


/**
 * @brief Get field factor from pod_1076_set_digital_zoom_factor message
 *
 * @return  The valid range is from 1 to the maximum digital scaling factor..
 */
static inline uint32_t ttalink_pod_1076_set_digital_zoom_factor_get_factor(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ack from pod_1076_set_digital_zoom_factor message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1076_set_digital_zoom_factor_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a pod_1076_set_digital_zoom_factor message into a struct
 *
 * @param msg The message to decode
 * @param pod_1076_set_digital_zoom_factor C-struct to decode the message contents into
 */
static inline void ttalink_pod_1076_set_digital_zoom_factor_decode(const ttalink_message_t* msg, ttalink_pod_1076_set_digital_zoom_factor_t* pod_1076_set_digital_zoom_factor)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1076_set_digital_zoom_factor->factor = ttalink_pod_1076_set_digital_zoom_factor_get_factor(msg);
    pod_1076_set_digital_zoom_factor->ack = ttalink_pod_1076_set_digital_zoom_factor_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN? msg->len : TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN;
        memset(pod_1076_set_digital_zoom_factor, 0, TTALINK_MSG_ID_POD_1076_SET_DIGITAL_ZOOM_FACTOR_LEN);
    memcpy(pod_1076_set_digital_zoom_factor, _TTA_PAYLOAD(msg), len);
#endif
}
