#pragma once
// MESSAGE POD_1093_SET_FOCUS_RING_MAX PACKING

#define TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX 11093

TTAPACKED(
typedef struct __ttalink_pod_1093_set_focus_ring_max_t {
 uint16_t focus_ring; /*< Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1093_set_focus_ring_max_t;

#define TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN 3
#define TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN 3
#define TTALINK_MSG_ID_11093_LEN 3
#define TTALINK_MSG_ID_11093_MIN_LEN 3

#define TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC 26
#define TTALINK_MSG_ID_11093_CRC 26



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1093_SET_FOCUS_RING_MAX { \
    11093, \
    "POD_1093_SET_FOCUS_RING_MAX", \
    2, \
    {  { "focus_ring", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pod_1093_set_focus_ring_max_t, focus_ring) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1093_set_focus_ring_max_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1093_SET_FOCUS_RING_MAX { \
    "POD_1093_SET_FOCUS_RING_MAX", \
    2, \
    {  { "focus_ring", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pod_1093_set_focus_ring_max_t, focus_ring) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1093_set_focus_ring_max_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1093_set_focus_ring_max_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t focus_ring, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN];
    _tta_put_uint16_t(buf, 0, focus_ring);
    _tta_put_uint8_t(buf, 2, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN);
#else
    ttalink_pod_1093_set_focus_ring_max_t packet;
    packet.focus_ring = focus_ring;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC, nocrc);
}

/**
 * @brief Pack a pod_1093_set_focus_ring_max message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param focus_ring Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t focus_ring, uint8_t ack)
{
    return _ttalink_pod_1093_set_focus_ring_max_pack(dst_addr, src_addr, msg,  focus_ring, ack, false);
}

/**
 * @brief Pack a pod_1093_set_focus_ring_max message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param focus_ring Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t focus_ring, uint8_t ack)
{
    return _ttalink_pod_1093_set_focus_ring_max_pack(dst_addr, src_addr, msg,  focus_ring, ack, true);
}


static inline uint16_t _ttalink_pod_1093_set_focus_ring_max_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t focus_ring,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN];
    _tta_put_uint16_t(buf, 0, focus_ring);
    _tta_put_uint8_t(buf, 2, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN);
#else
    ttalink_pod_1093_set_focus_ring_max_t packet;
    packet.focus_ring = focus_ring;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC, nocrc);
}

/**
 * @brief Pack a pod_1093_set_focus_ring_max message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param focus_ring Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t focus_ring,uint8_t ack)
{
    return _ttalink_pod_1093_set_focus_ring_max_pack_chan(dst_addr, src_addr, chan, msg,  focus_ring, ack, false);
}

/**
 * @brief Pack a pod_1093_set_focus_ring_max message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param focus_ring Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t focus_ring,uint8_t ack)
{
    return _ttalink_pod_1093_set_focus_ring_max_pack_chan(dst_addr, src_addr, chan, msg,  focus_ring, ack, true);
}


static inline uint16_t _ttalink_pod_1093_set_focus_ring_max_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1093_set_focus_ring_max_pack_nocrc(dst_addr, src_addr, msg, pod_1093_set_focus_ring_max->focus_ring, pod_1093_set_focus_ring_max->ack);
    }else{
        return ttalink_pod_1093_set_focus_ring_max_pack(dst_addr, src_addr, msg, pod_1093_set_focus_ring_max->focus_ring, pod_1093_set_focus_ring_max->ack);
    }
    
}

/**
 * @brief Encode a pod_1093_set_focus_ring_max struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1093_set_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max)
{
    return _ttalink_pod_1093_set_focus_ring_max_encode(dst_addr, src_addr, msg, pod_1093_set_focus_ring_max, false);
}

/**
 * @brief Encode a pod_1093_set_focus_ring_max struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1093_set_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max)
{
    return _ttalink_pod_1093_set_focus_ring_max_encode(dst_addr, src_addr, msg, pod_1093_set_focus_ring_max, true);
}


static inline uint16_t _ttalink_pod_1093_set_focus_ring_max_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1093_set_focus_ring_max_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1093_set_focus_ring_max->focus_ring, pod_1093_set_focus_ring_max->ack);
    }else{
        return ttalink_pod_1093_set_focus_ring_max_pack_chan(dst_addr, src_addr, chan, msg, pod_1093_set_focus_ring_max->focus_ring, pod_1093_set_focus_ring_max->ack);
    }
}

/**
 * @brief Encode a pod_1093_set_focus_ring_max struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1093_set_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max)
{
    return _ttalink_pod_1093_set_focus_ring_max_encode_chan(dst_addr, src_addr, chan, msg, pod_1093_set_focus_ring_max, false);
}

/**
 * @brief Encode a pod_1093_set_focus_ring_max struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1093_set_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max)
{
    return _ttalink_pod_1093_set_focus_ring_max_encode_chan(dst_addr, src_addr, chan, msg, pod_1093_set_focus_ring_max, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1093_set_focus_ring_max_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t focus_ring, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN];
    _tta_put_uint16_t(buf, 0, focus_ring);
    _tta_put_uint8_t(buf, 2, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX, buf, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC, nocrc);
#else
    ttalink_pod_1093_set_focus_ring_max_t packet;
    packet.focus_ring = focus_ring;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX, (const char *)&packet, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1093_set_focus_ring_max message
 * @param chan TTAlink channel to send the message
 *
 * @param focus_ring Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..
 * @param ack .
 */
static inline void ttalink_pod_1093_set_focus_ring_max_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t focus_ring, uint8_t ack)
{
    _ttalink_pod_1093_set_focus_ring_max_send(dst_addr, src_addr, chan, focus_ring, ack, false);
}

/**
 * @brief Send a pod_1093_set_focus_ring_max message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param focus_ring Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..
 * @param ack .
 */
static inline void ttalink_pod_1093_set_focus_ring_max_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t focus_ring, uint8_t ack)
{
    _ttalink_pod_1093_set_focus_ring_max_send(dst_addr, src_addr, chan, focus_ring, ack, true);
}


static inline void _ttalink_pod_1093_set_focus_ring_max_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1093_set_focus_ring_max_send_nocrc(dst_addr, src_addr, chan, pod_1093_set_focus_ring_max->focus_ring, pod_1093_set_focus_ring_max->ack);
    }else{
        ttalink_pod_1093_set_focus_ring_max_send(dst_addr, src_addr, chan, pod_1093_set_focus_ring_max->focus_ring, pod_1093_set_focus_ring_max->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX, (const char *)pod_1093_set_focus_ring_max, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1093_set_focus_ring_max message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1093_set_focus_ring_max_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max)
{
    _ttalink_pod_1093_set_focus_ring_max_send_struct(dst_addr, src_addr, chan, pod_1093_set_focus_ring_max, false);
}

/**
 * @brief Send a pod_1093_set_focus_ring_max message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1093_set_focus_ring_max_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max)
{
    _ttalink_pod_1093_set_focus_ring_max_send_struct(dst_addr, src_addr, chan, pod_1093_set_focus_ring_max, true);
}

#if TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1093_set_focus_ring_max_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t focus_ring, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, focus_ring);
    _tta_put_uint8_t(buf, 2, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX, buf, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC, nocrc);
#else
    ttalink_pod_1093_set_focus_ring_max_t *packet = (ttalink_pod_1093_set_focus_ring_max_t *)msgbuf;
    packet->focus_ring = focus_ring;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX, (const char *)packet, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1093_set_focus_ring_max_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t focus_ring, uint8_t ack)
{
    _ttalink_pod_1093_set_focus_ring_max_send_buf(dst_addr, src_addr, msgbuf, chan, focus_ring, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1093_set_focus_ring_max_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t focus_ring, uint8_t ack)
{
    _ttalink_pod_1093_set_focus_ring_max_send_buf(dst_addr, src_addr, msgbuf, chan, focus_ring, ack, true);
}
#endif

#endif

// MESSAGE POD_1093_SET_FOCUS_RING_MAX UNPACKING


/**
 * @brief Get field focus_ring from pod_1093_set_focus_ring_max message
 *
 * @return Focus ring value. The range of values can be [0, the upper limit of the focal ring value], 
											indicating the infinity and the closest focal length respectively..
 */
static inline uint16_t ttalink_pod_1093_set_focus_ring_max_get_focus_ring(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field ack from pod_1093_set_focus_ring_max message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1093_set_focus_ring_max_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a pod_1093_set_focus_ring_max message into a struct
 *
 * @param msg The message to decode
 * @param pod_1093_set_focus_ring_max C-struct to decode the message contents into
 */
static inline void ttalink_pod_1093_set_focus_ring_max_decode(const ttalink_message_t* msg, ttalink_pod_1093_set_focus_ring_max_t* pod_1093_set_focus_ring_max)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1093_set_focus_ring_max->focus_ring = ttalink_pod_1093_set_focus_ring_max_get_focus_ring(msg);
    pod_1093_set_focus_ring_max->ack = ttalink_pod_1093_set_focus_ring_max_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN? msg->len : TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN;
        memset(pod_1093_set_focus_ring_max, 0, TTALINK_MSG_ID_POD_1093_SET_FOCUS_RING_MAX_LEN);
    memcpy(pod_1093_set_focus_ring_max, _TTA_PAYLOAD(msg), len);
#endif
}
