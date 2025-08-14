#pragma once
// MESSAGE POD_1092_GET_FOCUS_RING_MAX PACKING

#define TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX 11092

TTAPACKED(
typedef struct __ttalink_pod_1092_get_focus_ring_max_t {
 uint8_t ack; /*< .*/
 uint16_t ack_max_ring; /*< Maximum value of lens focus ring value..*/
}) ttalink_pod_1092_get_focus_ring_max_t;

#define TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN 3
#define TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN 3
#define TTALINK_MSG_ID_11092_LEN 3
#define TTALINK_MSG_ID_11092_MIN_LEN 3

#define TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC 141
#define TTALINK_MSG_ID_11092_CRC 141



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1092_GET_FOCUS_RING_MAX { \
    11092, \
    "POD_1092_GET_FOCUS_RING_MAX", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1092_get_focus_ring_max_t, ack) }, \
         { "ack_max_ring", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1092_get_focus_ring_max_t, ack_max_ring) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1092_GET_FOCUS_RING_MAX { \
    "POD_1092_GET_FOCUS_RING_MAX", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1092_get_focus_ring_max_t, ack) }, \
         { "ack_max_ring", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_pod_1092_get_focus_ring_max_t, ack_max_ring) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1092_get_focus_ring_max_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_max_ring, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_max_ring);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN);
#else
    ttalink_pod_1092_get_focus_ring_max_t packet;
    packet.ack = ack;
    packet.ack_max_ring = ack_max_ring;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC, nocrc);
}

/**
 * @brief Pack a pod_1092_get_focus_ring_max message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_max_ring Maximum value of lens focus ring value..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_max_ring)
{
    return _ttalink_pod_1092_get_focus_ring_max_pack(dst_addr, src_addr, msg,  ack, ack_max_ring, false);
}

/**
 * @brief Pack a pod_1092_get_focus_ring_max message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_max_ring Maximum value of lens focus ring value..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint16_t ack_max_ring)
{
    return _ttalink_pod_1092_get_focus_ring_max_pack(dst_addr, src_addr, msg,  ack, ack_max_ring, true);
}


static inline uint16_t _ttalink_pod_1092_get_focus_ring_max_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_max_ring, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_max_ring);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN);
#else
    ttalink_pod_1092_get_focus_ring_max_t packet;
    packet.ack = ack;
    packet.ack_max_ring = ack_max_ring;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC, nocrc);
}

/**
 * @brief Pack a pod_1092_get_focus_ring_max message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_max_ring Maximum value of lens focus ring value..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_max_ring)
{
    return _ttalink_pod_1092_get_focus_ring_max_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_max_ring, false);
}

/**
 * @brief Pack a pod_1092_get_focus_ring_max message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_max_ring Maximum value of lens focus ring value..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint16_t ack_max_ring)
{
    return _ttalink_pod_1092_get_focus_ring_max_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_max_ring, true);
}


static inline uint16_t _ttalink_pod_1092_get_focus_ring_max_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1092_get_focus_ring_max_pack_nocrc(dst_addr, src_addr, msg, pod_1092_get_focus_ring_max->ack, pod_1092_get_focus_ring_max->ack_max_ring);
    }else{
        return ttalink_pod_1092_get_focus_ring_max_pack(dst_addr, src_addr, msg, pod_1092_get_focus_ring_max->ack, pod_1092_get_focus_ring_max->ack_max_ring);
    }
    
}

/**
 * @brief Encode a pod_1092_get_focus_ring_max struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1092_get_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max)
{
    return _ttalink_pod_1092_get_focus_ring_max_encode(dst_addr, src_addr, msg, pod_1092_get_focus_ring_max, false);
}

/**
 * @brief Encode a pod_1092_get_focus_ring_max struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1092_get_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max)
{
    return _ttalink_pod_1092_get_focus_ring_max_encode(dst_addr, src_addr, msg, pod_1092_get_focus_ring_max, true);
}


static inline uint16_t _ttalink_pod_1092_get_focus_ring_max_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1092_get_focus_ring_max_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1092_get_focus_ring_max->ack, pod_1092_get_focus_ring_max->ack_max_ring);
    }else{
        return ttalink_pod_1092_get_focus_ring_max_pack_chan(dst_addr, src_addr, chan, msg, pod_1092_get_focus_ring_max->ack, pod_1092_get_focus_ring_max->ack_max_ring);
    }
}

/**
 * @brief Encode a pod_1092_get_focus_ring_max struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1092_get_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max)
{
    return _ttalink_pod_1092_get_focus_ring_max_encode_chan(dst_addr, src_addr, chan, msg, pod_1092_get_focus_ring_max, false);
}

/**
 * @brief Encode a pod_1092_get_focus_ring_max struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1092_get_focus_ring_max C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max)
{
    return _ttalink_pod_1092_get_focus_ring_max_encode_chan(dst_addr, src_addr, chan, msg, pod_1092_get_focus_ring_max, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1092_get_focus_ring_max_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_max_ring, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_max_ring);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX, buf, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC, nocrc);
#else
    ttalink_pod_1092_get_focus_ring_max_t packet;
    packet.ack = ack;
    packet.ack_max_ring = ack_max_ring;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX, (const char *)&packet, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1092_get_focus_ring_max message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_max_ring Maximum value of lens focus ring value..
 */
static inline void ttalink_pod_1092_get_focus_ring_max_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_max_ring)
{
    _ttalink_pod_1092_get_focus_ring_max_send(dst_addr, src_addr, chan, ack, ack_max_ring, false);
}

/**
 * @brief Send a pod_1092_get_focus_ring_max message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_max_ring Maximum value of lens focus ring value..
 */
static inline void ttalink_pod_1092_get_focus_ring_max_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint16_t ack_max_ring)
{
    _ttalink_pod_1092_get_focus_ring_max_send(dst_addr, src_addr, chan, ack, ack_max_ring, true);
}


static inline void _ttalink_pod_1092_get_focus_ring_max_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1092_get_focus_ring_max_send_nocrc(dst_addr, src_addr, chan, pod_1092_get_focus_ring_max->ack, pod_1092_get_focus_ring_max->ack_max_ring);
    }else{
        ttalink_pod_1092_get_focus_ring_max_send(dst_addr, src_addr, chan, pod_1092_get_focus_ring_max->ack, pod_1092_get_focus_ring_max->ack_max_ring);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX, (const char *)pod_1092_get_focus_ring_max, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1092_get_focus_ring_max message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1092_get_focus_ring_max_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max)
{
    _ttalink_pod_1092_get_focus_ring_max_send_struct(dst_addr, src_addr, chan, pod_1092_get_focus_ring_max, false);
}

/**
 * @brief Send a pod_1092_get_focus_ring_max message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1092_get_focus_ring_max_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max)
{
    _ttalink_pod_1092_get_focus_ring_max_send_struct(dst_addr, src_addr, chan, pod_1092_get_focus_ring_max, true);
}

#if TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1092_get_focus_ring_max_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_max_ring, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint16_t(buf, 1, ack_max_ring);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX, buf, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC, nocrc);
#else
    ttalink_pod_1092_get_focus_ring_max_t *packet = (ttalink_pod_1092_get_focus_ring_max_t *)msgbuf;
    packet->ack = ack;
    packet->ack_max_ring = ack_max_ring;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX, (const char *)packet, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_MIN_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1092_get_focus_ring_max_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_max_ring)
{
    _ttalink_pod_1092_get_focus_ring_max_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_max_ring, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1092_get_focus_ring_max_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint16_t ack_max_ring)
{
    _ttalink_pod_1092_get_focus_ring_max_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_max_ring, true);
}
#endif

#endif

// MESSAGE POD_1092_GET_FOCUS_RING_MAX UNPACKING


/**
 * @brief Get field ack from pod_1092_get_focus_ring_max message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1092_get_focus_ring_max_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_max_ring from pod_1092_get_focus_ring_max message
 *
 * @return Maximum value of lens focus ring value..
 */
static inline uint16_t ttalink_pod_1092_get_focus_ring_max_get_ack_max_ring(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  1);
}

/**
 * @brief Decode a pod_1092_get_focus_ring_max message into a struct
 *
 * @param msg The message to decode
 * @param pod_1092_get_focus_ring_max C-struct to decode the message contents into
 */
static inline void ttalink_pod_1092_get_focus_ring_max_decode(const ttalink_message_t* msg, ttalink_pod_1092_get_focus_ring_max_t* pod_1092_get_focus_ring_max)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1092_get_focus_ring_max->ack = ttalink_pod_1092_get_focus_ring_max_get_ack(msg);
    pod_1092_get_focus_ring_max->ack_max_ring = ttalink_pod_1092_get_focus_ring_max_get_ack_max_ring(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN? msg->len : TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN;
        memset(pod_1092_get_focus_ring_max, 0, TTALINK_MSG_ID_POD_1092_GET_FOCUS_RING_MAX_LEN);
    memcpy(pod_1092_get_focus_ring_max, _TTA_PAYLOAD(msg), len);
#endif
}
