#pragma once
// MESSAGE POD_1080_GET_OPTICAL_ZOOM_FOCAL PACKING

#define TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL 11080

TTAPACKED(
typedef struct __ttalink_pod_1080_get_optical_zoom_focal_t {
 uint8_t ack; /*< .*/
 uint32_t ack_focal; /*<  Float. The focal length of the zoom lens, in 0.1 mm..*/
}) ttalink_pod_1080_get_optical_zoom_focal_t;

#define TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN 5
#define TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN 5
#define TTALINK_MSG_ID_11080_LEN 5
#define TTALINK_MSG_ID_11080_MIN_LEN 5

#define TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC 243
#define TTALINK_MSG_ID_11080_CRC 243



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1080_GET_OPTICAL_ZOOM_FOCAL { \
    11080, \
    "POD_1080_GET_OPTICAL_ZOOM_FOCAL", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1080_get_optical_zoom_focal_t, ack) }, \
         { "ack_focal", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_pod_1080_get_optical_zoom_focal_t, ack_focal) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1080_GET_OPTICAL_ZOOM_FOCAL { \
    "POD_1080_GET_OPTICAL_ZOOM_FOCAL", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1080_get_optical_zoom_focal_t, ack) }, \
         { "ack_focal", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_pod_1080_get_optical_zoom_focal_t, ack_focal) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1080_get_optical_zoom_focal_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint32_t ack_focal, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, ack_focal);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN);
#else
    ttalink_pod_1080_get_optical_zoom_focal_t packet;
    packet.ack = ack;
    packet.ack_focal = ack_focal;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1080_get_optical_zoom_focal message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_focal  Float. The focal length of the zoom lens, in 0.1 mm..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint32_t ack_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_pack(dst_addr, src_addr, msg,  ack, ack_focal, false);
}

/**
 * @brief Pack a pod_1080_get_optical_zoom_focal message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_focal  Float. The focal length of the zoom lens, in 0.1 mm..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint32_t ack_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_pack(dst_addr, src_addr, msg,  ack, ack_focal, true);
}


static inline uint16_t _ttalink_pod_1080_get_optical_zoom_focal_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint32_t ack_focal, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, ack_focal);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN);
#else
    ttalink_pod_1080_get_optical_zoom_focal_t packet;
    packet.ack = ack;
    packet.ack_focal = ack_focal;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1080_get_optical_zoom_focal message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_focal  Float. The focal length of the zoom lens, in 0.1 mm..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint32_t ack_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_focal, false);
}

/**
 * @brief Pack a pod_1080_get_optical_zoom_focal message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_focal  Float. The focal length of the zoom lens, in 0.1 mm..
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint32_t ack_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_focal, true);
}


static inline uint16_t _ttalink_pod_1080_get_optical_zoom_focal_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1080_get_optical_zoom_focal_pack_nocrc(dst_addr, src_addr, msg, pod_1080_get_optical_zoom_focal->ack, pod_1080_get_optical_zoom_focal->ack_focal);
    }else{
        return ttalink_pod_1080_get_optical_zoom_focal_pack(dst_addr, src_addr, msg, pod_1080_get_optical_zoom_focal->ack, pod_1080_get_optical_zoom_focal->ack_focal);
    }
    
}

/**
 * @brief Encode a pod_1080_get_optical_zoom_focal struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1080_get_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_encode(dst_addr, src_addr, msg, pod_1080_get_optical_zoom_focal, false);
}

/**
 * @brief Encode a pod_1080_get_optical_zoom_focal struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1080_get_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_encode(dst_addr, src_addr, msg, pod_1080_get_optical_zoom_focal, true);
}


static inline uint16_t _ttalink_pod_1080_get_optical_zoom_focal_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1080_get_optical_zoom_focal_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1080_get_optical_zoom_focal->ack, pod_1080_get_optical_zoom_focal->ack_focal);
    }else{
        return ttalink_pod_1080_get_optical_zoom_focal_pack_chan(dst_addr, src_addr, chan, msg, pod_1080_get_optical_zoom_focal->ack, pod_1080_get_optical_zoom_focal->ack_focal);
    }
}

/**
 * @brief Encode a pod_1080_get_optical_zoom_focal struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1080_get_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_encode_chan(dst_addr, src_addr, chan, msg, pod_1080_get_optical_zoom_focal, false);
}

/**
 * @brief Encode a pod_1080_get_optical_zoom_focal struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1080_get_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1080_get_optical_zoom_focal_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal)
{
    return _ttalink_pod_1080_get_optical_zoom_focal_encode_chan(dst_addr, src_addr, chan, msg, pod_1080_get_optical_zoom_focal, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1080_get_optical_zoom_focal_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint32_t ack_focal, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, ack_focal);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL, buf, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#else
    ttalink_pod_1080_get_optical_zoom_focal_t packet;
    packet.ack = ack;
    packet.ack_focal = ack_focal;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL, (const char *)&packet, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1080_get_optical_zoom_focal message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_focal  Float. The focal length of the zoom lens, in 0.1 mm..
 */
static inline void ttalink_pod_1080_get_optical_zoom_focal_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint32_t ack_focal)
{
    _ttalink_pod_1080_get_optical_zoom_focal_send(dst_addr, src_addr, chan, ack, ack_focal, false);
}

/**
 * @brief Send a pod_1080_get_optical_zoom_focal message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_focal  Float. The focal length of the zoom lens, in 0.1 mm..
 */
static inline void ttalink_pod_1080_get_optical_zoom_focal_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint32_t ack_focal)
{
    _ttalink_pod_1080_get_optical_zoom_focal_send(dst_addr, src_addr, chan, ack, ack_focal, true);
}


static inline void _ttalink_pod_1080_get_optical_zoom_focal_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1080_get_optical_zoom_focal_send_nocrc(dst_addr, src_addr, chan, pod_1080_get_optical_zoom_focal->ack, pod_1080_get_optical_zoom_focal->ack_focal);
    }else{
        ttalink_pod_1080_get_optical_zoom_focal_send(dst_addr, src_addr, chan, pod_1080_get_optical_zoom_focal->ack, pod_1080_get_optical_zoom_focal->ack_focal);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL, (const char *)pod_1080_get_optical_zoom_focal, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1080_get_optical_zoom_focal message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1080_get_optical_zoom_focal_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal)
{
    _ttalink_pod_1080_get_optical_zoom_focal_send_struct(dst_addr, src_addr, chan, pod_1080_get_optical_zoom_focal, false);
}

/**
 * @brief Send a pod_1080_get_optical_zoom_focal message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1080_get_optical_zoom_focal_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal)
{
    _ttalink_pod_1080_get_optical_zoom_focal_send_struct(dst_addr, src_addr, chan, pod_1080_get_optical_zoom_focal, true);
}

#if TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1080_get_optical_zoom_focal_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint32_t ack_focal, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, ack_focal);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL, buf, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#else
    ttalink_pod_1080_get_optical_zoom_focal_t *packet = (ttalink_pod_1080_get_optical_zoom_focal_t *)msgbuf;
    packet->ack = ack;
    packet->ack_focal = ack_focal;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL, (const char *)packet, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1080_get_optical_zoom_focal_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint32_t ack_focal)
{
    _ttalink_pod_1080_get_optical_zoom_focal_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_focal, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1080_get_optical_zoom_focal_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint32_t ack_focal)
{
    _ttalink_pod_1080_get_optical_zoom_focal_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_focal, true);
}
#endif

#endif

// MESSAGE POD_1080_GET_OPTICAL_ZOOM_FOCAL UNPACKING


/**
 * @brief Get field ack from pod_1080_get_optical_zoom_focal message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1080_get_optical_zoom_focal_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_focal from pod_1080_get_optical_zoom_focal message
 *
 * @return  Float. The focal length of the zoom lens, in 0.1 mm..
 */
static inline uint32_t ttalink_pod_1080_get_optical_zoom_focal_get_ack_focal(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  1);
}

/**
 * @brief Decode a pod_1080_get_optical_zoom_focal message into a struct
 *
 * @param msg The message to decode
 * @param pod_1080_get_optical_zoom_focal C-struct to decode the message contents into
 */
static inline void ttalink_pod_1080_get_optical_zoom_focal_decode(const ttalink_message_t* msg, ttalink_pod_1080_get_optical_zoom_focal_t* pod_1080_get_optical_zoom_focal)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1080_get_optical_zoom_focal->ack = ttalink_pod_1080_get_optical_zoom_focal_get_ack(msg);
    pod_1080_get_optical_zoom_focal->ack_focal = ttalink_pod_1080_get_optical_zoom_focal_get_ack_focal(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN? msg->len : TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN;
        memset(pod_1080_get_optical_zoom_focal, 0, TTALINK_MSG_ID_POD_1080_GET_OPTICAL_ZOOM_FOCAL_LEN);
    memcpy(pod_1080_get_optical_zoom_focal, _TTA_PAYLOAD(msg), len);
#endif
}
