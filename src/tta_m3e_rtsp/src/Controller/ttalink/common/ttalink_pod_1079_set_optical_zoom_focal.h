#pragma once
// MESSAGE POD_1079_SET_OPTICAL_ZOOM_FOCAL PACKING

#define TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL 11079

TTAPACKED(
typedef struct __ttalink_pod_1079_set_optical_zoom_focal_t {
 uint16_t focal; /*<  Float. The focal length of the zoom lens, in 0.1 mm...*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1079_set_optical_zoom_focal_t;

#define TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN 3
#define TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN 3
#define TTALINK_MSG_ID_11079_LEN 3
#define TTALINK_MSG_ID_11079_MIN_LEN 3

#define TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC 105
#define TTALINK_MSG_ID_11079_CRC 105



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1079_SET_OPTICAL_ZOOM_FOCAL { \
    11079, \
    "POD_1079_SET_OPTICAL_ZOOM_FOCAL", \
    2, \
    {  { "focal", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pod_1079_set_optical_zoom_focal_t, focal) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1079_set_optical_zoom_focal_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1079_SET_OPTICAL_ZOOM_FOCAL { \
    "POD_1079_SET_OPTICAL_ZOOM_FOCAL", \
    2, \
    {  { "focal", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pod_1079_set_optical_zoom_focal_t, focal) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pod_1079_set_optical_zoom_focal_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1079_set_optical_zoom_focal_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t focal, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN];
    _tta_put_uint16_t(buf, 0, focal);
    _tta_put_uint8_t(buf, 2, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN);
#else
    ttalink_pod_1079_set_optical_zoom_focal_t packet;
    packet.focal = focal;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1079_set_optical_zoom_focal message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param focal  Float. The focal length of the zoom lens, in 0.1 mm...
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t focal, uint8_t ack)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_pack(dst_addr, src_addr, msg,  focal, ack, false);
}

/**
 * @brief Pack a pod_1079_set_optical_zoom_focal message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param focal  Float. The focal length of the zoom lens, in 0.1 mm...
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t focal, uint8_t ack)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_pack(dst_addr, src_addr, msg,  focal, ack, true);
}


static inline uint16_t _ttalink_pod_1079_set_optical_zoom_focal_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t focal,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN];
    _tta_put_uint16_t(buf, 0, focal);
    _tta_put_uint8_t(buf, 2, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN);
#else
    ttalink_pod_1079_set_optical_zoom_focal_t packet;
    packet.focal = focal;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
}

/**
 * @brief Pack a pod_1079_set_optical_zoom_focal message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param focal  Float. The focal length of the zoom lens, in 0.1 mm...
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t focal,uint8_t ack)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_pack_chan(dst_addr, src_addr, chan, msg,  focal, ack, false);
}

/**
 * @brief Pack a pod_1079_set_optical_zoom_focal message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param focal  Float. The focal length of the zoom lens, in 0.1 mm...
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t focal,uint8_t ack)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_pack_chan(dst_addr, src_addr, chan, msg,  focal, ack, true);
}


static inline uint16_t _ttalink_pod_1079_set_optical_zoom_focal_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1079_set_optical_zoom_focal_pack_nocrc(dst_addr, src_addr, msg, pod_1079_set_optical_zoom_focal->focal, pod_1079_set_optical_zoom_focal->ack);
    }else{
        return ttalink_pod_1079_set_optical_zoom_focal_pack(dst_addr, src_addr, msg, pod_1079_set_optical_zoom_focal->focal, pod_1079_set_optical_zoom_focal->ack);
    }
    
}

/**
 * @brief Encode a pod_1079_set_optical_zoom_focal struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1079_set_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_encode(dst_addr, src_addr, msg, pod_1079_set_optical_zoom_focal, false);
}

/**
 * @brief Encode a pod_1079_set_optical_zoom_focal struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1079_set_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_encode(dst_addr, src_addr, msg, pod_1079_set_optical_zoom_focal, true);
}


static inline uint16_t _ttalink_pod_1079_set_optical_zoom_focal_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1079_set_optical_zoom_focal_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1079_set_optical_zoom_focal->focal, pod_1079_set_optical_zoom_focal->ack);
    }else{
        return ttalink_pod_1079_set_optical_zoom_focal_pack_chan(dst_addr, src_addr, chan, msg, pod_1079_set_optical_zoom_focal->focal, pod_1079_set_optical_zoom_focal->ack);
    }
}

/**
 * @brief Encode a pod_1079_set_optical_zoom_focal struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1079_set_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_encode_chan(dst_addr, src_addr, chan, msg, pod_1079_set_optical_zoom_focal, false);
}

/**
 * @brief Encode a pod_1079_set_optical_zoom_focal struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1079_set_optical_zoom_focal C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal)
{
    return _ttalink_pod_1079_set_optical_zoom_focal_encode_chan(dst_addr, src_addr, chan, msg, pod_1079_set_optical_zoom_focal, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1079_set_optical_zoom_focal_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t focal, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN];
    _tta_put_uint16_t(buf, 0, focal);
    _tta_put_uint8_t(buf, 2, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL, buf, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#else
    ttalink_pod_1079_set_optical_zoom_focal_t packet;
    packet.focal = focal;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL, (const char *)&packet, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1079_set_optical_zoom_focal message
 * @param chan TTAlink channel to send the message
 *
 * @param focal  Float. The focal length of the zoom lens, in 0.1 mm...
 * @param ack .
 */
static inline void ttalink_pod_1079_set_optical_zoom_focal_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t focal, uint8_t ack)
{
    _ttalink_pod_1079_set_optical_zoom_focal_send(dst_addr, src_addr, chan, focal, ack, false);
}

/**
 * @brief Send a pod_1079_set_optical_zoom_focal message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param focal  Float. The focal length of the zoom lens, in 0.1 mm...
 * @param ack .
 */
static inline void ttalink_pod_1079_set_optical_zoom_focal_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t focal, uint8_t ack)
{
    _ttalink_pod_1079_set_optical_zoom_focal_send(dst_addr, src_addr, chan, focal, ack, true);
}


static inline void _ttalink_pod_1079_set_optical_zoom_focal_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1079_set_optical_zoom_focal_send_nocrc(dst_addr, src_addr, chan, pod_1079_set_optical_zoom_focal->focal, pod_1079_set_optical_zoom_focal->ack);
    }else{
        ttalink_pod_1079_set_optical_zoom_focal_send(dst_addr, src_addr, chan, pod_1079_set_optical_zoom_focal->focal, pod_1079_set_optical_zoom_focal->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL, (const char *)pod_1079_set_optical_zoom_focal, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1079_set_optical_zoom_focal message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1079_set_optical_zoom_focal_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal)
{
    _ttalink_pod_1079_set_optical_zoom_focal_send_struct(dst_addr, src_addr, chan, pod_1079_set_optical_zoom_focal, false);
}

/**
 * @brief Send a pod_1079_set_optical_zoom_focal message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1079_set_optical_zoom_focal_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal)
{
    _ttalink_pod_1079_set_optical_zoom_focal_send_struct(dst_addr, src_addr, chan, pod_1079_set_optical_zoom_focal, true);
}

#if TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1079_set_optical_zoom_focal_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t focal, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, focal);
    _tta_put_uint8_t(buf, 2, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL, buf, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#else
    ttalink_pod_1079_set_optical_zoom_focal_t *packet = (ttalink_pod_1079_set_optical_zoom_focal_t *)msgbuf;
    packet->focal = focal;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL, (const char *)packet, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_MIN_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1079_set_optical_zoom_focal_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t focal, uint8_t ack)
{
    _ttalink_pod_1079_set_optical_zoom_focal_send_buf(dst_addr, src_addr, msgbuf, chan, focal, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1079_set_optical_zoom_focal_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t focal, uint8_t ack)
{
    _ttalink_pod_1079_set_optical_zoom_focal_send_buf(dst_addr, src_addr, msgbuf, chan, focal, ack, true);
}
#endif

#endif

// MESSAGE POD_1079_SET_OPTICAL_ZOOM_FOCAL UNPACKING


/**
 * @brief Get field focal from pod_1079_set_optical_zoom_focal message
 *
 * @return  Float. The focal length of the zoom lens, in 0.1 mm...
 */
static inline uint16_t ttalink_pod_1079_set_optical_zoom_focal_get_focal(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field ack from pod_1079_set_optical_zoom_focal message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1079_set_optical_zoom_focal_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a pod_1079_set_optical_zoom_focal message into a struct
 *
 * @param msg The message to decode
 * @param pod_1079_set_optical_zoom_focal C-struct to decode the message contents into
 */
static inline void ttalink_pod_1079_set_optical_zoom_focal_decode(const ttalink_message_t* msg, ttalink_pod_1079_set_optical_zoom_focal_t* pod_1079_set_optical_zoom_focal)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1079_set_optical_zoom_focal->focal = ttalink_pod_1079_set_optical_zoom_focal_get_focal(msg);
    pod_1079_set_optical_zoom_focal->ack = ttalink_pod_1079_set_optical_zoom_focal_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN? msg->len : TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN;
        memset(pod_1079_set_optical_zoom_focal, 0, TTALINK_MSG_ID_POD_1079_SET_OPTICAL_ZOOM_FOCAL_LEN);
    memcpy(pod_1079_set_optical_zoom_focal, _TTA_PAYLOAD(msg), len);
#endif
}
