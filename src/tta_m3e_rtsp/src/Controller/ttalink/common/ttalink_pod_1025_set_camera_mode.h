#pragma once
// MESSAGE POD_1025_SET_CAMERA_MODE PACKING

#define TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE 11025

TTAPACKED(
typedef struct __ttalink_pod_1025_set_camera_mode_t {
 uint8_t mode; /*<  0 :  Photographing mode;1 :  Recording video mode.*/
 uint8_t ack; /*<  .*/
}) ttalink_pod_1025_set_camera_mode_t;

#define TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN 2
#define TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN 2
#define TTALINK_MSG_ID_11025_LEN 2
#define TTALINK_MSG_ID_11025_MIN_LEN 2

#define TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC 199
#define TTALINK_MSG_ID_11025_CRC 199



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1025_SET_CAMERA_MODE { \
    11025, \
    "POD_1025_SET_CAMERA_MODE", \
    2, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1025_set_camera_mode_t, mode) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1025_set_camera_mode_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1025_SET_CAMERA_MODE { \
    "POD_1025_SET_CAMERA_MODE", \
    2, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1025_set_camera_mode_t, mode) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1025_set_camera_mode_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1025_set_camera_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN);
#else
    ttalink_pod_1025_set_camera_mode_t packet;
    packet.mode = mode;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1025_set_camera_mode message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  0 :  Photographing mode;1 :  Recording video mode.
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t ack)
{
    return _ttalink_pod_1025_set_camera_mode_pack(dst_addr, src_addr, msg,  mode, ack, false);
}

/**
 * @brief Pack a pod_1025_set_camera_mode message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  0 :  Photographing mode;1 :  Recording video mode.
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t ack)
{
    return _ttalink_pod_1025_set_camera_mode_pack(dst_addr, src_addr, msg,  mode, ack, true);
}


static inline uint16_t _ttalink_pod_1025_set_camera_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN);
#else
    ttalink_pod_1025_set_camera_mode_t packet;
    packet.mode = mode;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1025_set_camera_mode message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  0 :  Photographing mode;1 :  Recording video mode.
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t ack)
{
    return _ttalink_pod_1025_set_camera_mode_pack_chan(dst_addr, src_addr, chan, msg,  mode, ack, false);
}

/**
 * @brief Pack a pod_1025_set_camera_mode message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  0 :  Photographing mode;1 :  Recording video mode.
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t ack)
{
    return _ttalink_pod_1025_set_camera_mode_pack_chan(dst_addr, src_addr, chan, msg,  mode, ack, true);
}


static inline uint16_t _ttalink_pod_1025_set_camera_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1025_set_camera_mode_pack_nocrc(dst_addr, src_addr, msg, pod_1025_set_camera_mode->mode, pod_1025_set_camera_mode->ack);
    }else{
        return ttalink_pod_1025_set_camera_mode_pack(dst_addr, src_addr, msg, pod_1025_set_camera_mode->mode, pod_1025_set_camera_mode->ack);
    }
    
}

/**
 * @brief Encode a pod_1025_set_camera_mode struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1025_set_camera_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode)
{
    return _ttalink_pod_1025_set_camera_mode_encode(dst_addr, src_addr, msg, pod_1025_set_camera_mode, false);
}

/**
 * @brief Encode a pod_1025_set_camera_mode struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1025_set_camera_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode)
{
    return _ttalink_pod_1025_set_camera_mode_encode(dst_addr, src_addr, msg, pod_1025_set_camera_mode, true);
}


static inline uint16_t _ttalink_pod_1025_set_camera_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1025_set_camera_mode_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1025_set_camera_mode->mode, pod_1025_set_camera_mode->ack);
    }else{
        return ttalink_pod_1025_set_camera_mode_pack_chan(dst_addr, src_addr, chan, msg, pod_1025_set_camera_mode->mode, pod_1025_set_camera_mode->ack);
    }
}

/**
 * @brief Encode a pod_1025_set_camera_mode struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1025_set_camera_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode)
{
    return _ttalink_pod_1025_set_camera_mode_encode_chan(dst_addr, src_addr, chan, msg, pod_1025_set_camera_mode, false);
}

/**
 * @brief Encode a pod_1025_set_camera_mode struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1025_set_camera_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1025_set_camera_mode_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode)
{
    return _ttalink_pod_1025_set_camera_mode_encode_chan(dst_addr, src_addr, chan, msg, pod_1025_set_camera_mode, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1025_set_camera_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE, buf, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC, nocrc);
#else
    ttalink_pod_1025_set_camera_mode_t packet;
    packet.mode = mode;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE, (const char *)&packet, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1025_set_camera_mode message
 * @param chan TTAlink channel to send the message
 *
 * @param mode  0 :  Photographing mode;1 :  Recording video mode.
 * @param ack  .
 */
static inline void ttalink_pod_1025_set_camera_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t ack)
{
    _ttalink_pod_1025_set_camera_mode_send(dst_addr, src_addr, chan, mode, ack, false);
}

/**
 * @brief Send a pod_1025_set_camera_mode message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mode  0 :  Photographing mode;1 :  Recording video mode.
 * @param ack  .
 */
static inline void ttalink_pod_1025_set_camera_mode_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t ack)
{
    _ttalink_pod_1025_set_camera_mode_send(dst_addr, src_addr, chan, mode, ack, true);
}


static inline void _ttalink_pod_1025_set_camera_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1025_set_camera_mode_send_nocrc(dst_addr, src_addr, chan, pod_1025_set_camera_mode->mode, pod_1025_set_camera_mode->ack);
    }else{
        ttalink_pod_1025_set_camera_mode_send(dst_addr, src_addr, chan, pod_1025_set_camera_mode->mode, pod_1025_set_camera_mode->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE, (const char *)pod_1025_set_camera_mode, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1025_set_camera_mode message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1025_set_camera_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode)
{
    _ttalink_pod_1025_set_camera_mode_send_struct(dst_addr, src_addr, chan, pod_1025_set_camera_mode, false);
}

/**
 * @brief Send a pod_1025_set_camera_mode message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1025_set_camera_mode_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode)
{
    _ttalink_pod_1025_set_camera_mode_send_struct(dst_addr, src_addr, chan, pod_1025_set_camera_mode, true);
}

#if TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1025_set_camera_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE, buf, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC, nocrc);
#else
    ttalink_pod_1025_set_camera_mode_t *packet = (ttalink_pod_1025_set_camera_mode_t *)msgbuf;
    packet->mode = mode;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE, (const char *)packet, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1025_set_camera_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t ack)
{
    _ttalink_pod_1025_set_camera_mode_send_buf(dst_addr, src_addr, msgbuf, chan, mode, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1025_set_camera_mode_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t ack)
{
    _ttalink_pod_1025_set_camera_mode_send_buf(dst_addr, src_addr, msgbuf, chan, mode, ack, true);
}
#endif

#endif

// MESSAGE POD_1025_SET_CAMERA_MODE UNPACKING


/**
 * @brief Get field mode from pod_1025_set_camera_mode message
 *
 * @return  0 :  Photographing mode;1 :  Recording video mode.
 */
static inline uint8_t ttalink_pod_1025_set_camera_mode_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from pod_1025_set_camera_mode message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_1025_set_camera_mode_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a pod_1025_set_camera_mode message into a struct
 *
 * @param msg The message to decode
 * @param pod_1025_set_camera_mode C-struct to decode the message contents into
 */
static inline void ttalink_pod_1025_set_camera_mode_decode(const ttalink_message_t* msg, ttalink_pod_1025_set_camera_mode_t* pod_1025_set_camera_mode)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1025_set_camera_mode->mode = ttalink_pod_1025_set_camera_mode_get_mode(msg);
    pod_1025_set_camera_mode->ack = ttalink_pod_1025_set_camera_mode_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN? msg->len : TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN;
        memset(pod_1025_set_camera_mode, 0, TTALINK_MSG_ID_POD_1025_SET_CAMERA_MODE_LEN);
    memcpy(pod_1025_set_camera_mode, _TTA_PAYLOAD(msg), len);
#endif
}
