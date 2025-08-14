#pragma once
// MESSAGE POD_1283_PTZ_HEAD_BACK PACKING

#define TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK 11283

TTAPACKED(
typedef struct __ttalink_pod_1283_ptz_head_back_t {
 uint8_t ack; /*< .*/
}) ttalink_pod_1283_ptz_head_back_t;

#define TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN 1
#define TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN 1
#define TTALINK_MSG_ID_11283_LEN 1
#define TTALINK_MSG_ID_11283_MIN_LEN 1

#define TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC 117
#define TTALINK_MSG_ID_11283_CRC 117



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1283_PTZ_HEAD_BACK { \
    11283, \
    "POD_1283_PTZ_HEAD_BACK", \
    1, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1283_ptz_head_back_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1283_PTZ_HEAD_BACK { \
    "POD_1283_PTZ_HEAD_BACK", \
    1, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1283_ptz_head_back_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1283_ptz_head_back_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN);
#else
    ttalink_pod_1283_ptz_head_back_t packet;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC, nocrc);
}

/**
 * @brief Pack a pod_1283_ptz_head_back message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack)
{
    return _ttalink_pod_1283_ptz_head_back_pack(dst_addr, src_addr, msg,  ack, false);
}

/**
 * @brief Pack a pod_1283_ptz_head_back message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack)
{
    return _ttalink_pod_1283_ptz_head_back_pack(dst_addr, src_addr, msg,  ack, true);
}


static inline uint16_t _ttalink_pod_1283_ptz_head_back_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN);
#else
    ttalink_pod_1283_ptz_head_back_t packet;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC, nocrc);
}

/**
 * @brief Pack a pod_1283_ptz_head_back message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack)
{
    return _ttalink_pod_1283_ptz_head_back_pack_chan(dst_addr, src_addr, chan, msg,  ack, false);
}

/**
 * @brief Pack a pod_1283_ptz_head_back message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack)
{
    return _ttalink_pod_1283_ptz_head_back_pack_chan(dst_addr, src_addr, chan, msg,  ack, true);
}


static inline uint16_t _ttalink_pod_1283_ptz_head_back_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1283_ptz_head_back_pack_nocrc(dst_addr, src_addr, msg, pod_1283_ptz_head_back->ack);
    }else{
        return ttalink_pod_1283_ptz_head_back_pack(dst_addr, src_addr, msg, pod_1283_ptz_head_back->ack);
    }
    
}

/**
 * @brief Encode a pod_1283_ptz_head_back struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1283_ptz_head_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back)
{
    return _ttalink_pod_1283_ptz_head_back_encode(dst_addr, src_addr, msg, pod_1283_ptz_head_back, false);
}

/**
 * @brief Encode a pod_1283_ptz_head_back struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1283_ptz_head_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back)
{
    return _ttalink_pod_1283_ptz_head_back_encode(dst_addr, src_addr, msg, pod_1283_ptz_head_back, true);
}


static inline uint16_t _ttalink_pod_1283_ptz_head_back_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1283_ptz_head_back_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1283_ptz_head_back->ack);
    }else{
        return ttalink_pod_1283_ptz_head_back_pack_chan(dst_addr, src_addr, chan, msg, pod_1283_ptz_head_back->ack);
    }
}

/**
 * @brief Encode a pod_1283_ptz_head_back struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1283_ptz_head_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back)
{
    return _ttalink_pod_1283_ptz_head_back_encode_chan(dst_addr, src_addr, chan, msg, pod_1283_ptz_head_back, false);
}

/**
 * @brief Encode a pod_1283_ptz_head_back struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1283_ptz_head_back C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1283_ptz_head_back_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back)
{
    return _ttalink_pod_1283_ptz_head_back_encode_chan(dst_addr, src_addr, chan, msg, pod_1283_ptz_head_back, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1283_ptz_head_back_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK, buf, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC, nocrc);
#else
    ttalink_pod_1283_ptz_head_back_t packet;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK, (const char *)&packet, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1283_ptz_head_back message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 */
static inline void ttalink_pod_1283_ptz_head_back_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack)
{
    _ttalink_pod_1283_ptz_head_back_send(dst_addr, src_addr, chan, ack, false);
}

/**
 * @brief Send a pod_1283_ptz_head_back message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 */
static inline void ttalink_pod_1283_ptz_head_back_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack)
{
    _ttalink_pod_1283_ptz_head_back_send(dst_addr, src_addr, chan, ack, true);
}


static inline void _ttalink_pod_1283_ptz_head_back_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1283_ptz_head_back_send_nocrc(dst_addr, src_addr, chan, pod_1283_ptz_head_back->ack);
    }else{
        ttalink_pod_1283_ptz_head_back_send(dst_addr, src_addr, chan, pod_1283_ptz_head_back->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK, (const char *)pod_1283_ptz_head_back, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1283_ptz_head_back message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1283_ptz_head_back_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back)
{
    _ttalink_pod_1283_ptz_head_back_send_struct(dst_addr, src_addr, chan, pod_1283_ptz_head_back, false);
}

/**
 * @brief Send a pod_1283_ptz_head_back message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1283_ptz_head_back_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back)
{
    _ttalink_pod_1283_ptz_head_back_send_struct(dst_addr, src_addr, chan, pod_1283_ptz_head_back, true);
}

#if TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1283_ptz_head_back_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK, buf, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC, nocrc);
#else
    ttalink_pod_1283_ptz_head_back_t *packet = (ttalink_pod_1283_ptz_head_back_t *)msgbuf;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK, (const char *)packet, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_MIN_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1283_ptz_head_back_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack)
{
    _ttalink_pod_1283_ptz_head_back_send_buf(dst_addr, src_addr, msgbuf, chan, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1283_ptz_head_back_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack)
{
    _ttalink_pod_1283_ptz_head_back_send_buf(dst_addr, src_addr, msgbuf, chan, ack, true);
}
#endif

#endif

// MESSAGE POD_1283_PTZ_HEAD_BACK UNPACKING


/**
 * @brief Get field ack from pod_1283_ptz_head_back message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1283_ptz_head_back_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a pod_1283_ptz_head_back message into a struct
 *
 * @param msg The message to decode
 * @param pod_1283_ptz_head_back C-struct to decode the message contents into
 */
static inline void ttalink_pod_1283_ptz_head_back_decode(const ttalink_message_t* msg, ttalink_pod_1283_ptz_head_back_t* pod_1283_ptz_head_back)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1283_ptz_head_back->ack = ttalink_pod_1283_ptz_head_back_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN? msg->len : TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN;
        memset(pod_1283_ptz_head_back, 0, TTALINK_MSG_ID_POD_1283_PTZ_HEAD_BACK_LEN);
    memcpy(pod_1283_ptz_head_back, _TTA_PAYLOAD(msg), len);
#endif
}
