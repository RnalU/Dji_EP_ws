#pragma once
// MESSAGE POD_1287_SET_PTZ_CALI PACKING

#define TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI 11287

TTAPACKED(
typedef struct __ttalink_pod_1287_set_ptz_cali_t {
 uint8_t ack; /*< .*/
}) ttalink_pod_1287_set_ptz_cali_t;

#define TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN 1
#define TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN 1
#define TTALINK_MSG_ID_11287_LEN 1
#define TTALINK_MSG_ID_11287_MIN_LEN 1

#define TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC 62
#define TTALINK_MSG_ID_11287_CRC 62



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1287_SET_PTZ_CALI { \
    11287, \
    "POD_1287_SET_PTZ_CALI", \
    1, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1287_set_ptz_cali_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1287_SET_PTZ_CALI { \
    "POD_1287_SET_PTZ_CALI", \
    1, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1287_set_ptz_cali_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1287_set_ptz_cali_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN];
    _tta_put_uint8_t(buf, 0, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN);
#else
    ttalink_pod_1287_set_ptz_cali_t packet;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC, nocrc);
}

/**
 * @brief Pack a pod_1287_set_ptz_cali message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack)
{
    return _ttalink_pod_1287_set_ptz_cali_pack(dst_addr, src_addr, msg,  ack, false);
}

/**
 * @brief Pack a pod_1287_set_ptz_cali message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack)
{
    return _ttalink_pod_1287_set_ptz_cali_pack(dst_addr, src_addr, msg,  ack, true);
}


static inline uint16_t _ttalink_pod_1287_set_ptz_cali_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN];
    _tta_put_uint8_t(buf, 0, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN);
#else
    ttalink_pod_1287_set_ptz_cali_t packet;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC, nocrc);
}

/**
 * @brief Pack a pod_1287_set_ptz_cali message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack)
{
    return _ttalink_pod_1287_set_ptz_cali_pack_chan(dst_addr, src_addr, chan, msg,  ack, false);
}

/**
 * @brief Pack a pod_1287_set_ptz_cali message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack)
{
    return _ttalink_pod_1287_set_ptz_cali_pack_chan(dst_addr, src_addr, chan, msg,  ack, true);
}


static inline uint16_t _ttalink_pod_1287_set_ptz_cali_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1287_set_ptz_cali_pack_nocrc(dst_addr, src_addr, msg, pod_1287_set_ptz_cali->ack);
    }else{
        return ttalink_pod_1287_set_ptz_cali_pack(dst_addr, src_addr, msg, pod_1287_set_ptz_cali->ack);
    }
    
}

/**
 * @brief Encode a pod_1287_set_ptz_cali struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1287_set_ptz_cali C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali)
{
    return _ttalink_pod_1287_set_ptz_cali_encode(dst_addr, src_addr, msg, pod_1287_set_ptz_cali, false);
}

/**
 * @brief Encode a pod_1287_set_ptz_cali struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1287_set_ptz_cali C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali)
{
    return _ttalink_pod_1287_set_ptz_cali_encode(dst_addr, src_addr, msg, pod_1287_set_ptz_cali, true);
}


static inline uint16_t _ttalink_pod_1287_set_ptz_cali_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1287_set_ptz_cali_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1287_set_ptz_cali->ack);
    }else{
        return ttalink_pod_1287_set_ptz_cali_pack_chan(dst_addr, src_addr, chan, msg, pod_1287_set_ptz_cali->ack);
    }
}

/**
 * @brief Encode a pod_1287_set_ptz_cali struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1287_set_ptz_cali C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali)
{
    return _ttalink_pod_1287_set_ptz_cali_encode_chan(dst_addr, src_addr, chan, msg, pod_1287_set_ptz_cali, false);
}

/**
 * @brief Encode a pod_1287_set_ptz_cali struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1287_set_ptz_cali C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1287_set_ptz_cali_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali)
{
    return _ttalink_pod_1287_set_ptz_cali_encode_chan(dst_addr, src_addr, chan, msg, pod_1287_set_ptz_cali, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1287_set_ptz_cali_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN];
    _tta_put_uint8_t(buf, 0, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI, buf, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC, nocrc);
#else
    ttalink_pod_1287_set_ptz_cali_t packet;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI, (const char *)&packet, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1287_set_ptz_cali message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 */
static inline void ttalink_pod_1287_set_ptz_cali_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack)
{
    _ttalink_pod_1287_set_ptz_cali_send(dst_addr, src_addr, chan, ack, false);
}

/**
 * @brief Send a pod_1287_set_ptz_cali message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 */
static inline void ttalink_pod_1287_set_ptz_cali_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack)
{
    _ttalink_pod_1287_set_ptz_cali_send(dst_addr, src_addr, chan, ack, true);
}


static inline void _ttalink_pod_1287_set_ptz_cali_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1287_set_ptz_cali_send_nocrc(dst_addr, src_addr, chan, pod_1287_set_ptz_cali->ack);
    }else{
        ttalink_pod_1287_set_ptz_cali_send(dst_addr, src_addr, chan, pod_1287_set_ptz_cali->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI, (const char *)pod_1287_set_ptz_cali, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1287_set_ptz_cali message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1287_set_ptz_cali_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali)
{
    _ttalink_pod_1287_set_ptz_cali_send_struct(dst_addr, src_addr, chan, pod_1287_set_ptz_cali, false);
}

/**
 * @brief Send a pod_1287_set_ptz_cali message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1287_set_ptz_cali_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali)
{
    _ttalink_pod_1287_set_ptz_cali_send_struct(dst_addr, src_addr, chan, pod_1287_set_ptz_cali, true);
}

#if TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1287_set_ptz_cali_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI, buf, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC, nocrc);
#else
    ttalink_pod_1287_set_ptz_cali_t *packet = (ttalink_pod_1287_set_ptz_cali_t *)msgbuf;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI, (const char *)packet, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_MIN_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1287_set_ptz_cali_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack)
{
    _ttalink_pod_1287_set_ptz_cali_send_buf(dst_addr, src_addr, msgbuf, chan, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1287_set_ptz_cali_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack)
{
    _ttalink_pod_1287_set_ptz_cali_send_buf(dst_addr, src_addr, msgbuf, chan, ack, true);
}
#endif

#endif

// MESSAGE POD_1287_SET_PTZ_CALI UNPACKING


/**
 * @brief Get field ack from pod_1287_set_ptz_cali message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1287_set_ptz_cali_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a pod_1287_set_ptz_cali message into a struct
 *
 * @param msg The message to decode
 * @param pod_1287_set_ptz_cali C-struct to decode the message contents into
 */
static inline void ttalink_pod_1287_set_ptz_cali_decode(const ttalink_message_t* msg, ttalink_pod_1287_set_ptz_cali_t* pod_1287_set_ptz_cali)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1287_set_ptz_cali->ack = ttalink_pod_1287_set_ptz_cali_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN? msg->len : TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN;
        memset(pod_1287_set_ptz_cali, 0, TTALINK_MSG_ID_POD_1287_SET_PTZ_CALI_LEN);
    memcpy(pod_1287_set_ptz_cali, _TTA_PAYLOAD(msg), len);
#endif
}
