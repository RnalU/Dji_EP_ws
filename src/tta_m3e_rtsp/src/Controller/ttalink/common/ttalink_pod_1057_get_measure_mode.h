#pragma once
// MESSAGE POD_1057_GET_MEASURE_MODE PACKING

#define TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE 11057

TTAPACKED(
typedef struct __ttalink_pod_1057_get_measure_mode_t {
 uint8_t ack; /*< .*/
 uint8_t ack_mode; /*<  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.*/
}) ttalink_pod_1057_get_measure_mode_t;

#define TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN 2
#define TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN 2
#define TTALINK_MSG_ID_11057_LEN 2
#define TTALINK_MSG_ID_11057_MIN_LEN 2

#define TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC 4
#define TTALINK_MSG_ID_11057_CRC 4



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1057_GET_MEASURE_MODE { \
    11057, \
    "POD_1057_GET_MEASURE_MODE", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1057_get_measure_mode_t, ack) }, \
         { "ack_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1057_get_measure_mode_t, ack_mode) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1057_GET_MEASURE_MODE { \
    "POD_1057_GET_MEASURE_MODE", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1057_get_measure_mode_t, ack) }, \
         { "ack_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1057_get_measure_mode_t, ack_mode) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1057_get_measure_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN);
#else
    ttalink_pod_1057_get_measure_mode_t packet;
    packet.ack = ack;
    packet.ack_mode = ack_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1057_get_measure_mode message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_mode  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_mode)
{
    return _ttalink_pod_1057_get_measure_mode_pack(dst_addr, src_addr, msg,  ack, ack_mode, false);
}

/**
 * @brief Pack a pod_1057_get_measure_mode message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_mode  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_mode)
{
    return _ttalink_pod_1057_get_measure_mode_pack(dst_addr, src_addr, msg,  ack, ack_mode, true);
}


static inline uint16_t _ttalink_pod_1057_get_measure_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN);
#else
    ttalink_pod_1057_get_measure_mode_t packet;
    packet.ack = ack;
    packet.ack_mode = ack_mode;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1057_get_measure_mode message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_mode  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_mode)
{
    return _ttalink_pod_1057_get_measure_mode_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_mode, false);
}

/**
 * @brief Pack a pod_1057_get_measure_mode message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_mode  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_mode)
{
    return _ttalink_pod_1057_get_measure_mode_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_mode, true);
}


static inline uint16_t _ttalink_pod_1057_get_measure_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1057_get_measure_mode_pack_nocrc(dst_addr, src_addr, msg, pod_1057_get_measure_mode->ack, pod_1057_get_measure_mode->ack_mode);
    }else{
        return ttalink_pod_1057_get_measure_mode_pack(dst_addr, src_addr, msg, pod_1057_get_measure_mode->ack, pod_1057_get_measure_mode->ack_mode);
    }
    
}

/**
 * @brief Encode a pod_1057_get_measure_mode struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1057_get_measure_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode)
{
    return _ttalink_pod_1057_get_measure_mode_encode(dst_addr, src_addr, msg, pod_1057_get_measure_mode, false);
}

/**
 * @brief Encode a pod_1057_get_measure_mode struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1057_get_measure_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode)
{
    return _ttalink_pod_1057_get_measure_mode_encode(dst_addr, src_addr, msg, pod_1057_get_measure_mode, true);
}


static inline uint16_t _ttalink_pod_1057_get_measure_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1057_get_measure_mode_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1057_get_measure_mode->ack, pod_1057_get_measure_mode->ack_mode);
    }else{
        return ttalink_pod_1057_get_measure_mode_pack_chan(dst_addr, src_addr, chan, msg, pod_1057_get_measure_mode->ack, pod_1057_get_measure_mode->ack_mode);
    }
}

/**
 * @brief Encode a pod_1057_get_measure_mode struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1057_get_measure_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode)
{
    return _ttalink_pod_1057_get_measure_mode_encode_chan(dst_addr, src_addr, chan, msg, pod_1057_get_measure_mode, false);
}

/**
 * @brief Encode a pod_1057_get_measure_mode struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1057_get_measure_mode C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1057_get_measure_mode_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode)
{
    return _ttalink_pod_1057_get_measure_mode_encode_chan(dst_addr, src_addr, chan, msg, pod_1057_get_measure_mode, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1057_get_measure_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE, buf, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC, nocrc);
#else
    ttalink_pod_1057_get_measure_mode_t packet;
    packet.ack = ack;
    packet.ack_mode = ack_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE, (const char *)&packet, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1057_get_measure_mode message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_mode  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.
 */
static inline void ttalink_pod_1057_get_measure_mode_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_mode)
{
    _ttalink_pod_1057_get_measure_mode_send(dst_addr, src_addr, chan, ack, ack_mode, false);
}

/**
 * @brief Send a pod_1057_get_measure_mode message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_mode  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.
 */
static inline void ttalink_pod_1057_get_measure_mode_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_mode)
{
    _ttalink_pod_1057_get_measure_mode_send(dst_addr, src_addr, chan, ack, ack_mode, true);
}


static inline void _ttalink_pod_1057_get_measure_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1057_get_measure_mode_send_nocrc(dst_addr, src_addr, chan, pod_1057_get_measure_mode->ack, pod_1057_get_measure_mode->ack_mode);
    }else{
        ttalink_pod_1057_get_measure_mode_send(dst_addr, src_addr, chan, pod_1057_get_measure_mode->ack, pod_1057_get_measure_mode->ack_mode);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE, (const char *)pod_1057_get_measure_mode, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1057_get_measure_mode message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1057_get_measure_mode_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode)
{
    _ttalink_pod_1057_get_measure_mode_send_struct(dst_addr, src_addr, chan, pod_1057_get_measure_mode, false);
}

/**
 * @brief Send a pod_1057_get_measure_mode message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1057_get_measure_mode_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode)
{
    _ttalink_pod_1057_get_measure_mode_send_struct(dst_addr, src_addr, chan, pod_1057_get_measure_mode, true);
}

#if TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1057_get_measure_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_mode);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE, buf, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC, nocrc);
#else
    ttalink_pod_1057_get_measure_mode_t *packet = (ttalink_pod_1057_get_measure_mode_t *)msgbuf;
    packet->ack = ack;
    packet->ack_mode = ack_mode;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE, (const char *)packet, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_MIN_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1057_get_measure_mode_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_mode)
{
    _ttalink_pod_1057_get_measure_mode_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_mode, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1057_get_measure_mode_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_mode)
{
    _ttalink_pod_1057_get_measure_mode_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_mode, true);
}
#endif

#endif

// MESSAGE POD_1057_GET_MEASURE_MODE UNPACKING


/**
 * @brief Get field ack from pod_1057_get_measure_mode message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1057_get_measure_mode_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_mode from pod_1057_get_measure_mode message
 *
 * @return  Measurement mode
												0 = center measurement
												1 = average measurement
												2= point measurement.
 */
static inline uint8_t ttalink_pod_1057_get_measure_mode_get_ack_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a pod_1057_get_measure_mode message into a struct
 *
 * @param msg The message to decode
 * @param pod_1057_get_measure_mode C-struct to decode the message contents into
 */
static inline void ttalink_pod_1057_get_measure_mode_decode(const ttalink_message_t* msg, ttalink_pod_1057_get_measure_mode_t* pod_1057_get_measure_mode)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1057_get_measure_mode->ack = ttalink_pod_1057_get_measure_mode_get_ack(msg);
    pod_1057_get_measure_mode->ack_mode = ttalink_pod_1057_get_measure_mode_get_ack_mode(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN? msg->len : TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN;
        memset(pod_1057_get_measure_mode, 0, TTALINK_MSG_ID_POD_1057_GET_MEASURE_MODE_LEN);
    memcpy(pod_1057_get_measure_mode, _TTA_PAYLOAD(msg), len);
#endif
}
