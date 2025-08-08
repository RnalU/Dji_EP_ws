#pragma once
// MESSAGE POD_1286_CTRL_PTZ_ANGLE PACKING

#define TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE 11286

TTAPACKED(
typedef struct __ttalink_pod_1286_ctrl_ptz_angle_t {
 float pitch_angle; /*< .*/
 float roll_angle; /*< .*/
 float yaw_angle; /*< .*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1286_ctrl_ptz_angle_t;

#define TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN 13
#define TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN 13
#define TTALINK_MSG_ID_11286_LEN 13
#define TTALINK_MSG_ID_11286_MIN_LEN 13

#define TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC 128
#define TTALINK_MSG_ID_11286_CRC 128



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1286_CTRL_PTZ_ANGLE { \
    11286, \
    "POD_1286_CTRL_PTZ_ANGLE", \
    4, \
    {  { "pitch_angle", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, pitch_angle) }, \
         { "roll_angle", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, roll_angle) }, \
         { "yaw_angle", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, yaw_angle) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1286_CTRL_PTZ_ANGLE { \
    "POD_1286_CTRL_PTZ_ANGLE", \
    4, \
    {  { "pitch_angle", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, pitch_angle) }, \
         { "roll_angle", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, roll_angle) }, \
         { "yaw_angle", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, yaw_angle) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_pod_1286_ctrl_ptz_angle_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1286_ctrl_ptz_angle_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN];
    _tta_put_float(buf, 0, pitch_angle);
    _tta_put_float(buf, 4, roll_angle);
    _tta_put_float(buf, 8, yaw_angle);
    _tta_put_uint8_t(buf, 12, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN);
#else
    ttalink_pod_1286_ctrl_ptz_angle_t packet;
    packet.pitch_angle = pitch_angle;
    packet.roll_angle = roll_angle;
    packet.yaw_angle = yaw_angle;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1286_ctrl_ptz_angle message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param pitch_angle .
 * @param roll_angle .
 * @param yaw_angle .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_pack(dst_addr, src_addr, msg,  pitch_angle, roll_angle, yaw_angle, ack, false);
}

/**
 * @brief Pack a pod_1286_ctrl_ptz_angle message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param pitch_angle .
 * @param roll_angle .
 * @param yaw_angle .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_pack(dst_addr, src_addr, msg,  pitch_angle, roll_angle, yaw_angle, ack, true);
}


static inline uint16_t _ttalink_pod_1286_ctrl_ptz_angle_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float pitch_angle,float roll_angle,float yaw_angle,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN];
    _tta_put_float(buf, 0, pitch_angle);
    _tta_put_float(buf, 4, roll_angle);
    _tta_put_float(buf, 8, yaw_angle);
    _tta_put_uint8_t(buf, 12, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN);
#else
    ttalink_pod_1286_ctrl_ptz_angle_t packet;
    packet.pitch_angle = pitch_angle;
    packet.roll_angle = roll_angle;
    packet.yaw_angle = yaw_angle;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1286_ctrl_ptz_angle message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pitch_angle .
 * @param roll_angle .
 * @param yaw_angle .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float pitch_angle,float roll_angle,float yaw_angle,uint8_t ack)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_pack_chan(dst_addr, src_addr, chan, msg,  pitch_angle, roll_angle, yaw_angle, ack, false);
}

/**
 * @brief Pack a pod_1286_ctrl_ptz_angle message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pitch_angle .
 * @param roll_angle .
 * @param yaw_angle .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float pitch_angle,float roll_angle,float yaw_angle,uint8_t ack)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_pack_chan(dst_addr, src_addr, chan, msg,  pitch_angle, roll_angle, yaw_angle, ack, true);
}


static inline uint16_t _ttalink_pod_1286_ctrl_ptz_angle_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1286_ctrl_ptz_angle_pack_nocrc(dst_addr, src_addr, msg, pod_1286_ctrl_ptz_angle->pitch_angle, pod_1286_ctrl_ptz_angle->roll_angle, pod_1286_ctrl_ptz_angle->yaw_angle, pod_1286_ctrl_ptz_angle->ack);
    }else{
        return ttalink_pod_1286_ctrl_ptz_angle_pack(dst_addr, src_addr, msg, pod_1286_ctrl_ptz_angle->pitch_angle, pod_1286_ctrl_ptz_angle->roll_angle, pod_1286_ctrl_ptz_angle->yaw_angle, pod_1286_ctrl_ptz_angle->ack);
    }
    
}

/**
 * @brief Encode a pod_1286_ctrl_ptz_angle struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1286_ctrl_ptz_angle C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_encode(dst_addr, src_addr, msg, pod_1286_ctrl_ptz_angle, false);
}

/**
 * @brief Encode a pod_1286_ctrl_ptz_angle struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1286_ctrl_ptz_angle C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_encode(dst_addr, src_addr, msg, pod_1286_ctrl_ptz_angle, true);
}


static inline uint16_t _ttalink_pod_1286_ctrl_ptz_angle_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1286_ctrl_ptz_angle_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1286_ctrl_ptz_angle->pitch_angle, pod_1286_ctrl_ptz_angle->roll_angle, pod_1286_ctrl_ptz_angle->yaw_angle, pod_1286_ctrl_ptz_angle->ack);
    }else{
        return ttalink_pod_1286_ctrl_ptz_angle_pack_chan(dst_addr, src_addr, chan, msg, pod_1286_ctrl_ptz_angle->pitch_angle, pod_1286_ctrl_ptz_angle->roll_angle, pod_1286_ctrl_ptz_angle->yaw_angle, pod_1286_ctrl_ptz_angle->ack);
    }
}

/**
 * @brief Encode a pod_1286_ctrl_ptz_angle struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1286_ctrl_ptz_angle C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_encode_chan(dst_addr, src_addr, chan, msg, pod_1286_ctrl_ptz_angle, false);
}

/**
 * @brief Encode a pod_1286_ctrl_ptz_angle struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1286_ctrl_ptz_angle C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1286_ctrl_ptz_angle_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle)
{
    return _ttalink_pod_1286_ctrl_ptz_angle_encode_chan(dst_addr, src_addr, chan, msg, pod_1286_ctrl_ptz_angle, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1286_ctrl_ptz_angle_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN];
    _tta_put_float(buf, 0, pitch_angle);
    _tta_put_float(buf, 4, roll_angle);
    _tta_put_float(buf, 8, yaw_angle);
    _tta_put_uint8_t(buf, 12, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE, buf, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC, nocrc);
#else
    ttalink_pod_1286_ctrl_ptz_angle_t packet;
    packet.pitch_angle = pitch_angle;
    packet.roll_angle = roll_angle;
    packet.yaw_angle = yaw_angle;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE, (const char *)&packet, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1286_ctrl_ptz_angle message
 * @param chan TTAlink channel to send the message
 *
 * @param pitch_angle .
 * @param roll_angle .
 * @param yaw_angle .
 * @param ack .
 */
static inline void ttalink_pod_1286_ctrl_ptz_angle_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack)
{
    _ttalink_pod_1286_ctrl_ptz_angle_send(dst_addr, src_addr, chan, pitch_angle, roll_angle, yaw_angle, ack, false);
}

/**
 * @brief Send a pod_1286_ctrl_ptz_angle message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param pitch_angle .
 * @param roll_angle .
 * @param yaw_angle .
 * @param ack .
 */
static inline void ttalink_pod_1286_ctrl_ptz_angle_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack)
{
    _ttalink_pod_1286_ctrl_ptz_angle_send(dst_addr, src_addr, chan, pitch_angle, roll_angle, yaw_angle, ack, true);
}


static inline void _ttalink_pod_1286_ctrl_ptz_angle_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1286_ctrl_ptz_angle_send_nocrc(dst_addr, src_addr, chan, pod_1286_ctrl_ptz_angle->pitch_angle, pod_1286_ctrl_ptz_angle->roll_angle, pod_1286_ctrl_ptz_angle->yaw_angle, pod_1286_ctrl_ptz_angle->ack);
    }else{
        ttalink_pod_1286_ctrl_ptz_angle_send(dst_addr, src_addr, chan, pod_1286_ctrl_ptz_angle->pitch_angle, pod_1286_ctrl_ptz_angle->roll_angle, pod_1286_ctrl_ptz_angle->yaw_angle, pod_1286_ctrl_ptz_angle->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE, (const char *)pod_1286_ctrl_ptz_angle, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1286_ctrl_ptz_angle message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1286_ctrl_ptz_angle_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle)
{
    _ttalink_pod_1286_ctrl_ptz_angle_send_struct(dst_addr, src_addr, chan, pod_1286_ctrl_ptz_angle, false);
}

/**
 * @brief Send a pod_1286_ctrl_ptz_angle message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1286_ctrl_ptz_angle_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle)
{
    _ttalink_pod_1286_ctrl_ptz_angle_send_struct(dst_addr, src_addr, chan, pod_1286_ctrl_ptz_angle, true);
}

#if TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1286_ctrl_ptz_angle_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 0, pitch_angle);
    _tta_put_float(buf, 4, roll_angle);
    _tta_put_float(buf, 8, yaw_angle);
    _tta_put_uint8_t(buf, 12, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE, buf, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC, nocrc);
#else
    ttalink_pod_1286_ctrl_ptz_angle_t *packet = (ttalink_pod_1286_ctrl_ptz_angle_t *)msgbuf;
    packet->pitch_angle = pitch_angle;
    packet->roll_angle = roll_angle;
    packet->yaw_angle = yaw_angle;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE, (const char *)packet, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_MIN_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1286_ctrl_ptz_angle_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack)
{
    _ttalink_pod_1286_ctrl_ptz_angle_send_buf(dst_addr, src_addr, msgbuf, chan, pitch_angle, roll_angle, yaw_angle, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1286_ctrl_ptz_angle_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float pitch_angle, float roll_angle, float yaw_angle, uint8_t ack)
{
    _ttalink_pod_1286_ctrl_ptz_angle_send_buf(dst_addr, src_addr, msgbuf, chan, pitch_angle, roll_angle, yaw_angle, ack, true);
}
#endif

#endif

// MESSAGE POD_1286_CTRL_PTZ_ANGLE UNPACKING


/**
 * @brief Get field pitch_angle from pod_1286_ctrl_ptz_angle message
 *
 * @return .
 */
static inline float ttalink_pod_1286_ctrl_ptz_angle_get_pitch_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  0);
}

/**
 * @brief Get field roll_angle from pod_1286_ctrl_ptz_angle message
 *
 * @return .
 */
static inline float ttalink_pod_1286_ctrl_ptz_angle_get_roll_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw_angle from pod_1286_ctrl_ptz_angle message
 *
 * @return .
 */
static inline float ttalink_pod_1286_ctrl_ptz_angle_get_yaw_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field ack from pod_1286_ctrl_ptz_angle message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1286_ctrl_ptz_angle_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Decode a pod_1286_ctrl_ptz_angle message into a struct
 *
 * @param msg The message to decode
 * @param pod_1286_ctrl_ptz_angle C-struct to decode the message contents into
 */
static inline void ttalink_pod_1286_ctrl_ptz_angle_decode(const ttalink_message_t* msg, ttalink_pod_1286_ctrl_ptz_angle_t* pod_1286_ctrl_ptz_angle)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1286_ctrl_ptz_angle->pitch_angle = ttalink_pod_1286_ctrl_ptz_angle_get_pitch_angle(msg);
    pod_1286_ctrl_ptz_angle->roll_angle = ttalink_pod_1286_ctrl_ptz_angle_get_roll_angle(msg);
    pod_1286_ctrl_ptz_angle->yaw_angle = ttalink_pod_1286_ctrl_ptz_angle_get_yaw_angle(msg);
    pod_1286_ctrl_ptz_angle->ack = ttalink_pod_1286_ctrl_ptz_angle_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN? msg->len : TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN;
        memset(pod_1286_ctrl_ptz_angle, 0, TTALINK_MSG_ID_POD_1286_CTRL_PTZ_ANGLE_LEN);
    memcpy(pod_1286_ctrl_ptz_angle, _TTA_PAYLOAD(msg), len);
#endif
}
