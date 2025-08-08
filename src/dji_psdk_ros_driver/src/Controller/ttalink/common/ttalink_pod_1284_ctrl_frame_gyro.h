#pragma once
// MESSAGE POD_1284_CTRL_FRAME_GYRO PACKING

#define TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO 11284

TTAPACKED(
typedef struct __ttalink_pod_1284_ctrl_frame_gyro_t {
 int16_t pitch_gyro; /*< pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).*/
 int16_t roll_gyro; /*< roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).*/
 int16_t yaw_gyro; /*< yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).*/
 uint8_t ack_action; /*< Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1284_ctrl_frame_gyro_t;

#define TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN 8
#define TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN 8
#define TTALINK_MSG_ID_11284_LEN 8
#define TTALINK_MSG_ID_11284_MIN_LEN 8

#define TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC 247
#define TTALINK_MSG_ID_11284_CRC 247



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1284_CTRL_FRAME_GYRO { \
    11284, \
    "POD_1284_CTRL_FRAME_GYRO", \
    5, \
    {  { "pitch_gyro", NULL, TTALINK_TYPE_INT16_T, 0, 0, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, pitch_gyro) }, \
         { "roll_gyro", NULL, TTALINK_TYPE_INT16_T, 0, 2, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, roll_gyro) }, \
         { "yaw_gyro", NULL, TTALINK_TYPE_INT16_T, 0, 4, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, yaw_gyro) }, \
         { "ack_action", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, ack_action) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1284_CTRL_FRAME_GYRO { \
    "POD_1284_CTRL_FRAME_GYRO", \
    5, \
    {  { "pitch_gyro", NULL, TTALINK_TYPE_INT16_T, 0, 0, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, pitch_gyro) }, \
         { "roll_gyro", NULL, TTALINK_TYPE_INT16_T, 0, 2, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, roll_gyro) }, \
         { "yaw_gyro", NULL, TTALINK_TYPE_INT16_T, 0, 4, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, yaw_gyro) }, \
         { "ack_action", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, ack_action) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_pod_1284_ctrl_frame_gyro_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1284_ctrl_frame_gyro_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN];
    _tta_put_int16_t(buf, 0, pitch_gyro);
    _tta_put_int16_t(buf, 2, roll_gyro);
    _tta_put_int16_t(buf, 4, yaw_gyro);
    _tta_put_uint8_t(buf, 6, ack_action);
    _tta_put_uint8_t(buf, 7, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN);
#else
    ttalink_pod_1284_ctrl_frame_gyro_t packet;
    packet.pitch_gyro = pitch_gyro;
    packet.roll_gyro = roll_gyro;
    packet.yaw_gyro = yaw_gyro;
    packet.ack_action = ack_action;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC, nocrc);
}

/**
 * @brief Pack a pod_1284_ctrl_frame_gyro message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param pitch_gyro pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param roll_gyro roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param yaw_gyro yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param ack_action Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_pack(dst_addr, src_addr, msg,  pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, false);
}

/**
 * @brief Pack a pod_1284_ctrl_frame_gyro message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param pitch_gyro pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param roll_gyro roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param yaw_gyro yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param ack_action Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_pack(dst_addr, src_addr, msg,  pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, true);
}


static inline uint16_t _ttalink_pod_1284_ctrl_frame_gyro_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int16_t pitch_gyro,int16_t roll_gyro,int16_t yaw_gyro,uint8_t ack_action,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN];
    _tta_put_int16_t(buf, 0, pitch_gyro);
    _tta_put_int16_t(buf, 2, roll_gyro);
    _tta_put_int16_t(buf, 4, yaw_gyro);
    _tta_put_uint8_t(buf, 6, ack_action);
    _tta_put_uint8_t(buf, 7, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN);
#else
    ttalink_pod_1284_ctrl_frame_gyro_t packet;
    packet.pitch_gyro = pitch_gyro;
    packet.roll_gyro = roll_gyro;
    packet.yaw_gyro = yaw_gyro;
    packet.ack_action = ack_action;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC, nocrc);
}

/**
 * @brief Pack a pod_1284_ctrl_frame_gyro message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pitch_gyro pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param roll_gyro roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param yaw_gyro yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param ack_action Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int16_t pitch_gyro,int16_t roll_gyro,int16_t yaw_gyro,uint8_t ack_action,uint8_t ack)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_pack_chan(dst_addr, src_addr, chan, msg,  pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, false);
}

/**
 * @brief Pack a pod_1284_ctrl_frame_gyro message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pitch_gyro pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param roll_gyro roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param yaw_gyro yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param ack_action Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int16_t pitch_gyro,int16_t roll_gyro,int16_t yaw_gyro,uint8_t ack_action,uint8_t ack)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_pack_chan(dst_addr, src_addr, chan, msg,  pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, true);
}


static inline uint16_t _ttalink_pod_1284_ctrl_frame_gyro_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1284_ctrl_frame_gyro_pack_nocrc(dst_addr, src_addr, msg, pod_1284_ctrl_frame_gyro->pitch_gyro, pod_1284_ctrl_frame_gyro->roll_gyro, pod_1284_ctrl_frame_gyro->yaw_gyro, pod_1284_ctrl_frame_gyro->ack_action, pod_1284_ctrl_frame_gyro->ack);
    }else{
        return ttalink_pod_1284_ctrl_frame_gyro_pack(dst_addr, src_addr, msg, pod_1284_ctrl_frame_gyro->pitch_gyro, pod_1284_ctrl_frame_gyro->roll_gyro, pod_1284_ctrl_frame_gyro->yaw_gyro, pod_1284_ctrl_frame_gyro->ack_action, pod_1284_ctrl_frame_gyro->ack);
    }
    
}

/**
 * @brief Encode a pod_1284_ctrl_frame_gyro struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1284_ctrl_frame_gyro C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_encode(dst_addr, src_addr, msg, pod_1284_ctrl_frame_gyro, false);
}

/**
 * @brief Encode a pod_1284_ctrl_frame_gyro struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1284_ctrl_frame_gyro C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_encode(dst_addr, src_addr, msg, pod_1284_ctrl_frame_gyro, true);
}


static inline uint16_t _ttalink_pod_1284_ctrl_frame_gyro_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1284_ctrl_frame_gyro_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1284_ctrl_frame_gyro->pitch_gyro, pod_1284_ctrl_frame_gyro->roll_gyro, pod_1284_ctrl_frame_gyro->yaw_gyro, pod_1284_ctrl_frame_gyro->ack_action, pod_1284_ctrl_frame_gyro->ack);
    }else{
        return ttalink_pod_1284_ctrl_frame_gyro_pack_chan(dst_addr, src_addr, chan, msg, pod_1284_ctrl_frame_gyro->pitch_gyro, pod_1284_ctrl_frame_gyro->roll_gyro, pod_1284_ctrl_frame_gyro->yaw_gyro, pod_1284_ctrl_frame_gyro->ack_action, pod_1284_ctrl_frame_gyro->ack);
    }
}

/**
 * @brief Encode a pod_1284_ctrl_frame_gyro struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1284_ctrl_frame_gyro C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_encode_chan(dst_addr, src_addr, chan, msg, pod_1284_ctrl_frame_gyro, false);
}

/**
 * @brief Encode a pod_1284_ctrl_frame_gyro struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1284_ctrl_frame_gyro C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1284_ctrl_frame_gyro_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro)
{
    return _ttalink_pod_1284_ctrl_frame_gyro_encode_chan(dst_addr, src_addr, chan, msg, pod_1284_ctrl_frame_gyro, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1284_ctrl_frame_gyro_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN];
    _tta_put_int16_t(buf, 0, pitch_gyro);
    _tta_put_int16_t(buf, 2, roll_gyro);
    _tta_put_int16_t(buf, 4, yaw_gyro);
    _tta_put_uint8_t(buf, 6, ack_action);
    _tta_put_uint8_t(buf, 7, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO, buf, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC, nocrc);
#else
    ttalink_pod_1284_ctrl_frame_gyro_t packet;
    packet.pitch_gyro = pitch_gyro;
    packet.roll_gyro = roll_gyro;
    packet.yaw_gyro = yaw_gyro;
    packet.ack_action = ack_action;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO, (const char *)&packet, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1284_ctrl_frame_gyro message
 * @param chan TTAlink channel to send the message
 *
 * @param pitch_gyro pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param roll_gyro roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param yaw_gyro yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param ack_action Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.
 * @param ack .
 */
static inline void ttalink_pod_1284_ctrl_frame_gyro_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack)
{
    _ttalink_pod_1284_ctrl_frame_gyro_send(dst_addr, src_addr, chan, pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, false);
}

/**
 * @brief Send a pod_1284_ctrl_frame_gyro message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param pitch_gyro pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param roll_gyro roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param yaw_gyro yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 * @param ack_action Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.
 * @param ack .
 */
static inline void ttalink_pod_1284_ctrl_frame_gyro_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack)
{
    _ttalink_pod_1284_ctrl_frame_gyro_send(dst_addr, src_addr, chan, pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, true);
}


static inline void _ttalink_pod_1284_ctrl_frame_gyro_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1284_ctrl_frame_gyro_send_nocrc(dst_addr, src_addr, chan, pod_1284_ctrl_frame_gyro->pitch_gyro, pod_1284_ctrl_frame_gyro->roll_gyro, pod_1284_ctrl_frame_gyro->yaw_gyro, pod_1284_ctrl_frame_gyro->ack_action, pod_1284_ctrl_frame_gyro->ack);
    }else{
        ttalink_pod_1284_ctrl_frame_gyro_send(dst_addr, src_addr, chan, pod_1284_ctrl_frame_gyro->pitch_gyro, pod_1284_ctrl_frame_gyro->roll_gyro, pod_1284_ctrl_frame_gyro->yaw_gyro, pod_1284_ctrl_frame_gyro->ack_action, pod_1284_ctrl_frame_gyro->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO, (const char *)pod_1284_ctrl_frame_gyro, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1284_ctrl_frame_gyro message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1284_ctrl_frame_gyro_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro)
{
    _ttalink_pod_1284_ctrl_frame_gyro_send_struct(dst_addr, src_addr, chan, pod_1284_ctrl_frame_gyro, false);
}

/**
 * @brief Send a pod_1284_ctrl_frame_gyro message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1284_ctrl_frame_gyro_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro)
{
    _ttalink_pod_1284_ctrl_frame_gyro_send_struct(dst_addr, src_addr, chan, pod_1284_ctrl_frame_gyro, true);
}

#if TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1284_ctrl_frame_gyro_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int16_t(buf, 0, pitch_gyro);
    _tta_put_int16_t(buf, 2, roll_gyro);
    _tta_put_int16_t(buf, 4, yaw_gyro);
    _tta_put_uint8_t(buf, 6, ack_action);
    _tta_put_uint8_t(buf, 7, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO, buf, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC, nocrc);
#else
    ttalink_pod_1284_ctrl_frame_gyro_t *packet = (ttalink_pod_1284_ctrl_frame_gyro_t *)msgbuf;
    packet->pitch_gyro = pitch_gyro;
    packet->roll_gyro = roll_gyro;
    packet->yaw_gyro = yaw_gyro;
    packet->ack_action = ack_action;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO, (const char *)packet, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_MIN_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1284_ctrl_frame_gyro_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack)
{
    _ttalink_pod_1284_ctrl_frame_gyro_send_buf(dst_addr, src_addr, msgbuf, chan, pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1284_ctrl_frame_gyro_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int16_t pitch_gyro, int16_t roll_gyro, int16_t yaw_gyro, uint8_t ack_action, uint8_t ack)
{
    _ttalink_pod_1284_ctrl_frame_gyro_send_buf(dst_addr, src_addr, msgbuf, chan, pitch_gyro, roll_gyro, yaw_gyro, ack_action, ack, true);
}
#endif

#endif

// MESSAGE POD_1284_CTRL_FRAME_GYRO UNPACKING


/**
 * @brief Get field pitch_gyro from pod_1284_ctrl_frame_gyro message
 *
 * @return pitch_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 */
static inline int16_t ttalink_pod_1284_ctrl_frame_gyro_get_pitch_gyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field roll_gyro from pod_1284_ctrl_frame_gyro message
 *
 * @return roll_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 */
static inline int16_t ttalink_pod_1284_ctrl_frame_gyro_get_roll_gyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field yaw_gyro from pod_1284_ctrl_frame_gyro message
 *
 * @return yaw_gyro , in units of 0.1 degrees/s, in the range (-1800+1800).
 */
static inline int16_t ttalink_pod_1284_ctrl_frame_gyro_get_yaw_gyro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field ack_action from pod_1284_ctrl_frame_gyro message
 *
 * @return Cloud platform controls actions.
										0 : Stop control
										1 : Control Framework.
 */
static inline uint8_t ttalink_pod_1284_ctrl_frame_gyro_get_ack_action(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field ack from pod_1284_ctrl_frame_gyro message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1284_ctrl_frame_gyro_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a pod_1284_ctrl_frame_gyro message into a struct
 *
 * @param msg The message to decode
 * @param pod_1284_ctrl_frame_gyro C-struct to decode the message contents into
 */
static inline void ttalink_pod_1284_ctrl_frame_gyro_decode(const ttalink_message_t* msg, ttalink_pod_1284_ctrl_frame_gyro_t* pod_1284_ctrl_frame_gyro)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1284_ctrl_frame_gyro->pitch_gyro = ttalink_pod_1284_ctrl_frame_gyro_get_pitch_gyro(msg);
    pod_1284_ctrl_frame_gyro->roll_gyro = ttalink_pod_1284_ctrl_frame_gyro_get_roll_gyro(msg);
    pod_1284_ctrl_frame_gyro->yaw_gyro = ttalink_pod_1284_ctrl_frame_gyro_get_yaw_gyro(msg);
    pod_1284_ctrl_frame_gyro->ack_action = ttalink_pod_1284_ctrl_frame_gyro_get_ack_action(msg);
    pod_1284_ctrl_frame_gyro->ack = ttalink_pod_1284_ctrl_frame_gyro_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN? msg->len : TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN;
        memset(pod_1284_ctrl_frame_gyro, 0, TTALINK_MSG_ID_POD_1284_CTRL_FRAME_GYRO_LEN);
    memcpy(pod_1284_ctrl_frame_gyro, _TTA_PAYLOAD(msg), len);
#endif
}
