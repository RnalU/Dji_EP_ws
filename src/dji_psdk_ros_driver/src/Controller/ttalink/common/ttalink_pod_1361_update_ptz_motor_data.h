#pragma once
// MESSAGE POD_1361_UPDATE_PTZ_MOTOR_DATA PACKING

#define TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA 11361

TTAPACKED(
typedef struct __ttalink_pod_1361_update_ptz_motor_data_t {
 uint8_t ack; /*< .*/
 float motor_current; /*< .*/
 float motor_position; /*< .*/
 float reserve1; /*< .*/
 float reserve2; /*< .*/
}) ttalink_pod_1361_update_ptz_motor_data_t;

#define TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN 17
#define TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN 17
#define TTALINK_MSG_ID_11361_LEN 17
#define TTALINK_MSG_ID_11361_MIN_LEN 17

#define TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC 48
#define TTALINK_MSG_ID_11361_CRC 48



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1361_UPDATE_PTZ_MOTOR_DATA { \
    11361, \
    "POD_1361_UPDATE_PTZ_MOTOR_DATA", \
    5, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, ack) }, \
         { "motor_current", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, motor_current) }, \
         { "motor_position", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, motor_position) }, \
         { "reserve1", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, reserve1) }, \
         { "reserve2", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, reserve2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1361_UPDATE_PTZ_MOTOR_DATA { \
    "POD_1361_UPDATE_PTZ_MOTOR_DATA", \
    5, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, ack) }, \
         { "motor_current", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, motor_current) }, \
         { "motor_position", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, motor_position) }, \
         { "reserve1", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, reserve1) }, \
         { "reserve2", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_pod_1361_update_ptz_motor_data_t, reserve2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1361_update_ptz_motor_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, motor_current);
    _tta_put_float(buf, 5, motor_position);
    _tta_put_float(buf, 9, reserve1);
    _tta_put_float(buf, 13, reserve2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN);
#else
    ttalink_pod_1361_update_ptz_motor_data_t packet;
    packet.ack = ack;
    packet.motor_current = motor_current;
    packet.motor_position = motor_position;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1361_update_ptz_motor_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param motor_current .
 * @param motor_position .
 * @param reserve1 .
 * @param reserve2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2)
{
    return _ttalink_pod_1361_update_ptz_motor_data_pack(dst_addr, src_addr, msg,  ack, motor_current, motor_position, reserve1, reserve2, false);
}

/**
 * @brief Pack a pod_1361_update_ptz_motor_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param motor_current .
 * @param motor_position .
 * @param reserve1 .
 * @param reserve2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2)
{
    return _ttalink_pod_1361_update_ptz_motor_data_pack(dst_addr, src_addr, msg,  ack, motor_current, motor_position, reserve1, reserve2, true);
}


static inline uint16_t _ttalink_pod_1361_update_ptz_motor_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float motor_current,float motor_position,float reserve1,float reserve2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, motor_current);
    _tta_put_float(buf, 5, motor_position);
    _tta_put_float(buf, 9, reserve1);
    _tta_put_float(buf, 13, reserve2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN);
#else
    ttalink_pod_1361_update_ptz_motor_data_t packet;
    packet.ack = ack;
    packet.motor_current = motor_current;
    packet.motor_position = motor_position;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1361_update_ptz_motor_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param motor_current .
 * @param motor_position .
 * @param reserve1 .
 * @param reserve2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float motor_current,float motor_position,float reserve1,float reserve2)
{
    return _ttalink_pod_1361_update_ptz_motor_data_pack_chan(dst_addr, src_addr, chan, msg,  ack, motor_current, motor_position, reserve1, reserve2, false);
}

/**
 * @brief Pack a pod_1361_update_ptz_motor_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param motor_current .
 * @param motor_position .
 * @param reserve1 .
 * @param reserve2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float motor_current,float motor_position,float reserve1,float reserve2)
{
    return _ttalink_pod_1361_update_ptz_motor_data_pack_chan(dst_addr, src_addr, chan, msg,  ack, motor_current, motor_position, reserve1, reserve2, true);
}


static inline uint16_t _ttalink_pod_1361_update_ptz_motor_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1361_update_ptz_motor_data_pack_nocrc(dst_addr, src_addr, msg, pod_1361_update_ptz_motor_data->ack, pod_1361_update_ptz_motor_data->motor_current, pod_1361_update_ptz_motor_data->motor_position, pod_1361_update_ptz_motor_data->reserve1, pod_1361_update_ptz_motor_data->reserve2);
    }else{
        return ttalink_pod_1361_update_ptz_motor_data_pack(dst_addr, src_addr, msg, pod_1361_update_ptz_motor_data->ack, pod_1361_update_ptz_motor_data->motor_current, pod_1361_update_ptz_motor_data->motor_position, pod_1361_update_ptz_motor_data->reserve1, pod_1361_update_ptz_motor_data->reserve2);
    }
    
}

/**
 * @brief Encode a pod_1361_update_ptz_motor_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1361_update_ptz_motor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data)
{
    return _ttalink_pod_1361_update_ptz_motor_data_encode(dst_addr, src_addr, msg, pod_1361_update_ptz_motor_data, false);
}

/**
 * @brief Encode a pod_1361_update_ptz_motor_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1361_update_ptz_motor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data)
{
    return _ttalink_pod_1361_update_ptz_motor_data_encode(dst_addr, src_addr, msg, pod_1361_update_ptz_motor_data, true);
}


static inline uint16_t _ttalink_pod_1361_update_ptz_motor_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1361_update_ptz_motor_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1361_update_ptz_motor_data->ack, pod_1361_update_ptz_motor_data->motor_current, pod_1361_update_ptz_motor_data->motor_position, pod_1361_update_ptz_motor_data->reserve1, pod_1361_update_ptz_motor_data->reserve2);
    }else{
        return ttalink_pod_1361_update_ptz_motor_data_pack_chan(dst_addr, src_addr, chan, msg, pod_1361_update_ptz_motor_data->ack, pod_1361_update_ptz_motor_data->motor_current, pod_1361_update_ptz_motor_data->motor_position, pod_1361_update_ptz_motor_data->reserve1, pod_1361_update_ptz_motor_data->reserve2);
    }
}

/**
 * @brief Encode a pod_1361_update_ptz_motor_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1361_update_ptz_motor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data)
{
    return _ttalink_pod_1361_update_ptz_motor_data_encode_chan(dst_addr, src_addr, chan, msg, pod_1361_update_ptz_motor_data, false);
}

/**
 * @brief Encode a pod_1361_update_ptz_motor_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1361_update_ptz_motor_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1361_update_ptz_motor_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data)
{
    return _ttalink_pod_1361_update_ptz_motor_data_encode_chan(dst_addr, src_addr, chan, msg, pod_1361_update_ptz_motor_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1361_update_ptz_motor_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, motor_current);
    _tta_put_float(buf, 5, motor_position);
    _tta_put_float(buf, 9, reserve1);
    _tta_put_float(buf, 13, reserve2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA, buf, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC, nocrc);
#else
    ttalink_pod_1361_update_ptz_motor_data_t packet;
    packet.ack = ack;
    packet.motor_current = motor_current;
    packet.motor_position = motor_position;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA, (const char *)&packet, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1361_update_ptz_motor_data message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param motor_current .
 * @param motor_position .
 * @param reserve1 .
 * @param reserve2 .
 */
static inline void ttalink_pod_1361_update_ptz_motor_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2)
{
    _ttalink_pod_1361_update_ptz_motor_data_send(dst_addr, src_addr, chan, ack, motor_current, motor_position, reserve1, reserve2, false);
}

/**
 * @brief Send a pod_1361_update_ptz_motor_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param motor_current .
 * @param motor_position .
 * @param reserve1 .
 * @param reserve2 .
 */
static inline void ttalink_pod_1361_update_ptz_motor_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2)
{
    _ttalink_pod_1361_update_ptz_motor_data_send(dst_addr, src_addr, chan, ack, motor_current, motor_position, reserve1, reserve2, true);
}


static inline void _ttalink_pod_1361_update_ptz_motor_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1361_update_ptz_motor_data_send_nocrc(dst_addr, src_addr, chan, pod_1361_update_ptz_motor_data->ack, pod_1361_update_ptz_motor_data->motor_current, pod_1361_update_ptz_motor_data->motor_position, pod_1361_update_ptz_motor_data->reserve1, pod_1361_update_ptz_motor_data->reserve2);
    }else{
        ttalink_pod_1361_update_ptz_motor_data_send(dst_addr, src_addr, chan, pod_1361_update_ptz_motor_data->ack, pod_1361_update_ptz_motor_data->motor_current, pod_1361_update_ptz_motor_data->motor_position, pod_1361_update_ptz_motor_data->reserve1, pod_1361_update_ptz_motor_data->reserve2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA, (const char *)pod_1361_update_ptz_motor_data, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1361_update_ptz_motor_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1361_update_ptz_motor_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data)
{
    _ttalink_pod_1361_update_ptz_motor_data_send_struct(dst_addr, src_addr, chan, pod_1361_update_ptz_motor_data, false);
}

/**
 * @brief Send a pod_1361_update_ptz_motor_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1361_update_ptz_motor_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data)
{
    _ttalink_pod_1361_update_ptz_motor_data_send_struct(dst_addr, src_addr, chan, pod_1361_update_ptz_motor_data, true);
}

#if TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1361_update_ptz_motor_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, motor_current);
    _tta_put_float(buf, 5, motor_position);
    _tta_put_float(buf, 9, reserve1);
    _tta_put_float(buf, 13, reserve2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA, buf, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC, nocrc);
#else
    ttalink_pod_1361_update_ptz_motor_data_t *packet = (ttalink_pod_1361_update_ptz_motor_data_t *)msgbuf;
    packet->ack = ack;
    packet->motor_current = motor_current;
    packet->motor_position = motor_position;
    packet->reserve1 = reserve1;
    packet->reserve2 = reserve2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA, (const char *)packet, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1361_update_ptz_motor_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2)
{
    _ttalink_pod_1361_update_ptz_motor_data_send_buf(dst_addr, src_addr, msgbuf, chan, ack, motor_current, motor_position, reserve1, reserve2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1361_update_ptz_motor_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float motor_current, float motor_position, float reserve1, float reserve2)
{
    _ttalink_pod_1361_update_ptz_motor_data_send_buf(dst_addr, src_addr, msgbuf, chan, ack, motor_current, motor_position, reserve1, reserve2, true);
}
#endif

#endif

// MESSAGE POD_1361_UPDATE_PTZ_MOTOR_DATA UNPACKING


/**
 * @brief Get field ack from pod_1361_update_ptz_motor_data message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1361_update_ptz_motor_data_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field motor_current from pod_1361_update_ptz_motor_data message
 *
 * @return .
 */
static inline float ttalink_pod_1361_update_ptz_motor_data_get_motor_current(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  1);
}

/**
 * @brief Get field motor_position from pod_1361_update_ptz_motor_data message
 *
 * @return .
 */
static inline float ttalink_pod_1361_update_ptz_motor_data_get_motor_position(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Get field reserve1 from pod_1361_update_ptz_motor_data message
 *
 * @return .
 */
static inline float ttalink_pod_1361_update_ptz_motor_data_get_reserve1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field reserve2 from pod_1361_update_ptz_motor_data message
 *
 * @return .
 */
static inline float ttalink_pod_1361_update_ptz_motor_data_get_reserve2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Decode a pod_1361_update_ptz_motor_data message into a struct
 *
 * @param msg The message to decode
 * @param pod_1361_update_ptz_motor_data C-struct to decode the message contents into
 */
static inline void ttalink_pod_1361_update_ptz_motor_data_decode(const ttalink_message_t* msg, ttalink_pod_1361_update_ptz_motor_data_t* pod_1361_update_ptz_motor_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1361_update_ptz_motor_data->ack = ttalink_pod_1361_update_ptz_motor_data_get_ack(msg);
    pod_1361_update_ptz_motor_data->motor_current = ttalink_pod_1361_update_ptz_motor_data_get_motor_current(msg);
    pod_1361_update_ptz_motor_data->motor_position = ttalink_pod_1361_update_ptz_motor_data_get_motor_position(msg);
    pod_1361_update_ptz_motor_data->reserve1 = ttalink_pod_1361_update_ptz_motor_data_get_reserve1(msg);
    pod_1361_update_ptz_motor_data->reserve2 = ttalink_pod_1361_update_ptz_motor_data_get_reserve2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN? msg->len : TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN;
        memset(pod_1361_update_ptz_motor_data, 0, TTALINK_MSG_ID_POD_1361_UPDATE_PTZ_MOTOR_DATA_LEN);
    memcpy(pod_1361_update_ptz_motor_data, _TTA_PAYLOAD(msg), len);
#endif
}
