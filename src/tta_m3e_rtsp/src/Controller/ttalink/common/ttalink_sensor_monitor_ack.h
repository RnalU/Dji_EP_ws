#pragma once
// MESSAGE SENSOR_MONITOR_ACK PACKING

#define TTALINK_MSG_ID_SENSOR_MONITOR_ACK 2183

TTAPACKED(
typedef struct __ttalink_sensor_monitor_ack_t {
 uint16_t sensor_error_no; /*<  .*/
 uint32_t task_id; /*<  .*/
 uint8_t ack; /*<  .*/
}) ttalink_sensor_monitor_ack_t;

#define TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN 7
#define TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN 7
#define TTALINK_MSG_ID_2183_LEN 7
#define TTALINK_MSG_ID_2183_MIN_LEN 7

#define TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC 47
#define TTALINK_MSG_ID_2183_CRC 47



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SENSOR_MONITOR_ACK { \
    2183, \
    "SENSOR_MONITOR_ACK", \
    3, \
    {  { "sensor_error_no", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_sensor_monitor_ack_t, sensor_error_no) }, \
         { "task_id", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_sensor_monitor_ack_t, task_id) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_sensor_monitor_ack_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SENSOR_MONITOR_ACK { \
    "SENSOR_MONITOR_ACK", \
    3, \
    {  { "sensor_error_no", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_sensor_monitor_ack_t, sensor_error_no) }, \
         { "task_id", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_sensor_monitor_ack_t, task_id) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_sensor_monitor_ack_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sensor_monitor_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t sensor_error_no, uint32_t task_id, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN];
    _tta_put_uint16_t(buf, 0, sensor_error_no);
    _tta_put_uint32_t(buf, 2, task_id);
    _tta_put_uint8_t(buf, 6, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN);
#else
    ttalink_sensor_monitor_ack_t packet;
    packet.sensor_error_no = sensor_error_no;
    packet.task_id = task_id;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_MONITOR_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC, nocrc);
}

/**
 * @brief Pack a sensor_monitor_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor_error_no  .
 * @param task_id  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t sensor_error_no, uint32_t task_id, uint8_t ack)
{
    return _ttalink_sensor_monitor_ack_pack(dst_addr, src_addr, msg,  sensor_error_no, task_id, ack, false);
}

/**
 * @brief Pack a sensor_monitor_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor_error_no  .
 * @param task_id  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t sensor_error_no, uint32_t task_id, uint8_t ack)
{
    return _ttalink_sensor_monitor_ack_pack(dst_addr, src_addr, msg,  sensor_error_no, task_id, ack, true);
}


static inline uint16_t _ttalink_sensor_monitor_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t sensor_error_no,uint32_t task_id,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN];
    _tta_put_uint16_t(buf, 0, sensor_error_no);
    _tta_put_uint32_t(buf, 2, task_id);
    _tta_put_uint8_t(buf, 6, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN);
#else
    ttalink_sensor_monitor_ack_t packet;
    packet.sensor_error_no = sensor_error_no;
    packet.task_id = task_id;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_MONITOR_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC, nocrc);
}

/**
 * @brief Pack a sensor_monitor_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_error_no  .
 * @param task_id  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t sensor_error_no,uint32_t task_id,uint8_t ack)
{
    return _ttalink_sensor_monitor_ack_pack_chan(dst_addr, src_addr, chan, msg,  sensor_error_no, task_id, ack, false);
}

/**
 * @brief Pack a sensor_monitor_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_error_no  .
 * @param task_id  .
 * @param ack  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t sensor_error_no,uint32_t task_id,uint8_t ack)
{
    return _ttalink_sensor_monitor_ack_pack_chan(dst_addr, src_addr, chan, msg,  sensor_error_no, task_id, ack, true);
}


static inline uint16_t _ttalink_sensor_monitor_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_monitor_ack_pack_nocrc(dst_addr, src_addr, msg, sensor_monitor_ack->sensor_error_no, sensor_monitor_ack->task_id, sensor_monitor_ack->ack);
    }else{
        return ttalink_sensor_monitor_ack_pack(dst_addr, src_addr, msg, sensor_monitor_ack->sensor_error_no, sensor_monitor_ack->task_id, sensor_monitor_ack->ack);
    }
    
}

/**
 * @brief Encode a sensor_monitor_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack)
{
    return _ttalink_sensor_monitor_ack_encode(dst_addr, src_addr, msg, sensor_monitor_ack, false);
}

/**
 * @brief Encode a sensor_monitor_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack)
{
    return _ttalink_sensor_monitor_ack_encode(dst_addr, src_addr, msg, sensor_monitor_ack, true);
}


static inline uint16_t _ttalink_sensor_monitor_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_monitor_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sensor_monitor_ack->sensor_error_no, sensor_monitor_ack->task_id, sensor_monitor_ack->ack);
    }else{
        return ttalink_sensor_monitor_ack_pack_chan(dst_addr, src_addr, chan, msg, sensor_monitor_ack->sensor_error_no, sensor_monitor_ack->task_id, sensor_monitor_ack->ack);
    }
}

/**
 * @brief Encode a sensor_monitor_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack)
{
    return _ttalink_sensor_monitor_ack_encode_chan(dst_addr, src_addr, chan, msg, sensor_monitor_ack, false);
}

/**
 * @brief Encode a sensor_monitor_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack)
{
    return _ttalink_sensor_monitor_ack_encode_chan(dst_addr, src_addr, chan, msg, sensor_monitor_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sensor_monitor_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t sensor_error_no, uint32_t task_id, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN];
    _tta_put_uint16_t(buf, 0, sensor_error_no);
    _tta_put_uint32_t(buf, 2, task_id);
    _tta_put_uint8_t(buf, 6, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR_ACK, buf, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC, nocrc);
#else
    ttalink_sensor_monitor_ack_t packet;
    packet.sensor_error_no = sensor_error_no;
    packet.task_id = task_id;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR_ACK, (const char *)&packet, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_monitor_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param sensor_error_no  .
 * @param task_id  .
 * @param ack  .
 */
static inline void ttalink_sensor_monitor_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t sensor_error_no, uint32_t task_id, uint8_t ack)
{
    _ttalink_sensor_monitor_ack_send(dst_addr, src_addr, chan, sensor_error_no, task_id, ack, false);
}

/**
 * @brief Send a sensor_monitor_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sensor_error_no  .
 * @param task_id  .
 * @param ack  .
 */
static inline void ttalink_sensor_monitor_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t sensor_error_no, uint32_t task_id, uint8_t ack)
{
    _ttalink_sensor_monitor_ack_send(dst_addr, src_addr, chan, sensor_error_no, task_id, ack, true);
}


static inline void _ttalink_sensor_monitor_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sensor_monitor_ack_send_nocrc(dst_addr, src_addr, chan, sensor_monitor_ack->sensor_error_no, sensor_monitor_ack->task_id, sensor_monitor_ack->ack);
    }else{
        ttalink_sensor_monitor_ack_send(dst_addr, src_addr, chan, sensor_monitor_ack->sensor_error_no, sensor_monitor_ack->task_id, sensor_monitor_ack->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR_ACK, (const char *)sensor_monitor_ack, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_monitor_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_monitor_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack)
{
    _ttalink_sensor_monitor_ack_send_struct(dst_addr, src_addr, chan, sensor_monitor_ack, false);
}

/**
 * @brief Send a sensor_monitor_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_monitor_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_monitor_ack_t* sensor_monitor_ack)
{
    _ttalink_sensor_monitor_ack_send_struct(dst_addr, src_addr, chan, sensor_monitor_ack, true);
}

#if TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sensor_monitor_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t sensor_error_no, uint32_t task_id, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, sensor_error_no);
    _tta_put_uint32_t(buf, 2, task_id);
    _tta_put_uint8_t(buf, 6, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR_ACK, buf, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC, nocrc);
#else
    ttalink_sensor_monitor_ack_t *packet = (ttalink_sensor_monitor_ack_t *)msgbuf;
    packet->sensor_error_no = sensor_error_no;
    packet->task_id = task_id;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR_ACK, (const char *)packet, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sensor_monitor_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t sensor_error_no, uint32_t task_id, uint8_t ack)
{
    _ttalink_sensor_monitor_ack_send_buf(dst_addr, src_addr, msgbuf, chan, sensor_error_no, task_id, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sensor_monitor_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t sensor_error_no, uint32_t task_id, uint8_t ack)
{
    _ttalink_sensor_monitor_ack_send_buf(dst_addr, src_addr, msgbuf, chan, sensor_error_no, task_id, ack, true);
}
#endif

#endif

// MESSAGE SENSOR_MONITOR_ACK UNPACKING


/**
 * @brief Get field sensor_error_no from sensor_monitor_ack message
 *
 * @return  .
 */
static inline uint16_t ttalink_sensor_monitor_ack_get_sensor_error_no(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field task_id from sensor_monitor_ack message
 *
 * @return  .
 */
static inline uint32_t ttalink_sensor_monitor_ack_get_task_id(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  2);
}

/**
 * @brief Get field ack from sensor_monitor_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_sensor_monitor_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a sensor_monitor_ack message into a struct
 *
 * @param msg The message to decode
 * @param sensor_monitor_ack C-struct to decode the message contents into
 */
static inline void ttalink_sensor_monitor_ack_decode(const ttalink_message_t* msg, ttalink_sensor_monitor_ack_t* sensor_monitor_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sensor_monitor_ack->sensor_error_no = ttalink_sensor_monitor_ack_get_sensor_error_no(msg);
    sensor_monitor_ack->task_id = ttalink_sensor_monitor_ack_get_task_id(msg);
    sensor_monitor_ack->ack = ttalink_sensor_monitor_ack_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN? msg->len : TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN;
        memset(sensor_monitor_ack, 0, TTALINK_MSG_ID_SENSOR_MONITOR_ACK_LEN);
    memcpy(sensor_monitor_ack, _TTA_PAYLOAD(msg), len);
#endif
}
