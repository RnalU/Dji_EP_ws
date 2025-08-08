#pragma once
// MESSAGE SENSOR_MONITOR PACKING

#define TTALINK_MSG_ID_SENSOR_MONITOR 2182

TTAPACKED(
typedef struct __ttalink_sensor_monitor_t {
 uint8_t state; /*<  .*/
 uint16_t sensor_error_no; /*<  .*/
 uint32_t task_id; /*<  .*/
}) ttalink_sensor_monitor_t;

#define TTALINK_MSG_ID_SENSOR_MONITOR_LEN 7
#define TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN 7
#define TTALINK_MSG_ID_2182_LEN 7
#define TTALINK_MSG_ID_2182_MIN_LEN 7

#define TTALINK_MSG_ID_SENSOR_MONITOR_CRC 189
#define TTALINK_MSG_ID_2182_CRC 189



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SENSOR_MONITOR { \
    2182, \
    "SENSOR_MONITOR", \
    3, \
    {  { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_sensor_monitor_t, state) }, \
         { "sensor_error_no", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_sensor_monitor_t, sensor_error_no) }, \
         { "task_id", NULL, TTALINK_TYPE_UINT32_T, 0, 3, offsetof(ttalink_sensor_monitor_t, task_id) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SENSOR_MONITOR { \
    "SENSOR_MONITOR", \
    3, \
    {  { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_sensor_monitor_t, state) }, \
         { "sensor_error_no", NULL, TTALINK_TYPE_UINT16_T, 0, 1, offsetof(ttalink_sensor_monitor_t, sensor_error_no) }, \
         { "task_id", NULL, TTALINK_TYPE_UINT32_T, 0, 3, offsetof(ttalink_sensor_monitor_t, task_id) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sensor_monitor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, uint16_t sensor_error_no, uint32_t task_id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_MONITOR_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint16_t(buf, 1, sensor_error_no);
    _tta_put_uint32_t(buf, 3, task_id);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_MONITOR_LEN);
#else
    ttalink_sensor_monitor_t packet;
    packet.state = state;
    packet.sensor_error_no = sensor_error_no;
    packet.task_id = task_id;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_MONITOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_MONITOR;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_CRC, nocrc);
}

/**
 * @brief Pack a sensor_monitor message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param state  .
 * @param sensor_error_no  .
 * @param task_id  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, uint16_t sensor_error_no, uint32_t task_id)
{
    return _ttalink_sensor_monitor_pack(dst_addr, src_addr, msg,  state, sensor_error_no, task_id, false);
}

/**
 * @brief Pack a sensor_monitor message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param state  .
 * @param sensor_error_no  .
 * @param task_id  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t state, uint16_t sensor_error_no, uint32_t task_id)
{
    return _ttalink_sensor_monitor_pack(dst_addr, src_addr, msg,  state, sensor_error_no, task_id, true);
}


static inline uint16_t _ttalink_sensor_monitor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,uint16_t sensor_error_no,uint32_t task_id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_MONITOR_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint16_t(buf, 1, sensor_error_no);
    _tta_put_uint32_t(buf, 3, task_id);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_MONITOR_LEN);
#else
    ttalink_sensor_monitor_t packet;
    packet.state = state;
    packet.sensor_error_no = sensor_error_no;
    packet.task_id = task_id;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_MONITOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_MONITOR;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_CRC, nocrc);
}

/**
 * @brief Pack a sensor_monitor message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param state  .
 * @param sensor_error_no  .
 * @param task_id  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,uint16_t sensor_error_no,uint32_t task_id)
{
    return _ttalink_sensor_monitor_pack_chan(dst_addr, src_addr, chan, msg,  state, sensor_error_no, task_id, false);
}

/**
 * @brief Pack a sensor_monitor message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param state  .
 * @param sensor_error_no  .
 * @param task_id  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_monitor_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t state,uint16_t sensor_error_no,uint32_t task_id)
{
    return _ttalink_sensor_monitor_pack_chan(dst_addr, src_addr, chan, msg,  state, sensor_error_no, task_id, true);
}


static inline uint16_t _ttalink_sensor_monitor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_monitor_t* sensor_monitor, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_monitor_pack_nocrc(dst_addr, src_addr, msg, sensor_monitor->state, sensor_monitor->sensor_error_no, sensor_monitor->task_id);
    }else{
        return ttalink_sensor_monitor_pack(dst_addr, src_addr, msg, sensor_monitor->state, sensor_monitor->sensor_error_no, sensor_monitor->task_id);
    }
    
}

/**
 * @brief Encode a sensor_monitor struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_monitor_t* sensor_monitor)
{
    return _ttalink_sensor_monitor_encode(dst_addr, src_addr, msg, sensor_monitor, false);
}

/**
 * @brief Encode a sensor_monitor struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_monitor_t* sensor_monitor)
{
    return _ttalink_sensor_monitor_encode(dst_addr, src_addr, msg, sensor_monitor, true);
}


static inline uint16_t _ttalink_sensor_monitor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_monitor_t* sensor_monitor, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_monitor_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sensor_monitor->state, sensor_monitor->sensor_error_no, sensor_monitor->task_id);
    }else{
        return ttalink_sensor_monitor_pack_chan(dst_addr, src_addr, chan, msg, sensor_monitor->state, sensor_monitor->sensor_error_no, sensor_monitor->task_id);
    }
}

/**
 * @brief Encode a sensor_monitor struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_monitor_t* sensor_monitor)
{
    return _ttalink_sensor_monitor_encode_chan(dst_addr, src_addr, chan, msg, sensor_monitor, false);
}

/**
 * @brief Encode a sensor_monitor struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_monitor C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_monitor_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_monitor_t* sensor_monitor)
{
    return _ttalink_sensor_monitor_encode_chan(dst_addr, src_addr, chan, msg, sensor_monitor, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sensor_monitor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, uint16_t sensor_error_no, uint32_t task_id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_MONITOR_LEN];
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint16_t(buf, 1, sensor_error_no);
    _tta_put_uint32_t(buf, 3, task_id);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR, buf, TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_CRC, nocrc);
#else
    ttalink_sensor_monitor_t packet;
    packet.state = state;
    packet.sensor_error_no = sensor_error_no;
    packet.task_id = task_id;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR, (const char *)&packet, TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_monitor message
 * @param chan TTAlink channel to send the message
 *
 * @param state  .
 * @param sensor_error_no  .
 * @param task_id  .
 */
static inline void ttalink_sensor_monitor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, uint16_t sensor_error_no, uint32_t task_id)
{
    _ttalink_sensor_monitor_send(dst_addr, src_addr, chan, state, sensor_error_no, task_id, false);
}

/**
 * @brief Send a sensor_monitor message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param state  .
 * @param sensor_error_no  .
 * @param task_id  .
 */
static inline void ttalink_sensor_monitor_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t state, uint16_t sensor_error_no, uint32_t task_id)
{
    _ttalink_sensor_monitor_send(dst_addr, src_addr, chan, state, sensor_error_no, task_id, true);
}


static inline void _ttalink_sensor_monitor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_monitor_t* sensor_monitor, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sensor_monitor_send_nocrc(dst_addr, src_addr, chan, sensor_monitor->state, sensor_monitor->sensor_error_no, sensor_monitor->task_id);
    }else{
        ttalink_sensor_monitor_send(dst_addr, src_addr, chan, sensor_monitor->state, sensor_monitor->sensor_error_no, sensor_monitor->task_id);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR, (const char *)sensor_monitor, TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_monitor message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_monitor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_monitor_t* sensor_monitor)
{
    _ttalink_sensor_monitor_send_struct(dst_addr, src_addr, chan, sensor_monitor, false);
}

/**
 * @brief Send a sensor_monitor message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_monitor_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_monitor_t* sensor_monitor)
{
    _ttalink_sensor_monitor_send_struct(dst_addr, src_addr, chan, sensor_monitor, true);
}

#if TTALINK_MSG_ID_SENSOR_MONITOR_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sensor_monitor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, uint16_t sensor_error_no, uint32_t task_id, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, state);
    _tta_put_uint16_t(buf, 1, sensor_error_no);
    _tta_put_uint32_t(buf, 3, task_id);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR, buf, TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_CRC, nocrc);
#else
    ttalink_sensor_monitor_t *packet = (ttalink_sensor_monitor_t *)msgbuf;
    packet->state = state;
    packet->sensor_error_no = sensor_error_no;
    packet->task_id = task_id;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_MONITOR, (const char *)packet, TTALINK_MSG_ID_SENSOR_MONITOR_MIN_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_LEN, TTALINK_MSG_ID_SENSOR_MONITOR_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sensor_monitor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, uint16_t sensor_error_no, uint32_t task_id)
{
    _ttalink_sensor_monitor_send_buf(dst_addr, src_addr, msgbuf, chan, state, sensor_error_no, task_id, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sensor_monitor_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t state, uint16_t sensor_error_no, uint32_t task_id)
{
    _ttalink_sensor_monitor_send_buf(dst_addr, src_addr, msgbuf, chan, state, sensor_error_no, task_id, true);
}
#endif

#endif

// MESSAGE SENSOR_MONITOR UNPACKING


/**
 * @brief Get field state from sensor_monitor message
 *
 * @return  .
 */
static inline uint8_t ttalink_sensor_monitor_get_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field sensor_error_no from sensor_monitor message
 *
 * @return  .
 */
static inline uint16_t ttalink_sensor_monitor_get_sensor_error_no(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  1);
}

/**
 * @brief Get field task_id from sensor_monitor message
 *
 * @return  .
 */
static inline uint32_t ttalink_sensor_monitor_get_task_id(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  3);
}

/**
 * @brief Decode a sensor_monitor message into a struct
 *
 * @param msg The message to decode
 * @param sensor_monitor C-struct to decode the message contents into
 */
static inline void ttalink_sensor_monitor_decode(const ttalink_message_t* msg, ttalink_sensor_monitor_t* sensor_monitor)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sensor_monitor->state = ttalink_sensor_monitor_get_state(msg);
    sensor_monitor->sensor_error_no = ttalink_sensor_monitor_get_sensor_error_no(msg);
    sensor_monitor->task_id = ttalink_sensor_monitor_get_task_id(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SENSOR_MONITOR_LEN? msg->len : TTALINK_MSG_ID_SENSOR_MONITOR_LEN;
        memset(sensor_monitor, 0, TTALINK_MSG_ID_SENSOR_MONITOR_LEN);
    memcpy(sensor_monitor, _TTA_PAYLOAD(msg), len);
#endif
}
