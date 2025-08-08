#pragma once
// MESSAGE SENSOR_STATES PACKING

#define TTALINK_MSG_ID_SENSOR_STATES 2180

TTAPACKED(
typedef struct __ttalink_sensor_states_t {
 uint8_t sensor; /*<  .*/
 uint8_t sensor_state; /*<  .*/
 uint32_t update_time; /*<  .*/
}) ttalink_sensor_states_t;

#define TTALINK_MSG_ID_SENSOR_STATES_LEN 6
#define TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN 6
#define TTALINK_MSG_ID_2180_LEN 6
#define TTALINK_MSG_ID_2180_MIN_LEN 6

#define TTALINK_MSG_ID_SENSOR_STATES_CRC 56
#define TTALINK_MSG_ID_2180_CRC 56



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SENSOR_STATES { \
    2180, \
    "SENSOR_STATES", \
    3, \
    {  { "sensor", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_sensor_states_t, sensor) }, \
         { "sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_sensor_states_t, sensor_state) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_sensor_states_t, update_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SENSOR_STATES { \
    "SENSOR_STATES", \
    3, \
    {  { "sensor", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_sensor_states_t, sensor) }, \
         { "sensor_state", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_sensor_states_t, sensor_state) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 2, offsetof(ttalink_sensor_states_t, update_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sensor_states_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor, uint8_t sensor_state, uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_STATES_LEN];
    _tta_put_uint8_t(buf, 0, sensor);
    _tta_put_uint8_t(buf, 1, sensor_state);
    _tta_put_uint32_t(buf, 2, update_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_STATES_LEN);
#else
    ttalink_sensor_states_t packet;
    packet.sensor = sensor;
    packet.sensor_state = sensor_state;
    packet.update_time = update_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_STATES_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_STATES;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN, TTALINK_MSG_ID_SENSOR_STATES_LEN, TTALINK_MSG_ID_SENSOR_STATES_CRC, nocrc);
}

/**
 * @brief Pack a sensor_states message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor  .
 * @param sensor_state  .
 * @param update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_states_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor, uint8_t sensor_state, uint32_t update_time)
{
    return _ttalink_sensor_states_pack(dst_addr, src_addr, msg,  sensor, sensor_state, update_time, false);
}

/**
 * @brief Pack a sensor_states message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sensor  .
 * @param sensor_state  .
 * @param update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_states_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t sensor, uint8_t sensor_state, uint32_t update_time)
{
    return _ttalink_sensor_states_pack(dst_addr, src_addr, msg,  sensor, sensor_state, update_time, true);
}


static inline uint16_t _ttalink_sensor_states_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor,uint8_t sensor_state,uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_STATES_LEN];
    _tta_put_uint8_t(buf, 0, sensor);
    _tta_put_uint8_t(buf, 1, sensor_state);
    _tta_put_uint32_t(buf, 2, update_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_STATES_LEN);
#else
    ttalink_sensor_states_t packet;
    packet.sensor = sensor;
    packet.sensor_state = sensor_state;
    packet.update_time = update_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_STATES_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_STATES;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN, TTALINK_MSG_ID_SENSOR_STATES_LEN, TTALINK_MSG_ID_SENSOR_STATES_CRC, nocrc);
}

/**
 * @brief Pack a sensor_states message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor  .
 * @param sensor_state  .
 * @param update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_states_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor,uint8_t sensor_state,uint32_t update_time)
{
    return _ttalink_sensor_states_pack_chan(dst_addr, src_addr, chan, msg,  sensor, sensor_state, update_time, false);
}

/**
 * @brief Pack a sensor_states message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor  .
 * @param sensor_state  .
 * @param update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_states_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t sensor,uint8_t sensor_state,uint32_t update_time)
{
    return _ttalink_sensor_states_pack_chan(dst_addr, src_addr, chan, msg,  sensor, sensor_state, update_time, true);
}


static inline uint16_t _ttalink_sensor_states_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_states_t* sensor_states, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_states_pack_nocrc(dst_addr, src_addr, msg, sensor_states->sensor, sensor_states->sensor_state, sensor_states->update_time);
    }else{
        return ttalink_sensor_states_pack(dst_addr, src_addr, msg, sensor_states->sensor, sensor_states->sensor_state, sensor_states->update_time);
    }
    
}

/**
 * @brief Encode a sensor_states struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_states_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_states_t* sensor_states)
{
    return _ttalink_sensor_states_encode(dst_addr, src_addr, msg, sensor_states, false);
}

/**
 * @brief Encode a sensor_states struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_states_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_states_t* sensor_states)
{
    return _ttalink_sensor_states_encode(dst_addr, src_addr, msg, sensor_states, true);
}


static inline uint16_t _ttalink_sensor_states_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_states_t* sensor_states, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_states_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sensor_states->sensor, sensor_states->sensor_state, sensor_states->update_time);
    }else{
        return ttalink_sensor_states_pack_chan(dst_addr, src_addr, chan, msg, sensor_states->sensor, sensor_states->sensor_state, sensor_states->update_time);
    }
}

/**
 * @brief Encode a sensor_states struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_states_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_states_t* sensor_states)
{
    return _ttalink_sensor_states_encode_chan(dst_addr, src_addr, chan, msg, sensor_states, false);
}

/**
 * @brief Encode a sensor_states struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_states C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_states_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_states_t* sensor_states)
{
    return _ttalink_sensor_states_encode_chan(dst_addr, src_addr, chan, msg, sensor_states, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sensor_states_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor, uint8_t sensor_state, uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_STATES_LEN];
    _tta_put_uint8_t(buf, 0, sensor);
    _tta_put_uint8_t(buf, 1, sensor_state);
    _tta_put_uint32_t(buf, 2, update_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_STATES, buf, TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN, TTALINK_MSG_ID_SENSOR_STATES_LEN, TTALINK_MSG_ID_SENSOR_STATES_CRC, nocrc);
#else
    ttalink_sensor_states_t packet;
    packet.sensor = sensor;
    packet.sensor_state = sensor_state;
    packet.update_time = update_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_STATES, (const char *)&packet, TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN, TTALINK_MSG_ID_SENSOR_STATES_LEN, TTALINK_MSG_ID_SENSOR_STATES_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_states message
 * @param chan TTAlink channel to send the message
 *
 * @param sensor  .
 * @param sensor_state  .
 * @param update_time  .
 */
static inline void ttalink_sensor_states_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor, uint8_t sensor_state, uint32_t update_time)
{
    _ttalink_sensor_states_send(dst_addr, src_addr, chan, sensor, sensor_state, update_time, false);
}

/**
 * @brief Send a sensor_states message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sensor  .
 * @param sensor_state  .
 * @param update_time  .
 */
static inline void ttalink_sensor_states_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t sensor, uint8_t sensor_state, uint32_t update_time)
{
    _ttalink_sensor_states_send(dst_addr, src_addr, chan, sensor, sensor_state, update_time, true);
}


static inline void _ttalink_sensor_states_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_states_t* sensor_states, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sensor_states_send_nocrc(dst_addr, src_addr, chan, sensor_states->sensor, sensor_states->sensor_state, sensor_states->update_time);
    }else{
        ttalink_sensor_states_send(dst_addr, src_addr, chan, sensor_states->sensor, sensor_states->sensor_state, sensor_states->update_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_STATES, (const char *)sensor_states, TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN, TTALINK_MSG_ID_SENSOR_STATES_LEN, TTALINK_MSG_ID_SENSOR_STATES_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_states message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_states_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_states_t* sensor_states)
{
    _ttalink_sensor_states_send_struct(dst_addr, src_addr, chan, sensor_states, false);
}

/**
 * @brief Send a sensor_states message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_states_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_states_t* sensor_states)
{
    _ttalink_sensor_states_send_struct(dst_addr, src_addr, chan, sensor_states, true);
}

#if TTALINK_MSG_ID_SENSOR_STATES_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sensor_states_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor, uint8_t sensor_state, uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, sensor);
    _tta_put_uint8_t(buf, 1, sensor_state);
    _tta_put_uint32_t(buf, 2, update_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_STATES, buf, TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN, TTALINK_MSG_ID_SENSOR_STATES_LEN, TTALINK_MSG_ID_SENSOR_STATES_CRC, nocrc);
#else
    ttalink_sensor_states_t *packet = (ttalink_sensor_states_t *)msgbuf;
    packet->sensor = sensor;
    packet->sensor_state = sensor_state;
    packet->update_time = update_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_STATES, (const char *)packet, TTALINK_MSG_ID_SENSOR_STATES_MIN_LEN, TTALINK_MSG_ID_SENSOR_STATES_LEN, TTALINK_MSG_ID_SENSOR_STATES_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sensor_states_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor, uint8_t sensor_state, uint32_t update_time)
{
    _ttalink_sensor_states_send_buf(dst_addr, src_addr, msgbuf, chan, sensor, sensor_state, update_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sensor_states_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t sensor, uint8_t sensor_state, uint32_t update_time)
{
    _ttalink_sensor_states_send_buf(dst_addr, src_addr, msgbuf, chan, sensor, sensor_state, update_time, true);
}
#endif

#endif

// MESSAGE SENSOR_STATES UNPACKING


/**
 * @brief Get field sensor from sensor_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_sensor_states_get_sensor(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field sensor_state from sensor_states message
 *
 * @return  .
 */
static inline uint8_t ttalink_sensor_states_get_sensor_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field update_time from sensor_states message
 *
 * @return  .
 */
static inline uint32_t ttalink_sensor_states_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  2);
}

/**
 * @brief Decode a sensor_states message into a struct
 *
 * @param msg The message to decode
 * @param sensor_states C-struct to decode the message contents into
 */
static inline void ttalink_sensor_states_decode(const ttalink_message_t* msg, ttalink_sensor_states_t* sensor_states)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sensor_states->sensor = ttalink_sensor_states_get_sensor(msg);
    sensor_states->sensor_state = ttalink_sensor_states_get_sensor_state(msg);
    sensor_states->update_time = ttalink_sensor_states_get_update_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SENSOR_STATES_LEN? msg->len : TTALINK_MSG_ID_SENSOR_STATES_LEN;
        memset(sensor_states, 0, TTALINK_MSG_ID_SENSOR_STATES_LEN);
    memcpy(sensor_states, _TTA_PAYLOAD(msg), len);
#endif
}
