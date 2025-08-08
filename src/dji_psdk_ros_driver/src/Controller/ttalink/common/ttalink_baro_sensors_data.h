#pragma once
// MESSAGE BARO_SENSORS_DATA PACKING

#define TTALINK_MSG_ID_BARO_SENSORS_DATA 2170

TTAPACKED(
typedef struct __ttalink_baro_sensors_data_t {
 uint32_t update_time; /*< This is update time of baro.*/
 uint8_t sensor_index; /*<  .*/
 int32_t temperature; /*< This is baro sensor temperature.*/
 int32_t pressure; /*< This is air pressure.*/
 float baro_e; /*< This is mathematical expectation of air pressure.*/
 float baro_var; /*< This is variance of air pressure.*/
}) ttalink_baro_sensors_data_t;

#define TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN 21
#define TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN 21
#define TTALINK_MSG_ID_2170_LEN 21
#define TTALINK_MSG_ID_2170_MIN_LEN 21

#define TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC 124
#define TTALINK_MSG_ID_2170_CRC 124



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_BARO_SENSORS_DATA { \
    2170, \
    "BARO_SENSORS_DATA", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_baro_sensors_data_t, update_time) }, \
         { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_baro_sensors_data_t, sensor_index) }, \
         { "temperature", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_baro_sensors_data_t, temperature) }, \
         { "pressure", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_baro_sensors_data_t, pressure) }, \
         { "baro_e", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_baro_sensors_data_t, baro_e) }, \
         { "baro_var", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_baro_sensors_data_t, baro_var) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_BARO_SENSORS_DATA { \
    "BARO_SENSORS_DATA", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_baro_sensors_data_t, update_time) }, \
         { "sensor_index", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_baro_sensors_data_t, sensor_index) }, \
         { "temperature", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_baro_sensors_data_t, temperature) }, \
         { "pressure", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_baro_sensors_data_t, pressure) }, \
         { "baro_e", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_baro_sensors_data_t, baro_e) }, \
         { "baro_var", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_baro_sensors_data_t, baro_var) }, \
         } \
}
#endif


static inline uint16_t _ttalink_baro_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_int32_t(buf, 5, temperature);
    _tta_put_int32_t(buf, 9, pressure);
    _tta_put_float(buf, 13, baro_e);
    _tta_put_float(buf, 17, baro_var);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN);
#else
    ttalink_baro_sensors_data_t packet;
    packet.update_time = update_time;
    packet.sensor_index = sensor_index;
    packet.temperature = temperature;
    packet.pressure = pressure;
    packet.baro_e = baro_e;
    packet.baro_var = baro_var;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BARO_SENSORS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a baro_sensors_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time This is update time of baro.
 * @param sensor_index  .
 * @param temperature This is baro sensor temperature.
 * @param pressure This is air pressure.
 * @param baro_e This is mathematical expectation of air pressure.
 * @param baro_var This is variance of air pressure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_baro_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var)
{
    return _ttalink_baro_sensors_data_pack(dst_addr, src_addr, msg,  update_time, sensor_index, temperature, pressure, baro_e, baro_var, false);
}

/**
 * @brief Pack a baro_sensors_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time This is update time of baro.
 * @param sensor_index  .
 * @param temperature This is baro sensor temperature.
 * @param pressure This is air pressure.
 * @param baro_e This is mathematical expectation of air pressure.
 * @param baro_var This is variance of air pressure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_baro_sensors_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var)
{
    return _ttalink_baro_sensors_data_pack(dst_addr, src_addr, msg,  update_time, sensor_index, temperature, pressure, baro_e, baro_var, true);
}


static inline uint16_t _ttalink_baro_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t sensor_index,int32_t temperature,int32_t pressure,float baro_e,float baro_var, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_int32_t(buf, 5, temperature);
    _tta_put_int32_t(buf, 9, pressure);
    _tta_put_float(buf, 13, baro_e);
    _tta_put_float(buf, 17, baro_var);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN);
#else
    ttalink_baro_sensors_data_t packet;
    packet.update_time = update_time;
    packet.sensor_index = sensor_index;
    packet.temperature = temperature;
    packet.pressure = pressure;
    packet.baro_e = baro_e;
    packet.baro_var = baro_var;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BARO_SENSORS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a baro_sensors_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time This is update time of baro.
 * @param sensor_index  .
 * @param temperature This is baro sensor temperature.
 * @param pressure This is air pressure.
 * @param baro_e This is mathematical expectation of air pressure.
 * @param baro_var This is variance of air pressure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_baro_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t sensor_index,int32_t temperature,int32_t pressure,float baro_e,float baro_var)
{
    return _ttalink_baro_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sensor_index, temperature, pressure, baro_e, baro_var, false);
}

/**
 * @brief Pack a baro_sensors_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time This is update time of baro.
 * @param sensor_index  .
 * @param temperature This is baro sensor temperature.
 * @param pressure This is air pressure.
 * @param baro_e This is mathematical expectation of air pressure.
 * @param baro_var This is variance of air pressure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_baro_sensors_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t sensor_index,int32_t temperature,int32_t pressure,float baro_e,float baro_var)
{
    return _ttalink_baro_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sensor_index, temperature, pressure, baro_e, baro_var, true);
}


static inline uint16_t _ttalink_baro_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_baro_sensors_data_t* baro_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_baro_sensors_data_pack_nocrc(dst_addr, src_addr, msg, baro_sensors_data->update_time, baro_sensors_data->sensor_index, baro_sensors_data->temperature, baro_sensors_data->pressure, baro_sensors_data->baro_e, baro_sensors_data->baro_var);
    }else{
        return ttalink_baro_sensors_data_pack(dst_addr, src_addr, msg, baro_sensors_data->update_time, baro_sensors_data->sensor_index, baro_sensors_data->temperature, baro_sensors_data->pressure, baro_sensors_data->baro_e, baro_sensors_data->baro_var);
    }
    
}

/**
 * @brief Encode a baro_sensors_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param baro_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_baro_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_baro_sensors_data_t* baro_sensors_data)
{
    return _ttalink_baro_sensors_data_encode(dst_addr, src_addr, msg, baro_sensors_data, false);
}

/**
 * @brief Encode a baro_sensors_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param baro_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_baro_sensors_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_baro_sensors_data_t* baro_sensors_data)
{
    return _ttalink_baro_sensors_data_encode(dst_addr, src_addr, msg, baro_sensors_data, true);
}


static inline uint16_t _ttalink_baro_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_baro_sensors_data_t* baro_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_baro_sensors_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, baro_sensors_data->update_time, baro_sensors_data->sensor_index, baro_sensors_data->temperature, baro_sensors_data->pressure, baro_sensors_data->baro_e, baro_sensors_data->baro_var);
    }else{
        return ttalink_baro_sensors_data_pack_chan(dst_addr, src_addr, chan, msg, baro_sensors_data->update_time, baro_sensors_data->sensor_index, baro_sensors_data->temperature, baro_sensors_data->pressure, baro_sensors_data->baro_e, baro_sensors_data->baro_var);
    }
}

/**
 * @brief Encode a baro_sensors_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param baro_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_baro_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_baro_sensors_data_t* baro_sensors_data)
{
    return _ttalink_baro_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, baro_sensors_data, false);
}

/**
 * @brief Encode a baro_sensors_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param baro_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_baro_sensors_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_baro_sensors_data_t* baro_sensors_data)
{
    return _ttalink_baro_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, baro_sensors_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_baro_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_int32_t(buf, 5, temperature);
    _tta_put_int32_t(buf, 9, pressure);
    _tta_put_float(buf, 13, baro_e);
    _tta_put_float(buf, 17, baro_var);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BARO_SENSORS_DATA, buf, TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_baro_sensors_data_t packet;
    packet.update_time = update_time;
    packet.sensor_index = sensor_index;
    packet.temperature = temperature;
    packet.pressure = pressure;
    packet.baro_e = baro_e;
    packet.baro_var = baro_var;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BARO_SENSORS_DATA, (const char *)&packet, TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a baro_sensors_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time This is update time of baro.
 * @param sensor_index  .
 * @param temperature This is baro sensor temperature.
 * @param pressure This is air pressure.
 * @param baro_e This is mathematical expectation of air pressure.
 * @param baro_var This is variance of air pressure.
 */
static inline void ttalink_baro_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var)
{
    _ttalink_baro_sensors_data_send(dst_addr, src_addr, chan, update_time, sensor_index, temperature, pressure, baro_e, baro_var, false);
}

/**
 * @brief Send a baro_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time This is update time of baro.
 * @param sensor_index  .
 * @param temperature This is baro sensor temperature.
 * @param pressure This is air pressure.
 * @param baro_e This is mathematical expectation of air pressure.
 * @param baro_var This is variance of air pressure.
 */
static inline void ttalink_baro_sensors_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var)
{
    _ttalink_baro_sensors_data_send(dst_addr, src_addr, chan, update_time, sensor_index, temperature, pressure, baro_e, baro_var, true);
}


static inline void _ttalink_baro_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_baro_sensors_data_t* baro_sensors_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_baro_sensors_data_send_nocrc(dst_addr, src_addr, chan, baro_sensors_data->update_time, baro_sensors_data->sensor_index, baro_sensors_data->temperature, baro_sensors_data->pressure, baro_sensors_data->baro_e, baro_sensors_data->baro_var);
    }else{
        ttalink_baro_sensors_data_send(dst_addr, src_addr, chan, baro_sensors_data->update_time, baro_sensors_data->sensor_index, baro_sensors_data->temperature, baro_sensors_data->pressure, baro_sensors_data->baro_e, baro_sensors_data->baro_var);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BARO_SENSORS_DATA, (const char *)baro_sensors_data, TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a baro_sensors_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_baro_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_baro_sensors_data_t* baro_sensors_data)
{
    _ttalink_baro_sensors_data_send_struct(dst_addr, src_addr, chan, baro_sensors_data, false);
}

/**
 * @brief Send a baro_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_baro_sensors_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_baro_sensors_data_t* baro_sensors_data)
{
    _ttalink_baro_sensors_data_send_struct(dst_addr, src_addr, chan, baro_sensors_data, true);
}

#if TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_baro_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, sensor_index);
    _tta_put_int32_t(buf, 5, temperature);
    _tta_put_int32_t(buf, 9, pressure);
    _tta_put_float(buf, 13, baro_e);
    _tta_put_float(buf, 17, baro_var);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BARO_SENSORS_DATA, buf, TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_baro_sensors_data_t *packet = (ttalink_baro_sensors_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->sensor_index = sensor_index;
    packet->temperature = temperature;
    packet->pressure = pressure;
    packet->baro_e = baro_e;
    packet->baro_var = baro_var;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BARO_SENSORS_DATA, (const char *)packet, TTALINK_MSG_ID_BARO_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN, TTALINK_MSG_ID_BARO_SENSORS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_baro_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var)
{
    _ttalink_baro_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sensor_index, temperature, pressure, baro_e, baro_var, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_baro_sensors_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t sensor_index, int32_t temperature, int32_t pressure, float baro_e, float baro_var)
{
    _ttalink_baro_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sensor_index, temperature, pressure, baro_e, baro_var, true);
}
#endif

#endif

// MESSAGE BARO_SENSORS_DATA UNPACKING


/**
 * @brief Get field update_time from baro_sensors_data message
 *
 * @return This is update time of baro.
 */
static inline uint32_t ttalink_baro_sensors_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sensor_index from baro_sensors_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_baro_sensors_data_get_sensor_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field temperature from baro_sensors_data message
 *
 * @return This is baro sensor temperature.
 */
static inline int32_t ttalink_baro_sensors_data_get_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  5);
}

/**
 * @brief Get field pressure from baro_sensors_data message
 *
 * @return This is air pressure.
 */
static inline int32_t ttalink_baro_sensors_data_get_pressure(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  9);
}

/**
 * @brief Get field baro_e from baro_sensors_data message
 *
 * @return This is mathematical expectation of air pressure.
 */
static inline float ttalink_baro_sensors_data_get_baro_e(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field baro_var from baro_sensors_data message
 *
 * @return This is variance of air pressure.
 */
static inline float ttalink_baro_sensors_data_get_baro_var(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Decode a baro_sensors_data message into a struct
 *
 * @param msg The message to decode
 * @param baro_sensors_data C-struct to decode the message contents into
 */
static inline void ttalink_baro_sensors_data_decode(const ttalink_message_t* msg, ttalink_baro_sensors_data_t* baro_sensors_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    baro_sensors_data->update_time = ttalink_baro_sensors_data_get_update_time(msg);
    baro_sensors_data->sensor_index = ttalink_baro_sensors_data_get_sensor_index(msg);
    baro_sensors_data->temperature = ttalink_baro_sensors_data_get_temperature(msg);
    baro_sensors_data->pressure = ttalink_baro_sensors_data_get_pressure(msg);
    baro_sensors_data->baro_e = ttalink_baro_sensors_data_get_baro_e(msg);
    baro_sensors_data->baro_var = ttalink_baro_sensors_data_get_baro_var(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN? msg->len : TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN;
        memset(baro_sensors_data, 0, TTALINK_MSG_ID_BARO_SENSORS_DATA_LEN);
    memcpy(baro_sensors_data, _TTA_PAYLOAD(msg), len);
#endif
}
