#pragma once
// MESSAGE SENSORS_DEVICE_DATA PACKING

#define TTALINK_MSG_ID_SENSORS_DEVICE_DATA 2175

TTAPACKED(
typedef struct __ttalink_sensors_device_data_t {
 uint32_t update_time; /*<  .*/
 int32_t radar_latit; /*<  .*/
 int32_t radar_longi; /*<  .*/
 float radar_altit; /*<  .*/
 uint8_t radar_confidance; /*<  雷达可信度.*/
 float params[4]; /*<  .*/
}) ttalink_sensors_device_data_t;

#define TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN 33
#define TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN 33
#define TTALINK_MSG_ID_2175_LEN 33
#define TTALINK_MSG_ID_2175_MIN_LEN 33

#define TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC 53
#define TTALINK_MSG_ID_2175_CRC 53

#define TTALINK_MSG_SENSORS_DEVICE_DATA_FIELD_PARAMS_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SENSORS_DEVICE_DATA { \
    2175, \
    "SENSORS_DEVICE_DATA", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sensors_device_data_t, update_time) }, \
         { "radar_latit", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_sensors_device_data_t, radar_latit) }, \
         { "radar_longi", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_sensors_device_data_t, radar_longi) }, \
         { "radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_sensors_device_data_t, radar_altit) }, \
         { "radar_confidance", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_sensors_device_data_t, radar_confidance) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 17, offsetof(ttalink_sensors_device_data_t, params) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SENSORS_DEVICE_DATA { \
    "SENSORS_DEVICE_DATA", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sensors_device_data_t, update_time) }, \
         { "radar_latit", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_sensors_device_data_t, radar_latit) }, \
         { "radar_longi", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_sensors_device_data_t, radar_longi) }, \
         { "radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_sensors_device_data_t, radar_altit) }, \
         { "radar_confidance", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_sensors_device_data_t, radar_confidance) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 17, offsetof(ttalink_sensors_device_data_t, params) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sensors_device_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_uint8_t(buf, 16, radar_confidance);
    _tta_put_float_array(buf, 17, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN);
#else
    ttalink_sensors_device_data_t packet;
    packet.update_time = update_time;
    packet.radar_latit = radar_latit;
    packet.radar_longi = radar_longi;
    packet.radar_altit = radar_altit;
    packet.radar_confidance = radar_confidance;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSORS_DEVICE_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a sensors_device_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param radar_confidance  雷达可信度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensors_device_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params)
{
    return _ttalink_sensors_device_data_pack(dst_addr, src_addr, msg,  update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, false);
}

/**
 * @brief Pack a sensors_device_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param radar_confidance  雷达可信度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensors_device_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params)
{
    return _ttalink_sensors_device_data_pack(dst_addr, src_addr, msg,  update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, true);
}


static inline uint16_t _ttalink_sensors_device_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t radar_latit,int32_t radar_longi,float radar_altit,uint8_t radar_confidance,const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_uint8_t(buf, 16, radar_confidance);
    _tta_put_float_array(buf, 17, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN);
#else
    ttalink_sensors_device_data_t packet;
    packet.update_time = update_time;
    packet.radar_latit = radar_latit;
    packet.radar_longi = radar_longi;
    packet.radar_altit = radar_altit;
    packet.radar_confidance = radar_confidance;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSORS_DEVICE_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a sensors_device_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param radar_confidance  雷达可信度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensors_device_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t radar_latit,int32_t radar_longi,float radar_altit,uint8_t radar_confidance,const float *params)
{
    return _ttalink_sensors_device_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, false);
}

/**
 * @brief Pack a sensors_device_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param radar_confidance  雷达可信度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensors_device_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t radar_latit,int32_t radar_longi,float radar_altit,uint8_t radar_confidance,const float *params)
{
    return _ttalink_sensors_device_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, true);
}


static inline uint16_t _ttalink_sensors_device_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensors_device_data_t* sensors_device_data, bool nocrc)
{
    if(nocrc){
        return ttalink_sensors_device_data_pack_nocrc(dst_addr, src_addr, msg, sensors_device_data->update_time, sensors_device_data->radar_latit, sensors_device_data->radar_longi, sensors_device_data->radar_altit, sensors_device_data->radar_confidance, sensors_device_data->params);
    }else{
        return ttalink_sensors_device_data_pack(dst_addr, src_addr, msg, sensors_device_data->update_time, sensors_device_data->radar_latit, sensors_device_data->radar_longi, sensors_device_data->radar_altit, sensors_device_data->radar_confidance, sensors_device_data->params);
    }
    
}

/**
 * @brief Encode a sensors_device_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensors_device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensors_device_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensors_device_data_t* sensors_device_data)
{
    return _ttalink_sensors_device_data_encode(dst_addr, src_addr, msg, sensors_device_data, false);
}

/**
 * @brief Encode a sensors_device_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensors_device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensors_device_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensors_device_data_t* sensors_device_data)
{
    return _ttalink_sensors_device_data_encode(dst_addr, src_addr, msg, sensors_device_data, true);
}


static inline uint16_t _ttalink_sensors_device_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensors_device_data_t* sensors_device_data, bool nocrc)
{
    if(nocrc){
        return ttalink_sensors_device_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sensors_device_data->update_time, sensors_device_data->radar_latit, sensors_device_data->radar_longi, sensors_device_data->radar_altit, sensors_device_data->radar_confidance, sensors_device_data->params);
    }else{
        return ttalink_sensors_device_data_pack_chan(dst_addr, src_addr, chan, msg, sensors_device_data->update_time, sensors_device_data->radar_latit, sensors_device_data->radar_longi, sensors_device_data->radar_altit, sensors_device_data->radar_confidance, sensors_device_data->params);
    }
}

/**
 * @brief Encode a sensors_device_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensors_device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensors_device_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensors_device_data_t* sensors_device_data)
{
    return _ttalink_sensors_device_data_encode_chan(dst_addr, src_addr, chan, msg, sensors_device_data, false);
}

/**
 * @brief Encode a sensors_device_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensors_device_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensors_device_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensors_device_data_t* sensors_device_data)
{
    return _ttalink_sensors_device_data_encode_chan(dst_addr, src_addr, chan, msg, sensors_device_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sensors_device_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_uint8_t(buf, 16, radar_confidance);
    _tta_put_float_array(buf, 17, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSORS_DEVICE_DATA, buf, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC, nocrc);
#else
    ttalink_sensors_device_data_t packet;
    packet.update_time = update_time;
    packet.radar_latit = radar_latit;
    packet.radar_longi = radar_longi;
    packet.radar_altit = radar_altit;
    packet.radar_confidance = radar_confidance;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSORS_DEVICE_DATA, (const char *)&packet, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensors_device_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param radar_confidance  雷达可信度.
 * @param params  .
 */
static inline void ttalink_sensors_device_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params)
{
    _ttalink_sensors_device_data_send(dst_addr, src_addr, chan, update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, false);
}

/**
 * @brief Send a sensors_device_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param radar_confidance  雷达可信度.
 * @param params  .
 */
static inline void ttalink_sensors_device_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params)
{
    _ttalink_sensors_device_data_send(dst_addr, src_addr, chan, update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, true);
}


static inline void _ttalink_sensors_device_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensors_device_data_t* sensors_device_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sensors_device_data_send_nocrc(dst_addr, src_addr, chan, sensors_device_data->update_time, sensors_device_data->radar_latit, sensors_device_data->radar_longi, sensors_device_data->radar_altit, sensors_device_data->radar_confidance, sensors_device_data->params);
    }else{
        ttalink_sensors_device_data_send(dst_addr, src_addr, chan, sensors_device_data->update_time, sensors_device_data->radar_latit, sensors_device_data->radar_longi, sensors_device_data->radar_altit, sensors_device_data->radar_confidance, sensors_device_data->params);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSORS_DEVICE_DATA, (const char *)sensors_device_data, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensors_device_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensors_device_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensors_device_data_t* sensors_device_data)
{
    _ttalink_sensors_device_data_send_struct(dst_addr, src_addr, chan, sensors_device_data, false);
}

/**
 * @brief Send a sensors_device_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensors_device_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensors_device_data_t* sensors_device_data)
{
    _ttalink_sensors_device_data_send_struct(dst_addr, src_addr, chan, sensors_device_data, true);
}

#if TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sensors_device_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_uint8_t(buf, 16, radar_confidance);
    _tta_put_float_array(buf, 17, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSORS_DEVICE_DATA, buf, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC, nocrc);
#else
    ttalink_sensors_device_data_t *packet = (ttalink_sensors_device_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->radar_latit = radar_latit;
    packet->radar_longi = radar_longi;
    packet->radar_altit = radar_altit;
    packet->radar_confidance = radar_confidance;
    tta_array_memcpy(packet->params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSORS_DEVICE_DATA, (const char *)packet, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_MIN_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sensors_device_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params)
{
    _ttalink_sensors_device_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sensors_device_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, uint8_t radar_confidance, const float *params)
{
    _ttalink_sensors_device_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, radar_latit, radar_longi, radar_altit, radar_confidance, params, true);
}
#endif

#endif

// MESSAGE SENSORS_DEVICE_DATA UNPACKING


/**
 * @brief Get field update_time from sensors_device_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_sensors_device_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field radar_latit from sensors_device_data message
 *
 * @return  .
 */
static inline int32_t ttalink_sensors_device_data_get_radar_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field radar_longi from sensors_device_data message
 *
 * @return  .
 */
static inline int32_t ttalink_sensors_device_data_get_radar_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field radar_altit from sensors_device_data message
 *
 * @return  .
 */
static inline float ttalink_sensors_device_data_get_radar_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field radar_confidance from sensors_device_data message
 *
 * @return  雷达可信度.
 */
static inline uint8_t ttalink_sensors_device_data_get_radar_confidance(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field params from sensors_device_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_sensors_device_data_get_params(const ttalink_message_t* msg, float *params)
{
    return _TTA_RETURN_float_array(msg, params, 4,  17);
}

/**
 * @brief Decode a sensors_device_data message into a struct
 *
 * @param msg The message to decode
 * @param sensors_device_data C-struct to decode the message contents into
 */
static inline void ttalink_sensors_device_data_decode(const ttalink_message_t* msg, ttalink_sensors_device_data_t* sensors_device_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sensors_device_data->update_time = ttalink_sensors_device_data_get_update_time(msg);
    sensors_device_data->radar_latit = ttalink_sensors_device_data_get_radar_latit(msg);
    sensors_device_data->radar_longi = ttalink_sensors_device_data_get_radar_longi(msg);
    sensors_device_data->radar_altit = ttalink_sensors_device_data_get_radar_altit(msg);
    sensors_device_data->radar_confidance = ttalink_sensors_device_data_get_radar_confidance(msg);
    ttalink_sensors_device_data_get_params(msg, sensors_device_data->params);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN? msg->len : TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN;
        memset(sensors_device_data, 0, TTALINK_MSG_ID_SENSORS_DEVICE_DATA_LEN);
    memcpy(sensors_device_data, _TTA_PAYLOAD(msg), len);
#endif
}
