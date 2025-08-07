#pragma once
// MESSAGE TC_9021_SEND_OBSTACLE_DATA PACKING

#define TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA 9021

TTAPACKED(
typedef struct __ttalink_tc_9021_send_obstacle_data_t {
 uint32_t update_time; /*<  update_time*/
 int32_t radar_latit; /*<  .*/
 int32_t radar_longi; /*<  .*/
 float radar_altit; /*<  .*/
 int8_t wp_index; /*<  航线条数.*/
}) ttalink_tc_9021_send_obstacle_data_t;

#define TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN 17
#define TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN 17
#define TTALINK_MSG_ID_9021_LEN 17
#define TTALINK_MSG_ID_9021_MIN_LEN 17

#define TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC 84
#define TTALINK_MSG_ID_9021_CRC 84



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9021_SEND_OBSTACLE_DATA { \
    9021, \
    "TC_9021_SEND_OBSTACLE_DATA", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9021_send_obstacle_data_t, update_time) }, \
         { "radar_latit", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_tc_9021_send_obstacle_data_t, radar_latit) }, \
         { "radar_longi", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_tc_9021_send_obstacle_data_t, radar_longi) }, \
         { "radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_tc_9021_send_obstacle_data_t, radar_altit) }, \
         { "wp_index", NULL, TTALINK_TYPE_INT8_T, 0, 16, offsetof(ttalink_tc_9021_send_obstacle_data_t, wp_index) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9021_SEND_OBSTACLE_DATA { \
    "TC_9021_SEND_OBSTACLE_DATA", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9021_send_obstacle_data_t, update_time) }, \
         { "radar_latit", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_tc_9021_send_obstacle_data_t, radar_latit) }, \
         { "radar_longi", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_tc_9021_send_obstacle_data_t, radar_longi) }, \
         { "radar_altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_tc_9021_send_obstacle_data_t, radar_altit) }, \
         { "wp_index", NULL, TTALINK_TYPE_INT8_T, 0, 16, offsetof(ttalink_tc_9021_send_obstacle_data_t, wp_index) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9021_send_obstacle_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_int8_t(buf, 16, wp_index);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN);
#else
    ttalink_tc_9021_send_obstacle_data_t packet;
    packet.update_time = update_time;
    packet.radar_latit = radar_latit;
    packet.radar_longi = radar_longi;
    packet.radar_altit = radar_altit;
    packet.wp_index = wp_index;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9021_send_obstacle_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param wp_index  航线条数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index)
{
    return _ttalink_tc_9021_send_obstacle_data_pack(dst_addr, src_addr, msg,  update_time, radar_latit, radar_longi, radar_altit, wp_index, false);
}

/**
 * @brief Pack a tc_9021_send_obstacle_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param wp_index  航线条数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index)
{
    return _ttalink_tc_9021_send_obstacle_data_pack(dst_addr, src_addr, msg,  update_time, radar_latit, radar_longi, radar_altit, wp_index, true);
}


static inline uint16_t _ttalink_tc_9021_send_obstacle_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t radar_latit,int32_t radar_longi,float radar_altit,int8_t wp_index, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_int8_t(buf, 16, wp_index);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN);
#else
    ttalink_tc_9021_send_obstacle_data_t packet;
    packet.update_time = update_time;
    packet.radar_latit = radar_latit;
    packet.radar_longi = radar_longi;
    packet.radar_altit = radar_altit;
    packet.wp_index = wp_index;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9021_send_obstacle_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param wp_index  航线条数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t radar_latit,int32_t radar_longi,float radar_altit,int8_t wp_index)
{
    return _ttalink_tc_9021_send_obstacle_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, radar_latit, radar_longi, radar_altit, wp_index, false);
}

/**
 * @brief Pack a tc_9021_send_obstacle_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param wp_index  航线条数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t radar_latit,int32_t radar_longi,float radar_altit,int8_t wp_index)
{
    return _ttalink_tc_9021_send_obstacle_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, radar_latit, radar_longi, radar_altit, wp_index, true);
}


static inline uint16_t _ttalink_tc_9021_send_obstacle_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9021_send_obstacle_data_pack_nocrc(dst_addr, src_addr, msg, tc_9021_send_obstacle_data->update_time, tc_9021_send_obstacle_data->radar_latit, tc_9021_send_obstacle_data->radar_longi, tc_9021_send_obstacle_data->radar_altit, tc_9021_send_obstacle_data->wp_index);
    }else{
        return ttalink_tc_9021_send_obstacle_data_pack(dst_addr, src_addr, msg, tc_9021_send_obstacle_data->update_time, tc_9021_send_obstacle_data->radar_latit, tc_9021_send_obstacle_data->radar_longi, tc_9021_send_obstacle_data->radar_altit, tc_9021_send_obstacle_data->wp_index);
    }
    
}

/**
 * @brief Encode a tc_9021_send_obstacle_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9021_send_obstacle_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data)
{
    return _ttalink_tc_9021_send_obstacle_data_encode(dst_addr, src_addr, msg, tc_9021_send_obstacle_data, false);
}

/**
 * @brief Encode a tc_9021_send_obstacle_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9021_send_obstacle_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data)
{
    return _ttalink_tc_9021_send_obstacle_data_encode(dst_addr, src_addr, msg, tc_9021_send_obstacle_data, true);
}


static inline uint16_t _ttalink_tc_9021_send_obstacle_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9021_send_obstacle_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9021_send_obstacle_data->update_time, tc_9021_send_obstacle_data->radar_latit, tc_9021_send_obstacle_data->radar_longi, tc_9021_send_obstacle_data->radar_altit, tc_9021_send_obstacle_data->wp_index);
    }else{
        return ttalink_tc_9021_send_obstacle_data_pack_chan(dst_addr, src_addr, chan, msg, tc_9021_send_obstacle_data->update_time, tc_9021_send_obstacle_data->radar_latit, tc_9021_send_obstacle_data->radar_longi, tc_9021_send_obstacle_data->radar_altit, tc_9021_send_obstacle_data->wp_index);
    }
}

/**
 * @brief Encode a tc_9021_send_obstacle_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9021_send_obstacle_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data)
{
    return _ttalink_tc_9021_send_obstacle_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9021_send_obstacle_data, false);
}

/**
 * @brief Encode a tc_9021_send_obstacle_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9021_send_obstacle_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9021_send_obstacle_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data)
{
    return _ttalink_tc_9021_send_obstacle_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9021_send_obstacle_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9021_send_obstacle_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_int8_t(buf, 16, wp_index);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA, buf, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC, nocrc);
#else
    ttalink_tc_9021_send_obstacle_data_t packet;
    packet.update_time = update_time;
    packet.radar_latit = radar_latit;
    packet.radar_longi = radar_longi;
    packet.radar_altit = radar_altit;
    packet.wp_index = wp_index;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA, (const char *)&packet, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9021_send_obstacle_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param wp_index  航线条数.
 */
static inline void ttalink_tc_9021_send_obstacle_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index)
{
    _ttalink_tc_9021_send_obstacle_data_send(dst_addr, src_addr, chan, update_time, radar_latit, radar_longi, radar_altit, wp_index, false);
}

/**
 * @brief Send a tc_9021_send_obstacle_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param radar_latit  .
 * @param radar_longi  .
 * @param radar_altit  .
 * @param wp_index  航线条数.
 */
static inline void ttalink_tc_9021_send_obstacle_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index)
{
    _ttalink_tc_9021_send_obstacle_data_send(dst_addr, src_addr, chan, update_time, radar_latit, radar_longi, radar_altit, wp_index, true);
}


static inline void _ttalink_tc_9021_send_obstacle_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9021_send_obstacle_data_send_nocrc(dst_addr, src_addr, chan, tc_9021_send_obstacle_data->update_time, tc_9021_send_obstacle_data->radar_latit, tc_9021_send_obstacle_data->radar_longi, tc_9021_send_obstacle_data->radar_altit, tc_9021_send_obstacle_data->wp_index);
    }else{
        ttalink_tc_9021_send_obstacle_data_send(dst_addr, src_addr, chan, tc_9021_send_obstacle_data->update_time, tc_9021_send_obstacle_data->radar_latit, tc_9021_send_obstacle_data->radar_longi, tc_9021_send_obstacle_data->radar_altit, tc_9021_send_obstacle_data->wp_index);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA, (const char *)tc_9021_send_obstacle_data, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9021_send_obstacle_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9021_send_obstacle_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data)
{
    _ttalink_tc_9021_send_obstacle_data_send_struct(dst_addr, src_addr, chan, tc_9021_send_obstacle_data, false);
}

/**
 * @brief Send a tc_9021_send_obstacle_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9021_send_obstacle_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data)
{
    _ttalink_tc_9021_send_obstacle_data_send_struct(dst_addr, src_addr, chan, tc_9021_send_obstacle_data, true);
}

#if TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9021_send_obstacle_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, radar_latit);
    _tta_put_int32_t(buf, 8, radar_longi);
    _tta_put_float(buf, 12, radar_altit);
    _tta_put_int8_t(buf, 16, wp_index);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA, buf, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC, nocrc);
#else
    ttalink_tc_9021_send_obstacle_data_t *packet = (ttalink_tc_9021_send_obstacle_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->radar_latit = radar_latit;
    packet->radar_longi = radar_longi;
    packet->radar_altit = radar_altit;
    packet->wp_index = wp_index;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA, (const char *)packet, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9021_send_obstacle_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index)
{
    _ttalink_tc_9021_send_obstacle_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, radar_latit, radar_longi, radar_altit, wp_index, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9021_send_obstacle_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t radar_latit, int32_t radar_longi, float radar_altit, int8_t wp_index)
{
    _ttalink_tc_9021_send_obstacle_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, radar_latit, radar_longi, radar_altit, wp_index, true);
}
#endif

#endif

// MESSAGE TC_9021_SEND_OBSTACLE_DATA UNPACKING


/**
 * @brief Get field update_time from tc_9021_send_obstacle_data message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9021_send_obstacle_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field radar_latit from tc_9021_send_obstacle_data message
 *
 * @return  .
 */
static inline int32_t ttalink_tc_9021_send_obstacle_data_get_radar_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field radar_longi from tc_9021_send_obstacle_data message
 *
 * @return  .
 */
static inline int32_t ttalink_tc_9021_send_obstacle_data_get_radar_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field radar_altit from tc_9021_send_obstacle_data message
 *
 * @return  .
 */
static inline float ttalink_tc_9021_send_obstacle_data_get_radar_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field wp_index from tc_9021_send_obstacle_data message
 *
 * @return  航线条数.
 */
static inline int8_t ttalink_tc_9021_send_obstacle_data_get_wp_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  16);
}

/**
 * @brief Decode a tc_9021_send_obstacle_data message into a struct
 *
 * @param msg The message to decode
 * @param tc_9021_send_obstacle_data C-struct to decode the message contents into
 */
static inline void ttalink_tc_9021_send_obstacle_data_decode(const ttalink_message_t* msg, ttalink_tc_9021_send_obstacle_data_t* tc_9021_send_obstacle_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9021_send_obstacle_data->update_time = ttalink_tc_9021_send_obstacle_data_get_update_time(msg);
    tc_9021_send_obstacle_data->radar_latit = ttalink_tc_9021_send_obstacle_data_get_radar_latit(msg);
    tc_9021_send_obstacle_data->radar_longi = ttalink_tc_9021_send_obstacle_data_get_radar_longi(msg);
    tc_9021_send_obstacle_data->radar_altit = ttalink_tc_9021_send_obstacle_data_get_radar_altit(msg);
    tc_9021_send_obstacle_data->wp_index = ttalink_tc_9021_send_obstacle_data_get_wp_index(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN? msg->len : TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN;
        memset(tc_9021_send_obstacle_data, 0, TTALINK_MSG_ID_TC_9021_SEND_OBSTACLE_DATA_LEN);
    memcpy(tc_9021_send_obstacle_data, _TTA_PAYLOAD(msg), len);
#endif
}
