#pragma once
// MESSAGE PASS_TC_DEVICE_LOG PACKING

#define TTALINK_MSG_ID_PASS_TC_DEVICE_LOG 2243

TTAPACKED(
typedef struct __ttalink_pass_tc_device_log_t {
 uint16_t size; /*<  当前数据长度.*/
 uint8_t type; /*<  0=RAW_DATA  1=String*/
 uint16_t index; /*<  .*/
 int8_t payload[128]; /*<  payload .*/
}) ttalink_pass_tc_device_log_t;

#define TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN 133
#define TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN 133
#define TTALINK_MSG_ID_2243_LEN 133
#define TTALINK_MSG_ID_2243_MIN_LEN 133

#define TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC 130
#define TTALINK_MSG_ID_2243_CRC 130

#define TTALINK_MSG_PASS_TC_DEVICE_LOG_FIELD_PAYLOAD_LEN 128

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PASS_TC_DEVICE_LOG { \
    2243, \
    "PASS_TC_DEVICE_LOG", \
    4, \
    {  { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pass_tc_device_log_t, size) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pass_tc_device_log_t, type) }, \
         { "index", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pass_tc_device_log_t, index) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 128, 5, offsetof(ttalink_pass_tc_device_log_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PASS_TC_DEVICE_LOG { \
    "PASS_TC_DEVICE_LOG", \
    4, \
    {  { "size", NULL, TTALINK_TYPE_UINT16_T, 0, 0, offsetof(ttalink_pass_tc_device_log_t, size) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_pass_tc_device_log_t, type) }, \
         { "index", NULL, TTALINK_TYPE_UINT16_T, 0, 3, offsetof(ttalink_pass_tc_device_log_t, index) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 128, 5, offsetof(ttalink_pass_tc_device_log_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pass_tc_device_log_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, uint8_t type, uint16_t index, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint8_t(buf, 2, type);
    _tta_put_uint16_t(buf, 3, index);
    _tta_put_int8_t_array(buf, 5, payload, 128);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN);
#else
    ttalink_pass_tc_device_log_t packet;
    packet.size = size;
    packet.type = type;
    packet.index = index;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*128);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_TC_DEVICE_LOG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC, nocrc);
}

/**
 * @brief Pack a pass_tc_device_log message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  当前数据长度.
 * @param type  0=RAW_DATA  1=String
 * @param index  .
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_tc_device_log_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, uint8_t type, uint16_t index, const int8_t *payload)
{
    return _ttalink_pass_tc_device_log_pack(dst_addr, src_addr, msg,  size, type, index, payload, false);
}

/**
 * @brief Pack a pass_tc_device_log message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param size  当前数据长度.
 * @param type  0=RAW_DATA  1=String
 * @param index  .
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_tc_device_log_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint16_t size, uint8_t type, uint16_t index, const int8_t *payload)
{
    return _ttalink_pass_tc_device_log_pack(dst_addr, src_addr, msg,  size, type, index, payload, true);
}


static inline uint16_t _ttalink_pass_tc_device_log_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,uint8_t type,uint16_t index,const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint8_t(buf, 2, type);
    _tta_put_uint16_t(buf, 3, index);
    _tta_put_int8_t_array(buf, 5, payload, 128);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN);
#else
    ttalink_pass_tc_device_log_t packet;
    packet.size = size;
    packet.type = type;
    packet.index = index;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*128);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PASS_TC_DEVICE_LOG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC, nocrc);
}

/**
 * @brief Pack a pass_tc_device_log message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  当前数据长度.
 * @param type  0=RAW_DATA  1=String
 * @param index  .
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_tc_device_log_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,uint8_t type,uint16_t index,const int8_t *payload)
{
    return _ttalink_pass_tc_device_log_pack_chan(dst_addr, src_addr, chan, msg,  size, type, index, payload, false);
}

/**
 * @brief Pack a pass_tc_device_log message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param size  当前数据长度.
 * @param type  0=RAW_DATA  1=String
 * @param index  .
 * @param payload  payload .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pass_tc_device_log_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint16_t size,uint8_t type,uint16_t index,const int8_t *payload)
{
    return _ttalink_pass_tc_device_log_pack_chan(dst_addr, src_addr, chan, msg,  size, type, index, payload, true);
}


static inline uint16_t _ttalink_pass_tc_device_log_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_tc_device_log_t* pass_tc_device_log, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_tc_device_log_pack_nocrc(dst_addr, src_addr, msg, pass_tc_device_log->size, pass_tc_device_log->type, pass_tc_device_log->index, pass_tc_device_log->payload);
    }else{
        return ttalink_pass_tc_device_log_pack(dst_addr, src_addr, msg, pass_tc_device_log->size, pass_tc_device_log->type, pass_tc_device_log->index, pass_tc_device_log->payload);
    }
    
}

/**
 * @brief Encode a pass_tc_device_log struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_tc_device_log C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_tc_device_log_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_tc_device_log_t* pass_tc_device_log)
{
    return _ttalink_pass_tc_device_log_encode(dst_addr, src_addr, msg, pass_tc_device_log, false);
}

/**
 * @brief Encode a pass_tc_device_log struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pass_tc_device_log C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_tc_device_log_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pass_tc_device_log_t* pass_tc_device_log)
{
    return _ttalink_pass_tc_device_log_encode(dst_addr, src_addr, msg, pass_tc_device_log, true);
}


static inline uint16_t _ttalink_pass_tc_device_log_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_tc_device_log_t* pass_tc_device_log, bool nocrc)
{
    if(nocrc){
        return ttalink_pass_tc_device_log_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pass_tc_device_log->size, pass_tc_device_log->type, pass_tc_device_log->index, pass_tc_device_log->payload);
    }else{
        return ttalink_pass_tc_device_log_pack_chan(dst_addr, src_addr, chan, msg, pass_tc_device_log->size, pass_tc_device_log->type, pass_tc_device_log->index, pass_tc_device_log->payload);
    }
}

/**
 * @brief Encode a pass_tc_device_log struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_tc_device_log C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_tc_device_log_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_tc_device_log_t* pass_tc_device_log)
{
    return _ttalink_pass_tc_device_log_encode_chan(dst_addr, src_addr, chan, msg, pass_tc_device_log, false);
}

/**
 * @brief Encode a pass_tc_device_log struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pass_tc_device_log C-struct to read the message contents from
 */
static inline uint16_t ttalink_pass_tc_device_log_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pass_tc_device_log_t* pass_tc_device_log)
{
    return _ttalink_pass_tc_device_log_encode_chan(dst_addr, src_addr, chan, msg, pass_tc_device_log, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pass_tc_device_log_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, uint8_t type, uint16_t index, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN];
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint8_t(buf, 2, type);
    _tta_put_uint16_t(buf, 3, index);
    _tta_put_int8_t_array(buf, 5, payload, 128);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG, buf, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC, nocrc);
#else
    ttalink_pass_tc_device_log_t packet;
    packet.size = size;
    packet.type = type;
    packet.index = index;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*128);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG, (const char *)&packet, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_tc_device_log message
 * @param chan TTAlink channel to send the message
 *
 * @param size  当前数据长度.
 * @param type  0=RAW_DATA  1=String
 * @param index  .
 * @param payload  payload .
 */
static inline void ttalink_pass_tc_device_log_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, uint8_t type, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_tc_device_log_send(dst_addr, src_addr, chan, size, type, index, payload, false);
}

/**
 * @brief Send a pass_tc_device_log message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param size  当前数据长度.
 * @param type  0=RAW_DATA  1=String
 * @param index  .
 * @param payload  payload .
 */
static inline void ttalink_pass_tc_device_log_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint16_t size, uint8_t type, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_tc_device_log_send(dst_addr, src_addr, chan, size, type, index, payload, true);
}


static inline void _ttalink_pass_tc_device_log_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_tc_device_log_t* pass_tc_device_log, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pass_tc_device_log_send_nocrc(dst_addr, src_addr, chan, pass_tc_device_log->size, pass_tc_device_log->type, pass_tc_device_log->index, pass_tc_device_log->payload);
    }else{
        ttalink_pass_tc_device_log_send(dst_addr, src_addr, chan, pass_tc_device_log->size, pass_tc_device_log->type, pass_tc_device_log->index, pass_tc_device_log->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG, (const char *)pass_tc_device_log, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC, nocrc);
#endif
}

/**
 * @brief Send a pass_tc_device_log message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_tc_device_log_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_tc_device_log_t* pass_tc_device_log)
{
    _ttalink_pass_tc_device_log_send_struct(dst_addr, src_addr, chan, pass_tc_device_log, false);
}

/**
 * @brief Send a pass_tc_device_log message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pass_tc_device_log_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pass_tc_device_log_t* pass_tc_device_log)
{
    _ttalink_pass_tc_device_log_send_struct(dst_addr, src_addr, chan, pass_tc_device_log, true);
}

#if TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pass_tc_device_log_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, uint8_t type, uint16_t index, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint16_t(buf, 0, size);
    _tta_put_uint8_t(buf, 2, type);
    _tta_put_uint16_t(buf, 3, index);
    _tta_put_int8_t_array(buf, 5, payload, 128);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG, buf, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC, nocrc);
#else
    ttalink_pass_tc_device_log_t *packet = (ttalink_pass_tc_device_log_t *)msgbuf;
    packet->size = size;
    packet->type = type;
    packet->index = index;
    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*128);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG, (const char *)packet, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_MIN_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pass_tc_device_log_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, uint8_t type, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_tc_device_log_send_buf(dst_addr, src_addr, msgbuf, chan, size, type, index, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pass_tc_device_log_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint16_t size, uint8_t type, uint16_t index, const int8_t *payload)
{
    _ttalink_pass_tc_device_log_send_buf(dst_addr, src_addr, msgbuf, chan, size, type, index, payload, true);
}
#endif

#endif

// MESSAGE PASS_TC_DEVICE_LOG UNPACKING


/**
 * @brief Get field size from pass_tc_device_log message
 *
 * @return  当前数据长度.
 */
static inline uint16_t ttalink_pass_tc_device_log_get_size(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field type from pass_tc_device_log message
 *
 * @return  0=RAW_DATA  1=String
 */
static inline uint8_t ttalink_pass_tc_device_log_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field index from pass_tc_device_log message
 *
 * @return  .
 */
static inline uint16_t ttalink_pass_tc_device_log_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field payload from pass_tc_device_log message
 *
 * @return  payload .
 */
static inline uint16_t ttalink_pass_tc_device_log_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 128,  5);
}

/**
 * @brief Decode a pass_tc_device_log message into a struct
 *
 * @param msg The message to decode
 * @param pass_tc_device_log C-struct to decode the message contents into
 */
static inline void ttalink_pass_tc_device_log_decode(const ttalink_message_t* msg, ttalink_pass_tc_device_log_t* pass_tc_device_log)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pass_tc_device_log->size = ttalink_pass_tc_device_log_get_size(msg);
    pass_tc_device_log->type = ttalink_pass_tc_device_log_get_type(msg);
    pass_tc_device_log->index = ttalink_pass_tc_device_log_get_index(msg);
    ttalink_pass_tc_device_log_get_payload(msg, pass_tc_device_log->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN? msg->len : TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN;
        memset(pass_tc_device_log, 0, TTALINK_MSG_ID_PASS_TC_DEVICE_LOG_LEN);
    memcpy(pass_tc_device_log, _TTA_PAYLOAD(msg), len);
#endif
}
