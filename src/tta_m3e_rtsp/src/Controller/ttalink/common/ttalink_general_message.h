#pragma once
// MESSAGE GENERAL_MESSAGE PACKING

#define TTALINK_MSG_ID_GENERAL_MESSAGE 2240

TTAPACKED(
typedef struct __ttalink_general_message_t {
 uint8_t general_id; /*<  .*/
 uint8_t total_msgs; /*<  .*/
 int8_t general_msg_name[10]; /*<  .*/
 uint8_t params_size; /*<  .*/
 uint8_t params_type; /*<  .*/
 int8_t params_buf[60]; /*<  .*/
}) ttalink_general_message_t;

#define TTALINK_MSG_ID_GENERAL_MESSAGE_LEN 74
#define TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN 74
#define TTALINK_MSG_ID_2240_LEN 74
#define TTALINK_MSG_ID_2240_MIN_LEN 74

#define TTALINK_MSG_ID_GENERAL_MESSAGE_CRC 86
#define TTALINK_MSG_ID_2240_CRC 86

#define TTALINK_MSG_GENERAL_MESSAGE_FIELD_GENERAL_MSG_NAME_LEN 10
#define TTALINK_MSG_GENERAL_MESSAGE_FIELD_PARAMS_BUF_LEN 60

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_MESSAGE { \
    2240, \
    "GENERAL_MESSAGE", \
    6, \
    {  { "general_id", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_message_t, general_id) }, \
         { "total_msgs", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_general_message_t, total_msgs) }, \
         { "general_msg_name", NULL, TTALINK_TYPE_INT8_T, 10, 2, offsetof(ttalink_general_message_t, general_msg_name) }, \
         { "params_size", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_general_message_t, params_size) }, \
         { "params_type", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_general_message_t, params_type) }, \
         { "params_buf", NULL, TTALINK_TYPE_INT8_T, 60, 14, offsetof(ttalink_general_message_t, params_buf) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_MESSAGE { \
    "GENERAL_MESSAGE", \
    6, \
    {  { "general_id", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_message_t, general_id) }, \
         { "total_msgs", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_general_message_t, total_msgs) }, \
         { "general_msg_name", NULL, TTALINK_TYPE_INT8_T, 10, 2, offsetof(ttalink_general_message_t, general_msg_name) }, \
         { "params_size", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_general_message_t, params_size) }, \
         { "params_type", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_general_message_t, params_type) }, \
         { "params_buf", NULL, TTALINK_TYPE_INT8_T, 60, 14, offsetof(ttalink_general_message_t, params_buf) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_message_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_MESSAGE_LEN];
    _tta_put_uint8_t(buf, 0, general_id);
    _tta_put_uint8_t(buf, 1, total_msgs);
    _tta_put_uint8_t(buf, 12, params_size);
    _tta_put_uint8_t(buf, 13, params_type);
    _tta_put_int8_t_array(buf, 2, general_msg_name, 10);
    _tta_put_int8_t_array(buf, 14, params_buf, 60);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN);
#else
    ttalink_general_message_t packet;
    packet.general_id = general_id;
    packet.total_msgs = total_msgs;
    packet.params_size = params_size;
    packet.params_type = params_type;
    tta_array_memcpy(packet.general_msg_name, general_msg_name, sizeof(int8_t)*10);
    tta_array_memcpy(packet.params_buf, params_buf, sizeof(int8_t)*60);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_MESSAGE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_CRC, nocrc);
}

/**
 * @brief Pack a general_message message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param general_id  .
 * @param total_msgs  .
 * @param general_msg_name  .
 * @param params_size  .
 * @param params_type  .
 * @param params_buf  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_message_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf)
{
    return _ttalink_general_message_pack(dst_addr, src_addr, msg,  general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, false);
}

/**
 * @brief Pack a general_message message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param general_id  .
 * @param total_msgs  .
 * @param general_msg_name  .
 * @param params_size  .
 * @param params_type  .
 * @param params_buf  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_message_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf)
{
    return _ttalink_general_message_pack(dst_addr, src_addr, msg,  general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, true);
}


static inline uint16_t _ttalink_general_message_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t general_id,uint8_t total_msgs,const int8_t *general_msg_name,uint8_t params_size,uint8_t params_type,const int8_t *params_buf, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_MESSAGE_LEN];
    _tta_put_uint8_t(buf, 0, general_id);
    _tta_put_uint8_t(buf, 1, total_msgs);
    _tta_put_uint8_t(buf, 12, params_size);
    _tta_put_uint8_t(buf, 13, params_type);
    _tta_put_int8_t_array(buf, 2, general_msg_name, 10);
    _tta_put_int8_t_array(buf, 14, params_buf, 60);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN);
#else
    ttalink_general_message_t packet;
    packet.general_id = general_id;
    packet.total_msgs = total_msgs;
    packet.params_size = params_size;
    packet.params_type = params_type;
    tta_array_memcpy(packet.general_msg_name, general_msg_name, sizeof(int8_t)*10);
    tta_array_memcpy(packet.params_buf, params_buf, sizeof(int8_t)*60);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_MESSAGE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_CRC, nocrc);
}

/**
 * @brief Pack a general_message message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_id  .
 * @param total_msgs  .
 * @param general_msg_name  .
 * @param params_size  .
 * @param params_type  .
 * @param params_buf  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_message_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t general_id,uint8_t total_msgs,const int8_t *general_msg_name,uint8_t params_size,uint8_t params_type,const int8_t *params_buf)
{
    return _ttalink_general_message_pack_chan(dst_addr, src_addr, chan, msg,  general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, false);
}

/**
 * @brief Pack a general_message message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_id  .
 * @param total_msgs  .
 * @param general_msg_name  .
 * @param params_size  .
 * @param params_type  .
 * @param params_buf  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_message_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t general_id,uint8_t total_msgs,const int8_t *general_msg_name,uint8_t params_size,uint8_t params_type,const int8_t *params_buf)
{
    return _ttalink_general_message_pack_chan(dst_addr, src_addr, chan, msg,  general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, true);
}


static inline uint16_t _ttalink_general_message_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_message_t* general_message, bool nocrc)
{
    if(nocrc){
        return ttalink_general_message_pack_nocrc(dst_addr, src_addr, msg, general_message->general_id, general_message->total_msgs, general_message->general_msg_name, general_message->params_size, general_message->params_type, general_message->params_buf);
    }else{
        return ttalink_general_message_pack(dst_addr, src_addr, msg, general_message->general_id, general_message->total_msgs, general_message->general_msg_name, general_message->params_size, general_message->params_type, general_message->params_buf);
    }
    
}

/**
 * @brief Encode a general_message struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_message_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_message_t* general_message)
{
    return _ttalink_general_message_encode(dst_addr, src_addr, msg, general_message, false);
}

/**
 * @brief Encode a general_message struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_message_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_message_t* general_message)
{
    return _ttalink_general_message_encode(dst_addr, src_addr, msg, general_message, true);
}


static inline uint16_t _ttalink_general_message_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_message_t* general_message, bool nocrc)
{
    if(nocrc){
        return ttalink_general_message_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_message->general_id, general_message->total_msgs, general_message->general_msg_name, general_message->params_size, general_message->params_type, general_message->params_buf);
    }else{
        return ttalink_general_message_pack_chan(dst_addr, src_addr, chan, msg, general_message->general_id, general_message->total_msgs, general_message->general_msg_name, general_message->params_size, general_message->params_type, general_message->params_buf);
    }
}

/**
 * @brief Encode a general_message struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_message_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_message_t* general_message)
{
    return _ttalink_general_message_encode_chan(dst_addr, src_addr, chan, msg, general_message, false);
}

/**
 * @brief Encode a general_message struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_message C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_message_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_message_t* general_message)
{
    return _ttalink_general_message_encode_chan(dst_addr, src_addr, chan, msg, general_message, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_message_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_MESSAGE_LEN];
    _tta_put_uint8_t(buf, 0, general_id);
    _tta_put_uint8_t(buf, 1, total_msgs);
    _tta_put_uint8_t(buf, 12, params_size);
    _tta_put_uint8_t(buf, 13, params_type);
    _tta_put_int8_t_array(buf, 2, general_msg_name, 10);
    _tta_put_int8_t_array(buf, 14, params_buf, 60);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_MESSAGE, buf, TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_CRC, nocrc);
#else
    ttalink_general_message_t packet;
    packet.general_id = general_id;
    packet.total_msgs = total_msgs;
    packet.params_size = params_size;
    packet.params_type = params_type;
    tta_array_memcpy(packet.general_msg_name, general_msg_name, sizeof(int8_t)*10);
    tta_array_memcpy(packet.params_buf, params_buf, sizeof(int8_t)*60);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_MESSAGE, (const char *)&packet, TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_message message
 * @param chan TTAlink channel to send the message
 *
 * @param general_id  .
 * @param total_msgs  .
 * @param general_msg_name  .
 * @param params_size  .
 * @param params_type  .
 * @param params_buf  .
 */
static inline void ttalink_general_message_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf)
{
    _ttalink_general_message_send(dst_addr, src_addr, chan, general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, false);
}

/**
 * @brief Send a general_message message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param general_id  .
 * @param total_msgs  .
 * @param general_msg_name  .
 * @param params_size  .
 * @param params_type  .
 * @param params_buf  .
 */
static inline void ttalink_general_message_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf)
{
    _ttalink_general_message_send(dst_addr, src_addr, chan, general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, true);
}


static inline void _ttalink_general_message_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_message_t* general_message, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_message_send_nocrc(dst_addr, src_addr, chan, general_message->general_id, general_message->total_msgs, general_message->general_msg_name, general_message->params_size, general_message->params_type, general_message->params_buf);
    }else{
        ttalink_general_message_send(dst_addr, src_addr, chan, general_message->general_id, general_message->total_msgs, general_message->general_msg_name, general_message->params_size, general_message->params_type, general_message->params_buf);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_MESSAGE, (const char *)general_message, TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_message message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_message_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_message_t* general_message)
{
    _ttalink_general_message_send_struct(dst_addr, src_addr, chan, general_message, false);
}

/**
 * @brief Send a general_message message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_message_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_message_t* general_message)
{
    _ttalink_general_message_send_struct(dst_addr, src_addr, chan, general_message, true);
}

#if TTALINK_MSG_ID_GENERAL_MESSAGE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_message_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, general_id);
    _tta_put_uint8_t(buf, 1, total_msgs);
    _tta_put_uint8_t(buf, 12, params_size);
    _tta_put_uint8_t(buf, 13, params_type);
    _tta_put_int8_t_array(buf, 2, general_msg_name, 10);
    _tta_put_int8_t_array(buf, 14, params_buf, 60);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_MESSAGE, buf, TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_CRC, nocrc);
#else
    ttalink_general_message_t *packet = (ttalink_general_message_t *)msgbuf;
    packet->general_id = general_id;
    packet->total_msgs = total_msgs;
    packet->params_size = params_size;
    packet->params_type = params_type;
    tta_array_memcpy(packet->general_msg_name, general_msg_name, sizeof(int8_t)*10);
    tta_array_memcpy(packet->params_buf, params_buf, sizeof(int8_t)*60);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_MESSAGE, (const char *)packet, TTALINK_MSG_ID_GENERAL_MESSAGE_MIN_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN, TTALINK_MSG_ID_GENERAL_MESSAGE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_message_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf)
{
    _ttalink_general_message_send_buf(dst_addr, src_addr, msgbuf, chan, general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_message_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t general_id, uint8_t total_msgs, const int8_t *general_msg_name, uint8_t params_size, uint8_t params_type, const int8_t *params_buf)
{
    _ttalink_general_message_send_buf(dst_addr, src_addr, msgbuf, chan, general_id, total_msgs, general_msg_name, params_size, params_type, params_buf, true);
}
#endif

#endif

// MESSAGE GENERAL_MESSAGE UNPACKING


/**
 * @brief Get field general_id from general_message message
 *
 * @return  .
 */
static inline uint8_t ttalink_general_message_get_general_id(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field total_msgs from general_message message
 *
 * @return  .
 */
static inline uint8_t ttalink_general_message_get_total_msgs(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field general_msg_name from general_message message
 *
 * @return  .
 */
static inline uint16_t ttalink_general_message_get_general_msg_name(const ttalink_message_t* msg, int8_t *general_msg_name)
{
    return _TTA_RETURN_int8_t_array(msg, general_msg_name, 10,  2);
}

/**
 * @brief Get field params_size from general_message message
 *
 * @return  .
 */
static inline uint8_t ttalink_general_message_get_params_size(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field params_type from general_message message
 *
 * @return  .
 */
static inline uint8_t ttalink_general_message_get_params_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field params_buf from general_message message
 *
 * @return  .
 */
static inline uint16_t ttalink_general_message_get_params_buf(const ttalink_message_t* msg, int8_t *params_buf)
{
    return _TTA_RETURN_int8_t_array(msg, params_buf, 60,  14);
}

/**
 * @brief Decode a general_message message into a struct
 *
 * @param msg The message to decode
 * @param general_message C-struct to decode the message contents into
 */
static inline void ttalink_general_message_decode(const ttalink_message_t* msg, ttalink_general_message_t* general_message)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_message->general_id = ttalink_general_message_get_general_id(msg);
    general_message->total_msgs = ttalink_general_message_get_total_msgs(msg);
    ttalink_general_message_get_general_msg_name(msg, general_message->general_msg_name);
    general_message->params_size = ttalink_general_message_get_params_size(msg);
    general_message->params_type = ttalink_general_message_get_params_type(msg);
    ttalink_general_message_get_params_buf(msg, general_message->params_buf);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_MESSAGE_LEN? msg->len : TTALINK_MSG_ID_GENERAL_MESSAGE_LEN;
        memset(general_message, 0, TTALINK_MSG_ID_GENERAL_MESSAGE_LEN);
    memcpy(general_message, _TTA_PAYLOAD(msg), len);
#endif
}
