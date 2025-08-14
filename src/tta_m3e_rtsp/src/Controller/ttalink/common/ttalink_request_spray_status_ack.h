#pragma once
// MESSAGE REQUEST_SPRAY_STATUS_ACK PACKING

#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK 2167

TTAPACKED(
typedef struct __ttalink_request_spray_status_ack_t {
 uint32_t update_time; /*< .*/
 uint8_t mode; /*<  0=NULL   1=查询  2=设置动作 3=设置设备选择*/
 uint8_t type; /*<  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 */
 uint8_t state; /*<   0=关  1=开 */
 uint8_t ack; /*<   0=正常 非0为错误 */
 float param1; /*<   预留. */
}) ttalink_request_spray_status_ack_t;

#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN 12
#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN 12
#define TTALINK_MSG_ID_2167_LEN 12
#define TTALINK_MSG_ID_2167_MIN_LEN 12

#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC 165
#define TTALINK_MSG_ID_2167_CRC 165



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_SPRAY_STATUS_ACK { \
    2167, \
    "REQUEST_SPRAY_STATUS_ACK", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_request_spray_status_ack_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_request_spray_status_ack_t, mode) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_request_spray_status_ack_t, type) }, \
         { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_request_spray_status_ack_t, state) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_request_spray_status_ack_t, ack) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_request_spray_status_ack_t, param1) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_SPRAY_STATUS_ACK { \
    "REQUEST_SPRAY_STATUS_ACK", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_request_spray_status_ack_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_request_spray_status_ack_t, mode) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_request_spray_status_ack_t, type) }, \
         { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_request_spray_status_ack_t, state) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_request_spray_status_ack_t, ack) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_request_spray_status_ack_t, param1) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_spray_status_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, state);
    _tta_put_uint8_t(buf, 7, ack);
    _tta_put_float(buf, 8, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN);
#else
    ttalink_request_spray_status_ack_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.type = type;
    packet.state = state;
    packet.ack = ack;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_spray_status_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询  2=设置动作 3=设置设备选择
 * @param type  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param state   0=关  1=开 
 * @param ack   0=正常 非0为错误 
 * @param param1   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1)
{
    return _ttalink_request_spray_status_ack_pack(dst_addr, src_addr, msg,  update_time, mode, type, state, ack, param1, false);
}

/**
 * @brief Pack a request_spray_status_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询  2=设置动作 3=设置设备选择
 * @param type  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param state   0=关  1=开 
 * @param ack   0=正常 非0为错误 
 * @param param1   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1)
{
    return _ttalink_request_spray_status_ack_pack(dst_addr, src_addr, msg,  update_time, mode, type, state, ack, param1, true);
}


static inline uint16_t _ttalink_request_spray_status_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t type,uint8_t state,uint8_t ack,float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, state);
    _tta_put_uint8_t(buf, 7, ack);
    _tta_put_float(buf, 8, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN);
#else
    ttalink_request_spray_status_ack_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.type = type;
    packet.state = state;
    packet.ack = ack;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_spray_status_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param mode  0=NULL   1=查询  2=设置动作 3=设置设备选择
 * @param type  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param state   0=关  1=开 
 * @param ack   0=正常 非0为错误 
 * @param param1   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t type,uint8_t state,uint8_t ack,float param1)
{
    return _ttalink_request_spray_status_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, type, state, ack, param1, false);
}

/**
 * @brief Pack a request_spray_status_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param mode  0=NULL   1=查询  2=设置动作 3=设置设备选择
 * @param type  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param state   0=关  1=开 
 * @param ack   0=正常 非0为错误 
 * @param param1   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t type,uint8_t state,uint8_t ack,float param1)
{
    return _ttalink_request_spray_status_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, type, state, ack, param1, true);
}


static inline uint16_t _ttalink_request_spray_status_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_spray_status_ack_t* request_spray_status_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_spray_status_ack_pack_nocrc(dst_addr, src_addr, msg, request_spray_status_ack->update_time, request_spray_status_ack->mode, request_spray_status_ack->type, request_spray_status_ack->state, request_spray_status_ack->ack, request_spray_status_ack->param1);
    }else{
        return ttalink_request_spray_status_ack_pack(dst_addr, src_addr, msg, request_spray_status_ack->update_time, request_spray_status_ack->mode, request_spray_status_ack->type, request_spray_status_ack->state, request_spray_status_ack->ack, request_spray_status_ack->param1);
    }
    
}

/**
 * @brief Encode a request_spray_status_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_spray_status_ack_t* request_spray_status_ack)
{
    return _ttalink_request_spray_status_ack_encode(dst_addr, src_addr, msg, request_spray_status_ack, false);
}

/**
 * @brief Encode a request_spray_status_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_spray_status_ack_t* request_spray_status_ack)
{
    return _ttalink_request_spray_status_ack_encode(dst_addr, src_addr, msg, request_spray_status_ack, true);
}


static inline uint16_t _ttalink_request_spray_status_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_spray_status_ack_t* request_spray_status_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_spray_status_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_spray_status_ack->update_time, request_spray_status_ack->mode, request_spray_status_ack->type, request_spray_status_ack->state, request_spray_status_ack->ack, request_spray_status_ack->param1);
    }else{
        return ttalink_request_spray_status_ack_pack_chan(dst_addr, src_addr, chan, msg, request_spray_status_ack->update_time, request_spray_status_ack->mode, request_spray_status_ack->type, request_spray_status_ack->state, request_spray_status_ack->ack, request_spray_status_ack->param1);
    }
}

/**
 * @brief Encode a request_spray_status_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_spray_status_ack_t* request_spray_status_ack)
{
    return _ttalink_request_spray_status_ack_encode_chan(dst_addr, src_addr, chan, msg, request_spray_status_ack, false);
}

/**
 * @brief Encode a request_spray_status_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_spray_status_ack_t* request_spray_status_ack)
{
    return _ttalink_request_spray_status_ack_encode_chan(dst_addr, src_addr, chan, msg, request_spray_status_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_spray_status_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, state);
    _tta_put_uint8_t(buf, 7, ack);
    _tta_put_float(buf, 8, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK, buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC, nocrc);
#else
    ttalink_request_spray_status_ack_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.type = type;
    packet.state = state;
    packet.ack = ack;
    packet.param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK, (const char *)&packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_spray_status_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询  2=设置动作 3=设置设备选择
 * @param type  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param state   0=关  1=开 
 * @param ack   0=正常 非0为错误 
 * @param param1   预留. 
 */
static inline void ttalink_request_spray_status_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1)
{
    _ttalink_request_spray_status_ack_send(dst_addr, src_addr, chan, update_time, mode, type, state, ack, param1, false);
}

/**
 * @brief Send a request_spray_status_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询  2=设置动作 3=设置设备选择
 * @param type  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param state   0=关  1=开 
 * @param ack   0=正常 非0为错误 
 * @param param1   预留. 
 */
static inline void ttalink_request_spray_status_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1)
{
    _ttalink_request_spray_status_ack_send(dst_addr, src_addr, chan, update_time, mode, type, state, ack, param1, true);
}


static inline void _ttalink_request_spray_status_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_spray_status_ack_t* request_spray_status_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_spray_status_ack_send_nocrc(dst_addr, src_addr, chan, request_spray_status_ack->update_time, request_spray_status_ack->mode, request_spray_status_ack->type, request_spray_status_ack->state, request_spray_status_ack->ack, request_spray_status_ack->param1);
    }else{
        ttalink_request_spray_status_ack_send(dst_addr, src_addr, chan, request_spray_status_ack->update_time, request_spray_status_ack->mode, request_spray_status_ack->type, request_spray_status_ack->state, request_spray_status_ack->ack, request_spray_status_ack->param1);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK, (const char *)request_spray_status_ack, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_spray_status_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_spray_status_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_spray_status_ack_t* request_spray_status_ack)
{
    _ttalink_request_spray_status_ack_send_struct(dst_addr, src_addr, chan, request_spray_status_ack, false);
}

/**
 * @brief Send a request_spray_status_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_spray_status_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_spray_status_ack_t* request_spray_status_ack)
{
    _ttalink_request_spray_status_ack_send_struct(dst_addr, src_addr, chan, request_spray_status_ack, true);
}

#if TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_spray_status_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, state);
    _tta_put_uint8_t(buf, 7, ack);
    _tta_put_float(buf, 8, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK, buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC, nocrc);
#else
    ttalink_request_spray_status_ack_t *packet = (ttalink_request_spray_status_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->mode = mode;
    packet->type = type;
    packet->state = state;
    packet->ack = ack;
    packet->param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK, (const char *)packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_spray_status_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1)
{
    _ttalink_request_spray_status_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, type, state, ack, param1, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_spray_status_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t type, uint8_t state, uint8_t ack, float param1)
{
    _ttalink_request_spray_status_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, type, state, ack, param1, true);
}
#endif

#endif

// MESSAGE REQUEST_SPRAY_STATUS_ACK UNPACKING


/**
 * @brief Get field update_time from request_spray_status_ack message
 *
 * @return .
 */
static inline uint32_t ttalink_request_spray_status_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mode from request_spray_status_ack message
 *
 * @return  0=NULL   1=查询  2=设置动作 3=设置设备选择
 */
static inline uint8_t ttalink_request_spray_status_ack_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field type from request_spray_status_ack message
 *
 * @return  0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 */
static inline uint8_t ttalink_request_spray_status_ack_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field state from request_spray_status_ack message
 *
 * @return   0=关  1=开 
 */
static inline uint8_t ttalink_request_spray_status_ack_get_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field ack from request_spray_status_ack message
 *
 * @return   0=正常 非0为错误 
 */
static inline uint8_t ttalink_request_spray_status_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field param1 from request_spray_status_ack message
 *
 * @return   预留. 
 */
static inline float ttalink_request_spray_status_ack_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Decode a request_spray_status_ack message into a struct
 *
 * @param msg The message to decode
 * @param request_spray_status_ack C-struct to decode the message contents into
 */
static inline void ttalink_request_spray_status_ack_decode(const ttalink_message_t* msg, ttalink_request_spray_status_ack_t* request_spray_status_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_spray_status_ack->update_time = ttalink_request_spray_status_ack_get_update_time(msg);
    request_spray_status_ack->mode = ttalink_request_spray_status_ack_get_mode(msg);
    request_spray_status_ack->type = ttalink_request_spray_status_ack_get_type(msg);
    request_spray_status_ack->state = ttalink_request_spray_status_ack_get_state(msg);
    request_spray_status_ack->ack = ttalink_request_spray_status_ack_get_ack(msg);
    request_spray_status_ack->param1 = ttalink_request_spray_status_ack_get_param1(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN? msg->len : TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN;
        memset(request_spray_status_ack, 0, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_ACK_LEN);
    memcpy(request_spray_status_ack, _TTA_PAYLOAD(msg), len);
#endif
}
