#pragma once
// MESSAGE REQUEST_SPRAY_STATUS PACKING

#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS 2166

TTAPACKED(
typedef struct __ttalink_request_spray_status_t {
 uint32_t update_time; /*< .*/
 uint8_t mode; /*<  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择*/
 uint8_t type; /*<   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 */
 uint8_t ctrl_state; /*<   0=手动关  1=手动开 2=自动开*/
 float data; /*<   控制数值. */
 float param1; /*<   预留. */
 float param2; /*<   预留. */
}) ttalink_request_spray_status_t;

#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN 19
#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN 19
#define TTALINK_MSG_ID_2166_LEN 19
#define TTALINK_MSG_ID_2166_MIN_LEN 19

#define TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC 68
#define TTALINK_MSG_ID_2166_CRC 68



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_SPRAY_STATUS { \
    2166, \
    "REQUEST_SPRAY_STATUS", \
    7, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_request_spray_status_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_request_spray_status_t, mode) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_request_spray_status_t, type) }, \
         { "ctrl_state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_request_spray_status_t, ctrl_state) }, \
         { "data", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_request_spray_status_t, data) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 11, offsetof(ttalink_request_spray_status_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 15, offsetof(ttalink_request_spray_status_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_SPRAY_STATUS { \
    "REQUEST_SPRAY_STATUS", \
    7, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_request_spray_status_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_request_spray_status_t, mode) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_request_spray_status_t, type) }, \
         { "ctrl_state", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_request_spray_status_t, ctrl_state) }, \
         { "data", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_request_spray_status_t, data) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 11, offsetof(ttalink_request_spray_status_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 15, offsetof(ttalink_request_spray_status_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_spray_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, ctrl_state);
    _tta_put_float(buf, 7, data);
    _tta_put_float(buf, 11, param1);
    _tta_put_float(buf, 15, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN);
#else
    ttalink_request_spray_status_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.type = type;
    packet.ctrl_state = ctrl_state;
    packet.data = data;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_SPRAY_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a request_spray_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param ctrl_state   0=手动关  1=手动开 2=自动开
 * @param data   控制数值. 
 * @param param1   预留. 
 * @param param2   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2)
{
    return _ttalink_request_spray_status_pack(dst_addr, src_addr, msg,  update_time, mode, type, ctrl_state, data, param1, param2, false);
}

/**
 * @brief Pack a request_spray_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param ctrl_state   0=手动关  1=手动开 2=自动开
 * @param data   控制数值. 
 * @param param1   预留. 
 * @param param2   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2)
{
    return _ttalink_request_spray_status_pack(dst_addr, src_addr, msg,  update_time, mode, type, ctrl_state, data, param1, param2, true);
}


static inline uint16_t _ttalink_request_spray_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t type,uint8_t ctrl_state,float data,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, ctrl_state);
    _tta_put_float(buf, 7, data);
    _tta_put_float(buf, 11, param1);
    _tta_put_float(buf, 15, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN);
#else
    ttalink_request_spray_status_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.type = type;
    packet.ctrl_state = ctrl_state;
    packet.data = data;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_SPRAY_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a request_spray_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param mode  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param ctrl_state   0=手动关  1=手动开 2=自动开
 * @param data   控制数值. 
 * @param param1   预留. 
 * @param param2   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t type,uint8_t ctrl_state,float data,float param1,float param2)
{
    return _ttalink_request_spray_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, type, ctrl_state, data, param1, param2, false);
}

/**
 * @brief Pack a request_spray_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param mode  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param ctrl_state   0=手动关  1=手动开 2=自动开
 * @param data   控制数值. 
 * @param param1   预留. 
 * @param param2   预留. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_spray_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t type,uint8_t ctrl_state,float data,float param1,float param2)
{
    return _ttalink_request_spray_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, type, ctrl_state, data, param1, param2, true);
}


static inline uint16_t _ttalink_request_spray_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_spray_status_t* request_spray_status, bool nocrc)
{
    if(nocrc){
        return ttalink_request_spray_status_pack_nocrc(dst_addr, src_addr, msg, request_spray_status->update_time, request_spray_status->mode, request_spray_status->type, request_spray_status->ctrl_state, request_spray_status->data, request_spray_status->param1, request_spray_status->param2);
    }else{
        return ttalink_request_spray_status_pack(dst_addr, src_addr, msg, request_spray_status->update_time, request_spray_status->mode, request_spray_status->type, request_spray_status->ctrl_state, request_spray_status->data, request_spray_status->param1, request_spray_status->param2);
    }
    
}

/**
 * @brief Encode a request_spray_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_spray_status_t* request_spray_status)
{
    return _ttalink_request_spray_status_encode(dst_addr, src_addr, msg, request_spray_status, false);
}

/**
 * @brief Encode a request_spray_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_spray_status_t* request_spray_status)
{
    return _ttalink_request_spray_status_encode(dst_addr, src_addr, msg, request_spray_status, true);
}


static inline uint16_t _ttalink_request_spray_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_spray_status_t* request_spray_status, bool nocrc)
{
    if(nocrc){
        return ttalink_request_spray_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_spray_status->update_time, request_spray_status->mode, request_spray_status->type, request_spray_status->ctrl_state, request_spray_status->data, request_spray_status->param1, request_spray_status->param2);
    }else{
        return ttalink_request_spray_status_pack_chan(dst_addr, src_addr, chan, msg, request_spray_status->update_time, request_spray_status->mode, request_spray_status->type, request_spray_status->ctrl_state, request_spray_status->data, request_spray_status->param1, request_spray_status->param2);
    }
}

/**
 * @brief Encode a request_spray_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_spray_status_t* request_spray_status)
{
    return _ttalink_request_spray_status_encode_chan(dst_addr, src_addr, chan, msg, request_spray_status, false);
}

/**
 * @brief Encode a request_spray_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_spray_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_spray_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_spray_status_t* request_spray_status)
{
    return _ttalink_request_spray_status_encode_chan(dst_addr, src_addr, chan, msg, request_spray_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_spray_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, ctrl_state);
    _tta_put_float(buf, 7, data);
    _tta_put_float(buf, 11, param1);
    _tta_put_float(buf, 15, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS, buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC, nocrc);
#else
    ttalink_request_spray_status_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.type = type;
    packet.ctrl_state = ctrl_state;
    packet.data = data;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS, (const char *)&packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_spray_status message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param ctrl_state   0=手动关  1=手动开 2=自动开
 * @param data   控制数值. 
 * @param param1   预留. 
 * @param param2   预留. 
 */
static inline void ttalink_request_spray_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2)
{
    _ttalink_request_spray_status_send(dst_addr, src_addr, chan, update_time, mode, type, ctrl_state, data, param1, param2, false);
}

/**
 * @brief Send a request_spray_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param mode  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 * @param ctrl_state   0=手动关  1=手动开 2=自动开
 * @param data   控制数值. 
 * @param param1   预留. 
 * @param param2   预留. 
 */
static inline void ttalink_request_spray_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2)
{
    _ttalink_request_spray_status_send(dst_addr, src_addr, chan, update_time, mode, type, ctrl_state, data, param1, param2, true);
}


static inline void _ttalink_request_spray_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_spray_status_t* request_spray_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_spray_status_send_nocrc(dst_addr, src_addr, chan, request_spray_status->update_time, request_spray_status->mode, request_spray_status->type, request_spray_status->ctrl_state, request_spray_status->data, request_spray_status->param1, request_spray_status->param2);
    }else{
        ttalink_request_spray_status_send(dst_addr, src_addr, chan, request_spray_status->update_time, request_spray_status->mode, request_spray_status->type, request_spray_status->ctrl_state, request_spray_status->data, request_spray_status->param1, request_spray_status->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS, (const char *)request_spray_status, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_spray_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_spray_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_spray_status_t* request_spray_status)
{
    _ttalink_request_spray_status_send_struct(dst_addr, src_addr, chan, request_spray_status, false);
}

/**
 * @brief Send a request_spray_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_spray_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_spray_status_t* request_spray_status)
{
    _ttalink_request_spray_status_send_struct(dst_addr, src_addr, chan, request_spray_status, true);
}

#if TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_spray_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, type);
    _tta_put_uint8_t(buf, 6, ctrl_state);
    _tta_put_float(buf, 7, data);
    _tta_put_float(buf, 11, param1);
    _tta_put_float(buf, 15, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS, buf, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC, nocrc);
#else
    ttalink_request_spray_status_t *packet = (ttalink_request_spray_status_t *)msgbuf;
    packet->update_time = update_time;
    packet->mode = mode;
    packet->type = type;
    packet->ctrl_state = ctrl_state;
    packet->data = data;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS, (const char *)packet, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_MIN_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_spray_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2)
{
    _ttalink_request_spray_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, type, ctrl_state, data, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_spray_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t type, uint8_t ctrl_state, float data, float param1, float param2)
{
    _ttalink_request_spray_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, type, ctrl_state, data, param1, param2, true);
}
#endif

#endif

// MESSAGE REQUEST_SPRAY_STATUS UNPACKING


/**
 * @brief Get field update_time from request_spray_status message
 *
 * @return .
 */
static inline uint32_t ttalink_request_spray_status_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mode from request_spray_status message
 *
 * @return  0=NULL   1=查询动作  2=设置动作 3=查询设备选择 4=设置设备选择
 */
static inline uint8_t ttalink_request_spray_status_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field type from request_spray_status message
 *
 * @return   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器  5=播撒机 
 */
static inline uint8_t ttalink_request_spray_status_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field ctrl_state from request_spray_status message
 *
 * @return   0=手动关  1=手动开 2=自动开
 */
static inline uint8_t ttalink_request_spray_status_get_ctrl_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field data from request_spray_status message
 *
 * @return   控制数值. 
 */
static inline float ttalink_request_spray_status_get_data(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  7);
}

/**
 * @brief Get field param1 from request_spray_status message
 *
 * @return   预留. 
 */
static inline float ttalink_request_spray_status_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  11);
}

/**
 * @brief Get field param2 from request_spray_status message
 *
 * @return   预留. 
 */
static inline float ttalink_request_spray_status_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  15);
}

/**
 * @brief Decode a request_spray_status message into a struct
 *
 * @param msg The message to decode
 * @param request_spray_status C-struct to decode the message contents into
 */
static inline void ttalink_request_spray_status_decode(const ttalink_message_t* msg, ttalink_request_spray_status_t* request_spray_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_spray_status->update_time = ttalink_request_spray_status_get_update_time(msg);
    request_spray_status->mode = ttalink_request_spray_status_get_mode(msg);
    request_spray_status->type = ttalink_request_spray_status_get_type(msg);
    request_spray_status->ctrl_state = ttalink_request_spray_status_get_ctrl_state(msg);
    request_spray_status->data = ttalink_request_spray_status_get_data(msg);
    request_spray_status->param1 = ttalink_request_spray_status_get_param1(msg);
    request_spray_status->param2 = ttalink_request_spray_status_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN? msg->len : TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN;
        memset(request_spray_status, 0, TTALINK_MSG_ID_REQUEST_SPRAY_STATUS_LEN);
    memcpy(request_spray_status, _TTA_PAYLOAD(msg), len);
#endif
}
