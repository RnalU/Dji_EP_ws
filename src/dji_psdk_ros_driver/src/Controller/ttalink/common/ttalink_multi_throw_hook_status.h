#pragma once
// MESSAGE MULTI_THROW_HOOK_STATUS PACKING

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS 2272

TTAPACKED(
typedef struct __ttalink_multi_throw_hook_status_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t device_type; /*<  设备组 默认为 1 */
 uint32_t bit_check_state; /*< 位域表示 设备是否安装 */
 uint32_t bit_action_state; /*< 位域表示 设备是否动作 */
 float param[5]; /*< 预留*/
}) ttalink_multi_throw_hook_status_t;

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN 33
#define TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN 33
#define TTALINK_MSG_ID_2272_LEN 33
#define TTALINK_MSG_ID_2272_MIN_LEN 33

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC 123
#define TTALINK_MSG_ID_2272_CRC 123

#define TTALINK_MSG_MULTI_THROW_HOOK_STATUS_FIELD_PARAM_LEN 5

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MULTI_THROW_HOOK_STATUS { \
    2272, \
    "MULTI_THROW_HOOK_STATUS", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_multi_throw_hook_status_t, update_time) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_multi_throw_hook_status_t, device_type) }, \
         { "bit_check_state", NULL, TTALINK_TYPE_UINT32_T, 0, 5, offsetof(ttalink_multi_throw_hook_status_t, bit_check_state) }, \
         { "bit_action_state", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_multi_throw_hook_status_t, bit_action_state) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 5, 13, offsetof(ttalink_multi_throw_hook_status_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MULTI_THROW_HOOK_STATUS { \
    "MULTI_THROW_HOOK_STATUS", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_multi_throw_hook_status_t, update_time) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_multi_throw_hook_status_t, device_type) }, \
         { "bit_check_state", NULL, TTALINK_TYPE_UINT32_T, 0, 5, offsetof(ttalink_multi_throw_hook_status_t, bit_check_state) }, \
         { "bit_action_state", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_multi_throw_hook_status_t, bit_action_state) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 5, 13, offsetof(ttalink_multi_throw_hook_status_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_multi_throw_hook_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint32_t(buf, 5, bit_check_state);
    _tta_put_uint32_t(buf, 9, bit_action_state);
    _tta_put_float_array(buf, 13, param, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN);
#else
    ttalink_multi_throw_hook_status_t packet;
    packet.update_time = update_time;
    packet.device_type = device_type;
    packet.bit_check_state = bit_check_state;
    packet.bit_action_state = bit_action_state;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a multi_throw_hook_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1 
 * @param bit_check_state 位域表示 设备是否安装 
 * @param bit_action_state 位域表示 设备是否动作 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param)
{
    return _ttalink_multi_throw_hook_status_pack(dst_addr, src_addr, msg,  update_time, device_type, bit_check_state, bit_action_state, param, false);
}

/**
 * @brief Pack a multi_throw_hook_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1 
 * @param bit_check_state 位域表示 设备是否安装 
 * @param bit_action_state 位域表示 设备是否动作 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param)
{
    return _ttalink_multi_throw_hook_status_pack(dst_addr, src_addr, msg,  update_time, device_type, bit_check_state, bit_action_state, param, true);
}


static inline uint16_t _ttalink_multi_throw_hook_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t device_type,uint32_t bit_check_state,uint32_t bit_action_state,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint32_t(buf, 5, bit_check_state);
    _tta_put_uint32_t(buf, 9, bit_action_state);
    _tta_put_float_array(buf, 13, param, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN);
#else
    ttalink_multi_throw_hook_status_t packet;
    packet.update_time = update_time;
    packet.device_type = device_type;
    packet.bit_check_state = bit_check_state;
    packet.bit_action_state = bit_action_state;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a multi_throw_hook_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1 
 * @param bit_check_state 位域表示 设备是否安装 
 * @param bit_action_state 位域表示 设备是否动作 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t device_type,uint32_t bit_check_state,uint32_t bit_action_state,const float *param)
{
    return _ttalink_multi_throw_hook_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, device_type, bit_check_state, bit_action_state, param, false);
}

/**
 * @brief Pack a multi_throw_hook_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1 
 * @param bit_check_state 位域表示 设备是否安装 
 * @param bit_action_state 位域表示 设备是否动作 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t device_type,uint32_t bit_check_state,uint32_t bit_action_state,const float *param)
{
    return _ttalink_multi_throw_hook_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, device_type, bit_check_state, bit_action_state, param, true);
}


static inline uint16_t _ttalink_multi_throw_hook_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status, bool nocrc)
{
    if(nocrc){
        return ttalink_multi_throw_hook_status_pack_nocrc(dst_addr, src_addr, msg, multi_throw_hook_status->update_time, multi_throw_hook_status->device_type, multi_throw_hook_status->bit_check_state, multi_throw_hook_status->bit_action_state, multi_throw_hook_status->param);
    }else{
        return ttalink_multi_throw_hook_status_pack(dst_addr, src_addr, msg, multi_throw_hook_status->update_time, multi_throw_hook_status->device_type, multi_throw_hook_status->bit_check_state, multi_throw_hook_status->bit_action_state, multi_throw_hook_status->param);
    }
    
}

/**
 * @brief Encode a multi_throw_hook_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status)
{
    return _ttalink_multi_throw_hook_status_encode(dst_addr, src_addr, msg, multi_throw_hook_status, false);
}

/**
 * @brief Encode a multi_throw_hook_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status)
{
    return _ttalink_multi_throw_hook_status_encode(dst_addr, src_addr, msg, multi_throw_hook_status, true);
}


static inline uint16_t _ttalink_multi_throw_hook_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status, bool nocrc)
{
    if(nocrc){
        return ttalink_multi_throw_hook_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, multi_throw_hook_status->update_time, multi_throw_hook_status->device_type, multi_throw_hook_status->bit_check_state, multi_throw_hook_status->bit_action_state, multi_throw_hook_status->param);
    }else{
        return ttalink_multi_throw_hook_status_pack_chan(dst_addr, src_addr, chan, msg, multi_throw_hook_status->update_time, multi_throw_hook_status->device_type, multi_throw_hook_status->bit_check_state, multi_throw_hook_status->bit_action_state, multi_throw_hook_status->param);
    }
}

/**
 * @brief Encode a multi_throw_hook_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status)
{
    return _ttalink_multi_throw_hook_status_encode_chan(dst_addr, src_addr, chan, msg, multi_throw_hook_status, false);
}

/**
 * @brief Encode a multi_throw_hook_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status)
{
    return _ttalink_multi_throw_hook_status_encode_chan(dst_addr, src_addr, chan, msg, multi_throw_hook_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_multi_throw_hook_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint32_t(buf, 5, bit_check_state);
    _tta_put_uint32_t(buf, 9, bit_action_state);
    _tta_put_float_array(buf, 13, param, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS, buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC, nocrc);
#else
    ttalink_multi_throw_hook_status_t packet;
    packet.update_time = update_time;
    packet.device_type = device_type;
    packet.bit_check_state = bit_check_state;
    packet.bit_action_state = bit_action_state;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS, (const char *)&packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a multi_throw_hook_status message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1 
 * @param bit_check_state 位域表示 设备是否安装 
 * @param bit_action_state 位域表示 设备是否动作 
 * @param param 预留
 */
static inline void ttalink_multi_throw_hook_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param)
{
    _ttalink_multi_throw_hook_status_send(dst_addr, src_addr, chan, update_time, device_type, bit_check_state, bit_action_state, param, false);
}

/**
 * @brief Send a multi_throw_hook_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1 
 * @param bit_check_state 位域表示 设备是否安装 
 * @param bit_action_state 位域表示 设备是否动作 
 * @param param 预留
 */
static inline void ttalink_multi_throw_hook_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param)
{
    _ttalink_multi_throw_hook_status_send(dst_addr, src_addr, chan, update_time, device_type, bit_check_state, bit_action_state, param, true);
}


static inline void _ttalink_multi_throw_hook_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_multi_throw_hook_status_send_nocrc(dst_addr, src_addr, chan, multi_throw_hook_status->update_time, multi_throw_hook_status->device_type, multi_throw_hook_status->bit_check_state, multi_throw_hook_status->bit_action_state, multi_throw_hook_status->param);
    }else{
        ttalink_multi_throw_hook_status_send(dst_addr, src_addr, chan, multi_throw_hook_status->update_time, multi_throw_hook_status->device_type, multi_throw_hook_status->bit_check_state, multi_throw_hook_status->bit_action_state, multi_throw_hook_status->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS, (const char *)multi_throw_hook_status, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a multi_throw_hook_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_multi_throw_hook_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status)
{
    _ttalink_multi_throw_hook_status_send_struct(dst_addr, src_addr, chan, multi_throw_hook_status, false);
}

/**
 * @brief Send a multi_throw_hook_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_multi_throw_hook_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_status_t* multi_throw_hook_status)
{
    _ttalink_multi_throw_hook_status_send_struct(dst_addr, src_addr, chan, multi_throw_hook_status, true);
}

#if TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_multi_throw_hook_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint32_t(buf, 5, bit_check_state);
    _tta_put_uint32_t(buf, 9, bit_action_state);
    _tta_put_float_array(buf, 13, param, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS, buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC, nocrc);
#else
    ttalink_multi_throw_hook_status_t *packet = (ttalink_multi_throw_hook_status_t *)msgbuf;
    packet->update_time = update_time;
    packet->device_type = device_type;
    packet->bit_check_state = bit_check_state;
    packet->bit_action_state = bit_action_state;
    tta_array_memcpy(packet->param, param, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS, (const char *)packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_multi_throw_hook_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param)
{
    _ttalink_multi_throw_hook_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, device_type, bit_check_state, bit_action_state, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_multi_throw_hook_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t device_type, uint32_t bit_check_state, uint32_t bit_action_state, const float *param)
{
    _ttalink_multi_throw_hook_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, device_type, bit_check_state, bit_action_state, param, true);
}
#endif

#endif

// MESSAGE MULTI_THROW_HOOK_STATUS UNPACKING


/**
 * @brief Get field update_time from multi_throw_hook_status message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_multi_throw_hook_status_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field device_type from multi_throw_hook_status message
 *
 * @return  设备组 默认为 1 
 */
static inline uint8_t ttalink_multi_throw_hook_status_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field bit_check_state from multi_throw_hook_status message
 *
 * @return 位域表示 设备是否安装 
 */
static inline uint32_t ttalink_multi_throw_hook_status_get_bit_check_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  5);
}

/**
 * @brief Get field bit_action_state from multi_throw_hook_status message
 *
 * @return 位域表示 设备是否动作 
 */
static inline uint32_t ttalink_multi_throw_hook_status_get_bit_action_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Get field param from multi_throw_hook_status message
 *
 * @return 预留
 */
static inline uint16_t ttalink_multi_throw_hook_status_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 5,  13);
}

/**
 * @brief Decode a multi_throw_hook_status message into a struct
 *
 * @param msg The message to decode
 * @param multi_throw_hook_status C-struct to decode the message contents into
 */
static inline void ttalink_multi_throw_hook_status_decode(const ttalink_message_t* msg, ttalink_multi_throw_hook_status_t* multi_throw_hook_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    multi_throw_hook_status->update_time = ttalink_multi_throw_hook_status_get_update_time(msg);
    multi_throw_hook_status->device_type = ttalink_multi_throw_hook_status_get_device_type(msg);
    multi_throw_hook_status->bit_check_state = ttalink_multi_throw_hook_status_get_bit_check_state(msg);
    multi_throw_hook_status->bit_action_state = ttalink_multi_throw_hook_status_get_bit_action_state(msg);
    ttalink_multi_throw_hook_status_get_param(msg, multi_throw_hook_status->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN? msg->len : TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN;
        memset(multi_throw_hook_status, 0, TTALINK_MSG_ID_MULTI_THROW_HOOK_STATUS_LEN);
    memcpy(multi_throw_hook_status, _TTA_PAYLOAD(msg), len);
#endif
}
