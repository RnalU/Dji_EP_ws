#pragma once
// MESSAGE MULTI_THROW_HOOK PACKING

#define TTALINK_MSG_ID_MULTI_THROW_HOOK 2270

TTAPACKED(
typedef struct __ttalink_multi_throw_hook_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t device_type; /*<  设备组 默认为 1,后续增加 */
 uint8_t time_type; /*< 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟*/
 uint32_t time_delay; /*<  间隔时间,单位 MS */
 uint32_t bit_device_action; /*< 位域表示 动作设备 */
 float param[5]; /*< 预留*/
}) ttalink_multi_throw_hook_t;

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN 34
#define TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN 34
#define TTALINK_MSG_ID_2270_LEN 34
#define TTALINK_MSG_ID_2270_MIN_LEN 34

#define TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC 28
#define TTALINK_MSG_ID_2270_CRC 28

#define TTALINK_MSG_MULTI_THROW_HOOK_FIELD_PARAM_LEN 5

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MULTI_THROW_HOOK { \
    2270, \
    "MULTI_THROW_HOOK", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_multi_throw_hook_t, update_time) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_multi_throw_hook_t, device_type) }, \
         { "time_type", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_multi_throw_hook_t, time_type) }, \
         { "time_delay", NULL, TTALINK_TYPE_UINT32_T, 0, 6, offsetof(ttalink_multi_throw_hook_t, time_delay) }, \
         { "bit_device_action", NULL, TTALINK_TYPE_UINT32_T, 0, 10, offsetof(ttalink_multi_throw_hook_t, bit_device_action) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 5, 14, offsetof(ttalink_multi_throw_hook_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MULTI_THROW_HOOK { \
    "MULTI_THROW_HOOK", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_multi_throw_hook_t, update_time) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_multi_throw_hook_t, device_type) }, \
         { "time_type", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_multi_throw_hook_t, time_type) }, \
         { "time_delay", NULL, TTALINK_TYPE_UINT32_T, 0, 6, offsetof(ttalink_multi_throw_hook_t, time_delay) }, \
         { "bit_device_action", NULL, TTALINK_TYPE_UINT32_T, 0, 10, offsetof(ttalink_multi_throw_hook_t, bit_device_action) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 5, 14, offsetof(ttalink_multi_throw_hook_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_multi_throw_hook_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint8_t(buf, 5, time_type);
    _tta_put_uint32_t(buf, 6, time_delay);
    _tta_put_uint32_t(buf, 10, bit_device_action);
    _tta_put_float_array(buf, 14, param, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN);
#else
    ttalink_multi_throw_hook_t packet;
    packet.update_time = update_time;
    packet.device_type = device_type;
    packet.time_type = time_type;
    packet.time_delay = time_delay;
    packet.bit_device_action = bit_device_action;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MULTI_THROW_HOOK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC, nocrc);
}

/**
 * @brief Pack a multi_throw_hook message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1,后续增加 
 * @param time_type 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟
 * @param time_delay  间隔时间,单位 MS 
 * @param bit_device_action 位域表示 动作设备 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param)
{
    return _ttalink_multi_throw_hook_pack(dst_addr, src_addr, msg,  update_time, device_type, time_type, time_delay, bit_device_action, param, false);
}

/**
 * @brief Pack a multi_throw_hook message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1,后续增加 
 * @param time_type 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟
 * @param time_delay  间隔时间,单位 MS 
 * @param bit_device_action 位域表示 动作设备 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param)
{
    return _ttalink_multi_throw_hook_pack(dst_addr, src_addr, msg,  update_time, device_type, time_type, time_delay, bit_device_action, param, true);
}


static inline uint16_t _ttalink_multi_throw_hook_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t device_type,uint8_t time_type,uint32_t time_delay,uint32_t bit_device_action,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint8_t(buf, 5, time_type);
    _tta_put_uint32_t(buf, 6, time_delay);
    _tta_put_uint32_t(buf, 10, bit_device_action);
    _tta_put_float_array(buf, 14, param, 5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN);
#else
    ttalink_multi_throw_hook_t packet;
    packet.update_time = update_time;
    packet.device_type = device_type;
    packet.time_type = time_type;
    packet.time_delay = time_delay;
    packet.bit_device_action = bit_device_action;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MULTI_THROW_HOOK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC, nocrc);
}

/**
 * @brief Pack a multi_throw_hook message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1,后续增加 
 * @param time_type 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟
 * @param time_delay  间隔时间,单位 MS 
 * @param bit_device_action 位域表示 动作设备 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t device_type,uint8_t time_type,uint32_t time_delay,uint32_t bit_device_action,const float *param)
{
    return _ttalink_multi_throw_hook_pack_chan(dst_addr, src_addr, chan, msg,  update_time, device_type, time_type, time_delay, bit_device_action, param, false);
}

/**
 * @brief Pack a multi_throw_hook message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1,后续增加 
 * @param time_type 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟
 * @param time_delay  间隔时间,单位 MS 
 * @param bit_device_action 位域表示 动作设备 
 * @param param 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_multi_throw_hook_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t device_type,uint8_t time_type,uint32_t time_delay,uint32_t bit_device_action,const float *param)
{
    return _ttalink_multi_throw_hook_pack_chan(dst_addr, src_addr, chan, msg,  update_time, device_type, time_type, time_delay, bit_device_action, param, true);
}


static inline uint16_t _ttalink_multi_throw_hook_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_t* multi_throw_hook, bool nocrc)
{
    if(nocrc){
        return ttalink_multi_throw_hook_pack_nocrc(dst_addr, src_addr, msg, multi_throw_hook->update_time, multi_throw_hook->device_type, multi_throw_hook->time_type, multi_throw_hook->time_delay, multi_throw_hook->bit_device_action, multi_throw_hook->param);
    }else{
        return ttalink_multi_throw_hook_pack(dst_addr, src_addr, msg, multi_throw_hook->update_time, multi_throw_hook->device_type, multi_throw_hook->time_type, multi_throw_hook->time_delay, multi_throw_hook->bit_device_action, multi_throw_hook->param);
    }
    
}

/**
 * @brief Encode a multi_throw_hook struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_t* multi_throw_hook)
{
    return _ttalink_multi_throw_hook_encode(dst_addr, src_addr, msg, multi_throw_hook, false);
}

/**
 * @brief Encode a multi_throw_hook struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_multi_throw_hook_t* multi_throw_hook)
{
    return _ttalink_multi_throw_hook_encode(dst_addr, src_addr, msg, multi_throw_hook, true);
}


static inline uint16_t _ttalink_multi_throw_hook_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_t* multi_throw_hook, bool nocrc)
{
    if(nocrc){
        return ttalink_multi_throw_hook_pack_chan_nocrc(dst_addr, src_addr, chan, msg, multi_throw_hook->update_time, multi_throw_hook->device_type, multi_throw_hook->time_type, multi_throw_hook->time_delay, multi_throw_hook->bit_device_action, multi_throw_hook->param);
    }else{
        return ttalink_multi_throw_hook_pack_chan(dst_addr, src_addr, chan, msg, multi_throw_hook->update_time, multi_throw_hook->device_type, multi_throw_hook->time_type, multi_throw_hook->time_delay, multi_throw_hook->bit_device_action, multi_throw_hook->param);
    }
}

/**
 * @brief Encode a multi_throw_hook struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_t* multi_throw_hook)
{
    return _ttalink_multi_throw_hook_encode_chan(dst_addr, src_addr, chan, msg, multi_throw_hook, false);
}

/**
 * @brief Encode a multi_throw_hook struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param multi_throw_hook C-struct to read the message contents from
 */
static inline uint16_t ttalink_multi_throw_hook_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_multi_throw_hook_t* multi_throw_hook)
{
    return _ttalink_multi_throw_hook_encode_chan(dst_addr, src_addr, chan, msg, multi_throw_hook, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_multi_throw_hook_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint8_t(buf, 5, time_type);
    _tta_put_uint32_t(buf, 6, time_delay);
    _tta_put_uint32_t(buf, 10, bit_device_action);
    _tta_put_float_array(buf, 14, param, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK, buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC, nocrc);
#else
    ttalink_multi_throw_hook_t packet;
    packet.update_time = update_time;
    packet.device_type = device_type;
    packet.time_type = time_type;
    packet.time_delay = time_delay;
    packet.bit_device_action = bit_device_action;
    tta_array_memcpy(packet.param, param, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK, (const char *)&packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC, nocrc);
#endif
}

/**
 * @brief Send a multi_throw_hook message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1,后续增加 
 * @param time_type 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟
 * @param time_delay  间隔时间,单位 MS 
 * @param bit_device_action 位域表示 动作设备 
 * @param param 预留
 */
static inline void ttalink_multi_throw_hook_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param)
{
    _ttalink_multi_throw_hook_send(dst_addr, src_addr, chan, update_time, device_type, time_type, time_delay, bit_device_action, param, false);
}

/**
 * @brief Send a multi_throw_hook message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param device_type  设备组 默认为 1,后续增加 
 * @param time_type 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟
 * @param time_delay  间隔时间,单位 MS 
 * @param bit_device_action 位域表示 动作设备 
 * @param param 预留
 */
static inline void ttalink_multi_throw_hook_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param)
{
    _ttalink_multi_throw_hook_send(dst_addr, src_addr, chan, update_time, device_type, time_type, time_delay, bit_device_action, param, true);
}


static inline void _ttalink_multi_throw_hook_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_t* multi_throw_hook, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_multi_throw_hook_send_nocrc(dst_addr, src_addr, chan, multi_throw_hook->update_time, multi_throw_hook->device_type, multi_throw_hook->time_type, multi_throw_hook->time_delay, multi_throw_hook->bit_device_action, multi_throw_hook->param);
    }else{
        ttalink_multi_throw_hook_send(dst_addr, src_addr, chan, multi_throw_hook->update_time, multi_throw_hook->device_type, multi_throw_hook->time_type, multi_throw_hook->time_delay, multi_throw_hook->bit_device_action, multi_throw_hook->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK, (const char *)multi_throw_hook, TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC, nocrc);
#endif
}

/**
 * @brief Send a multi_throw_hook message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_multi_throw_hook_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_t* multi_throw_hook)
{
    _ttalink_multi_throw_hook_send_struct(dst_addr, src_addr, chan, multi_throw_hook, false);
}

/**
 * @brief Send a multi_throw_hook message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_multi_throw_hook_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_multi_throw_hook_t* multi_throw_hook)
{
    _ttalink_multi_throw_hook_send_struct(dst_addr, src_addr, chan, multi_throw_hook, true);
}

#if TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_multi_throw_hook_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, device_type);
    _tta_put_uint8_t(buf, 5, time_type);
    _tta_put_uint32_t(buf, 6, time_delay);
    _tta_put_uint32_t(buf, 10, bit_device_action);
    _tta_put_float_array(buf, 14, param, 5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK, buf, TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC, nocrc);
#else
    ttalink_multi_throw_hook_t *packet = (ttalink_multi_throw_hook_t *)msgbuf;
    packet->update_time = update_time;
    packet->device_type = device_type;
    packet->time_type = time_type;
    packet->time_delay = time_delay;
    packet->bit_device_action = bit_device_action;
    tta_array_memcpy(packet->param, param, sizeof(float)*5);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MULTI_THROW_HOOK, (const char *)packet, TTALINK_MSG_ID_MULTI_THROW_HOOK_MIN_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN, TTALINK_MSG_ID_MULTI_THROW_HOOK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_multi_throw_hook_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param)
{
    _ttalink_multi_throw_hook_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, device_type, time_type, time_delay, bit_device_action, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_multi_throw_hook_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t device_type, uint8_t time_type, uint32_t time_delay, uint32_t bit_device_action, const float *param)
{
    _ttalink_multi_throw_hook_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, device_type, time_type, time_delay, bit_device_action, param, true);
}
#endif

#endif

// MESSAGE MULTI_THROW_HOOK UNPACKING


/**
 * @brief Get field update_time from multi_throw_hook message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_multi_throw_hook_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field device_type from multi_throw_hook message
 *
 * @return  设备组 默认为 1,后续增加 
 */
static inline uint8_t ttalink_multi_throw_hook_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field time_type from multi_throw_hook message
 *
 * @return 延时时间的标志，0=先延迟后抛放 1=先抛放后延迟
 */
static inline uint8_t ttalink_multi_throw_hook_get_time_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field time_delay from multi_throw_hook message
 *
 * @return  间隔时间,单位 MS 
 */
static inline uint32_t ttalink_multi_throw_hook_get_time_delay(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  6);
}

/**
 * @brief Get field bit_device_action from multi_throw_hook message
 *
 * @return 位域表示 动作设备 
 */
static inline uint32_t ttalink_multi_throw_hook_get_bit_device_action(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  10);
}

/**
 * @brief Get field param from multi_throw_hook message
 *
 * @return 预留
 */
static inline uint16_t ttalink_multi_throw_hook_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 5,  14);
}

/**
 * @brief Decode a multi_throw_hook message into a struct
 *
 * @param msg The message to decode
 * @param multi_throw_hook C-struct to decode the message contents into
 */
static inline void ttalink_multi_throw_hook_decode(const ttalink_message_t* msg, ttalink_multi_throw_hook_t* multi_throw_hook)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    multi_throw_hook->update_time = ttalink_multi_throw_hook_get_update_time(msg);
    multi_throw_hook->device_type = ttalink_multi_throw_hook_get_device_type(msg);
    multi_throw_hook->time_type = ttalink_multi_throw_hook_get_time_type(msg);
    multi_throw_hook->time_delay = ttalink_multi_throw_hook_get_time_delay(msg);
    multi_throw_hook->bit_device_action = ttalink_multi_throw_hook_get_bit_device_action(msg);
    ttalink_multi_throw_hook_get_param(msg, multi_throw_hook->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN? msg->len : TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN;
        memset(multi_throw_hook, 0, TTALINK_MSG_ID_MULTI_THROW_HOOK_LEN);
    memcpy(multi_throw_hook, _TTA_PAYLOAD(msg), len);
#endif
}
