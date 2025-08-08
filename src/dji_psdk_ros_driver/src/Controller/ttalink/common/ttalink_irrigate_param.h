#pragma once
// MESSAGE IRRIGATE_PARAM PACKING

#define TTALINK_MSG_ID_IRRIGATE_PARAM 2164

TTAPACKED(
typedef struct __ttalink_irrigate_param_t {
 uint32_t update_time; /*< .*/
 uint8_t cmd; /*<   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程*/
 float tar_cap; /*<   容量. */
 float tar_flow_speed; /*<   流速. */
 float param1; /*<   . */
 float param2; /*<   . */
}) ttalink_irrigate_param_t;

#define TTALINK_MSG_ID_IRRIGATE_PARAM_LEN 21
#define TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN 21
#define TTALINK_MSG_ID_2164_LEN 21
#define TTALINK_MSG_ID_2164_MIN_LEN 21

#define TTALINK_MSG_ID_IRRIGATE_PARAM_CRC 83
#define TTALINK_MSG_ID_2164_CRC 83



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_IRRIGATE_PARAM { \
    2164, \
    "IRRIGATE_PARAM", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_irrigate_param_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_irrigate_param_t, cmd) }, \
         { "tar_cap", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_irrigate_param_t, tar_cap) }, \
         { "tar_flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_irrigate_param_t, tar_flow_speed) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_irrigate_param_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_irrigate_param_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_IRRIGATE_PARAM { \
    "IRRIGATE_PARAM", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_irrigate_param_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_irrigate_param_t, cmd) }, \
         { "tar_cap", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_irrigate_param_t, tar_cap) }, \
         { "tar_flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_irrigate_param_t, tar_flow_speed) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_irrigate_param_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_irrigate_param_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_irrigate_param_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_PARAM_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, tar_cap);
    _tta_put_float(buf, 9, tar_flow_speed);
    _tta_put_float(buf, 13, param1);
    _tta_put_float(buf, 17, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN);
#else
    ttalink_irrigate_param_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.tar_cap = tar_cap;
    packet.tar_flow_speed = tar_flow_speed;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IRRIGATE_PARAM;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_CRC, nocrc);
}

/**
 * @brief Pack a irrigate_param message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param cmd   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程
 * @param tar_cap   容量. 
 * @param tar_flow_speed   流速. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_param_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2)
{
    return _ttalink_irrigate_param_pack(dst_addr, src_addr, msg,  update_time, cmd, tar_cap, tar_flow_speed, param1, param2, false);
}

/**
 * @brief Pack a irrigate_param message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param cmd   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程
 * @param tar_cap   容量. 
 * @param tar_flow_speed   流速. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_param_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2)
{
    return _ttalink_irrigate_param_pack(dst_addr, src_addr, msg,  update_time, cmd, tar_cap, tar_flow_speed, param1, param2, true);
}


static inline uint16_t _ttalink_irrigate_param_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,float tar_cap,float tar_flow_speed,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_PARAM_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, tar_cap);
    _tta_put_float(buf, 9, tar_flow_speed);
    _tta_put_float(buf, 13, param1);
    _tta_put_float(buf, 17, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN);
#else
    ttalink_irrigate_param_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.tar_cap = tar_cap;
    packet.tar_flow_speed = tar_flow_speed;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IRRIGATE_PARAM;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_CRC, nocrc);
}

/**
 * @brief Pack a irrigate_param message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param cmd   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程
 * @param tar_cap   容量. 
 * @param tar_flow_speed   流速. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_param_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,float tar_cap,float tar_flow_speed,float param1,float param2)
{
    return _ttalink_irrigate_param_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, tar_cap, tar_flow_speed, param1, param2, false);
}

/**
 * @brief Pack a irrigate_param message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param cmd   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程
 * @param tar_cap   容量. 
 * @param tar_flow_speed   流速. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_param_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,float tar_cap,float tar_flow_speed,float param1,float param2)
{
    return _ttalink_irrigate_param_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, tar_cap, tar_flow_speed, param1, param2, true);
}


static inline uint16_t _ttalink_irrigate_param_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_param_t* irrigate_param, bool nocrc)
{
    if(nocrc){
        return ttalink_irrigate_param_pack_nocrc(dst_addr, src_addr, msg, irrigate_param->update_time, irrigate_param->cmd, irrigate_param->tar_cap, irrigate_param->tar_flow_speed, irrigate_param->param1, irrigate_param->param2);
    }else{
        return ttalink_irrigate_param_pack(dst_addr, src_addr, msg, irrigate_param->update_time, irrigate_param->cmd, irrigate_param->tar_cap, irrigate_param->tar_flow_speed, irrigate_param->param1, irrigate_param->param2);
    }
    
}

/**
 * @brief Encode a irrigate_param struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_param_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_param_t* irrigate_param)
{
    return _ttalink_irrigate_param_encode(dst_addr, src_addr, msg, irrigate_param, false);
}

/**
 * @brief Encode a irrigate_param struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_param_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_param_t* irrigate_param)
{
    return _ttalink_irrigate_param_encode(dst_addr, src_addr, msg, irrigate_param, true);
}


static inline uint16_t _ttalink_irrigate_param_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_param_t* irrigate_param, bool nocrc)
{
    if(nocrc){
        return ttalink_irrigate_param_pack_chan_nocrc(dst_addr, src_addr, chan, msg, irrigate_param->update_time, irrigate_param->cmd, irrigate_param->tar_cap, irrigate_param->tar_flow_speed, irrigate_param->param1, irrigate_param->param2);
    }else{
        return ttalink_irrigate_param_pack_chan(dst_addr, src_addr, chan, msg, irrigate_param->update_time, irrigate_param->cmd, irrigate_param->tar_cap, irrigate_param->tar_flow_speed, irrigate_param->param1, irrigate_param->param2);
    }
}

/**
 * @brief Encode a irrigate_param struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_param_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_param_t* irrigate_param)
{
    return _ttalink_irrigate_param_encode_chan(dst_addr, src_addr, chan, msg, irrigate_param, false);
}

/**
 * @brief Encode a irrigate_param struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_param C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_param_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_param_t* irrigate_param)
{
    return _ttalink_irrigate_param_encode_chan(dst_addr, src_addr, chan, msg, irrigate_param, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_irrigate_param_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_PARAM_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, tar_cap);
    _tta_put_float(buf, 9, tar_flow_speed);
    _tta_put_float(buf, 13, param1);
    _tta_put_float(buf, 17, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_PARAM, buf, TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_CRC, nocrc);
#else
    ttalink_irrigate_param_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.tar_cap = tar_cap;
    packet.tar_flow_speed = tar_flow_speed;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_PARAM, (const char *)&packet, TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_CRC, nocrc);
#endif
}

/**
 * @brief Send a irrigate_param message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param cmd   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程
 * @param tar_cap   容量. 
 * @param tar_flow_speed   流速. 
 * @param param1   . 
 * @param param2   . 
 */
static inline void ttalink_irrigate_param_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2)
{
    _ttalink_irrigate_param_send(dst_addr, src_addr, chan, update_time, cmd, tar_cap, tar_flow_speed, param1, param2, false);
}

/**
 * @brief Send a irrigate_param message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param cmd   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程
 * @param tar_cap   容量. 
 * @param tar_flow_speed   流速. 
 * @param param1   . 
 * @param param2   . 
 */
static inline void ttalink_irrigate_param_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2)
{
    _ttalink_irrigate_param_send(dst_addr, src_addr, chan, update_time, cmd, tar_cap, tar_flow_speed, param1, param2, true);
}


static inline void _ttalink_irrigate_param_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_param_t* irrigate_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_irrigate_param_send_nocrc(dst_addr, src_addr, chan, irrigate_param->update_time, irrigate_param->cmd, irrigate_param->tar_cap, irrigate_param->tar_flow_speed, irrigate_param->param1, irrigate_param->param2);
    }else{
        ttalink_irrigate_param_send(dst_addr, src_addr, chan, irrigate_param->update_time, irrigate_param->cmd, irrigate_param->tar_cap, irrigate_param->tar_flow_speed, irrigate_param->param1, irrigate_param->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_PARAM, (const char *)irrigate_param, TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_CRC, nocrc);
#endif
}

/**
 * @brief Send a irrigate_param message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_irrigate_param_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_param_t* irrigate_param)
{
    _ttalink_irrigate_param_send_struct(dst_addr, src_addr, chan, irrigate_param, false);
}

/**
 * @brief Send a irrigate_param message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_irrigate_param_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_param_t* irrigate_param)
{
    _ttalink_irrigate_param_send_struct(dst_addr, src_addr, chan, irrigate_param, true);
}

#if TTALINK_MSG_ID_IRRIGATE_PARAM_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_irrigate_param_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_float(buf, 5, tar_cap);
    _tta_put_float(buf, 9, tar_flow_speed);
    _tta_put_float(buf, 13, param1);
    _tta_put_float(buf, 17, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_PARAM, buf, TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_CRC, nocrc);
#else
    ttalink_irrigate_param_t *packet = (ttalink_irrigate_param_t *)msgbuf;
    packet->update_time = update_time;
    packet->cmd = cmd;
    packet->tar_cap = tar_cap;
    packet->tar_flow_speed = tar_flow_speed;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_PARAM, (const char *)packet, TTALINK_MSG_ID_IRRIGATE_PARAM_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN, TTALINK_MSG_ID_IRRIGATE_PARAM_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_irrigate_param_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2)
{
    _ttalink_irrigate_param_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, tar_cap, tar_flow_speed, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_irrigate_param_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, float tar_cap, float tar_flow_speed, float param1, float param2)
{
    _ttalink_irrigate_param_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, tar_cap, tar_flow_speed, param1, param2, true);
}
#endif

#endif

// MESSAGE IRRIGATE_PARAM UNPACKING


/**
 * @brief Get field update_time from irrigate_param message
 *
 * @return .
 */
static inline uint32_t ttalink_irrigate_param_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd from irrigate_param message
 *
 * @return   0=NULL  1=设置标定因数 2 查询标定因数 3=开启喷洒标定 4=关闭喷洒标定  5=喷洒标定过程
 */
static inline uint8_t ttalink_irrigate_param_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field tar_cap from irrigate_param message
 *
 * @return   容量. 
 */
static inline float ttalink_irrigate_param_get_tar_cap(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Get field tar_flow_speed from irrigate_param message
 *
 * @return   流速. 
 */
static inline float ttalink_irrigate_param_get_tar_flow_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field param1 from irrigate_param message
 *
 * @return   . 
 */
static inline float ttalink_irrigate_param_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field param2 from irrigate_param message
 *
 * @return   . 
 */
static inline float ttalink_irrigate_param_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Decode a irrigate_param message into a struct
 *
 * @param msg The message to decode
 * @param irrigate_param C-struct to decode the message contents into
 */
static inline void ttalink_irrigate_param_decode(const ttalink_message_t* msg, ttalink_irrigate_param_t* irrigate_param)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    irrigate_param->update_time = ttalink_irrigate_param_get_update_time(msg);
    irrigate_param->cmd = ttalink_irrigate_param_get_cmd(msg);
    irrigate_param->tar_cap = ttalink_irrigate_param_get_tar_cap(msg);
    irrigate_param->tar_flow_speed = ttalink_irrigate_param_get_tar_flow_speed(msg);
    irrigate_param->param1 = ttalink_irrigate_param_get_param1(msg);
    irrigate_param->param2 = ttalink_irrigate_param_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_IRRIGATE_PARAM_LEN? msg->len : TTALINK_MSG_ID_IRRIGATE_PARAM_LEN;
        memset(irrigate_param, 0, TTALINK_MSG_ID_IRRIGATE_PARAM_LEN);
    memcpy(irrigate_param, _TTA_PAYLOAD(msg), len);
#endif
}
