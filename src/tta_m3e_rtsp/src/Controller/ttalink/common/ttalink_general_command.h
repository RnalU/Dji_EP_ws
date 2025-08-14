#pragma once
// MESSAGE GENERAL_COMMAND PACKING

#define TTALINK_MSG_ID_GENERAL_COMMAND 4097

TTAPACKED(
typedef struct __ttalink_general_command_t {
 int64_t update_time; /*<  update_time*/
 int64_t sn; /*< 设备序列号.*/
 uint8_t device_type; /*< 设备类型.*/
 uint16_t cmd; /*<  参考命令枚举*/
 uint8_t confirm; /*<  0:第一次发送此命令。1-255：超时重发。*/
 float param1; /*<  Parameter.*/
 float param2; /*<  Parameter.*/
 float param3; /*<  Parameter.*/
 float param4; /*<  Parameter.*/
 float param5; /*<  Parameter.*/
 float param6; /*<  Parameter.*/
 float param7; /*<  Parameter.*/
}) ttalink_general_command_t;

#define TTALINK_MSG_ID_GENERAL_COMMAND_LEN 48
#define TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN 48
#define TTALINK_MSG_ID_4097_LEN 48
#define TTALINK_MSG_ID_4097_MIN_LEN 48

#define TTALINK_MSG_ID_GENERAL_COMMAND_CRC 216
#define TTALINK_MSG_ID_4097_CRC 216



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_COMMAND { \
    4097, \
    "GENERAL_COMMAND", \
    12, \
    {  { "update_time", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_command_t, update_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 8, offsetof(ttalink_general_command_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_general_command_t, device_type) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT16_T, 0, 17, offsetof(ttalink_general_command_t, cmd) }, \
         { "confirm", NULL, TTALINK_TYPE_UINT8_T, 0, 19, offsetof(ttalink_general_command_t, confirm) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_general_command_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_general_command_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_general_command_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_general_command_t, param4) }, \
         { "param5", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_general_command_t, param5) }, \
         { "param6", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_general_command_t, param6) }, \
         { "param7", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_general_command_t, param7) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_COMMAND { \
    "GENERAL_COMMAND", \
    12, \
    {  { "update_time", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_command_t, update_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 8, offsetof(ttalink_general_command_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_general_command_t, device_type) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT16_T, 0, 17, offsetof(ttalink_general_command_t, cmd) }, \
         { "confirm", NULL, TTALINK_TYPE_UINT8_T, 0, 19, offsetof(ttalink_general_command_t, confirm) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_general_command_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_general_command_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_general_command_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_general_command_t, param4) }, \
         { "param5", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_general_command_t, param5) }, \
         { "param6", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_general_command_t, param6) }, \
         { "param7", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_general_command_t, param7) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_command_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_COMMAND_LEN];
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_float(buf, 20, param1);
    _tta_put_float(buf, 24, param2);
    _tta_put_float(buf, 28, param3);
    _tta_put_float(buf, 32, param4);
    _tta_put_float(buf, 36, param5);
    _tta_put_float(buf, 40, param6);
    _tta_put_float(buf, 44, param7);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_COMMAND_LEN);
#else
    ttalink_general_command_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.cmd = cmd;
    packet.confirm = confirm;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_COMMAND_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_COMMAND;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_CRC, nocrc);
}

/**
 * @brief Pack a general_command message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  0:第一次发送此命令。1-255：超时重发。
 * @param param1  Parameter.
 * @param param2  Parameter.
 * @param param3  Parameter.
 * @param param4  Parameter.
 * @param param5  Parameter.
 * @param param6  Parameter.
 * @param param7  Parameter.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    return _ttalink_general_command_pack(dst_addr, src_addr, msg,  update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, false);
}

/**
 * @brief Pack a general_command message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  0:第一次发送此命令。1-255：超时重发。
 * @param param1  Parameter.
 * @param param2  Parameter.
 * @param param3  Parameter.
 * @param param4  Parameter.
 * @param param5  Parameter.
 * @param param6  Parameter.
 * @param param7  Parameter.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    return _ttalink_general_command_pack(dst_addr, src_addr, msg,  update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, true);
}


static inline uint16_t _ttalink_general_command_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t update_time,int64_t sn,uint8_t device_type,uint16_t cmd,uint8_t confirm,float param1,float param2,float param3,float param4,float param5,float param6,float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_COMMAND_LEN];
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_float(buf, 20, param1);
    _tta_put_float(buf, 24, param2);
    _tta_put_float(buf, 28, param3);
    _tta_put_float(buf, 32, param4);
    _tta_put_float(buf, 36, param5);
    _tta_put_float(buf, 40, param6);
    _tta_put_float(buf, 44, param7);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_COMMAND_LEN);
#else
    ttalink_general_command_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.cmd = cmd;
    packet.confirm = confirm;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_COMMAND_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_COMMAND;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_CRC, nocrc);
}

/**
 * @brief Pack a general_command message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  0:第一次发送此命令。1-255：超时重发。
 * @param param1  Parameter.
 * @param param2  Parameter.
 * @param param3  Parameter.
 * @param param4  Parameter.
 * @param param5  Parameter.
 * @param param6  Parameter.
 * @param param7  Parameter.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t update_time,int64_t sn,uint8_t device_type,uint16_t cmd,uint8_t confirm,float param1,float param2,float param3,float param4,float param5,float param6,float param7)
{
    return _ttalink_general_command_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, false);
}

/**
 * @brief Pack a general_command message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  0:第一次发送此命令。1-255：超时重发。
 * @param param1  Parameter.
 * @param param2  Parameter.
 * @param param3  Parameter.
 * @param param4  Parameter.
 * @param param5  Parameter.
 * @param param6  Parameter.
 * @param param7  Parameter.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t update_time,int64_t sn,uint8_t device_type,uint16_t cmd,uint8_t confirm,float param1,float param2,float param3,float param4,float param5,float param6,float param7)
{
    return _ttalink_general_command_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, true);
}


static inline uint16_t _ttalink_general_command_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_command_t* general_command, bool nocrc)
{
    if(nocrc){
        return ttalink_general_command_pack_nocrc(dst_addr, src_addr, msg, general_command->update_time, general_command->sn, general_command->device_type, general_command->cmd, general_command->confirm, general_command->param1, general_command->param2, general_command->param3, general_command->param4, general_command->param5, general_command->param6, general_command->param7);
    }else{
        return ttalink_general_command_pack(dst_addr, src_addr, msg, general_command->update_time, general_command->sn, general_command->device_type, general_command->cmd, general_command->confirm, general_command->param1, general_command->param2, general_command->param3, general_command->param4, general_command->param5, general_command->param6, general_command->param7);
    }
    
}

/**
 * @brief Encode a general_command struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_command_t* general_command)
{
    return _ttalink_general_command_encode(dst_addr, src_addr, msg, general_command, false);
}

/**
 * @brief Encode a general_command struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_command_t* general_command)
{
    return _ttalink_general_command_encode(dst_addr, src_addr, msg, general_command, true);
}


static inline uint16_t _ttalink_general_command_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_command_t* general_command, bool nocrc)
{
    if(nocrc){
        return ttalink_general_command_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_command->update_time, general_command->sn, general_command->device_type, general_command->cmd, general_command->confirm, general_command->param1, general_command->param2, general_command->param3, general_command->param4, general_command->param5, general_command->param6, general_command->param7);
    }else{
        return ttalink_general_command_pack_chan(dst_addr, src_addr, chan, msg, general_command->update_time, general_command->sn, general_command->device_type, general_command->cmd, general_command->confirm, general_command->param1, general_command->param2, general_command->param3, general_command->param4, general_command->param5, general_command->param6, general_command->param7);
    }
}

/**
 * @brief Encode a general_command struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_command_t* general_command)
{
    return _ttalink_general_command_encode_chan(dst_addr, src_addr, chan, msg, general_command, false);
}

/**
 * @brief Encode a general_command struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_command_t* general_command)
{
    return _ttalink_general_command_encode_chan(dst_addr, src_addr, chan, msg, general_command, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_command_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_COMMAND_LEN];
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_float(buf, 20, param1);
    _tta_put_float(buf, 24, param2);
    _tta_put_float(buf, 28, param3);
    _tta_put_float(buf, 32, param4);
    _tta_put_float(buf, 36, param5);
    _tta_put_float(buf, 40, param6);
    _tta_put_float(buf, 44, param7);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND, buf, TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_CRC, nocrc);
#else
    ttalink_general_command_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.cmd = cmd;
    packet.confirm = confirm;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND, (const char *)&packet, TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_command message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  0:第一次发送此命令。1-255：超时重发。
 * @param param1  Parameter.
 * @param param2  Parameter.
 * @param param3  Parameter.
 * @param param4  Parameter.
 * @param param5  Parameter.
 * @param param6  Parameter.
 * @param param7  Parameter.
 */
static inline void ttalink_general_command_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_general_command_send(dst_addr, src_addr, chan, update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, false);
}

/**
 * @brief Send a general_command message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  0:第一次发送此命令。1-255：超时重发。
 * @param param1  Parameter.
 * @param param2  Parameter.
 * @param param3  Parameter.
 * @param param4  Parameter.
 * @param param5  Parameter.
 * @param param6  Parameter.
 * @param param7  Parameter.
 */
static inline void ttalink_general_command_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_general_command_send(dst_addr, src_addr, chan, update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, true);
}


static inline void _ttalink_general_command_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_command_t* general_command, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_command_send_nocrc(dst_addr, src_addr, chan, general_command->update_time, general_command->sn, general_command->device_type, general_command->cmd, general_command->confirm, general_command->param1, general_command->param2, general_command->param3, general_command->param4, general_command->param5, general_command->param6, general_command->param7);
    }else{
        ttalink_general_command_send(dst_addr, src_addr, chan, general_command->update_time, general_command->sn, general_command->device_type, general_command->cmd, general_command->confirm, general_command->param1, general_command->param2, general_command->param3, general_command->param4, general_command->param5, general_command->param6, general_command->param7);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND, (const char *)general_command, TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_command message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_command_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_command_t* general_command)
{
    _ttalink_general_command_send_struct(dst_addr, src_addr, chan, general_command, false);
}

/**
 * @brief Send a general_command message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_command_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_command_t* general_command)
{
    _ttalink_general_command_send_struct(dst_addr, src_addr, chan, general_command, true);
}

#if TTALINK_MSG_ID_GENERAL_COMMAND_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_command_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_float(buf, 20, param1);
    _tta_put_float(buf, 24, param2);
    _tta_put_float(buf, 28, param3);
    _tta_put_float(buf, 32, param4);
    _tta_put_float(buf, 36, param5);
    _tta_put_float(buf, 40, param6);
    _tta_put_float(buf, 44, param7);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND, buf, TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_CRC, nocrc);
#else
    ttalink_general_command_t *packet = (ttalink_general_command_t *)msgbuf;
    packet->update_time = update_time;
    packet->sn = sn;
    packet->device_type = device_type;
    packet->cmd = cmd;
    packet->confirm = confirm;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param6 = param6;
    packet->param7 = param7;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND, (const char *)packet, TTALINK_MSG_ID_GENERAL_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_command_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_general_command_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_command_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    _ttalink_general_command_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sn, device_type, cmd, confirm, param1, param2, param3, param4, param5, param6, param7, true);
}
#endif

#endif

// MESSAGE GENERAL_COMMAND UNPACKING


/**
 * @brief Get field update_time from general_command message
 *
 * @return  update_time
 */
static inline int64_t ttalink_general_command_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field sn from general_command message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_general_command_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  8);
}

/**
 * @brief Get field device_type from general_command message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_general_command_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field cmd from general_command message
 *
 * @return  参考命令枚举
 */
static inline uint16_t ttalink_general_command_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  17);
}

/**
 * @brief Get field confirm from general_command message
 *
 * @return  0:第一次发送此命令。1-255：超时重发。
 */
static inline uint8_t ttalink_general_command_get_confirm(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field param1 from general_command message
 *
 * @return  Parameter.
 */
static inline float ttalink_general_command_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field param2 from general_command message
 *
 * @return  Parameter.
 */
static inline float ttalink_general_command_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field param3 from general_command message
 *
 * @return  Parameter.
 */
static inline float ttalink_general_command_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field param4 from general_command message
 *
 * @return  Parameter.
 */
static inline float ttalink_general_command_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field param5 from general_command message
 *
 * @return  Parameter.
 */
static inline float ttalink_general_command_get_param5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field param6 from general_command message
 *
 * @return  Parameter.
 */
static inline float ttalink_general_command_get_param6(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field param7 from general_command message
 *
 * @return  Parameter.
 */
static inline float ttalink_general_command_get_param7(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Decode a general_command message into a struct
 *
 * @param msg The message to decode
 * @param general_command C-struct to decode the message contents into
 */
static inline void ttalink_general_command_decode(const ttalink_message_t* msg, ttalink_general_command_t* general_command)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_command->update_time = ttalink_general_command_get_update_time(msg);
    general_command->sn = ttalink_general_command_get_sn(msg);
    general_command->device_type = ttalink_general_command_get_device_type(msg);
    general_command->cmd = ttalink_general_command_get_cmd(msg);
    general_command->confirm = ttalink_general_command_get_confirm(msg);
    general_command->param1 = ttalink_general_command_get_param1(msg);
    general_command->param2 = ttalink_general_command_get_param2(msg);
    general_command->param3 = ttalink_general_command_get_param3(msg);
    general_command->param4 = ttalink_general_command_get_param4(msg);
    general_command->param5 = ttalink_general_command_get_param5(msg);
    general_command->param6 = ttalink_general_command_get_param6(msg);
    general_command->param7 = ttalink_general_command_get_param7(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_COMMAND_LEN? msg->len : TTALINK_MSG_ID_GENERAL_COMMAND_LEN;
        memset(general_command, 0, TTALINK_MSG_ID_GENERAL_COMMAND_LEN);
    memcpy(general_command, _TTA_PAYLOAD(msg), len);
#endif
}
