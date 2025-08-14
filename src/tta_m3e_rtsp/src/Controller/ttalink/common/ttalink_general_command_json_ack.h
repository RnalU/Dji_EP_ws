#pragma once
// MESSAGE GENERAL_COMMAND_JSON_ACK PACKING

#define TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK 4113

TTAPACKED(
typedef struct __ttalink_general_command_json_ack_t {
 
    /* 
      update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间） 
     */
    int64_t update_time;
 
    /* 
     设备序列号. 
     */
    int64_t sn;
 
    /* 
     设备类型. 
     */
    uint8_t device_type;
 
    /* 
      参考命令枚举 
     */
    uint16_t cmd;
 
    /* 
      表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复. 
     */
    uint8_t confirm;
 
    /* 
      参考回复枚举. 
     */
    uint8_t result;
 
    /* 
      . 
     */
    int8_t payload[480];
}) ttalink_general_command_json_ack_t;

#define TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN 501
#define TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN 501
#define TTALINK_MSG_ID_4113_LEN 501
#define TTALINK_MSG_ID_4113_MIN_LEN 501

#define TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC 251
#define TTALINK_MSG_ID_4113_CRC 251

#define TTALINK_MSG_GENERAL_COMMAND_JSON_ACK_FIELD_PAYLOAD_LEN 480

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_COMMAND_JSON_ACK { \
    4113, \
    "GENERAL_COMMAND_JSON_ACK", \
    7, \
    {  { "update_time", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_command_json_ack_t, update_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 8, offsetof(ttalink_general_command_json_ack_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_general_command_json_ack_t, device_type) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT16_T, 0, 17, offsetof(ttalink_general_command_json_ack_t, cmd) }, \
         { "confirm", NULL, TTALINK_TYPE_UINT8_T, 0, 19, offsetof(ttalink_general_command_json_ack_t, confirm) }, \
         { "result", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_general_command_json_ack_t, result) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 480, 21, offsetof(ttalink_general_command_json_ack_t, payload) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_COMMAND_JSON_ACK { \
    "GENERAL_COMMAND_JSON_ACK", \
    7, \
    {  { "update_time", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_command_json_ack_t, update_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 8, offsetof(ttalink_general_command_json_ack_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_general_command_json_ack_t, device_type) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT16_T, 0, 17, offsetof(ttalink_general_command_json_ack_t, cmd) }, \
         { "confirm", NULL, TTALINK_TYPE_UINT8_T, 0, 19, offsetof(ttalink_general_command_json_ack_t, confirm) }, \
         { "result", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_general_command_json_ack_t, result) }, \
         { "payload", NULL, TTALINK_TYPE_INT8_T, 480, 21, offsetof(ttalink_general_command_json_ack_t, payload) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_command_json_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN];
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_uint8_t(buf, 20, result);
    _tta_put_int8_t_array(buf, 21, payload, 480);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN);
#else
    ttalink_general_command_json_ack_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.cmd = cmd;
    packet.confirm = confirm;
    packet.result = result;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*480);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC, nocrc);
}

/**
 * @brief Pack a general_command_json_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间）
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复.
 * @param result  参考回复枚举.
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_json_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload)
{
    return _ttalink_general_command_json_ack_pack(dst_addr, src_addr, msg,  update_time, sn, device_type, cmd, confirm, result, payload, false);
}

/**
 * @brief Pack a general_command_json_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间）
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复.
 * @param result  参考回复枚举.
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_json_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload)
{
    return _ttalink_general_command_json_ack_pack(dst_addr, src_addr, msg,  update_time, sn, device_type, cmd, confirm, result, payload, true);
}


static inline uint16_t _ttalink_general_command_json_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t update_time,int64_t sn,uint8_t device_type,uint16_t cmd,uint8_t confirm,uint8_t result,const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN];
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_uint8_t(buf, 20, result);
    _tta_put_int8_t_array(buf, 21, payload, 480);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN);
#else
    ttalink_general_command_json_ack_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.cmd = cmd;
    packet.confirm = confirm;
    packet.result = result;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*480);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC, nocrc);
}

/**
 * @brief Pack a general_command_json_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间）
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复.
 * @param result  参考回复枚举.
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_json_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t update_time,int64_t sn,uint8_t device_type,uint16_t cmd,uint8_t confirm,uint8_t result,const int8_t *payload)
{
    return _ttalink_general_command_json_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sn, device_type, cmd, confirm, result, payload, false);
}

/**
 * @brief Pack a general_command_json_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间）
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复.
 * @param result  参考回复枚举.
 * @param payload  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_command_json_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t update_time,int64_t sn,uint8_t device_type,uint16_t cmd,uint8_t confirm,uint8_t result,const int8_t *payload)
{
    return _ttalink_general_command_json_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, sn, device_type, cmd, confirm, result, payload, true);
}


static inline uint16_t _ttalink_general_command_json_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_command_json_ack_t* general_command_json_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_general_command_json_ack_pack_nocrc(dst_addr, src_addr, msg, general_command_json_ack->update_time, general_command_json_ack->sn, general_command_json_ack->device_type, general_command_json_ack->cmd, general_command_json_ack->confirm, general_command_json_ack->result, general_command_json_ack->payload);
    }else{
        return ttalink_general_command_json_ack_pack(dst_addr, src_addr, msg, general_command_json_ack->update_time, general_command_json_ack->sn, general_command_json_ack->device_type, general_command_json_ack->cmd, general_command_json_ack->confirm, general_command_json_ack->result, general_command_json_ack->payload);
    }
    
}

/**
 * @brief Encode a general_command_json_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_command_json_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_json_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_command_json_ack_t* general_command_json_ack)
{
    return _ttalink_general_command_json_ack_encode(dst_addr, src_addr, msg, general_command_json_ack, false);
}

/**
 * @brief Encode a general_command_json_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_command_json_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_json_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_command_json_ack_t* general_command_json_ack)
{
    return _ttalink_general_command_json_ack_encode(dst_addr, src_addr, msg, general_command_json_ack, true);
}


static inline uint16_t _ttalink_general_command_json_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_command_json_ack_t* general_command_json_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_general_command_json_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_command_json_ack->update_time, general_command_json_ack->sn, general_command_json_ack->device_type, general_command_json_ack->cmd, general_command_json_ack->confirm, general_command_json_ack->result, general_command_json_ack->payload);
    }else{
        return ttalink_general_command_json_ack_pack_chan(dst_addr, src_addr, chan, msg, general_command_json_ack->update_time, general_command_json_ack->sn, general_command_json_ack->device_type, general_command_json_ack->cmd, general_command_json_ack->confirm, general_command_json_ack->result, general_command_json_ack->payload);
    }
}

/**
 * @brief Encode a general_command_json_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_command_json_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_json_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_command_json_ack_t* general_command_json_ack)
{
    return _ttalink_general_command_json_ack_encode_chan(dst_addr, src_addr, chan, msg, general_command_json_ack, false);
}

/**
 * @brief Encode a general_command_json_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_command_json_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_command_json_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_command_json_ack_t* general_command_json_ack)
{
    return _ttalink_general_command_json_ack_encode_chan(dst_addr, src_addr, chan, msg, general_command_json_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_command_json_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN];
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_uint8_t(buf, 20, result);
    _tta_put_int8_t_array(buf, 21, payload, 480);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK, buf, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC, nocrc);
#else
    ttalink_general_command_json_ack_t packet;
    packet.update_time = update_time;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.cmd = cmd;
    packet.confirm = confirm;
    packet.result = result;
    tta_array_memcpy(packet.payload, payload, sizeof(int8_t)*480);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK, (const char *)&packet, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_command_json_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间）
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复.
 * @param result  参考回复枚举.
 * @param payload  .
 */
static inline void ttalink_general_command_json_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload)
{
    _ttalink_general_command_json_ack_send(dst_addr, src_addr, chan, update_time, sn, device_type, cmd, confirm, result, payload, false);
}

/**
 * @brief Send a general_command_json_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间）
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param cmd  参考命令枚举
 * @param confirm  表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复.
 * @param result  参考回复枚举.
 * @param payload  .
 */
static inline void ttalink_general_command_json_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload)
{
    _ttalink_general_command_json_ack_send(dst_addr, src_addr, chan, update_time, sn, device_type, cmd, confirm, result, payload, true);
}


static inline void _ttalink_general_command_json_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_command_json_ack_t* general_command_json_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_command_json_ack_send_nocrc(dst_addr, src_addr, chan, general_command_json_ack->update_time, general_command_json_ack->sn, general_command_json_ack->device_type, general_command_json_ack->cmd, general_command_json_ack->confirm, general_command_json_ack->result, general_command_json_ack->payload);
    }else{
        ttalink_general_command_json_ack_send(dst_addr, src_addr, chan, general_command_json_ack->update_time, general_command_json_ack->sn, general_command_json_ack->device_type, general_command_json_ack->cmd, general_command_json_ack->confirm, general_command_json_ack->result, general_command_json_ack->payload);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK, (const char *)general_command_json_ack, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_command_json_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_command_json_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_command_json_ack_t* general_command_json_ack)
{
    _ttalink_general_command_json_ack_send_struct(dst_addr, src_addr, chan, general_command_json_ack, false);
}

/**
 * @brief Send a general_command_json_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_command_json_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_command_json_ack_t* general_command_json_ack)
{
    _ttalink_general_command_json_ack_send_struct(dst_addr, src_addr, chan, general_command_json_ack, true);
}

#if TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_command_json_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, update_time);
    _tta_put_int64_t(buf, 8, sn);
    _tta_put_uint8_t(buf, 16, device_type);
    _tta_put_uint16_t(buf, 17, cmd);
    _tta_put_uint8_t(buf, 19, confirm);
    _tta_put_uint8_t(buf, 20, result);
    _tta_put_int8_t_array(buf, 21, payload, 480);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK, buf, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC, nocrc);
#else
    ttalink_general_command_json_ack_t *packet = (ttalink_general_command_json_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->sn = sn;
    packet->device_type = device_type;
    packet->cmd = cmd;
    packet->confirm = confirm;
    packet->result = result;
    tta_array_memcpy(packet->payload, payload, sizeof(int8_t)*480);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK, (const char *)packet, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_MIN_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_command_json_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload)
{
    _ttalink_general_command_json_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sn, device_type, cmd, confirm, result, payload, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_command_json_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t update_time, int64_t sn, uint8_t device_type, uint16_t cmd, uint8_t confirm, uint8_t result, const int8_t *payload)
{
    _ttalink_general_command_json_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, sn, device_type, cmd, confirm, result, payload, true);
}
#endif

#endif

// MESSAGE GENERAL_COMMAND_JSON_ACK UNPACKING


/**
 * @brief Get field update_time from general_command_json_ack message
 *
 * @return  update_time,根据时间判定是否为同一条协议（超时重发时间不变，规定初始协议时间）
 */
static inline int64_t ttalink_general_command_json_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field sn from general_command_json_ack message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_general_command_json_ack_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  8);
}

/**
 * @brief Get field device_type from general_command_json_ack message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_general_command_json_ack_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field cmd from general_command_json_ack message
 *
 * @return  参考命令枚举
 */
static inline uint16_t ttalink_general_command_json_ack_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  17);
}

/**
 * @brief Get field confirm from general_command_json_ack message
 *
 * @return  表明回复的是第几条命令，收到不为0时，采用0那时刻的ACK回复.
 */
static inline uint8_t ttalink_general_command_json_ack_get_confirm(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field result from general_command_json_ack message
 *
 * @return  参考回复枚举.
 */
static inline uint8_t ttalink_general_command_json_ack_get_result(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field payload from general_command_json_ack message
 *
 * @return  .
 */
static inline uint16_t ttalink_general_command_json_ack_get_payload(const ttalink_message_t* msg, int8_t *payload)
{
    return _TTA_RETURN_int8_t_array(msg, payload, 480,  21);
}

/**
 * @brief Decode a general_command_json_ack message into a struct
 *
 * @param msg The message to decode
 * @param general_command_json_ack C-struct to decode the message contents into
 */
static inline void ttalink_general_command_json_ack_decode(const ttalink_message_t* msg, ttalink_general_command_json_ack_t* general_command_json_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_command_json_ack->update_time = ttalink_general_command_json_ack_get_update_time(msg);
    general_command_json_ack->sn = ttalink_general_command_json_ack_get_sn(msg);
    general_command_json_ack->device_type = ttalink_general_command_json_ack_get_device_type(msg);
    general_command_json_ack->cmd = ttalink_general_command_json_ack_get_cmd(msg);
    general_command_json_ack->confirm = ttalink_general_command_json_ack_get_confirm(msg);
    general_command_json_ack->result = ttalink_general_command_json_ack_get_result(msg);
    ttalink_general_command_json_ack_get_payload(msg, general_command_json_ack->payload);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN? msg->len : TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN;
        memset(general_command_json_ack, 0, TTALINK_MSG_ID_GENERAL_COMMAND_JSON_ACK_LEN);
    memcpy(general_command_json_ack, _TTA_PAYLOAD(msg), len);
#endif
}
