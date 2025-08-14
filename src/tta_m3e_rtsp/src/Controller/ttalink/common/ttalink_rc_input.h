#pragma once
// MESSAGE RC_INPUT PACKING

#define TTALINK_MSG_ID_RC_INPUT 2229

TTAPACKED(
typedef struct __ttalink_rc_input_t {
 int64_t sn; /*<  .*/
 uint16_t rc_input[20]; /*< SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能*/
}) ttalink_rc_input_t;

#define TTALINK_MSG_ID_RC_INPUT_LEN 48
#define TTALINK_MSG_ID_RC_INPUT_MIN_LEN 48
#define TTALINK_MSG_ID_2229_LEN 48
#define TTALINK_MSG_ID_2229_MIN_LEN 48

#define TTALINK_MSG_ID_RC_INPUT_CRC 124
#define TTALINK_MSG_ID_2229_CRC 124

#define TTALINK_MSG_RC_INPUT_FIELD_RC_INPUT_LEN 20

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_RC_INPUT { \
    2229, \
    "RC_INPUT", \
    2, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_rc_input_t, sn) }, \
         { "rc_input", NULL, TTALINK_TYPE_UINT16_T, 20, 8, offsetof(ttalink_rc_input_t, rc_input) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_RC_INPUT { \
    "RC_INPUT", \
    2, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_rc_input_t, sn) }, \
         { "rc_input", NULL, TTALINK_TYPE_UINT16_T, 20, 8, offsetof(ttalink_rc_input_t, rc_input) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rc_input_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_INPUT_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint16_t_array(buf, 8, rc_input, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RC_INPUT_LEN);
#else
    ttalink_rc_input_t packet;
    packet.sn = sn;
    tta_array_memcpy(packet.rc_input, rc_input, sizeof(uint16_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RC_INPUT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RC_INPUT;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_RC_INPUT_MIN_LEN, TTALINK_MSG_ID_RC_INPUT_LEN, TTALINK_MSG_ID_RC_INPUT_CRC, nocrc);
}

/**
 * @brief Pack a rc_input message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn  .
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_input_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, const uint16_t *rc_input)
{
    return _ttalink_rc_input_pack(dst_addr, src_addr, msg,  sn, rc_input, false);
}

/**
 * @brief Pack a rc_input message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn  .
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_input_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, const uint16_t *rc_input)
{
    return _ttalink_rc_input_pack(dst_addr, src_addr, msg,  sn, rc_input, true);
}


static inline uint16_t _ttalink_rc_input_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_INPUT_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint16_t_array(buf, 8, rc_input, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RC_INPUT_LEN);
#else
    ttalink_rc_input_t packet;
    packet.sn = sn;
    tta_array_memcpy(packet.rc_input, rc_input, sizeof(uint16_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RC_INPUT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RC_INPUT;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_INPUT_MIN_LEN, TTALINK_MSG_ID_RC_INPUT_LEN, TTALINK_MSG_ID_RC_INPUT_CRC, nocrc);
}

/**
 * @brief Pack a rc_input message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn  .
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_input_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,const uint16_t *rc_input)
{
    return _ttalink_rc_input_pack_chan(dst_addr, src_addr, chan, msg,  sn, rc_input, false);
}

/**
 * @brief Pack a rc_input message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn  .
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_input_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,const uint16_t *rc_input)
{
    return _ttalink_rc_input_pack_chan(dst_addr, src_addr, chan, msg,  sn, rc_input, true);
}


static inline uint16_t _ttalink_rc_input_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_input_t* rc_input, bool nocrc)
{
    if(nocrc){
        return ttalink_rc_input_pack_nocrc(dst_addr, src_addr, msg, rc_input->sn, rc_input->rc_input);
    }else{
        return ttalink_rc_input_pack(dst_addr, src_addr, msg, rc_input->sn, rc_input->rc_input);
    }
    
}

/**
 * @brief Encode a rc_input struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rc_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_input_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_input_t* rc_input)
{
    return _ttalink_rc_input_encode(dst_addr, src_addr, msg, rc_input, false);
}

/**
 * @brief Encode a rc_input struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rc_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_input_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_input_t* rc_input)
{
    return _ttalink_rc_input_encode(dst_addr, src_addr, msg, rc_input, true);
}


static inline uint16_t _ttalink_rc_input_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_input_t* rc_input, bool nocrc)
{
    if(nocrc){
        return ttalink_rc_input_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rc_input->sn, rc_input->rc_input);
    }else{
        return ttalink_rc_input_pack_chan(dst_addr, src_addr, chan, msg, rc_input->sn, rc_input->rc_input);
    }
}

/**
 * @brief Encode a rc_input struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_input_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_input_t* rc_input)
{
    return _ttalink_rc_input_encode_chan(dst_addr, src_addr, chan, msg, rc_input, false);
}

/**
 * @brief Encode a rc_input struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_input_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_input_t* rc_input)
{
    return _ttalink_rc_input_encode_chan(dst_addr, src_addr, chan, msg, rc_input, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rc_input_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_INPUT_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint16_t_array(buf, 8, rc_input, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_INPUT, buf, TTALINK_MSG_ID_RC_INPUT_MIN_LEN, TTALINK_MSG_ID_RC_INPUT_LEN, TTALINK_MSG_ID_RC_INPUT_CRC, nocrc);
#else
    ttalink_rc_input_t packet;
    packet.sn = sn;
    tta_array_memcpy(packet.rc_input, rc_input, sizeof(uint16_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_INPUT, (const char *)&packet, TTALINK_MSG_ID_RC_INPUT_MIN_LEN, TTALINK_MSG_ID_RC_INPUT_LEN, TTALINK_MSG_ID_RC_INPUT_CRC, nocrc);
#endif
}

/**
 * @brief Send a rc_input message
 * @param chan TTAlink channel to send the message
 *
 * @param sn  .
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能
 */
static inline void ttalink_rc_input_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, const uint16_t *rc_input)
{
    _ttalink_rc_input_send(dst_addr, src_addr, chan, sn, rc_input, false);
}

/**
 * @brief Send a rc_input message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sn  .
 * @param rc_input SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能
 */
static inline void ttalink_rc_input_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, const uint16_t *rc_input)
{
    _ttalink_rc_input_send(dst_addr, src_addr, chan, sn, rc_input, true);
}


static inline void _ttalink_rc_input_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_input_t* rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rc_input_send_nocrc(dst_addr, src_addr, chan, rc_input->sn, rc_input->rc_input);
    }else{
        ttalink_rc_input_send(dst_addr, src_addr, chan, rc_input->sn, rc_input->rc_input);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_INPUT, (const char *)rc_input, TTALINK_MSG_ID_RC_INPUT_MIN_LEN, TTALINK_MSG_ID_RC_INPUT_LEN, TTALINK_MSG_ID_RC_INPUT_CRC, nocrc);
#endif
}

/**
 * @brief Send a rc_input message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rc_input_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_input_t* rc_input)
{
    _ttalink_rc_input_send_struct(dst_addr, src_addr, chan, rc_input, false);
}

/**
 * @brief Send a rc_input message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rc_input_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_input_t* rc_input)
{
    _ttalink_rc_input_send_struct(dst_addr, src_addr, chan, rc_input, true);
}

#if TTALINK_MSG_ID_RC_INPUT_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rc_input_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, const uint16_t *rc_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint16_t_array(buf, 8, rc_input, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_INPUT, buf, TTALINK_MSG_ID_RC_INPUT_MIN_LEN, TTALINK_MSG_ID_RC_INPUT_LEN, TTALINK_MSG_ID_RC_INPUT_CRC, nocrc);
#else
    ttalink_rc_input_t *packet = (ttalink_rc_input_t *)msgbuf;
    packet->sn = sn;
    tta_array_memcpy(packet->rc_input, rc_input, sizeof(uint16_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_INPUT, (const char *)packet, TTALINK_MSG_ID_RC_INPUT_MIN_LEN, TTALINK_MSG_ID_RC_INPUT_LEN, TTALINK_MSG_ID_RC_INPUT_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rc_input_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, const uint16_t *rc_input)
{
    _ttalink_rc_input_send_buf(dst_addr, src_addr, msgbuf, chan, sn, rc_input, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rc_input_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, const uint16_t *rc_input)
{
    _ttalink_rc_input_send_buf(dst_addr, src_addr, msgbuf, chan, sn, rc_input, true);
}
#endif

#endif

// MESSAGE RC_INPUT UNPACKING


/**
 * @brief Get field sn from rc_input message
 *
 * @return  .
 */
static inline int64_t ttalink_rc_input_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field rc_input from rc_input message
 *
 * @return SBUS协议支持的（0-15）16个2048分辨率的通道、（16-17）2个开关量的通道、（18）信号丢失标记 位域判断连接遥控器类型，与（19）激活故障保护功能
 */
static inline uint16_t ttalink_rc_input_get_rc_input(const ttalink_message_t* msg, uint16_t *rc_input)
{
    return _TTA_RETURN_uint16_t_array(msg, rc_input, 20,  8);
}

/**
 * @brief Decode a rc_input message into a struct
 *
 * @param msg The message to decode
 * @param rc_input C-struct to decode the message contents into
 */
static inline void ttalink_rc_input_decode(const ttalink_message_t* msg, ttalink_rc_input_t* rc_input)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rc_input->sn = ttalink_rc_input_get_sn(msg);
    ttalink_rc_input_get_rc_input(msg, rc_input->rc_input);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_RC_INPUT_LEN? msg->len : TTALINK_MSG_ID_RC_INPUT_LEN;
        memset(rc_input, 0, TTALINK_MSG_ID_RC_INPUT_LEN);
    memcpy(rc_input, _TTA_PAYLOAD(msg), len);
#endif
}
