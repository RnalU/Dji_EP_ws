#pragma once
// MESSAGE SET_EX_PWM_ACK PACKING

#define TTALINK_MSG_ID_SET_EX_PWM_ACK 2237

TTAPACKED(
typedef struct __ttalink_set_ex_pwm_ack_t {
 uint32_t update_time; /*<  .*/
 uint8_t mode; /*<  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.*/
 uint8_t pwm_index; /*<   pwm的序号： 0：全部关闭  1~8：开启.*/
 uint8_t ack; /*<  非0为错误.*/
 float param1; /*< 预留.*/
}) ttalink_set_ex_pwm_ack_t;

#define TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN 11
#define TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN 11
#define TTALINK_MSG_ID_2237_LEN 11
#define TTALINK_MSG_ID_2237_MIN_LEN 11

#define TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC 46
#define TTALINK_MSG_ID_2237_CRC 46



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SET_EX_PWM_ACK { \
    2237, \
    "SET_EX_PWM_ACK", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_set_ex_pwm_ack_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_set_ex_pwm_ack_t, mode) }, \
         { "pwm_index", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_set_ex_pwm_ack_t, pwm_index) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_set_ex_pwm_ack_t, ack) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_set_ex_pwm_ack_t, param1) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SET_EX_PWM_ACK { \
    "SET_EX_PWM_ACK", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_set_ex_pwm_ack_t, update_time) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_set_ex_pwm_ack_t, mode) }, \
         { "pwm_index", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_set_ex_pwm_ack_t, pwm_index) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_set_ex_pwm_ack_t, ack) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_set_ex_pwm_ack_t, param1) }, \
         } \
}
#endif


static inline uint16_t _ttalink_set_ex_pwm_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, pwm_index);
    _tta_put_uint8_t(buf, 6, ack);
    _tta_put_float(buf, 7, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN);
#else
    ttalink_set_ex_pwm_ack_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.pwm_index = pwm_index;
    packet.ack = ack;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_EX_PWM_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC, nocrc);
}

/**
 * @brief Pack a set_ex_pwm_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param mode  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.
 * @param pwm_index   pwm的序号： 0：全部关闭  1~8：开启.
 * @param ack  非0为错误.
 * @param param1 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_ex_pwm_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1)
{
    return _ttalink_set_ex_pwm_ack_pack(dst_addr, src_addr, msg,  update_time, mode, pwm_index, ack, param1, false);
}

/**
 * @brief Pack a set_ex_pwm_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param mode  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.
 * @param pwm_index   pwm的序号： 0：全部关闭  1~8：开启.
 * @param ack  非0为错误.
 * @param param1 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_ex_pwm_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1)
{
    return _ttalink_set_ex_pwm_ack_pack(dst_addr, src_addr, msg,  update_time, mode, pwm_index, ack, param1, true);
}


static inline uint16_t _ttalink_set_ex_pwm_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t pwm_index,uint8_t ack,float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, pwm_index);
    _tta_put_uint8_t(buf, 6, ack);
    _tta_put_float(buf, 7, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN);
#else
    ttalink_set_ex_pwm_ack_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.pwm_index = pwm_index;
    packet.ack = ack;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_EX_PWM_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC, nocrc);
}

/**
 * @brief Pack a set_ex_pwm_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param mode  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.
 * @param pwm_index   pwm的序号： 0：全部关闭  1~8：开启.
 * @param ack  非0为错误.
 * @param param1 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_ex_pwm_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t pwm_index,uint8_t ack,float param1)
{
    return _ttalink_set_ex_pwm_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, pwm_index, ack, param1, false);
}

/**
 * @brief Pack a set_ex_pwm_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param mode  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.
 * @param pwm_index   pwm的序号： 0：全部关闭  1~8：开启.
 * @param ack  非0为错误.
 * @param param1 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_ex_pwm_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t mode,uint8_t pwm_index,uint8_t ack,float param1)
{
    return _ttalink_set_ex_pwm_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, mode, pwm_index, ack, param1, true);
}


static inline uint16_t _ttalink_set_ex_pwm_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_set_ex_pwm_ack_pack_nocrc(dst_addr, src_addr, msg, set_ex_pwm_ack->update_time, set_ex_pwm_ack->mode, set_ex_pwm_ack->pwm_index, set_ex_pwm_ack->ack, set_ex_pwm_ack->param1);
    }else{
        return ttalink_set_ex_pwm_ack_pack(dst_addr, src_addr, msg, set_ex_pwm_ack->update_time, set_ex_pwm_ack->mode, set_ex_pwm_ack->pwm_index, set_ex_pwm_ack->ack, set_ex_pwm_ack->param1);
    }
    
}

/**
 * @brief Encode a set_ex_pwm_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_ex_pwm_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_ex_pwm_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack)
{
    return _ttalink_set_ex_pwm_ack_encode(dst_addr, src_addr, msg, set_ex_pwm_ack, false);
}

/**
 * @brief Encode a set_ex_pwm_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_ex_pwm_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_ex_pwm_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack)
{
    return _ttalink_set_ex_pwm_ack_encode(dst_addr, src_addr, msg, set_ex_pwm_ack, true);
}


static inline uint16_t _ttalink_set_ex_pwm_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_set_ex_pwm_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, set_ex_pwm_ack->update_time, set_ex_pwm_ack->mode, set_ex_pwm_ack->pwm_index, set_ex_pwm_ack->ack, set_ex_pwm_ack->param1);
    }else{
        return ttalink_set_ex_pwm_ack_pack_chan(dst_addr, src_addr, chan, msg, set_ex_pwm_ack->update_time, set_ex_pwm_ack->mode, set_ex_pwm_ack->pwm_index, set_ex_pwm_ack->ack, set_ex_pwm_ack->param1);
    }
}

/**
 * @brief Encode a set_ex_pwm_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_ex_pwm_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_ex_pwm_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack)
{
    return _ttalink_set_ex_pwm_ack_encode_chan(dst_addr, src_addr, chan, msg, set_ex_pwm_ack, false);
}

/**
 * @brief Encode a set_ex_pwm_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_ex_pwm_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_ex_pwm_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack)
{
    return _ttalink_set_ex_pwm_ack_encode_chan(dst_addr, src_addr, chan, msg, set_ex_pwm_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_set_ex_pwm_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, pwm_index);
    _tta_put_uint8_t(buf, 6, ack);
    _tta_put_float(buf, 7, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_EX_PWM_ACK, buf, TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC, nocrc);
#else
    ttalink_set_ex_pwm_ack_t packet;
    packet.update_time = update_time;
    packet.mode = mode;
    packet.pwm_index = pwm_index;
    packet.ack = ack;
    packet.param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_EX_PWM_ACK, (const char *)&packet, TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_ex_pwm_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param mode  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.
 * @param pwm_index   pwm的序号： 0：全部关闭  1~8：开启.
 * @param ack  非0为错误.
 * @param param1 预留.
 */
static inline void ttalink_set_ex_pwm_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1)
{
    _ttalink_set_ex_pwm_ack_send(dst_addr, src_addr, chan, update_time, mode, pwm_index, ack, param1, false);
}

/**
 * @brief Send a set_ex_pwm_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param mode  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.
 * @param pwm_index   pwm的序号： 0：全部关闭  1~8：开启.
 * @param ack  非0为错误.
 * @param param1 预留.
 */
static inline void ttalink_set_ex_pwm_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1)
{
    _ttalink_set_ex_pwm_ack_send(dst_addr, src_addr, chan, update_time, mode, pwm_index, ack, param1, true);
}


static inline void _ttalink_set_ex_pwm_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_set_ex_pwm_ack_send_nocrc(dst_addr, src_addr, chan, set_ex_pwm_ack->update_time, set_ex_pwm_ack->mode, set_ex_pwm_ack->pwm_index, set_ex_pwm_ack->ack, set_ex_pwm_ack->param1);
    }else{
        ttalink_set_ex_pwm_ack_send(dst_addr, src_addr, chan, set_ex_pwm_ack->update_time, set_ex_pwm_ack->mode, set_ex_pwm_ack->pwm_index, set_ex_pwm_ack->ack, set_ex_pwm_ack->param1);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_EX_PWM_ACK, (const char *)set_ex_pwm_ack, TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_ex_pwm_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_ex_pwm_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack)
{
    _ttalink_set_ex_pwm_ack_send_struct(dst_addr, src_addr, chan, set_ex_pwm_ack, false);
}

/**
 * @brief Send a set_ex_pwm_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_ex_pwm_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack)
{
    _ttalink_set_ex_pwm_ack_send_struct(dst_addr, src_addr, chan, set_ex_pwm_ack, true);
}

#if TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_set_ex_pwm_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, mode);
    _tta_put_uint8_t(buf, 5, pwm_index);
    _tta_put_uint8_t(buf, 6, ack);
    _tta_put_float(buf, 7, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_EX_PWM_ACK, buf, TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC, nocrc);
#else
    ttalink_set_ex_pwm_ack_t *packet = (ttalink_set_ex_pwm_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->mode = mode;
    packet->pwm_index = pwm_index;
    packet->ack = ack;
    packet->param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_EX_PWM_ACK, (const char *)packet, TTALINK_MSG_ID_SET_EX_PWM_ACK_MIN_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN, TTALINK_MSG_ID_SET_EX_PWM_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_set_ex_pwm_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1)
{
    _ttalink_set_ex_pwm_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, pwm_index, ack, param1, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_set_ex_pwm_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t mode, uint8_t pwm_index, uint8_t ack, float param1)
{
    _ttalink_set_ex_pwm_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, mode, pwm_index, ack, param1, true);
}
#endif

#endif

// MESSAGE SET_EX_PWM_ACK UNPACKING


/**
 * @brief Get field update_time from set_ex_pwm_ack message
 *
 * @return  .
 */
static inline uint32_t ttalink_set_ex_pwm_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mode from set_ex_pwm_ack message
 *
 * @return  默认为0；0=单次转动 （转动1秒）  1=持续转动  2=正弦单次方式（1000~value）3=正弦持续方式（1000~value）.
 */
static inline uint8_t ttalink_set_ex_pwm_ack_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field pwm_index from set_ex_pwm_ack message
 *
 * @return   pwm的序号： 0：全部关闭  1~8：开启.
 */
static inline uint8_t ttalink_set_ex_pwm_ack_get_pwm_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field ack from set_ex_pwm_ack message
 *
 * @return  非0为错误.
 */
static inline uint8_t ttalink_set_ex_pwm_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field param1 from set_ex_pwm_ack message
 *
 * @return 预留.
 */
static inline float ttalink_set_ex_pwm_ack_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  7);
}

/**
 * @brief Decode a set_ex_pwm_ack message into a struct
 *
 * @param msg The message to decode
 * @param set_ex_pwm_ack C-struct to decode the message contents into
 */
static inline void ttalink_set_ex_pwm_ack_decode(const ttalink_message_t* msg, ttalink_set_ex_pwm_ack_t* set_ex_pwm_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    set_ex_pwm_ack->update_time = ttalink_set_ex_pwm_ack_get_update_time(msg);
    set_ex_pwm_ack->mode = ttalink_set_ex_pwm_ack_get_mode(msg);
    set_ex_pwm_ack->pwm_index = ttalink_set_ex_pwm_ack_get_pwm_index(msg);
    set_ex_pwm_ack->ack = ttalink_set_ex_pwm_ack_get_ack(msg);
    set_ex_pwm_ack->param1 = ttalink_set_ex_pwm_ack_get_param1(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN? msg->len : TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN;
        memset(set_ex_pwm_ack, 0, TTALINK_MSG_ID_SET_EX_PWM_ACK_LEN);
    memcpy(set_ex_pwm_ack, _TTA_PAYLOAD(msg), len);
#endif
}
