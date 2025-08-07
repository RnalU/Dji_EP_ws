#pragma once
// MESSAGE FOC_ARM_STATE_ACK PACKING

#define TTALINK_MSG_ID_FOC_ARM_STATE_ACK 2017

TTAPACKED(
typedef struct __ttalink_foc_arm_state_ack_t {
 uint32_t update_time; /*< system time*/
 uint32_t ack; /*< FOC锁死状态  0=disarm:正常转动   1=arm:锁死*/
 uint32_t data; /*< 预留*/
}) ttalink_foc_arm_state_ack_t;

#define TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN 12
#define TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN 12
#define TTALINK_MSG_ID_2017_LEN 12
#define TTALINK_MSG_ID_2017_MIN_LEN 12

#define TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC 10
#define TTALINK_MSG_ID_2017_CRC 10



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_ARM_STATE_ACK { \
    2017, \
    "FOC_ARM_STATE_ACK", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_arm_state_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_foc_arm_state_ack_t, ack) }, \
         { "data", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_foc_arm_state_ack_t, data) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_ARM_STATE_ACK { \
    "FOC_ARM_STATE_ACK", \
    3, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_arm_state_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_foc_arm_state_ack_t, ack) }, \
         { "data", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_foc_arm_state_ack_t, data) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_arm_state_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ack, uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ack);
    _tta_put_uint32_t(buf, 8, data);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN);
#else
    ttalink_foc_arm_state_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.data = data;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_ARM_STATE_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a foc_arm_state_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param ack FOC锁死状态  0=disarm:正常转动   1=arm:锁死
 * @param data 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_arm_state_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ack, uint32_t data)
{
    return _ttalink_foc_arm_state_ack_pack(dst_addr, src_addr, msg,  update_time, ack, data, false);
}

/**
 * @brief Pack a foc_arm_state_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param ack FOC锁死状态  0=disarm:正常转动   1=arm:锁死
 * @param data 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_arm_state_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ack, uint32_t data)
{
    return _ttalink_foc_arm_state_ack_pack(dst_addr, src_addr, msg,  update_time, ack, data, true);
}


static inline uint16_t _ttalink_foc_arm_state_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ack,uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ack);
    _tta_put_uint32_t(buf, 8, data);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN);
#else
    ttalink_foc_arm_state_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.data = data;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_ARM_STATE_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a foc_arm_state_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param ack FOC锁死状态  0=disarm:正常转动   1=arm:锁死
 * @param data 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_arm_state_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ack,uint32_t data)
{
    return _ttalink_foc_arm_state_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, data, false);
}

/**
 * @brief Pack a foc_arm_state_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param ack FOC锁死状态  0=disarm:正常转动   1=arm:锁死
 * @param data 预留
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_arm_state_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ack,uint32_t data)
{
    return _ttalink_foc_arm_state_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, data, true);
}


static inline uint16_t _ttalink_foc_arm_state_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_arm_state_ack_pack_nocrc(dst_addr, src_addr, msg, foc_arm_state_ack->update_time, foc_arm_state_ack->ack, foc_arm_state_ack->data);
    }else{
        return ttalink_foc_arm_state_ack_pack(dst_addr, src_addr, msg, foc_arm_state_ack->update_time, foc_arm_state_ack->ack, foc_arm_state_ack->data);
    }
    
}

/**
 * @brief Encode a foc_arm_state_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_arm_state_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_arm_state_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack)
{
    return _ttalink_foc_arm_state_ack_encode(dst_addr, src_addr, msg, foc_arm_state_ack, false);
}

/**
 * @brief Encode a foc_arm_state_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_arm_state_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_arm_state_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack)
{
    return _ttalink_foc_arm_state_ack_encode(dst_addr, src_addr, msg, foc_arm_state_ack, true);
}


static inline uint16_t _ttalink_foc_arm_state_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_arm_state_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_arm_state_ack->update_time, foc_arm_state_ack->ack, foc_arm_state_ack->data);
    }else{
        return ttalink_foc_arm_state_ack_pack_chan(dst_addr, src_addr, chan, msg, foc_arm_state_ack->update_time, foc_arm_state_ack->ack, foc_arm_state_ack->data);
    }
}

/**
 * @brief Encode a foc_arm_state_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_arm_state_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_arm_state_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack)
{
    return _ttalink_foc_arm_state_ack_encode_chan(dst_addr, src_addr, chan, msg, foc_arm_state_ack, false);
}

/**
 * @brief Encode a foc_arm_state_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_arm_state_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_arm_state_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack)
{
    return _ttalink_foc_arm_state_ack_encode_chan(dst_addr, src_addr, chan, msg, foc_arm_state_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_arm_state_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ack, uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ack);
    _tta_put_uint32_t(buf, 8, data);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_ARM_STATE_ACK, buf, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC, nocrc);
#else
    ttalink_foc_arm_state_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.data = data;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_ARM_STATE_ACK, (const char *)&packet, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_arm_state_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param ack FOC锁死状态  0=disarm:正常转动   1=arm:锁死
 * @param data 预留
 */
static inline void ttalink_foc_arm_state_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ack, uint32_t data)
{
    _ttalink_foc_arm_state_ack_send(dst_addr, src_addr, chan, update_time, ack, data, false);
}

/**
 * @brief Send a foc_arm_state_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param ack FOC锁死状态  0=disarm:正常转动   1=arm:锁死
 * @param data 预留
 */
static inline void ttalink_foc_arm_state_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ack, uint32_t data)
{
    _ttalink_foc_arm_state_ack_send(dst_addr, src_addr, chan, update_time, ack, data, true);
}


static inline void _ttalink_foc_arm_state_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_arm_state_ack_send_nocrc(dst_addr, src_addr, chan, foc_arm_state_ack->update_time, foc_arm_state_ack->ack, foc_arm_state_ack->data);
    }else{
        ttalink_foc_arm_state_ack_send(dst_addr, src_addr, chan, foc_arm_state_ack->update_time, foc_arm_state_ack->ack, foc_arm_state_ack->data);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_ARM_STATE_ACK, (const char *)foc_arm_state_ack, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_arm_state_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_arm_state_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack)
{
    _ttalink_foc_arm_state_ack_send_struct(dst_addr, src_addr, chan, foc_arm_state_ack, false);
}

/**
 * @brief Send a foc_arm_state_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_arm_state_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_arm_state_ack_t* foc_arm_state_ack)
{
    _ttalink_foc_arm_state_ack_send_struct(dst_addr, src_addr, chan, foc_arm_state_ack, true);
}

#if TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_arm_state_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ack, uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ack);
    _tta_put_uint32_t(buf, 8, data);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_ARM_STATE_ACK, buf, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC, nocrc);
#else
    ttalink_foc_arm_state_ack_t *packet = (ttalink_foc_arm_state_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->ack = ack;
    packet->data = data;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_ARM_STATE_ACK, (const char *)packet, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_arm_state_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ack, uint32_t data)
{
    _ttalink_foc_arm_state_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, data, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_arm_state_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ack, uint32_t data)
{
    _ttalink_foc_arm_state_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, data, true);
}
#endif

#endif

// MESSAGE FOC_ARM_STATE_ACK UNPACKING


/**
 * @brief Get field update_time from foc_arm_state_ack message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_arm_state_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ack from foc_arm_state_ack message
 *
 * @return FOC锁死状态  0=disarm:正常转动   1=arm:锁死
 */
static inline uint32_t ttalink_foc_arm_state_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field data from foc_arm_state_ack message
 *
 * @return 预留
 */
static inline uint32_t ttalink_foc_arm_state_ack_get_data(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a foc_arm_state_ack message into a struct
 *
 * @param msg The message to decode
 * @param foc_arm_state_ack C-struct to decode the message contents into
 */
static inline void ttalink_foc_arm_state_ack_decode(const ttalink_message_t* msg, ttalink_foc_arm_state_ack_t* foc_arm_state_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_arm_state_ack->update_time = ttalink_foc_arm_state_ack_get_update_time(msg);
    foc_arm_state_ack->ack = ttalink_foc_arm_state_ack_get_ack(msg);
    foc_arm_state_ack->data = ttalink_foc_arm_state_ack_get_data(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN? msg->len : TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN;
        memset(foc_arm_state_ack, 0, TTALINK_MSG_ID_FOC_ARM_STATE_ACK_LEN);
    memcpy(foc_arm_state_ack, _TTA_PAYLOAD(msg), len);
#endif
}
