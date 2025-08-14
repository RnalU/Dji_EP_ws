#pragma once
// MESSAGE RC_STATUS_ACK PACKING

#define TTALINK_MSG_ID_RC_STATUS_ACK 2233

TTAPACKED(
typedef struct __ttalink_rc_status_ack_t {
 uint32_t update_time; /*<  .*/
 uint8_t ack; /*< 0：成功 1：失败.*/
 uint8_t cmd; /*< 0：NULL 1：设置 2：查询.3：通知.*/
 uint8_t rc_device; /*<   0：T12/FUATBA  1:H12.*/
 uint8_t rc_mode; /*<   0：NULL  1：美国手  2：日本手.*/
 float param1; /*<  预留.*/
}) ttalink_rc_status_ack_t;

#define TTALINK_MSG_ID_RC_STATUS_ACK_LEN 12
#define TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN 12
#define TTALINK_MSG_ID_2233_LEN 12
#define TTALINK_MSG_ID_2233_MIN_LEN 12

#define TTALINK_MSG_ID_RC_STATUS_ACK_CRC 240
#define TTALINK_MSG_ID_2233_CRC 240



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_RC_STATUS_ACK { \
    2233, \
    "RC_STATUS_ACK", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_rc_status_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_rc_status_ack_t, ack) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_rc_status_ack_t, cmd) }, \
         { "rc_device", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_rc_status_ack_t, rc_device) }, \
         { "rc_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_rc_status_ack_t, rc_mode) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_rc_status_ack_t, param1) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_RC_STATUS_ACK { \
    "RC_STATUS_ACK", \
    6, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_rc_status_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_rc_status_ack_t, ack) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_rc_status_ack_t, cmd) }, \
         { "rc_device", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_rc_status_ack_t, rc_device) }, \
         { "rc_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_rc_status_ack_t, rc_mode) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_rc_status_ack_t, param1) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rc_status_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_STATUS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, cmd);
    _tta_put_uint8_t(buf, 6, rc_device);
    _tta_put_uint8_t(buf, 7, rc_mode);
    _tta_put_float(buf, 8, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RC_STATUS_ACK_LEN);
#else
    ttalink_rc_status_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.cmd = cmd;
    packet.rc_device = rc_device;
    packet.rc_mode = rc_mode;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RC_STATUS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RC_STATUS_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a rc_status_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param ack 0：成功 1：失败.
 * @param cmd 0：NULL 1：设置 2：查询.3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    return _ttalink_rc_status_ack_pack(dst_addr, src_addr, msg,  update_time, ack, cmd, rc_device, rc_mode, param1, false);
}

/**
 * @brief Pack a rc_status_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param ack 0：成功 1：失败.
 * @param cmd 0：NULL 1：设置 2：查询.3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    return _ttalink_rc_status_ack_pack(dst_addr, src_addr, msg,  update_time, ack, cmd, rc_device, rc_mode, param1, true);
}


static inline uint16_t _ttalink_rc_status_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,uint8_t cmd,uint8_t rc_device,uint8_t rc_mode,float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_STATUS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, cmd);
    _tta_put_uint8_t(buf, 6, rc_device);
    _tta_put_uint8_t(buf, 7, rc_mode);
    _tta_put_float(buf, 8, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RC_STATUS_ACK_LEN);
#else
    ttalink_rc_status_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.cmd = cmd;
    packet.rc_device = rc_device;
    packet.rc_mode = rc_mode;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RC_STATUS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RC_STATUS_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a rc_status_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param ack 0：成功 1：失败.
 * @param cmd 0：NULL 1：设置 2：查询.3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,uint8_t cmd,uint8_t rc_device,uint8_t rc_mode,float param1)
{
    return _ttalink_rc_status_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, cmd, rc_device, rc_mode, param1, false);
}

/**
 * @brief Pack a rc_status_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param ack 0：成功 1：失败.
 * @param cmd 0：NULL 1：设置 2：查询.3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,uint8_t cmd,uint8_t rc_device,uint8_t rc_mode,float param1)
{
    return _ttalink_rc_status_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, cmd, rc_device, rc_mode, param1, true);
}


static inline uint16_t _ttalink_rc_status_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_status_ack_t* rc_status_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_rc_status_ack_pack_nocrc(dst_addr, src_addr, msg, rc_status_ack->update_time, rc_status_ack->ack, rc_status_ack->cmd, rc_status_ack->rc_device, rc_status_ack->rc_mode, rc_status_ack->param1);
    }else{
        return ttalink_rc_status_ack_pack(dst_addr, src_addr, msg, rc_status_ack->update_time, rc_status_ack->ack, rc_status_ack->cmd, rc_status_ack->rc_device, rc_status_ack->rc_mode, rc_status_ack->param1);
    }
    
}

/**
 * @brief Encode a rc_status_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rc_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_status_ack_t* rc_status_ack)
{
    return _ttalink_rc_status_ack_encode(dst_addr, src_addr, msg, rc_status_ack, false);
}

/**
 * @brief Encode a rc_status_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rc_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_status_ack_t* rc_status_ack)
{
    return _ttalink_rc_status_ack_encode(dst_addr, src_addr, msg, rc_status_ack, true);
}


static inline uint16_t _ttalink_rc_status_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_status_ack_t* rc_status_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_rc_status_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rc_status_ack->update_time, rc_status_ack->ack, rc_status_ack->cmd, rc_status_ack->rc_device, rc_status_ack->rc_mode, rc_status_ack->param1);
    }else{
        return ttalink_rc_status_ack_pack_chan(dst_addr, src_addr, chan, msg, rc_status_ack->update_time, rc_status_ack->ack, rc_status_ack->cmd, rc_status_ack->rc_device, rc_status_ack->rc_mode, rc_status_ack->param1);
    }
}

/**
 * @brief Encode a rc_status_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_status_ack_t* rc_status_ack)
{
    return _ttalink_rc_status_ack_encode_chan(dst_addr, src_addr, chan, msg, rc_status_ack, false);
}

/**
 * @brief Encode a rc_status_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_status_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_status_ack_t* rc_status_ack)
{
    return _ttalink_rc_status_ack_encode_chan(dst_addr, src_addr, chan, msg, rc_status_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rc_status_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_STATUS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, cmd);
    _tta_put_uint8_t(buf, 6, rc_device);
    _tta_put_uint8_t(buf, 7, rc_mode);
    _tta_put_float(buf, 8, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS_ACK, buf, TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_CRC, nocrc);
#else
    ttalink_rc_status_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.cmd = cmd;
    packet.rc_device = rc_device;
    packet.rc_mode = rc_mode;
    packet.param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS_ACK, (const char *)&packet, TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a rc_status_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param ack 0：成功 1：失败.
 * @param cmd 0：NULL 1：设置 2：查询.3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 */
static inline void ttalink_rc_status_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_ack_send(dst_addr, src_addr, chan, update_time, ack, cmd, rc_device, rc_mode, param1, false);
}

/**
 * @brief Send a rc_status_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param ack 0：成功 1：失败.
 * @param cmd 0：NULL 1：设置 2：查询.3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 */
static inline void ttalink_rc_status_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_ack_send(dst_addr, src_addr, chan, update_time, ack, cmd, rc_device, rc_mode, param1, true);
}


static inline void _ttalink_rc_status_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_status_ack_t* rc_status_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rc_status_ack_send_nocrc(dst_addr, src_addr, chan, rc_status_ack->update_time, rc_status_ack->ack, rc_status_ack->cmd, rc_status_ack->rc_device, rc_status_ack->rc_mode, rc_status_ack->param1);
    }else{
        ttalink_rc_status_ack_send(dst_addr, src_addr, chan, rc_status_ack->update_time, rc_status_ack->ack, rc_status_ack->cmd, rc_status_ack->rc_device, rc_status_ack->rc_mode, rc_status_ack->param1);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS_ACK, (const char *)rc_status_ack, TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a rc_status_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rc_status_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_status_ack_t* rc_status_ack)
{
    _ttalink_rc_status_ack_send_struct(dst_addr, src_addr, chan, rc_status_ack, false);
}

/**
 * @brief Send a rc_status_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rc_status_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_status_ack_t* rc_status_ack)
{
    _ttalink_rc_status_ack_send_struct(dst_addr, src_addr, chan, rc_status_ack, true);
}

#if TTALINK_MSG_ID_RC_STATUS_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rc_status_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, cmd);
    _tta_put_uint8_t(buf, 6, rc_device);
    _tta_put_uint8_t(buf, 7, rc_mode);
    _tta_put_float(buf, 8, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS_ACK, buf, TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_CRC, nocrc);
#else
    ttalink_rc_status_ack_t *packet = (ttalink_rc_status_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->ack = ack;
    packet->cmd = cmd;
    packet->rc_device = rc_device;
    packet->rc_mode = rc_mode;
    packet->param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS_ACK, (const char *)packet, TTALINK_MSG_ID_RC_STATUS_ACK_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_LEN, TTALINK_MSG_ID_RC_STATUS_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rc_status_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, cmd, rc_device, rc_mode, param1, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rc_status_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, cmd, rc_device, rc_mode, param1, true);
}
#endif

#endif

// MESSAGE RC_STATUS_ACK UNPACKING


/**
 * @brief Get field update_time from rc_status_ack message
 *
 * @return  .
 */
static inline uint32_t ttalink_rc_status_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ack from rc_status_ack message
 *
 * @return 0：成功 1：失败.
 */
static inline uint8_t ttalink_rc_status_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field cmd from rc_status_ack message
 *
 * @return 0：NULL 1：设置 2：查询.3：通知.
 */
static inline uint8_t ttalink_rc_status_ack_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field rc_device from rc_status_ack message
 *
 * @return   0：T12/FUATBA  1:H12.
 */
static inline uint8_t ttalink_rc_status_ack_get_rc_device(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field rc_mode from rc_status_ack message
 *
 * @return   0：NULL  1：美国手  2：日本手.
 */
static inline uint8_t ttalink_rc_status_ack_get_rc_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field param1 from rc_status_ack message
 *
 * @return  预留.
 */
static inline float ttalink_rc_status_ack_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Decode a rc_status_ack message into a struct
 *
 * @param msg The message to decode
 * @param rc_status_ack C-struct to decode the message contents into
 */
static inline void ttalink_rc_status_ack_decode(const ttalink_message_t* msg, ttalink_rc_status_ack_t* rc_status_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rc_status_ack->update_time = ttalink_rc_status_ack_get_update_time(msg);
    rc_status_ack->ack = ttalink_rc_status_ack_get_ack(msg);
    rc_status_ack->cmd = ttalink_rc_status_ack_get_cmd(msg);
    rc_status_ack->rc_device = ttalink_rc_status_ack_get_rc_device(msg);
    rc_status_ack->rc_mode = ttalink_rc_status_ack_get_rc_mode(msg);
    rc_status_ack->param1 = ttalink_rc_status_ack_get_param1(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_RC_STATUS_ACK_LEN? msg->len : TTALINK_MSG_ID_RC_STATUS_ACK_LEN;
        memset(rc_status_ack, 0, TTALINK_MSG_ID_RC_STATUS_ACK_LEN);
    memcpy(rc_status_ack, _TTA_PAYLOAD(msg), len);
#endif
}
