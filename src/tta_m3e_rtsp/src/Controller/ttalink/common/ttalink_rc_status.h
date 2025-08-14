#pragma once
// MESSAGE RC_STATUS PACKING

#define TTALINK_MSG_ID_RC_STATUS 2232

TTAPACKED(
typedef struct __ttalink_rc_status_t {
 uint32_t update_time; /*<  .*/
 uint8_t cmd; /*< 0：NULL 1：设置 2：查询 3：通知.*/
 uint8_t rc_device; /*<   0：T12/FUATBA  1:H12.*/
 uint8_t rc_mode; /*<   0：NULL  1：美国手  2：日本手.*/
 float param1; /*<  预留.*/
}) ttalink_rc_status_t;

#define TTALINK_MSG_ID_RC_STATUS_LEN 11
#define TTALINK_MSG_ID_RC_STATUS_MIN_LEN 11
#define TTALINK_MSG_ID_2232_LEN 11
#define TTALINK_MSG_ID_2232_MIN_LEN 11

#define TTALINK_MSG_ID_RC_STATUS_CRC 210
#define TTALINK_MSG_ID_2232_CRC 210



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_RC_STATUS { \
    2232, \
    "RC_STATUS", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_rc_status_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_rc_status_t, cmd) }, \
         { "rc_device", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_rc_status_t, rc_device) }, \
         { "rc_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_rc_status_t, rc_mode) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_rc_status_t, param1) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_RC_STATUS { \
    "RC_STATUS", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_rc_status_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_rc_status_t, cmd) }, \
         { "rc_device", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_rc_status_t, rc_device) }, \
         { "rc_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_rc_status_t, rc_mode) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_rc_status_t, param1) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rc_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, rc_device);
    _tta_put_uint8_t(buf, 6, rc_mode);
    _tta_put_float(buf, 7, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RC_STATUS_LEN);
#else
    ttalink_rc_status_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.rc_device = rc_device;
    packet.rc_mode = rc_mode;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RC_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RC_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_RC_STATUS_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_LEN, TTALINK_MSG_ID_RC_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a rc_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param cmd 0：NULL 1：设置 2：查询 3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    return _ttalink_rc_status_pack(dst_addr, src_addr, msg,  update_time, cmd, rc_device, rc_mode, param1, false);
}

/**
 * @brief Pack a rc_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param cmd 0：NULL 1：设置 2：查询 3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    return _ttalink_rc_status_pack(dst_addr, src_addr, msg,  update_time, cmd, rc_device, rc_mode, param1, true);
}


static inline uint16_t _ttalink_rc_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,uint8_t rc_device,uint8_t rc_mode,float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, rc_device);
    _tta_put_uint8_t(buf, 6, rc_mode);
    _tta_put_float(buf, 7, param1);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RC_STATUS_LEN);
#else
    ttalink_rc_status_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.rc_device = rc_device;
    packet.rc_mode = rc_mode;
    packet.param1 = param1;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RC_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RC_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_LEN, TTALINK_MSG_ID_RC_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a rc_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param cmd 0：NULL 1：设置 2：查询 3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,uint8_t rc_device,uint8_t rc_mode,float param1)
{
    return _ttalink_rc_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, rc_device, rc_mode, param1, false);
}

/**
 * @brief Pack a rc_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param cmd 0：NULL 1：设置 2：查询 3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rc_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd,uint8_t rc_device,uint8_t rc_mode,float param1)
{
    return _ttalink_rc_status_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, rc_device, rc_mode, param1, true);
}


static inline uint16_t _ttalink_rc_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_status_t* rc_status, bool nocrc)
{
    if(nocrc){
        return ttalink_rc_status_pack_nocrc(dst_addr, src_addr, msg, rc_status->update_time, rc_status->cmd, rc_status->rc_device, rc_status->rc_mode, rc_status->param1);
    }else{
        return ttalink_rc_status_pack(dst_addr, src_addr, msg, rc_status->update_time, rc_status->cmd, rc_status->rc_device, rc_status->rc_mode, rc_status->param1);
    }
    
}

/**
 * @brief Encode a rc_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rc_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_status_t* rc_status)
{
    return _ttalink_rc_status_encode(dst_addr, src_addr, msg, rc_status, false);
}

/**
 * @brief Encode a rc_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rc_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rc_status_t* rc_status)
{
    return _ttalink_rc_status_encode(dst_addr, src_addr, msg, rc_status, true);
}


static inline uint16_t _ttalink_rc_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_status_t* rc_status, bool nocrc)
{
    if(nocrc){
        return ttalink_rc_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rc_status->update_time, rc_status->cmd, rc_status->rc_device, rc_status->rc_mode, rc_status->param1);
    }else{
        return ttalink_rc_status_pack_chan(dst_addr, src_addr, chan, msg, rc_status->update_time, rc_status->cmd, rc_status->rc_device, rc_status->rc_mode, rc_status->param1);
    }
}

/**
 * @brief Encode a rc_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_status_t* rc_status)
{
    return _ttalink_rc_status_encode_chan(dst_addr, src_addr, chan, msg, rc_status, false);
}

/**
 * @brief Encode a rc_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rc_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_rc_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rc_status_t* rc_status)
{
    return _ttalink_rc_status_encode_chan(dst_addr, src_addr, chan, msg, rc_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rc_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RC_STATUS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, rc_device);
    _tta_put_uint8_t(buf, 6, rc_mode);
    _tta_put_float(buf, 7, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS, buf, TTALINK_MSG_ID_RC_STATUS_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_LEN, TTALINK_MSG_ID_RC_STATUS_CRC, nocrc);
#else
    ttalink_rc_status_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;
    packet.rc_device = rc_device;
    packet.rc_mode = rc_mode;
    packet.param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS, (const char *)&packet, TTALINK_MSG_ID_RC_STATUS_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_LEN, TTALINK_MSG_ID_RC_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a rc_status message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param cmd 0：NULL 1：设置 2：查询 3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 */
static inline void ttalink_rc_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_send(dst_addr, src_addr, chan, update_time, cmd, rc_device, rc_mode, param1, false);
}

/**
 * @brief Send a rc_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param cmd 0：NULL 1：设置 2：查询 3：通知.
 * @param rc_device   0：T12/FUATBA  1:H12.
 * @param rc_mode   0：NULL  1：美国手  2：日本手.
 * @param param1  预留.
 */
static inline void ttalink_rc_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_send(dst_addr, src_addr, chan, update_time, cmd, rc_device, rc_mode, param1, true);
}


static inline void _ttalink_rc_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_status_t* rc_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rc_status_send_nocrc(dst_addr, src_addr, chan, rc_status->update_time, rc_status->cmd, rc_status->rc_device, rc_status->rc_mode, rc_status->param1);
    }else{
        ttalink_rc_status_send(dst_addr, src_addr, chan, rc_status->update_time, rc_status->cmd, rc_status->rc_device, rc_status->rc_mode, rc_status->param1);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS, (const char *)rc_status, TTALINK_MSG_ID_RC_STATUS_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_LEN, TTALINK_MSG_ID_RC_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a rc_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rc_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_status_t* rc_status)
{
    _ttalink_rc_status_send_struct(dst_addr, src_addr, chan, rc_status, false);
}

/**
 * @brief Send a rc_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rc_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rc_status_t* rc_status)
{
    _ttalink_rc_status_send_struct(dst_addr, src_addr, chan, rc_status, true);
}

#if TTALINK_MSG_ID_RC_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rc_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);
    _tta_put_uint8_t(buf, 5, rc_device);
    _tta_put_uint8_t(buf, 6, rc_mode);
    _tta_put_float(buf, 7, param1);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS, buf, TTALINK_MSG_ID_RC_STATUS_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_LEN, TTALINK_MSG_ID_RC_STATUS_CRC, nocrc);
#else
    ttalink_rc_status_t *packet = (ttalink_rc_status_t *)msgbuf;
    packet->update_time = update_time;
    packet->cmd = cmd;
    packet->rc_device = rc_device;
    packet->rc_mode = rc_mode;
    packet->param1 = param1;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RC_STATUS, (const char *)packet, TTALINK_MSG_ID_RC_STATUS_MIN_LEN, TTALINK_MSG_ID_RC_STATUS_LEN, TTALINK_MSG_ID_RC_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rc_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, rc_device, rc_mode, param1, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rc_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, uint8_t rc_device, uint8_t rc_mode, float param1)
{
    _ttalink_rc_status_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, rc_device, rc_mode, param1, true);
}
#endif

#endif

// MESSAGE RC_STATUS UNPACKING


/**
 * @brief Get field update_time from rc_status message
 *
 * @return  .
 */
static inline uint32_t ttalink_rc_status_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd from rc_status message
 *
 * @return 0：NULL 1：设置 2：查询 3：通知.
 */
static inline uint8_t ttalink_rc_status_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field rc_device from rc_status message
 *
 * @return   0：T12/FUATBA  1:H12.
 */
static inline uint8_t ttalink_rc_status_get_rc_device(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field rc_mode from rc_status message
 *
 * @return   0：NULL  1：美国手  2：日本手.
 */
static inline uint8_t ttalink_rc_status_get_rc_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field param1 from rc_status message
 *
 * @return  预留.
 */
static inline float ttalink_rc_status_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  7);
}

/**
 * @brief Decode a rc_status message into a struct
 *
 * @param msg The message to decode
 * @param rc_status C-struct to decode the message contents into
 */
static inline void ttalink_rc_status_decode(const ttalink_message_t* msg, ttalink_rc_status_t* rc_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rc_status->update_time = ttalink_rc_status_get_update_time(msg);
    rc_status->cmd = ttalink_rc_status_get_cmd(msg);
    rc_status->rc_device = ttalink_rc_status_get_rc_device(msg);
    rc_status->rc_mode = ttalink_rc_status_get_rc_mode(msg);
    rc_status->param1 = ttalink_rc_status_get_param1(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_RC_STATUS_LEN? msg->len : TTALINK_MSG_ID_RC_STATUS_LEN;
        memset(rc_status, 0, TTALINK_MSG_ID_RC_STATUS_LEN);
    memcpy(rc_status, _TTA_PAYLOAD(msg), len);
#endif
}
