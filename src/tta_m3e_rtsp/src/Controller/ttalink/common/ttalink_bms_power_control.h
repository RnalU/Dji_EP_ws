#pragma once
// MESSAGE BMS_POWER_CONTROL PACKING

#define TTALINK_MSG_ID_BMS_POWER_CONTROL 2050

TTAPACKED(
typedef struct __ttalink_bms_power_control_t {
 uint32_t update_time; /*<  . .*/
 uint8_t cmd; /*<  . */
}) ttalink_bms_power_control_t;

#define TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN 5
#define TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN 5
#define TTALINK_MSG_ID_2050_LEN 5
#define TTALINK_MSG_ID_2050_MIN_LEN 5

#define TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC 147
#define TTALINK_MSG_ID_2050_CRC 147



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_BMS_POWER_CONTROL { \
    2050, \
    "BMS_POWER_CONTROL", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_power_control_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_bms_power_control_t, cmd) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_BMS_POWER_CONTROL { \
    "BMS_POWER_CONTROL", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_power_control_t, update_time) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_bms_power_control_t, cmd) }, \
         } \
}
#endif


static inline uint16_t _ttalink_bms_power_control_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN);
#else
    ttalink_bms_power_control_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_POWER_CONTROL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC, nocrc);
}

/**
 * @brief Pack a bms_power_control message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param cmd  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_power_control_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd)
{
    return _ttalink_bms_power_control_pack(dst_addr, src_addr, msg,  update_time, cmd, false);
}

/**
 * @brief Pack a bms_power_control message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @param cmd  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_power_control_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd)
{
    return _ttalink_bms_power_control_pack(dst_addr, src_addr, msg,  update_time, cmd, true);
}


static inline uint16_t _ttalink_bms_power_control_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN);
#else
    ttalink_bms_power_control_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_POWER_CONTROL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC, nocrc);
}

/**
 * @brief Pack a bms_power_control message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param cmd  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_power_control_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd)
{
    return _ttalink_bms_power_control_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, false);
}

/**
 * @brief Pack a bms_power_control message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @param cmd  . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_power_control_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd)
{
    return _ttalink_bms_power_control_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd, true);
}


static inline uint16_t _ttalink_bms_power_control_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_power_control_t* bms_power_control, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_power_control_pack_nocrc(dst_addr, src_addr, msg, bms_power_control->update_time, bms_power_control->cmd);
    }else{
        return ttalink_bms_power_control_pack(dst_addr, src_addr, msg, bms_power_control->update_time, bms_power_control->cmd);
    }
    
}

/**
 * @brief Encode a bms_power_control struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_power_control C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_power_control_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_power_control_t* bms_power_control)
{
    return _ttalink_bms_power_control_encode(dst_addr, src_addr, msg, bms_power_control, false);
}

/**
 * @brief Encode a bms_power_control struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_power_control C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_power_control_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_power_control_t* bms_power_control)
{
    return _ttalink_bms_power_control_encode(dst_addr, src_addr, msg, bms_power_control, true);
}


static inline uint16_t _ttalink_bms_power_control_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_power_control_t* bms_power_control, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_power_control_pack_chan_nocrc(dst_addr, src_addr, chan, msg, bms_power_control->update_time, bms_power_control->cmd);
    }else{
        return ttalink_bms_power_control_pack_chan(dst_addr, src_addr, chan, msg, bms_power_control->update_time, bms_power_control->cmd);
    }
}

/**
 * @brief Encode a bms_power_control struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_power_control C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_power_control_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_power_control_t* bms_power_control)
{
    return _ttalink_bms_power_control_encode_chan(dst_addr, src_addr, chan, msg, bms_power_control, false);
}

/**
 * @brief Encode a bms_power_control struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_power_control C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_power_control_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_power_control_t* bms_power_control)
{
    return _ttalink_bms_power_control_encode_chan(dst_addr, src_addr, chan, msg, bms_power_control, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_bms_power_control_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_POWER_CONTROL, buf, TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC, nocrc);
#else
    ttalink_bms_power_control_t packet;
    packet.update_time = update_time;
    packet.cmd = cmd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_POWER_CONTROL, (const char *)&packet, TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_power_control message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param cmd  . 
 */
static inline void ttalink_bms_power_control_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd)
{
    _ttalink_bms_power_control_send(dst_addr, src_addr, chan, update_time, cmd, false);
}

/**
 * @brief Send a bms_power_control message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 * @param cmd  . 
 */
static inline void ttalink_bms_power_control_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd)
{
    _ttalink_bms_power_control_send(dst_addr, src_addr, chan, update_time, cmd, true);
}


static inline void _ttalink_bms_power_control_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_power_control_t* bms_power_control, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_bms_power_control_send_nocrc(dst_addr, src_addr, chan, bms_power_control->update_time, bms_power_control->cmd);
    }else{
        ttalink_bms_power_control_send(dst_addr, src_addr, chan, bms_power_control->update_time, bms_power_control->cmd);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_POWER_CONTROL, (const char *)bms_power_control, TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_power_control message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_power_control_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_power_control_t* bms_power_control)
{
    _ttalink_bms_power_control_send_struct(dst_addr, src_addr, chan, bms_power_control, false);
}

/**
 * @brief Send a bms_power_control message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_power_control_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_power_control_t* bms_power_control)
{
    _ttalink_bms_power_control_send_struct(dst_addr, src_addr, chan, bms_power_control, true);
}

#if TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_bms_power_control_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_POWER_CONTROL, buf, TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC, nocrc);
#else
    ttalink_bms_power_control_t *packet = (ttalink_bms_power_control_t *)msgbuf;
    packet->update_time = update_time;
    packet->cmd = cmd;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_POWER_CONTROL, (const char *)packet, TTALINK_MSG_ID_BMS_POWER_CONTROL_MIN_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN, TTALINK_MSG_ID_BMS_POWER_CONTROL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_bms_power_control_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd)
{
    _ttalink_bms_power_control_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_bms_power_control_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd)
{
    _ttalink_bms_power_control_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd, true);
}
#endif

#endif

// MESSAGE BMS_POWER_CONTROL UNPACKING


/**
 * @brief Get field update_time from bms_power_control message
 *
 * @return  . .
 */
static inline uint32_t ttalink_bms_power_control_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd from bms_power_control message
 *
 * @return  . 
 */
static inline uint8_t ttalink_bms_power_control_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a bms_power_control message into a struct
 *
 * @param msg The message to decode
 * @param bms_power_control C-struct to decode the message contents into
 */
static inline void ttalink_bms_power_control_decode(const ttalink_message_t* msg, ttalink_bms_power_control_t* bms_power_control)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    bms_power_control->update_time = ttalink_bms_power_control_get_update_time(msg);
    bms_power_control->cmd = ttalink_bms_power_control_get_cmd(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN? msg->len : TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN;
        memset(bms_power_control, 0, TTALINK_MSG_ID_BMS_POWER_CONTROL_LEN);
    memcpy(bms_power_control, _TTA_PAYLOAD(msg), len);
#endif
}
