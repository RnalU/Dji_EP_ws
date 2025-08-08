#pragma once
// MESSAGE YAW_CMD PACKING

#define TTALINK_MSG_ID_YAW_CMD 2070

TTAPACKED(
typedef struct __ttalink_yaw_cmd_t {
 uint8_t cmd; /*<  cmd.*/
 float target_yaw; /*<  yaw.*/
 int32_t target_latit; /*<  latit*10000000.*/
 int32_t target_longi; /*< longi*10000000.*/
}) ttalink_yaw_cmd_t;

#define TTALINK_MSG_ID_YAW_CMD_LEN 13
#define TTALINK_MSG_ID_YAW_CMD_MIN_LEN 13
#define TTALINK_MSG_ID_2070_LEN 13
#define TTALINK_MSG_ID_2070_MIN_LEN 13

#define TTALINK_MSG_ID_YAW_CMD_CRC 30
#define TTALINK_MSG_ID_2070_CRC 30



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_YAW_CMD { \
    2070, \
    "YAW_CMD", \
    4, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_yaw_cmd_t, cmd) }, \
         { "target_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_yaw_cmd_t, target_yaw) }, \
         { "target_latit", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_yaw_cmd_t, target_latit) }, \
         { "target_longi", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_yaw_cmd_t, target_longi) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_YAW_CMD { \
    "YAW_CMD", \
    4, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_yaw_cmd_t, cmd) }, \
         { "target_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_yaw_cmd_t, target_yaw) }, \
         { "target_latit", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_yaw_cmd_t, target_latit) }, \
         { "target_longi", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_yaw_cmd_t, target_longi) }, \
         } \
}
#endif


static inline uint16_t _ttalink_yaw_cmd_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_YAW_CMD_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_float(buf, 1, target_yaw);
    _tta_put_int32_t(buf, 5, target_latit);
    _tta_put_int32_t(buf, 9, target_longi);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_YAW_CMD_LEN);
#else
    ttalink_yaw_cmd_t packet;
    packet.cmd = cmd;
    packet.target_yaw = target_yaw;
    packet.target_latit = target_latit;
    packet.target_longi = target_longi;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_YAW_CMD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_YAW_CMD;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_YAW_CMD_MIN_LEN, TTALINK_MSG_ID_YAW_CMD_LEN, TTALINK_MSG_ID_YAW_CMD_CRC, nocrc);
}

/**
 * @brief Pack a yaw_cmd message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  cmd.
 * @param target_yaw  yaw.
 * @param target_latit  latit*10000000.
 * @param target_longi longi*10000000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_yaw_cmd_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi)
{
    return _ttalink_yaw_cmd_pack(dst_addr, src_addr, msg,  cmd, target_yaw, target_latit, target_longi, false);
}

/**
 * @brief Pack a yaw_cmd message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  cmd.
 * @param target_yaw  yaw.
 * @param target_latit  latit*10000000.
 * @param target_longi longi*10000000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_yaw_cmd_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi)
{
    return _ttalink_yaw_cmd_pack(dst_addr, src_addr, msg,  cmd, target_yaw, target_latit, target_longi, true);
}


static inline uint16_t _ttalink_yaw_cmd_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,float target_yaw,int32_t target_latit,int32_t target_longi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_YAW_CMD_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_float(buf, 1, target_yaw);
    _tta_put_int32_t(buf, 5, target_latit);
    _tta_put_int32_t(buf, 9, target_longi);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_YAW_CMD_LEN);
#else
    ttalink_yaw_cmd_t packet;
    packet.cmd = cmd;
    packet.target_yaw = target_yaw;
    packet.target_latit = target_latit;
    packet.target_longi = target_longi;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_YAW_CMD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_YAW_CMD;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_YAW_CMD_MIN_LEN, TTALINK_MSG_ID_YAW_CMD_LEN, TTALINK_MSG_ID_YAW_CMD_CRC, nocrc);
}

/**
 * @brief Pack a yaw_cmd message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  cmd.
 * @param target_yaw  yaw.
 * @param target_latit  latit*10000000.
 * @param target_longi longi*10000000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_yaw_cmd_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,float target_yaw,int32_t target_latit,int32_t target_longi)
{
    return _ttalink_yaw_cmd_pack_chan(dst_addr, src_addr, chan, msg,  cmd, target_yaw, target_latit, target_longi, false);
}

/**
 * @brief Pack a yaw_cmd message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  cmd.
 * @param target_yaw  yaw.
 * @param target_latit  latit*10000000.
 * @param target_longi longi*10000000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_yaw_cmd_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,float target_yaw,int32_t target_latit,int32_t target_longi)
{
    return _ttalink_yaw_cmd_pack_chan(dst_addr, src_addr, chan, msg,  cmd, target_yaw, target_latit, target_longi, true);
}


static inline uint16_t _ttalink_yaw_cmd_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_yaw_cmd_t* yaw_cmd, bool nocrc)
{
    if(nocrc){
        return ttalink_yaw_cmd_pack_nocrc(dst_addr, src_addr, msg, yaw_cmd->cmd, yaw_cmd->target_yaw, yaw_cmd->target_latit, yaw_cmd->target_longi);
    }else{
        return ttalink_yaw_cmd_pack(dst_addr, src_addr, msg, yaw_cmd->cmd, yaw_cmd->target_yaw, yaw_cmd->target_latit, yaw_cmd->target_longi);
    }
    
}

/**
 * @brief Encode a yaw_cmd struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param yaw_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_yaw_cmd_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_yaw_cmd_t* yaw_cmd)
{
    return _ttalink_yaw_cmd_encode(dst_addr, src_addr, msg, yaw_cmd, false);
}

/**
 * @brief Encode a yaw_cmd struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param yaw_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_yaw_cmd_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_yaw_cmd_t* yaw_cmd)
{
    return _ttalink_yaw_cmd_encode(dst_addr, src_addr, msg, yaw_cmd, true);
}


static inline uint16_t _ttalink_yaw_cmd_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_yaw_cmd_t* yaw_cmd, bool nocrc)
{
    if(nocrc){
        return ttalink_yaw_cmd_pack_chan_nocrc(dst_addr, src_addr, chan, msg, yaw_cmd->cmd, yaw_cmd->target_yaw, yaw_cmd->target_latit, yaw_cmd->target_longi);
    }else{
        return ttalink_yaw_cmd_pack_chan(dst_addr, src_addr, chan, msg, yaw_cmd->cmd, yaw_cmd->target_yaw, yaw_cmd->target_latit, yaw_cmd->target_longi);
    }
}

/**
 * @brief Encode a yaw_cmd struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param yaw_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_yaw_cmd_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_yaw_cmd_t* yaw_cmd)
{
    return _ttalink_yaw_cmd_encode_chan(dst_addr, src_addr, chan, msg, yaw_cmd, false);
}

/**
 * @brief Encode a yaw_cmd struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param yaw_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_yaw_cmd_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_yaw_cmd_t* yaw_cmd)
{
    return _ttalink_yaw_cmd_encode_chan(dst_addr, src_addr, chan, msg, yaw_cmd, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_yaw_cmd_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_YAW_CMD_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_float(buf, 1, target_yaw);
    _tta_put_int32_t(buf, 5, target_latit);
    _tta_put_int32_t(buf, 9, target_longi);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_YAW_CMD, buf, TTALINK_MSG_ID_YAW_CMD_MIN_LEN, TTALINK_MSG_ID_YAW_CMD_LEN, TTALINK_MSG_ID_YAW_CMD_CRC, nocrc);
#else
    ttalink_yaw_cmd_t packet;
    packet.cmd = cmd;
    packet.target_yaw = target_yaw;
    packet.target_latit = target_latit;
    packet.target_longi = target_longi;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_YAW_CMD, (const char *)&packet, TTALINK_MSG_ID_YAW_CMD_MIN_LEN, TTALINK_MSG_ID_YAW_CMD_LEN, TTALINK_MSG_ID_YAW_CMD_CRC, nocrc);
#endif
}

/**
 * @brief Send a yaw_cmd message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  cmd.
 * @param target_yaw  yaw.
 * @param target_latit  latit*10000000.
 * @param target_longi longi*10000000.
 */
static inline void ttalink_yaw_cmd_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi)
{
    _ttalink_yaw_cmd_send(dst_addr, src_addr, chan, cmd, target_yaw, target_latit, target_longi, false);
}

/**
 * @brief Send a yaw_cmd message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  cmd.
 * @param target_yaw  yaw.
 * @param target_latit  latit*10000000.
 * @param target_longi longi*10000000.
 */
static inline void ttalink_yaw_cmd_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi)
{
    _ttalink_yaw_cmd_send(dst_addr, src_addr, chan, cmd, target_yaw, target_latit, target_longi, true);
}


static inline void _ttalink_yaw_cmd_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_yaw_cmd_t* yaw_cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_yaw_cmd_send_nocrc(dst_addr, src_addr, chan, yaw_cmd->cmd, yaw_cmd->target_yaw, yaw_cmd->target_latit, yaw_cmd->target_longi);
    }else{
        ttalink_yaw_cmd_send(dst_addr, src_addr, chan, yaw_cmd->cmd, yaw_cmd->target_yaw, yaw_cmd->target_latit, yaw_cmd->target_longi);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_YAW_CMD, (const char *)yaw_cmd, TTALINK_MSG_ID_YAW_CMD_MIN_LEN, TTALINK_MSG_ID_YAW_CMD_LEN, TTALINK_MSG_ID_YAW_CMD_CRC, nocrc);
#endif
}

/**
 * @brief Send a yaw_cmd message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_yaw_cmd_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_yaw_cmd_t* yaw_cmd)
{
    _ttalink_yaw_cmd_send_struct(dst_addr, src_addr, chan, yaw_cmd, false);
}

/**
 * @brief Send a yaw_cmd message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_yaw_cmd_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_yaw_cmd_t* yaw_cmd)
{
    _ttalink_yaw_cmd_send_struct(dst_addr, src_addr, chan, yaw_cmd, true);
}

#if TTALINK_MSG_ID_YAW_CMD_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_yaw_cmd_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_float(buf, 1, target_yaw);
    _tta_put_int32_t(buf, 5, target_latit);
    _tta_put_int32_t(buf, 9, target_longi);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_YAW_CMD, buf, TTALINK_MSG_ID_YAW_CMD_MIN_LEN, TTALINK_MSG_ID_YAW_CMD_LEN, TTALINK_MSG_ID_YAW_CMD_CRC, nocrc);
#else
    ttalink_yaw_cmd_t *packet = (ttalink_yaw_cmd_t *)msgbuf;
    packet->cmd = cmd;
    packet->target_yaw = target_yaw;
    packet->target_latit = target_latit;
    packet->target_longi = target_longi;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_YAW_CMD, (const char *)packet, TTALINK_MSG_ID_YAW_CMD_MIN_LEN, TTALINK_MSG_ID_YAW_CMD_LEN, TTALINK_MSG_ID_YAW_CMD_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_yaw_cmd_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi)
{
    _ttalink_yaw_cmd_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, target_yaw, target_latit, target_longi, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_yaw_cmd_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, float target_yaw, int32_t target_latit, int32_t target_longi)
{
    _ttalink_yaw_cmd_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, target_yaw, target_latit, target_longi, true);
}
#endif

#endif

// MESSAGE YAW_CMD UNPACKING


/**
 * @brief Get field cmd from yaw_cmd message
 *
 * @return  cmd.
 */
static inline uint8_t ttalink_yaw_cmd_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_yaw from yaw_cmd message
 *
 * @return  yaw.
 */
static inline float ttalink_yaw_cmd_get_target_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  1);
}

/**
 * @brief Get field target_latit from yaw_cmd message
 *
 * @return  latit*10000000.
 */
static inline int32_t ttalink_yaw_cmd_get_target_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  5);
}

/**
 * @brief Get field target_longi from yaw_cmd message
 *
 * @return longi*10000000.
 */
static inline int32_t ttalink_yaw_cmd_get_target_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  9);
}

/**
 * @brief Decode a yaw_cmd message into a struct
 *
 * @param msg The message to decode
 * @param yaw_cmd C-struct to decode the message contents into
 */
static inline void ttalink_yaw_cmd_decode(const ttalink_message_t* msg, ttalink_yaw_cmd_t* yaw_cmd)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    yaw_cmd->cmd = ttalink_yaw_cmd_get_cmd(msg);
    yaw_cmd->target_yaw = ttalink_yaw_cmd_get_target_yaw(msg);
    yaw_cmd->target_latit = ttalink_yaw_cmd_get_target_latit(msg);
    yaw_cmd->target_longi = ttalink_yaw_cmd_get_target_longi(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_YAW_CMD_LEN? msg->len : TTALINK_MSG_ID_YAW_CMD_LEN;
        memset(yaw_cmd, 0, TTALINK_MSG_ID_YAW_CMD_LEN);
    memcpy(yaw_cmd, _TTA_PAYLOAD(msg), len);
#endif
}
