#pragma once
// MESSAGE FOC_PARAMETERS PACKING

#define TTALINK_MSG_ID_FOC_PARAMETERS 2014

TTAPACKED(
typedef struct __ttalink_foc_parameters_t {
 uint32_t update_time; /*< system time*/
 uint8_t cmd_rw; /*< Judging the direction of command,0 read 1 write*/
 uint8_t parameters; /*< Parameter type*/
 uint16_t param1; /*< parameter data*/
 uint16_t param2; /*< parameter data*/
}) ttalink_foc_parameters_t;

#define TTALINK_MSG_ID_FOC_PARAMETERS_LEN 10
#define TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN 10
#define TTALINK_MSG_ID_2014_LEN 10
#define TTALINK_MSG_ID_2014_MIN_LEN 10

#define TTALINK_MSG_ID_FOC_PARAMETERS_CRC 34
#define TTALINK_MSG_ID_2014_CRC 34



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_PARAMETERS { \
    2014, \
    "FOC_PARAMETERS", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_parameters_t, update_time) }, \
         { "cmd_rw", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_parameters_t, cmd_rw) }, \
         { "parameters", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_foc_parameters_t, parameters) }, \
         { "param1", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_foc_parameters_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_UINT16_T, 0, 8, offsetof(ttalink_foc_parameters_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_PARAMETERS { \
    "FOC_PARAMETERS", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_parameters_t, update_time) }, \
         { "cmd_rw", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_parameters_t, cmd_rw) }, \
         { "parameters", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_foc_parameters_t, parameters) }, \
         { "param1", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_foc_parameters_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_UINT16_T, 0, 8, offsetof(ttalink_foc_parameters_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_parameters_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_PARAMETERS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint16_t(buf, 6, param1);
    _tta_put_uint16_t(buf, 8, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_PARAMETERS_LEN);
#else
    ttalink_foc_parameters_t packet;
    packet.update_time = update_time;
    packet.cmd_rw = cmd_rw;
    packet.parameters = parameters;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_PARAMETERS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_PARAMETERS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_CRC, nocrc);
}

/**
 * @brief Pack a foc_parameters message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param cmd_rw Judging the direction of command,0 read 1 write
 * @param parameters Parameter type
 * @param param1 parameter data
 * @param param2 parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2)
{
    return _ttalink_foc_parameters_pack(dst_addr, src_addr, msg,  update_time, cmd_rw, parameters, param1, param2, false);
}

/**
 * @brief Pack a foc_parameters message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param cmd_rw Judging the direction of command,0 read 1 write
 * @param parameters Parameter type
 * @param param1 parameter data
 * @param param2 parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2)
{
    return _ttalink_foc_parameters_pack(dst_addr, src_addr, msg,  update_time, cmd_rw, parameters, param1, param2, true);
}


static inline uint16_t _ttalink_foc_parameters_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd_rw,uint8_t parameters,uint16_t param1,uint16_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_PARAMETERS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint16_t(buf, 6, param1);
    _tta_put_uint16_t(buf, 8, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_PARAMETERS_LEN);
#else
    ttalink_foc_parameters_t packet;
    packet.update_time = update_time;
    packet.cmd_rw = cmd_rw;
    packet.parameters = parameters;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_PARAMETERS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_PARAMETERS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_CRC, nocrc);
}

/**
 * @brief Pack a foc_parameters message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param cmd_rw Judging the direction of command,0 read 1 write
 * @param parameters Parameter type
 * @param param1 parameter data
 * @param param2 parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd_rw,uint8_t parameters,uint16_t param1,uint16_t param2)
{
    return _ttalink_foc_parameters_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd_rw, parameters, param1, param2, false);
}

/**
 * @brief Pack a foc_parameters message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param cmd_rw Judging the direction of command,0 read 1 write
 * @param parameters Parameter type
 * @param param1 parameter data
 * @param param2 parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t cmd_rw,uint8_t parameters,uint16_t param1,uint16_t param2)
{
    return _ttalink_foc_parameters_pack_chan(dst_addr, src_addr, chan, msg,  update_time, cmd_rw, parameters, param1, param2, true);
}


static inline uint16_t _ttalink_foc_parameters_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_parameters_t* foc_parameters, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_parameters_pack_nocrc(dst_addr, src_addr, msg, foc_parameters->update_time, foc_parameters->cmd_rw, foc_parameters->parameters, foc_parameters->param1, foc_parameters->param2);
    }else{
        return ttalink_foc_parameters_pack(dst_addr, src_addr, msg, foc_parameters->update_time, foc_parameters->cmd_rw, foc_parameters->parameters, foc_parameters->param1, foc_parameters->param2);
    }
    
}

/**
 * @brief Encode a foc_parameters struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_parameters_t* foc_parameters)
{
    return _ttalink_foc_parameters_encode(dst_addr, src_addr, msg, foc_parameters, false);
}

/**
 * @brief Encode a foc_parameters struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_parameters_t* foc_parameters)
{
    return _ttalink_foc_parameters_encode(dst_addr, src_addr, msg, foc_parameters, true);
}


static inline uint16_t _ttalink_foc_parameters_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_parameters_t* foc_parameters, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_parameters_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_parameters->update_time, foc_parameters->cmd_rw, foc_parameters->parameters, foc_parameters->param1, foc_parameters->param2);
    }else{
        return ttalink_foc_parameters_pack_chan(dst_addr, src_addr, chan, msg, foc_parameters->update_time, foc_parameters->cmd_rw, foc_parameters->parameters, foc_parameters->param1, foc_parameters->param2);
    }
}

/**
 * @brief Encode a foc_parameters struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_parameters_t* foc_parameters)
{
    return _ttalink_foc_parameters_encode_chan(dst_addr, src_addr, chan, msg, foc_parameters, false);
}

/**
 * @brief Encode a foc_parameters struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_parameters_t* foc_parameters)
{
    return _ttalink_foc_parameters_encode_chan(dst_addr, src_addr, chan, msg, foc_parameters, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_parameters_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_PARAMETERS_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint16_t(buf, 6, param1);
    _tta_put_uint16_t(buf, 8, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS, buf, TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_CRC, nocrc);
#else
    ttalink_foc_parameters_t packet;
    packet.update_time = update_time;
    packet.cmd_rw = cmd_rw;
    packet.parameters = parameters;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS, (const char *)&packet, TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_parameters message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param cmd_rw Judging the direction of command,0 read 1 write
 * @param parameters Parameter type
 * @param param1 parameter data
 * @param param2 parameter data
 */
static inline void ttalink_foc_parameters_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2)
{
    _ttalink_foc_parameters_send(dst_addr, src_addr, chan, update_time, cmd_rw, parameters, param1, param2, false);
}

/**
 * @brief Send a foc_parameters message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param cmd_rw Judging the direction of command,0 read 1 write
 * @param parameters Parameter type
 * @param param1 parameter data
 * @param param2 parameter data
 */
static inline void ttalink_foc_parameters_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2)
{
    _ttalink_foc_parameters_send(dst_addr, src_addr, chan, update_time, cmd_rw, parameters, param1, param2, true);
}


static inline void _ttalink_foc_parameters_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_parameters_t* foc_parameters, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_parameters_send_nocrc(dst_addr, src_addr, chan, foc_parameters->update_time, foc_parameters->cmd_rw, foc_parameters->parameters, foc_parameters->param1, foc_parameters->param2);
    }else{
        ttalink_foc_parameters_send(dst_addr, src_addr, chan, foc_parameters->update_time, foc_parameters->cmd_rw, foc_parameters->parameters, foc_parameters->param1, foc_parameters->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS, (const char *)foc_parameters, TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_parameters message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_parameters_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_parameters_t* foc_parameters)
{
    _ttalink_foc_parameters_send_struct(dst_addr, src_addr, chan, foc_parameters, false);
}

/**
 * @brief Send a foc_parameters message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_parameters_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_parameters_t* foc_parameters)
{
    _ttalink_foc_parameters_send_struct(dst_addr, src_addr, chan, foc_parameters, true);
}

#if TTALINK_MSG_ID_FOC_PARAMETERS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_parameters_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, cmd_rw);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint16_t(buf, 6, param1);
    _tta_put_uint16_t(buf, 8, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS, buf, TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_CRC, nocrc);
#else
    ttalink_foc_parameters_t *packet = (ttalink_foc_parameters_t *)msgbuf;
    packet->update_time = update_time;
    packet->cmd_rw = cmd_rw;
    packet->parameters = parameters;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS, (const char *)packet, TTALINK_MSG_ID_FOC_PARAMETERS_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_parameters_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2)
{
    _ttalink_foc_parameters_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd_rw, parameters, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_parameters_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t cmd_rw, uint8_t parameters, uint16_t param1, uint16_t param2)
{
    _ttalink_foc_parameters_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, cmd_rw, parameters, param1, param2, true);
}
#endif

#endif

// MESSAGE FOC_PARAMETERS UNPACKING


/**
 * @brief Get field update_time from foc_parameters message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_parameters_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd_rw from foc_parameters message
 *
 * @return Judging the direction of command,0 read 1 write
 */
static inline uint8_t ttalink_foc_parameters_get_cmd_rw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field parameters from foc_parameters message
 *
 * @return Parameter type
 */
static inline uint8_t ttalink_foc_parameters_get_parameters(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field param1 from foc_parameters message
 *
 * @return parameter data
 */
static inline uint16_t ttalink_foc_parameters_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field param2 from foc_parameters message
 *
 * @return parameter data
 */
static inline uint16_t ttalink_foc_parameters_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Decode a foc_parameters message into a struct
 *
 * @param msg The message to decode
 * @param foc_parameters C-struct to decode the message contents into
 */
static inline void ttalink_foc_parameters_decode(const ttalink_message_t* msg, ttalink_foc_parameters_t* foc_parameters)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_parameters->update_time = ttalink_foc_parameters_get_update_time(msg);
    foc_parameters->cmd_rw = ttalink_foc_parameters_get_cmd_rw(msg);
    foc_parameters->parameters = ttalink_foc_parameters_get_parameters(msg);
    foc_parameters->param1 = ttalink_foc_parameters_get_param1(msg);
    foc_parameters->param2 = ttalink_foc_parameters_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_PARAMETERS_LEN? msg->len : TTALINK_MSG_ID_FOC_PARAMETERS_LEN;
        memset(foc_parameters, 0, TTALINK_MSG_ID_FOC_PARAMETERS_LEN);
    memcpy(foc_parameters, _TTA_PAYLOAD(msg), len);
#endif
}
