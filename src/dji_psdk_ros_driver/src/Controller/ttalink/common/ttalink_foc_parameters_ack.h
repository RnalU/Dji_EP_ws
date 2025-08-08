#pragma once
// MESSAGE FOC_PARAMETERS_ACK PACKING

#define TTALINK_MSG_ID_FOC_PARAMETERS_ACK 2015

TTAPACKED(
typedef struct __ttalink_foc_parameters_ack_t {
 uint32_t update_time; /*< system time*/
 uint8_t ack; /*< feedback configuration results.*/
 uint8_t parameters; /*< Parameter type*/
 uint32_t data; /*< parameter data*/
}) ttalink_foc_parameters_ack_t;

#define TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN 10
#define TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN 10
#define TTALINK_MSG_ID_2015_LEN 10
#define TTALINK_MSG_ID_2015_MIN_LEN 10

#define TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC 148
#define TTALINK_MSG_ID_2015_CRC 148



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_PARAMETERS_ACK { \
    2015, \
    "FOC_PARAMETERS_ACK", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_parameters_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_parameters_ack_t, ack) }, \
         { "parameters", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_foc_parameters_ack_t, parameters) }, \
         { "data", NULL, TTALINK_TYPE_UINT32_T, 0, 6, offsetof(ttalink_foc_parameters_ack_t, data) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_PARAMETERS_ACK { \
    "FOC_PARAMETERS_ACK", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_parameters_ack_t, update_time) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_parameters_ack_t, ack) }, \
         { "parameters", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_foc_parameters_ack_t, parameters) }, \
         { "data", NULL, TTALINK_TYPE_UINT32_T, 0, 6, offsetof(ttalink_foc_parameters_ack_t, data) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_parameters_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint32_t(buf, 6, data);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN);
#else
    ttalink_foc_parameters_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.parameters = parameters;
    packet.data = data;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_PARAMETERS_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a foc_parameters_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param ack feedback configuration results.
 * @param parameters Parameter type
 * @param data parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data)
{
    return _ttalink_foc_parameters_ack_pack(dst_addr, src_addr, msg,  update_time, ack, parameters, data, false);
}

/**
 * @brief Pack a foc_parameters_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param ack feedback configuration results.
 * @param parameters Parameter type
 * @param data parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data)
{
    return _ttalink_foc_parameters_ack_pack(dst_addr, src_addr, msg,  update_time, ack, parameters, data, true);
}


static inline uint16_t _ttalink_foc_parameters_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,uint8_t parameters,uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint32_t(buf, 6, data);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN);
#else
    ttalink_foc_parameters_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.parameters = parameters;
    packet.data = data;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_PARAMETERS_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC, nocrc);
}

/**
 * @brief Pack a foc_parameters_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param ack feedback configuration results.
 * @param parameters Parameter type
 * @param data parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,uint8_t parameters,uint32_t data)
{
    return _ttalink_foc_parameters_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, parameters, data, false);
}

/**
 * @brief Pack a foc_parameters_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param ack feedback configuration results.
 * @param parameters Parameter type
 * @param data parameter data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_parameters_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t ack,uint8_t parameters,uint32_t data)
{
    return _ttalink_foc_parameters_ack_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ack, parameters, data, true);
}


static inline uint16_t _ttalink_foc_parameters_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_parameters_ack_t* foc_parameters_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_parameters_ack_pack_nocrc(dst_addr, src_addr, msg, foc_parameters_ack->update_time, foc_parameters_ack->ack, foc_parameters_ack->parameters, foc_parameters_ack->data);
    }else{
        return ttalink_foc_parameters_ack_pack(dst_addr, src_addr, msg, foc_parameters_ack->update_time, foc_parameters_ack->ack, foc_parameters_ack->parameters, foc_parameters_ack->data);
    }
    
}

/**
 * @brief Encode a foc_parameters_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_parameters_ack_t* foc_parameters_ack)
{
    return _ttalink_foc_parameters_ack_encode(dst_addr, src_addr, msg, foc_parameters_ack, false);
}

/**
 * @brief Encode a foc_parameters_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_parameters_ack_t* foc_parameters_ack)
{
    return _ttalink_foc_parameters_ack_encode(dst_addr, src_addr, msg, foc_parameters_ack, true);
}


static inline uint16_t _ttalink_foc_parameters_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_parameters_ack_t* foc_parameters_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_parameters_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_parameters_ack->update_time, foc_parameters_ack->ack, foc_parameters_ack->parameters, foc_parameters_ack->data);
    }else{
        return ttalink_foc_parameters_ack_pack_chan(dst_addr, src_addr, chan, msg, foc_parameters_ack->update_time, foc_parameters_ack->ack, foc_parameters_ack->parameters, foc_parameters_ack->data);
    }
}

/**
 * @brief Encode a foc_parameters_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_parameters_ack_t* foc_parameters_ack)
{
    return _ttalink_foc_parameters_ack_encode_chan(dst_addr, src_addr, chan, msg, foc_parameters_ack, false);
}

/**
 * @brief Encode a foc_parameters_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_parameters_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_parameters_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_parameters_ack_t* foc_parameters_ack)
{
    return _ttalink_foc_parameters_ack_encode_chan(dst_addr, src_addr, chan, msg, foc_parameters_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_parameters_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint32_t(buf, 6, data);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS_ACK, buf, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC, nocrc);
#else
    ttalink_foc_parameters_ack_t packet;
    packet.update_time = update_time;
    packet.ack = ack;
    packet.parameters = parameters;
    packet.data = data;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS_ACK, (const char *)&packet, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_parameters_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param ack feedback configuration results.
 * @param parameters Parameter type
 * @param data parameter data
 */
static inline void ttalink_foc_parameters_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data)
{
    _ttalink_foc_parameters_ack_send(dst_addr, src_addr, chan, update_time, ack, parameters, data, false);
}

/**
 * @brief Send a foc_parameters_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param ack feedback configuration results.
 * @param parameters Parameter type
 * @param data parameter data
 */
static inline void ttalink_foc_parameters_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data)
{
    _ttalink_foc_parameters_ack_send(dst_addr, src_addr, chan, update_time, ack, parameters, data, true);
}


static inline void _ttalink_foc_parameters_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_parameters_ack_t* foc_parameters_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_parameters_ack_send_nocrc(dst_addr, src_addr, chan, foc_parameters_ack->update_time, foc_parameters_ack->ack, foc_parameters_ack->parameters, foc_parameters_ack->data);
    }else{
        ttalink_foc_parameters_ack_send(dst_addr, src_addr, chan, foc_parameters_ack->update_time, foc_parameters_ack->ack, foc_parameters_ack->parameters, foc_parameters_ack->data);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS_ACK, (const char *)foc_parameters_ack, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_parameters_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_parameters_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_parameters_ack_t* foc_parameters_ack)
{
    _ttalink_foc_parameters_ack_send_struct(dst_addr, src_addr, chan, foc_parameters_ack, false);
}

/**
 * @brief Send a foc_parameters_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_parameters_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_parameters_ack_t* foc_parameters_ack)
{
    _ttalink_foc_parameters_ack_send_struct(dst_addr, src_addr, chan, foc_parameters_ack, true);
}

#if TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_parameters_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, ack);
    _tta_put_uint8_t(buf, 5, parameters);
    _tta_put_uint32_t(buf, 6, data);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS_ACK, buf, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC, nocrc);
#else
    ttalink_foc_parameters_ack_t *packet = (ttalink_foc_parameters_ack_t *)msgbuf;
    packet->update_time = update_time;
    packet->ack = ack;
    packet->parameters = parameters;
    packet->data = data;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_PARAMETERS_ACK, (const char *)packet, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_MIN_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_parameters_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data)
{
    _ttalink_foc_parameters_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, parameters, data, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_parameters_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t ack, uint8_t parameters, uint32_t data)
{
    _ttalink_foc_parameters_ack_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ack, parameters, data, true);
}
#endif

#endif

// MESSAGE FOC_PARAMETERS_ACK UNPACKING


/**
 * @brief Get field update_time from foc_parameters_ack message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_parameters_ack_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ack from foc_parameters_ack message
 *
 * @return feedback configuration results.
 */
static inline uint8_t ttalink_foc_parameters_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field parameters from foc_parameters_ack message
 *
 * @return Parameter type
 */
static inline uint8_t ttalink_foc_parameters_ack_get_parameters(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field data from foc_parameters_ack message
 *
 * @return parameter data
 */
static inline uint32_t ttalink_foc_parameters_ack_get_data(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  6);
}

/**
 * @brief Decode a foc_parameters_ack message into a struct
 *
 * @param msg The message to decode
 * @param foc_parameters_ack C-struct to decode the message contents into
 */
static inline void ttalink_foc_parameters_ack_decode(const ttalink_message_t* msg, ttalink_foc_parameters_ack_t* foc_parameters_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_parameters_ack->update_time = ttalink_foc_parameters_ack_get_update_time(msg);
    foc_parameters_ack->ack = ttalink_foc_parameters_ack_get_ack(msg);
    foc_parameters_ack->parameters = ttalink_foc_parameters_ack_get_parameters(msg);
    foc_parameters_ack->data = ttalink_foc_parameters_ack_get_data(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN? msg->len : TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN;
        memset(foc_parameters_ack, 0, TTALINK_MSG_ID_FOC_PARAMETERS_ACK_LEN);
    memcpy(foc_parameters_ack, _TTA_PAYLOAD(msg), len);
#endif
}
