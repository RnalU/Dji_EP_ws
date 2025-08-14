#pragma once
// MESSAGE PTZ_ERROR_CODE PACKING

#define TTALINK_MSG_ID_PTZ_ERROR_CODE 2307

TTAPACKED(
typedef struct __ttalink_ptz_error_code_t {
 uint32_t update_time; /*<  update_time*/
 uint8_t error; /*<  .*/
}) ttalink_ptz_error_code_t;

#define TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN 5
#define TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN 5
#define TTALINK_MSG_ID_2307_LEN 5
#define TTALINK_MSG_ID_2307_MIN_LEN 5

#define TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC 168
#define TTALINK_MSG_ID_2307_CRC 168



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PTZ_ERROR_CODE { \
    2307, \
    "PTZ_ERROR_CODE", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_error_code_t, update_time) }, \
         { "error", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_error_code_t, error) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PTZ_ERROR_CODE { \
    "PTZ_ERROR_CODE", \
    2, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_error_code_t, update_time) }, \
         { "error", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_ptz_error_code_t, error) }, \
         } \
}
#endif


static inline uint16_t _ttalink_ptz_error_code_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t error, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, error);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN);
#else
    ttalink_ptz_error_code_t packet;
    packet.update_time = update_time;
    packet.error = error;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_ERROR_CODE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC, nocrc);
}

/**
 * @brief Pack a ptz_error_code message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param error  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_error_code_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t error)
{
    return _ttalink_ptz_error_code_pack(dst_addr, src_addr, msg,  update_time, error, false);
}

/**
 * @brief Pack a ptz_error_code message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param error  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_error_code_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t error)
{
    return _ttalink_ptz_error_code_pack(dst_addr, src_addr, msg,  update_time, error, true);
}


static inline uint16_t _ttalink_ptz_error_code_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t error, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, error);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN);
#else
    ttalink_ptz_error_code_t packet;
    packet.update_time = update_time;
    packet.error = error;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_ERROR_CODE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC, nocrc);
}

/**
 * @brief Pack a ptz_error_code message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param error  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_error_code_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t error)
{
    return _ttalink_ptz_error_code_pack_chan(dst_addr, src_addr, chan, msg,  update_time, error, false);
}

/**
 * @brief Pack a ptz_error_code message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param error  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_error_code_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t error)
{
    return _ttalink_ptz_error_code_pack_chan(dst_addr, src_addr, chan, msg,  update_time, error, true);
}


static inline uint16_t _ttalink_ptz_error_code_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_error_code_t* ptz_error_code, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_error_code_pack_nocrc(dst_addr, src_addr, msg, ptz_error_code->update_time, ptz_error_code->error);
    }else{
        return ttalink_ptz_error_code_pack(dst_addr, src_addr, msg, ptz_error_code->update_time, ptz_error_code->error);
    }
    
}

/**
 * @brief Encode a ptz_error_code struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_error_code_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_error_code_t* ptz_error_code)
{
    return _ttalink_ptz_error_code_encode(dst_addr, src_addr, msg, ptz_error_code, false);
}

/**
 * @brief Encode a ptz_error_code struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_error_code_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_error_code_t* ptz_error_code)
{
    return _ttalink_ptz_error_code_encode(dst_addr, src_addr, msg, ptz_error_code, true);
}


static inline uint16_t _ttalink_ptz_error_code_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_error_code_t* ptz_error_code, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_error_code_pack_chan_nocrc(dst_addr, src_addr, chan, msg, ptz_error_code->update_time, ptz_error_code->error);
    }else{
        return ttalink_ptz_error_code_pack_chan(dst_addr, src_addr, chan, msg, ptz_error_code->update_time, ptz_error_code->error);
    }
}

/**
 * @brief Encode a ptz_error_code struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_error_code_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_error_code_t* ptz_error_code)
{
    return _ttalink_ptz_error_code_encode_chan(dst_addr, src_addr, chan, msg, ptz_error_code, false);
}

/**
 * @brief Encode a ptz_error_code struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_error_code_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_error_code_t* ptz_error_code)
{
    return _ttalink_ptz_error_code_encode_chan(dst_addr, src_addr, chan, msg, ptz_error_code, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_ptz_error_code_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t error, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, error);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ERROR_CODE, buf, TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC, nocrc);
#else
    ttalink_ptz_error_code_t packet;
    packet.update_time = update_time;
    packet.error = error;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ERROR_CODE, (const char *)&packet, TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_error_code message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param error  .
 */
static inline void ttalink_ptz_error_code_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t error)
{
    _ttalink_ptz_error_code_send(dst_addr, src_addr, chan, update_time, error, false);
}

/**
 * @brief Send a ptz_error_code message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param error  .
 */
static inline void ttalink_ptz_error_code_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t error)
{
    _ttalink_ptz_error_code_send(dst_addr, src_addr, chan, update_time, error, true);
}


static inline void _ttalink_ptz_error_code_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_error_code_t* ptz_error_code, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_ptz_error_code_send_nocrc(dst_addr, src_addr, chan, ptz_error_code->update_time, ptz_error_code->error);
    }else{
        ttalink_ptz_error_code_send(dst_addr, src_addr, chan, ptz_error_code->update_time, ptz_error_code->error);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ERROR_CODE, (const char *)ptz_error_code, TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_error_code message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_error_code_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_error_code_t* ptz_error_code)
{
    _ttalink_ptz_error_code_send_struct(dst_addr, src_addr, chan, ptz_error_code, false);
}

/**
 * @brief Send a ptz_error_code message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_error_code_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_error_code_t* ptz_error_code)
{
    _ttalink_ptz_error_code_send_struct(dst_addr, src_addr, chan, ptz_error_code, true);
}

#if TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_ptz_error_code_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t error, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, error);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ERROR_CODE, buf, TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC, nocrc);
#else
    ttalink_ptz_error_code_t *packet = (ttalink_ptz_error_code_t *)msgbuf;
    packet->update_time = update_time;
    packet->error = error;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ERROR_CODE, (const char *)packet, TTALINK_MSG_ID_PTZ_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN, TTALINK_MSG_ID_PTZ_ERROR_CODE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_ptz_error_code_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t error)
{
    _ttalink_ptz_error_code_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, error, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_ptz_error_code_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t error)
{
    _ttalink_ptz_error_code_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, error, true);
}
#endif

#endif

// MESSAGE PTZ_ERROR_CODE UNPACKING


/**
 * @brief Get field update_time from ptz_error_code message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_ptz_error_code_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field error from ptz_error_code message
 *
 * @return  .
 */
static inline uint8_t ttalink_ptz_error_code_get_error(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a ptz_error_code message into a struct
 *
 * @param msg The message to decode
 * @param ptz_error_code C-struct to decode the message contents into
 */
static inline void ttalink_ptz_error_code_decode(const ttalink_message_t* msg, ttalink_ptz_error_code_t* ptz_error_code)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ptz_error_code->update_time = ttalink_ptz_error_code_get_update_time(msg);
    ptz_error_code->error = ttalink_ptz_error_code_get_error(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN? msg->len : TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN;
        memset(ptz_error_code, 0, TTALINK_MSG_ID_PTZ_ERROR_CODE_LEN);
    memcpy(ptz_error_code, _TTA_PAYLOAD(msg), len);
#endif
}
