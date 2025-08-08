#pragma once
// MESSAGE ERROR_CODE PACKING

#define TTALINK_MSG_ID_ERROR_CODE 2254

TTAPACKED(
typedef struct __ttalink_error_code_t {
 uint8_t error_version; /*<   . .*/
 uint32_t class_error; /*<   . .*/
 uint16_t error_no[8]; /*<   . .*/
}) ttalink_error_code_t;

#define TTALINK_MSG_ID_ERROR_CODE_LEN 21
#define TTALINK_MSG_ID_ERROR_CODE_MIN_LEN 21
#define TTALINK_MSG_ID_2254_LEN 21
#define TTALINK_MSG_ID_2254_MIN_LEN 21

#define TTALINK_MSG_ID_ERROR_CODE_CRC 58
#define TTALINK_MSG_ID_2254_CRC 58

#define TTALINK_MSG_ERROR_CODE_FIELD_ERROR_NO_LEN 8

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ERROR_CODE { \
    2254, \
    "ERROR_CODE", \
    3, \
    {  { "error_version", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_error_code_t, error_version) }, \
         { "class_error", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_error_code_t, class_error) }, \
         { "error_no", NULL, TTALINK_TYPE_UINT16_T, 8, 5, offsetof(ttalink_error_code_t, error_no) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ERROR_CODE { \
    "ERROR_CODE", \
    3, \
    {  { "error_version", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_error_code_t, error_version) }, \
         { "class_error", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_error_code_t, class_error) }, \
         { "error_no", NULL, TTALINK_TYPE_UINT16_T, 8, 5, offsetof(ttalink_error_code_t, error_no) }, \
         } \
}
#endif


static inline uint16_t _ttalink_error_code_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t error_version, uint32_t class_error, const uint16_t *error_no, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ERROR_CODE_LEN];
    _tta_put_uint8_t(buf, 0, error_version);
    _tta_put_uint32_t(buf, 1, class_error);
    _tta_put_uint16_t_array(buf, 5, error_no, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ERROR_CODE_LEN);
#else
    ttalink_error_code_t packet;
    packet.error_version = error_version;
    packet.class_error = class_error;
    tta_array_memcpy(packet.error_no, error_no, sizeof(uint16_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ERROR_CODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ERROR_CODE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_ERROR_CODE_LEN, TTALINK_MSG_ID_ERROR_CODE_CRC, nocrc);
}

/**
 * @brief Pack a error_code message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param error_version   . .
 * @param class_error   . .
 * @param error_no   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_error_code_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t error_version, uint32_t class_error, const uint16_t *error_no)
{
    return _ttalink_error_code_pack(dst_addr, src_addr, msg,  error_version, class_error, error_no, false);
}

/**
 * @brief Pack a error_code message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param error_version   . .
 * @param class_error   . .
 * @param error_no   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_error_code_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t error_version, uint32_t class_error, const uint16_t *error_no)
{
    return _ttalink_error_code_pack(dst_addr, src_addr, msg,  error_version, class_error, error_no, true);
}


static inline uint16_t _ttalink_error_code_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t error_version,uint32_t class_error,const uint16_t *error_no, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ERROR_CODE_LEN];
    _tta_put_uint8_t(buf, 0, error_version);
    _tta_put_uint32_t(buf, 1, class_error);
    _tta_put_uint16_t_array(buf, 5, error_no, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ERROR_CODE_LEN);
#else
    ttalink_error_code_t packet;
    packet.error_version = error_version;
    packet.class_error = class_error;
    tta_array_memcpy(packet.error_no, error_no, sizeof(uint16_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ERROR_CODE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ERROR_CODE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_ERROR_CODE_LEN, TTALINK_MSG_ID_ERROR_CODE_CRC, nocrc);
}

/**
 * @brief Pack a error_code message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param error_version   . .
 * @param class_error   . .
 * @param error_no   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_error_code_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t error_version,uint32_t class_error,const uint16_t *error_no)
{
    return _ttalink_error_code_pack_chan(dst_addr, src_addr, chan, msg,  error_version, class_error, error_no, false);
}

/**
 * @brief Pack a error_code message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param error_version   . .
 * @param class_error   . .
 * @param error_no   . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_error_code_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t error_version,uint32_t class_error,const uint16_t *error_no)
{
    return _ttalink_error_code_pack_chan(dst_addr, src_addr, chan, msg,  error_version, class_error, error_no, true);
}


static inline uint16_t _ttalink_error_code_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_error_code_t* error_code, bool nocrc)
{
    if(nocrc){
        return ttalink_error_code_pack_nocrc(dst_addr, src_addr, msg, error_code->error_version, error_code->class_error, error_code->error_no);
    }else{
        return ttalink_error_code_pack(dst_addr, src_addr, msg, error_code->error_version, error_code->class_error, error_code->error_no);
    }
    
}

/**
 * @brief Encode a error_code struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_error_code_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_error_code_t* error_code)
{
    return _ttalink_error_code_encode(dst_addr, src_addr, msg, error_code, false);
}

/**
 * @brief Encode a error_code struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_error_code_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_error_code_t* error_code)
{
    return _ttalink_error_code_encode(dst_addr, src_addr, msg, error_code, true);
}


static inline uint16_t _ttalink_error_code_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_error_code_t* error_code, bool nocrc)
{
    if(nocrc){
        return ttalink_error_code_pack_chan_nocrc(dst_addr, src_addr, chan, msg, error_code->error_version, error_code->class_error, error_code->error_no);
    }else{
        return ttalink_error_code_pack_chan(dst_addr, src_addr, chan, msg, error_code->error_version, error_code->class_error, error_code->error_no);
    }
}

/**
 * @brief Encode a error_code struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_error_code_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_error_code_t* error_code)
{
    return _ttalink_error_code_encode_chan(dst_addr, src_addr, chan, msg, error_code, false);
}

/**
 * @brief Encode a error_code struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param error_code C-struct to read the message contents from
 */
static inline uint16_t ttalink_error_code_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_error_code_t* error_code)
{
    return _ttalink_error_code_encode_chan(dst_addr, src_addr, chan, msg, error_code, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_error_code_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t error_version, uint32_t class_error, const uint16_t *error_no, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ERROR_CODE_LEN];
    _tta_put_uint8_t(buf, 0, error_version);
    _tta_put_uint32_t(buf, 1, class_error);
    _tta_put_uint16_t_array(buf, 5, error_no, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ERROR_CODE, buf, TTALINK_MSG_ID_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_ERROR_CODE_LEN, TTALINK_MSG_ID_ERROR_CODE_CRC, nocrc);
#else
    ttalink_error_code_t packet;
    packet.error_version = error_version;
    packet.class_error = class_error;
    tta_array_memcpy(packet.error_no, error_no, sizeof(uint16_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ERROR_CODE, (const char *)&packet, TTALINK_MSG_ID_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_ERROR_CODE_LEN, TTALINK_MSG_ID_ERROR_CODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a error_code message
 * @param chan TTAlink channel to send the message
 *
 * @param error_version   . .
 * @param class_error   . .
 * @param error_no   . .
 */
static inline void ttalink_error_code_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t error_version, uint32_t class_error, const uint16_t *error_no)
{
    _ttalink_error_code_send(dst_addr, src_addr, chan, error_version, class_error, error_no, false);
}

/**
 * @brief Send a error_code message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param error_version   . .
 * @param class_error   . .
 * @param error_no   . .
 */
static inline void ttalink_error_code_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t error_version, uint32_t class_error, const uint16_t *error_no)
{
    _ttalink_error_code_send(dst_addr, src_addr, chan, error_version, class_error, error_no, true);
}


static inline void _ttalink_error_code_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_error_code_t* error_code, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_error_code_send_nocrc(dst_addr, src_addr, chan, error_code->error_version, error_code->class_error, error_code->error_no);
    }else{
        ttalink_error_code_send(dst_addr, src_addr, chan, error_code->error_version, error_code->class_error, error_code->error_no);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ERROR_CODE, (const char *)error_code, TTALINK_MSG_ID_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_ERROR_CODE_LEN, TTALINK_MSG_ID_ERROR_CODE_CRC, nocrc);
#endif
}

/**
 * @brief Send a error_code message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_error_code_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_error_code_t* error_code)
{
    _ttalink_error_code_send_struct(dst_addr, src_addr, chan, error_code, false);
}

/**
 * @brief Send a error_code message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_error_code_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_error_code_t* error_code)
{
    _ttalink_error_code_send_struct(dst_addr, src_addr, chan, error_code, true);
}

#if TTALINK_MSG_ID_ERROR_CODE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_error_code_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t error_version, uint32_t class_error, const uint16_t *error_no, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, error_version);
    _tta_put_uint32_t(buf, 1, class_error);
    _tta_put_uint16_t_array(buf, 5, error_no, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ERROR_CODE, buf, TTALINK_MSG_ID_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_ERROR_CODE_LEN, TTALINK_MSG_ID_ERROR_CODE_CRC, nocrc);
#else
    ttalink_error_code_t *packet = (ttalink_error_code_t *)msgbuf;
    packet->error_version = error_version;
    packet->class_error = class_error;
    tta_array_memcpy(packet->error_no, error_no, sizeof(uint16_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ERROR_CODE, (const char *)packet, TTALINK_MSG_ID_ERROR_CODE_MIN_LEN, TTALINK_MSG_ID_ERROR_CODE_LEN, TTALINK_MSG_ID_ERROR_CODE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_error_code_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t error_version, uint32_t class_error, const uint16_t *error_no)
{
    _ttalink_error_code_send_buf(dst_addr, src_addr, msgbuf, chan, error_version, class_error, error_no, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_error_code_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t error_version, uint32_t class_error, const uint16_t *error_no)
{
    _ttalink_error_code_send_buf(dst_addr, src_addr, msgbuf, chan, error_version, class_error, error_no, true);
}
#endif

#endif

// MESSAGE ERROR_CODE UNPACKING


/**
 * @brief Get field error_version from error_code message
 *
 * @return   . .
 */
static inline uint8_t ttalink_error_code_get_error_version(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field class_error from error_code message
 *
 * @return   . .
 */
static inline uint32_t ttalink_error_code_get_class_error(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  1);
}

/**
 * @brief Get field error_no from error_code message
 *
 * @return   . .
 */
static inline uint16_t ttalink_error_code_get_error_no(const ttalink_message_t* msg, uint16_t *error_no)
{
    return _TTA_RETURN_uint16_t_array(msg, error_no, 8,  5);
}

/**
 * @brief Decode a error_code message into a struct
 *
 * @param msg The message to decode
 * @param error_code C-struct to decode the message contents into
 */
static inline void ttalink_error_code_decode(const ttalink_message_t* msg, ttalink_error_code_t* error_code)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    error_code->error_version = ttalink_error_code_get_error_version(msg);
    error_code->class_error = ttalink_error_code_get_class_error(msg);
    ttalink_error_code_get_error_no(msg, error_code->error_no);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ERROR_CODE_LEN? msg->len : TTALINK_MSG_ID_ERROR_CODE_LEN;
        memset(error_code, 0, TTALINK_MSG_ID_ERROR_CODE_LEN);
    memcpy(error_code, _TTA_PAYLOAD(msg), len);
#endif
}
