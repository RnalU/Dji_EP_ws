#pragma once
// MESSAGE BMS_INFO_CHECK PACKING

#define TTALINK_MSG_ID_BMS_INFO_CHECK 2053

TTAPACKED(
typedef struct __ttalink_bms_info_check_t {
 uint32_t update_time; /*<  . .*/
}) ttalink_bms_info_check_t;

#define TTALINK_MSG_ID_BMS_INFO_CHECK_LEN 4
#define TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN 4
#define TTALINK_MSG_ID_2053_LEN 4
#define TTALINK_MSG_ID_2053_MIN_LEN 4

#define TTALINK_MSG_ID_BMS_INFO_CHECK_CRC 103
#define TTALINK_MSG_ID_2053_CRC 103



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_BMS_INFO_CHECK { \
    2053, \
    "BMS_INFO_CHECK", \
    1, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_info_check_t, update_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_BMS_INFO_CHECK { \
    "BMS_INFO_CHECK", \
    1, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_bms_info_check_t, update_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_bms_info_check_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_INFO_CHECK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN);
#else
    ttalink_bms_info_check_t packet;
    packet.update_time = update_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_INFO_CHECK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_CRC, nocrc);
}

/**
 * @brief Pack a bms_info_check message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_check_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time)
{
    return _ttalink_bms_info_check_pack(dst_addr, src_addr, msg,  update_time, false);
}

/**
 * @brief Pack a bms_info_check message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_check_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time)
{
    return _ttalink_bms_info_check_pack(dst_addr, src_addr, msg,  update_time, true);
}


static inline uint16_t _ttalink_bms_info_check_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_INFO_CHECK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN);
#else
    ttalink_bms_info_check_t packet;
    packet.update_time = update_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_BMS_INFO_CHECK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_CRC, nocrc);
}

/**
 * @brief Pack a bms_info_check message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_check_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time)
{
    return _ttalink_bms_info_check_pack_chan(dst_addr, src_addr, chan, msg,  update_time, false);
}

/**
 * @brief Pack a bms_info_check message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_bms_info_check_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time)
{
    return _ttalink_bms_info_check_pack_chan(dst_addr, src_addr, chan, msg,  update_time, true);
}


static inline uint16_t _ttalink_bms_info_check_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_info_check_t* bms_info_check, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_info_check_pack_nocrc(dst_addr, src_addr, msg, bms_info_check->update_time);
    }else{
        return ttalink_bms_info_check_pack(dst_addr, src_addr, msg, bms_info_check->update_time);
    }
    
}

/**
 * @brief Encode a bms_info_check struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_info_check C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_check_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_info_check_t* bms_info_check)
{
    return _ttalink_bms_info_check_encode(dst_addr, src_addr, msg, bms_info_check, false);
}

/**
 * @brief Encode a bms_info_check struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param bms_info_check C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_check_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_bms_info_check_t* bms_info_check)
{
    return _ttalink_bms_info_check_encode(dst_addr, src_addr, msg, bms_info_check, true);
}


static inline uint16_t _ttalink_bms_info_check_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_info_check_t* bms_info_check, bool nocrc)
{
    if(nocrc){
        return ttalink_bms_info_check_pack_chan_nocrc(dst_addr, src_addr, chan, msg, bms_info_check->update_time);
    }else{
        return ttalink_bms_info_check_pack_chan(dst_addr, src_addr, chan, msg, bms_info_check->update_time);
    }
}

/**
 * @brief Encode a bms_info_check struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_info_check C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_check_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_info_check_t* bms_info_check)
{
    return _ttalink_bms_info_check_encode_chan(dst_addr, src_addr, chan, msg, bms_info_check, false);
}

/**
 * @brief Encode a bms_info_check struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param bms_info_check C-struct to read the message contents from
 */
static inline uint16_t ttalink_bms_info_check_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_bms_info_check_t* bms_info_check)
{
    return _ttalink_bms_info_check_encode_chan(dst_addr, src_addr, chan, msg, bms_info_check, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_bms_info_check_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_BMS_INFO_CHECK_LEN];
    _tta_put_uint32_t(buf, 0, update_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO_CHECK, buf, TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_CRC, nocrc);
#else
    ttalink_bms_info_check_t packet;
    packet.update_time = update_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO_CHECK, (const char *)&packet, TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_info_check message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 */
static inline void ttalink_bms_info_check_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time)
{
    _ttalink_bms_info_check_send(dst_addr, src_addr, chan, update_time, false);
}

/**
 * @brief Send a bms_info_check message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  . .
 */
static inline void ttalink_bms_info_check_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time)
{
    _ttalink_bms_info_check_send(dst_addr, src_addr, chan, update_time, true);
}


static inline void _ttalink_bms_info_check_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_info_check_t* bms_info_check, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_bms_info_check_send_nocrc(dst_addr, src_addr, chan, bms_info_check->update_time);
    }else{
        ttalink_bms_info_check_send(dst_addr, src_addr, chan, bms_info_check->update_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO_CHECK, (const char *)bms_info_check, TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_CRC, nocrc);
#endif
}

/**
 * @brief Send a bms_info_check message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_info_check_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_info_check_t* bms_info_check)
{
    _ttalink_bms_info_check_send_struct(dst_addr, src_addr, chan, bms_info_check, false);
}

/**
 * @brief Send a bms_info_check message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_bms_info_check_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_bms_info_check_t* bms_info_check)
{
    _ttalink_bms_info_check_send_struct(dst_addr, src_addr, chan, bms_info_check, true);
}

#if TTALINK_MSG_ID_BMS_INFO_CHECK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_bms_info_check_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO_CHECK, buf, TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_CRC, nocrc);
#else
    ttalink_bms_info_check_t *packet = (ttalink_bms_info_check_t *)msgbuf;
    packet->update_time = update_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_BMS_INFO_CHECK, (const char *)packet, TTALINK_MSG_ID_BMS_INFO_CHECK_MIN_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN, TTALINK_MSG_ID_BMS_INFO_CHECK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_bms_info_check_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time)
{
    _ttalink_bms_info_check_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_bms_info_check_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time)
{
    _ttalink_bms_info_check_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, true);
}
#endif

#endif

// MESSAGE BMS_INFO_CHECK UNPACKING


/**
 * @brief Get field update_time from bms_info_check message
 *
 * @return  . .
 */
static inline uint32_t ttalink_bms_info_check_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a bms_info_check message into a struct
 *
 * @param msg The message to decode
 * @param bms_info_check C-struct to decode the message contents into
 */
static inline void ttalink_bms_info_check_decode(const ttalink_message_t* msg, ttalink_bms_info_check_t* bms_info_check)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    bms_info_check->update_time = ttalink_bms_info_check_get_update_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_BMS_INFO_CHECK_LEN? msg->len : TTALINK_MSG_ID_BMS_INFO_CHECK_LEN;
        memset(bms_info_check, 0, TTALINK_MSG_ID_BMS_INFO_CHECK_LEN);
    memcpy(bms_info_check, _TTA_PAYLOAD(msg), len);
#endif
}
