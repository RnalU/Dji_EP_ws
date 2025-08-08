#pragma once
// MESSAGE MAV_HIL_SYSTIME PACKING

#define TTALINK_MSG_ID_MAV_HIL_SYSTIME 8209

TTAPACKED(
typedef struct __ttalink_mav_hil_systime_t {
 uint64_t time_unix_usec; /*< Timestamp of the master clock in microseconds since UNIX epoch.*/
 uint32_t time_boot_ms; /*< Timestamp of the component clock since boot time in milliseconds.*/
}) ttalink_mav_hil_systime_t;

#define TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN 12
#define TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN 12
#define TTALINK_MSG_ID_8209_LEN 12
#define TTALINK_MSG_ID_8209_MIN_LEN 12

#define TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC 81
#define TTALINK_MSG_ID_8209_CRC 81



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAV_HIL_SYSTIME { \
    8209, \
    "MAV_HIL_SYSTIME", \
    2, \
    {  { "time_unix_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_systime_t, time_unix_usec) }, \
         { "time_boot_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_mav_hil_systime_t, time_boot_ms) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAV_HIL_SYSTIME { \
    "MAV_HIL_SYSTIME", \
    2, \
    {  { "time_unix_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_systime_t, time_unix_usec) }, \
         { "time_boot_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_mav_hil_systime_t, time_boot_ms) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mav_hil_systime_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_unix_usec, uint32_t time_boot_ms, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN];
    _tta_put_uint64_t(buf, 0, time_unix_usec);
    _tta_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN);
#else
    ttalink_mav_hil_systime_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_SYSTIME;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_systime message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_systime_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    return _ttalink_mav_hil_systime_pack(dst_addr, src_addr, msg,  time_unix_usec, time_boot_ms, false);
}

/**
 * @brief Pack a mav_hil_systime message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_systime_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    return _ttalink_mav_hil_systime_pack(dst_addr, src_addr, msg,  time_unix_usec, time_boot_ms, true);
}


static inline uint16_t _ttalink_mav_hil_systime_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_unix_usec,uint32_t time_boot_ms, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN];
    _tta_put_uint64_t(buf, 0, time_unix_usec);
    _tta_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN);
#else
    ttalink_mav_hil_systime_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_SYSTIME;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_systime message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_systime_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_unix_usec,uint32_t time_boot_ms)
{
    return _ttalink_mav_hil_systime_pack_chan(dst_addr, src_addr, chan, msg,  time_unix_usec, time_boot_ms, false);
}

/**
 * @brief Pack a mav_hil_systime message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_systime_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_unix_usec,uint32_t time_boot_ms)
{
    return _ttalink_mav_hil_systime_pack_chan(dst_addr, src_addr, chan, msg,  time_unix_usec, time_boot_ms, true);
}


static inline uint16_t _ttalink_mav_hil_systime_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_systime_t* mav_hil_systime, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_systime_pack_nocrc(dst_addr, src_addr, msg, mav_hil_systime->time_unix_usec, mav_hil_systime->time_boot_ms);
    }else{
        return ttalink_mav_hil_systime_pack(dst_addr, src_addr, msg, mav_hil_systime->time_unix_usec, mav_hil_systime->time_boot_ms);
    }
    
}

/**
 * @brief Encode a mav_hil_systime struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_systime C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_systime_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_systime_t* mav_hil_systime)
{
    return _ttalink_mav_hil_systime_encode(dst_addr, src_addr, msg, mav_hil_systime, false);
}

/**
 * @brief Encode a mav_hil_systime struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_systime C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_systime_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_systime_t* mav_hil_systime)
{
    return _ttalink_mav_hil_systime_encode(dst_addr, src_addr, msg, mav_hil_systime, true);
}


static inline uint16_t _ttalink_mav_hil_systime_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_systime_t* mav_hil_systime, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_systime_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mav_hil_systime->time_unix_usec, mav_hil_systime->time_boot_ms);
    }else{
        return ttalink_mav_hil_systime_pack_chan(dst_addr, src_addr, chan, msg, mav_hil_systime->time_unix_usec, mav_hil_systime->time_boot_ms);
    }
}

/**
 * @brief Encode a mav_hil_systime struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_systime C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_systime_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_systime_t* mav_hil_systime)
{
    return _ttalink_mav_hil_systime_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_systime, false);
}

/**
 * @brief Encode a mav_hil_systime struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_systime C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_systime_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_systime_t* mav_hil_systime)
{
    return _ttalink_mav_hil_systime_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_systime, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mav_hil_systime_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_unix_usec, uint32_t time_boot_ms, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN];
    _tta_put_uint64_t(buf, 0, time_unix_usec);
    _tta_put_uint32_t(buf, 8, time_boot_ms);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SYSTIME, buf, TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC, nocrc);
#else
    ttalink_mav_hil_systime_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SYSTIME, (const char *)&packet, TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_systime message
 * @param chan TTAlink channel to send the message
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 */
static inline void ttalink_mav_hil_systime_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    _ttalink_mav_hil_systime_send(dst_addr, src_addr, chan, time_unix_usec, time_boot_ms, false);
}

/**
 * @brief Send a mav_hil_systime message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 */
static inline void ttalink_mav_hil_systime_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    _ttalink_mav_hil_systime_send(dst_addr, src_addr, chan, time_unix_usec, time_boot_ms, true);
}


static inline void _ttalink_mav_hil_systime_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_systime_t* mav_hil_systime, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mav_hil_systime_send_nocrc(dst_addr, src_addr, chan, mav_hil_systime->time_unix_usec, mav_hil_systime->time_boot_ms);
    }else{
        ttalink_mav_hil_systime_send(dst_addr, src_addr, chan, mav_hil_systime->time_unix_usec, mav_hil_systime->time_boot_ms);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SYSTIME, (const char *)mav_hil_systime, TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_systime message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_systime_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_systime_t* mav_hil_systime)
{
    _ttalink_mav_hil_systime_send_struct(dst_addr, src_addr, chan, mav_hil_systime, false);
}

/**
 * @brief Send a mav_hil_systime message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_systime_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_systime_t* mav_hil_systime)
{
    _ttalink_mav_hil_systime_send_struct(dst_addr, src_addr, chan, mav_hil_systime, true);
}

#if TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mav_hil_systime_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_unix_usec, uint32_t time_boot_ms, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, time_unix_usec);
    _tta_put_uint32_t(buf, 8, time_boot_ms);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SYSTIME, buf, TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC, nocrc);
#else
    ttalink_mav_hil_systime_t *packet = (ttalink_mav_hil_systime_t *)msgbuf;
    packet->time_unix_usec = time_unix_usec;
    packet->time_boot_ms = time_boot_ms;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_SYSTIME, (const char *)packet, TTALINK_MSG_ID_MAV_HIL_SYSTIME_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN, TTALINK_MSG_ID_MAV_HIL_SYSTIME_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mav_hil_systime_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    _ttalink_mav_hil_systime_send_buf(dst_addr, src_addr, msgbuf, chan, time_unix_usec, time_boot_ms, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mav_hil_systime_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    _ttalink_mav_hil_systime_send_buf(dst_addr, src_addr, msgbuf, chan, time_unix_usec, time_boot_ms, true);
}
#endif

#endif

// MESSAGE MAV_HIL_SYSTIME UNPACKING


/**
 * @brief Get field time_unix_usec from mav_hil_systime message
 *
 * @return Timestamp of the master clock in microseconds since UNIX epoch.
 */
static inline uint64_t ttalink_mav_hil_systime_get_time_unix_usec(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_boot_ms from mav_hil_systime message
 *
 * @return Timestamp of the component clock since boot time in milliseconds.
 */
static inline uint32_t ttalink_mav_hil_systime_get_time_boot_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a mav_hil_systime message into a struct
 *
 * @param msg The message to decode
 * @param mav_hil_systime C-struct to decode the message contents into
 */
static inline void ttalink_mav_hil_systime_decode(const ttalink_message_t* msg, ttalink_mav_hil_systime_t* mav_hil_systime)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mav_hil_systime->time_unix_usec = ttalink_mav_hil_systime_get_time_unix_usec(msg);
    mav_hil_systime->time_boot_ms = ttalink_mav_hil_systime_get_time_boot_ms(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN? msg->len : TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN;
        memset(mav_hil_systime, 0, TTALINK_MSG_ID_MAV_HIL_SYSTIME_LEN);
    memcpy(mav_hil_systime, _TTA_PAYLOAD(msg), len);
#endif
}
