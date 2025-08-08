#pragma once
// MESSAGE PTZ_ATTI PACKING

#define TTALINK_MSG_ID_PTZ_ATTI 2300

TTAPACKED(
typedef struct __ttalink_ptz_atti_t {
 uint32_t update_time; /*<  update_time*/
 float pitch; /*<  FC_Pitch_Ptz*/
 float roll; /*<  FC_Roll_Ptz*/
 float yaw; /*<  FC_Yaw_Ptz*/
}) ttalink_ptz_atti_t;

#define TTALINK_MSG_ID_PTZ_ATTI_LEN 16
#define TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN 16
#define TTALINK_MSG_ID_2300_LEN 16
#define TTALINK_MSG_ID_2300_MIN_LEN 16

#define TTALINK_MSG_ID_PTZ_ATTI_CRC 252
#define TTALINK_MSG_ID_2300_CRC 252



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PTZ_ATTI { \
    2300, \
    "PTZ_ATTI", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_atti_t, update_time) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_ptz_atti_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_ptz_atti_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_ptz_atti_t, yaw) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PTZ_ATTI { \
    "PTZ_ATTI", \
    4, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_ptz_atti_t, update_time) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_ptz_atti_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_ptz_atti_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_ptz_atti_t, yaw) }, \
         } \
}
#endif


static inline uint16_t _ttalink_ptz_atti_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float pitch, float roll, float yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_ATTI_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, pitch);
    _tta_put_float(buf, 8, roll);
    _tta_put_float(buf, 12, yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_ATTI_LEN);
#else
    ttalink_ptz_atti_t packet;
    packet.update_time = update_time;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_ATTI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_ATTI;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN, TTALINK_MSG_ID_PTZ_ATTI_LEN, TTALINK_MSG_ID_PTZ_ATTI_CRC, nocrc);
}

/**
 * @brief Pack a ptz_atti message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param pitch  FC_Pitch_Ptz
 * @param roll  FC_Roll_Ptz
 * @param yaw  FC_Yaw_Ptz
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_atti_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float pitch, float roll, float yaw)
{
    return _ttalink_ptz_atti_pack(dst_addr, src_addr, msg,  update_time, pitch, roll, yaw, false);
}

/**
 * @brief Pack a ptz_atti message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param pitch  FC_Pitch_Ptz
 * @param roll  FC_Roll_Ptz
 * @param yaw  FC_Yaw_Ptz
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_atti_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float pitch, float roll, float yaw)
{
    return _ttalink_ptz_atti_pack(dst_addr, src_addr, msg,  update_time, pitch, roll, yaw, true);
}


static inline uint16_t _ttalink_ptz_atti_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float pitch,float roll,float yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_ATTI_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, pitch);
    _tta_put_float(buf, 8, roll);
    _tta_put_float(buf, 12, yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PTZ_ATTI_LEN);
#else
    ttalink_ptz_atti_t packet;
    packet.update_time = update_time;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PTZ_ATTI_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PTZ_ATTI;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN, TTALINK_MSG_ID_PTZ_ATTI_LEN, TTALINK_MSG_ID_PTZ_ATTI_CRC, nocrc);
}

/**
 * @brief Pack a ptz_atti message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param pitch  FC_Pitch_Ptz
 * @param roll  FC_Roll_Ptz
 * @param yaw  FC_Yaw_Ptz
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_atti_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float pitch,float roll,float yaw)
{
    return _ttalink_ptz_atti_pack_chan(dst_addr, src_addr, chan, msg,  update_time, pitch, roll, yaw, false);
}

/**
 * @brief Pack a ptz_atti message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param pitch  FC_Pitch_Ptz
 * @param roll  FC_Roll_Ptz
 * @param yaw  FC_Yaw_Ptz
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ptz_atti_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float pitch,float roll,float yaw)
{
    return _ttalink_ptz_atti_pack_chan(dst_addr, src_addr, chan, msg,  update_time, pitch, roll, yaw, true);
}


static inline uint16_t _ttalink_ptz_atti_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_atti_t* ptz_atti, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_atti_pack_nocrc(dst_addr, src_addr, msg, ptz_atti->update_time, ptz_atti->pitch, ptz_atti->roll, ptz_atti->yaw);
    }else{
        return ttalink_ptz_atti_pack(dst_addr, src_addr, msg, ptz_atti->update_time, ptz_atti->pitch, ptz_atti->roll, ptz_atti->yaw);
    }
    
}

/**
 * @brief Encode a ptz_atti struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_atti_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_atti_t* ptz_atti)
{
    return _ttalink_ptz_atti_encode(dst_addr, src_addr, msg, ptz_atti, false);
}

/**
 * @brief Encode a ptz_atti struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ptz_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_atti_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ptz_atti_t* ptz_atti)
{
    return _ttalink_ptz_atti_encode(dst_addr, src_addr, msg, ptz_atti, true);
}


static inline uint16_t _ttalink_ptz_atti_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_atti_t* ptz_atti, bool nocrc)
{
    if(nocrc){
        return ttalink_ptz_atti_pack_chan_nocrc(dst_addr, src_addr, chan, msg, ptz_atti->update_time, ptz_atti->pitch, ptz_atti->roll, ptz_atti->yaw);
    }else{
        return ttalink_ptz_atti_pack_chan(dst_addr, src_addr, chan, msg, ptz_atti->update_time, ptz_atti->pitch, ptz_atti->roll, ptz_atti->yaw);
    }
}

/**
 * @brief Encode a ptz_atti struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_atti_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_atti_t* ptz_atti)
{
    return _ttalink_ptz_atti_encode_chan(dst_addr, src_addr, chan, msg, ptz_atti, false);
}

/**
 * @brief Encode a ptz_atti struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ptz_atti C-struct to read the message contents from
 */
static inline uint16_t ttalink_ptz_atti_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ptz_atti_t* ptz_atti)
{
    return _ttalink_ptz_atti_encode_chan(dst_addr, src_addr, chan, msg, ptz_atti, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_ptz_atti_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float pitch, float roll, float yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PTZ_ATTI_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, pitch);
    _tta_put_float(buf, 8, roll);
    _tta_put_float(buf, 12, yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ATTI, buf, TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN, TTALINK_MSG_ID_PTZ_ATTI_LEN, TTALINK_MSG_ID_PTZ_ATTI_CRC, nocrc);
#else
    ttalink_ptz_atti_t packet;
    packet.update_time = update_time;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ATTI, (const char *)&packet, TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN, TTALINK_MSG_ID_PTZ_ATTI_LEN, TTALINK_MSG_ID_PTZ_ATTI_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_atti message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param pitch  FC_Pitch_Ptz
 * @param roll  FC_Roll_Ptz
 * @param yaw  FC_Yaw_Ptz
 */
static inline void ttalink_ptz_atti_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float pitch, float roll, float yaw)
{
    _ttalink_ptz_atti_send(dst_addr, src_addr, chan, update_time, pitch, roll, yaw, false);
}

/**
 * @brief Send a ptz_atti message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param pitch  FC_Pitch_Ptz
 * @param roll  FC_Roll_Ptz
 * @param yaw  FC_Yaw_Ptz
 */
static inline void ttalink_ptz_atti_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float pitch, float roll, float yaw)
{
    _ttalink_ptz_atti_send(dst_addr, src_addr, chan, update_time, pitch, roll, yaw, true);
}


static inline void _ttalink_ptz_atti_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_atti_t* ptz_atti, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_ptz_atti_send_nocrc(dst_addr, src_addr, chan, ptz_atti->update_time, ptz_atti->pitch, ptz_atti->roll, ptz_atti->yaw);
    }else{
        ttalink_ptz_atti_send(dst_addr, src_addr, chan, ptz_atti->update_time, ptz_atti->pitch, ptz_atti->roll, ptz_atti->yaw);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ATTI, (const char *)ptz_atti, TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN, TTALINK_MSG_ID_PTZ_ATTI_LEN, TTALINK_MSG_ID_PTZ_ATTI_CRC, nocrc);
#endif
}

/**
 * @brief Send a ptz_atti message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_atti_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_atti_t* ptz_atti)
{
    _ttalink_ptz_atti_send_struct(dst_addr, src_addr, chan, ptz_atti, false);
}

/**
 * @brief Send a ptz_atti message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ptz_atti_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ptz_atti_t* ptz_atti)
{
    _ttalink_ptz_atti_send_struct(dst_addr, src_addr, chan, ptz_atti, true);
}

#if TTALINK_MSG_ID_PTZ_ATTI_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_ptz_atti_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float pitch, float roll, float yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, pitch);
    _tta_put_float(buf, 8, roll);
    _tta_put_float(buf, 12, yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ATTI, buf, TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN, TTALINK_MSG_ID_PTZ_ATTI_LEN, TTALINK_MSG_ID_PTZ_ATTI_CRC, nocrc);
#else
    ttalink_ptz_atti_t *packet = (ttalink_ptz_atti_t *)msgbuf;
    packet->update_time = update_time;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->yaw = yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PTZ_ATTI, (const char *)packet, TTALINK_MSG_ID_PTZ_ATTI_MIN_LEN, TTALINK_MSG_ID_PTZ_ATTI_LEN, TTALINK_MSG_ID_PTZ_ATTI_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_ptz_atti_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float pitch, float roll, float yaw)
{
    _ttalink_ptz_atti_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, pitch, roll, yaw, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_ptz_atti_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float pitch, float roll, float yaw)
{
    _ttalink_ptz_atti_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, pitch, roll, yaw, true);
}
#endif

#endif

// MESSAGE PTZ_ATTI UNPACKING


/**
 * @brief Get field update_time from ptz_atti message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_ptz_atti_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field pitch from ptz_atti message
 *
 * @return  FC_Pitch_Ptz
 */
static inline float ttalink_ptz_atti_get_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field roll from ptz_atti message
 *
 * @return  FC_Roll_Ptz
 */
static inline float ttalink_ptz_atti_get_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from ptz_atti message
 *
 * @return  FC_Yaw_Ptz
 */
static inline float ttalink_ptz_atti_get_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Decode a ptz_atti message into a struct
 *
 * @param msg The message to decode
 * @param ptz_atti C-struct to decode the message contents into
 */
static inline void ttalink_ptz_atti_decode(const ttalink_message_t* msg, ttalink_ptz_atti_t* ptz_atti)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ptz_atti->update_time = ttalink_ptz_atti_get_update_time(msg);
    ptz_atti->pitch = ttalink_ptz_atti_get_pitch(msg);
    ptz_atti->roll = ttalink_ptz_atti_get_roll(msg);
    ptz_atti->yaw = ttalink_ptz_atti_get_yaw(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PTZ_ATTI_LEN? msg->len : TTALINK_MSG_ID_PTZ_ATTI_LEN;
        memset(ptz_atti, 0, TTALINK_MSG_ID_PTZ_ATTI_LEN);
    memcpy(ptz_atti, _TTA_PAYLOAD(msg), len);
#endif
}
