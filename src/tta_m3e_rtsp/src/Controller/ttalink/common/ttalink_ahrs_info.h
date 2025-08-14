#pragma once
// MESSAGE AHRS_INFO PACKING

#define TTALINK_MSG_ID_AHRS_INFO 2003

TTAPACKED(
typedef struct __ttalink_ahrs_info_t {
 float target_pitch; /*<  target_attiude_pitch.*/
 float target_roll; /*<  target_attiude_roll.*/
 float target_yaw; /*<  target_attiude_yaw.*/
 float att_pitch; /*<  real attiude Pitch.*/
 float att_roll; /*<  real attiude Roll.*/
 float att_yaw; /*<  real attiude Yaw.*/
}) ttalink_ahrs_info_t;

#define TTALINK_MSG_ID_AHRS_INFO_LEN 24
#define TTALINK_MSG_ID_AHRS_INFO_MIN_LEN 24
#define TTALINK_MSG_ID_2003_LEN 24
#define TTALINK_MSG_ID_2003_MIN_LEN 24

#define TTALINK_MSG_ID_AHRS_INFO_CRC 80
#define TTALINK_MSG_ID_2003_CRC 80



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_AHRS_INFO { \
    2003, \
    "AHRS_INFO", \
    6, \
    {  { "target_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_ahrs_info_t, target_pitch) }, \
         { "target_roll", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_ahrs_info_t, target_roll) }, \
         { "target_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_ahrs_info_t, target_yaw) }, \
         { "att_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_ahrs_info_t, att_pitch) }, \
         { "att_roll", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_ahrs_info_t, att_roll) }, \
         { "att_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_ahrs_info_t, att_yaw) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_AHRS_INFO { \
    "AHRS_INFO", \
    6, \
    {  { "target_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_ahrs_info_t, target_pitch) }, \
         { "target_roll", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_ahrs_info_t, target_roll) }, \
         { "target_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_ahrs_info_t, target_yaw) }, \
         { "att_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_ahrs_info_t, att_pitch) }, \
         { "att_roll", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_ahrs_info_t, att_roll) }, \
         { "att_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_ahrs_info_t, att_yaw) }, \
         } \
}
#endif


static inline uint16_t _ttalink_ahrs_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_AHRS_INFO_LEN];
    _tta_put_float(buf, 0, target_pitch);
    _tta_put_float(buf, 4, target_roll);
    _tta_put_float(buf, 8, target_yaw);
    _tta_put_float(buf, 12, att_pitch);
    _tta_put_float(buf, 16, att_roll);
    _tta_put_float(buf, 20, att_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_AHRS_INFO_LEN);
#else
    ttalink_ahrs_info_t packet;
    packet.target_pitch = target_pitch;
    packet.target_roll = target_roll;
    packet.target_yaw = target_yaw;
    packet.att_pitch = att_pitch;
    packet.att_roll = att_roll;
    packet.att_yaw = att_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_AHRS_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_AHRS_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_AHRS_INFO_MIN_LEN, TTALINK_MSG_ID_AHRS_INFO_LEN, TTALINK_MSG_ID_AHRS_INFO_CRC, nocrc);
}

/**
 * @brief Pack a ahrs_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param target_pitch  target_attiude_pitch.
 * @param target_roll  target_attiude_roll.
 * @param target_yaw  target_attiude_yaw.
 * @param att_pitch  real attiude Pitch.
 * @param att_roll  real attiude Roll.
 * @param att_yaw  real attiude Yaw.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ahrs_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw)
{
    return _ttalink_ahrs_info_pack(dst_addr, src_addr, msg,  target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, false);
}

/**
 * @brief Pack a ahrs_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param target_pitch  target_attiude_pitch.
 * @param target_roll  target_attiude_roll.
 * @param target_yaw  target_attiude_yaw.
 * @param att_pitch  real attiude Pitch.
 * @param att_roll  real attiude Roll.
 * @param att_yaw  real attiude Yaw.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ahrs_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw)
{
    return _ttalink_ahrs_info_pack(dst_addr, src_addr, msg,  target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, true);
}


static inline uint16_t _ttalink_ahrs_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float target_pitch,float target_roll,float target_yaw,float att_pitch,float att_roll,float att_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_AHRS_INFO_LEN];
    _tta_put_float(buf, 0, target_pitch);
    _tta_put_float(buf, 4, target_roll);
    _tta_put_float(buf, 8, target_yaw);
    _tta_put_float(buf, 12, att_pitch);
    _tta_put_float(buf, 16, att_roll);
    _tta_put_float(buf, 20, att_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_AHRS_INFO_LEN);
#else
    ttalink_ahrs_info_t packet;
    packet.target_pitch = target_pitch;
    packet.target_roll = target_roll;
    packet.target_yaw = target_yaw;
    packet.att_pitch = att_pitch;
    packet.att_roll = att_roll;
    packet.att_yaw = att_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_AHRS_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_AHRS_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_AHRS_INFO_MIN_LEN, TTALINK_MSG_ID_AHRS_INFO_LEN, TTALINK_MSG_ID_AHRS_INFO_CRC, nocrc);
}

/**
 * @brief Pack a ahrs_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param target_pitch  target_attiude_pitch.
 * @param target_roll  target_attiude_roll.
 * @param target_yaw  target_attiude_yaw.
 * @param att_pitch  real attiude Pitch.
 * @param att_roll  real attiude Roll.
 * @param att_yaw  real attiude Yaw.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ahrs_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float target_pitch,float target_roll,float target_yaw,float att_pitch,float att_roll,float att_yaw)
{
    return _ttalink_ahrs_info_pack_chan(dst_addr, src_addr, chan, msg,  target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, false);
}

/**
 * @brief Pack a ahrs_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param target_pitch  target_attiude_pitch.
 * @param target_roll  target_attiude_roll.
 * @param target_yaw  target_attiude_yaw.
 * @param att_pitch  real attiude Pitch.
 * @param att_roll  real attiude Roll.
 * @param att_yaw  real attiude Yaw.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_ahrs_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float target_pitch,float target_roll,float target_yaw,float att_pitch,float att_roll,float att_yaw)
{
    return _ttalink_ahrs_info_pack_chan(dst_addr, src_addr, chan, msg,  target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, true);
}


static inline uint16_t _ttalink_ahrs_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ahrs_info_t* ahrs_info, bool nocrc)
{
    if(nocrc){
        return ttalink_ahrs_info_pack_nocrc(dst_addr, src_addr, msg, ahrs_info->target_pitch, ahrs_info->target_roll, ahrs_info->target_yaw, ahrs_info->att_pitch, ahrs_info->att_roll, ahrs_info->att_yaw);
    }else{
        return ttalink_ahrs_info_pack(dst_addr, src_addr, msg, ahrs_info->target_pitch, ahrs_info->target_roll, ahrs_info->target_yaw, ahrs_info->att_pitch, ahrs_info->att_roll, ahrs_info->att_yaw);
    }
    
}

/**
 * @brief Encode a ahrs_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ahrs_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_ahrs_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ahrs_info_t* ahrs_info)
{
    return _ttalink_ahrs_info_encode(dst_addr, src_addr, msg, ahrs_info, false);
}

/**
 * @brief Encode a ahrs_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param ahrs_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_ahrs_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_ahrs_info_t* ahrs_info)
{
    return _ttalink_ahrs_info_encode(dst_addr, src_addr, msg, ahrs_info, true);
}


static inline uint16_t _ttalink_ahrs_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ahrs_info_t* ahrs_info, bool nocrc)
{
    if(nocrc){
        return ttalink_ahrs_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, ahrs_info->target_pitch, ahrs_info->target_roll, ahrs_info->target_yaw, ahrs_info->att_pitch, ahrs_info->att_roll, ahrs_info->att_yaw);
    }else{
        return ttalink_ahrs_info_pack_chan(dst_addr, src_addr, chan, msg, ahrs_info->target_pitch, ahrs_info->target_roll, ahrs_info->target_yaw, ahrs_info->att_pitch, ahrs_info->att_roll, ahrs_info->att_yaw);
    }
}

/**
 * @brief Encode a ahrs_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ahrs_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_ahrs_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ahrs_info_t* ahrs_info)
{
    return _ttalink_ahrs_info_encode_chan(dst_addr, src_addr, chan, msg, ahrs_info, false);
}

/**
 * @brief Encode a ahrs_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ahrs_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_ahrs_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_ahrs_info_t* ahrs_info)
{
    return _ttalink_ahrs_info_encode_chan(dst_addr, src_addr, chan, msg, ahrs_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_ahrs_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_AHRS_INFO_LEN];
    _tta_put_float(buf, 0, target_pitch);
    _tta_put_float(buf, 4, target_roll);
    _tta_put_float(buf, 8, target_yaw);
    _tta_put_float(buf, 12, att_pitch);
    _tta_put_float(buf, 16, att_roll);
    _tta_put_float(buf, 20, att_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_AHRS_INFO, buf, TTALINK_MSG_ID_AHRS_INFO_MIN_LEN, TTALINK_MSG_ID_AHRS_INFO_LEN, TTALINK_MSG_ID_AHRS_INFO_CRC, nocrc);
#else
    ttalink_ahrs_info_t packet;
    packet.target_pitch = target_pitch;
    packet.target_roll = target_roll;
    packet.target_yaw = target_yaw;
    packet.att_pitch = att_pitch;
    packet.att_roll = att_roll;
    packet.att_yaw = att_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_AHRS_INFO, (const char *)&packet, TTALINK_MSG_ID_AHRS_INFO_MIN_LEN, TTALINK_MSG_ID_AHRS_INFO_LEN, TTALINK_MSG_ID_AHRS_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a ahrs_info message
 * @param chan TTAlink channel to send the message
 *
 * @param target_pitch  target_attiude_pitch.
 * @param target_roll  target_attiude_roll.
 * @param target_yaw  target_attiude_yaw.
 * @param att_pitch  real attiude Pitch.
 * @param att_roll  real attiude Roll.
 * @param att_yaw  real attiude Yaw.
 */
static inline void ttalink_ahrs_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw)
{
    _ttalink_ahrs_info_send(dst_addr, src_addr, chan, target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, false);
}

/**
 * @brief Send a ahrs_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param target_pitch  target_attiude_pitch.
 * @param target_roll  target_attiude_roll.
 * @param target_yaw  target_attiude_yaw.
 * @param att_pitch  real attiude Pitch.
 * @param att_roll  real attiude Roll.
 * @param att_yaw  real attiude Yaw.
 */
static inline void ttalink_ahrs_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw)
{
    _ttalink_ahrs_info_send(dst_addr, src_addr, chan, target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, true);
}


static inline void _ttalink_ahrs_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ahrs_info_t* ahrs_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_ahrs_info_send_nocrc(dst_addr, src_addr, chan, ahrs_info->target_pitch, ahrs_info->target_roll, ahrs_info->target_yaw, ahrs_info->att_pitch, ahrs_info->att_roll, ahrs_info->att_yaw);
    }else{
        ttalink_ahrs_info_send(dst_addr, src_addr, chan, ahrs_info->target_pitch, ahrs_info->target_roll, ahrs_info->target_yaw, ahrs_info->att_pitch, ahrs_info->att_roll, ahrs_info->att_yaw);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_AHRS_INFO, (const char *)ahrs_info, TTALINK_MSG_ID_AHRS_INFO_MIN_LEN, TTALINK_MSG_ID_AHRS_INFO_LEN, TTALINK_MSG_ID_AHRS_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a ahrs_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ahrs_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ahrs_info_t* ahrs_info)
{
    _ttalink_ahrs_info_send_struct(dst_addr, src_addr, chan, ahrs_info, false);
}

/**
 * @brief Send a ahrs_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_ahrs_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_ahrs_info_t* ahrs_info)
{
    _ttalink_ahrs_info_send_struct(dst_addr, src_addr, chan, ahrs_info, true);
}

#if TTALINK_MSG_ID_AHRS_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_ahrs_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 0, target_pitch);
    _tta_put_float(buf, 4, target_roll);
    _tta_put_float(buf, 8, target_yaw);
    _tta_put_float(buf, 12, att_pitch);
    _tta_put_float(buf, 16, att_roll);
    _tta_put_float(buf, 20, att_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_AHRS_INFO, buf, TTALINK_MSG_ID_AHRS_INFO_MIN_LEN, TTALINK_MSG_ID_AHRS_INFO_LEN, TTALINK_MSG_ID_AHRS_INFO_CRC, nocrc);
#else
    ttalink_ahrs_info_t *packet = (ttalink_ahrs_info_t *)msgbuf;
    packet->target_pitch = target_pitch;
    packet->target_roll = target_roll;
    packet->target_yaw = target_yaw;
    packet->att_pitch = att_pitch;
    packet->att_roll = att_roll;
    packet->att_yaw = att_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_AHRS_INFO, (const char *)packet, TTALINK_MSG_ID_AHRS_INFO_MIN_LEN, TTALINK_MSG_ID_AHRS_INFO_LEN, TTALINK_MSG_ID_AHRS_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_ahrs_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw)
{
    _ttalink_ahrs_info_send_buf(dst_addr, src_addr, msgbuf, chan, target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_ahrs_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float target_pitch, float target_roll, float target_yaw, float att_pitch, float att_roll, float att_yaw)
{
    _ttalink_ahrs_info_send_buf(dst_addr, src_addr, msgbuf, chan, target_pitch, target_roll, target_yaw, att_pitch, att_roll, att_yaw, true);
}
#endif

#endif

// MESSAGE AHRS_INFO UNPACKING


/**
 * @brief Get field target_pitch from ahrs_info message
 *
 * @return  target_attiude_pitch.
 */
static inline float ttalink_ahrs_info_get_target_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  0);
}

/**
 * @brief Get field target_roll from ahrs_info message
 *
 * @return  target_attiude_roll.
 */
static inline float ttalink_ahrs_info_get_target_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field target_yaw from ahrs_info message
 *
 * @return  target_attiude_yaw.
 */
static inline float ttalink_ahrs_info_get_target_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field att_pitch from ahrs_info message
 *
 * @return  real attiude Pitch.
 */
static inline float ttalink_ahrs_info_get_att_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field att_roll from ahrs_info message
 *
 * @return  real attiude Roll.
 */
static inline float ttalink_ahrs_info_get_att_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field att_yaw from ahrs_info message
 *
 * @return  real attiude Yaw.
 */
static inline float ttalink_ahrs_info_get_att_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Decode a ahrs_info message into a struct
 *
 * @param msg The message to decode
 * @param ahrs_info C-struct to decode the message contents into
 */
static inline void ttalink_ahrs_info_decode(const ttalink_message_t* msg, ttalink_ahrs_info_t* ahrs_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ahrs_info->target_pitch = ttalink_ahrs_info_get_target_pitch(msg);
    ahrs_info->target_roll = ttalink_ahrs_info_get_target_roll(msg);
    ahrs_info->target_yaw = ttalink_ahrs_info_get_target_yaw(msg);
    ahrs_info->att_pitch = ttalink_ahrs_info_get_att_pitch(msg);
    ahrs_info->att_roll = ttalink_ahrs_info_get_att_roll(msg);
    ahrs_info->att_yaw = ttalink_ahrs_info_get_att_yaw(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_AHRS_INFO_LEN? msg->len : TTALINK_MSG_ID_AHRS_INFO_LEN;
        memset(ahrs_info, 0, TTALINK_MSG_ID_AHRS_INFO_LEN);
    memcpy(ahrs_info, _TTA_PAYLOAD(msg), len);
#endif
}
