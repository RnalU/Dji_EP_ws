#pragma once
// MESSAGE MAV_HIL_MOTOR PACKING

#define TTALINK_MSG_ID_MAV_HIL_MOTOR 8220

TTAPACKED(
typedef struct __ttalink_mav_hil_motor_t {
 uint64_t time_usec; /*< Timestamp (microseconds, synced to UNIX time or since system boot)*/
 uint64_t flags; /*<  =0*/
 float controls[16]; /*<  PWM */
 uint8_t mode; /*<  =1 */
}) ttalink_mav_hil_motor_t;

#define TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN 81
#define TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN 81
#define TTALINK_MSG_ID_8220_LEN 81
#define TTALINK_MSG_ID_8220_MIN_LEN 81

#define TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC 18
#define TTALINK_MSG_ID_8220_CRC 18

#define TTALINK_MSG_MAV_HIL_MOTOR_FIELD_CONTROLS_LEN 16

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_MAV_HIL_MOTOR { \
    8220, \
    "MAV_HIL_MOTOR", \
    4, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_motor_t, time_usec) }, \
         { "flags", NULL, TTALINK_TYPE_UINT64_T, 0, 8, offsetof(ttalink_mav_hil_motor_t, flags) }, \
         { "controls", NULL, TTALINK_TYPE_FLOAT, 16, 16, offsetof(ttalink_mav_hil_motor_t, controls) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 80, offsetof(ttalink_mav_hil_motor_t, mode) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_MAV_HIL_MOTOR { \
    "MAV_HIL_MOTOR", \
    4, \
    {  { "time_usec", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_mav_hil_motor_t, time_usec) }, \
         { "flags", NULL, TTALINK_TYPE_UINT64_T, 0, 8, offsetof(ttalink_mav_hil_motor_t, flags) }, \
         { "controls", NULL, TTALINK_TYPE_FLOAT, 16, 16, offsetof(ttalink_mav_hil_motor_t, controls) }, \
         { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 80, offsetof(ttalink_mav_hil_motor_t, mode) }, \
         } \
}
#endif


static inline uint16_t _ttalink_mav_hil_motor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint64_t(buf, 8, flags);
    _tta_put_uint8_t(buf, 80, mode);
    _tta_put_float_array(buf, 16, controls, 16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN);
#else
    ttalink_mav_hil_motor_t packet;
    packet.time_usec = time_usec;
    packet.flags = flags;
    packet.mode = mode;
    tta_array_memcpy(packet.controls, controls, sizeof(float)*16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_MOTOR;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_motor message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param flags  =0
 * @param controls  PWM 
 * @param mode  =1 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_motor_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode)
{
    return _ttalink_mav_hil_motor_pack(dst_addr, src_addr, msg,  time_usec, flags, controls, mode, false);
}

/**
 * @brief Pack a mav_hil_motor message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param flags  =0
 * @param controls  PWM 
 * @param mode  =1 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_motor_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode)
{
    return _ttalink_mav_hil_motor_pack(dst_addr, src_addr, msg,  time_usec, flags, controls, mode, true);
}


static inline uint16_t _ttalink_mav_hil_motor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,uint64_t flags,const float *controls,uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint64_t(buf, 8, flags);
    _tta_put_uint8_t(buf, 80, mode);
    _tta_put_float_array(buf, 16, controls, 16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN);
#else
    ttalink_mav_hil_motor_t packet;
    packet.time_usec = time_usec;
    packet.flags = flags;
    packet.mode = mode;
    tta_array_memcpy(packet.controls, controls, sizeof(float)*16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_MAV_HIL_MOTOR;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC, nocrc);
}

/**
 * @brief Pack a mav_hil_motor message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param flags  =0
 * @param controls  PWM 
 * @param mode  =1 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_motor_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,uint64_t flags,const float *controls,uint8_t mode)
{
    return _ttalink_mav_hil_motor_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, flags, controls, mode, false);
}

/**
 * @brief Pack a mav_hil_motor message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param flags  =0
 * @param controls  PWM 
 * @param mode  =1 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_mav_hil_motor_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time_usec,uint64_t flags,const float *controls,uint8_t mode)
{
    return _ttalink_mav_hil_motor_pack_chan(dst_addr, src_addr, chan, msg,  time_usec, flags, controls, mode, true);
}


static inline uint16_t _ttalink_mav_hil_motor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_motor_t* mav_hil_motor, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_motor_pack_nocrc(dst_addr, src_addr, msg, mav_hil_motor->time_usec, mav_hil_motor->flags, mav_hil_motor->controls, mav_hil_motor->mode);
    }else{
        return ttalink_mav_hil_motor_pack(dst_addr, src_addr, msg, mav_hil_motor->time_usec, mav_hil_motor->flags, mav_hil_motor->controls, mav_hil_motor->mode);
    }
    
}

/**
 * @brief Encode a mav_hil_motor struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_motor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_motor_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_motor_t* mav_hil_motor)
{
    return _ttalink_mav_hil_motor_encode(dst_addr, src_addr, msg, mav_hil_motor, false);
}

/**
 * @brief Encode a mav_hil_motor struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_motor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_motor_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_mav_hil_motor_t* mav_hil_motor)
{
    return _ttalink_mav_hil_motor_encode(dst_addr, src_addr, msg, mav_hil_motor, true);
}


static inline uint16_t _ttalink_mav_hil_motor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_motor_t* mav_hil_motor, bool nocrc)
{
    if(nocrc){
        return ttalink_mav_hil_motor_pack_chan_nocrc(dst_addr, src_addr, chan, msg, mav_hil_motor->time_usec, mav_hil_motor->flags, mav_hil_motor->controls, mav_hil_motor->mode);
    }else{
        return ttalink_mav_hil_motor_pack_chan(dst_addr, src_addr, chan, msg, mav_hil_motor->time_usec, mav_hil_motor->flags, mav_hil_motor->controls, mav_hil_motor->mode);
    }
}

/**
 * @brief Encode a mav_hil_motor struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_motor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_motor_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_motor_t* mav_hil_motor)
{
    return _ttalink_mav_hil_motor_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_motor, false);
}

/**
 * @brief Encode a mav_hil_motor struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mav_hil_motor C-struct to read the message contents from
 */
static inline uint16_t ttalink_mav_hil_motor_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_mav_hil_motor_t* mav_hil_motor)
{
    return _ttalink_mav_hil_motor_encode_chan(dst_addr, src_addr, chan, msg, mav_hil_motor, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_mav_hil_motor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN];
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint64_t(buf, 8, flags);
    _tta_put_uint8_t(buf, 80, mode);
    _tta_put_float_array(buf, 16, controls, 16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_MOTOR, buf, TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC, nocrc);
#else
    ttalink_mav_hil_motor_t packet;
    packet.time_usec = time_usec;
    packet.flags = flags;
    packet.mode = mode;
    tta_array_memcpy(packet.controls, controls, sizeof(float)*16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_MOTOR, (const char *)&packet, TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_motor message
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param flags  =0
 * @param controls  PWM 
 * @param mode  =1 
 */
static inline void ttalink_mav_hil_motor_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode)
{
    _ttalink_mav_hil_motor_send(dst_addr, src_addr, chan, time_usec, flags, controls, mode, false);
}

/**
 * @brief Send a mav_hil_motor message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param flags  =0
 * @param controls  PWM 
 * @param mode  =1 
 */
static inline void ttalink_mav_hil_motor_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode)
{
    _ttalink_mav_hil_motor_send(dst_addr, src_addr, chan, time_usec, flags, controls, mode, true);
}


static inline void _ttalink_mav_hil_motor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_motor_t* mav_hil_motor, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_mav_hil_motor_send_nocrc(dst_addr, src_addr, chan, mav_hil_motor->time_usec, mav_hil_motor->flags, mav_hil_motor->controls, mav_hil_motor->mode);
    }else{
        ttalink_mav_hil_motor_send(dst_addr, src_addr, chan, mav_hil_motor->time_usec, mav_hil_motor->flags, mav_hil_motor->controls, mav_hil_motor->mode);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_MOTOR, (const char *)mav_hil_motor, TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC, nocrc);
#endif
}

/**
 * @brief Send a mav_hil_motor message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_motor_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_motor_t* mav_hil_motor)
{
    _ttalink_mav_hil_motor_send_struct(dst_addr, src_addr, chan, mav_hil_motor, false);
}

/**
 * @brief Send a mav_hil_motor message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_mav_hil_motor_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_mav_hil_motor_t* mav_hil_motor)
{
    _ttalink_mav_hil_motor_send_struct(dst_addr, src_addr, chan, mav_hil_motor, true);
}

#if TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_mav_hil_motor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, time_usec);
    _tta_put_uint64_t(buf, 8, flags);
    _tta_put_uint8_t(buf, 80, mode);
    _tta_put_float_array(buf, 16, controls, 16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_MOTOR, buf, TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC, nocrc);
#else
    ttalink_mav_hil_motor_t *packet = (ttalink_mav_hil_motor_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->flags = flags;
    packet->mode = mode;
    tta_array_memcpy(packet->controls, controls, sizeof(float)*16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_MAV_HIL_MOTOR, (const char *)packet, TTALINK_MSG_ID_MAV_HIL_MOTOR_MIN_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN, TTALINK_MSG_ID_MAV_HIL_MOTOR_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_mav_hil_motor_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode)
{
    _ttalink_mav_hil_motor_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, flags, controls, mode, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_mav_hil_motor_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time_usec, uint64_t flags, const float *controls, uint8_t mode)
{
    _ttalink_mav_hil_motor_send_buf(dst_addr, src_addr, msgbuf, chan, time_usec, flags, controls, mode, true);
}
#endif

#endif

// MESSAGE MAV_HIL_MOTOR UNPACKING


/**
 * @brief Get field time_usec from mav_hil_motor message
 *
 * @return Timestamp (microseconds, synced to UNIX time or since system boot)
 */
static inline uint64_t ttalink_mav_hil_motor_get_time_usec(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field flags from mav_hil_motor message
 *
 * @return  =0
 */
static inline uint64_t ttalink_mav_hil_motor_get_flags(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field controls from mav_hil_motor message
 *
 * @return  PWM 
 */
static inline uint16_t ttalink_mav_hil_motor_get_controls(const ttalink_message_t* msg, float *controls)
{
    return _TTA_RETURN_float_array(msg, controls, 16,  16);
}

/**
 * @brief Get field mode from mav_hil_motor message
 *
 * @return  =1 
 */
static inline uint8_t ttalink_mav_hil_motor_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Decode a mav_hil_motor message into a struct
 *
 * @param msg The message to decode
 * @param mav_hil_motor C-struct to decode the message contents into
 */
static inline void ttalink_mav_hil_motor_decode(const ttalink_message_t* msg, ttalink_mav_hil_motor_t* mav_hil_motor)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    mav_hil_motor->time_usec = ttalink_mav_hil_motor_get_time_usec(msg);
    mav_hil_motor->flags = ttalink_mav_hil_motor_get_flags(msg);
    ttalink_mav_hil_motor_get_controls(msg, mav_hil_motor->controls);
    mav_hil_motor->mode = ttalink_mav_hil_motor_get_mode(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN? msg->len : TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN;
        memset(mav_hil_motor, 0, TTALINK_MSG_ID_MAV_HIL_MOTOR_LEN);
    memcpy(mav_hil_motor, _TTA_PAYLOAD(msg), len);
#endif
}
