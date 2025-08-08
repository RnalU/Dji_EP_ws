#pragma once
// MESSAGE SYNC_INTEGRAL_INFO PACKING

#define TTALINK_MSG_ID_SYNC_INTEGRAL_INFO 2106

TTAPACKED(
typedef struct __ttalink_sync_integral_info_t {
 uint32_t simple_time; /*<  . .*/
 float gas_integral; /*<  . .*/
 float gyro_pitch_integral; /*<  . .*/
 float gyro_roll_integral; /*<  . .*/
 float gyro_yaw_integral; /*<  . .*/
 float vel_integral_n; /*<  . .*/
 float vel_integral_e; /*<  . .*/
}) ttalink_sync_integral_info_t;

#define TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN 28
#define TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN 28
#define TTALINK_MSG_ID_2106_LEN 28
#define TTALINK_MSG_ID_2106_MIN_LEN 28

#define TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC 213
#define TTALINK_MSG_ID_2106_CRC 213



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_INTEGRAL_INFO { \
    2106, \
    "SYNC_INTEGRAL_INFO", \
    7, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_integral_info_t, simple_time) }, \
         { "gas_integral", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_sync_integral_info_t, gas_integral) }, \
         { "gyro_pitch_integral", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_sync_integral_info_t, gyro_pitch_integral) }, \
         { "gyro_roll_integral", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_sync_integral_info_t, gyro_roll_integral) }, \
         { "gyro_yaw_integral", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_sync_integral_info_t, gyro_yaw_integral) }, \
         { "vel_integral_n", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_sync_integral_info_t, vel_integral_n) }, \
         { "vel_integral_e", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_sync_integral_info_t, vel_integral_e) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_INTEGRAL_INFO { \
    "SYNC_INTEGRAL_INFO", \
    7, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_integral_info_t, simple_time) }, \
         { "gas_integral", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_sync_integral_info_t, gas_integral) }, \
         { "gyro_pitch_integral", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_sync_integral_info_t, gyro_pitch_integral) }, \
         { "gyro_roll_integral", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_sync_integral_info_t, gyro_roll_integral) }, \
         { "gyro_yaw_integral", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_sync_integral_info_t, gyro_yaw_integral) }, \
         { "vel_integral_n", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_sync_integral_info_t, vel_integral_n) }, \
         { "vel_integral_e", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_sync_integral_info_t, vel_integral_e) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_integral_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, gas_integral);
    _tta_put_float(buf, 8, gyro_pitch_integral);
    _tta_put_float(buf, 12, gyro_roll_integral);
    _tta_put_float(buf, 16, gyro_yaw_integral);
    _tta_put_float(buf, 20, vel_integral_n);
    _tta_put_float(buf, 24, vel_integral_e);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN);
#else
    ttalink_sync_integral_info_t packet;
    packet.simple_time = simple_time;
    packet.gas_integral = gas_integral;
    packet.gyro_pitch_integral = gyro_pitch_integral;
    packet.gyro_roll_integral = gyro_roll_integral;
    packet.gyro_yaw_integral = gyro_yaw_integral;
    packet.vel_integral_n = vel_integral_n;
    packet.vel_integral_e = vel_integral_e;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_INTEGRAL_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_integral_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param gas_integral  . .
 * @param gyro_pitch_integral  . .
 * @param gyro_roll_integral  . .
 * @param gyro_yaw_integral  . .
 * @param vel_integral_n  . .
 * @param vel_integral_e  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_integral_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e)
{
    return _ttalink_sync_integral_info_pack(dst_addr, src_addr, msg,  simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, false);
}

/**
 * @brief Pack a sync_integral_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param gas_integral  . .
 * @param gyro_pitch_integral  . .
 * @param gyro_roll_integral  . .
 * @param gyro_yaw_integral  . .
 * @param vel_integral_n  . .
 * @param vel_integral_e  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_integral_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e)
{
    return _ttalink_sync_integral_info_pack(dst_addr, src_addr, msg,  simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, true);
}


static inline uint16_t _ttalink_sync_integral_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,float gas_integral,float gyro_pitch_integral,float gyro_roll_integral,float gyro_yaw_integral,float vel_integral_n,float vel_integral_e, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, gas_integral);
    _tta_put_float(buf, 8, gyro_pitch_integral);
    _tta_put_float(buf, 12, gyro_roll_integral);
    _tta_put_float(buf, 16, gyro_yaw_integral);
    _tta_put_float(buf, 20, vel_integral_n);
    _tta_put_float(buf, 24, vel_integral_e);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN);
#else
    ttalink_sync_integral_info_t packet;
    packet.simple_time = simple_time;
    packet.gas_integral = gas_integral;
    packet.gyro_pitch_integral = gyro_pitch_integral;
    packet.gyro_roll_integral = gyro_roll_integral;
    packet.gyro_yaw_integral = gyro_yaw_integral;
    packet.vel_integral_n = vel_integral_n;
    packet.vel_integral_e = vel_integral_e;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_INTEGRAL_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_integral_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param gas_integral  . .
 * @param gyro_pitch_integral  . .
 * @param gyro_roll_integral  . .
 * @param gyro_yaw_integral  . .
 * @param vel_integral_n  . .
 * @param vel_integral_e  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_integral_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,float gas_integral,float gyro_pitch_integral,float gyro_roll_integral,float gyro_yaw_integral,float vel_integral_n,float vel_integral_e)
{
    return _ttalink_sync_integral_info_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, false);
}

/**
 * @brief Pack a sync_integral_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param gas_integral  . .
 * @param gyro_pitch_integral  . .
 * @param gyro_roll_integral  . .
 * @param gyro_yaw_integral  . .
 * @param vel_integral_n  . .
 * @param vel_integral_e  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_integral_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,float gas_integral,float gyro_pitch_integral,float gyro_roll_integral,float gyro_yaw_integral,float vel_integral_n,float vel_integral_e)
{
    return _ttalink_sync_integral_info_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, true);
}


static inline uint16_t _ttalink_sync_integral_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_integral_info_t* sync_integral_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_integral_info_pack_nocrc(dst_addr, src_addr, msg, sync_integral_info->simple_time, sync_integral_info->gas_integral, sync_integral_info->gyro_pitch_integral, sync_integral_info->gyro_roll_integral, sync_integral_info->gyro_yaw_integral, sync_integral_info->vel_integral_n, sync_integral_info->vel_integral_e);
    }else{
        return ttalink_sync_integral_info_pack(dst_addr, src_addr, msg, sync_integral_info->simple_time, sync_integral_info->gas_integral, sync_integral_info->gyro_pitch_integral, sync_integral_info->gyro_roll_integral, sync_integral_info->gyro_yaw_integral, sync_integral_info->vel_integral_n, sync_integral_info->vel_integral_e);
    }
    
}

/**
 * @brief Encode a sync_integral_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_integral_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_integral_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_integral_info_t* sync_integral_info)
{
    return _ttalink_sync_integral_info_encode(dst_addr, src_addr, msg, sync_integral_info, false);
}

/**
 * @brief Encode a sync_integral_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_integral_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_integral_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_integral_info_t* sync_integral_info)
{
    return _ttalink_sync_integral_info_encode(dst_addr, src_addr, msg, sync_integral_info, true);
}


static inline uint16_t _ttalink_sync_integral_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_integral_info_t* sync_integral_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_integral_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_integral_info->simple_time, sync_integral_info->gas_integral, sync_integral_info->gyro_pitch_integral, sync_integral_info->gyro_roll_integral, sync_integral_info->gyro_yaw_integral, sync_integral_info->vel_integral_n, sync_integral_info->vel_integral_e);
    }else{
        return ttalink_sync_integral_info_pack_chan(dst_addr, src_addr, chan, msg, sync_integral_info->simple_time, sync_integral_info->gas_integral, sync_integral_info->gyro_pitch_integral, sync_integral_info->gyro_roll_integral, sync_integral_info->gyro_yaw_integral, sync_integral_info->vel_integral_n, sync_integral_info->vel_integral_e);
    }
}

/**
 * @brief Encode a sync_integral_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_integral_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_integral_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_integral_info_t* sync_integral_info)
{
    return _ttalink_sync_integral_info_encode_chan(dst_addr, src_addr, chan, msg, sync_integral_info, false);
}

/**
 * @brief Encode a sync_integral_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_integral_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_integral_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_integral_info_t* sync_integral_info)
{
    return _ttalink_sync_integral_info_encode_chan(dst_addr, src_addr, chan, msg, sync_integral_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_integral_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, gas_integral);
    _tta_put_float(buf, 8, gyro_pitch_integral);
    _tta_put_float(buf, 12, gyro_roll_integral);
    _tta_put_float(buf, 16, gyro_yaw_integral);
    _tta_put_float(buf, 20, vel_integral_n);
    _tta_put_float(buf, 24, vel_integral_e);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO, buf, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC, nocrc);
#else
    ttalink_sync_integral_info_t packet;
    packet.simple_time = simple_time;
    packet.gas_integral = gas_integral;
    packet.gyro_pitch_integral = gyro_pitch_integral;
    packet.gyro_roll_integral = gyro_roll_integral;
    packet.gyro_yaw_integral = gyro_yaw_integral;
    packet.vel_integral_n = vel_integral_n;
    packet.vel_integral_e = vel_integral_e;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO, (const char *)&packet, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_integral_info message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param gas_integral  . .
 * @param gyro_pitch_integral  . .
 * @param gyro_roll_integral  . .
 * @param gyro_yaw_integral  . .
 * @param vel_integral_n  . .
 * @param vel_integral_e  . .
 */
static inline void ttalink_sync_integral_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e)
{
    _ttalink_sync_integral_info_send(dst_addr, src_addr, chan, simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, false);
}

/**
 * @brief Send a sync_integral_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param gas_integral  . .
 * @param gyro_pitch_integral  . .
 * @param gyro_roll_integral  . .
 * @param gyro_yaw_integral  . .
 * @param vel_integral_n  . .
 * @param vel_integral_e  . .
 */
static inline void ttalink_sync_integral_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e)
{
    _ttalink_sync_integral_info_send(dst_addr, src_addr, chan, simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, true);
}


static inline void _ttalink_sync_integral_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_integral_info_t* sync_integral_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_integral_info_send_nocrc(dst_addr, src_addr, chan, sync_integral_info->simple_time, sync_integral_info->gas_integral, sync_integral_info->gyro_pitch_integral, sync_integral_info->gyro_roll_integral, sync_integral_info->gyro_yaw_integral, sync_integral_info->vel_integral_n, sync_integral_info->vel_integral_e);
    }else{
        ttalink_sync_integral_info_send(dst_addr, src_addr, chan, sync_integral_info->simple_time, sync_integral_info->gas_integral, sync_integral_info->gyro_pitch_integral, sync_integral_info->gyro_roll_integral, sync_integral_info->gyro_yaw_integral, sync_integral_info->vel_integral_n, sync_integral_info->vel_integral_e);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO, (const char *)sync_integral_info, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_integral_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_integral_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_integral_info_t* sync_integral_info)
{
    _ttalink_sync_integral_info_send_struct(dst_addr, src_addr, chan, sync_integral_info, false);
}

/**
 * @brief Send a sync_integral_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_integral_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_integral_info_t* sync_integral_info)
{
    _ttalink_sync_integral_info_send_struct(dst_addr, src_addr, chan, sync_integral_info, true);
}

#if TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_integral_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float(buf, 4, gas_integral);
    _tta_put_float(buf, 8, gyro_pitch_integral);
    _tta_put_float(buf, 12, gyro_roll_integral);
    _tta_put_float(buf, 16, gyro_yaw_integral);
    _tta_put_float(buf, 20, vel_integral_n);
    _tta_put_float(buf, 24, vel_integral_e);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO, buf, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC, nocrc);
#else
    ttalink_sync_integral_info_t *packet = (ttalink_sync_integral_info_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->gas_integral = gas_integral;
    packet->gyro_pitch_integral = gyro_pitch_integral;
    packet->gyro_roll_integral = gyro_roll_integral;
    packet->gyro_yaw_integral = gyro_yaw_integral;
    packet->vel_integral_n = vel_integral_n;
    packet->vel_integral_e = vel_integral_e;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO, (const char *)packet, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_integral_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e)
{
    _ttalink_sync_integral_info_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_integral_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, float gas_integral, float gyro_pitch_integral, float gyro_roll_integral, float gyro_yaw_integral, float vel_integral_n, float vel_integral_e)
{
    _ttalink_sync_integral_info_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, gas_integral, gyro_pitch_integral, gyro_roll_integral, gyro_yaw_integral, vel_integral_n, vel_integral_e, true);
}
#endif

#endif

// MESSAGE SYNC_INTEGRAL_INFO UNPACKING


/**
 * @brief Get field simple_time from sync_integral_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_integral_info_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field gas_integral from sync_integral_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_integral_info_get_gas_integral(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field gyro_pitch_integral from sync_integral_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_integral_info_get_gyro_pitch_integral(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field gyro_roll_integral from sync_integral_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_integral_info_get_gyro_roll_integral(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_yaw_integral from sync_integral_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_integral_info_get_gyro_yaw_integral(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_integral_n from sync_integral_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_integral_info_get_vel_integral_n(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field vel_integral_e from sync_integral_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_integral_info_get_vel_integral_e(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Decode a sync_integral_info message into a struct
 *
 * @param msg The message to decode
 * @param sync_integral_info C-struct to decode the message contents into
 */
static inline void ttalink_sync_integral_info_decode(const ttalink_message_t* msg, ttalink_sync_integral_info_t* sync_integral_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_integral_info->simple_time = ttalink_sync_integral_info_get_simple_time(msg);
    sync_integral_info->gas_integral = ttalink_sync_integral_info_get_gas_integral(msg);
    sync_integral_info->gyro_pitch_integral = ttalink_sync_integral_info_get_gyro_pitch_integral(msg);
    sync_integral_info->gyro_roll_integral = ttalink_sync_integral_info_get_gyro_roll_integral(msg);
    sync_integral_info->gyro_yaw_integral = ttalink_sync_integral_info_get_gyro_yaw_integral(msg);
    sync_integral_info->vel_integral_n = ttalink_sync_integral_info_get_vel_integral_n(msg);
    sync_integral_info->vel_integral_e = ttalink_sync_integral_info_get_vel_integral_e(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN? msg->len : TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN;
        memset(sync_integral_info, 0, TTALINK_MSG_ID_SYNC_INTEGRAL_INFO_LEN);
    memcpy(sync_integral_info, _TTA_PAYLOAD(msg), len);
#endif
}
