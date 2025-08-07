#pragma once
// MESSAGE SYNC_NAV_ACTIVATE_CHECK_DATA PACKING

#define TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA 2111

TTAPACKED(
typedef struct __ttalink_sync_nav_activate_check_data_t {
 uint32_t simple_time; /*<  . .*/
 uint8_t AttMode_flag; /*<  . .*/
 float f_Vnc_COM[3]; /*<  . .*/
 float f_Atti_COM[3]; /*<  . .*/
}) ttalink_sync_nav_activate_check_data_t;

#define TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN 29
#define TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN 29
#define TTALINK_MSG_ID_2111_LEN 29
#define TTALINK_MSG_ID_2111_MIN_LEN 29

#define TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC 81
#define TTALINK_MSG_ID_2111_CRC 81

#define TTALINK_MSG_SYNC_NAV_ACTIVATE_CHECK_DATA_FIELD_F_VNC_COM_LEN 3
#define TTALINK_MSG_SYNC_NAV_ACTIVATE_CHECK_DATA_FIELD_F_ATTI_COM_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_NAV_ACTIVATE_CHECK_DATA { \
    2111, \
    "SYNC_NAV_ACTIVATE_CHECK_DATA", \
    4, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_nav_activate_check_data_t, simple_time) }, \
         { "AttMode_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_nav_activate_check_data_t, AttMode_flag) }, \
         { "f_Vnc_COM", NULL, TTALINK_TYPE_FLOAT, 3, 5, offsetof(ttalink_sync_nav_activate_check_data_t, f_Vnc_COM) }, \
         { "f_Atti_COM", NULL, TTALINK_TYPE_FLOAT, 3, 17, offsetof(ttalink_sync_nav_activate_check_data_t, f_Atti_COM) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_NAV_ACTIVATE_CHECK_DATA { \
    "SYNC_NAV_ACTIVATE_CHECK_DATA", \
    4, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_nav_activate_check_data_t, simple_time) }, \
         { "AttMode_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_nav_activate_check_data_t, AttMode_flag) }, \
         { "f_Vnc_COM", NULL, TTALINK_TYPE_FLOAT, 3, 5, offsetof(ttalink_sync_nav_activate_check_data_t, f_Vnc_COM) }, \
         { "f_Atti_COM", NULL, TTALINK_TYPE_FLOAT, 3, 17, offsetof(ttalink_sync_nav_activate_check_data_t, f_Atti_COM) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_nav_activate_check_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, AttMode_flag);
    _tta_put_float_array(buf, 5, f_Vnc_COM, 3);
    _tta_put_float_array(buf, 17, f_Atti_COM, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN);
#else
    ttalink_sync_nav_activate_check_data_t packet;
    packet.simple_time = simple_time;
    packet.AttMode_flag = AttMode_flag;
    tta_array_memcpy(packet.f_Vnc_COM, f_Vnc_COM, sizeof(float)*3);
    tta_array_memcpy(packet.f_Atti_COM, f_Atti_COM, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC, nocrc);
}

/**
 * @brief Pack a sync_nav_activate_check_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param AttMode_flag  . .
 * @param f_Vnc_COM  . .
 * @param f_Atti_COM  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM)
{
    return _ttalink_sync_nav_activate_check_data_pack(dst_addr, src_addr, msg,  simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, false);
}

/**
 * @brief Pack a sync_nav_activate_check_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param AttMode_flag  . .
 * @param f_Vnc_COM  . .
 * @param f_Atti_COM  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM)
{
    return _ttalink_sync_nav_activate_check_data_pack(dst_addr, src_addr, msg,  simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, true);
}


static inline uint16_t _ttalink_sync_nav_activate_check_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t AttMode_flag,const float *f_Vnc_COM,const float *f_Atti_COM, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, AttMode_flag);
    _tta_put_float_array(buf, 5, f_Vnc_COM, 3);
    _tta_put_float_array(buf, 17, f_Atti_COM, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN);
#else
    ttalink_sync_nav_activate_check_data_t packet;
    packet.simple_time = simple_time;
    packet.AttMode_flag = AttMode_flag;
    tta_array_memcpy(packet.f_Vnc_COM, f_Vnc_COM, sizeof(float)*3);
    tta_array_memcpy(packet.f_Atti_COM, f_Atti_COM, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC, nocrc);
}

/**
 * @brief Pack a sync_nav_activate_check_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param AttMode_flag  . .
 * @param f_Vnc_COM  . .
 * @param f_Atti_COM  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t AttMode_flag,const float *f_Vnc_COM,const float *f_Atti_COM)
{
    return _ttalink_sync_nav_activate_check_data_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, false);
}

/**
 * @brief Pack a sync_nav_activate_check_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param AttMode_flag  . .
 * @param f_Vnc_COM  . .
 * @param f_Atti_COM  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t AttMode_flag,const float *f_Vnc_COM,const float *f_Atti_COM)
{
    return _ttalink_sync_nav_activate_check_data_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, true);
}


static inline uint16_t _ttalink_sync_nav_activate_check_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_nav_activate_check_data_pack_nocrc(dst_addr, src_addr, msg, sync_nav_activate_check_data->simple_time, sync_nav_activate_check_data->AttMode_flag, sync_nav_activate_check_data->f_Vnc_COM, sync_nav_activate_check_data->f_Atti_COM);
    }else{
        return ttalink_sync_nav_activate_check_data_pack(dst_addr, src_addr, msg, sync_nav_activate_check_data->simple_time, sync_nav_activate_check_data->AttMode_flag, sync_nav_activate_check_data->f_Vnc_COM, sync_nav_activate_check_data->f_Atti_COM);
    }
    
}

/**
 * @brief Encode a sync_nav_activate_check_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_nav_activate_check_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data)
{
    return _ttalink_sync_nav_activate_check_data_encode(dst_addr, src_addr, msg, sync_nav_activate_check_data, false);
}

/**
 * @brief Encode a sync_nav_activate_check_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_nav_activate_check_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data)
{
    return _ttalink_sync_nav_activate_check_data_encode(dst_addr, src_addr, msg, sync_nav_activate_check_data, true);
}


static inline uint16_t _ttalink_sync_nav_activate_check_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_nav_activate_check_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_nav_activate_check_data->simple_time, sync_nav_activate_check_data->AttMode_flag, sync_nav_activate_check_data->f_Vnc_COM, sync_nav_activate_check_data->f_Atti_COM);
    }else{
        return ttalink_sync_nav_activate_check_data_pack_chan(dst_addr, src_addr, chan, msg, sync_nav_activate_check_data->simple_time, sync_nav_activate_check_data->AttMode_flag, sync_nav_activate_check_data->f_Vnc_COM, sync_nav_activate_check_data->f_Atti_COM);
    }
}

/**
 * @brief Encode a sync_nav_activate_check_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_nav_activate_check_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data)
{
    return _ttalink_sync_nav_activate_check_data_encode_chan(dst_addr, src_addr, chan, msg, sync_nav_activate_check_data, false);
}

/**
 * @brief Encode a sync_nav_activate_check_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_nav_activate_check_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data)
{
    return _ttalink_sync_nav_activate_check_data_encode_chan(dst_addr, src_addr, chan, msg, sync_nav_activate_check_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_nav_activate_check_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, AttMode_flag);
    _tta_put_float_array(buf, 5, f_Vnc_COM, 3);
    _tta_put_float_array(buf, 17, f_Atti_COM, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA, buf, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC, nocrc);
#else
    ttalink_sync_nav_activate_check_data_t packet;
    packet.simple_time = simple_time;
    packet.AttMode_flag = AttMode_flag;
    tta_array_memcpy(packet.f_Vnc_COM, f_Vnc_COM, sizeof(float)*3);
    tta_array_memcpy(packet.f_Atti_COM, f_Atti_COM, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA, (const char *)&packet, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_nav_activate_check_data message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param AttMode_flag  . .
 * @param f_Vnc_COM  . .
 * @param f_Atti_COM  . .
 */
static inline void ttalink_sync_nav_activate_check_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM)
{
    _ttalink_sync_nav_activate_check_data_send(dst_addr, src_addr, chan, simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, false);
}

/**
 * @brief Send a sync_nav_activate_check_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param AttMode_flag  . .
 * @param f_Vnc_COM  . .
 * @param f_Atti_COM  . .
 */
static inline void ttalink_sync_nav_activate_check_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM)
{
    _ttalink_sync_nav_activate_check_data_send(dst_addr, src_addr, chan, simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, true);
}


static inline void _ttalink_sync_nav_activate_check_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_nav_activate_check_data_send_nocrc(dst_addr, src_addr, chan, sync_nav_activate_check_data->simple_time, sync_nav_activate_check_data->AttMode_flag, sync_nav_activate_check_data->f_Vnc_COM, sync_nav_activate_check_data->f_Atti_COM);
    }else{
        ttalink_sync_nav_activate_check_data_send(dst_addr, src_addr, chan, sync_nav_activate_check_data->simple_time, sync_nav_activate_check_data->AttMode_flag, sync_nav_activate_check_data->f_Vnc_COM, sync_nav_activate_check_data->f_Atti_COM);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA, (const char *)sync_nav_activate_check_data, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_nav_activate_check_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_nav_activate_check_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data)
{
    _ttalink_sync_nav_activate_check_data_send_struct(dst_addr, src_addr, chan, sync_nav_activate_check_data, false);
}

/**
 * @brief Send a sync_nav_activate_check_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_nav_activate_check_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data)
{
    _ttalink_sync_nav_activate_check_data_send_struct(dst_addr, src_addr, chan, sync_nav_activate_check_data, true);
}

#if TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_nav_activate_check_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, AttMode_flag);
    _tta_put_float_array(buf, 5, f_Vnc_COM, 3);
    _tta_put_float_array(buf, 17, f_Atti_COM, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA, buf, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC, nocrc);
#else
    ttalink_sync_nav_activate_check_data_t *packet = (ttalink_sync_nav_activate_check_data_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->AttMode_flag = AttMode_flag;
    tta_array_memcpy(packet->f_Vnc_COM, f_Vnc_COM, sizeof(float)*3);
    tta_array_memcpy(packet->f_Atti_COM, f_Atti_COM, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA, (const char *)packet, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_MIN_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_nav_activate_check_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM)
{
    _ttalink_sync_nav_activate_check_data_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_nav_activate_check_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t AttMode_flag, const float *f_Vnc_COM, const float *f_Atti_COM)
{
    _ttalink_sync_nav_activate_check_data_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, AttMode_flag, f_Vnc_COM, f_Atti_COM, true);
}
#endif

#endif

// MESSAGE SYNC_NAV_ACTIVATE_CHECK_DATA UNPACKING


/**
 * @brief Get field simple_time from sync_nav_activate_check_data message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_nav_activate_check_data_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field AttMode_flag from sync_nav_activate_check_data message
 *
 * @return  . .
 */
static inline uint8_t ttalink_sync_nav_activate_check_data_get_AttMode_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field f_Vnc_COM from sync_nav_activate_check_data message
 *
 * @return  . .
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_get_f_Vnc_COM(const ttalink_message_t* msg, float *f_Vnc_COM)
{
    return _TTA_RETURN_float_array(msg, f_Vnc_COM, 3,  5);
}

/**
 * @brief Get field f_Atti_COM from sync_nav_activate_check_data message
 *
 * @return  . .
 */
static inline uint16_t ttalink_sync_nav_activate_check_data_get_f_Atti_COM(const ttalink_message_t* msg, float *f_Atti_COM)
{
    return _TTA_RETURN_float_array(msg, f_Atti_COM, 3,  17);
}

/**
 * @brief Decode a sync_nav_activate_check_data message into a struct
 *
 * @param msg The message to decode
 * @param sync_nav_activate_check_data C-struct to decode the message contents into
 */
static inline void ttalink_sync_nav_activate_check_data_decode(const ttalink_message_t* msg, ttalink_sync_nav_activate_check_data_t* sync_nav_activate_check_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_nav_activate_check_data->simple_time = ttalink_sync_nav_activate_check_data_get_simple_time(msg);
    sync_nav_activate_check_data->AttMode_flag = ttalink_sync_nav_activate_check_data_get_AttMode_flag(msg);
    ttalink_sync_nav_activate_check_data_get_f_Vnc_COM(msg, sync_nav_activate_check_data->f_Vnc_COM);
    ttalink_sync_nav_activate_check_data_get_f_Atti_COM(msg, sync_nav_activate_check_data->f_Atti_COM);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN? msg->len : TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN;
        memset(sync_nav_activate_check_data, 0, TTALINK_MSG_ID_SYNC_NAV_ACTIVATE_CHECK_DATA_LEN);
    memcpy(sync_nav_activate_check_data, _TTA_PAYLOAD(msg), len);
#endif
}
