#pragma once
// MESSAGE SYNC_CALI_RC PACKING

#define TTALINK_MSG_ID_SYNC_CALI_RC 2122

TTAPACKED(
typedef struct __ttalink_sync_cali_rc_t {
 uint32_t simple_time; /*<  .*/
 int16_t ChMax[4]; /*<  .*/
 int16_t ChMid[4]; /*<  .*/
 int16_t ChMin[4]; /*<  .*/
}) ttalink_sync_cali_rc_t;

#define TTALINK_MSG_ID_SYNC_CALI_RC_LEN 28
#define TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN 28
#define TTALINK_MSG_ID_2122_LEN 28
#define TTALINK_MSG_ID_2122_MIN_LEN 28

#define TTALINK_MSG_ID_SYNC_CALI_RC_CRC 226
#define TTALINK_MSG_ID_2122_CRC 226

#define TTALINK_MSG_SYNC_CALI_RC_FIELD_CHMAX_LEN 4
#define TTALINK_MSG_SYNC_CALI_RC_FIELD_CHMID_LEN 4
#define TTALINK_MSG_SYNC_CALI_RC_FIELD_CHMIN_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CALI_RC { \
    2122, \
    "SYNC_CALI_RC", \
    4, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_rc_t, simple_time) }, \
         { "ChMax", NULL, TTALINK_TYPE_INT16_T, 4, 4, offsetof(ttalink_sync_cali_rc_t, ChMax) }, \
         { "ChMid", NULL, TTALINK_TYPE_INT16_T, 4, 12, offsetof(ttalink_sync_cali_rc_t, ChMid) }, \
         { "ChMin", NULL, TTALINK_TYPE_INT16_T, 4, 20, offsetof(ttalink_sync_cali_rc_t, ChMin) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CALI_RC { \
    "SYNC_CALI_RC", \
    4, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_rc_t, simple_time) }, \
         { "ChMax", NULL, TTALINK_TYPE_INT16_T, 4, 4, offsetof(ttalink_sync_cali_rc_t, ChMax) }, \
         { "ChMid", NULL, TTALINK_TYPE_INT16_T, 4, 12, offsetof(ttalink_sync_cali_rc_t, ChMid) }, \
         { "ChMin", NULL, TTALINK_TYPE_INT16_T, 4, 20, offsetof(ttalink_sync_cali_rc_t, ChMin) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_cali_rc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_RC_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_int16_t_array(buf, 4, ChMax, 4);
    _tta_put_int16_t_array(buf, 12, ChMid, 4);
    _tta_put_int16_t_array(buf, 20, ChMin, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_RC_LEN);
#else
    ttalink_sync_cali_rc_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.ChMax, ChMax, sizeof(int16_t)*4);
    tta_array_memcpy(packet.ChMid, ChMid, sizeof(int16_t)*4);
    tta_array_memcpy(packet.ChMin, ChMin, sizeof(int16_t)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_RC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_RC;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_rc message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param ChMax  .
 * @param ChMid  .
 * @param ChMin  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_rc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin)
{
    return _ttalink_sync_cali_rc_pack(dst_addr, src_addr, msg,  simple_time, ChMax, ChMid, ChMin, false);
}

/**
 * @brief Pack a sync_cali_rc message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param ChMax  .
 * @param ChMid  .
 * @param ChMin  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_rc_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin)
{
    return _ttalink_sync_cali_rc_pack(dst_addr, src_addr, msg,  simple_time, ChMax, ChMid, ChMin, true);
}


static inline uint16_t _ttalink_sync_cali_rc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const int16_t *ChMax,const int16_t *ChMid,const int16_t *ChMin, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_RC_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_int16_t_array(buf, 4, ChMax, 4);
    _tta_put_int16_t_array(buf, 12, ChMid, 4);
    _tta_put_int16_t_array(buf, 20, ChMin, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_RC_LEN);
#else
    ttalink_sync_cali_rc_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.ChMax, ChMax, sizeof(int16_t)*4);
    tta_array_memcpy(packet.ChMid, ChMid, sizeof(int16_t)*4);
    tta_array_memcpy(packet.ChMin, ChMin, sizeof(int16_t)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_RC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_RC;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_rc message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param ChMax  .
 * @param ChMid  .
 * @param ChMin  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_rc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const int16_t *ChMax,const int16_t *ChMid,const int16_t *ChMin)
{
    return _ttalink_sync_cali_rc_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, ChMax, ChMid, ChMin, false);
}

/**
 * @brief Pack a sync_cali_rc message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param ChMax  .
 * @param ChMid  .
 * @param ChMin  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_rc_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const int16_t *ChMax,const int16_t *ChMid,const int16_t *ChMin)
{
    return _ttalink_sync_cali_rc_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, ChMax, ChMid, ChMin, true);
}


static inline uint16_t _ttalink_sync_cali_rc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_rc_t* sync_cali_rc, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_rc_pack_nocrc(dst_addr, src_addr, msg, sync_cali_rc->simple_time, sync_cali_rc->ChMax, sync_cali_rc->ChMid, sync_cali_rc->ChMin);
    }else{
        return ttalink_sync_cali_rc_pack(dst_addr, src_addr, msg, sync_cali_rc->simple_time, sync_cali_rc->ChMax, sync_cali_rc->ChMid, sync_cali_rc->ChMin);
    }
    
}

/**
 * @brief Encode a sync_cali_rc struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_rc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_rc_t* sync_cali_rc)
{
    return _ttalink_sync_cali_rc_encode(dst_addr, src_addr, msg, sync_cali_rc, false);
}

/**
 * @brief Encode a sync_cali_rc struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_rc_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_rc_t* sync_cali_rc)
{
    return _ttalink_sync_cali_rc_encode(dst_addr, src_addr, msg, sync_cali_rc, true);
}


static inline uint16_t _ttalink_sync_cali_rc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_rc_t* sync_cali_rc, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_rc_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_cali_rc->simple_time, sync_cali_rc->ChMax, sync_cali_rc->ChMid, sync_cali_rc->ChMin);
    }else{
        return ttalink_sync_cali_rc_pack_chan(dst_addr, src_addr, chan, msg, sync_cali_rc->simple_time, sync_cali_rc->ChMax, sync_cali_rc->ChMid, sync_cali_rc->ChMin);
    }
}

/**
 * @brief Encode a sync_cali_rc struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_rc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_rc_t* sync_cali_rc)
{
    return _ttalink_sync_cali_rc_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_rc, false);
}

/**
 * @brief Encode a sync_cali_rc struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_rc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_rc_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_rc_t* sync_cali_rc)
{
    return _ttalink_sync_cali_rc_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_rc, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_cali_rc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_RC_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_int16_t_array(buf, 4, ChMax, 4);
    _tta_put_int16_t_array(buf, 12, ChMid, 4);
    _tta_put_int16_t_array(buf, 20, ChMin, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_RC, buf, TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_CRC, nocrc);
#else
    ttalink_sync_cali_rc_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.ChMax, ChMax, sizeof(int16_t)*4);
    tta_array_memcpy(packet.ChMid, ChMid, sizeof(int16_t)*4);
    tta_array_memcpy(packet.ChMin, ChMin, sizeof(int16_t)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_RC, (const char *)&packet, TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_rc message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param ChMax  .
 * @param ChMid  .
 * @param ChMin  .
 */
static inline void ttalink_sync_cali_rc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin)
{
    _ttalink_sync_cali_rc_send(dst_addr, src_addr, chan, simple_time, ChMax, ChMid, ChMin, false);
}

/**
 * @brief Send a sync_cali_rc message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param ChMax  .
 * @param ChMid  .
 * @param ChMin  .
 */
static inline void ttalink_sync_cali_rc_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin)
{
    _ttalink_sync_cali_rc_send(dst_addr, src_addr, chan, simple_time, ChMax, ChMid, ChMin, true);
}


static inline void _ttalink_sync_cali_rc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_rc_t* sync_cali_rc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_cali_rc_send_nocrc(dst_addr, src_addr, chan, sync_cali_rc->simple_time, sync_cali_rc->ChMax, sync_cali_rc->ChMid, sync_cali_rc->ChMin);
    }else{
        ttalink_sync_cali_rc_send(dst_addr, src_addr, chan, sync_cali_rc->simple_time, sync_cali_rc->ChMax, sync_cali_rc->ChMid, sync_cali_rc->ChMin);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_RC, (const char *)sync_cali_rc, TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_rc message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_rc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_rc_t* sync_cali_rc)
{
    _ttalink_sync_cali_rc_send_struct(dst_addr, src_addr, chan, sync_cali_rc, false);
}

/**
 * @brief Send a sync_cali_rc message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_rc_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_rc_t* sync_cali_rc)
{
    _ttalink_sync_cali_rc_send_struct(dst_addr, src_addr, chan, sync_cali_rc, true);
}

#if TTALINK_MSG_ID_SYNC_CALI_RC_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_cali_rc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_int16_t_array(buf, 4, ChMax, 4);
    _tta_put_int16_t_array(buf, 12, ChMid, 4);
    _tta_put_int16_t_array(buf, 20, ChMin, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_RC, buf, TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_CRC, nocrc);
#else
    ttalink_sync_cali_rc_t *packet = (ttalink_sync_cali_rc_t *)msgbuf;
    packet->simple_time = simple_time;
    tta_array_memcpy(packet->ChMax, ChMax, sizeof(int16_t)*4);
    tta_array_memcpy(packet->ChMid, ChMid, sizeof(int16_t)*4);
    tta_array_memcpy(packet->ChMin, ChMin, sizeof(int16_t)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_RC, (const char *)packet, TTALINK_MSG_ID_SYNC_CALI_RC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_LEN, TTALINK_MSG_ID_SYNC_CALI_RC_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_cali_rc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin)
{
    _ttalink_sync_cali_rc_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, ChMax, ChMid, ChMin, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_cali_rc_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const int16_t *ChMax, const int16_t *ChMid, const int16_t *ChMin)
{
    _ttalink_sync_cali_rc_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, ChMax, ChMid, ChMin, true);
}
#endif

#endif

// MESSAGE SYNC_CALI_RC UNPACKING


/**
 * @brief Get field simple_time from sync_cali_rc message
 *
 * @return  .
 */
static inline uint32_t ttalink_sync_cali_rc_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ChMax from sync_cali_rc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_rc_get_ChMax(const ttalink_message_t* msg, int16_t *ChMax)
{
    return _TTA_RETURN_int16_t_array(msg, ChMax, 4,  4);
}

/**
 * @brief Get field ChMid from sync_cali_rc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_rc_get_ChMid(const ttalink_message_t* msg, int16_t *ChMid)
{
    return _TTA_RETURN_int16_t_array(msg, ChMid, 4,  12);
}

/**
 * @brief Get field ChMin from sync_cali_rc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_rc_get_ChMin(const ttalink_message_t* msg, int16_t *ChMin)
{
    return _TTA_RETURN_int16_t_array(msg, ChMin, 4,  20);
}

/**
 * @brief Decode a sync_cali_rc message into a struct
 *
 * @param msg The message to decode
 * @param sync_cali_rc C-struct to decode the message contents into
 */
static inline void ttalink_sync_cali_rc_decode(const ttalink_message_t* msg, ttalink_sync_cali_rc_t* sync_cali_rc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_cali_rc->simple_time = ttalink_sync_cali_rc_get_simple_time(msg);
    ttalink_sync_cali_rc_get_ChMax(msg, sync_cali_rc->ChMax);
    ttalink_sync_cali_rc_get_ChMid(msg, sync_cali_rc->ChMid);
    ttalink_sync_cali_rc_get_ChMin(msg, sync_cali_rc->ChMin);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CALI_RC_LEN? msg->len : TTALINK_MSG_ID_SYNC_CALI_RC_LEN;
        memset(sync_cali_rc, 0, TTALINK_MSG_ID_SYNC_CALI_RC_LEN);
    memcpy(sync_cali_rc, _TTA_PAYLOAD(msg), len);
#endif
}
