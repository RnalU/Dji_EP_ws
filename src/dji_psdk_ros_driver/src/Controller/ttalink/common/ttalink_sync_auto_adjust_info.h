#pragma once
// MESSAGE SYNC_AUTO_ADJUST_INFO PACKING

#define TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO 2107

TTAPACKED(
typedef struct __ttalink_sync_auto_adjust_info_t {
 uint32_t simple_time; /*<  . .*/
 uint8_t type; /*<  . .*/
 float rate; /*<  . .*/
 float target; /*<  . .*/
 float plan; /*<  . .*/
 float default_value; /*<  . .*/
 uint8_t status; /*<  . .*/
}) ttalink_sync_auto_adjust_info_t;

#define TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN 22
#define TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN 22
#define TTALINK_MSG_ID_2107_LEN 22
#define TTALINK_MSG_ID_2107_MIN_LEN 22

#define TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC 58
#define TTALINK_MSG_ID_2107_CRC 58



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_AUTO_ADJUST_INFO { \
    2107, \
    "SYNC_AUTO_ADJUST_INFO", \
    7, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_auto_adjust_info_t, simple_time) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_auto_adjust_info_t, type) }, \
         { "rate", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_sync_auto_adjust_info_t, rate) }, \
         { "target", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_sync_auto_adjust_info_t, target) }, \
         { "plan", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_sync_auto_adjust_info_t, plan) }, \
         { "default_value", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_sync_auto_adjust_info_t, default_value) }, \
         { "status", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_sync_auto_adjust_info_t, status) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_AUTO_ADJUST_INFO { \
    "SYNC_AUTO_ADJUST_INFO", \
    7, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_auto_adjust_info_t, simple_time) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_sync_auto_adjust_info_t, type) }, \
         { "rate", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_sync_auto_adjust_info_t, rate) }, \
         { "target", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_sync_auto_adjust_info_t, target) }, \
         { "plan", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_sync_auto_adjust_info_t, plan) }, \
         { "default_value", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_sync_auto_adjust_info_t, default_value) }, \
         { "status", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_sync_auto_adjust_info_t, status) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_auto_adjust_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_float(buf, 5, rate);
    _tta_put_float(buf, 9, target);
    _tta_put_float(buf, 13, plan);
    _tta_put_float(buf, 17, default_value);
    _tta_put_uint8_t(buf, 21, status);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN);
#else
    ttalink_sync_auto_adjust_info_t packet;
    packet.simple_time = simple_time;
    packet.type = type;
    packet.rate = rate;
    packet.target = target;
    packet.plan = plan;
    packet.default_value = default_value;
    packet.status = status;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_auto_adjust_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param type  . .
 * @param rate  . .
 * @param target  . .
 * @param plan  . .
 * @param default_value  . .
 * @param status  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_auto_adjust_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status)
{
    return _ttalink_sync_auto_adjust_info_pack(dst_addr, src_addr, msg,  simple_time, type, rate, target, plan, default_value, status, false);
}

/**
 * @brief Pack a sync_auto_adjust_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  . .
 * @param type  . .
 * @param rate  . .
 * @param target  . .
 * @param plan  . .
 * @param default_value  . .
 * @param status  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_auto_adjust_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status)
{
    return _ttalink_sync_auto_adjust_info_pack(dst_addr, src_addr, msg,  simple_time, type, rate, target, plan, default_value, status, true);
}


static inline uint16_t _ttalink_sync_auto_adjust_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t type,float rate,float target,float plan,float default_value,uint8_t status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_float(buf, 5, rate);
    _tta_put_float(buf, 9, target);
    _tta_put_float(buf, 13, plan);
    _tta_put_float(buf, 17, default_value);
    _tta_put_uint8_t(buf, 21, status);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN);
#else
    ttalink_sync_auto_adjust_info_t packet;
    packet.simple_time = simple_time;
    packet.type = type;
    packet.rate = rate;
    packet.target = target;
    packet.plan = plan;
    packet.default_value = default_value;
    packet.status = status;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_auto_adjust_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param type  . .
 * @param rate  . .
 * @param target  . .
 * @param plan  . .
 * @param default_value  . .
 * @param status  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_auto_adjust_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t type,float rate,float target,float plan,float default_value,uint8_t status)
{
    return _ttalink_sync_auto_adjust_info_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, type, rate, target, plan, default_value, status, false);
}

/**
 * @brief Pack a sync_auto_adjust_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  . .
 * @param type  . .
 * @param rate  . .
 * @param target  . .
 * @param plan  . .
 * @param default_value  . .
 * @param status  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_auto_adjust_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,uint8_t type,float rate,float target,float plan,float default_value,uint8_t status)
{
    return _ttalink_sync_auto_adjust_info_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, type, rate, target, plan, default_value, status, true);
}


static inline uint16_t _ttalink_sync_auto_adjust_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_auto_adjust_info_pack_nocrc(dst_addr, src_addr, msg, sync_auto_adjust_info->simple_time, sync_auto_adjust_info->type, sync_auto_adjust_info->rate, sync_auto_adjust_info->target, sync_auto_adjust_info->plan, sync_auto_adjust_info->default_value, sync_auto_adjust_info->status);
    }else{
        return ttalink_sync_auto_adjust_info_pack(dst_addr, src_addr, msg, sync_auto_adjust_info->simple_time, sync_auto_adjust_info->type, sync_auto_adjust_info->rate, sync_auto_adjust_info->target, sync_auto_adjust_info->plan, sync_auto_adjust_info->default_value, sync_auto_adjust_info->status);
    }
    
}

/**
 * @brief Encode a sync_auto_adjust_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_auto_adjust_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_auto_adjust_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info)
{
    return _ttalink_sync_auto_adjust_info_encode(dst_addr, src_addr, msg, sync_auto_adjust_info, false);
}

/**
 * @brief Encode a sync_auto_adjust_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_auto_adjust_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_auto_adjust_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info)
{
    return _ttalink_sync_auto_adjust_info_encode(dst_addr, src_addr, msg, sync_auto_adjust_info, true);
}


static inline uint16_t _ttalink_sync_auto_adjust_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_auto_adjust_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_auto_adjust_info->simple_time, sync_auto_adjust_info->type, sync_auto_adjust_info->rate, sync_auto_adjust_info->target, sync_auto_adjust_info->plan, sync_auto_adjust_info->default_value, sync_auto_adjust_info->status);
    }else{
        return ttalink_sync_auto_adjust_info_pack_chan(dst_addr, src_addr, chan, msg, sync_auto_adjust_info->simple_time, sync_auto_adjust_info->type, sync_auto_adjust_info->rate, sync_auto_adjust_info->target, sync_auto_adjust_info->plan, sync_auto_adjust_info->default_value, sync_auto_adjust_info->status);
    }
}

/**
 * @brief Encode a sync_auto_adjust_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_auto_adjust_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_auto_adjust_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info)
{
    return _ttalink_sync_auto_adjust_info_encode_chan(dst_addr, src_addr, chan, msg, sync_auto_adjust_info, false);
}

/**
 * @brief Encode a sync_auto_adjust_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_auto_adjust_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_auto_adjust_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info)
{
    return _ttalink_sync_auto_adjust_info_encode_chan(dst_addr, src_addr, chan, msg, sync_auto_adjust_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_auto_adjust_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_float(buf, 5, rate);
    _tta_put_float(buf, 9, target);
    _tta_put_float(buf, 13, plan);
    _tta_put_float(buf, 17, default_value);
    _tta_put_uint8_t(buf, 21, status);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO, buf, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC, nocrc);
#else
    ttalink_sync_auto_adjust_info_t packet;
    packet.simple_time = simple_time;
    packet.type = type;
    packet.rate = rate;
    packet.target = target;
    packet.plan = plan;
    packet.default_value = default_value;
    packet.status = status;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO, (const char *)&packet, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_auto_adjust_info message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param type  . .
 * @param rate  . .
 * @param target  . .
 * @param plan  . .
 * @param default_value  . .
 * @param status  . .
 */
static inline void ttalink_sync_auto_adjust_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status)
{
    _ttalink_sync_auto_adjust_info_send(dst_addr, src_addr, chan, simple_time, type, rate, target, plan, default_value, status, false);
}

/**
 * @brief Send a sync_auto_adjust_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  . .
 * @param type  . .
 * @param rate  . .
 * @param target  . .
 * @param plan  . .
 * @param default_value  . .
 * @param status  . .
 */
static inline void ttalink_sync_auto_adjust_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status)
{
    _ttalink_sync_auto_adjust_info_send(dst_addr, src_addr, chan, simple_time, type, rate, target, plan, default_value, status, true);
}


static inline void _ttalink_sync_auto_adjust_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_auto_adjust_info_send_nocrc(dst_addr, src_addr, chan, sync_auto_adjust_info->simple_time, sync_auto_adjust_info->type, sync_auto_adjust_info->rate, sync_auto_adjust_info->target, sync_auto_adjust_info->plan, sync_auto_adjust_info->default_value, sync_auto_adjust_info->status);
    }else{
        ttalink_sync_auto_adjust_info_send(dst_addr, src_addr, chan, sync_auto_adjust_info->simple_time, sync_auto_adjust_info->type, sync_auto_adjust_info->rate, sync_auto_adjust_info->target, sync_auto_adjust_info->plan, sync_auto_adjust_info->default_value, sync_auto_adjust_info->status);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO, (const char *)sync_auto_adjust_info, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_auto_adjust_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_auto_adjust_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info)
{
    _ttalink_sync_auto_adjust_info_send_struct(dst_addr, src_addr, chan, sync_auto_adjust_info, false);
}

/**
 * @brief Send a sync_auto_adjust_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_auto_adjust_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info)
{
    _ttalink_sync_auto_adjust_info_send_struct(dst_addr, src_addr, chan, sync_auto_adjust_info, true);
}

#if TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_auto_adjust_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_float(buf, 5, rate);
    _tta_put_float(buf, 9, target);
    _tta_put_float(buf, 13, plan);
    _tta_put_float(buf, 17, default_value);
    _tta_put_uint8_t(buf, 21, status);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO, buf, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC, nocrc);
#else
    ttalink_sync_auto_adjust_info_t *packet = (ttalink_sync_auto_adjust_info_t *)msgbuf;
    packet->simple_time = simple_time;
    packet->type = type;
    packet->rate = rate;
    packet->target = target;
    packet->plan = plan;
    packet->default_value = default_value;
    packet->status = status;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO, (const char *)packet, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_auto_adjust_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status)
{
    _ttalink_sync_auto_adjust_info_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, type, rate, target, plan, default_value, status, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_auto_adjust_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, uint8_t type, float rate, float target, float plan, float default_value, uint8_t status)
{
    _ttalink_sync_auto_adjust_info_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, type, rate, target, plan, default_value, status, true);
}
#endif

#endif

// MESSAGE SYNC_AUTO_ADJUST_INFO UNPACKING


/**
 * @brief Get field simple_time from sync_auto_adjust_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_auto_adjust_info_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field type from sync_auto_adjust_info message
 *
 * @return  . .
 */
static inline uint8_t ttalink_sync_auto_adjust_info_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field rate from sync_auto_adjust_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_auto_adjust_info_get_rate(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Get field target from sync_auto_adjust_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_auto_adjust_info_get_target(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field plan from sync_auto_adjust_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_auto_adjust_info_get_plan(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field default_value from sync_auto_adjust_info message
 *
 * @return  . .
 */
static inline float ttalink_sync_auto_adjust_info_get_default_value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Get field status from sync_auto_adjust_info message
 *
 * @return  . .
 */
static inline uint8_t ttalink_sync_auto_adjust_info_get_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a sync_auto_adjust_info message into a struct
 *
 * @param msg The message to decode
 * @param sync_auto_adjust_info C-struct to decode the message contents into
 */
static inline void ttalink_sync_auto_adjust_info_decode(const ttalink_message_t* msg, ttalink_sync_auto_adjust_info_t* sync_auto_adjust_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_auto_adjust_info->simple_time = ttalink_sync_auto_adjust_info_get_simple_time(msg);
    sync_auto_adjust_info->type = ttalink_sync_auto_adjust_info_get_type(msg);
    sync_auto_adjust_info->rate = ttalink_sync_auto_adjust_info_get_rate(msg);
    sync_auto_adjust_info->target = ttalink_sync_auto_adjust_info_get_target(msg);
    sync_auto_adjust_info->plan = ttalink_sync_auto_adjust_info_get_plan(msg);
    sync_auto_adjust_info->default_value = ttalink_sync_auto_adjust_info_get_default_value(msg);
    sync_auto_adjust_info->status = ttalink_sync_auto_adjust_info_get_status(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN? msg->len : TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN;
        memset(sync_auto_adjust_info, 0, TTALINK_MSG_ID_SYNC_AUTO_ADJUST_INFO_LEN);
    memcpy(sync_auto_adjust_info, _TTA_PAYLOAD(msg), len);
#endif
}
