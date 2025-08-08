#pragma once
// MESSAGE GENERAL_STATUS PACKING

#define TTALINK_MSG_ID_GENERAL_STATUS 4103

TTAPACKED(
typedef struct __ttalink_general_status_t {
 int64_t sn; /*< 设备序列号.*/
 uint8_t device_type; /*< 设备类型.*/
 int8_t status_id[20]; /*<   */
 double status_value; /*< .*/
}) ttalink_general_status_t;

#define TTALINK_MSG_ID_GENERAL_STATUS_LEN 37
#define TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN 37
#define TTALINK_MSG_ID_4103_LEN 37
#define TTALINK_MSG_ID_4103_MIN_LEN 37

#define TTALINK_MSG_ID_GENERAL_STATUS_CRC 117
#define TTALINK_MSG_ID_4103_CRC 117

#define TTALINK_MSG_GENERAL_STATUS_FIELD_STATUS_ID_LEN 20

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_STATUS { \
    4103, \
    "GENERAL_STATUS", \
    4, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_status_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_status_t, device_type) }, \
         { "status_id", NULL, TTALINK_TYPE_INT8_T, 20, 9, offsetof(ttalink_general_status_t, status_id) }, \
         { "status_value", NULL, TTALINK_TYPE_DOUBLE, 0, 29, offsetof(ttalink_general_status_t, status_value) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_STATUS { \
    "GENERAL_STATUS", \
    4, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_status_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_status_t, device_type) }, \
         { "status_id", NULL, TTALINK_TYPE_INT8_T, 20, 9, offsetof(ttalink_general_status_t, status_id) }, \
         { "status_value", NULL, TTALINK_TYPE_DOUBLE, 0, 29, offsetof(ttalink_general_status_t, status_value) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_STATUS_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 29, status_value);
    _tta_put_int8_t_array(buf, 9, status_id, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_STATUS_LEN);
#else
    ttalink_general_status_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.status_value = status_value;
    tta_array_memcpy(packet.status_id, status_id, sizeof(int8_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_STATUS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN, TTALINK_MSG_ID_GENERAL_STATUS_LEN, TTALINK_MSG_ID_GENERAL_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a general_status message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param status_id   
 * @param status_value .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_status_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value)
{
    return _ttalink_general_status_pack(dst_addr, src_addr, msg,  sn, device_type, status_id, status_value, false);
}

/**
 * @brief Pack a general_status message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param status_id   
 * @param status_value .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_status_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value)
{
    return _ttalink_general_status_pack(dst_addr, src_addr, msg,  sn, device_type, status_id, status_value, true);
}


static inline uint16_t _ttalink_general_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *status_id,double status_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_STATUS_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 29, status_value);
    _tta_put_int8_t_array(buf, 9, status_id, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_STATUS_LEN);
#else
    ttalink_general_status_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.status_value = status_value;
    tta_array_memcpy(packet.status_id, status_id, sizeof(int8_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_STATUS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_STATUS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN, TTALINK_MSG_ID_GENERAL_STATUS_LEN, TTALINK_MSG_ID_GENERAL_STATUS_CRC, nocrc);
}

/**
 * @brief Pack a general_status message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param status_id   
 * @param status_value .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_status_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *status_id,double status_value)
{
    return _ttalink_general_status_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, status_id, status_value, false);
}

/**
 * @brief Pack a general_status message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param status_id   
 * @param status_value .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_status_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *status_id,double status_value)
{
    return _ttalink_general_status_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, status_id, status_value, true);
}


static inline uint16_t _ttalink_general_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_status_t* general_status, bool nocrc)
{
    if(nocrc){
        return ttalink_general_status_pack_nocrc(dst_addr, src_addr, msg, general_status->sn, general_status->device_type, general_status->status_id, general_status->status_value);
    }else{
        return ttalink_general_status_pack(dst_addr, src_addr, msg, general_status->sn, general_status->device_type, general_status->status_id, general_status->status_value);
    }
    
}

/**
 * @brief Encode a general_status struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_status_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_status_t* general_status)
{
    return _ttalink_general_status_encode(dst_addr, src_addr, msg, general_status, false);
}

/**
 * @brief Encode a general_status struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_status_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_status_t* general_status)
{
    return _ttalink_general_status_encode(dst_addr, src_addr, msg, general_status, true);
}


static inline uint16_t _ttalink_general_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_status_t* general_status, bool nocrc)
{
    if(nocrc){
        return ttalink_general_status_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_status->sn, general_status->device_type, general_status->status_id, general_status->status_value);
    }else{
        return ttalink_general_status_pack_chan(dst_addr, src_addr, chan, msg, general_status->sn, general_status->device_type, general_status->status_id, general_status->status_value);
    }
}

/**
 * @brief Encode a general_status struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_status_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_status_t* general_status)
{
    return _ttalink_general_status_encode_chan(dst_addr, src_addr, chan, msg, general_status, false);
}

/**
 * @brief Encode a general_status struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_status C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_status_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_status_t* general_status)
{
    return _ttalink_general_status_encode_chan(dst_addr, src_addr, chan, msg, general_status, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_STATUS_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 29, status_value);
    _tta_put_int8_t_array(buf, 9, status_id, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_STATUS, buf, TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN, TTALINK_MSG_ID_GENERAL_STATUS_LEN, TTALINK_MSG_ID_GENERAL_STATUS_CRC, nocrc);
#else
    ttalink_general_status_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.status_value = status_value;
    tta_array_memcpy(packet.status_id, status_id, sizeof(int8_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_STATUS, (const char *)&packet, TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN, TTALINK_MSG_ID_GENERAL_STATUS_LEN, TTALINK_MSG_ID_GENERAL_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_status message
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param status_id   
 * @param status_value .
 */
static inline void ttalink_general_status_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value)
{
    _ttalink_general_status_send(dst_addr, src_addr, chan, sn, device_type, status_id, status_value, false);
}

/**
 * @brief Send a general_status message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param status_id   
 * @param status_value .
 */
static inline void ttalink_general_status_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value)
{
    _ttalink_general_status_send(dst_addr, src_addr, chan, sn, device_type, status_id, status_value, true);
}


static inline void _ttalink_general_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_status_t* general_status, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_status_send_nocrc(dst_addr, src_addr, chan, general_status->sn, general_status->device_type, general_status->status_id, general_status->status_value);
    }else{
        ttalink_general_status_send(dst_addr, src_addr, chan, general_status->sn, general_status->device_type, general_status->status_id, general_status->status_value);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_STATUS, (const char *)general_status, TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN, TTALINK_MSG_ID_GENERAL_STATUS_LEN, TTALINK_MSG_ID_GENERAL_STATUS_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_status message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_status_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_status_t* general_status)
{
    _ttalink_general_status_send_struct(dst_addr, src_addr, chan, general_status, false);
}

/**
 * @brief Send a general_status message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_status_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_status_t* general_status)
{
    _ttalink_general_status_send_struct(dst_addr, src_addr, chan, general_status, true);
}

#if TTALINK_MSG_ID_GENERAL_STATUS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 29, status_value);
    _tta_put_int8_t_array(buf, 9, status_id, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_STATUS, buf, TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN, TTALINK_MSG_ID_GENERAL_STATUS_LEN, TTALINK_MSG_ID_GENERAL_STATUS_CRC, nocrc);
#else
    ttalink_general_status_t *packet = (ttalink_general_status_t *)msgbuf;
    packet->sn = sn;
    packet->device_type = device_type;
    packet->status_value = status_value;
    tta_array_memcpy(packet->status_id, status_id, sizeof(int8_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_STATUS, (const char *)packet, TTALINK_MSG_ID_GENERAL_STATUS_MIN_LEN, TTALINK_MSG_ID_GENERAL_STATUS_LEN, TTALINK_MSG_ID_GENERAL_STATUS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_status_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value)
{
    _ttalink_general_status_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, status_id, status_value, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_status_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *status_id, double status_value)
{
    _ttalink_general_status_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, status_id, status_value, true);
}
#endif

#endif

// MESSAGE GENERAL_STATUS UNPACKING


/**
 * @brief Get field sn from general_status message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_general_status_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field device_type from general_status message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_general_status_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field status_id from general_status message
 *
 * @return   
 */
static inline uint16_t ttalink_general_status_get_status_id(const ttalink_message_t* msg, int8_t *status_id)
{
    return _TTA_RETURN_int8_t_array(msg, status_id, 20,  9);
}

/**
 * @brief Get field status_value from general_status message
 *
 * @return .
 */
static inline double ttalink_general_status_get_status_value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  29);
}

/**
 * @brief Decode a general_status message into a struct
 *
 * @param msg The message to decode
 * @param general_status C-struct to decode the message contents into
 */
static inline void ttalink_general_status_decode(const ttalink_message_t* msg, ttalink_general_status_t* general_status)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_status->sn = ttalink_general_status_get_sn(msg);
    general_status->device_type = ttalink_general_status_get_device_type(msg);
    ttalink_general_status_get_status_id(msg, general_status->status_id);
    general_status->status_value = ttalink_general_status_get_status_value(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_STATUS_LEN? msg->len : TTALINK_MSG_ID_GENERAL_STATUS_LEN;
        memset(general_status, 0, TTALINK_MSG_ID_GENERAL_STATUS_LEN);
    memcpy(general_status, _TTA_PAYLOAD(msg), len);
#endif
}
