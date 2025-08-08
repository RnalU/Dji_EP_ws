#pragma once
// MESSAGE GENERAL_PARAM_SET PACKING

#define TTALINK_MSG_ID_GENERAL_PARAM_SET 4102

TTAPACKED(
typedef struct __ttalink_general_param_set_t {
 int64_t sn; /*< 设备序列号.*/
 uint8_t device_type; /*< 设备类型.*/
 int8_t param_id[32]; /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.*/
 double param_value; /*< Onboard parameter value .*/
 uint8_t param_type; /*< Onboard parameter type .*/
}) ttalink_general_param_set_t;

#define TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN 50
#define TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN 50
#define TTALINK_MSG_ID_4102_LEN 50
#define TTALINK_MSG_ID_4102_MIN_LEN 50

#define TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC 190
#define TTALINK_MSG_ID_4102_CRC 190

#define TTALINK_MSG_GENERAL_PARAM_SET_FIELD_PARAM_ID_LEN 32

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_PARAM_SET { \
    4102, \
    "GENERAL_PARAM_SET", \
    5, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_param_set_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_param_set_t, device_type) }, \
         { "param_id", NULL, TTALINK_TYPE_INT8_T, 32, 9, offsetof(ttalink_general_param_set_t, param_id) }, \
         { "param_value", NULL, TTALINK_TYPE_DOUBLE, 0, 41, offsetof(ttalink_general_param_set_t, param_value) }, \
         { "param_type", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_general_param_set_t, param_type) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_PARAM_SET { \
    "GENERAL_PARAM_SET", \
    5, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_param_set_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_param_set_t, device_type) }, \
         { "param_id", NULL, TTALINK_TYPE_INT8_T, 32, 9, offsetof(ttalink_general_param_set_t, param_id) }, \
         { "param_value", NULL, TTALINK_TYPE_DOUBLE, 0, 41, offsetof(ttalink_general_param_set_t, param_value) }, \
         { "param_type", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_general_param_set_t, param_type) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_param_set_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 41, param_value);
    _tta_put_uint8_t(buf, 49, param_type);
    _tta_put_int8_t_array(buf, 9, param_id, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN);
#else
    ttalink_general_param_set_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.param_value = param_value;
    packet.param_type = param_type;
    tta_array_memcpy(packet.param_id, param_id, sizeof(int8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_PARAM_SET;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC, nocrc);
}

/**
 * @brief Pack a general_param_set message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.
 * @param param_value Onboard parameter value .
 * @param param_type Onboard parameter type .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_set_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type)
{
    return _ttalink_general_param_set_pack(dst_addr, src_addr, msg,  sn, device_type, param_id, param_value, param_type, false);
}

/**
 * @brief Pack a general_param_set message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.
 * @param param_value Onboard parameter value .
 * @param param_type Onboard parameter type .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_set_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type)
{
    return _ttalink_general_param_set_pack(dst_addr, src_addr, msg,  sn, device_type, param_id, param_value, param_type, true);
}


static inline uint16_t _ttalink_general_param_set_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *param_id,double param_value,uint8_t param_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 41, param_value);
    _tta_put_uint8_t(buf, 49, param_type);
    _tta_put_int8_t_array(buf, 9, param_id, 32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN);
#else
    ttalink_general_param_set_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.param_value = param_value;
    packet.param_type = param_type;
    tta_array_memcpy(packet.param_id, param_id, sizeof(int8_t)*32);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_PARAM_SET;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC, nocrc);
}

/**
 * @brief Pack a general_param_set message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.
 * @param param_value Onboard parameter value .
 * @param param_type Onboard parameter type .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_set_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *param_id,double param_value,uint8_t param_type)
{
    return _ttalink_general_param_set_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, param_id, param_value, param_type, false);
}

/**
 * @brief Pack a general_param_set message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.
 * @param param_value Onboard parameter value .
 * @param param_type Onboard parameter type .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_set_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,const int8_t *param_id,double param_value,uint8_t param_type)
{
    return _ttalink_general_param_set_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, param_id, param_value, param_type, true);
}


static inline uint16_t _ttalink_general_param_set_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_param_set_t* general_param_set, bool nocrc)
{
    if(nocrc){
        return ttalink_general_param_set_pack_nocrc(dst_addr, src_addr, msg, general_param_set->sn, general_param_set->device_type, general_param_set->param_id, general_param_set->param_value, general_param_set->param_type);
    }else{
        return ttalink_general_param_set_pack(dst_addr, src_addr, msg, general_param_set->sn, general_param_set->device_type, general_param_set->param_id, general_param_set->param_value, general_param_set->param_type);
    }
    
}

/**
 * @brief Encode a general_param_set struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_param_set C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_set_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_param_set_t* general_param_set)
{
    return _ttalink_general_param_set_encode(dst_addr, src_addr, msg, general_param_set, false);
}

/**
 * @brief Encode a general_param_set struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_param_set C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_set_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_param_set_t* general_param_set)
{
    return _ttalink_general_param_set_encode(dst_addr, src_addr, msg, general_param_set, true);
}


static inline uint16_t _ttalink_general_param_set_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_param_set_t* general_param_set, bool nocrc)
{
    if(nocrc){
        return ttalink_general_param_set_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_param_set->sn, general_param_set->device_type, general_param_set->param_id, general_param_set->param_value, general_param_set->param_type);
    }else{
        return ttalink_general_param_set_pack_chan(dst_addr, src_addr, chan, msg, general_param_set->sn, general_param_set->device_type, general_param_set->param_id, general_param_set->param_value, general_param_set->param_type);
    }
}

/**
 * @brief Encode a general_param_set struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_param_set C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_set_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_param_set_t* general_param_set)
{
    return _ttalink_general_param_set_encode_chan(dst_addr, src_addr, chan, msg, general_param_set, false);
}

/**
 * @brief Encode a general_param_set struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_param_set C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_set_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_param_set_t* general_param_set)
{
    return _ttalink_general_param_set_encode_chan(dst_addr, src_addr, chan, msg, general_param_set, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_param_set_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 41, param_value);
    _tta_put_uint8_t(buf, 49, param_type);
    _tta_put_int8_t_array(buf, 9, param_id, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_SET, buf, TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC, nocrc);
#else
    ttalink_general_param_set_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.param_value = param_value;
    packet.param_type = param_type;
    tta_array_memcpy(packet.param_id, param_id, sizeof(int8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_SET, (const char *)&packet, TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_param_set message
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.
 * @param param_value Onboard parameter value .
 * @param param_type Onboard parameter type .
 */
static inline void ttalink_general_param_set_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type)
{
    _ttalink_general_param_set_send(dst_addr, src_addr, chan, sn, device_type, param_id, param_value, param_type, false);
}

/**
 * @brief Send a general_param_set message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.
 * @param param_value Onboard parameter value .
 * @param param_type Onboard parameter type .
 */
static inline void ttalink_general_param_set_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type)
{
    _ttalink_general_param_set_send(dst_addr, src_addr, chan, sn, device_type, param_id, param_value, param_type, true);
}


static inline void _ttalink_general_param_set_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_param_set_t* general_param_set, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_param_set_send_nocrc(dst_addr, src_addr, chan, general_param_set->sn, general_param_set->device_type, general_param_set->param_id, general_param_set->param_value, general_param_set->param_type);
    }else{
        ttalink_general_param_set_send(dst_addr, src_addr, chan, general_param_set->sn, general_param_set->device_type, general_param_set->param_id, general_param_set->param_value, general_param_set->param_type);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_SET, (const char *)general_param_set, TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_param_set message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_param_set_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_param_set_t* general_param_set)
{
    _ttalink_general_param_set_send_struct(dst_addr, src_addr, chan, general_param_set, false);
}

/**
 * @brief Send a general_param_set message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_param_set_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_param_set_t* general_param_set)
{
    _ttalink_general_param_set_send_struct(dst_addr, src_addr, chan, general_param_set, true);
}

#if TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_param_set_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_double(buf, 41, param_value);
    _tta_put_uint8_t(buf, 49, param_type);
    _tta_put_int8_t_array(buf, 9, param_id, 32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_SET, buf, TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC, nocrc);
#else
    ttalink_general_param_set_t *packet = (ttalink_general_param_set_t *)msgbuf;
    packet->sn = sn;
    packet->device_type = device_type;
    packet->param_value = param_value;
    packet->param_type = param_type;
    tta_array_memcpy(packet->param_id, param_id, sizeof(int8_t)*32);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_SET, (const char *)packet, TTALINK_MSG_ID_GENERAL_PARAM_SET_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN, TTALINK_MSG_ID_GENERAL_PARAM_SET_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_param_set_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type)
{
    _ttalink_general_param_set_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, param_id, param_value, param_type, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_param_set_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, const int8_t *param_id, double param_value, uint8_t param_type)
{
    _ttalink_general_param_set_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, param_id, param_value, param_type, true);
}
#endif

#endif

// MESSAGE GENERAL_PARAM_SET UNPACKING


/**
 * @brief Get field sn from general_param_set message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_general_param_set_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field device_type from general_param_set message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_general_param_set_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field param_id from general_param_set message
 *
 * @return  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string.
 */
static inline uint16_t ttalink_general_param_set_get_param_id(const ttalink_message_t* msg, int8_t *param_id)
{
    return _TTA_RETURN_int8_t_array(msg, param_id, 32,  9);
}

/**
 * @brief Get field param_value from general_param_set message
 *
 * @return Onboard parameter value .
 */
static inline double ttalink_general_param_set_get_param_value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  41);
}

/**
 * @brief Get field param_type from general_param_set message
 *
 * @return Onboard parameter type .
 */
static inline uint8_t ttalink_general_param_set_get_param_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Decode a general_param_set message into a struct
 *
 * @param msg The message to decode
 * @param general_param_set C-struct to decode the message contents into
 */
static inline void ttalink_general_param_set_decode(const ttalink_message_t* msg, ttalink_general_param_set_t* general_param_set)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_param_set->sn = ttalink_general_param_set_get_sn(msg);
    general_param_set->device_type = ttalink_general_param_set_get_device_type(msg);
    ttalink_general_param_set_get_param_id(msg, general_param_set->param_id);
    general_param_set->param_value = ttalink_general_param_set_get_param_value(msg);
    general_param_set->param_type = ttalink_general_param_set_get_param_type(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN? msg->len : TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN;
        memset(general_param_set, 0, TTALINK_MSG_ID_GENERAL_PARAM_SET_LEN);
    memcpy(general_param_set, _TTA_PAYLOAD(msg), len);
#endif
}
