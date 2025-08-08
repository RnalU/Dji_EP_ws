#pragma once
// MESSAGE POD_260_GET_PRODUCT_INFO PACKING

#define TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO 10260

TTAPACKED(
typedef struct __ttalink_pod_260_get_product_info_t {
 uint8_t cmd; /*<  .*/
 uint8_t ack; /*<  .*/
 uint8_t ack_name[32]; /*<  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.*/
 uint8_t ack_id[16]; /*<  Product ID.*/
 uint8_t ack_accout[64]; /*<  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.*/
}) ttalink_pod_260_get_product_info_t;

#define TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN 114
#define TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN 114
#define TTALINK_MSG_ID_10260_LEN 114
#define TTALINK_MSG_ID_10260_MIN_LEN 114

#define TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC 33
#define TTALINK_MSG_ID_10260_CRC 33

#define TTALINK_MSG_POD_260_GET_PRODUCT_INFO_FIELD_ACK_NAME_LEN 32
#define TTALINK_MSG_POD_260_GET_PRODUCT_INFO_FIELD_ACK_ID_LEN 16
#define TTALINK_MSG_POD_260_GET_PRODUCT_INFO_FIELD_ACK_ACCOUT_LEN 64

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_260_GET_PRODUCT_INFO { \
    10260, \
    "POD_260_GET_PRODUCT_INFO", \
    5, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_260_get_product_info_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_260_get_product_info_t, ack) }, \
         { "ack_name", NULL, TTALINK_TYPE_UINT8_T, 32, 2, offsetof(ttalink_pod_260_get_product_info_t, ack_name) }, \
         { "ack_id", NULL, TTALINK_TYPE_UINT8_T, 16, 34, offsetof(ttalink_pod_260_get_product_info_t, ack_id) }, \
         { "ack_accout", NULL, TTALINK_TYPE_UINT8_T, 64, 50, offsetof(ttalink_pod_260_get_product_info_t, ack_accout) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_260_GET_PRODUCT_INFO { \
    "POD_260_GET_PRODUCT_INFO", \
    5, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_260_get_product_info_t, cmd) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_260_get_product_info_t, ack) }, \
         { "ack_name", NULL, TTALINK_TYPE_UINT8_T, 32, 2, offsetof(ttalink_pod_260_get_product_info_t, ack_name) }, \
         { "ack_id", NULL, TTALINK_TYPE_UINT8_T, 16, 34, offsetof(ttalink_pod_260_get_product_info_t, ack_id) }, \
         { "ack_accout", NULL, TTALINK_TYPE_UINT8_T, 64, 50, offsetof(ttalink_pod_260_get_product_info_t, ack_accout) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_260_get_product_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_name, 32);
    _tta_put_uint8_t_array(buf, 34, ack_id, 16);
    _tta_put_uint8_t_array(buf, 50, ack_accout, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN);
#else
    ttalink_pod_260_get_product_info_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    tta_array_memcpy(packet.ack_name, ack_name, sizeof(uint8_t)*32);
    tta_array_memcpy(packet.ack_id, ack_id, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.ack_accout, ack_accout, sizeof(uint8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC, nocrc);
}

/**
 * @brief Pack a pod_260_get_product_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_name  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.
 * @param ack_id  Product ID.
 * @param ack_accout  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_260_get_product_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout)
{
    return _ttalink_pod_260_get_product_info_pack(dst_addr, src_addr, msg,  cmd, ack, ack_name, ack_id, ack_accout, false);
}

/**
 * @brief Pack a pod_260_get_product_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_name  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.
 * @param ack_id  Product ID.
 * @param ack_accout  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_260_get_product_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout)
{
    return _ttalink_pod_260_get_product_info_pack(dst_addr, src_addr, msg,  cmd, ack, ack_name, ack_id, ack_accout, true);
}


static inline uint16_t _ttalink_pod_260_get_product_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,const uint8_t *ack_name,const uint8_t *ack_id,const uint8_t *ack_accout, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_name, 32);
    _tta_put_uint8_t_array(buf, 34, ack_id, 16);
    _tta_put_uint8_t_array(buf, 50, ack_accout, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN);
#else
    ttalink_pod_260_get_product_info_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    tta_array_memcpy(packet.ack_name, ack_name, sizeof(uint8_t)*32);
    tta_array_memcpy(packet.ack_id, ack_id, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.ack_accout, ack_accout, sizeof(uint8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC, nocrc);
}

/**
 * @brief Pack a pod_260_get_product_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_name  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.
 * @param ack_id  Product ID.
 * @param ack_accout  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_260_get_product_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,const uint8_t *ack_name,const uint8_t *ack_id,const uint8_t *ack_accout)
{
    return _ttalink_pod_260_get_product_info_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_name, ack_id, ack_accout, false);
}

/**
 * @brief Pack a pod_260_get_product_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param ack  .
 * @param ack_name  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.
 * @param ack_id  Product ID.
 * @param ack_accout  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_260_get_product_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t ack,const uint8_t *ack_name,const uint8_t *ack_id,const uint8_t *ack_accout)
{
    return _ttalink_pod_260_get_product_info_pack_chan(dst_addr, src_addr, chan, msg,  cmd, ack, ack_name, ack_id, ack_accout, true);
}


static inline uint16_t _ttalink_pod_260_get_product_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_260_get_product_info_pack_nocrc(dst_addr, src_addr, msg, pod_260_get_product_info->cmd, pod_260_get_product_info->ack, pod_260_get_product_info->ack_name, pod_260_get_product_info->ack_id, pod_260_get_product_info->ack_accout);
    }else{
        return ttalink_pod_260_get_product_info_pack(dst_addr, src_addr, msg, pod_260_get_product_info->cmd, pod_260_get_product_info->ack, pod_260_get_product_info->ack_name, pod_260_get_product_info->ack_id, pod_260_get_product_info->ack_accout);
    }
    
}

/**
 * @brief Encode a pod_260_get_product_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_260_get_product_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_260_get_product_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info)
{
    return _ttalink_pod_260_get_product_info_encode(dst_addr, src_addr, msg, pod_260_get_product_info, false);
}

/**
 * @brief Encode a pod_260_get_product_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_260_get_product_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_260_get_product_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info)
{
    return _ttalink_pod_260_get_product_info_encode(dst_addr, src_addr, msg, pod_260_get_product_info, true);
}


static inline uint16_t _ttalink_pod_260_get_product_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_260_get_product_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_260_get_product_info->cmd, pod_260_get_product_info->ack, pod_260_get_product_info->ack_name, pod_260_get_product_info->ack_id, pod_260_get_product_info->ack_accout);
    }else{
        return ttalink_pod_260_get_product_info_pack_chan(dst_addr, src_addr, chan, msg, pod_260_get_product_info->cmd, pod_260_get_product_info->ack, pod_260_get_product_info->ack_name, pod_260_get_product_info->ack_id, pod_260_get_product_info->ack_accout);
    }
}

/**
 * @brief Encode a pod_260_get_product_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_260_get_product_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_260_get_product_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info)
{
    return _ttalink_pod_260_get_product_info_encode_chan(dst_addr, src_addr, chan, msg, pod_260_get_product_info, false);
}

/**
 * @brief Encode a pod_260_get_product_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_260_get_product_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_260_get_product_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info)
{
    return _ttalink_pod_260_get_product_info_encode_chan(dst_addr, src_addr, chan, msg, pod_260_get_product_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_260_get_product_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_name, 32);
    _tta_put_uint8_t_array(buf, 34, ack_id, 16);
    _tta_put_uint8_t_array(buf, 50, ack_accout, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO, buf, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC, nocrc);
#else
    ttalink_pod_260_get_product_info_t packet;
    packet.cmd = cmd;
    packet.ack = ack;
    tta_array_memcpy(packet.ack_name, ack_name, sizeof(uint8_t)*32);
    tta_array_memcpy(packet.ack_id, ack_id, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.ack_accout, ack_accout, sizeof(uint8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO, (const char *)&packet, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_260_get_product_info message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_name  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.
 * @param ack_id  Product ID.
 * @param ack_accout  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.
 */
static inline void ttalink_pod_260_get_product_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout)
{
    _ttalink_pod_260_get_product_info_send(dst_addr, src_addr, chan, cmd, ack, ack_name, ack_id, ack_accout, false);
}

/**
 * @brief Send a pod_260_get_product_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param ack  .
 * @param ack_name  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.
 * @param ack_id  Product ID.
 * @param ack_accout  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.
 */
static inline void ttalink_pod_260_get_product_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout)
{
    _ttalink_pod_260_get_product_info_send(dst_addr, src_addr, chan, cmd, ack, ack_name, ack_id, ack_accout, true);
}


static inline void _ttalink_pod_260_get_product_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_260_get_product_info_send_nocrc(dst_addr, src_addr, chan, pod_260_get_product_info->cmd, pod_260_get_product_info->ack, pod_260_get_product_info->ack_name, pod_260_get_product_info->ack_id, pod_260_get_product_info->ack_accout);
    }else{
        ttalink_pod_260_get_product_info_send(dst_addr, src_addr, chan, pod_260_get_product_info->cmd, pod_260_get_product_info->ack, pod_260_get_product_info->ack_name, pod_260_get_product_info->ack_id, pod_260_get_product_info->ack_accout);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO, (const char *)pod_260_get_product_info, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_260_get_product_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_260_get_product_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info)
{
    _ttalink_pod_260_get_product_info_send_struct(dst_addr, src_addr, chan, pod_260_get_product_info, false);
}

/**
 * @brief Send a pod_260_get_product_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_260_get_product_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_260_get_product_info_t* pod_260_get_product_info)
{
    _ttalink_pod_260_get_product_info_send_struct(dst_addr, src_addr, chan, pod_260_get_product_info, true);
}

#if TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_260_get_product_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, ack);
    _tta_put_uint8_t_array(buf, 2, ack_name, 32);
    _tta_put_uint8_t_array(buf, 34, ack_id, 16);
    _tta_put_uint8_t_array(buf, 50, ack_accout, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO, buf, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC, nocrc);
#else
    ttalink_pod_260_get_product_info_t *packet = (ttalink_pod_260_get_product_info_t *)msgbuf;
    packet->cmd = cmd;
    packet->ack = ack;
    tta_array_memcpy(packet->ack_name, ack_name, sizeof(uint8_t)*32);
    tta_array_memcpy(packet->ack_id, ack_id, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->ack_accout, ack_accout, sizeof(uint8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO, (const char *)packet, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_MIN_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_260_get_product_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout)
{
    _ttalink_pod_260_get_product_info_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_name, ack_id, ack_accout, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_260_get_product_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t ack, const uint8_t *ack_name, const uint8_t *ack_id, const uint8_t *ack_accout)
{
    _ttalink_pod_260_get_product_info_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, ack, ack_name, ack_id, ack_accout, true);
}
#endif

#endif

// MESSAGE POD_260_GET_PRODUCT_INFO UNPACKING


/**
 * @brief Get field cmd from pod_260_get_product_info message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_260_get_product_info_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from pod_260_get_product_info message
 *
 * @return  .
 */
static inline uint8_t ttalink_pod_260_get_product_info_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ack_name from pod_260_get_product_info message
 *
 * @return  Product name, with "\ 0" as the ending character, the maximum length of product name is 32 bytes.
 */
static inline uint16_t ttalink_pod_260_get_product_info_get_ack_name(const ttalink_message_t* msg, uint8_t *ack_name)
{
    return _TTA_RETURN_uint8_t_array(msg, ack_name, 32,  2);
}

/**
 * @brief Get field ack_id from pod_260_get_product_info message
 *
 * @return  Product ID.
 */
static inline uint16_t ttalink_pod_260_get_product_info_get_ack_id(const ttalink_message_t* msg, uint8_t *ack_id)
{
    return _TTA_RETURN_uint8_t_array(msg, ack_id, 16,  34);
}

/**
 * @brief Get field ack_accout from pod_260_get_product_info message
 *
 * @return  Developer account, with "\ 0" as the ending character, the maximum length of developer account is 64 bytes.
 */
static inline uint16_t ttalink_pod_260_get_product_info_get_ack_accout(const ttalink_message_t* msg, uint8_t *ack_accout)
{
    return _TTA_RETURN_uint8_t_array(msg, ack_accout, 64,  50);
}

/**
 * @brief Decode a pod_260_get_product_info message into a struct
 *
 * @param msg The message to decode
 * @param pod_260_get_product_info C-struct to decode the message contents into
 */
static inline void ttalink_pod_260_get_product_info_decode(const ttalink_message_t* msg, ttalink_pod_260_get_product_info_t* pod_260_get_product_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_260_get_product_info->cmd = ttalink_pod_260_get_product_info_get_cmd(msg);
    pod_260_get_product_info->ack = ttalink_pod_260_get_product_info_get_ack(msg);
    ttalink_pod_260_get_product_info_get_ack_name(msg, pod_260_get_product_info->ack_name);
    ttalink_pod_260_get_product_info_get_ack_id(msg, pod_260_get_product_info->ack_id);
    ttalink_pod_260_get_product_info_get_ack_accout(msg, pod_260_get_product_info->ack_accout);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN? msg->len : TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN;
        memset(pod_260_get_product_info, 0, TTALINK_MSG_ID_POD_260_GET_PRODUCT_INFO_LEN);
    memcpy(pod_260_get_product_info, _TTA_PAYLOAD(msg), len);
#endif
}
