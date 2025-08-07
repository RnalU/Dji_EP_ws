#pragma once
// MESSAGE REQUEST_WAYPOINT_LIST_FILE_ACK PACKING

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK 2211

TTAPACKED(
typedef struct __ttalink_request_waypoint_list_file_ack_t {
 uint8_t ack; /*<  非0为错误. */
 uint8_t request_type; /*<   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198*/
 uint8_t file_type; /*<  0:请求航线航点文件  1：请求航线描述文件. */
 int8_t wp_name[8]; /*<  返回当前航线文件名称 */
 uint16_t current_wp_index; /*<  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). */
 uint16_t total_wp_index; /*<  总共航线文件数目(toatl)/总共航线航点数目. */
 uint32_t flie_status; /*<  文件标志位，存在航线文件则置位.*/
 float param2; /*<  .*/
}) ttalink_request_waypoint_list_file_ack_t;

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN 23
#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN 23
#define TTALINK_MSG_ID_2211_LEN 23
#define TTALINK_MSG_ID_2211_MIN_LEN 23

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC 197
#define TTALINK_MSG_ID_2211_CRC 197

#define TTALINK_MSG_REQUEST_WAYPOINT_LIST_FILE_ACK_FIELD_WP_NAME_LEN 8

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_WAYPOINT_LIST_FILE_ACK { \
    2211, \
    "REQUEST_WAYPOINT_LIST_FILE_ACK", \
    8, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_waypoint_list_file_ack_t, ack) }, \
         { "request_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_waypoint_list_file_ack_t, request_type) }, \
         { "file_type", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_request_waypoint_list_file_ack_t, file_type) }, \
         { "wp_name", NULL, TTALINK_TYPE_INT8_T, 8, 3, offsetof(ttalink_request_waypoint_list_file_ack_t, wp_name) }, \
         { "current_wp_index", NULL, TTALINK_TYPE_UINT16_T, 0, 11, offsetof(ttalink_request_waypoint_list_file_ack_t, current_wp_index) }, \
         { "total_wp_index", NULL, TTALINK_TYPE_UINT16_T, 0, 13, offsetof(ttalink_request_waypoint_list_file_ack_t, total_wp_index) }, \
         { "flie_status", NULL, TTALINK_TYPE_UINT32_T, 0, 15, offsetof(ttalink_request_waypoint_list_file_ack_t, flie_status) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_request_waypoint_list_file_ack_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_WAYPOINT_LIST_FILE_ACK { \
    "REQUEST_WAYPOINT_LIST_FILE_ACK", \
    8, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_waypoint_list_file_ack_t, ack) }, \
         { "request_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_waypoint_list_file_ack_t, request_type) }, \
         { "file_type", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_request_waypoint_list_file_ack_t, file_type) }, \
         { "wp_name", NULL, TTALINK_TYPE_INT8_T, 8, 3, offsetof(ttalink_request_waypoint_list_file_ack_t, wp_name) }, \
         { "current_wp_index", NULL, TTALINK_TYPE_UINT16_T, 0, 11, offsetof(ttalink_request_waypoint_list_file_ack_t, current_wp_index) }, \
         { "total_wp_index", NULL, TTALINK_TYPE_UINT16_T, 0, 13, offsetof(ttalink_request_waypoint_list_file_ack_t, total_wp_index) }, \
         { "flie_status", NULL, TTALINK_TYPE_UINT32_T, 0, 15, offsetof(ttalink_request_waypoint_list_file_ack_t, flie_status) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_request_waypoint_list_file_ack_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_waypoint_list_file_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, request_type);
    _tta_put_uint8_t(buf, 2, file_type);
    _tta_put_uint16_t(buf, 11, current_wp_index);
    _tta_put_uint16_t(buf, 13, total_wp_index);
    _tta_put_uint32_t(buf, 15, flie_status);
    _tta_put_float(buf, 19, param2);
    _tta_put_int8_t_array(buf, 3, wp_name, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN);
#else
    ttalink_request_waypoint_list_file_ack_t packet;
    packet.ack = ack;
    packet.request_type = request_type;
    packet.file_type = file_type;
    packet.current_wp_index = current_wp_index;
    packet.total_wp_index = total_wp_index;
    packet.flie_status = flie_status;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_waypoint_list_file_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  非0为错误. 
 * @param request_type   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称 
 * @param current_wp_index  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). 
 * @param total_wp_index  总共航线文件数目(toatl)/总共航线航点数目. 
 * @param flie_status  文件标志位，存在航线文件则置位.
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2)
{
    return _ttalink_request_waypoint_list_file_ack_pack(dst_addr, src_addr, msg,  ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, false);
}

/**
 * @brief Pack a request_waypoint_list_file_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  非0为错误. 
 * @param request_type   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称 
 * @param current_wp_index  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). 
 * @param total_wp_index  总共航线文件数目(toatl)/总共航线航点数目. 
 * @param flie_status  文件标志位，存在航线文件则置位.
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2)
{
    return _ttalink_request_waypoint_list_file_ack_pack(dst_addr, src_addr, msg,  ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, true);
}


static inline uint16_t _ttalink_request_waypoint_list_file_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t request_type,uint8_t file_type,const int8_t *wp_name,uint16_t current_wp_index,uint16_t total_wp_index,uint32_t flie_status,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, request_type);
    _tta_put_uint8_t(buf, 2, file_type);
    _tta_put_uint16_t(buf, 11, current_wp_index);
    _tta_put_uint16_t(buf, 13, total_wp_index);
    _tta_put_uint32_t(buf, 15, flie_status);
    _tta_put_float(buf, 19, param2);
    _tta_put_int8_t_array(buf, 3, wp_name, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN);
#else
    ttalink_request_waypoint_list_file_ack_t packet;
    packet.ack = ack;
    packet.request_type = request_type;
    packet.file_type = file_type;
    packet.current_wp_index = current_wp_index;
    packet.total_wp_index = total_wp_index;
    packet.flie_status = flie_status;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_waypoint_list_file_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  非0为错误. 
 * @param request_type   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称 
 * @param current_wp_index  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). 
 * @param total_wp_index  总共航线文件数目(toatl)/总共航线航点数目. 
 * @param flie_status  文件标志位，存在航线文件则置位.
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t request_type,uint8_t file_type,const int8_t *wp_name,uint16_t current_wp_index,uint16_t total_wp_index,uint32_t flie_status,float param2)
{
    return _ttalink_request_waypoint_list_file_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, false);
}

/**
 * @brief Pack a request_waypoint_list_file_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  非0为错误. 
 * @param request_type   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称 
 * @param current_wp_index  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). 
 * @param total_wp_index  总共航线文件数目(toatl)/总共航线航点数目. 
 * @param flie_status  文件标志位，存在航线文件则置位.
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t request_type,uint8_t file_type,const int8_t *wp_name,uint16_t current_wp_index,uint16_t total_wp_index,uint32_t flie_status,float param2)
{
    return _ttalink_request_waypoint_list_file_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, true);
}


static inline uint16_t _ttalink_request_waypoint_list_file_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_waypoint_list_file_ack_pack_nocrc(dst_addr, src_addr, msg, request_waypoint_list_file_ack->ack, request_waypoint_list_file_ack->request_type, request_waypoint_list_file_ack->file_type, request_waypoint_list_file_ack->wp_name, request_waypoint_list_file_ack->current_wp_index, request_waypoint_list_file_ack->total_wp_index, request_waypoint_list_file_ack->flie_status, request_waypoint_list_file_ack->param2);
    }else{
        return ttalink_request_waypoint_list_file_ack_pack(dst_addr, src_addr, msg, request_waypoint_list_file_ack->ack, request_waypoint_list_file_ack->request_type, request_waypoint_list_file_ack->file_type, request_waypoint_list_file_ack->wp_name, request_waypoint_list_file_ack->current_wp_index, request_waypoint_list_file_ack->total_wp_index, request_waypoint_list_file_ack->flie_status, request_waypoint_list_file_ack->param2);
    }
    
}

/**
 * @brief Encode a request_waypoint_list_file_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack)
{
    return _ttalink_request_waypoint_list_file_ack_encode(dst_addr, src_addr, msg, request_waypoint_list_file_ack, false);
}

/**
 * @brief Encode a request_waypoint_list_file_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack)
{
    return _ttalink_request_waypoint_list_file_ack_encode(dst_addr, src_addr, msg, request_waypoint_list_file_ack, true);
}


static inline uint16_t _ttalink_request_waypoint_list_file_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_waypoint_list_file_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_waypoint_list_file_ack->ack, request_waypoint_list_file_ack->request_type, request_waypoint_list_file_ack->file_type, request_waypoint_list_file_ack->wp_name, request_waypoint_list_file_ack->current_wp_index, request_waypoint_list_file_ack->total_wp_index, request_waypoint_list_file_ack->flie_status, request_waypoint_list_file_ack->param2);
    }else{
        return ttalink_request_waypoint_list_file_ack_pack_chan(dst_addr, src_addr, chan, msg, request_waypoint_list_file_ack->ack, request_waypoint_list_file_ack->request_type, request_waypoint_list_file_ack->file_type, request_waypoint_list_file_ack->wp_name, request_waypoint_list_file_ack->current_wp_index, request_waypoint_list_file_ack->total_wp_index, request_waypoint_list_file_ack->flie_status, request_waypoint_list_file_ack->param2);
    }
}

/**
 * @brief Encode a request_waypoint_list_file_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack)
{
    return _ttalink_request_waypoint_list_file_ack_encode_chan(dst_addr, src_addr, chan, msg, request_waypoint_list_file_ack, false);
}

/**
 * @brief Encode a request_waypoint_list_file_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack)
{
    return _ttalink_request_waypoint_list_file_ack_encode_chan(dst_addr, src_addr, chan, msg, request_waypoint_list_file_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_waypoint_list_file_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, request_type);
    _tta_put_uint8_t(buf, 2, file_type);
    _tta_put_uint16_t(buf, 11, current_wp_index);
    _tta_put_uint16_t(buf, 13, total_wp_index);
    _tta_put_uint32_t(buf, 15, flie_status);
    _tta_put_float(buf, 19, param2);
    _tta_put_int8_t_array(buf, 3, wp_name, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK, buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC, nocrc);
#else
    ttalink_request_waypoint_list_file_ack_t packet;
    packet.ack = ack;
    packet.request_type = request_type;
    packet.file_type = file_type;
    packet.current_wp_index = current_wp_index;
    packet.total_wp_index = total_wp_index;
    packet.flie_status = flie_status;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK, (const char *)&packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_waypoint_list_file_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  非0为错误. 
 * @param request_type   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称 
 * @param current_wp_index  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). 
 * @param total_wp_index  总共航线文件数目(toatl)/总共航线航点数目. 
 * @param flie_status  文件标志位，存在航线文件则置位.
 * @param param2  .
 */
static inline void ttalink_request_waypoint_list_file_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2)
{
    _ttalink_request_waypoint_list_file_ack_send(dst_addr, src_addr, chan, ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, false);
}

/**
 * @brief Send a request_waypoint_list_file_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  非0为错误. 
 * @param request_type   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称 
 * @param current_wp_index  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). 
 * @param total_wp_index  总共航线文件数目(toatl)/总共航线航点数目. 
 * @param flie_status  文件标志位，存在航线文件则置位.
 * @param param2  .
 */
static inline void ttalink_request_waypoint_list_file_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2)
{
    _ttalink_request_waypoint_list_file_ack_send(dst_addr, src_addr, chan, ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, true);
}


static inline void _ttalink_request_waypoint_list_file_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_waypoint_list_file_ack_send_nocrc(dst_addr, src_addr, chan, request_waypoint_list_file_ack->ack, request_waypoint_list_file_ack->request_type, request_waypoint_list_file_ack->file_type, request_waypoint_list_file_ack->wp_name, request_waypoint_list_file_ack->current_wp_index, request_waypoint_list_file_ack->total_wp_index, request_waypoint_list_file_ack->flie_status, request_waypoint_list_file_ack->param2);
    }else{
        ttalink_request_waypoint_list_file_ack_send(dst_addr, src_addr, chan, request_waypoint_list_file_ack->ack, request_waypoint_list_file_ack->request_type, request_waypoint_list_file_ack->file_type, request_waypoint_list_file_ack->wp_name, request_waypoint_list_file_ack->current_wp_index, request_waypoint_list_file_ack->total_wp_index, request_waypoint_list_file_ack->flie_status, request_waypoint_list_file_ack->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK, (const char *)request_waypoint_list_file_ack, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_waypoint_list_file_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_waypoint_list_file_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack)
{
    _ttalink_request_waypoint_list_file_ack_send_struct(dst_addr, src_addr, chan, request_waypoint_list_file_ack, false);
}

/**
 * @brief Send a request_waypoint_list_file_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_waypoint_list_file_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack)
{
    _ttalink_request_waypoint_list_file_ack_send_struct(dst_addr, src_addr, chan, request_waypoint_list_file_ack, true);
}

#if TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_waypoint_list_file_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, request_type);
    _tta_put_uint8_t(buf, 2, file_type);
    _tta_put_uint16_t(buf, 11, current_wp_index);
    _tta_put_uint16_t(buf, 13, total_wp_index);
    _tta_put_uint32_t(buf, 15, flie_status);
    _tta_put_float(buf, 19, param2);
    _tta_put_int8_t_array(buf, 3, wp_name, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK, buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC, nocrc);
#else
    ttalink_request_waypoint_list_file_ack_t *packet = (ttalink_request_waypoint_list_file_ack_t *)msgbuf;
    packet->ack = ack;
    packet->request_type = request_type;
    packet->file_type = file_type;
    packet->current_wp_index = current_wp_index;
    packet->total_wp_index = total_wp_index;
    packet->flie_status = flie_status;
    packet->param2 = param2;
    tta_array_memcpy(packet->wp_name, wp_name, sizeof(int8_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK, (const char *)packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_waypoint_list_file_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2)
{
    _ttalink_request_waypoint_list_file_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_waypoint_list_file_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, uint16_t current_wp_index, uint16_t total_wp_index, uint32_t flie_status, float param2)
{
    _ttalink_request_waypoint_list_file_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, request_type, file_type, wp_name, current_wp_index, total_wp_index, flie_status, param2, true);
}
#endif

#endif

// MESSAGE REQUEST_WAYPOINT_LIST_FILE_ACK UNPACKING


/**
 * @brief Get field ack from request_waypoint_list_file_ack message
 *
 * @return  非0为错误. 
 */
static inline uint8_t ttalink_request_waypoint_list_file_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field request_type from request_waypoint_list_file_ack message
 *
 * @return   0：列表 1：文件 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 */
static inline uint8_t ttalink_request_waypoint_list_file_ack_get_request_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field file_type from request_waypoint_list_file_ack message
 *
 * @return  0:请求航线航点文件  1：请求航线描述文件. 
 */
static inline uint8_t ttalink_request_waypoint_list_file_ack_get_file_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field wp_name from request_waypoint_list_file_ack message
 *
 * @return  返回当前航线文件名称 
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_get_wp_name(const ttalink_message_t* msg, int8_t *wp_name)
{
    return _TTA_RETURN_int8_t_array(msg, wp_name, 8,  3);
}

/**
 * @brief Get field current_wp_index from request_waypoint_list_file_ack message
 *
 * @return  当前航线文件数目(0-total-1)/当前航线航点数目(0-total-1). 
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_get_current_wp_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  11);
}

/**
 * @brief Get field total_wp_index from request_waypoint_list_file_ack message
 *
 * @return  总共航线文件数目(toatl)/总共航线航点数目. 
 */
static inline uint16_t ttalink_request_waypoint_list_file_ack_get_total_wp_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  13);
}

/**
 * @brief Get field flie_status from request_waypoint_list_file_ack message
 *
 * @return  文件标志位，存在航线文件则置位.
 */
static inline uint32_t ttalink_request_waypoint_list_file_ack_get_flie_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  15);
}

/**
 * @brief Get field param2 from request_waypoint_list_file_ack message
 *
 * @return  .
 */
static inline float ttalink_request_waypoint_list_file_ack_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  19);
}

/**
 * @brief Decode a request_waypoint_list_file_ack message into a struct
 *
 * @param msg The message to decode
 * @param request_waypoint_list_file_ack C-struct to decode the message contents into
 */
static inline void ttalink_request_waypoint_list_file_ack_decode(const ttalink_message_t* msg, ttalink_request_waypoint_list_file_ack_t* request_waypoint_list_file_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_waypoint_list_file_ack->ack = ttalink_request_waypoint_list_file_ack_get_ack(msg);
    request_waypoint_list_file_ack->request_type = ttalink_request_waypoint_list_file_ack_get_request_type(msg);
    request_waypoint_list_file_ack->file_type = ttalink_request_waypoint_list_file_ack_get_file_type(msg);
    ttalink_request_waypoint_list_file_ack_get_wp_name(msg, request_waypoint_list_file_ack->wp_name);
    request_waypoint_list_file_ack->current_wp_index = ttalink_request_waypoint_list_file_ack_get_current_wp_index(msg);
    request_waypoint_list_file_ack->total_wp_index = ttalink_request_waypoint_list_file_ack_get_total_wp_index(msg);
    request_waypoint_list_file_ack->flie_status = ttalink_request_waypoint_list_file_ack_get_flie_status(msg);
    request_waypoint_list_file_ack->param2 = ttalink_request_waypoint_list_file_ack_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN? msg->len : TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN;
        memset(request_waypoint_list_file_ack, 0, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_ACK_LEN);
    memcpy(request_waypoint_list_file_ack, _TTA_PAYLOAD(msg), len);
#endif
}
