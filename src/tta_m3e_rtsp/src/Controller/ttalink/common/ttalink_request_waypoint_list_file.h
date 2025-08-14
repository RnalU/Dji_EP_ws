#pragma once
// MESSAGE REQUEST_WAYPOINT_LIST_FILE PACKING

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE 2210

TTAPACKED(
typedef struct __ttalink_request_waypoint_list_file_t {
 uint8_t request_type; /*<  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198*/
 uint8_t file_type; /*<  0:请求航线航点文件  1：请求航线描述文件. */
 int8_t wp_name[8]; /*<  请求当前航线文件名称 */
 float param1; /*<  .*/
 float param2; /*<  .*/
}) ttalink_request_waypoint_list_file_t;

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN 18
#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN 18
#define TTALINK_MSG_ID_2210_LEN 18
#define TTALINK_MSG_ID_2210_MIN_LEN 18

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC 225
#define TTALINK_MSG_ID_2210_CRC 225

#define TTALINK_MSG_REQUEST_WAYPOINT_LIST_FILE_FIELD_WP_NAME_LEN 8

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_WAYPOINT_LIST_FILE { \
    2210, \
    "REQUEST_WAYPOINT_LIST_FILE", \
    5, \
    {  { "request_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_waypoint_list_file_t, request_type) }, \
         { "file_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_waypoint_list_file_t, file_type) }, \
         { "wp_name", NULL, TTALINK_TYPE_INT8_T, 8, 2, offsetof(ttalink_request_waypoint_list_file_t, wp_name) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_request_waypoint_list_file_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_request_waypoint_list_file_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_WAYPOINT_LIST_FILE { \
    "REQUEST_WAYPOINT_LIST_FILE", \
    5, \
    {  { "request_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_waypoint_list_file_t, request_type) }, \
         { "file_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_waypoint_list_file_t, file_type) }, \
         { "wp_name", NULL, TTALINK_TYPE_INT8_T, 8, 2, offsetof(ttalink_request_waypoint_list_file_t, wp_name) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_request_waypoint_list_file_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_request_waypoint_list_file_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_waypoint_list_file_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN];
    _tta_put_uint8_t(buf, 0, request_type);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_float(buf, 10, param1);
    _tta_put_float(buf, 14, param2);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN);
#else
    ttalink_request_waypoint_list_file_t packet;
    packet.request_type = request_type;
    packet.file_type = file_type;
    packet.param1 = param1;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC, nocrc);
}

/**
 * @brief Pack a request_waypoint_list_file message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param request_type  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  请求当前航线文件名称 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2)
{
    return _ttalink_request_waypoint_list_file_pack(dst_addr, src_addr, msg,  request_type, file_type, wp_name, param1, param2, false);
}

/**
 * @brief Pack a request_waypoint_list_file message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param request_type  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  请求当前航线文件名称 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2)
{
    return _ttalink_request_waypoint_list_file_pack(dst_addr, src_addr, msg,  request_type, file_type, wp_name, param1, param2, true);
}


static inline uint16_t _ttalink_request_waypoint_list_file_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t request_type,uint8_t file_type,const int8_t *wp_name,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN];
    _tta_put_uint8_t(buf, 0, request_type);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_float(buf, 10, param1);
    _tta_put_float(buf, 14, param2);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN);
#else
    ttalink_request_waypoint_list_file_t packet;
    packet.request_type = request_type;
    packet.file_type = file_type;
    packet.param1 = param1;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC, nocrc);
}

/**
 * @brief Pack a request_waypoint_list_file message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_type  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  请求当前航线文件名称 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t request_type,uint8_t file_type,const int8_t *wp_name,float param1,float param2)
{
    return _ttalink_request_waypoint_list_file_pack_chan(dst_addr, src_addr, chan, msg,  request_type, file_type, wp_name, param1, param2, false);
}

/**
 * @brief Pack a request_waypoint_list_file message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_type  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  请求当前航线文件名称 
 * @param param1  .
 * @param param2  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_list_file_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t request_type,uint8_t file_type,const int8_t *wp_name,float param1,float param2)
{
    return _ttalink_request_waypoint_list_file_pack_chan(dst_addr, src_addr, chan, msg,  request_type, file_type, wp_name, param1, param2, true);
}


static inline uint16_t _ttalink_request_waypoint_list_file_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file, bool nocrc)
{
    if(nocrc){
        return ttalink_request_waypoint_list_file_pack_nocrc(dst_addr, src_addr, msg, request_waypoint_list_file->request_type, request_waypoint_list_file->file_type, request_waypoint_list_file->wp_name, request_waypoint_list_file->param1, request_waypoint_list_file->param2);
    }else{
        return ttalink_request_waypoint_list_file_pack(dst_addr, src_addr, msg, request_waypoint_list_file->request_type, request_waypoint_list_file->file_type, request_waypoint_list_file->wp_name, request_waypoint_list_file->param1, request_waypoint_list_file->param2);
    }
    
}

/**
 * @brief Encode a request_waypoint_list_file struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file)
{
    return _ttalink_request_waypoint_list_file_encode(dst_addr, src_addr, msg, request_waypoint_list_file, false);
}

/**
 * @brief Encode a request_waypoint_list_file struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file)
{
    return _ttalink_request_waypoint_list_file_encode(dst_addr, src_addr, msg, request_waypoint_list_file, true);
}


static inline uint16_t _ttalink_request_waypoint_list_file_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file, bool nocrc)
{
    if(nocrc){
        return ttalink_request_waypoint_list_file_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_waypoint_list_file->request_type, request_waypoint_list_file->file_type, request_waypoint_list_file->wp_name, request_waypoint_list_file->param1, request_waypoint_list_file->param2);
    }else{
        return ttalink_request_waypoint_list_file_pack_chan(dst_addr, src_addr, chan, msg, request_waypoint_list_file->request_type, request_waypoint_list_file->file_type, request_waypoint_list_file->wp_name, request_waypoint_list_file->param1, request_waypoint_list_file->param2);
    }
}

/**
 * @brief Encode a request_waypoint_list_file struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file)
{
    return _ttalink_request_waypoint_list_file_encode_chan(dst_addr, src_addr, chan, msg, request_waypoint_list_file, false);
}

/**
 * @brief Encode a request_waypoint_list_file struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_list_file C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_list_file_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file)
{
    return _ttalink_request_waypoint_list_file_encode_chan(dst_addr, src_addr, chan, msg, request_waypoint_list_file, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_waypoint_list_file_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN];
    _tta_put_uint8_t(buf, 0, request_type);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_float(buf, 10, param1);
    _tta_put_float(buf, 14, param2);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE, buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC, nocrc);
#else
    ttalink_request_waypoint_list_file_t packet;
    packet.request_type = request_type;
    packet.file_type = file_type;
    packet.param1 = param1;
    packet.param2 = param2;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE, (const char *)&packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_waypoint_list_file message
 * @param chan TTAlink channel to send the message
 *
 * @param request_type  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  请求当前航线文件名称 
 * @param param1  .
 * @param param2  .
 */
static inline void ttalink_request_waypoint_list_file_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2)
{
    _ttalink_request_waypoint_list_file_send(dst_addr, src_addr, chan, request_type, file_type, wp_name, param1, param2, false);
}

/**
 * @brief Send a request_waypoint_list_file message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param request_type  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  请求当前航线文件名称 
 * @param param1  .
 * @param param2  .
 */
static inline void ttalink_request_waypoint_list_file_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2)
{
    _ttalink_request_waypoint_list_file_send(dst_addr, src_addr, chan, request_type, file_type, wp_name, param1, param2, true);
}


static inline void _ttalink_request_waypoint_list_file_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_waypoint_list_file_send_nocrc(dst_addr, src_addr, chan, request_waypoint_list_file->request_type, request_waypoint_list_file->file_type, request_waypoint_list_file->wp_name, request_waypoint_list_file->param1, request_waypoint_list_file->param2);
    }else{
        ttalink_request_waypoint_list_file_send(dst_addr, src_addr, chan, request_waypoint_list_file->request_type, request_waypoint_list_file->file_type, request_waypoint_list_file->wp_name, request_waypoint_list_file->param1, request_waypoint_list_file->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE, (const char *)request_waypoint_list_file, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_waypoint_list_file message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_waypoint_list_file_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file)
{
    _ttalink_request_waypoint_list_file_send_struct(dst_addr, src_addr, chan, request_waypoint_list_file, false);
}

/**
 * @brief Send a request_waypoint_list_file message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_waypoint_list_file_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_list_file_t* request_waypoint_list_file)
{
    _ttalink_request_waypoint_list_file_send_struct(dst_addr, src_addr, chan, request_waypoint_list_file, true);
}

#if TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_waypoint_list_file_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, request_type);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_float(buf, 10, param1);
    _tta_put_float(buf, 14, param2);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE, buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC, nocrc);
#else
    ttalink_request_waypoint_list_file_t *packet = (ttalink_request_waypoint_list_file_t *)msgbuf;
    packet->request_type = request_type;
    packet->file_type = file_type;
    packet->param1 = param1;
    packet->param2 = param2;
    tta_array_memcpy(packet->wp_name, wp_name, sizeof(int8_t)*8);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE, (const char *)packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_waypoint_list_file_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2)
{
    _ttalink_request_waypoint_list_file_send_buf(dst_addr, src_addr, msgbuf, chan, request_type, file_type, wp_name, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_waypoint_list_file_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t request_type, uint8_t file_type, const int8_t *wp_name, float param1, float param2)
{
    _ttalink_request_waypoint_list_file_send_buf(dst_addr, src_addr, msgbuf, chan, request_type, file_type, wp_name, param1, param2, true);
}
#endif

#endif

// MESSAGE REQUEST_WAYPOINT_LIST_FILE UNPACKING


/**
 * @brief Get field request_type from request_waypoint_list_file message
 *
 * @return  0：:列表  1：文件. 2：文件标志 3:航线配置信息，回复ACK后再回复#2198
 */
static inline uint8_t ttalink_request_waypoint_list_file_get_request_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field file_type from request_waypoint_list_file message
 *
 * @return  0:请求航线航点文件  1：请求航线描述文件. 
 */
static inline uint8_t ttalink_request_waypoint_list_file_get_file_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field wp_name from request_waypoint_list_file message
 *
 * @return  请求当前航线文件名称 
 */
static inline uint16_t ttalink_request_waypoint_list_file_get_wp_name(const ttalink_message_t* msg, int8_t *wp_name)
{
    return _TTA_RETURN_int8_t_array(msg, wp_name, 8,  2);
}

/**
 * @brief Get field param1 from request_waypoint_list_file message
 *
 * @return  .
 */
static inline float ttalink_request_waypoint_list_file_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  10);
}

/**
 * @brief Get field param2 from request_waypoint_list_file message
 *
 * @return  .
 */
static inline float ttalink_request_waypoint_list_file_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  14);
}

/**
 * @brief Decode a request_waypoint_list_file message into a struct
 *
 * @param msg The message to decode
 * @param request_waypoint_list_file C-struct to decode the message contents into
 */
static inline void ttalink_request_waypoint_list_file_decode(const ttalink_message_t* msg, ttalink_request_waypoint_list_file_t* request_waypoint_list_file)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_waypoint_list_file->request_type = ttalink_request_waypoint_list_file_get_request_type(msg);
    request_waypoint_list_file->file_type = ttalink_request_waypoint_list_file_get_file_type(msg);
    ttalink_request_waypoint_list_file_get_wp_name(msg, request_waypoint_list_file->wp_name);
    request_waypoint_list_file->param1 = ttalink_request_waypoint_list_file_get_param1(msg);
    request_waypoint_list_file->param2 = ttalink_request_waypoint_list_file_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN? msg->len : TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN;
        memset(request_waypoint_list_file, 0, TTALINK_MSG_ID_REQUEST_WAYPOINT_LIST_FILE_LEN);
    memcpy(request_waypoint_list_file, _TTA_PAYLOAD(msg), len);
#endif
}
