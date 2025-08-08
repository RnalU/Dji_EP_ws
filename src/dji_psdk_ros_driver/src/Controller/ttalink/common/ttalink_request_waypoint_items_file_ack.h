#pragma once
// MESSAGE REQUEST_WAYPOINT_ITEMS_FILE_ACK PACKING

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK 2213

TTAPACKED(
typedef struct __ttalink_request_waypoint_items_file_ack_t {
 uint8_t ack; /*<  非0为错误. */
 uint8_t file_type; /*<  0:请求航线航点文件  1：请求航线描述文件. */
 int8_t wp_name[8]; /*<  返回当前航线文件名称.*/
 int32_t index; /*<  .*/
 int32_t total_num; /*<  .*/
 int32_t latit; /*<  .*/
 int32_t longi; /*<  .*/
 float altit; /*<  .*/
 float speed; /*<  .*/
 float acc_max; /*<  .*/
 int32_t delay_time; /*<  .*/
 uint8_t item_mode; /*<  .*/
 float point_r; /*<  .*/
 uint8_t turn_ctrl; /*<  .*/
 uint8_t event_ctrl; /*<  .*/
 uint8_t event_param; /*<  .*/
 uint8_t yaw_cmd; /*<  .*/
 float params[4]; /*<  .*/
}) ttalink_request_waypoint_items_file_ack_t;

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN 67
#define TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN 67
#define TTALINK_MSG_ID_2213_LEN 67
#define TTALINK_MSG_ID_2213_MIN_LEN 67

#define TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC 65
#define TTALINK_MSG_ID_2213_CRC 65

#define TTALINK_MSG_REQUEST_WAYPOINT_ITEMS_FILE_ACK_FIELD_WP_NAME_LEN 8
#define TTALINK_MSG_REQUEST_WAYPOINT_ITEMS_FILE_ACK_FIELD_PARAMS_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_REQUEST_WAYPOINT_ITEMS_FILE_ACK { \
    2213, \
    "REQUEST_WAYPOINT_ITEMS_FILE_ACK", \
    18, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_waypoint_items_file_ack_t, ack) }, \
         { "file_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_waypoint_items_file_ack_t, file_type) }, \
         { "wp_name", NULL, TTALINK_TYPE_INT8_T, 8, 2, offsetof(ttalink_request_waypoint_items_file_ack_t, wp_name) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 10, offsetof(ttalink_request_waypoint_items_file_ack_t, index) }, \
         { "total_num", NULL, TTALINK_TYPE_INT32_T, 0, 14, offsetof(ttalink_request_waypoint_items_file_ack_t, total_num) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 18, offsetof(ttalink_request_waypoint_items_file_ack_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 22, offsetof(ttalink_request_waypoint_items_file_ack_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_request_waypoint_items_file_ack_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_request_waypoint_items_file_ack_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_request_waypoint_items_file_ack_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 38, offsetof(ttalink_request_waypoint_items_file_ack_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 42, offsetof(ttalink_request_waypoint_items_file_ack_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_request_waypoint_items_file_ack_t, point_r) }, \
         { "turn_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 47, offsetof(ttalink_request_waypoint_items_file_ack_t, turn_ctrl) }, \
         { "event_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 48, offsetof(ttalink_request_waypoint_items_file_ack_t, event_ctrl) }, \
         { "event_param", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_request_waypoint_items_file_ack_t, event_param) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 50, offsetof(ttalink_request_waypoint_items_file_ack_t, yaw_cmd) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 51, offsetof(ttalink_request_waypoint_items_file_ack_t, params) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_REQUEST_WAYPOINT_ITEMS_FILE_ACK { \
    "REQUEST_WAYPOINT_ITEMS_FILE_ACK", \
    18, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_request_waypoint_items_file_ack_t, ack) }, \
         { "file_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_request_waypoint_items_file_ack_t, file_type) }, \
         { "wp_name", NULL, TTALINK_TYPE_INT8_T, 8, 2, offsetof(ttalink_request_waypoint_items_file_ack_t, wp_name) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 10, offsetof(ttalink_request_waypoint_items_file_ack_t, index) }, \
         { "total_num", NULL, TTALINK_TYPE_INT32_T, 0, 14, offsetof(ttalink_request_waypoint_items_file_ack_t, total_num) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 18, offsetof(ttalink_request_waypoint_items_file_ack_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 22, offsetof(ttalink_request_waypoint_items_file_ack_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_request_waypoint_items_file_ack_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_request_waypoint_items_file_ack_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_request_waypoint_items_file_ack_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 38, offsetof(ttalink_request_waypoint_items_file_ack_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 42, offsetof(ttalink_request_waypoint_items_file_ack_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_request_waypoint_items_file_ack_t, point_r) }, \
         { "turn_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 47, offsetof(ttalink_request_waypoint_items_file_ack_t, turn_ctrl) }, \
         { "event_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 48, offsetof(ttalink_request_waypoint_items_file_ack_t, event_ctrl) }, \
         { "event_param", NULL, TTALINK_TYPE_UINT8_T, 0, 49, offsetof(ttalink_request_waypoint_items_file_ack_t, event_param) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 50, offsetof(ttalink_request_waypoint_items_file_ack_t, yaw_cmd) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 51, offsetof(ttalink_request_waypoint_items_file_ack_t, params) }, \
         } \
}
#endif


static inline uint16_t _ttalink_request_waypoint_items_file_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_int32_t(buf, 10, index);
    _tta_put_int32_t(buf, 14, total_num);
    _tta_put_int32_t(buf, 18, latit);
    _tta_put_int32_t(buf, 22, longi);
    _tta_put_float(buf, 26, altit);
    _tta_put_float(buf, 30, speed);
    _tta_put_float(buf, 34, acc_max);
    _tta_put_int32_t(buf, 38, delay_time);
    _tta_put_uint8_t(buf, 42, item_mode);
    _tta_put_float(buf, 43, point_r);
    _tta_put_uint8_t(buf, 47, turn_ctrl);
    _tta_put_uint8_t(buf, 48, event_ctrl);
    _tta_put_uint8_t(buf, 49, event_param);
    _tta_put_uint8_t(buf, 50, yaw_cmd);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
    _tta_put_float_array(buf, 51, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN);
#else
    ttalink_request_waypoint_items_file_ack_t packet;
    packet.ack = ack;
    packet.file_type = file_type;
    packet.index = index;
    packet.total_num = total_num;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.speed = speed;
    packet.acc_max = acc_max;
    packet.delay_time = delay_time;
    packet.item_mode = item_mode;
    packet.point_r = point_r;
    packet.turn_ctrl = turn_ctrl;
    packet.event_ctrl = event_ctrl;
    packet.event_param = event_param;
    packet.yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_waypoint_items_file_ack message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  非0为错误. 
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称.
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param turn_ctrl  .
 * @param event_ctrl  .
 * @param event_param  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    return _ttalink_request_waypoint_items_file_ack_pack(dst_addr, src_addr, msg,  ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/**
 * @brief Pack a request_waypoint_items_file_ack message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack  非0为错误. 
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称.
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param turn_ctrl  .
 * @param event_ctrl  .
 * @param event_param  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    return _ttalink_request_waypoint_items_file_ack_pack(dst_addr, src_addr, msg,  ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}


static inline uint16_t _ttalink_request_waypoint_items_file_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t file_type,const int8_t *wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t turn_ctrl,uint8_t event_ctrl,uint8_t event_param,uint8_t yaw_cmd,const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_int32_t(buf, 10, index);
    _tta_put_int32_t(buf, 14, total_num);
    _tta_put_int32_t(buf, 18, latit);
    _tta_put_int32_t(buf, 22, longi);
    _tta_put_float(buf, 26, altit);
    _tta_put_float(buf, 30, speed);
    _tta_put_float(buf, 34, acc_max);
    _tta_put_int32_t(buf, 38, delay_time);
    _tta_put_uint8_t(buf, 42, item_mode);
    _tta_put_float(buf, 43, point_r);
    _tta_put_uint8_t(buf, 47, turn_ctrl);
    _tta_put_uint8_t(buf, 48, event_ctrl);
    _tta_put_uint8_t(buf, 49, event_param);
    _tta_put_uint8_t(buf, 50, yaw_cmd);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
    _tta_put_float_array(buf, 51, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN);
#else
    ttalink_request_waypoint_items_file_ack_t packet;
    packet.ack = ack;
    packet.file_type = file_type;
    packet.index = index;
    packet.total_num = total_num;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.speed = speed;
    packet.acc_max = acc_max;
    packet.delay_time = delay_time;
    packet.item_mode = item_mode;
    packet.point_r = point_r;
    packet.turn_ctrl = turn_ctrl;
    packet.event_ctrl = event_ctrl;
    packet.event_param = event_param;
    packet.yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC, nocrc);
}

/**
 * @brief Pack a request_waypoint_items_file_ack message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  非0为错误. 
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称.
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param turn_ctrl  .
 * @param event_ctrl  .
 * @param event_param  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t file_type,const int8_t *wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t turn_ctrl,uint8_t event_ctrl,uint8_t event_param,uint8_t yaw_cmd,const float *params)
{
    return _ttalink_request_waypoint_items_file_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/**
 * @brief Pack a request_waypoint_items_file_ack message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack  非0为错误. 
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称.
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param turn_ctrl  .
 * @param event_ctrl  .
 * @param event_param  .
 * @param yaw_cmd  .
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t file_type,const int8_t *wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t turn_ctrl,uint8_t event_ctrl,uint8_t event_param,uint8_t yaw_cmd,const float *params)
{
    return _ttalink_request_waypoint_items_file_ack_pack_chan(dst_addr, src_addr, chan, msg,  ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}


static inline uint16_t _ttalink_request_waypoint_items_file_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_waypoint_items_file_ack_pack_nocrc(dst_addr, src_addr, msg, request_waypoint_items_file_ack->ack, request_waypoint_items_file_ack->file_type, request_waypoint_items_file_ack->wp_name, request_waypoint_items_file_ack->index, request_waypoint_items_file_ack->total_num, request_waypoint_items_file_ack->latit, request_waypoint_items_file_ack->longi, request_waypoint_items_file_ack->altit, request_waypoint_items_file_ack->speed, request_waypoint_items_file_ack->acc_max, request_waypoint_items_file_ack->delay_time, request_waypoint_items_file_ack->item_mode, request_waypoint_items_file_ack->point_r, request_waypoint_items_file_ack->turn_ctrl, request_waypoint_items_file_ack->event_ctrl, request_waypoint_items_file_ack->event_param, request_waypoint_items_file_ack->yaw_cmd, request_waypoint_items_file_ack->params);
    }else{
        return ttalink_request_waypoint_items_file_ack_pack(dst_addr, src_addr, msg, request_waypoint_items_file_ack->ack, request_waypoint_items_file_ack->file_type, request_waypoint_items_file_ack->wp_name, request_waypoint_items_file_ack->index, request_waypoint_items_file_ack->total_num, request_waypoint_items_file_ack->latit, request_waypoint_items_file_ack->longi, request_waypoint_items_file_ack->altit, request_waypoint_items_file_ack->speed, request_waypoint_items_file_ack->acc_max, request_waypoint_items_file_ack->delay_time, request_waypoint_items_file_ack->item_mode, request_waypoint_items_file_ack->point_r, request_waypoint_items_file_ack->turn_ctrl, request_waypoint_items_file_ack->event_ctrl, request_waypoint_items_file_ack->event_param, request_waypoint_items_file_ack->yaw_cmd, request_waypoint_items_file_ack->params);
    }
    
}

/**
 * @brief Encode a request_waypoint_items_file_ack struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_items_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack)
{
    return _ttalink_request_waypoint_items_file_ack_encode(dst_addr, src_addr, msg, request_waypoint_items_file_ack, false);
}

/**
 * @brief Encode a request_waypoint_items_file_ack struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_items_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack)
{
    return _ttalink_request_waypoint_items_file_ack_encode(dst_addr, src_addr, msg, request_waypoint_items_file_ack, true);
}


static inline uint16_t _ttalink_request_waypoint_items_file_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack, bool nocrc)
{
    if(nocrc){
        return ttalink_request_waypoint_items_file_ack_pack_chan_nocrc(dst_addr, src_addr, chan, msg, request_waypoint_items_file_ack->ack, request_waypoint_items_file_ack->file_type, request_waypoint_items_file_ack->wp_name, request_waypoint_items_file_ack->index, request_waypoint_items_file_ack->total_num, request_waypoint_items_file_ack->latit, request_waypoint_items_file_ack->longi, request_waypoint_items_file_ack->altit, request_waypoint_items_file_ack->speed, request_waypoint_items_file_ack->acc_max, request_waypoint_items_file_ack->delay_time, request_waypoint_items_file_ack->item_mode, request_waypoint_items_file_ack->point_r, request_waypoint_items_file_ack->turn_ctrl, request_waypoint_items_file_ack->event_ctrl, request_waypoint_items_file_ack->event_param, request_waypoint_items_file_ack->yaw_cmd, request_waypoint_items_file_ack->params);
    }else{
        return ttalink_request_waypoint_items_file_ack_pack_chan(dst_addr, src_addr, chan, msg, request_waypoint_items_file_ack->ack, request_waypoint_items_file_ack->file_type, request_waypoint_items_file_ack->wp_name, request_waypoint_items_file_ack->index, request_waypoint_items_file_ack->total_num, request_waypoint_items_file_ack->latit, request_waypoint_items_file_ack->longi, request_waypoint_items_file_ack->altit, request_waypoint_items_file_ack->speed, request_waypoint_items_file_ack->acc_max, request_waypoint_items_file_ack->delay_time, request_waypoint_items_file_ack->item_mode, request_waypoint_items_file_ack->point_r, request_waypoint_items_file_ack->turn_ctrl, request_waypoint_items_file_ack->event_ctrl, request_waypoint_items_file_ack->event_param, request_waypoint_items_file_ack->yaw_cmd, request_waypoint_items_file_ack->params);
    }
}

/**
 * @brief Encode a request_waypoint_items_file_ack struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_items_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack)
{
    return _ttalink_request_waypoint_items_file_ack_encode_chan(dst_addr, src_addr, chan, msg, request_waypoint_items_file_ack, false);
}

/**
 * @brief Encode a request_waypoint_items_file_ack struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param request_waypoint_items_file_ack C-struct to read the message contents from
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack)
{
    return _ttalink_request_waypoint_items_file_ack_encode_chan(dst_addr, src_addr, chan, msg, request_waypoint_items_file_ack, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_request_waypoint_items_file_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_int32_t(buf, 10, index);
    _tta_put_int32_t(buf, 14, total_num);
    _tta_put_int32_t(buf, 18, latit);
    _tta_put_int32_t(buf, 22, longi);
    _tta_put_float(buf, 26, altit);
    _tta_put_float(buf, 30, speed);
    _tta_put_float(buf, 34, acc_max);
    _tta_put_int32_t(buf, 38, delay_time);
    _tta_put_uint8_t(buf, 42, item_mode);
    _tta_put_float(buf, 43, point_r);
    _tta_put_uint8_t(buf, 47, turn_ctrl);
    _tta_put_uint8_t(buf, 48, event_ctrl);
    _tta_put_uint8_t(buf, 49, event_param);
    _tta_put_uint8_t(buf, 50, yaw_cmd);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
    _tta_put_float_array(buf, 51, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK, buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC, nocrc);
#else
    ttalink_request_waypoint_items_file_ack_t packet;
    packet.ack = ack;
    packet.file_type = file_type;
    packet.index = index;
    packet.total_num = total_num;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.speed = speed;
    packet.acc_max = acc_max;
    packet.delay_time = delay_time;
    packet.item_mode = item_mode;
    packet.point_r = point_r;
    packet.turn_ctrl = turn_ctrl;
    packet.event_ctrl = event_ctrl;
    packet.event_param = event_param;
    packet.yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet.wp_name, wp_name, sizeof(int8_t)*8);
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK, (const char *)&packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_waypoint_items_file_ack message
 * @param chan TTAlink channel to send the message
 *
 * @param ack  非0为错误. 
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称.
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param turn_ctrl  .
 * @param event_ctrl  .
 * @param event_param  .
 * @param yaw_cmd  .
 * @param params  .
 */
static inline void ttalink_request_waypoint_items_file_ack_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_request_waypoint_items_file_ack_send(dst_addr, src_addr, chan, ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/**
 * @brief Send a request_waypoint_items_file_ack message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack  非0为错误. 
 * @param file_type  0:请求航线航点文件  1：请求航线描述文件. 
 * @param wp_name  返回当前航线文件名称.
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  .
 * @param point_r  .
 * @param turn_ctrl  .
 * @param event_ctrl  .
 * @param event_param  .
 * @param yaw_cmd  .
 * @param params  .
 */
static inline void ttalink_request_waypoint_items_file_ack_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_request_waypoint_items_file_ack_send(dst_addr, src_addr, chan, ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}


static inline void _ttalink_request_waypoint_items_file_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_request_waypoint_items_file_ack_send_nocrc(dst_addr, src_addr, chan, request_waypoint_items_file_ack->ack, request_waypoint_items_file_ack->file_type, request_waypoint_items_file_ack->wp_name, request_waypoint_items_file_ack->index, request_waypoint_items_file_ack->total_num, request_waypoint_items_file_ack->latit, request_waypoint_items_file_ack->longi, request_waypoint_items_file_ack->altit, request_waypoint_items_file_ack->speed, request_waypoint_items_file_ack->acc_max, request_waypoint_items_file_ack->delay_time, request_waypoint_items_file_ack->item_mode, request_waypoint_items_file_ack->point_r, request_waypoint_items_file_ack->turn_ctrl, request_waypoint_items_file_ack->event_ctrl, request_waypoint_items_file_ack->event_param, request_waypoint_items_file_ack->yaw_cmd, request_waypoint_items_file_ack->params);
    }else{
        ttalink_request_waypoint_items_file_ack_send(dst_addr, src_addr, chan, request_waypoint_items_file_ack->ack, request_waypoint_items_file_ack->file_type, request_waypoint_items_file_ack->wp_name, request_waypoint_items_file_ack->index, request_waypoint_items_file_ack->total_num, request_waypoint_items_file_ack->latit, request_waypoint_items_file_ack->longi, request_waypoint_items_file_ack->altit, request_waypoint_items_file_ack->speed, request_waypoint_items_file_ack->acc_max, request_waypoint_items_file_ack->delay_time, request_waypoint_items_file_ack->item_mode, request_waypoint_items_file_ack->point_r, request_waypoint_items_file_ack->turn_ctrl, request_waypoint_items_file_ack->event_ctrl, request_waypoint_items_file_ack->event_param, request_waypoint_items_file_ack->yaw_cmd, request_waypoint_items_file_ack->params);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK, (const char *)request_waypoint_items_file_ack, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC, nocrc);
#endif
}

/**
 * @brief Send a request_waypoint_items_file_ack message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_waypoint_items_file_ack_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack)
{
    _ttalink_request_waypoint_items_file_ack_send_struct(dst_addr, src_addr, chan, request_waypoint_items_file_ack, false);
}

/**
 * @brief Send a request_waypoint_items_file_ack message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_request_waypoint_items_file_ack_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack)
{
    _ttalink_request_waypoint_items_file_ack_send_struct(dst_addr, src_addr, chan, request_waypoint_items_file_ack, true);
}

#if TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_request_waypoint_items_file_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, file_type);
    _tta_put_int32_t(buf, 10, index);
    _tta_put_int32_t(buf, 14, total_num);
    _tta_put_int32_t(buf, 18, latit);
    _tta_put_int32_t(buf, 22, longi);
    _tta_put_float(buf, 26, altit);
    _tta_put_float(buf, 30, speed);
    _tta_put_float(buf, 34, acc_max);
    _tta_put_int32_t(buf, 38, delay_time);
    _tta_put_uint8_t(buf, 42, item_mode);
    _tta_put_float(buf, 43, point_r);
    _tta_put_uint8_t(buf, 47, turn_ctrl);
    _tta_put_uint8_t(buf, 48, event_ctrl);
    _tta_put_uint8_t(buf, 49, event_param);
    _tta_put_uint8_t(buf, 50, yaw_cmd);
    _tta_put_int8_t_array(buf, 2, wp_name, 8);
    _tta_put_float_array(buf, 51, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK, buf, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC, nocrc);
#else
    ttalink_request_waypoint_items_file_ack_t *packet = (ttalink_request_waypoint_items_file_ack_t *)msgbuf;
    packet->ack = ack;
    packet->file_type = file_type;
    packet->index = index;
    packet->total_num = total_num;
    packet->latit = latit;
    packet->longi = longi;
    packet->altit = altit;
    packet->speed = speed;
    packet->acc_max = acc_max;
    packet->delay_time = delay_time;
    packet->item_mode = item_mode;
    packet->point_r = point_r;
    packet->turn_ctrl = turn_ctrl;
    packet->event_ctrl = event_ctrl;
    packet->event_param = event_param;
    packet->yaw_cmd = yaw_cmd;
    tta_array_memcpy(packet->wp_name, wp_name, sizeof(int8_t)*8);
    tta_array_memcpy(packet->params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK, (const char *)packet, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_MIN_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_request_waypoint_items_file_ack_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_request_waypoint_items_file_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_request_waypoint_items_file_ack_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t file_type, const int8_t *wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_request_waypoint_items_file_ack_send_buf(dst_addr, src_addr, msgbuf, chan, ack, file_type, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}
#endif

#endif

// MESSAGE REQUEST_WAYPOINT_ITEMS_FILE_ACK UNPACKING


/**
 * @brief Get field ack from request_waypoint_items_file_ack message
 *
 * @return  非0为错误. 
 */
static inline uint8_t ttalink_request_waypoint_items_file_ack_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field file_type from request_waypoint_items_file_ack message
 *
 * @return  0:请求航线航点文件  1：请求航线描述文件. 
 */
static inline uint8_t ttalink_request_waypoint_items_file_ack_get_file_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field wp_name from request_waypoint_items_file_ack message
 *
 * @return  返回当前航线文件名称.
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_get_wp_name(const ttalink_message_t* msg, int8_t *wp_name)
{
    return _TTA_RETURN_int8_t_array(msg, wp_name, 8,  2);
}

/**
 * @brief Get field index from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline int32_t ttalink_request_waypoint_items_file_ack_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  10);
}

/**
 * @brief Get field total_num from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline int32_t ttalink_request_waypoint_items_file_ack_get_total_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  14);
}

/**
 * @brief Get field latit from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline int32_t ttalink_request_waypoint_items_file_ack_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  18);
}

/**
 * @brief Get field longi from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline int32_t ttalink_request_waypoint_items_file_ack_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  22);
}

/**
 * @brief Get field altit from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline float ttalink_request_waypoint_items_file_ack_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Get field speed from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline float ttalink_request_waypoint_items_file_ack_get_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  30);
}

/**
 * @brief Get field acc_max from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline float ttalink_request_waypoint_items_file_ack_get_acc_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  34);
}

/**
 * @brief Get field delay_time from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline int32_t ttalink_request_waypoint_items_file_ack_get_delay_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  38);
}

/**
 * @brief Get field item_mode from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_waypoint_items_file_ack_get_item_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field point_r from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline float ttalink_request_waypoint_items_file_ack_get_point_r(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  43);
}

/**
 * @brief Get field turn_ctrl from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_waypoint_items_file_ack_get_turn_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field event_ctrl from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_waypoint_items_file_ack_get_event_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field event_param from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_waypoint_items_file_ack_get_event_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field yaw_cmd from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline uint8_t ttalink_request_waypoint_items_file_ack_get_yaw_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field params from request_waypoint_items_file_ack message
 *
 * @return  .
 */
static inline uint16_t ttalink_request_waypoint_items_file_ack_get_params(const ttalink_message_t* msg, float *params)
{
    return _TTA_RETURN_float_array(msg, params, 4,  51);
}

/**
 * @brief Decode a request_waypoint_items_file_ack message into a struct
 *
 * @param msg The message to decode
 * @param request_waypoint_items_file_ack C-struct to decode the message contents into
 */
static inline void ttalink_request_waypoint_items_file_ack_decode(const ttalink_message_t* msg, ttalink_request_waypoint_items_file_ack_t* request_waypoint_items_file_ack)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    request_waypoint_items_file_ack->ack = ttalink_request_waypoint_items_file_ack_get_ack(msg);
    request_waypoint_items_file_ack->file_type = ttalink_request_waypoint_items_file_ack_get_file_type(msg);
    ttalink_request_waypoint_items_file_ack_get_wp_name(msg, request_waypoint_items_file_ack->wp_name);
    request_waypoint_items_file_ack->index = ttalink_request_waypoint_items_file_ack_get_index(msg);
    request_waypoint_items_file_ack->total_num = ttalink_request_waypoint_items_file_ack_get_total_num(msg);
    request_waypoint_items_file_ack->latit = ttalink_request_waypoint_items_file_ack_get_latit(msg);
    request_waypoint_items_file_ack->longi = ttalink_request_waypoint_items_file_ack_get_longi(msg);
    request_waypoint_items_file_ack->altit = ttalink_request_waypoint_items_file_ack_get_altit(msg);
    request_waypoint_items_file_ack->speed = ttalink_request_waypoint_items_file_ack_get_speed(msg);
    request_waypoint_items_file_ack->acc_max = ttalink_request_waypoint_items_file_ack_get_acc_max(msg);
    request_waypoint_items_file_ack->delay_time = ttalink_request_waypoint_items_file_ack_get_delay_time(msg);
    request_waypoint_items_file_ack->item_mode = ttalink_request_waypoint_items_file_ack_get_item_mode(msg);
    request_waypoint_items_file_ack->point_r = ttalink_request_waypoint_items_file_ack_get_point_r(msg);
    request_waypoint_items_file_ack->turn_ctrl = ttalink_request_waypoint_items_file_ack_get_turn_ctrl(msg);
    request_waypoint_items_file_ack->event_ctrl = ttalink_request_waypoint_items_file_ack_get_event_ctrl(msg);
    request_waypoint_items_file_ack->event_param = ttalink_request_waypoint_items_file_ack_get_event_param(msg);
    request_waypoint_items_file_ack->yaw_cmd = ttalink_request_waypoint_items_file_ack_get_yaw_cmd(msg);
    ttalink_request_waypoint_items_file_ack_get_params(msg, request_waypoint_items_file_ack->params);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN? msg->len : TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN;
        memset(request_waypoint_items_file_ack, 0, TTALINK_MSG_ID_REQUEST_WAYPOINT_ITEMS_FILE_ACK_LEN);
    memcpy(request_waypoint_items_file_ack, _TTA_PAYLOAD(msg), len);
#endif
}
