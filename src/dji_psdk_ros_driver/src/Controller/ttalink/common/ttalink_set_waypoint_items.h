#pragma once
// MESSAGE SET_WAYPOINT_ITEMS PACKING

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS 2200

TTAPACKED(
typedef struct __ttalink_set_waypoint_items_t {
 uint8_t cmd; /*<  .*/
 uint8_t wp_name; /*<  这里以后需要改成 8个字节的数组.*/
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
 uint8_t event_ctrl; /*<  spray_ctrl->envent_ctrl.*/
 uint8_t event_param; /*<  user_ctrl->event_param.*/
 uint8_t yaw_cmd; /*<  2：特定角度.*/
 float params[4]; /*<  .*/
}) ttalink_set_waypoint_items_t;

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN 59
#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN 59
#define TTALINK_MSG_ID_2200_LEN 59
#define TTALINK_MSG_ID_2200_MIN_LEN 59

#define TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC 35
#define TTALINK_MSG_ID_2200_CRC 35

#define TTALINK_MSG_SET_WAYPOINT_ITEMS_FIELD_PARAMS_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_ITEMS { \
    2200, \
    "SET_WAYPOINT_ITEMS", \
    17, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_items_t, cmd) }, \
         { "wp_name", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_set_waypoint_items_t, wp_name) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 2, offsetof(ttalink_set_waypoint_items_t, index) }, \
         { "total_num", NULL, TTALINK_TYPE_INT32_T, 0, 6, offsetof(ttalink_set_waypoint_items_t, total_num) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 10, offsetof(ttalink_set_waypoint_items_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 14, offsetof(ttalink_set_waypoint_items_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_set_waypoint_items_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_set_waypoint_items_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_set_waypoint_items_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 30, offsetof(ttalink_set_waypoint_items_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 34, offsetof(ttalink_set_waypoint_items_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_set_waypoint_items_t, point_r) }, \
         { "turn_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 39, offsetof(ttalink_set_waypoint_items_t, turn_ctrl) }, \
         { "event_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 40, offsetof(ttalink_set_waypoint_items_t, event_ctrl) }, \
         { "event_param", NULL, TTALINK_TYPE_UINT8_T, 0, 41, offsetof(ttalink_set_waypoint_items_t, event_param) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 42, offsetof(ttalink_set_waypoint_items_t, yaw_cmd) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 43, offsetof(ttalink_set_waypoint_items_t, params) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_ITEMS { \
    "SET_WAYPOINT_ITEMS", \
    17, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_items_t, cmd) }, \
         { "wp_name", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_set_waypoint_items_t, wp_name) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 2, offsetof(ttalink_set_waypoint_items_t, index) }, \
         { "total_num", NULL, TTALINK_TYPE_INT32_T, 0, 6, offsetof(ttalink_set_waypoint_items_t, total_num) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 10, offsetof(ttalink_set_waypoint_items_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 14, offsetof(ttalink_set_waypoint_items_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_set_waypoint_items_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_set_waypoint_items_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_set_waypoint_items_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 30, offsetof(ttalink_set_waypoint_items_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 34, offsetof(ttalink_set_waypoint_items_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 35, offsetof(ttalink_set_waypoint_items_t, point_r) }, \
         { "turn_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 39, offsetof(ttalink_set_waypoint_items_t, turn_ctrl) }, \
         { "event_ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 40, offsetof(ttalink_set_waypoint_items_t, event_ctrl) }, \
         { "event_param", NULL, TTALINK_TYPE_UINT8_T, 0, 41, offsetof(ttalink_set_waypoint_items_t, event_param) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 42, offsetof(ttalink_set_waypoint_items_t, yaw_cmd) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 43, offsetof(ttalink_set_waypoint_items_t, params) }, \
         } \
}
#endif


static inline uint16_t _ttalink_set_waypoint_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, wp_name);
    _tta_put_int32_t(buf, 2, index);
    _tta_put_int32_t(buf, 6, total_num);
    _tta_put_int32_t(buf, 10, latit);
    _tta_put_int32_t(buf, 14, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, speed);
    _tta_put_float(buf, 26, acc_max);
    _tta_put_int32_t(buf, 30, delay_time);
    _tta_put_uint8_t(buf, 34, item_mode);
    _tta_put_float(buf, 35, point_r);
    _tta_put_uint8_t(buf, 39, turn_ctrl);
    _tta_put_uint8_t(buf, 40, event_ctrl);
    _tta_put_uint8_t(buf, 41, event_param);
    _tta_put_uint8_t(buf, 42, yaw_cmd);
    _tta_put_float_array(buf, 43, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN);
#else
    ttalink_set_waypoint_items_t packet;
    packet.cmd = cmd;
    packet.wp_name = wp_name;
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
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param wp_name  这里以后需要改成 8个字节的数组.
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
 * @param event_ctrl  spray_ctrl->envent_ctrl.
 * @param event_param  user_ctrl->event_param.
 * @param yaw_cmd  2：特定角度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    return _ttalink_set_waypoint_items_pack(dst_addr, src_addr, msg,  cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/**
 * @brief Pack a set_waypoint_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param wp_name  这里以后需要改成 8个字节的数组.
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
 * @param event_ctrl  spray_ctrl->envent_ctrl.
 * @param event_param  user_ctrl->event_param.
 * @param yaw_cmd  2：特定角度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    return _ttalink_set_waypoint_items_pack(dst_addr, src_addr, msg,  cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}


static inline uint16_t _ttalink_set_waypoint_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t turn_ctrl,uint8_t event_ctrl,uint8_t event_param,uint8_t yaw_cmd,const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, wp_name);
    _tta_put_int32_t(buf, 2, index);
    _tta_put_int32_t(buf, 6, total_num);
    _tta_put_int32_t(buf, 10, latit);
    _tta_put_int32_t(buf, 14, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, speed);
    _tta_put_float(buf, 26, acc_max);
    _tta_put_int32_t(buf, 30, delay_time);
    _tta_put_uint8_t(buf, 34, item_mode);
    _tta_put_float(buf, 35, point_r);
    _tta_put_uint8_t(buf, 39, turn_ctrl);
    _tta_put_uint8_t(buf, 40, event_ctrl);
    _tta_put_uint8_t(buf, 41, event_param);
    _tta_put_uint8_t(buf, 42, yaw_cmd);
    _tta_put_float_array(buf, 43, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN);
#else
    ttalink_set_waypoint_items_t packet;
    packet.cmd = cmd;
    packet.wp_name = wp_name;
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
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param wp_name  这里以后需要改成 8个字节的数组.
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
 * @param event_ctrl  spray_ctrl->envent_ctrl.
 * @param event_param  user_ctrl->event_param.
 * @param yaw_cmd  2：特定角度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t turn_ctrl,uint8_t event_ctrl,uint8_t event_param,uint8_t yaw_cmd,const float *params)
{
    return _ttalink_set_waypoint_items_pack_chan(dst_addr, src_addr, chan, msg,  cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/**
 * @brief Pack a set_waypoint_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param wp_name  这里以后需要改成 8个字节的数组.
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
 * @param event_ctrl  spray_ctrl->envent_ctrl.
 * @param event_param  user_ctrl->event_param.
 * @param yaw_cmd  2：特定角度.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t turn_ctrl,uint8_t event_ctrl,uint8_t event_param,uint8_t yaw_cmd,const float *params)
{
    return _ttalink_set_waypoint_items_pack_chan(dst_addr, src_addr, chan, msg,  cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}


static inline uint16_t _ttalink_set_waypoint_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_t* set_waypoint_items, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_items_pack_nocrc(dst_addr, src_addr, msg, set_waypoint_items->cmd, set_waypoint_items->wp_name, set_waypoint_items->index, set_waypoint_items->total_num, set_waypoint_items->latit, set_waypoint_items->longi, set_waypoint_items->altit, set_waypoint_items->speed, set_waypoint_items->acc_max, set_waypoint_items->delay_time, set_waypoint_items->item_mode, set_waypoint_items->point_r, set_waypoint_items->turn_ctrl, set_waypoint_items->event_ctrl, set_waypoint_items->event_param, set_waypoint_items->yaw_cmd, set_waypoint_items->params);
    }else{
        return ttalink_set_waypoint_items_pack(dst_addr, src_addr, msg, set_waypoint_items->cmd, set_waypoint_items->wp_name, set_waypoint_items->index, set_waypoint_items->total_num, set_waypoint_items->latit, set_waypoint_items->longi, set_waypoint_items->altit, set_waypoint_items->speed, set_waypoint_items->acc_max, set_waypoint_items->delay_time, set_waypoint_items->item_mode, set_waypoint_items->point_r, set_waypoint_items->turn_ctrl, set_waypoint_items->event_ctrl, set_waypoint_items->event_param, set_waypoint_items->yaw_cmd, set_waypoint_items->params);
    }
    
}

/**
 * @brief Encode a set_waypoint_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_t* set_waypoint_items)
{
    return _ttalink_set_waypoint_items_encode(dst_addr, src_addr, msg, set_waypoint_items, false);
}

/**
 * @brief Encode a set_waypoint_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_items_t* set_waypoint_items)
{
    return _ttalink_set_waypoint_items_encode(dst_addr, src_addr, msg, set_waypoint_items, true);
}


static inline uint16_t _ttalink_set_waypoint_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_t* set_waypoint_items, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, set_waypoint_items->cmd, set_waypoint_items->wp_name, set_waypoint_items->index, set_waypoint_items->total_num, set_waypoint_items->latit, set_waypoint_items->longi, set_waypoint_items->altit, set_waypoint_items->speed, set_waypoint_items->acc_max, set_waypoint_items->delay_time, set_waypoint_items->item_mode, set_waypoint_items->point_r, set_waypoint_items->turn_ctrl, set_waypoint_items->event_ctrl, set_waypoint_items->event_param, set_waypoint_items->yaw_cmd, set_waypoint_items->params);
    }else{
        return ttalink_set_waypoint_items_pack_chan(dst_addr, src_addr, chan, msg, set_waypoint_items->cmd, set_waypoint_items->wp_name, set_waypoint_items->index, set_waypoint_items->total_num, set_waypoint_items->latit, set_waypoint_items->longi, set_waypoint_items->altit, set_waypoint_items->speed, set_waypoint_items->acc_max, set_waypoint_items->delay_time, set_waypoint_items->item_mode, set_waypoint_items->point_r, set_waypoint_items->turn_ctrl, set_waypoint_items->event_ctrl, set_waypoint_items->event_param, set_waypoint_items->yaw_cmd, set_waypoint_items->params);
    }
}

/**
 * @brief Encode a set_waypoint_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_t* set_waypoint_items)
{
    return _ttalink_set_waypoint_items_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_items, false);
}

/**
 * @brief Encode a set_waypoint_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_items_t* set_waypoint_items)
{
    return _ttalink_set_waypoint_items_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_set_waypoint_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, wp_name);
    _tta_put_int32_t(buf, 2, index);
    _tta_put_int32_t(buf, 6, total_num);
    _tta_put_int32_t(buf, 10, latit);
    _tta_put_int32_t(buf, 14, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, speed);
    _tta_put_float(buf, 26, acc_max);
    _tta_put_int32_t(buf, 30, delay_time);
    _tta_put_uint8_t(buf, 34, item_mode);
    _tta_put_float(buf, 35, point_r);
    _tta_put_uint8_t(buf, 39, turn_ctrl);
    _tta_put_uint8_t(buf, 40, event_ctrl);
    _tta_put_uint8_t(buf, 41, event_param);
    _tta_put_uint8_t(buf, 42, yaw_cmd);
    _tta_put_float_array(buf, 43, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS, buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC, nocrc);
#else
    ttalink_set_waypoint_items_t packet;
    packet.cmd = cmd;
    packet.wp_name = wp_name;
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
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS, (const char *)&packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_items message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param wp_name  这里以后需要改成 8个字节的数组.
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
 * @param event_ctrl  spray_ctrl->envent_ctrl.
 * @param event_param  user_ctrl->event_param.
 * @param yaw_cmd  2：特定角度.
 * @param params  .
 */
static inline void ttalink_set_waypoint_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_set_waypoint_items_send(dst_addr, src_addr, chan, cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/**
 * @brief Send a set_waypoint_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param wp_name  这里以后需要改成 8个字节的数组.
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
 * @param event_ctrl  spray_ctrl->envent_ctrl.
 * @param event_param  user_ctrl->event_param.
 * @param yaw_cmd  2：特定角度.
 * @param params  .
 */
static inline void ttalink_set_waypoint_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_set_waypoint_items_send(dst_addr, src_addr, chan, cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}


static inline void _ttalink_set_waypoint_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_t* set_waypoint_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_set_waypoint_items_send_nocrc(dst_addr, src_addr, chan, set_waypoint_items->cmd, set_waypoint_items->wp_name, set_waypoint_items->index, set_waypoint_items->total_num, set_waypoint_items->latit, set_waypoint_items->longi, set_waypoint_items->altit, set_waypoint_items->speed, set_waypoint_items->acc_max, set_waypoint_items->delay_time, set_waypoint_items->item_mode, set_waypoint_items->point_r, set_waypoint_items->turn_ctrl, set_waypoint_items->event_ctrl, set_waypoint_items->event_param, set_waypoint_items->yaw_cmd, set_waypoint_items->params);
    }else{
        ttalink_set_waypoint_items_send(dst_addr, src_addr, chan, set_waypoint_items->cmd, set_waypoint_items->wp_name, set_waypoint_items->index, set_waypoint_items->total_num, set_waypoint_items->latit, set_waypoint_items->longi, set_waypoint_items->altit, set_waypoint_items->speed, set_waypoint_items->acc_max, set_waypoint_items->delay_time, set_waypoint_items->item_mode, set_waypoint_items->point_r, set_waypoint_items->turn_ctrl, set_waypoint_items->event_ctrl, set_waypoint_items->event_param, set_waypoint_items->yaw_cmd, set_waypoint_items->params);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS, (const char *)set_waypoint_items, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_t* set_waypoint_items)
{
    _ttalink_set_waypoint_items_send_struct(dst_addr, src_addr, chan, set_waypoint_items, false);
}

/**
 * @brief Send a set_waypoint_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_items_t* set_waypoint_items)
{
    _ttalink_set_waypoint_items_send_struct(dst_addr, src_addr, chan, set_waypoint_items, true);
}

#if TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_set_waypoint_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, wp_name);
    _tta_put_int32_t(buf, 2, index);
    _tta_put_int32_t(buf, 6, total_num);
    _tta_put_int32_t(buf, 10, latit);
    _tta_put_int32_t(buf, 14, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, speed);
    _tta_put_float(buf, 26, acc_max);
    _tta_put_int32_t(buf, 30, delay_time);
    _tta_put_uint8_t(buf, 34, item_mode);
    _tta_put_float(buf, 35, point_r);
    _tta_put_uint8_t(buf, 39, turn_ctrl);
    _tta_put_uint8_t(buf, 40, event_ctrl);
    _tta_put_uint8_t(buf, 41, event_param);
    _tta_put_uint8_t(buf, 42, yaw_cmd);
    _tta_put_float_array(buf, 43, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS, buf, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC, nocrc);
#else
    ttalink_set_waypoint_items_t *packet = (ttalink_set_waypoint_items_t *)msgbuf;
    packet->cmd = cmd;
    packet->wp_name = wp_name;
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
    tta_array_memcpy(packet->params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS, (const char *)packet, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_set_waypoint_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_set_waypoint_items_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_set_waypoint_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t turn_ctrl, uint8_t event_ctrl, uint8_t event_param, uint8_t yaw_cmd, const float *params)
{
    _ttalink_set_waypoint_items_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, turn_ctrl, event_ctrl, event_param, yaw_cmd, params, true);
}
#endif

#endif

// MESSAGE SET_WAYPOINT_ITEMS UNPACKING


/**
 * @brief Get field cmd from set_waypoint_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_items_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field wp_name from set_waypoint_items message
 *
 * @return  这里以后需要改成 8个字节的数组.
 */
static inline uint8_t ttalink_set_waypoint_items_get_wp_name(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field index from set_waypoint_items message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_items_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  2);
}

/**
 * @brief Get field total_num from set_waypoint_items message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_items_get_total_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  6);
}

/**
 * @brief Get field latit from set_waypoint_items message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_items_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  10);
}

/**
 * @brief Get field longi from set_waypoint_items message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_items_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  14);
}

/**
 * @brief Get field altit from set_waypoint_items message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  18);
}

/**
 * @brief Get field speed from set_waypoint_items message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_get_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  22);
}

/**
 * @brief Get field acc_max from set_waypoint_items message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_get_acc_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Get field delay_time from set_waypoint_items message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_items_get_delay_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  30);
}

/**
 * @brief Get field item_mode from set_waypoint_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_items_get_item_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field point_r from set_waypoint_items message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_items_get_point_r(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  35);
}

/**
 * @brief Get field turn_ctrl from set_waypoint_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_items_get_turn_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field event_ctrl from set_waypoint_items message
 *
 * @return  spray_ctrl->envent_ctrl.
 */
static inline uint8_t ttalink_set_waypoint_items_get_event_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field event_param from set_waypoint_items message
 *
 * @return  user_ctrl->event_param.
 */
static inline uint8_t ttalink_set_waypoint_items_get_event_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field yaw_cmd from set_waypoint_items message
 *
 * @return  2：特定角度.
 */
static inline uint8_t ttalink_set_waypoint_items_get_yaw_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field params from set_waypoint_items message
 *
 * @return  .
 */
static inline uint16_t ttalink_set_waypoint_items_get_params(const ttalink_message_t* msg, float *params)
{
    return _TTA_RETURN_float_array(msg, params, 4,  43);
}

/**
 * @brief Decode a set_waypoint_items message into a struct
 *
 * @param msg The message to decode
 * @param set_waypoint_items C-struct to decode the message contents into
 */
static inline void ttalink_set_waypoint_items_decode(const ttalink_message_t* msg, ttalink_set_waypoint_items_t* set_waypoint_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    set_waypoint_items->cmd = ttalink_set_waypoint_items_get_cmd(msg);
    set_waypoint_items->wp_name = ttalink_set_waypoint_items_get_wp_name(msg);
    set_waypoint_items->index = ttalink_set_waypoint_items_get_index(msg);
    set_waypoint_items->total_num = ttalink_set_waypoint_items_get_total_num(msg);
    set_waypoint_items->latit = ttalink_set_waypoint_items_get_latit(msg);
    set_waypoint_items->longi = ttalink_set_waypoint_items_get_longi(msg);
    set_waypoint_items->altit = ttalink_set_waypoint_items_get_altit(msg);
    set_waypoint_items->speed = ttalink_set_waypoint_items_get_speed(msg);
    set_waypoint_items->acc_max = ttalink_set_waypoint_items_get_acc_max(msg);
    set_waypoint_items->delay_time = ttalink_set_waypoint_items_get_delay_time(msg);
    set_waypoint_items->item_mode = ttalink_set_waypoint_items_get_item_mode(msg);
    set_waypoint_items->point_r = ttalink_set_waypoint_items_get_point_r(msg);
    set_waypoint_items->turn_ctrl = ttalink_set_waypoint_items_get_turn_ctrl(msg);
    set_waypoint_items->event_ctrl = ttalink_set_waypoint_items_get_event_ctrl(msg);
    set_waypoint_items->event_param = ttalink_set_waypoint_items_get_event_param(msg);
    set_waypoint_items->yaw_cmd = ttalink_set_waypoint_items_get_yaw_cmd(msg);
    ttalink_set_waypoint_items_get_params(msg, set_waypoint_items->params);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN? msg->len : TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN;
        memset(set_waypoint_items, 0, TTALINK_MSG_ID_SET_WAYPOINT_ITEMS_LEN);
    memcpy(set_waypoint_items, _TTA_PAYLOAD(msg), len);
#endif
}
