#pragma once
// MESSAGE ROUTE_CONFIG_ITEMS PACKING

#define TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS 2198

TTAPACKED(
typedef struct __ttalink_route_config_items_t {
 float start_alt_param; /*< 起飞高度.*/
 uint8_t way_index; /*< 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).*/
 uint8_t point_index; /*< 切入点.*/
 uint8_t cycle_time; /*<  循环次数.*/
 uint8_t end_item; /*<  完成动作.*/
 uint8_t cycle_type; /*<  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 */
 uint8_t climb_type; /*<  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度*/
 uint8_t receive_flag; /*<  作为收到数据标识.*/
 uint8_t play_break; /*< 执行航线断点（已取消）.*/
 uint16_t delay_time; /*<  .*/
 uint8_t param; /*<  .*/
 uint8_t param1; /*<  .*/
 float wp_width; /*<  传输规划的航线间距信息.*/
 uint32_t wp_target_time; /*<  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.*/
}) ttalink_route_config_items_t;

#define TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN 24
#define TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN 24
#define TTALINK_MSG_ID_2198_LEN 24
#define TTALINK_MSG_ID_2198_MIN_LEN 24

#define TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC 250
#define TTALINK_MSG_ID_2198_CRC 250



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROUTE_CONFIG_ITEMS { \
    2198, \
    "ROUTE_CONFIG_ITEMS", \
    14, \
    {  { "start_alt_param", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_route_config_items_t, start_alt_param) }, \
         { "way_index", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_route_config_items_t, way_index) }, \
         { "point_index", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_route_config_items_t, point_index) }, \
         { "cycle_time", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_route_config_items_t, cycle_time) }, \
         { "end_item", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_route_config_items_t, end_item) }, \
         { "cycle_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_route_config_items_t, cycle_type) }, \
         { "climb_type", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_route_config_items_t, climb_type) }, \
         { "receive_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_route_config_items_t, receive_flag) }, \
         { "play_break", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_route_config_items_t, play_break) }, \
         { "delay_time", NULL, TTALINK_TYPE_UINT16_T, 0, 12, offsetof(ttalink_route_config_items_t, delay_time) }, \
         { "param", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_route_config_items_t, param) }, \
         { "param1", NULL, TTALINK_TYPE_UINT8_T, 0, 15, offsetof(ttalink_route_config_items_t, param1) }, \
         { "wp_width", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_route_config_items_t, wp_width) }, \
         { "wp_target_time", NULL, TTALINK_TYPE_UINT32_T, 0, 20, offsetof(ttalink_route_config_items_t, wp_target_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROUTE_CONFIG_ITEMS { \
    "ROUTE_CONFIG_ITEMS", \
    14, \
    {  { "start_alt_param", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_route_config_items_t, start_alt_param) }, \
         { "way_index", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_route_config_items_t, way_index) }, \
         { "point_index", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_route_config_items_t, point_index) }, \
         { "cycle_time", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_route_config_items_t, cycle_time) }, \
         { "end_item", NULL, TTALINK_TYPE_UINT8_T, 0, 7, offsetof(ttalink_route_config_items_t, end_item) }, \
         { "cycle_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_route_config_items_t, cycle_type) }, \
         { "climb_type", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_route_config_items_t, climb_type) }, \
         { "receive_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_route_config_items_t, receive_flag) }, \
         { "play_break", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_route_config_items_t, play_break) }, \
         { "delay_time", NULL, TTALINK_TYPE_UINT16_T, 0, 12, offsetof(ttalink_route_config_items_t, delay_time) }, \
         { "param", NULL, TTALINK_TYPE_UINT8_T, 0, 14, offsetof(ttalink_route_config_items_t, param) }, \
         { "param1", NULL, TTALINK_TYPE_UINT8_T, 0, 15, offsetof(ttalink_route_config_items_t, param1) }, \
         { "wp_width", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_route_config_items_t, wp_width) }, \
         { "wp_target_time", NULL, TTALINK_TYPE_UINT32_T, 0, 20, offsetof(ttalink_route_config_items_t, wp_target_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_route_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN];
    _tta_put_float(buf, 0, start_alt_param);
    _tta_put_uint8_t(buf, 4, way_index);
    _tta_put_uint8_t(buf, 5, point_index);
    _tta_put_uint8_t(buf, 6, cycle_time);
    _tta_put_uint8_t(buf, 7, end_item);
    _tta_put_uint8_t(buf, 8, cycle_type);
    _tta_put_uint8_t(buf, 9, climb_type);
    _tta_put_uint8_t(buf, 10, receive_flag);
    _tta_put_uint8_t(buf, 11, play_break);
    _tta_put_uint16_t(buf, 12, delay_time);
    _tta_put_uint8_t(buf, 14, param);
    _tta_put_uint8_t(buf, 15, param1);
    _tta_put_float(buf, 16, wp_width);
    _tta_put_uint32_t(buf, 20, wp_target_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN);
#else
    ttalink_route_config_items_t packet;
    packet.start_alt_param = start_alt_param;
    packet.way_index = way_index;
    packet.point_index = point_index;
    packet.cycle_time = cycle_time;
    packet.end_item = end_item;
    packet.cycle_type = cycle_type;
    packet.climb_type = climb_type;
    packet.receive_flag = receive_flag;
    packet.play_break = play_break;
    packet.delay_time = delay_time;
    packet.param = param;
    packet.param1 = param1;
    packet.wp_width = wp_width;
    packet.wp_target_time = wp_target_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a route_config_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param start_alt_param 起飞高度.
 * @param way_index 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).
 * @param point_index 切入点.
 * @param cycle_time  循环次数.
 * @param end_item  完成动作.
 * @param cycle_type  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 
 * @param climb_type  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度
 * @param receive_flag  作为收到数据标识.
 * @param play_break 执行航线断点（已取消）.
 * @param delay_time  .
 * @param param  .
 * @param param1  .
 * @param wp_width  传输规划的航线间距信息.
 * @param wp_target_time  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_route_config_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time)
{
    return _ttalink_route_config_items_pack(dst_addr, src_addr, msg,  start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, false);
}

/**
 * @brief Pack a route_config_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param start_alt_param 起飞高度.
 * @param way_index 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).
 * @param point_index 切入点.
 * @param cycle_time  循环次数.
 * @param end_item  完成动作.
 * @param cycle_type  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 
 * @param climb_type  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度
 * @param receive_flag  作为收到数据标识.
 * @param play_break 执行航线断点（已取消）.
 * @param delay_time  .
 * @param param  .
 * @param param1  .
 * @param wp_width  传输规划的航线间距信息.
 * @param wp_target_time  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_route_config_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time)
{
    return _ttalink_route_config_items_pack(dst_addr, src_addr, msg,  start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, true);
}


static inline uint16_t _ttalink_route_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float start_alt_param,uint8_t way_index,uint8_t point_index,uint8_t cycle_time,uint8_t end_item,uint8_t cycle_type,uint8_t climb_type,uint8_t receive_flag,uint8_t play_break,uint16_t delay_time,uint8_t param,uint8_t param1,float wp_width,uint32_t wp_target_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN];
    _tta_put_float(buf, 0, start_alt_param);
    _tta_put_uint8_t(buf, 4, way_index);
    _tta_put_uint8_t(buf, 5, point_index);
    _tta_put_uint8_t(buf, 6, cycle_time);
    _tta_put_uint8_t(buf, 7, end_item);
    _tta_put_uint8_t(buf, 8, cycle_type);
    _tta_put_uint8_t(buf, 9, climb_type);
    _tta_put_uint8_t(buf, 10, receive_flag);
    _tta_put_uint8_t(buf, 11, play_break);
    _tta_put_uint16_t(buf, 12, delay_time);
    _tta_put_uint8_t(buf, 14, param);
    _tta_put_uint8_t(buf, 15, param1);
    _tta_put_float(buf, 16, wp_width);
    _tta_put_uint32_t(buf, 20, wp_target_time);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN);
#else
    ttalink_route_config_items_t packet;
    packet.start_alt_param = start_alt_param;
    packet.way_index = way_index;
    packet.point_index = point_index;
    packet.cycle_time = cycle_time;
    packet.end_item = end_item;
    packet.cycle_type = cycle_type;
    packet.climb_type = climb_type;
    packet.receive_flag = receive_flag;
    packet.play_break = play_break;
    packet.delay_time = delay_time;
    packet.param = param;
    packet.param1 = param1;
    packet.wp_width = wp_width;
    packet.wp_target_time = wp_target_time;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a route_config_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param start_alt_param 起飞高度.
 * @param way_index 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).
 * @param point_index 切入点.
 * @param cycle_time  循环次数.
 * @param end_item  完成动作.
 * @param cycle_type  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 
 * @param climb_type  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度
 * @param receive_flag  作为收到数据标识.
 * @param play_break 执行航线断点（已取消）.
 * @param delay_time  .
 * @param param  .
 * @param param1  .
 * @param wp_width  传输规划的航线间距信息.
 * @param wp_target_time  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_route_config_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float start_alt_param,uint8_t way_index,uint8_t point_index,uint8_t cycle_time,uint8_t end_item,uint8_t cycle_type,uint8_t climb_type,uint8_t receive_flag,uint8_t play_break,uint16_t delay_time,uint8_t param,uint8_t param1,float wp_width,uint32_t wp_target_time)
{
    return _ttalink_route_config_items_pack_chan(dst_addr, src_addr, chan, msg,  start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, false);
}

/**
 * @brief Pack a route_config_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param start_alt_param 起飞高度.
 * @param way_index 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).
 * @param point_index 切入点.
 * @param cycle_time  循环次数.
 * @param end_item  完成动作.
 * @param cycle_type  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 
 * @param climb_type  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度
 * @param receive_flag  作为收到数据标识.
 * @param play_break 执行航线断点（已取消）.
 * @param delay_time  .
 * @param param  .
 * @param param1  .
 * @param wp_width  传输规划的航线间距信息.
 * @param wp_target_time  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_route_config_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float start_alt_param,uint8_t way_index,uint8_t point_index,uint8_t cycle_time,uint8_t end_item,uint8_t cycle_type,uint8_t climb_type,uint8_t receive_flag,uint8_t play_break,uint16_t delay_time,uint8_t param,uint8_t param1,float wp_width,uint32_t wp_target_time)
{
    return _ttalink_route_config_items_pack_chan(dst_addr, src_addr, chan, msg,  start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, true);
}


static inline uint16_t _ttalink_route_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_route_config_items_t* route_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_route_config_items_pack_nocrc(dst_addr, src_addr, msg, route_config_items->start_alt_param, route_config_items->way_index, route_config_items->point_index, route_config_items->cycle_time, route_config_items->end_item, route_config_items->cycle_type, route_config_items->climb_type, route_config_items->receive_flag, route_config_items->play_break, route_config_items->delay_time, route_config_items->param, route_config_items->param1, route_config_items->wp_width, route_config_items->wp_target_time);
    }else{
        return ttalink_route_config_items_pack(dst_addr, src_addr, msg, route_config_items->start_alt_param, route_config_items->way_index, route_config_items->point_index, route_config_items->cycle_time, route_config_items->end_item, route_config_items->cycle_type, route_config_items->climb_type, route_config_items->receive_flag, route_config_items->play_break, route_config_items->delay_time, route_config_items->param, route_config_items->param1, route_config_items->wp_width, route_config_items->wp_target_time);
    }
    
}

/**
 * @brief Encode a route_config_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param route_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_route_config_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_route_config_items_t* route_config_items)
{
    return _ttalink_route_config_items_encode(dst_addr, src_addr, msg, route_config_items, false);
}

/**
 * @brief Encode a route_config_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param route_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_route_config_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_route_config_items_t* route_config_items)
{
    return _ttalink_route_config_items_encode(dst_addr, src_addr, msg, route_config_items, true);
}


static inline uint16_t _ttalink_route_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_route_config_items_t* route_config_items, bool nocrc)
{
    if(nocrc){
        return ttalink_route_config_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, route_config_items->start_alt_param, route_config_items->way_index, route_config_items->point_index, route_config_items->cycle_time, route_config_items->end_item, route_config_items->cycle_type, route_config_items->climb_type, route_config_items->receive_flag, route_config_items->play_break, route_config_items->delay_time, route_config_items->param, route_config_items->param1, route_config_items->wp_width, route_config_items->wp_target_time);
    }else{
        return ttalink_route_config_items_pack_chan(dst_addr, src_addr, chan, msg, route_config_items->start_alt_param, route_config_items->way_index, route_config_items->point_index, route_config_items->cycle_time, route_config_items->end_item, route_config_items->cycle_type, route_config_items->climb_type, route_config_items->receive_flag, route_config_items->play_break, route_config_items->delay_time, route_config_items->param, route_config_items->param1, route_config_items->wp_width, route_config_items->wp_target_time);
    }
}

/**
 * @brief Encode a route_config_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param route_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_route_config_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_route_config_items_t* route_config_items)
{
    return _ttalink_route_config_items_encode_chan(dst_addr, src_addr, chan, msg, route_config_items, false);
}

/**
 * @brief Encode a route_config_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param route_config_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_route_config_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_route_config_items_t* route_config_items)
{
    return _ttalink_route_config_items_encode_chan(dst_addr, src_addr, chan, msg, route_config_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_route_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN];
    _tta_put_float(buf, 0, start_alt_param);
    _tta_put_uint8_t(buf, 4, way_index);
    _tta_put_uint8_t(buf, 5, point_index);
    _tta_put_uint8_t(buf, 6, cycle_time);
    _tta_put_uint8_t(buf, 7, end_item);
    _tta_put_uint8_t(buf, 8, cycle_type);
    _tta_put_uint8_t(buf, 9, climb_type);
    _tta_put_uint8_t(buf, 10, receive_flag);
    _tta_put_uint8_t(buf, 11, play_break);
    _tta_put_uint16_t(buf, 12, delay_time);
    _tta_put_uint8_t(buf, 14, param);
    _tta_put_uint8_t(buf, 15, param1);
    _tta_put_float(buf, 16, wp_width);
    _tta_put_uint32_t(buf, 20, wp_target_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS, buf, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_route_config_items_t packet;
    packet.start_alt_param = start_alt_param;
    packet.way_index = way_index;
    packet.point_index = point_index;
    packet.cycle_time = cycle_time;
    packet.end_item = end_item;
    packet.cycle_type = cycle_type;
    packet.climb_type = climb_type;
    packet.receive_flag = receive_flag;
    packet.play_break = play_break;
    packet.delay_time = delay_time;
    packet.param = param;
    packet.param1 = param1;
    packet.wp_width = wp_width;
    packet.wp_target_time = wp_target_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS, (const char *)&packet, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a route_config_items message
 * @param chan TTAlink channel to send the message
 *
 * @param start_alt_param 起飞高度.
 * @param way_index 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).
 * @param point_index 切入点.
 * @param cycle_time  循环次数.
 * @param end_item  完成动作.
 * @param cycle_type  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 
 * @param climb_type  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度
 * @param receive_flag  作为收到数据标识.
 * @param play_break 执行航线断点（已取消）.
 * @param delay_time  .
 * @param param  .
 * @param param1  .
 * @param wp_width  传输规划的航线间距信息.
 * @param wp_target_time  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.
 */
static inline void ttalink_route_config_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time)
{
    _ttalink_route_config_items_send(dst_addr, src_addr, chan, start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, false);
}

/**
 * @brief Send a route_config_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param start_alt_param 起飞高度.
 * @param way_index 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).
 * @param point_index 切入点.
 * @param cycle_time  循环次数.
 * @param end_item  完成动作.
 * @param cycle_type  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 
 * @param climb_type  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度
 * @param receive_flag  作为收到数据标识.
 * @param play_break 执行航线断点（已取消）.
 * @param delay_time  .
 * @param param  .
 * @param param1  .
 * @param wp_width  传输规划的航线间距信息.
 * @param wp_target_time  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.
 */
static inline void ttalink_route_config_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time)
{
    _ttalink_route_config_items_send(dst_addr, src_addr, chan, start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, true);
}


static inline void _ttalink_route_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_route_config_items_t* route_config_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_route_config_items_send_nocrc(dst_addr, src_addr, chan, route_config_items->start_alt_param, route_config_items->way_index, route_config_items->point_index, route_config_items->cycle_time, route_config_items->end_item, route_config_items->cycle_type, route_config_items->climb_type, route_config_items->receive_flag, route_config_items->play_break, route_config_items->delay_time, route_config_items->param, route_config_items->param1, route_config_items->wp_width, route_config_items->wp_target_time);
    }else{
        ttalink_route_config_items_send(dst_addr, src_addr, chan, route_config_items->start_alt_param, route_config_items->way_index, route_config_items->point_index, route_config_items->cycle_time, route_config_items->end_item, route_config_items->cycle_type, route_config_items->climb_type, route_config_items->receive_flag, route_config_items->play_break, route_config_items->delay_time, route_config_items->param, route_config_items->param1, route_config_items->wp_width, route_config_items->wp_target_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS, (const char *)route_config_items, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a route_config_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_route_config_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_route_config_items_t* route_config_items)
{
    _ttalink_route_config_items_send_struct(dst_addr, src_addr, chan, route_config_items, false);
}

/**
 * @brief Send a route_config_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_route_config_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_route_config_items_t* route_config_items)
{
    _ttalink_route_config_items_send_struct(dst_addr, src_addr, chan, route_config_items, true);
}

#if TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_route_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 0, start_alt_param);
    _tta_put_uint8_t(buf, 4, way_index);
    _tta_put_uint8_t(buf, 5, point_index);
    _tta_put_uint8_t(buf, 6, cycle_time);
    _tta_put_uint8_t(buf, 7, end_item);
    _tta_put_uint8_t(buf, 8, cycle_type);
    _tta_put_uint8_t(buf, 9, climb_type);
    _tta_put_uint8_t(buf, 10, receive_flag);
    _tta_put_uint8_t(buf, 11, play_break);
    _tta_put_uint16_t(buf, 12, delay_time);
    _tta_put_uint8_t(buf, 14, param);
    _tta_put_uint8_t(buf, 15, param1);
    _tta_put_float(buf, 16, wp_width);
    _tta_put_uint32_t(buf, 20, wp_target_time);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS, buf, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC, nocrc);
#else
    ttalink_route_config_items_t *packet = (ttalink_route_config_items_t *)msgbuf;
    packet->start_alt_param = start_alt_param;
    packet->way_index = way_index;
    packet->point_index = point_index;
    packet->cycle_time = cycle_time;
    packet->end_item = end_item;
    packet->cycle_type = cycle_type;
    packet->climb_type = climb_type;
    packet->receive_flag = receive_flag;
    packet->play_break = play_break;
    packet->delay_time = delay_time;
    packet->param = param;
    packet->param1 = param1;
    packet->wp_width = wp_width;
    packet->wp_target_time = wp_target_time;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS, (const char *)packet, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_MIN_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_route_config_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time)
{
    _ttalink_route_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_route_config_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float start_alt_param, uint8_t way_index, uint8_t point_index, uint8_t cycle_time, uint8_t end_item, uint8_t cycle_type, uint8_t climb_type, uint8_t receive_flag, uint8_t play_break, uint16_t delay_time, uint8_t param, uint8_t param1, float wp_width, uint32_t wp_target_time)
{
    _ttalink_route_config_items_send_buf(dst_addr, src_addr, msgbuf, chan, start_alt_param, way_index, point_index, cycle_time, end_item, cycle_type, climb_type, receive_flag, play_break, delay_time, param, param1, wp_width, wp_target_time, true);
}
#endif

#endif

// MESSAGE ROUTE_CONFIG_ITEMS UNPACKING


/**
 * @brief Get field start_alt_param from route_config_items message
 *
 * @return 起飞高度.
 */
static inline float ttalink_route_config_items_get_start_alt_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  0);
}

/**
 * @brief Get field way_index from route_config_items message
 *
 * @return 航线index(1-20：正常航线 255为航线断点文件)(以后需要改为wp_name).
 */
static inline uint8_t ttalink_route_config_items_get_way_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field point_index from route_config_items message
 *
 * @return 切入点.
 */
static inline uint8_t ttalink_route_config_items_get_point_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field cycle_time from route_config_items message
 *
 * @return  循环次数.
 */
static inline uint8_t ttalink_route_config_items_get_cycle_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field end_item from route_config_items message
 *
 * @return  完成动作.
 */
static inline uint8_t ttalink_route_config_items_get_end_item(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field cycle_type from route_config_items message
 *
 * @return  循环切入点方式. 0:下次从切入点开始循环 1：下次从起始点开始循环 
 */
static inline uint8_t ttalink_route_config_items_get_cycle_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field climb_type from route_config_items message
 *
 * @return  爬升类型. 0：斜率爬升到起始点高度  1：垂直爬升到起始点高度
 */
static inline uint8_t ttalink_route_config_items_get_climb_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field receive_flag from route_config_items message
 *
 * @return  作为收到数据标识.
 */
static inline uint8_t ttalink_route_config_items_get_receive_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field play_break from route_config_items message
 *
 * @return 执行航线断点（已取消）.
 */
static inline uint8_t ttalink_route_config_items_get_play_break(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field delay_time from route_config_items message
 *
 * @return  .
 */
static inline uint16_t ttalink_route_config_items_get_delay_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field param from route_config_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_route_config_items_get_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field param1 from route_config_items message
 *
 * @return  .
 */
static inline uint8_t ttalink_route_config_items_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field wp_width from route_config_items message
 *
 * @return  传输规划的航线间距信息.
 */
static inline float ttalink_route_config_items_get_wp_width(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field wp_target_time from route_config_items message
 *
 * @return  航线执行时间 转换为 UTC 时间执行，比如在 2021.9.26 00:00:00 时刻执行，如果未到则等待到达，如果超过了，则立即执行.
 */
static inline uint32_t ttalink_route_config_items_get_wp_target_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Decode a route_config_items message into a struct
 *
 * @param msg The message to decode
 * @param route_config_items C-struct to decode the message contents into
 */
static inline void ttalink_route_config_items_decode(const ttalink_message_t* msg, ttalink_route_config_items_t* route_config_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    route_config_items->start_alt_param = ttalink_route_config_items_get_start_alt_param(msg);
    route_config_items->way_index = ttalink_route_config_items_get_way_index(msg);
    route_config_items->point_index = ttalink_route_config_items_get_point_index(msg);
    route_config_items->cycle_time = ttalink_route_config_items_get_cycle_time(msg);
    route_config_items->end_item = ttalink_route_config_items_get_end_item(msg);
    route_config_items->cycle_type = ttalink_route_config_items_get_cycle_type(msg);
    route_config_items->climb_type = ttalink_route_config_items_get_climb_type(msg);
    route_config_items->receive_flag = ttalink_route_config_items_get_receive_flag(msg);
    route_config_items->play_break = ttalink_route_config_items_get_play_break(msg);
    route_config_items->delay_time = ttalink_route_config_items_get_delay_time(msg);
    route_config_items->param = ttalink_route_config_items_get_param(msg);
    route_config_items->param1 = ttalink_route_config_items_get_param1(msg);
    route_config_items->wp_width = ttalink_route_config_items_get_wp_width(msg);
    route_config_items->wp_target_time = ttalink_route_config_items_get_wp_target_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN? msg->len : TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN;
        memset(route_config_items, 0, TTALINK_MSG_ID_ROUTE_CONFIG_ITEMS_LEN);
    memcpy(route_config_items, _TTA_PAYLOAD(msg), len);
#endif
}
