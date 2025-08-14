#pragma once
// MESSAGE SET_WAYPOINT_MISSION PACKING

#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION 2196

TTAPACKED(
typedef struct __ttalink_set_waypoint_mission_t {
 uint8_t cmd; /*<  .*/
 int8_t way_id[8]; /*<  这里可以传输航线名称（备用，方便以后扩展文件名方式）.*/
 uint8_t wp_name; /*<  .*/
 int32_t index; /*<  .*/
 int32_t total_num; /*<  .*/
 int32_t latit; /*<  .*/
 int32_t longi; /*<  .*/
 float altit; /*<  .*/
 float speed; /*<  .*/
 float acc_max; /*<  .*/
 int32_t delay_time; /*<  .*/
 uint8_t item_mode; /*<  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.*/
 float point_r; /*<  .*/
 uint8_t yaw_cmd; /*<  1：特定角度.，20，21*/
 float yaw_param[2]; /*< yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度*/
 uint8_t event_device_index; /*<  触发设备需要.action ，0、1、2设备*/
 uint16_t event_ctrl; /*<  action 动作类型 .*/
 uint8_t event_ctrl_extend; /*<  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） */
 uint8_t event_ctrl_switch; /*<  action 选择预留.*/
 float event_param; /*<  value.*/
 float event_eq_time; /*<  等时拍照.参数*/
 float event_eq_space; /*<  等距拍照 参数.*/
 float event_reserve3; /*<  .*/
 float calc_altit; /*<  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.*/
 float calc_region; /*<  第一个点：地块总面积.正常航点：作为航线间距*/
 float calc_reserve1; /*<  */
 int8_t calc_reserve2; /*<  .*/
 int8_t calc_reserve3; /*<  .*/
 int8_t calc_reserve4; /*<  .*/
 int8_t calc_wp_done; /*<  第一个点：循环次数 最后一个点：航线完成动作.*/
 float spk[3]; /*<  用于编队的队形位置.*/
 float spk_param; /*<  用于编队的队形位置预留参数.*/
 float params[4]; /*<  .*/
}) ttalink_set_waypoint_mission_t;

#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN 125
#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN 125
#define TTALINK_MSG_ID_2196_LEN 125
#define TTALINK_MSG_ID_2196_MIN_LEN 125

#define TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC 2
#define TTALINK_MSG_ID_2196_CRC 2

#define TTALINK_MSG_SET_WAYPOINT_MISSION_FIELD_WAY_ID_LEN 8
#define TTALINK_MSG_SET_WAYPOINT_MISSION_FIELD_YAW_PARAM_LEN 2
#define TTALINK_MSG_SET_WAYPOINT_MISSION_FIELD_SPK_LEN 3
#define TTALINK_MSG_SET_WAYPOINT_MISSION_FIELD_PARAMS_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_MISSION { \
    2196, \
    "SET_WAYPOINT_MISSION", \
    33, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_mission_t, cmd) }, \
         { "way_id", NULL, TTALINK_TYPE_INT8_T, 8, 1, offsetof(ttalink_set_waypoint_mission_t, way_id) }, \
         { "wp_name", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_set_waypoint_mission_t, wp_name) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 10, offsetof(ttalink_set_waypoint_mission_t, index) }, \
         { "total_num", NULL, TTALINK_TYPE_INT32_T, 0, 14, offsetof(ttalink_set_waypoint_mission_t, total_num) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 18, offsetof(ttalink_set_waypoint_mission_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 22, offsetof(ttalink_set_waypoint_mission_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_set_waypoint_mission_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_set_waypoint_mission_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_set_waypoint_mission_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 38, offsetof(ttalink_set_waypoint_mission_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 42, offsetof(ttalink_set_waypoint_mission_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_set_waypoint_mission_t, point_r) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 47, offsetof(ttalink_set_waypoint_mission_t, yaw_cmd) }, \
         { "yaw_param", NULL, TTALINK_TYPE_FLOAT, 2, 48, offsetof(ttalink_set_waypoint_mission_t, yaw_param) }, \
         { "event_device_index", NULL, TTALINK_TYPE_UINT8_T, 0, 56, offsetof(ttalink_set_waypoint_mission_t, event_device_index) }, \
         { "event_ctrl", NULL, TTALINK_TYPE_UINT16_T, 0, 57, offsetof(ttalink_set_waypoint_mission_t, event_ctrl) }, \
         { "event_ctrl_extend", NULL, TTALINK_TYPE_UINT8_T, 0, 59, offsetof(ttalink_set_waypoint_mission_t, event_ctrl_extend) }, \
         { "event_ctrl_switch", NULL, TTALINK_TYPE_UINT8_T, 0, 60, offsetof(ttalink_set_waypoint_mission_t, event_ctrl_switch) }, \
         { "event_param", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_set_waypoint_mission_t, event_param) }, \
         { "event_eq_time", NULL, TTALINK_TYPE_FLOAT, 0, 65, offsetof(ttalink_set_waypoint_mission_t, event_eq_time) }, \
         { "event_eq_space", NULL, TTALINK_TYPE_FLOAT, 0, 69, offsetof(ttalink_set_waypoint_mission_t, event_eq_space) }, \
         { "event_reserve3", NULL, TTALINK_TYPE_FLOAT, 0, 73, offsetof(ttalink_set_waypoint_mission_t, event_reserve3) }, \
         { "calc_altit", NULL, TTALINK_TYPE_FLOAT, 0, 77, offsetof(ttalink_set_waypoint_mission_t, calc_altit) }, \
         { "calc_region", NULL, TTALINK_TYPE_FLOAT, 0, 81, offsetof(ttalink_set_waypoint_mission_t, calc_region) }, \
         { "calc_reserve1", NULL, TTALINK_TYPE_FLOAT, 0, 85, offsetof(ttalink_set_waypoint_mission_t, calc_reserve1) }, \
         { "calc_reserve2", NULL, TTALINK_TYPE_INT8_T, 0, 89, offsetof(ttalink_set_waypoint_mission_t, calc_reserve2) }, \
         { "calc_reserve3", NULL, TTALINK_TYPE_INT8_T, 0, 90, offsetof(ttalink_set_waypoint_mission_t, calc_reserve3) }, \
         { "calc_reserve4", NULL, TTALINK_TYPE_INT8_T, 0, 91, offsetof(ttalink_set_waypoint_mission_t, calc_reserve4) }, \
         { "calc_wp_done", NULL, TTALINK_TYPE_INT8_T, 0, 92, offsetof(ttalink_set_waypoint_mission_t, calc_wp_done) }, \
         { "spk", NULL, TTALINK_TYPE_FLOAT, 3, 93, offsetof(ttalink_set_waypoint_mission_t, spk) }, \
         { "spk_param", NULL, TTALINK_TYPE_FLOAT, 0, 105, offsetof(ttalink_set_waypoint_mission_t, spk_param) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 109, offsetof(ttalink_set_waypoint_mission_t, params) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SET_WAYPOINT_MISSION { \
    "SET_WAYPOINT_MISSION", \
    33, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_set_waypoint_mission_t, cmd) }, \
         { "way_id", NULL, TTALINK_TYPE_INT8_T, 8, 1, offsetof(ttalink_set_waypoint_mission_t, way_id) }, \
         { "wp_name", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_set_waypoint_mission_t, wp_name) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 10, offsetof(ttalink_set_waypoint_mission_t, index) }, \
         { "total_num", NULL, TTALINK_TYPE_INT32_T, 0, 14, offsetof(ttalink_set_waypoint_mission_t, total_num) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 18, offsetof(ttalink_set_waypoint_mission_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 22, offsetof(ttalink_set_waypoint_mission_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_set_waypoint_mission_t, altit) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_set_waypoint_mission_t, speed) }, \
         { "acc_max", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_set_waypoint_mission_t, acc_max) }, \
         { "delay_time", NULL, TTALINK_TYPE_INT32_T, 0, 38, offsetof(ttalink_set_waypoint_mission_t, delay_time) }, \
         { "item_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 42, offsetof(ttalink_set_waypoint_mission_t, item_mode) }, \
         { "point_r", NULL, TTALINK_TYPE_FLOAT, 0, 43, offsetof(ttalink_set_waypoint_mission_t, point_r) }, \
         { "yaw_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 47, offsetof(ttalink_set_waypoint_mission_t, yaw_cmd) }, \
         { "yaw_param", NULL, TTALINK_TYPE_FLOAT, 2, 48, offsetof(ttalink_set_waypoint_mission_t, yaw_param) }, \
         { "event_device_index", NULL, TTALINK_TYPE_UINT8_T, 0, 56, offsetof(ttalink_set_waypoint_mission_t, event_device_index) }, \
         { "event_ctrl", NULL, TTALINK_TYPE_UINT16_T, 0, 57, offsetof(ttalink_set_waypoint_mission_t, event_ctrl) }, \
         { "event_ctrl_extend", NULL, TTALINK_TYPE_UINT8_T, 0, 59, offsetof(ttalink_set_waypoint_mission_t, event_ctrl_extend) }, \
         { "event_ctrl_switch", NULL, TTALINK_TYPE_UINT8_T, 0, 60, offsetof(ttalink_set_waypoint_mission_t, event_ctrl_switch) }, \
         { "event_param", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_set_waypoint_mission_t, event_param) }, \
         { "event_eq_time", NULL, TTALINK_TYPE_FLOAT, 0, 65, offsetof(ttalink_set_waypoint_mission_t, event_eq_time) }, \
         { "event_eq_space", NULL, TTALINK_TYPE_FLOAT, 0, 69, offsetof(ttalink_set_waypoint_mission_t, event_eq_space) }, \
         { "event_reserve3", NULL, TTALINK_TYPE_FLOAT, 0, 73, offsetof(ttalink_set_waypoint_mission_t, event_reserve3) }, \
         { "calc_altit", NULL, TTALINK_TYPE_FLOAT, 0, 77, offsetof(ttalink_set_waypoint_mission_t, calc_altit) }, \
         { "calc_region", NULL, TTALINK_TYPE_FLOAT, 0, 81, offsetof(ttalink_set_waypoint_mission_t, calc_region) }, \
         { "calc_reserve1", NULL, TTALINK_TYPE_FLOAT, 0, 85, offsetof(ttalink_set_waypoint_mission_t, calc_reserve1) }, \
         { "calc_reserve2", NULL, TTALINK_TYPE_INT8_T, 0, 89, offsetof(ttalink_set_waypoint_mission_t, calc_reserve2) }, \
         { "calc_reserve3", NULL, TTALINK_TYPE_INT8_T, 0, 90, offsetof(ttalink_set_waypoint_mission_t, calc_reserve3) }, \
         { "calc_reserve4", NULL, TTALINK_TYPE_INT8_T, 0, 91, offsetof(ttalink_set_waypoint_mission_t, calc_reserve4) }, \
         { "calc_wp_done", NULL, TTALINK_TYPE_INT8_T, 0, 92, offsetof(ttalink_set_waypoint_mission_t, calc_wp_done) }, \
         { "spk", NULL, TTALINK_TYPE_FLOAT, 3, 93, offsetof(ttalink_set_waypoint_mission_t, spk) }, \
         { "spk_param", NULL, TTALINK_TYPE_FLOAT, 0, 105, offsetof(ttalink_set_waypoint_mission_t, spk_param) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 109, offsetof(ttalink_set_waypoint_mission_t, params) }, \
         } \
}
#endif


static inline uint16_t _ttalink_set_waypoint_mission_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 9, wp_name);
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
    _tta_put_uint8_t(buf, 47, yaw_cmd);
    _tta_put_uint8_t(buf, 56, event_device_index);
    _tta_put_uint16_t(buf, 57, event_ctrl);
    _tta_put_uint8_t(buf, 59, event_ctrl_extend);
    _tta_put_uint8_t(buf, 60, event_ctrl_switch);
    _tta_put_float(buf, 61, event_param);
    _tta_put_float(buf, 65, event_eq_time);
    _tta_put_float(buf, 69, event_eq_space);
    _tta_put_float(buf, 73, event_reserve3);
    _tta_put_float(buf, 77, calc_altit);
    _tta_put_float(buf, 81, calc_region);
    _tta_put_float(buf, 85, calc_reserve1);
    _tta_put_int8_t(buf, 89, calc_reserve2);
    _tta_put_int8_t(buf, 90, calc_reserve3);
    _tta_put_int8_t(buf, 91, calc_reserve4);
    _tta_put_int8_t(buf, 92, calc_wp_done);
    _tta_put_float(buf, 105, spk_param);
    _tta_put_int8_t_array(buf, 1, way_id, 8);
    _tta_put_float_array(buf, 48, yaw_param, 2);
    _tta_put_float_array(buf, 93, spk, 3);
    _tta_put_float_array(buf, 109, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN);
#else
    ttalink_set_waypoint_mission_t packet;
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
    packet.yaw_cmd = yaw_cmd;
    packet.event_device_index = event_device_index;
    packet.event_ctrl = event_ctrl;
    packet.event_ctrl_extend = event_ctrl_extend;
    packet.event_ctrl_switch = event_ctrl_switch;
    packet.event_param = event_param;
    packet.event_eq_time = event_eq_time;
    packet.event_eq_space = event_eq_space;
    packet.event_reserve3 = event_reserve3;
    packet.calc_altit = calc_altit;
    packet.calc_region = calc_region;
    packet.calc_reserve1 = calc_reserve1;
    packet.calc_reserve2 = calc_reserve2;
    packet.calc_reserve3 = calc_reserve3;
    packet.calc_reserve4 = calc_reserve4;
    packet.calc_wp_done = calc_wp_done;
    packet.spk_param = spk_param;
    tta_array_memcpy(packet.way_id, way_id, sizeof(int8_t)*8);
    tta_array_memcpy(packet.yaw_param, yaw_param, sizeof(float)*2);
    tta_array_memcpy(packet.spk, spk, sizeof(float)*3);
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_MISSION;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_mission message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param way_id  这里可以传输航线名称（备用，方便以后扩展文件名方式）.
 * @param wp_name  .
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.
 * @param point_r  .
 * @param yaw_cmd  1：特定角度.，20，21
 * @param yaw_param yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度
 * @param event_device_index  触发设备需要.action ，0、1、2设备
 * @param event_ctrl  action 动作类型 .
 * @param event_ctrl_extend  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） 
 * @param event_ctrl_switch  action 选择预留.
 * @param event_param  value.
 * @param event_eq_time  等时拍照.参数
 * @param event_eq_space  等距拍照 参数.
 * @param event_reserve3  .
 * @param calc_altit  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.
 * @param calc_region  第一个点：地块总面积.正常航点：作为航线间距
 * @param calc_reserve1  
 * @param calc_reserve2  .
 * @param calc_reserve3  .
 * @param calc_reserve4  .
 * @param calc_wp_done  第一个点：循环次数 最后一个点：航线完成动作.
 * @param spk  用于编队的队形位置.
 * @param spk_param  用于编队的队形位置预留参数.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params)
{
    return _ttalink_set_waypoint_mission_pack(dst_addr, src_addr, msg,  cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, false);
}

/**
 * @brief Pack a set_waypoint_mission message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd  .
 * @param way_id  这里可以传输航线名称（备用，方便以后扩展文件名方式）.
 * @param wp_name  .
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.
 * @param point_r  .
 * @param yaw_cmd  1：特定角度.，20，21
 * @param yaw_param yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度
 * @param event_device_index  触发设备需要.action ，0、1、2设备
 * @param event_ctrl  action 动作类型 .
 * @param event_ctrl_extend  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） 
 * @param event_ctrl_switch  action 选择预留.
 * @param event_param  value.
 * @param event_eq_time  等时拍照.参数
 * @param event_eq_space  等距拍照 参数.
 * @param event_reserve3  .
 * @param calc_altit  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.
 * @param calc_region  第一个点：地块总面积.正常航点：作为航线间距
 * @param calc_reserve1  
 * @param calc_reserve2  .
 * @param calc_reserve3  .
 * @param calc_reserve4  .
 * @param calc_wp_done  第一个点：循环次数 最后一个点：航线完成动作.
 * @param spk  用于编队的队形位置.
 * @param spk_param  用于编队的队形位置预留参数.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params)
{
    return _ttalink_set_waypoint_mission_pack(dst_addr, src_addr, msg,  cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, true);
}


static inline uint16_t _ttalink_set_waypoint_mission_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,const int8_t *way_id,uint8_t wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t yaw_cmd,const float *yaw_param,uint8_t event_device_index,uint16_t event_ctrl,uint8_t event_ctrl_extend,uint8_t event_ctrl_switch,float event_param,float event_eq_time,float event_eq_space,float event_reserve3,float calc_altit,float calc_region,float calc_reserve1,int8_t calc_reserve2,int8_t calc_reserve3,int8_t calc_reserve4,int8_t calc_wp_done,const float *spk,float spk_param,const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 9, wp_name);
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
    _tta_put_uint8_t(buf, 47, yaw_cmd);
    _tta_put_uint8_t(buf, 56, event_device_index);
    _tta_put_uint16_t(buf, 57, event_ctrl);
    _tta_put_uint8_t(buf, 59, event_ctrl_extend);
    _tta_put_uint8_t(buf, 60, event_ctrl_switch);
    _tta_put_float(buf, 61, event_param);
    _tta_put_float(buf, 65, event_eq_time);
    _tta_put_float(buf, 69, event_eq_space);
    _tta_put_float(buf, 73, event_reserve3);
    _tta_put_float(buf, 77, calc_altit);
    _tta_put_float(buf, 81, calc_region);
    _tta_put_float(buf, 85, calc_reserve1);
    _tta_put_int8_t(buf, 89, calc_reserve2);
    _tta_put_int8_t(buf, 90, calc_reserve3);
    _tta_put_int8_t(buf, 91, calc_reserve4);
    _tta_put_int8_t(buf, 92, calc_wp_done);
    _tta_put_float(buf, 105, spk_param);
    _tta_put_int8_t_array(buf, 1, way_id, 8);
    _tta_put_float_array(buf, 48, yaw_param, 2);
    _tta_put_float_array(buf, 93, spk, 3);
    _tta_put_float_array(buf, 109, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN);
#else
    ttalink_set_waypoint_mission_t packet;
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
    packet.yaw_cmd = yaw_cmd;
    packet.event_device_index = event_device_index;
    packet.event_ctrl = event_ctrl;
    packet.event_ctrl_extend = event_ctrl_extend;
    packet.event_ctrl_switch = event_ctrl_switch;
    packet.event_param = event_param;
    packet.event_eq_time = event_eq_time;
    packet.event_eq_space = event_eq_space;
    packet.event_reserve3 = event_reserve3;
    packet.calc_altit = calc_altit;
    packet.calc_region = calc_region;
    packet.calc_reserve1 = calc_reserve1;
    packet.calc_reserve2 = calc_reserve2;
    packet.calc_reserve3 = calc_reserve3;
    packet.calc_reserve4 = calc_reserve4;
    packet.calc_wp_done = calc_wp_done;
    packet.spk_param = spk_param;
    tta_array_memcpy(packet.way_id, way_id, sizeof(int8_t)*8);
    tta_array_memcpy(packet.yaw_param, yaw_param, sizeof(float)*2);
    tta_array_memcpy(packet.spk, spk, sizeof(float)*3);
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SET_WAYPOINT_MISSION;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC, nocrc);
}

/**
 * @brief Pack a set_waypoint_mission message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param way_id  这里可以传输航线名称（备用，方便以后扩展文件名方式）.
 * @param wp_name  .
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.
 * @param point_r  .
 * @param yaw_cmd  1：特定角度.，20，21
 * @param yaw_param yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度
 * @param event_device_index  触发设备需要.action ，0、1、2设备
 * @param event_ctrl  action 动作类型 .
 * @param event_ctrl_extend  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） 
 * @param event_ctrl_switch  action 选择预留.
 * @param event_param  value.
 * @param event_eq_time  等时拍照.参数
 * @param event_eq_space  等距拍照 参数.
 * @param event_reserve3  .
 * @param calc_altit  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.
 * @param calc_region  第一个点：地块总面积.正常航点：作为航线间距
 * @param calc_reserve1  
 * @param calc_reserve2  .
 * @param calc_reserve3  .
 * @param calc_reserve4  .
 * @param calc_wp_done  第一个点：循环次数 最后一个点：航线完成动作.
 * @param spk  用于编队的队形位置.
 * @param spk_param  用于编队的队形位置预留参数.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,const int8_t *way_id,uint8_t wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t yaw_cmd,const float *yaw_param,uint8_t event_device_index,uint16_t event_ctrl,uint8_t event_ctrl_extend,uint8_t event_ctrl_switch,float event_param,float event_eq_time,float event_eq_space,float event_reserve3,float calc_altit,float calc_region,float calc_reserve1,int8_t calc_reserve2,int8_t calc_reserve3,int8_t calc_reserve4,int8_t calc_wp_done,const float *spk,float spk_param,const float *params)
{
    return _ttalink_set_waypoint_mission_pack_chan(dst_addr, src_addr, chan, msg,  cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, false);
}

/**
 * @brief Pack a set_waypoint_mission message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd  .
 * @param way_id  这里可以传输航线名称（备用，方便以后扩展文件名方式）.
 * @param wp_name  .
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.
 * @param point_r  .
 * @param yaw_cmd  1：特定角度.，20，21
 * @param yaw_param yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度
 * @param event_device_index  触发设备需要.action ，0、1、2设备
 * @param event_ctrl  action 动作类型 .
 * @param event_ctrl_extend  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） 
 * @param event_ctrl_switch  action 选择预留.
 * @param event_param  value.
 * @param event_eq_time  等时拍照.参数
 * @param event_eq_space  等距拍照 参数.
 * @param event_reserve3  .
 * @param calc_altit  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.
 * @param calc_region  第一个点：地块总面积.正常航点：作为航线间距
 * @param calc_reserve1  
 * @param calc_reserve2  .
 * @param calc_reserve3  .
 * @param calc_reserve4  .
 * @param calc_wp_done  第一个点：循环次数 最后一个点：航线完成动作.
 * @param spk  用于编队的队形位置.
 * @param spk_param  用于编队的队形位置预留参数.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_set_waypoint_mission_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,const int8_t *way_id,uint8_t wp_name,int32_t index,int32_t total_num,int32_t latit,int32_t longi,float altit,float speed,float acc_max,int32_t delay_time,uint8_t item_mode,float point_r,uint8_t yaw_cmd,const float *yaw_param,uint8_t event_device_index,uint16_t event_ctrl,uint8_t event_ctrl_extend,uint8_t event_ctrl_switch,float event_param,float event_eq_time,float event_eq_space,float event_reserve3,float calc_altit,float calc_region,float calc_reserve1,int8_t calc_reserve2,int8_t calc_reserve3,int8_t calc_reserve4,int8_t calc_wp_done,const float *spk,float spk_param,const float *params)
{
    return _ttalink_set_waypoint_mission_pack_chan(dst_addr, src_addr, chan, msg,  cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, true);
}


static inline uint16_t _ttalink_set_waypoint_mission_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_mission_t* set_waypoint_mission, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_mission_pack_nocrc(dst_addr, src_addr, msg, set_waypoint_mission->cmd, set_waypoint_mission->way_id, set_waypoint_mission->wp_name, set_waypoint_mission->index, set_waypoint_mission->total_num, set_waypoint_mission->latit, set_waypoint_mission->longi, set_waypoint_mission->altit, set_waypoint_mission->speed, set_waypoint_mission->acc_max, set_waypoint_mission->delay_time, set_waypoint_mission->item_mode, set_waypoint_mission->point_r, set_waypoint_mission->yaw_cmd, set_waypoint_mission->yaw_param, set_waypoint_mission->event_device_index, set_waypoint_mission->event_ctrl, set_waypoint_mission->event_ctrl_extend, set_waypoint_mission->event_ctrl_switch, set_waypoint_mission->event_param, set_waypoint_mission->event_eq_time, set_waypoint_mission->event_eq_space, set_waypoint_mission->event_reserve3, set_waypoint_mission->calc_altit, set_waypoint_mission->calc_region, set_waypoint_mission->calc_reserve1, set_waypoint_mission->calc_reserve2, set_waypoint_mission->calc_reserve3, set_waypoint_mission->calc_reserve4, set_waypoint_mission->calc_wp_done, set_waypoint_mission->spk, set_waypoint_mission->spk_param, set_waypoint_mission->params);
    }else{
        return ttalink_set_waypoint_mission_pack(dst_addr, src_addr, msg, set_waypoint_mission->cmd, set_waypoint_mission->way_id, set_waypoint_mission->wp_name, set_waypoint_mission->index, set_waypoint_mission->total_num, set_waypoint_mission->latit, set_waypoint_mission->longi, set_waypoint_mission->altit, set_waypoint_mission->speed, set_waypoint_mission->acc_max, set_waypoint_mission->delay_time, set_waypoint_mission->item_mode, set_waypoint_mission->point_r, set_waypoint_mission->yaw_cmd, set_waypoint_mission->yaw_param, set_waypoint_mission->event_device_index, set_waypoint_mission->event_ctrl, set_waypoint_mission->event_ctrl_extend, set_waypoint_mission->event_ctrl_switch, set_waypoint_mission->event_param, set_waypoint_mission->event_eq_time, set_waypoint_mission->event_eq_space, set_waypoint_mission->event_reserve3, set_waypoint_mission->calc_altit, set_waypoint_mission->calc_region, set_waypoint_mission->calc_reserve1, set_waypoint_mission->calc_reserve2, set_waypoint_mission->calc_reserve3, set_waypoint_mission->calc_reserve4, set_waypoint_mission->calc_wp_done, set_waypoint_mission->spk, set_waypoint_mission->spk_param, set_waypoint_mission->params);
    }
    
}

/**
 * @brief Encode a set_waypoint_mission struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_mission_t* set_waypoint_mission)
{
    return _ttalink_set_waypoint_mission_encode(dst_addr, src_addr, msg, set_waypoint_mission, false);
}

/**
 * @brief Encode a set_waypoint_mission struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_set_waypoint_mission_t* set_waypoint_mission)
{
    return _ttalink_set_waypoint_mission_encode(dst_addr, src_addr, msg, set_waypoint_mission, true);
}


static inline uint16_t _ttalink_set_waypoint_mission_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_mission_t* set_waypoint_mission, bool nocrc)
{
    if(nocrc){
        return ttalink_set_waypoint_mission_pack_chan_nocrc(dst_addr, src_addr, chan, msg, set_waypoint_mission->cmd, set_waypoint_mission->way_id, set_waypoint_mission->wp_name, set_waypoint_mission->index, set_waypoint_mission->total_num, set_waypoint_mission->latit, set_waypoint_mission->longi, set_waypoint_mission->altit, set_waypoint_mission->speed, set_waypoint_mission->acc_max, set_waypoint_mission->delay_time, set_waypoint_mission->item_mode, set_waypoint_mission->point_r, set_waypoint_mission->yaw_cmd, set_waypoint_mission->yaw_param, set_waypoint_mission->event_device_index, set_waypoint_mission->event_ctrl, set_waypoint_mission->event_ctrl_extend, set_waypoint_mission->event_ctrl_switch, set_waypoint_mission->event_param, set_waypoint_mission->event_eq_time, set_waypoint_mission->event_eq_space, set_waypoint_mission->event_reserve3, set_waypoint_mission->calc_altit, set_waypoint_mission->calc_region, set_waypoint_mission->calc_reserve1, set_waypoint_mission->calc_reserve2, set_waypoint_mission->calc_reserve3, set_waypoint_mission->calc_reserve4, set_waypoint_mission->calc_wp_done, set_waypoint_mission->spk, set_waypoint_mission->spk_param, set_waypoint_mission->params);
    }else{
        return ttalink_set_waypoint_mission_pack_chan(dst_addr, src_addr, chan, msg, set_waypoint_mission->cmd, set_waypoint_mission->way_id, set_waypoint_mission->wp_name, set_waypoint_mission->index, set_waypoint_mission->total_num, set_waypoint_mission->latit, set_waypoint_mission->longi, set_waypoint_mission->altit, set_waypoint_mission->speed, set_waypoint_mission->acc_max, set_waypoint_mission->delay_time, set_waypoint_mission->item_mode, set_waypoint_mission->point_r, set_waypoint_mission->yaw_cmd, set_waypoint_mission->yaw_param, set_waypoint_mission->event_device_index, set_waypoint_mission->event_ctrl, set_waypoint_mission->event_ctrl_extend, set_waypoint_mission->event_ctrl_switch, set_waypoint_mission->event_param, set_waypoint_mission->event_eq_time, set_waypoint_mission->event_eq_space, set_waypoint_mission->event_reserve3, set_waypoint_mission->calc_altit, set_waypoint_mission->calc_region, set_waypoint_mission->calc_reserve1, set_waypoint_mission->calc_reserve2, set_waypoint_mission->calc_reserve3, set_waypoint_mission->calc_reserve4, set_waypoint_mission->calc_wp_done, set_waypoint_mission->spk, set_waypoint_mission->spk_param, set_waypoint_mission->params);
    }
}

/**
 * @brief Encode a set_waypoint_mission struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_mission_t* set_waypoint_mission)
{
    return _ttalink_set_waypoint_mission_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_mission, false);
}

/**
 * @brief Encode a set_waypoint_mission struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param set_waypoint_mission C-struct to read the message contents from
 */
static inline uint16_t ttalink_set_waypoint_mission_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_set_waypoint_mission_t* set_waypoint_mission)
{
    return _ttalink_set_waypoint_mission_encode_chan(dst_addr, src_addr, chan, msg, set_waypoint_mission, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_set_waypoint_mission_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 9, wp_name);
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
    _tta_put_uint8_t(buf, 47, yaw_cmd);
    _tta_put_uint8_t(buf, 56, event_device_index);
    _tta_put_uint16_t(buf, 57, event_ctrl);
    _tta_put_uint8_t(buf, 59, event_ctrl_extend);
    _tta_put_uint8_t(buf, 60, event_ctrl_switch);
    _tta_put_float(buf, 61, event_param);
    _tta_put_float(buf, 65, event_eq_time);
    _tta_put_float(buf, 69, event_eq_space);
    _tta_put_float(buf, 73, event_reserve3);
    _tta_put_float(buf, 77, calc_altit);
    _tta_put_float(buf, 81, calc_region);
    _tta_put_float(buf, 85, calc_reserve1);
    _tta_put_int8_t(buf, 89, calc_reserve2);
    _tta_put_int8_t(buf, 90, calc_reserve3);
    _tta_put_int8_t(buf, 91, calc_reserve4);
    _tta_put_int8_t(buf, 92, calc_wp_done);
    _tta_put_float(buf, 105, spk_param);
    _tta_put_int8_t_array(buf, 1, way_id, 8);
    _tta_put_float_array(buf, 48, yaw_param, 2);
    _tta_put_float_array(buf, 93, spk, 3);
    _tta_put_float_array(buf, 109, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION, buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC, nocrc);
#else
    ttalink_set_waypoint_mission_t packet;
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
    packet.yaw_cmd = yaw_cmd;
    packet.event_device_index = event_device_index;
    packet.event_ctrl = event_ctrl;
    packet.event_ctrl_extend = event_ctrl_extend;
    packet.event_ctrl_switch = event_ctrl_switch;
    packet.event_param = event_param;
    packet.event_eq_time = event_eq_time;
    packet.event_eq_space = event_eq_space;
    packet.event_reserve3 = event_reserve3;
    packet.calc_altit = calc_altit;
    packet.calc_region = calc_region;
    packet.calc_reserve1 = calc_reserve1;
    packet.calc_reserve2 = calc_reserve2;
    packet.calc_reserve3 = calc_reserve3;
    packet.calc_reserve4 = calc_reserve4;
    packet.calc_wp_done = calc_wp_done;
    packet.spk_param = spk_param;
    tta_array_memcpy(packet.way_id, way_id, sizeof(int8_t)*8);
    tta_array_memcpy(packet.yaw_param, yaw_param, sizeof(float)*2);
    tta_array_memcpy(packet.spk, spk, sizeof(float)*3);
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION, (const char *)&packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_mission message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param way_id  这里可以传输航线名称（备用，方便以后扩展文件名方式）.
 * @param wp_name  .
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.
 * @param point_r  .
 * @param yaw_cmd  1：特定角度.，20，21
 * @param yaw_param yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度
 * @param event_device_index  触发设备需要.action ，0、1、2设备
 * @param event_ctrl  action 动作类型 .
 * @param event_ctrl_extend  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） 
 * @param event_ctrl_switch  action 选择预留.
 * @param event_param  value.
 * @param event_eq_time  等时拍照.参数
 * @param event_eq_space  等距拍照 参数.
 * @param event_reserve3  .
 * @param calc_altit  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.
 * @param calc_region  第一个点：地块总面积.正常航点：作为航线间距
 * @param calc_reserve1  
 * @param calc_reserve2  .
 * @param calc_reserve3  .
 * @param calc_reserve4  .
 * @param calc_wp_done  第一个点：循环次数 最后一个点：航线完成动作.
 * @param spk  用于编队的队形位置.
 * @param spk_param  用于编队的队形位置预留参数.
 * @param params  .
 */
static inline void ttalink_set_waypoint_mission_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params)
{
    _ttalink_set_waypoint_mission_send(dst_addr, src_addr, chan, cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, false);
}

/**
 * @brief Send a set_waypoint_mission message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd  .
 * @param way_id  这里可以传输航线名称（备用，方便以后扩展文件名方式）.
 * @param wp_name  .
 * @param index  .
 * @param total_num  .
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param speed  .
 * @param acc_max  .
 * @param delay_time  .
 * @param item_mode  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.
 * @param point_r  .
 * @param yaw_cmd  1：特定角度.，20，21
 * @param yaw_param yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度
 * @param event_device_index  触发设备需要.action ，0、1、2设备
 * @param event_ctrl  action 动作类型 .
 * @param event_ctrl_extend  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） 
 * @param event_ctrl_switch  action 选择预留.
 * @param event_param  value.
 * @param event_eq_time  等时拍照.参数
 * @param event_eq_space  等距拍照 参数.
 * @param event_reserve3  .
 * @param calc_altit  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.
 * @param calc_region  第一个点：地块总面积.正常航点：作为航线间距
 * @param calc_reserve1  
 * @param calc_reserve2  .
 * @param calc_reserve3  .
 * @param calc_reserve4  .
 * @param calc_wp_done  第一个点：循环次数 最后一个点：航线完成动作.
 * @param spk  用于编队的队形位置.
 * @param spk_param  用于编队的队形位置预留参数.
 * @param params  .
 */
static inline void ttalink_set_waypoint_mission_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params)
{
    _ttalink_set_waypoint_mission_send(dst_addr, src_addr, chan, cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, true);
}


static inline void _ttalink_set_waypoint_mission_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_mission_t* set_waypoint_mission, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_set_waypoint_mission_send_nocrc(dst_addr, src_addr, chan, set_waypoint_mission->cmd, set_waypoint_mission->way_id, set_waypoint_mission->wp_name, set_waypoint_mission->index, set_waypoint_mission->total_num, set_waypoint_mission->latit, set_waypoint_mission->longi, set_waypoint_mission->altit, set_waypoint_mission->speed, set_waypoint_mission->acc_max, set_waypoint_mission->delay_time, set_waypoint_mission->item_mode, set_waypoint_mission->point_r, set_waypoint_mission->yaw_cmd, set_waypoint_mission->yaw_param, set_waypoint_mission->event_device_index, set_waypoint_mission->event_ctrl, set_waypoint_mission->event_ctrl_extend, set_waypoint_mission->event_ctrl_switch, set_waypoint_mission->event_param, set_waypoint_mission->event_eq_time, set_waypoint_mission->event_eq_space, set_waypoint_mission->event_reserve3, set_waypoint_mission->calc_altit, set_waypoint_mission->calc_region, set_waypoint_mission->calc_reserve1, set_waypoint_mission->calc_reserve2, set_waypoint_mission->calc_reserve3, set_waypoint_mission->calc_reserve4, set_waypoint_mission->calc_wp_done, set_waypoint_mission->spk, set_waypoint_mission->spk_param, set_waypoint_mission->params);
    }else{
        ttalink_set_waypoint_mission_send(dst_addr, src_addr, chan, set_waypoint_mission->cmd, set_waypoint_mission->way_id, set_waypoint_mission->wp_name, set_waypoint_mission->index, set_waypoint_mission->total_num, set_waypoint_mission->latit, set_waypoint_mission->longi, set_waypoint_mission->altit, set_waypoint_mission->speed, set_waypoint_mission->acc_max, set_waypoint_mission->delay_time, set_waypoint_mission->item_mode, set_waypoint_mission->point_r, set_waypoint_mission->yaw_cmd, set_waypoint_mission->yaw_param, set_waypoint_mission->event_device_index, set_waypoint_mission->event_ctrl, set_waypoint_mission->event_ctrl_extend, set_waypoint_mission->event_ctrl_switch, set_waypoint_mission->event_param, set_waypoint_mission->event_eq_time, set_waypoint_mission->event_eq_space, set_waypoint_mission->event_reserve3, set_waypoint_mission->calc_altit, set_waypoint_mission->calc_region, set_waypoint_mission->calc_reserve1, set_waypoint_mission->calc_reserve2, set_waypoint_mission->calc_reserve3, set_waypoint_mission->calc_reserve4, set_waypoint_mission->calc_wp_done, set_waypoint_mission->spk, set_waypoint_mission->spk_param, set_waypoint_mission->params);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION, (const char *)set_waypoint_mission, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC, nocrc);
#endif
}

/**
 * @brief Send a set_waypoint_mission message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_mission_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_mission_t* set_waypoint_mission)
{
    _ttalink_set_waypoint_mission_send_struct(dst_addr, src_addr, chan, set_waypoint_mission, false);
}

/**
 * @brief Send a set_waypoint_mission message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_set_waypoint_mission_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_set_waypoint_mission_t* set_waypoint_mission)
{
    _ttalink_set_waypoint_mission_send_struct(dst_addr, src_addr, chan, set_waypoint_mission, true);
}

#if TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_set_waypoint_mission_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 9, wp_name);
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
    _tta_put_uint8_t(buf, 47, yaw_cmd);
    _tta_put_uint8_t(buf, 56, event_device_index);
    _tta_put_uint16_t(buf, 57, event_ctrl);
    _tta_put_uint8_t(buf, 59, event_ctrl_extend);
    _tta_put_uint8_t(buf, 60, event_ctrl_switch);
    _tta_put_float(buf, 61, event_param);
    _tta_put_float(buf, 65, event_eq_time);
    _tta_put_float(buf, 69, event_eq_space);
    _tta_put_float(buf, 73, event_reserve3);
    _tta_put_float(buf, 77, calc_altit);
    _tta_put_float(buf, 81, calc_region);
    _tta_put_float(buf, 85, calc_reserve1);
    _tta_put_int8_t(buf, 89, calc_reserve2);
    _tta_put_int8_t(buf, 90, calc_reserve3);
    _tta_put_int8_t(buf, 91, calc_reserve4);
    _tta_put_int8_t(buf, 92, calc_wp_done);
    _tta_put_float(buf, 105, spk_param);
    _tta_put_int8_t_array(buf, 1, way_id, 8);
    _tta_put_float_array(buf, 48, yaw_param, 2);
    _tta_put_float_array(buf, 93, spk, 3);
    _tta_put_float_array(buf, 109, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION, buf, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC, nocrc);
#else
    ttalink_set_waypoint_mission_t *packet = (ttalink_set_waypoint_mission_t *)msgbuf;
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
    packet->yaw_cmd = yaw_cmd;
    packet->event_device_index = event_device_index;
    packet->event_ctrl = event_ctrl;
    packet->event_ctrl_extend = event_ctrl_extend;
    packet->event_ctrl_switch = event_ctrl_switch;
    packet->event_param = event_param;
    packet->event_eq_time = event_eq_time;
    packet->event_eq_space = event_eq_space;
    packet->event_reserve3 = event_reserve3;
    packet->calc_altit = calc_altit;
    packet->calc_region = calc_region;
    packet->calc_reserve1 = calc_reserve1;
    packet->calc_reserve2 = calc_reserve2;
    packet->calc_reserve3 = calc_reserve3;
    packet->calc_reserve4 = calc_reserve4;
    packet->calc_wp_done = calc_wp_done;
    packet->spk_param = spk_param;
    tta_array_memcpy(packet->way_id, way_id, sizeof(int8_t)*8);
    tta_array_memcpy(packet->yaw_param, yaw_param, sizeof(float)*2);
    tta_array_memcpy(packet->spk, spk, sizeof(float)*3);
    tta_array_memcpy(packet->params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SET_WAYPOINT_MISSION, (const char *)packet, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_MIN_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_set_waypoint_mission_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params)
{
    _ttalink_set_waypoint_mission_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_set_waypoint_mission_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, const int8_t *way_id, uint8_t wp_name, int32_t index, int32_t total_num, int32_t latit, int32_t longi, float altit, float speed, float acc_max, int32_t delay_time, uint8_t item_mode, float point_r, uint8_t yaw_cmd, const float *yaw_param, uint8_t event_device_index, uint16_t event_ctrl, uint8_t event_ctrl_extend, uint8_t event_ctrl_switch, float event_param, float event_eq_time, float event_eq_space, float event_reserve3, float calc_altit, float calc_region, float calc_reserve1, int8_t calc_reserve2, int8_t calc_reserve3, int8_t calc_reserve4, int8_t calc_wp_done, const float *spk, float spk_param, const float *params)
{
    _ttalink_set_waypoint_mission_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, way_id, wp_name, index, total_num, latit, longi, altit, speed, acc_max, delay_time, item_mode, point_r, yaw_cmd, yaw_param, event_device_index, event_ctrl, event_ctrl_extend, event_ctrl_switch, event_param, event_eq_time, event_eq_space, event_reserve3, calc_altit, calc_region, calc_reserve1, calc_reserve2, calc_reserve3, calc_reserve4, calc_wp_done, spk, spk_param, params, true);
}
#endif

#endif

// MESSAGE SET_WAYPOINT_MISSION UNPACKING


/**
 * @brief Get field cmd from set_waypoint_mission message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_mission_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field way_id from set_waypoint_mission message
 *
 * @return  这里可以传输航线名称（备用，方便以后扩展文件名方式）.
 */
static inline uint16_t ttalink_set_waypoint_mission_get_way_id(const ttalink_message_t* msg, int8_t *way_id)
{
    return _TTA_RETURN_int8_t_array(msg, way_id, 8,  1);
}

/**
 * @brief Get field wp_name from set_waypoint_mission message
 *
 * @return  .
 */
static inline uint8_t ttalink_set_waypoint_mission_get_wp_name(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field index from set_waypoint_mission message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_mission_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  10);
}

/**
 * @brief Get field total_num from set_waypoint_mission message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_mission_get_total_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  14);
}

/**
 * @brief Get field latit from set_waypoint_mission message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_mission_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  18);
}

/**
 * @brief Get field longi from set_waypoint_mission message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_mission_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  22);
}

/**
 * @brief Get field altit from set_waypoint_mission message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_mission_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Get field speed from set_waypoint_mission message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_mission_get_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  30);
}

/**
 * @brief Get field acc_max from set_waypoint_mission message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_mission_get_acc_max(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  34);
}

/**
 * @brief Get field delay_time from set_waypoint_mission message
 *
 * @return  .
 */
static inline int32_t ttalink_set_waypoint_mission_get_delay_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  38);
}

/**
 * @brief Get field item_mode from set_waypoint_mission message
 *
 * @return  4=eWP_TIME 5=eWP_HOVER 13=eWP_PLAY_ROUTE 14=eWP_PLAY_WAITING 29=eWP_EVENT.
 */
static inline uint8_t ttalink_set_waypoint_mission_get_item_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field point_r from set_waypoint_mission message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_mission_get_point_r(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  43);
}

/**
 * @brief Get field yaw_cmd from set_waypoint_mission message
 *
 * @return  1：特定角度.，20，21
 */
static inline uint8_t ttalink_set_waypoint_mission_get_yaw_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field yaw_param from set_waypoint_mission message
 *
 * @return yaw_param[0]=固定角度 或 yaw_param[0]=兴趣点经度  yaw_param[1]=兴趣点纬度
 */
static inline uint16_t ttalink_set_waypoint_mission_get_yaw_param(const ttalink_message_t* msg, float *yaw_param)
{
    return _TTA_RETURN_float_array(msg, yaw_param, 2,  48);
}

/**
 * @brief Get field event_device_index from set_waypoint_mission message
 *
 * @return  触发设备需要.action ，0、1、2设备
 */
static inline uint8_t ttalink_set_waypoint_mission_get_event_device_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Get field event_ctrl from set_waypoint_mission message
 *
 * @return  action 动作类型 .
 */
static inline uint16_t ttalink_set_waypoint_mission_get_event_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  57);
}

/**
 * @brief Get field event_ctrl_extend from set_waypoint_mission message
 *
 * @return  action 是否等待 0非等待，1 需要等待（之前的实际飞行航点必须是悬停点） 
 */
static inline uint8_t ttalink_set_waypoint_mission_get_event_ctrl_extend(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  59);
}

/**
 * @brief Get field event_ctrl_switch from set_waypoint_mission message
 *
 * @return  action 选择预留.
 */
static inline uint8_t ttalink_set_waypoint_mission_get_event_ctrl_switch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field event_param from set_waypoint_mission message
 *
 * @return  value.
 */
static inline float ttalink_set_waypoint_mission_get_event_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  61);
}

/**
 * @brief Get field event_eq_time from set_waypoint_mission message
 *
 * @return  等时拍照.参数
 */
static inline float ttalink_set_waypoint_mission_get_event_eq_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  65);
}

/**
 * @brief Get field event_eq_space from set_waypoint_mission message
 *
 * @return  等距拍照 参数.
 */
static inline float ttalink_set_waypoint_mission_get_event_eq_space(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  69);
}

/**
 * @brief Get field event_reserve3 from set_waypoint_mission message
 *
 * @return  .
 */
static inline float ttalink_set_waypoint_mission_get_event_reserve3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  73);
}

/**
 * @brief Get field calc_altit from set_waypoint_mission message
 *
 * @return  第一个点：起飞点海拔高度（补偿到飞行高度数据上），当类型为LAND类型是降落点相对起飞点的海拔高度（需要补偿到降落高度生成降落速度）.
 */
static inline float ttalink_set_waypoint_mission_get_calc_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  77);
}

/**
 * @brief Get field calc_region from set_waypoint_mission message
 *
 * @return  第一个点：地块总面积.正常航点：作为航线间距
 */
static inline float ttalink_set_waypoint_mission_get_calc_region(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  81);
}

/**
 * @brief Get field calc_reserve1 from set_waypoint_mission message
 *
 * @return  
 */
static inline float ttalink_set_waypoint_mission_get_calc_reserve1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  85);
}

/**
 * @brief Get field calc_reserve2 from set_waypoint_mission message
 *
 * @return  .
 */
static inline int8_t ttalink_set_waypoint_mission_get_calc_reserve2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  89);
}

/**
 * @brief Get field calc_reserve3 from set_waypoint_mission message
 *
 * @return  .
 */
static inline int8_t ttalink_set_waypoint_mission_get_calc_reserve3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  90);
}

/**
 * @brief Get field calc_reserve4 from set_waypoint_mission message
 *
 * @return  .
 */
static inline int8_t ttalink_set_waypoint_mission_get_calc_reserve4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  91);
}

/**
 * @brief Get field calc_wp_done from set_waypoint_mission message
 *
 * @return  第一个点：循环次数 最后一个点：航线完成动作.
 */
static inline int8_t ttalink_set_waypoint_mission_get_calc_wp_done(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  92);
}

/**
 * @brief Get field spk from set_waypoint_mission message
 *
 * @return  用于编队的队形位置.
 */
static inline uint16_t ttalink_set_waypoint_mission_get_spk(const ttalink_message_t* msg, float *spk)
{
    return _TTA_RETURN_float_array(msg, spk, 3,  93);
}

/**
 * @brief Get field spk_param from set_waypoint_mission message
 *
 * @return  用于编队的队形位置预留参数.
 */
static inline float ttalink_set_waypoint_mission_get_spk_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  105);
}

/**
 * @brief Get field params from set_waypoint_mission message
 *
 * @return  .
 */
static inline uint16_t ttalink_set_waypoint_mission_get_params(const ttalink_message_t* msg, float *params)
{
    return _TTA_RETURN_float_array(msg, params, 4,  109);
}

/**
 * @brief Decode a set_waypoint_mission message into a struct
 *
 * @param msg The message to decode
 * @param set_waypoint_mission C-struct to decode the message contents into
 */
static inline void ttalink_set_waypoint_mission_decode(const ttalink_message_t* msg, ttalink_set_waypoint_mission_t* set_waypoint_mission)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    set_waypoint_mission->cmd = ttalink_set_waypoint_mission_get_cmd(msg);
    ttalink_set_waypoint_mission_get_way_id(msg, set_waypoint_mission->way_id);
    set_waypoint_mission->wp_name = ttalink_set_waypoint_mission_get_wp_name(msg);
    set_waypoint_mission->index = ttalink_set_waypoint_mission_get_index(msg);
    set_waypoint_mission->total_num = ttalink_set_waypoint_mission_get_total_num(msg);
    set_waypoint_mission->latit = ttalink_set_waypoint_mission_get_latit(msg);
    set_waypoint_mission->longi = ttalink_set_waypoint_mission_get_longi(msg);
    set_waypoint_mission->altit = ttalink_set_waypoint_mission_get_altit(msg);
    set_waypoint_mission->speed = ttalink_set_waypoint_mission_get_speed(msg);
    set_waypoint_mission->acc_max = ttalink_set_waypoint_mission_get_acc_max(msg);
    set_waypoint_mission->delay_time = ttalink_set_waypoint_mission_get_delay_time(msg);
    set_waypoint_mission->item_mode = ttalink_set_waypoint_mission_get_item_mode(msg);
    set_waypoint_mission->point_r = ttalink_set_waypoint_mission_get_point_r(msg);
    set_waypoint_mission->yaw_cmd = ttalink_set_waypoint_mission_get_yaw_cmd(msg);
    ttalink_set_waypoint_mission_get_yaw_param(msg, set_waypoint_mission->yaw_param);
    set_waypoint_mission->event_device_index = ttalink_set_waypoint_mission_get_event_device_index(msg);
    set_waypoint_mission->event_ctrl = ttalink_set_waypoint_mission_get_event_ctrl(msg);
    set_waypoint_mission->event_ctrl_extend = ttalink_set_waypoint_mission_get_event_ctrl_extend(msg);
    set_waypoint_mission->event_ctrl_switch = ttalink_set_waypoint_mission_get_event_ctrl_switch(msg);
    set_waypoint_mission->event_param = ttalink_set_waypoint_mission_get_event_param(msg);
    set_waypoint_mission->event_eq_time = ttalink_set_waypoint_mission_get_event_eq_time(msg);
    set_waypoint_mission->event_eq_space = ttalink_set_waypoint_mission_get_event_eq_space(msg);
    set_waypoint_mission->event_reserve3 = ttalink_set_waypoint_mission_get_event_reserve3(msg);
    set_waypoint_mission->calc_altit = ttalink_set_waypoint_mission_get_calc_altit(msg);
    set_waypoint_mission->calc_region = ttalink_set_waypoint_mission_get_calc_region(msg);
    set_waypoint_mission->calc_reserve1 = ttalink_set_waypoint_mission_get_calc_reserve1(msg);
    set_waypoint_mission->calc_reserve2 = ttalink_set_waypoint_mission_get_calc_reserve2(msg);
    set_waypoint_mission->calc_reserve3 = ttalink_set_waypoint_mission_get_calc_reserve3(msg);
    set_waypoint_mission->calc_reserve4 = ttalink_set_waypoint_mission_get_calc_reserve4(msg);
    set_waypoint_mission->calc_wp_done = ttalink_set_waypoint_mission_get_calc_wp_done(msg);
    ttalink_set_waypoint_mission_get_spk(msg, set_waypoint_mission->spk);
    set_waypoint_mission->spk_param = ttalink_set_waypoint_mission_get_spk_param(msg);
    ttalink_set_waypoint_mission_get_params(msg, set_waypoint_mission->params);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN? msg->len : TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN;
        memset(set_waypoint_mission, 0, TTALINK_MSG_ID_SET_WAYPOINT_MISSION_LEN);
    memcpy(set_waypoint_mission, _TTA_PAYLOAD(msg), len);
#endif
}
