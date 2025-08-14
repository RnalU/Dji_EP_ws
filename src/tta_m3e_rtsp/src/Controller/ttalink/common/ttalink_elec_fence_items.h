#pragma once
// MESSAGE ELEC_FENCE_ITEMS PACKING

#define TTALINK_MSG_ID_ELEC_FENCE_ITEMS 2216

TTAPACKED(
typedef struct __ttalink_elec_fence_items_t {
 uint8_t num; /*<  围栏编号.*/
 uint8_t totle_num; /*<  总围栏个数.*/
 uint8_t num_param1; /*<  预留.*/
 uint8_t num_param2; /*<  预留.*/
 uint8_t fence_rw; /*<  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏*/
 uint8_t fence_mode; /*<  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏*/
 uint8_t fence_type; /*<  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.*/
 float core_radius; /*<  圆形围栏半径.*/
 int32_t core_longi; /*<  围栏核心经度.*/
 int32_t core_latit; /*<  围栏核心纬度.*/
 float core_altit; /*<  围栏核心限制高度.*/
 int32_t fence_longi[20]; /*<  多边形围栏经度.*/
 int32_t fence_latit[20]; /*<  多边形围栏纬度.*/
 uint32_t param1; /*<  .*/
 uint32_t param2; /*<  获取所有电子围栏 按位算.*/
 float param3; /*<  .*/
 float param4; /*<  .*/
}) ttalink_elec_fence_items_t;

#define TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN 199
#define TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN 199
#define TTALINK_MSG_ID_2216_LEN 199
#define TTALINK_MSG_ID_2216_MIN_LEN 199

#define TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC 212
#define TTALINK_MSG_ID_2216_CRC 212

#define TTALINK_MSG_ELEC_FENCE_ITEMS_FIELD_FENCE_LONGI_LEN 20
#define TTALINK_MSG_ELEC_FENCE_ITEMS_FIELD_FENCE_LATIT_LEN 20

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ELEC_FENCE_ITEMS { \
    2216, \
    "ELEC_FENCE_ITEMS", \
    17, \
    {  { "num", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_elec_fence_items_t, num) }, \
         { "totle_num", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_elec_fence_items_t, totle_num) }, \
         { "num_param1", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_elec_fence_items_t, num_param1) }, \
         { "num_param2", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_elec_fence_items_t, num_param2) }, \
         { "fence_rw", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_elec_fence_items_t, fence_rw) }, \
         { "fence_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_elec_fence_items_t, fence_mode) }, \
         { "fence_type", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_elec_fence_items_t, fence_type) }, \
         { "core_radius", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_elec_fence_items_t, core_radius) }, \
         { "core_longi", NULL, TTALINK_TYPE_INT32_T, 0, 11, offsetof(ttalink_elec_fence_items_t, core_longi) }, \
         { "core_latit", NULL, TTALINK_TYPE_INT32_T, 0, 15, offsetof(ttalink_elec_fence_items_t, core_latit) }, \
         { "core_altit", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_elec_fence_items_t, core_altit) }, \
         { "fence_longi", NULL, TTALINK_TYPE_INT32_T, 20, 23, offsetof(ttalink_elec_fence_items_t, fence_longi) }, \
         { "fence_latit", NULL, TTALINK_TYPE_INT32_T, 20, 103, offsetof(ttalink_elec_fence_items_t, fence_latit) }, \
         { "param1", NULL, TTALINK_TYPE_UINT32_T, 0, 183, offsetof(ttalink_elec_fence_items_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_UINT32_T, 0, 187, offsetof(ttalink_elec_fence_items_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 191, offsetof(ttalink_elec_fence_items_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 195, offsetof(ttalink_elec_fence_items_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ELEC_FENCE_ITEMS { \
    "ELEC_FENCE_ITEMS", \
    17, \
    {  { "num", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_elec_fence_items_t, num) }, \
         { "totle_num", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_elec_fence_items_t, totle_num) }, \
         { "num_param1", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_elec_fence_items_t, num_param1) }, \
         { "num_param2", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_elec_fence_items_t, num_param2) }, \
         { "fence_rw", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_elec_fence_items_t, fence_rw) }, \
         { "fence_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_elec_fence_items_t, fence_mode) }, \
         { "fence_type", NULL, TTALINK_TYPE_UINT8_T, 0, 6, offsetof(ttalink_elec_fence_items_t, fence_type) }, \
         { "core_radius", NULL, TTALINK_TYPE_FLOAT, 0, 7, offsetof(ttalink_elec_fence_items_t, core_radius) }, \
         { "core_longi", NULL, TTALINK_TYPE_INT32_T, 0, 11, offsetof(ttalink_elec_fence_items_t, core_longi) }, \
         { "core_latit", NULL, TTALINK_TYPE_INT32_T, 0, 15, offsetof(ttalink_elec_fence_items_t, core_latit) }, \
         { "core_altit", NULL, TTALINK_TYPE_FLOAT, 0, 19, offsetof(ttalink_elec_fence_items_t, core_altit) }, \
         { "fence_longi", NULL, TTALINK_TYPE_INT32_T, 20, 23, offsetof(ttalink_elec_fence_items_t, fence_longi) }, \
         { "fence_latit", NULL, TTALINK_TYPE_INT32_T, 20, 103, offsetof(ttalink_elec_fence_items_t, fence_latit) }, \
         { "param1", NULL, TTALINK_TYPE_UINT32_T, 0, 183, offsetof(ttalink_elec_fence_items_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_UINT32_T, 0, 187, offsetof(ttalink_elec_fence_items_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 191, offsetof(ttalink_elec_fence_items_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 195, offsetof(ttalink_elec_fence_items_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_elec_fence_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, num);
    _tta_put_uint8_t(buf, 1, totle_num);
    _tta_put_uint8_t(buf, 2, num_param1);
    _tta_put_uint8_t(buf, 3, num_param2);
    _tta_put_uint8_t(buf, 4, fence_rw);
    _tta_put_uint8_t(buf, 5, fence_mode);
    _tta_put_uint8_t(buf, 6, fence_type);
    _tta_put_float(buf, 7, core_radius);
    _tta_put_int32_t(buf, 11, core_longi);
    _tta_put_int32_t(buf, 15, core_latit);
    _tta_put_float(buf, 19, core_altit);
    _tta_put_uint32_t(buf, 183, param1);
    _tta_put_uint32_t(buf, 187, param2);
    _tta_put_float(buf, 191, param3);
    _tta_put_float(buf, 195, param4);
    _tta_put_int32_t_array(buf, 23, fence_longi, 20);
    _tta_put_int32_t_array(buf, 103, fence_latit, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN);
#else
    ttalink_elec_fence_items_t packet;
    packet.num = num;
    packet.totle_num = totle_num;
    packet.num_param1 = num_param1;
    packet.num_param2 = num_param2;
    packet.fence_rw = fence_rw;
    packet.fence_mode = fence_mode;
    packet.fence_type = fence_type;
    packet.core_radius = core_radius;
    packet.core_longi = core_longi;
    packet.core_latit = core_latit;
    packet.core_altit = core_altit;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.fence_longi, fence_longi, sizeof(int32_t)*20);
    tta_array_memcpy(packet.fence_latit, fence_latit, sizeof(int32_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ELEC_FENCE_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a elec_fence_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param num  围栏编号.
 * @param totle_num  总围栏个数.
 * @param num_param1  预留.
 * @param num_param2  预留.
 * @param fence_rw  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏
 * @param fence_mode  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏
 * @param fence_type  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.
 * @param core_radius  圆形围栏半径.
 * @param core_longi  围栏核心经度.
 * @param core_latit  围栏核心纬度.
 * @param core_altit  围栏核心限制高度.
 * @param fence_longi  多边形围栏经度.
 * @param fence_latit  多边形围栏纬度.
 * @param param1  .
 * @param param2  获取所有电子围栏 按位算.
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_elec_fence_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4)
{
    return _ttalink_elec_fence_items_pack(dst_addr, src_addr, msg,  num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a elec_fence_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param num  围栏编号.
 * @param totle_num  总围栏个数.
 * @param num_param1  预留.
 * @param num_param2  预留.
 * @param fence_rw  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏
 * @param fence_mode  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏
 * @param fence_type  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.
 * @param core_radius  圆形围栏半径.
 * @param core_longi  围栏核心经度.
 * @param core_latit  围栏核心纬度.
 * @param core_altit  围栏核心限制高度.
 * @param fence_longi  多边形围栏经度.
 * @param fence_latit  多边形围栏纬度.
 * @param param1  .
 * @param param2  获取所有电子围栏 按位算.
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_elec_fence_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4)
{
    return _ttalink_elec_fence_items_pack(dst_addr, src_addr, msg,  num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_elec_fence_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t num,uint8_t totle_num,uint8_t num_param1,uint8_t num_param2,uint8_t fence_rw,uint8_t fence_mode,uint8_t fence_type,float core_radius,int32_t core_longi,int32_t core_latit,float core_altit,const int32_t *fence_longi,const int32_t *fence_latit,uint32_t param1,uint32_t param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, num);
    _tta_put_uint8_t(buf, 1, totle_num);
    _tta_put_uint8_t(buf, 2, num_param1);
    _tta_put_uint8_t(buf, 3, num_param2);
    _tta_put_uint8_t(buf, 4, fence_rw);
    _tta_put_uint8_t(buf, 5, fence_mode);
    _tta_put_uint8_t(buf, 6, fence_type);
    _tta_put_float(buf, 7, core_radius);
    _tta_put_int32_t(buf, 11, core_longi);
    _tta_put_int32_t(buf, 15, core_latit);
    _tta_put_float(buf, 19, core_altit);
    _tta_put_uint32_t(buf, 183, param1);
    _tta_put_uint32_t(buf, 187, param2);
    _tta_put_float(buf, 191, param3);
    _tta_put_float(buf, 195, param4);
    _tta_put_int32_t_array(buf, 23, fence_longi, 20);
    _tta_put_int32_t_array(buf, 103, fence_latit, 20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN);
#else
    ttalink_elec_fence_items_t packet;
    packet.num = num;
    packet.totle_num = totle_num;
    packet.num_param1 = num_param1;
    packet.num_param2 = num_param2;
    packet.fence_rw = fence_rw;
    packet.fence_mode = fence_mode;
    packet.fence_type = fence_type;
    packet.core_radius = core_radius;
    packet.core_longi = core_longi;
    packet.core_latit = core_latit;
    packet.core_altit = core_altit;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.fence_longi, fence_longi, sizeof(int32_t)*20);
    tta_array_memcpy(packet.fence_latit, fence_latit, sizeof(int32_t)*20);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ELEC_FENCE_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a elec_fence_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param num  围栏编号.
 * @param totle_num  总围栏个数.
 * @param num_param1  预留.
 * @param num_param2  预留.
 * @param fence_rw  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏
 * @param fence_mode  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏
 * @param fence_type  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.
 * @param core_radius  圆形围栏半径.
 * @param core_longi  围栏核心经度.
 * @param core_latit  围栏核心纬度.
 * @param core_altit  围栏核心限制高度.
 * @param fence_longi  多边形围栏经度.
 * @param fence_latit  多边形围栏纬度.
 * @param param1  .
 * @param param2  获取所有电子围栏 按位算.
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_elec_fence_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t num,uint8_t totle_num,uint8_t num_param1,uint8_t num_param2,uint8_t fence_rw,uint8_t fence_mode,uint8_t fence_type,float core_radius,int32_t core_longi,int32_t core_latit,float core_altit,const int32_t *fence_longi,const int32_t *fence_latit,uint32_t param1,uint32_t param2,float param3,float param4)
{
    return _ttalink_elec_fence_items_pack_chan(dst_addr, src_addr, chan, msg,  num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a elec_fence_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param num  围栏编号.
 * @param totle_num  总围栏个数.
 * @param num_param1  预留.
 * @param num_param2  预留.
 * @param fence_rw  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏
 * @param fence_mode  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏
 * @param fence_type  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.
 * @param core_radius  圆形围栏半径.
 * @param core_longi  围栏核心经度.
 * @param core_latit  围栏核心纬度.
 * @param core_altit  围栏核心限制高度.
 * @param fence_longi  多边形围栏经度.
 * @param fence_latit  多边形围栏纬度.
 * @param param1  .
 * @param param2  获取所有电子围栏 按位算.
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_elec_fence_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t num,uint8_t totle_num,uint8_t num_param1,uint8_t num_param2,uint8_t fence_rw,uint8_t fence_mode,uint8_t fence_type,float core_radius,int32_t core_longi,int32_t core_latit,float core_altit,const int32_t *fence_longi,const int32_t *fence_latit,uint32_t param1,uint32_t param2,float param3,float param4)
{
    return _ttalink_elec_fence_items_pack_chan(dst_addr, src_addr, chan, msg,  num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_elec_fence_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_elec_fence_items_t* elec_fence_items, bool nocrc)
{
    if(nocrc){
        return ttalink_elec_fence_items_pack_nocrc(dst_addr, src_addr, msg, elec_fence_items->num, elec_fence_items->totle_num, elec_fence_items->num_param1, elec_fence_items->num_param2, elec_fence_items->fence_rw, elec_fence_items->fence_mode, elec_fence_items->fence_type, elec_fence_items->core_radius, elec_fence_items->core_longi, elec_fence_items->core_latit, elec_fence_items->core_altit, elec_fence_items->fence_longi, elec_fence_items->fence_latit, elec_fence_items->param1, elec_fence_items->param2, elec_fence_items->param3, elec_fence_items->param4);
    }else{
        return ttalink_elec_fence_items_pack(dst_addr, src_addr, msg, elec_fence_items->num, elec_fence_items->totle_num, elec_fence_items->num_param1, elec_fence_items->num_param2, elec_fence_items->fence_rw, elec_fence_items->fence_mode, elec_fence_items->fence_type, elec_fence_items->core_radius, elec_fence_items->core_longi, elec_fence_items->core_latit, elec_fence_items->core_altit, elec_fence_items->fence_longi, elec_fence_items->fence_latit, elec_fence_items->param1, elec_fence_items->param2, elec_fence_items->param3, elec_fence_items->param4);
    }
    
}

/**
 * @brief Encode a elec_fence_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param elec_fence_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_elec_fence_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_elec_fence_items_t* elec_fence_items)
{
    return _ttalink_elec_fence_items_encode(dst_addr, src_addr, msg, elec_fence_items, false);
}

/**
 * @brief Encode a elec_fence_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param elec_fence_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_elec_fence_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_elec_fence_items_t* elec_fence_items)
{
    return _ttalink_elec_fence_items_encode(dst_addr, src_addr, msg, elec_fence_items, true);
}


static inline uint16_t _ttalink_elec_fence_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_elec_fence_items_t* elec_fence_items, bool nocrc)
{
    if(nocrc){
        return ttalink_elec_fence_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, elec_fence_items->num, elec_fence_items->totle_num, elec_fence_items->num_param1, elec_fence_items->num_param2, elec_fence_items->fence_rw, elec_fence_items->fence_mode, elec_fence_items->fence_type, elec_fence_items->core_radius, elec_fence_items->core_longi, elec_fence_items->core_latit, elec_fence_items->core_altit, elec_fence_items->fence_longi, elec_fence_items->fence_latit, elec_fence_items->param1, elec_fence_items->param2, elec_fence_items->param3, elec_fence_items->param4);
    }else{
        return ttalink_elec_fence_items_pack_chan(dst_addr, src_addr, chan, msg, elec_fence_items->num, elec_fence_items->totle_num, elec_fence_items->num_param1, elec_fence_items->num_param2, elec_fence_items->fence_rw, elec_fence_items->fence_mode, elec_fence_items->fence_type, elec_fence_items->core_radius, elec_fence_items->core_longi, elec_fence_items->core_latit, elec_fence_items->core_altit, elec_fence_items->fence_longi, elec_fence_items->fence_latit, elec_fence_items->param1, elec_fence_items->param2, elec_fence_items->param3, elec_fence_items->param4);
    }
}

/**
 * @brief Encode a elec_fence_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param elec_fence_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_elec_fence_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_elec_fence_items_t* elec_fence_items)
{
    return _ttalink_elec_fence_items_encode_chan(dst_addr, src_addr, chan, msg, elec_fence_items, false);
}

/**
 * @brief Encode a elec_fence_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param elec_fence_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_elec_fence_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_elec_fence_items_t* elec_fence_items)
{
    return _ttalink_elec_fence_items_encode_chan(dst_addr, src_addr, chan, msg, elec_fence_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_elec_fence_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN];
    _tta_put_uint8_t(buf, 0, num);
    _tta_put_uint8_t(buf, 1, totle_num);
    _tta_put_uint8_t(buf, 2, num_param1);
    _tta_put_uint8_t(buf, 3, num_param2);
    _tta_put_uint8_t(buf, 4, fence_rw);
    _tta_put_uint8_t(buf, 5, fence_mode);
    _tta_put_uint8_t(buf, 6, fence_type);
    _tta_put_float(buf, 7, core_radius);
    _tta_put_int32_t(buf, 11, core_longi);
    _tta_put_int32_t(buf, 15, core_latit);
    _tta_put_float(buf, 19, core_altit);
    _tta_put_uint32_t(buf, 183, param1);
    _tta_put_uint32_t(buf, 187, param2);
    _tta_put_float(buf, 191, param3);
    _tta_put_float(buf, 195, param4);
    _tta_put_int32_t_array(buf, 23, fence_longi, 20);
    _tta_put_int32_t_array(buf, 103, fence_latit, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ELEC_FENCE_ITEMS, buf, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC, nocrc);
#else
    ttalink_elec_fence_items_t packet;
    packet.num = num;
    packet.totle_num = totle_num;
    packet.num_param1 = num_param1;
    packet.num_param2 = num_param2;
    packet.fence_rw = fence_rw;
    packet.fence_mode = fence_mode;
    packet.fence_type = fence_type;
    packet.core_radius = core_radius;
    packet.core_longi = core_longi;
    packet.core_latit = core_latit;
    packet.core_altit = core_altit;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    tta_array_memcpy(packet.fence_longi, fence_longi, sizeof(int32_t)*20);
    tta_array_memcpy(packet.fence_latit, fence_latit, sizeof(int32_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ELEC_FENCE_ITEMS, (const char *)&packet, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a elec_fence_items message
 * @param chan TTAlink channel to send the message
 *
 * @param num  围栏编号.
 * @param totle_num  总围栏个数.
 * @param num_param1  预留.
 * @param num_param2  预留.
 * @param fence_rw  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏
 * @param fence_mode  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏
 * @param fence_type  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.
 * @param core_radius  圆形围栏半径.
 * @param core_longi  围栏核心经度.
 * @param core_latit  围栏核心纬度.
 * @param core_altit  围栏核心限制高度.
 * @param fence_longi  多边形围栏经度.
 * @param fence_latit  多边形围栏纬度.
 * @param param1  .
 * @param param2  获取所有电子围栏 按位算.
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_elec_fence_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4)
{
    _ttalink_elec_fence_items_send(dst_addr, src_addr, chan, num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, false);
}

/**
 * @brief Send a elec_fence_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param num  围栏编号.
 * @param totle_num  总围栏个数.
 * @param num_param1  预留.
 * @param num_param2  预留.
 * @param fence_rw  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏
 * @param fence_mode  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏
 * @param fence_type  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.
 * @param core_radius  圆形围栏半径.
 * @param core_longi  围栏核心经度.
 * @param core_latit  围栏核心纬度.
 * @param core_altit  围栏核心限制高度.
 * @param fence_longi  多边形围栏经度.
 * @param fence_latit  多边形围栏纬度.
 * @param param1  .
 * @param param2  获取所有电子围栏 按位算.
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_elec_fence_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4)
{
    _ttalink_elec_fence_items_send(dst_addr, src_addr, chan, num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, true);
}


static inline void _ttalink_elec_fence_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_elec_fence_items_t* elec_fence_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_elec_fence_items_send_nocrc(dst_addr, src_addr, chan, elec_fence_items->num, elec_fence_items->totle_num, elec_fence_items->num_param1, elec_fence_items->num_param2, elec_fence_items->fence_rw, elec_fence_items->fence_mode, elec_fence_items->fence_type, elec_fence_items->core_radius, elec_fence_items->core_longi, elec_fence_items->core_latit, elec_fence_items->core_altit, elec_fence_items->fence_longi, elec_fence_items->fence_latit, elec_fence_items->param1, elec_fence_items->param2, elec_fence_items->param3, elec_fence_items->param4);
    }else{
        ttalink_elec_fence_items_send(dst_addr, src_addr, chan, elec_fence_items->num, elec_fence_items->totle_num, elec_fence_items->num_param1, elec_fence_items->num_param2, elec_fence_items->fence_rw, elec_fence_items->fence_mode, elec_fence_items->fence_type, elec_fence_items->core_radius, elec_fence_items->core_longi, elec_fence_items->core_latit, elec_fence_items->core_altit, elec_fence_items->fence_longi, elec_fence_items->fence_latit, elec_fence_items->param1, elec_fence_items->param2, elec_fence_items->param3, elec_fence_items->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ELEC_FENCE_ITEMS, (const char *)elec_fence_items, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a elec_fence_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_elec_fence_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_elec_fence_items_t* elec_fence_items)
{
    _ttalink_elec_fence_items_send_struct(dst_addr, src_addr, chan, elec_fence_items, false);
}

/**
 * @brief Send a elec_fence_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_elec_fence_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_elec_fence_items_t* elec_fence_items)
{
    _ttalink_elec_fence_items_send_struct(dst_addr, src_addr, chan, elec_fence_items, true);
}

#if TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_elec_fence_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, num);
    _tta_put_uint8_t(buf, 1, totle_num);
    _tta_put_uint8_t(buf, 2, num_param1);
    _tta_put_uint8_t(buf, 3, num_param2);
    _tta_put_uint8_t(buf, 4, fence_rw);
    _tta_put_uint8_t(buf, 5, fence_mode);
    _tta_put_uint8_t(buf, 6, fence_type);
    _tta_put_float(buf, 7, core_radius);
    _tta_put_int32_t(buf, 11, core_longi);
    _tta_put_int32_t(buf, 15, core_latit);
    _tta_put_float(buf, 19, core_altit);
    _tta_put_uint32_t(buf, 183, param1);
    _tta_put_uint32_t(buf, 187, param2);
    _tta_put_float(buf, 191, param3);
    _tta_put_float(buf, 195, param4);
    _tta_put_int32_t_array(buf, 23, fence_longi, 20);
    _tta_put_int32_t_array(buf, 103, fence_latit, 20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ELEC_FENCE_ITEMS, buf, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC, nocrc);
#else
    ttalink_elec_fence_items_t *packet = (ttalink_elec_fence_items_t *)msgbuf;
    packet->num = num;
    packet->totle_num = totle_num;
    packet->num_param1 = num_param1;
    packet->num_param2 = num_param2;
    packet->fence_rw = fence_rw;
    packet->fence_mode = fence_mode;
    packet->fence_type = fence_type;
    packet->core_radius = core_radius;
    packet->core_longi = core_longi;
    packet->core_latit = core_latit;
    packet->core_altit = core_altit;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    tta_array_memcpy(packet->fence_longi, fence_longi, sizeof(int32_t)*20);
    tta_array_memcpy(packet->fence_latit, fence_latit, sizeof(int32_t)*20);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ELEC_FENCE_ITEMS, (const char *)packet, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_MIN_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_elec_fence_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4)
{
    _ttalink_elec_fence_items_send_buf(dst_addr, src_addr, msgbuf, chan, num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_elec_fence_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t num, uint8_t totle_num, uint8_t num_param1, uint8_t num_param2, uint8_t fence_rw, uint8_t fence_mode, uint8_t fence_type, float core_radius, int32_t core_longi, int32_t core_latit, float core_altit, const int32_t *fence_longi, const int32_t *fence_latit, uint32_t param1, uint32_t param2, float param3, float param4)
{
    _ttalink_elec_fence_items_send_buf(dst_addr, src_addr, msgbuf, chan, num, totle_num, num_param1, num_param2, fence_rw, fence_mode, fence_type, core_radius, core_longi, core_latit, core_altit, fence_longi, fence_latit, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE ELEC_FENCE_ITEMS UNPACKING


/**
 * @brief Get field num from elec_fence_items message
 *
 * @return  围栏编号.
 */
static inline uint8_t ttalink_elec_fence_items_get_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field totle_num from elec_fence_items message
 *
 * @return  总围栏个数.
 */
static inline uint8_t ttalink_elec_fence_items_get_totle_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field num_param1 from elec_fence_items message
 *
 * @return  预留.
 */
static inline uint8_t ttalink_elec_fence_items_get_num_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field num_param2 from elec_fence_items message
 *
 * @return  预留.
 */
static inline uint8_t ttalink_elec_fence_items_get_num_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field fence_rw from elec_fence_items message
 *
 * @return  读写标志 0=读取围栏信息 1=写入围栏信息.2=清除单个 3=清除全部 4=读取全部围栏
 */
static inline uint8_t ttalink_elec_fence_items_get_fence_rw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field fence_mode from elec_fence_items message
 *
 * @return  围栏的模式 0=默认限飞区域 1=设置电子围栏 2=临时禁飞区.3=清除设置过的电子围栏 253=设置权限高于禁飞区的电子围栏（临时） 254=设置权限高于禁飞区的电子围栏
 */
static inline uint8_t ttalink_elec_fence_items_get_fence_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field fence_type from elec_fence_items message
 *
 * @return  围栏的类型 0=默认限飞区域 1=圆形电子围栏 2=多边形电子围栏.
 */
static inline uint8_t ttalink_elec_fence_items_get_fence_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field core_radius from elec_fence_items message
 *
 * @return  圆形围栏半径.
 */
static inline float ttalink_elec_fence_items_get_core_radius(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  7);
}

/**
 * @brief Get field core_longi from elec_fence_items message
 *
 * @return  围栏核心经度.
 */
static inline int32_t ttalink_elec_fence_items_get_core_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  11);
}

/**
 * @brief Get field core_latit from elec_fence_items message
 *
 * @return  围栏核心纬度.
 */
static inline int32_t ttalink_elec_fence_items_get_core_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  15);
}

/**
 * @brief Get field core_altit from elec_fence_items message
 *
 * @return  围栏核心限制高度.
 */
static inline float ttalink_elec_fence_items_get_core_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  19);
}

/**
 * @brief Get field fence_longi from elec_fence_items message
 *
 * @return  多边形围栏经度.
 */
static inline uint16_t ttalink_elec_fence_items_get_fence_longi(const ttalink_message_t* msg, int32_t *fence_longi)
{
    return _TTA_RETURN_int32_t_array(msg, fence_longi, 20,  23);
}

/**
 * @brief Get field fence_latit from elec_fence_items message
 *
 * @return  多边形围栏纬度.
 */
static inline uint16_t ttalink_elec_fence_items_get_fence_latit(const ttalink_message_t* msg, int32_t *fence_latit)
{
    return _TTA_RETURN_int32_t_array(msg, fence_latit, 20,  103);
}

/**
 * @brief Get field param1 from elec_fence_items message
 *
 * @return  .
 */
static inline uint32_t ttalink_elec_fence_items_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  183);
}

/**
 * @brief Get field param2 from elec_fence_items message
 *
 * @return  获取所有电子围栏 按位算.
 */
static inline uint32_t ttalink_elec_fence_items_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  187);
}

/**
 * @brief Get field param3 from elec_fence_items message
 *
 * @return  .
 */
static inline float ttalink_elec_fence_items_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  191);
}

/**
 * @brief Get field param4 from elec_fence_items message
 *
 * @return  .
 */
static inline float ttalink_elec_fence_items_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  195);
}

/**
 * @brief Decode a elec_fence_items message into a struct
 *
 * @param msg The message to decode
 * @param elec_fence_items C-struct to decode the message contents into
 */
static inline void ttalink_elec_fence_items_decode(const ttalink_message_t* msg, ttalink_elec_fence_items_t* elec_fence_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    elec_fence_items->num = ttalink_elec_fence_items_get_num(msg);
    elec_fence_items->totle_num = ttalink_elec_fence_items_get_totle_num(msg);
    elec_fence_items->num_param1 = ttalink_elec_fence_items_get_num_param1(msg);
    elec_fence_items->num_param2 = ttalink_elec_fence_items_get_num_param2(msg);
    elec_fence_items->fence_rw = ttalink_elec_fence_items_get_fence_rw(msg);
    elec_fence_items->fence_mode = ttalink_elec_fence_items_get_fence_mode(msg);
    elec_fence_items->fence_type = ttalink_elec_fence_items_get_fence_type(msg);
    elec_fence_items->core_radius = ttalink_elec_fence_items_get_core_radius(msg);
    elec_fence_items->core_longi = ttalink_elec_fence_items_get_core_longi(msg);
    elec_fence_items->core_latit = ttalink_elec_fence_items_get_core_latit(msg);
    elec_fence_items->core_altit = ttalink_elec_fence_items_get_core_altit(msg);
    ttalink_elec_fence_items_get_fence_longi(msg, elec_fence_items->fence_longi);
    ttalink_elec_fence_items_get_fence_latit(msg, elec_fence_items->fence_latit);
    elec_fence_items->param1 = ttalink_elec_fence_items_get_param1(msg);
    elec_fence_items->param2 = ttalink_elec_fence_items_get_param2(msg);
    elec_fence_items->param3 = ttalink_elec_fence_items_get_param3(msg);
    elec_fence_items->param4 = ttalink_elec_fence_items_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN? msg->len : TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN;
        memset(elec_fence_items, 0, TTALINK_MSG_ID_ELEC_FENCE_ITEMS_LEN);
    memcpy(elec_fence_items, _TTA_PAYLOAD(msg), len);
#endif
}
