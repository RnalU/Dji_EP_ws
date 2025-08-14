#pragma once
// MESSAGE POINT_AB_ITEMS PACKING

#define TTALINK_MSG_ID_POINT_AB_ITEMS 2188

TTAPACKED(
typedef struct __ttalink_point_ab_items_t {
 int32_t A_latit; /*<   Latitude (WGS84), in degrees * 1E7*.*/
 int32_t A_longi; /*<   Longitude (WGS84), in degrees * 1E7*.*/
 int32_t B_latit; /*<  Latitude (WGS84), in degrees * 1E7*.*/
 int32_t B_longi; /*<  Longitude (WGS84), in degrees * 1E7*.*/
 int32_t Break_latit; /*<  Latitude (WGS84), in degrees * 1E7*.*/
 int32_t Break_longi; /*<  Longitude (WGS84), in degrees * 1E7*.*/
 float width; /*<  线宽.*/
 float speed; /*<  速度.*/
 float flow; /*<  喷洒量 L/min.*/
 uint8_t status; /*< 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.*/
 uint8_t cmd; /*<  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.*/
 uint8_t process; /*<  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.*/
 float params[4]; /*<  .*/
}) ttalink_point_ab_items_t;

#define TTALINK_MSG_ID_POINT_AB_ITEMS_LEN 55
#define TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN 55
#define TTALINK_MSG_ID_2188_LEN 55
#define TTALINK_MSG_ID_2188_MIN_LEN 55

#define TTALINK_MSG_ID_POINT_AB_ITEMS_CRC 241
#define TTALINK_MSG_ID_2188_CRC 241

#define TTALINK_MSG_POINT_AB_ITEMS_FIELD_PARAMS_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POINT_AB_ITEMS { \
    2188, \
    "POINT_AB_ITEMS", \
    13, \
    {  { "A_latit", NULL, TTALINK_TYPE_INT32_T, 0, 0, offsetof(ttalink_point_ab_items_t, A_latit) }, \
         { "A_longi", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_point_ab_items_t, A_longi) }, \
         { "B_latit", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_point_ab_items_t, B_latit) }, \
         { "B_longi", NULL, TTALINK_TYPE_INT32_T, 0, 12, offsetof(ttalink_point_ab_items_t, B_longi) }, \
         { "Break_latit", NULL, TTALINK_TYPE_INT32_T, 0, 16, offsetof(ttalink_point_ab_items_t, Break_latit) }, \
         { "Break_longi", NULL, TTALINK_TYPE_INT32_T, 0, 20, offsetof(ttalink_point_ab_items_t, Break_longi) }, \
         { "width", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_point_ab_items_t, width) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_point_ab_items_t, speed) }, \
         { "flow", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_point_ab_items_t, flow) }, \
         { "status", NULL, TTALINK_TYPE_UINT8_T, 0, 36, offsetof(ttalink_point_ab_items_t, status) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 37, offsetof(ttalink_point_ab_items_t, cmd) }, \
         { "process", NULL, TTALINK_TYPE_UINT8_T, 0, 38, offsetof(ttalink_point_ab_items_t, process) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 39, offsetof(ttalink_point_ab_items_t, params) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POINT_AB_ITEMS { \
    "POINT_AB_ITEMS", \
    13, \
    {  { "A_latit", NULL, TTALINK_TYPE_INT32_T, 0, 0, offsetof(ttalink_point_ab_items_t, A_latit) }, \
         { "A_longi", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_point_ab_items_t, A_longi) }, \
         { "B_latit", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_point_ab_items_t, B_latit) }, \
         { "B_longi", NULL, TTALINK_TYPE_INT32_T, 0, 12, offsetof(ttalink_point_ab_items_t, B_longi) }, \
         { "Break_latit", NULL, TTALINK_TYPE_INT32_T, 0, 16, offsetof(ttalink_point_ab_items_t, Break_latit) }, \
         { "Break_longi", NULL, TTALINK_TYPE_INT32_T, 0, 20, offsetof(ttalink_point_ab_items_t, Break_longi) }, \
         { "width", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_point_ab_items_t, width) }, \
         { "speed", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_point_ab_items_t, speed) }, \
         { "flow", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_point_ab_items_t, flow) }, \
         { "status", NULL, TTALINK_TYPE_UINT8_T, 0, 36, offsetof(ttalink_point_ab_items_t, status) }, \
         { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 37, offsetof(ttalink_point_ab_items_t, cmd) }, \
         { "process", NULL, TTALINK_TYPE_UINT8_T, 0, 38, offsetof(ttalink_point_ab_items_t, process) }, \
         { "params", NULL, TTALINK_TYPE_FLOAT, 4, 39, offsetof(ttalink_point_ab_items_t, params) }, \
         } \
}
#endif


static inline uint16_t _ttalink_point_ab_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_AB_ITEMS_LEN];
    _tta_put_int32_t(buf, 0, A_latit);
    _tta_put_int32_t(buf, 4, A_longi);
    _tta_put_int32_t(buf, 8, B_latit);
    _tta_put_int32_t(buf, 12, B_longi);
    _tta_put_int32_t(buf, 16, Break_latit);
    _tta_put_int32_t(buf, 20, Break_longi);
    _tta_put_float(buf, 24, width);
    _tta_put_float(buf, 28, speed);
    _tta_put_float(buf, 32, flow);
    _tta_put_uint8_t(buf, 36, status);
    _tta_put_uint8_t(buf, 37, cmd);
    _tta_put_uint8_t(buf, 38, process);
    _tta_put_float_array(buf, 39, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN);
#else
    ttalink_point_ab_items_t packet;
    packet.A_latit = A_latit;
    packet.A_longi = A_longi;
    packet.B_latit = B_latit;
    packet.B_longi = B_longi;
    packet.Break_latit = Break_latit;
    packet.Break_longi = Break_longi;
    packet.width = width;
    packet.speed = speed;
    packet.flow = flow;
    packet.status = status;
    packet.cmd = cmd;
    packet.process = process;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POINT_AB_ITEMS;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a point_ab_items message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param A_latit   Latitude (WGS84), in degrees * 1E7*.
 * @param A_longi   Longitude (WGS84), in degrees * 1E7*.
 * @param B_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param B_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param Break_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param Break_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param width  线宽.
 * @param speed  速度.
 * @param flow  喷洒量 L/min.
 * @param status 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.
 * @param cmd  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.
 * @param process  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_ab_items_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params)
{
    return _ttalink_point_ab_items_pack(dst_addr, src_addr, msg,  A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, false);
}

/**
 * @brief Pack a point_ab_items message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param A_latit   Latitude (WGS84), in degrees * 1E7*.
 * @param A_longi   Longitude (WGS84), in degrees * 1E7*.
 * @param B_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param B_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param Break_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param Break_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param width  线宽.
 * @param speed  速度.
 * @param flow  喷洒量 L/min.
 * @param status 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.
 * @param cmd  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.
 * @param process  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_ab_items_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params)
{
    return _ttalink_point_ab_items_pack(dst_addr, src_addr, msg,  A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, true);
}


static inline uint16_t _ttalink_point_ab_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t A_latit,int32_t A_longi,int32_t B_latit,int32_t B_longi,int32_t Break_latit,int32_t Break_longi,float width,float speed,float flow,uint8_t status,uint8_t cmd,uint8_t process,const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_AB_ITEMS_LEN];
    _tta_put_int32_t(buf, 0, A_latit);
    _tta_put_int32_t(buf, 4, A_longi);
    _tta_put_int32_t(buf, 8, B_latit);
    _tta_put_int32_t(buf, 12, B_longi);
    _tta_put_int32_t(buf, 16, Break_latit);
    _tta_put_int32_t(buf, 20, Break_longi);
    _tta_put_float(buf, 24, width);
    _tta_put_float(buf, 28, speed);
    _tta_put_float(buf, 32, flow);
    _tta_put_uint8_t(buf, 36, status);
    _tta_put_uint8_t(buf, 37, cmd);
    _tta_put_uint8_t(buf, 38, process);
    _tta_put_float_array(buf, 39, params, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN);
#else
    ttalink_point_ab_items_t packet;
    packet.A_latit = A_latit;
    packet.A_longi = A_longi;
    packet.B_latit = B_latit;
    packet.B_longi = B_longi;
    packet.Break_latit = Break_latit;
    packet.Break_longi = Break_longi;
    packet.width = width;
    packet.speed = speed;
    packet.flow = flow;
    packet.status = status;
    packet.cmd = cmd;
    packet.process = process;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POINT_AB_ITEMS;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_CRC, nocrc);
}

/**
 * @brief Pack a point_ab_items message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param A_latit   Latitude (WGS84), in degrees * 1E7*.
 * @param A_longi   Longitude (WGS84), in degrees * 1E7*.
 * @param B_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param B_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param Break_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param Break_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param width  线宽.
 * @param speed  速度.
 * @param flow  喷洒量 L/min.
 * @param status 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.
 * @param cmd  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.
 * @param process  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_ab_items_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t A_latit,int32_t A_longi,int32_t B_latit,int32_t B_longi,int32_t Break_latit,int32_t Break_longi,float width,float speed,float flow,uint8_t status,uint8_t cmd,uint8_t process,const float *params)
{
    return _ttalink_point_ab_items_pack_chan(dst_addr, src_addr, chan, msg,  A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, false);
}

/**
 * @brief Pack a point_ab_items message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param A_latit   Latitude (WGS84), in degrees * 1E7*.
 * @param A_longi   Longitude (WGS84), in degrees * 1E7*.
 * @param B_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param B_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param Break_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param Break_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param width  线宽.
 * @param speed  速度.
 * @param flow  喷洒量 L/min.
 * @param status 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.
 * @param cmd  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.
 * @param process  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.
 * @param params  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_point_ab_items_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int32_t A_latit,int32_t A_longi,int32_t B_latit,int32_t B_longi,int32_t Break_latit,int32_t Break_longi,float width,float speed,float flow,uint8_t status,uint8_t cmd,uint8_t process,const float *params)
{
    return _ttalink_point_ab_items_pack_chan(dst_addr, src_addr, chan, msg,  A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, true);
}


static inline uint16_t _ttalink_point_ab_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_ab_items_t* point_ab_items, bool nocrc)
{
    if(nocrc){
        return ttalink_point_ab_items_pack_nocrc(dst_addr, src_addr, msg, point_ab_items->A_latit, point_ab_items->A_longi, point_ab_items->B_latit, point_ab_items->B_longi, point_ab_items->Break_latit, point_ab_items->Break_longi, point_ab_items->width, point_ab_items->speed, point_ab_items->flow, point_ab_items->status, point_ab_items->cmd, point_ab_items->process, point_ab_items->params);
    }else{
        return ttalink_point_ab_items_pack(dst_addr, src_addr, msg, point_ab_items->A_latit, point_ab_items->A_longi, point_ab_items->B_latit, point_ab_items->B_longi, point_ab_items->Break_latit, point_ab_items->Break_longi, point_ab_items->width, point_ab_items->speed, point_ab_items->flow, point_ab_items->status, point_ab_items->cmd, point_ab_items->process, point_ab_items->params);
    }
    
}

/**
 * @brief Encode a point_ab_items struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param point_ab_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_ab_items_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_ab_items_t* point_ab_items)
{
    return _ttalink_point_ab_items_encode(dst_addr, src_addr, msg, point_ab_items, false);
}

/**
 * @brief Encode a point_ab_items struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param point_ab_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_ab_items_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_point_ab_items_t* point_ab_items)
{
    return _ttalink_point_ab_items_encode(dst_addr, src_addr, msg, point_ab_items, true);
}


static inline uint16_t _ttalink_point_ab_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_ab_items_t* point_ab_items, bool nocrc)
{
    if(nocrc){
        return ttalink_point_ab_items_pack_chan_nocrc(dst_addr, src_addr, chan, msg, point_ab_items->A_latit, point_ab_items->A_longi, point_ab_items->B_latit, point_ab_items->B_longi, point_ab_items->Break_latit, point_ab_items->Break_longi, point_ab_items->width, point_ab_items->speed, point_ab_items->flow, point_ab_items->status, point_ab_items->cmd, point_ab_items->process, point_ab_items->params);
    }else{
        return ttalink_point_ab_items_pack_chan(dst_addr, src_addr, chan, msg, point_ab_items->A_latit, point_ab_items->A_longi, point_ab_items->B_latit, point_ab_items->B_longi, point_ab_items->Break_latit, point_ab_items->Break_longi, point_ab_items->width, point_ab_items->speed, point_ab_items->flow, point_ab_items->status, point_ab_items->cmd, point_ab_items->process, point_ab_items->params);
    }
}

/**
 * @brief Encode a point_ab_items struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param point_ab_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_ab_items_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_ab_items_t* point_ab_items)
{
    return _ttalink_point_ab_items_encode_chan(dst_addr, src_addr, chan, msg, point_ab_items, false);
}

/**
 * @brief Encode a point_ab_items struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param point_ab_items C-struct to read the message contents from
 */
static inline uint16_t ttalink_point_ab_items_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_point_ab_items_t* point_ab_items)
{
    return _ttalink_point_ab_items_encode_chan(dst_addr, src_addr, chan, msg, point_ab_items, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_point_ab_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POINT_AB_ITEMS_LEN];
    _tta_put_int32_t(buf, 0, A_latit);
    _tta_put_int32_t(buf, 4, A_longi);
    _tta_put_int32_t(buf, 8, B_latit);
    _tta_put_int32_t(buf, 12, B_longi);
    _tta_put_int32_t(buf, 16, Break_latit);
    _tta_put_int32_t(buf, 20, Break_longi);
    _tta_put_float(buf, 24, width);
    _tta_put_float(buf, 28, speed);
    _tta_put_float(buf, 32, flow);
    _tta_put_uint8_t(buf, 36, status);
    _tta_put_uint8_t(buf, 37, cmd);
    _tta_put_uint8_t(buf, 38, process);
    _tta_put_float_array(buf, 39, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_AB_ITEMS, buf, TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_CRC, nocrc);
#else
    ttalink_point_ab_items_t packet;
    packet.A_latit = A_latit;
    packet.A_longi = A_longi;
    packet.B_latit = B_latit;
    packet.B_longi = B_longi;
    packet.Break_latit = Break_latit;
    packet.Break_longi = Break_longi;
    packet.width = width;
    packet.speed = speed;
    packet.flow = flow;
    packet.status = status;
    packet.cmd = cmd;
    packet.process = process;
    tta_array_memcpy(packet.params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_AB_ITEMS, (const char *)&packet, TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a point_ab_items message
 * @param chan TTAlink channel to send the message
 *
 * @param A_latit   Latitude (WGS84), in degrees * 1E7*.
 * @param A_longi   Longitude (WGS84), in degrees * 1E7*.
 * @param B_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param B_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param Break_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param Break_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param width  线宽.
 * @param speed  速度.
 * @param flow  喷洒量 L/min.
 * @param status 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.
 * @param cmd  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.
 * @param process  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.
 * @param params  .
 */
static inline void ttalink_point_ab_items_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params)
{
    _ttalink_point_ab_items_send(dst_addr, src_addr, chan, A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, false);
}

/**
 * @brief Send a point_ab_items message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param A_latit   Latitude (WGS84), in degrees * 1E7*.
 * @param A_longi   Longitude (WGS84), in degrees * 1E7*.
 * @param B_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param B_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param Break_latit  Latitude (WGS84), in degrees * 1E7*.
 * @param Break_longi  Longitude (WGS84), in degrees * 1E7*.
 * @param width  线宽.
 * @param speed  速度.
 * @param flow  喷洒量 L/min.
 * @param status 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.
 * @param cmd  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.
 * @param process  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.
 * @param params  .
 */
static inline void ttalink_point_ab_items_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params)
{
    _ttalink_point_ab_items_send(dst_addr, src_addr, chan, A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, true);
}


static inline void _ttalink_point_ab_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_ab_items_t* point_ab_items, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_point_ab_items_send_nocrc(dst_addr, src_addr, chan, point_ab_items->A_latit, point_ab_items->A_longi, point_ab_items->B_latit, point_ab_items->B_longi, point_ab_items->Break_latit, point_ab_items->Break_longi, point_ab_items->width, point_ab_items->speed, point_ab_items->flow, point_ab_items->status, point_ab_items->cmd, point_ab_items->process, point_ab_items->params);
    }else{
        ttalink_point_ab_items_send(dst_addr, src_addr, chan, point_ab_items->A_latit, point_ab_items->A_longi, point_ab_items->B_latit, point_ab_items->B_longi, point_ab_items->Break_latit, point_ab_items->Break_longi, point_ab_items->width, point_ab_items->speed, point_ab_items->flow, point_ab_items->status, point_ab_items->cmd, point_ab_items->process, point_ab_items->params);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_AB_ITEMS, (const char *)point_ab_items, TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_CRC, nocrc);
#endif
}

/**
 * @brief Send a point_ab_items message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_point_ab_items_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_ab_items_t* point_ab_items)
{
    _ttalink_point_ab_items_send_struct(dst_addr, src_addr, chan, point_ab_items, false);
}

/**
 * @brief Send a point_ab_items message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_point_ab_items_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_point_ab_items_t* point_ab_items)
{
    _ttalink_point_ab_items_send_struct(dst_addr, src_addr, chan, point_ab_items, true);
}

#if TTALINK_MSG_ID_POINT_AB_ITEMS_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_point_ab_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int32_t(buf, 0, A_latit);
    _tta_put_int32_t(buf, 4, A_longi);
    _tta_put_int32_t(buf, 8, B_latit);
    _tta_put_int32_t(buf, 12, B_longi);
    _tta_put_int32_t(buf, 16, Break_latit);
    _tta_put_int32_t(buf, 20, Break_longi);
    _tta_put_float(buf, 24, width);
    _tta_put_float(buf, 28, speed);
    _tta_put_float(buf, 32, flow);
    _tta_put_uint8_t(buf, 36, status);
    _tta_put_uint8_t(buf, 37, cmd);
    _tta_put_uint8_t(buf, 38, process);
    _tta_put_float_array(buf, 39, params, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_AB_ITEMS, buf, TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_CRC, nocrc);
#else
    ttalink_point_ab_items_t *packet = (ttalink_point_ab_items_t *)msgbuf;
    packet->A_latit = A_latit;
    packet->A_longi = A_longi;
    packet->B_latit = B_latit;
    packet->B_longi = B_longi;
    packet->Break_latit = Break_latit;
    packet->Break_longi = Break_longi;
    packet->width = width;
    packet->speed = speed;
    packet->flow = flow;
    packet->status = status;
    packet->cmd = cmd;
    packet->process = process;
    tta_array_memcpy(packet->params, params, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POINT_AB_ITEMS, (const char *)packet, TTALINK_MSG_ID_POINT_AB_ITEMS_MIN_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN, TTALINK_MSG_ID_POINT_AB_ITEMS_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_point_ab_items_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params)
{
    _ttalink_point_ab_items_send_buf(dst_addr, src_addr, msgbuf, chan, A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_point_ab_items_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int32_t A_latit, int32_t A_longi, int32_t B_latit, int32_t B_longi, int32_t Break_latit, int32_t Break_longi, float width, float speed, float flow, uint8_t status, uint8_t cmd, uint8_t process, const float *params)
{
    _ttalink_point_ab_items_send_buf(dst_addr, src_addr, msgbuf, chan, A_latit, A_longi, B_latit, B_longi, Break_latit, Break_longi, width, speed, flow, status, cmd, process, params, true);
}
#endif

#endif

// MESSAGE POINT_AB_ITEMS UNPACKING


/**
 * @brief Get field A_latit from point_ab_items message
 *
 * @return   Latitude (WGS84), in degrees * 1E7*.
 */
static inline int32_t ttalink_point_ab_items_get_A_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field A_longi from point_ab_items message
 *
 * @return   Longitude (WGS84), in degrees * 1E7*.
 */
static inline int32_t ttalink_point_ab_items_get_A_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field B_latit from point_ab_items message
 *
 * @return  Latitude (WGS84), in degrees * 1E7*.
 */
static inline int32_t ttalink_point_ab_items_get_B_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field B_longi from point_ab_items message
 *
 * @return  Longitude (WGS84), in degrees * 1E7*.
 */
static inline int32_t ttalink_point_ab_items_get_B_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field Break_latit from point_ab_items message
 *
 * @return  Latitude (WGS84), in degrees * 1E7*.
 */
static inline int32_t ttalink_point_ab_items_get_Break_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field Break_longi from point_ab_items message
 *
 * @return  Longitude (WGS84), in degrees * 1E7*.
 */
static inline int32_t ttalink_point_ab_items_get_Break_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field width from point_ab_items message
 *
 * @return  线宽.
 */
static inline float ttalink_point_ab_items_get_width(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field speed from point_ab_items message
 *
 * @return  速度.
 */
static inline float ttalink_point_ab_items_get_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field flow from point_ab_items message
 *
 * @return  喷洒量 L/min.
 */
static inline float ttalink_point_ab_items_get_flow(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field status from point_ab_items message
 *
 * @return 位域方式：01：记录A点  10：记录B点 100：左侧触发 000 右侧触发.
 */
static inline uint8_t ttalink_point_ab_items_get_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field cmd from point_ab_items message
 *
 * @return  (0:APP触发飞控设置AB点 1：APP一次性设置AB点 2：清除 3：运行 4:飞控更新AB点 5:飞控更新断药点.
 */
static inline uint8_t ttalink_point_ab_items_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field process from point_ab_items message
 *
 * @return  (FM_AB_START_TO_A = 0,FM_AB_START_TO_B = 1,FM_AB_START_TO_BREAK = 2,FM_AB_A_TO_B = 3, FM_AB_B_TO_B = 4,FM_AB_B_TO_A = 5,FM_AB_A_TO_A = 6,FM_AB_RETURN_TO_AB_LINE=7,FM_AB_START_O=8.
 */
static inline uint8_t ttalink_point_ab_items_get_process(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field params from point_ab_items message
 *
 * @return  .
 */
static inline uint16_t ttalink_point_ab_items_get_params(const ttalink_message_t* msg, float *params)
{
    return _TTA_RETURN_float_array(msg, params, 4,  39);
}

/**
 * @brief Decode a point_ab_items message into a struct
 *
 * @param msg The message to decode
 * @param point_ab_items C-struct to decode the message contents into
 */
static inline void ttalink_point_ab_items_decode(const ttalink_message_t* msg, ttalink_point_ab_items_t* point_ab_items)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    point_ab_items->A_latit = ttalink_point_ab_items_get_A_latit(msg);
    point_ab_items->A_longi = ttalink_point_ab_items_get_A_longi(msg);
    point_ab_items->B_latit = ttalink_point_ab_items_get_B_latit(msg);
    point_ab_items->B_longi = ttalink_point_ab_items_get_B_longi(msg);
    point_ab_items->Break_latit = ttalink_point_ab_items_get_Break_latit(msg);
    point_ab_items->Break_longi = ttalink_point_ab_items_get_Break_longi(msg);
    point_ab_items->width = ttalink_point_ab_items_get_width(msg);
    point_ab_items->speed = ttalink_point_ab_items_get_speed(msg);
    point_ab_items->flow = ttalink_point_ab_items_get_flow(msg);
    point_ab_items->status = ttalink_point_ab_items_get_status(msg);
    point_ab_items->cmd = ttalink_point_ab_items_get_cmd(msg);
    point_ab_items->process = ttalink_point_ab_items_get_process(msg);
    ttalink_point_ab_items_get_params(msg, point_ab_items->params);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POINT_AB_ITEMS_LEN? msg->len : TTALINK_MSG_ID_POINT_AB_ITEMS_LEN;
        memset(point_ab_items, 0, TTALINK_MSG_ID_POINT_AB_ITEMS_LEN);
    memcpy(point_ab_items, _TTA_PAYLOAD(msg), len);
#endif
}
