#pragma once
// MESSAGE IRRIGATE_DATA PACKING

#define TTALINK_MSG_ID_IRRIGATE_DATA 2163

TTAPACKED(
typedef struct __ttalink_irrigate_data_t {
 uint32_t update_time; /*< .*/
 uint8_t type; /*<   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . */
 uint8_t ctrl; /*<   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 */
 float data; /*<   根据ctrl控制的数值 */
 uint8_t state; /*<   0=关闭 1=开启.		//开启状态 */
 float flow_speed; /*<   流量计返回的数据，流速数据. */
 uint8_t liquid; /*<   液位计返回的数据，只有开关量. */
 float spray_area; /*<  有效喷洒面积. */
 float volume_usage; /*<   已使用容量. */
 float param3; /*<   . */
 float param4; /*<   . */
}) ttalink_irrigate_data_t;

#define TTALINK_MSG_ID_IRRIGATE_DATA_LEN 32
#define TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN 32
#define TTALINK_MSG_ID_2163_LEN 32
#define TTALINK_MSG_ID_2163_MIN_LEN 32

#define TTALINK_MSG_ID_IRRIGATE_DATA_CRC 184
#define TTALINK_MSG_ID_2163_CRC 184



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_IRRIGATE_DATA { \
    2163, \
    "IRRIGATE_DATA", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_irrigate_data_t, update_time) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_irrigate_data_t, type) }, \
         { "ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_irrigate_data_t, ctrl) }, \
         { "data", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_irrigate_data_t, data) }, \
         { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_irrigate_data_t, state) }, \
         { "flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 11, offsetof(ttalink_irrigate_data_t, flow_speed) }, \
         { "liquid", NULL, TTALINK_TYPE_UINT8_T, 0, 15, offsetof(ttalink_irrigate_data_t, liquid) }, \
         { "spray_area", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_irrigate_data_t, spray_area) }, \
         { "volume_usage", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_irrigate_data_t, volume_usage) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_irrigate_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_irrigate_data_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_IRRIGATE_DATA { \
    "IRRIGATE_DATA", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_irrigate_data_t, update_time) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_irrigate_data_t, type) }, \
         { "ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_irrigate_data_t, ctrl) }, \
         { "data", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_irrigate_data_t, data) }, \
         { "state", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_irrigate_data_t, state) }, \
         { "flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 11, offsetof(ttalink_irrigate_data_t, flow_speed) }, \
         { "liquid", NULL, TTALINK_TYPE_UINT8_T, 0, 15, offsetof(ttalink_irrigate_data_t, liquid) }, \
         { "spray_area", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_irrigate_data_t, spray_area) }, \
         { "volume_usage", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_irrigate_data_t, volume_usage) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_irrigate_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_irrigate_data_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_irrigate_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_float(buf, 6, data);
    _tta_put_uint8_t(buf, 10, state);
    _tta_put_float(buf, 11, flow_speed);
    _tta_put_uint8_t(buf, 15, liquid);
    _tta_put_float(buf, 16, spray_area);
    _tta_put_float(buf, 20, volume_usage);
    _tta_put_float(buf, 24, param3);
    _tta_put_float(buf, 28, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IRRIGATE_DATA_LEN);
#else
    ttalink_irrigate_data_t packet;
    packet.update_time = update_time;
    packet.type = type;
    packet.ctrl = ctrl;
    packet.data = data;
    packet.state = state;
    packet.flow_speed = flow_speed;
    packet.liquid = liquid;
    packet.spray_area = spray_area;
    packet.volume_usage = volume_usage;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IRRIGATE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IRRIGATE_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a irrigate_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . 
 * @param ctrl   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 
 * @param data   根据ctrl控制的数值 
 * @param state   0=关闭 1=开启.		//开启状态 
 * @param flow_speed   流量计返回的数据，流速数据. 
 * @param liquid   液位计返回的数据，只有开关量. 
 * @param spray_area  有效喷洒面积. 
 * @param volume_usage   已使用容量. 
 * @param param3   . 
 * @param param4   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4)
{
    return _ttalink_irrigate_data_pack(dst_addr, src_addr, msg,  update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, false);
}

/**
 * @brief Pack a irrigate_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . 
 * @param ctrl   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 
 * @param data   根据ctrl控制的数值 
 * @param state   0=关闭 1=开启.		//开启状态 
 * @param flow_speed   流量计返回的数据，流速数据. 
 * @param liquid   液位计返回的数据，只有开关量. 
 * @param spray_area  有效喷洒面积. 
 * @param volume_usage   已使用容量. 
 * @param param3   . 
 * @param param4   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4)
{
    return _ttalink_irrigate_data_pack(dst_addr, src_addr, msg,  update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, true);
}


static inline uint16_t _ttalink_irrigate_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t type,uint8_t ctrl,float data,uint8_t state,float flow_speed,uint8_t liquid,float spray_area,float volume_usage,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_float(buf, 6, data);
    _tta_put_uint8_t(buf, 10, state);
    _tta_put_float(buf, 11, flow_speed);
    _tta_put_uint8_t(buf, 15, liquid);
    _tta_put_float(buf, 16, spray_area);
    _tta_put_float(buf, 20, volume_usage);
    _tta_put_float(buf, 24, param3);
    _tta_put_float(buf, 28, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IRRIGATE_DATA_LEN);
#else
    ttalink_irrigate_data_t packet;
    packet.update_time = update_time;
    packet.type = type;
    packet.ctrl = ctrl;
    packet.data = data;
    packet.state = state;
    packet.flow_speed = flow_speed;
    packet.liquid = liquid;
    packet.spray_area = spray_area;
    packet.volume_usage = volume_usage;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IRRIGATE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IRRIGATE_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a irrigate_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param type   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . 
 * @param ctrl   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 
 * @param data   根据ctrl控制的数值 
 * @param state   0=关闭 1=开启.		//开启状态 
 * @param flow_speed   流量计返回的数据，流速数据. 
 * @param liquid   液位计返回的数据，只有开关量. 
 * @param spray_area  有效喷洒面积. 
 * @param volume_usage   已使用容量. 
 * @param param3   . 
 * @param param4   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t type,uint8_t ctrl,float data,uint8_t state,float flow_speed,uint8_t liquid,float spray_area,float volume_usage,float param3,float param4)
{
    return _ttalink_irrigate_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, false);
}

/**
 * @brief Pack a irrigate_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param type   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . 
 * @param ctrl   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 
 * @param data   根据ctrl控制的数值 
 * @param state   0=关闭 1=开启.		//开启状态 
 * @param flow_speed   流量计返回的数据，流速数据. 
 * @param liquid   液位计返回的数据，只有开关量. 
 * @param spray_area  有效喷洒面积. 
 * @param volume_usage   已使用容量. 
 * @param param3   . 
 * @param param4   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t type,uint8_t ctrl,float data,uint8_t state,float flow_speed,uint8_t liquid,float spray_area,float volume_usage,float param3,float param4)
{
    return _ttalink_irrigate_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, true);
}


static inline uint16_t _ttalink_irrigate_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_data_t* irrigate_data, bool nocrc)
{
    if(nocrc){
        return ttalink_irrigate_data_pack_nocrc(dst_addr, src_addr, msg, irrigate_data->update_time, irrigate_data->type, irrigate_data->ctrl, irrigate_data->data, irrigate_data->state, irrigate_data->flow_speed, irrigate_data->liquid, irrigate_data->spray_area, irrigate_data->volume_usage, irrigate_data->param3, irrigate_data->param4);
    }else{
        return ttalink_irrigate_data_pack(dst_addr, src_addr, msg, irrigate_data->update_time, irrigate_data->type, irrigate_data->ctrl, irrigate_data->data, irrigate_data->state, irrigate_data->flow_speed, irrigate_data->liquid, irrigate_data->spray_area, irrigate_data->volume_usage, irrigate_data->param3, irrigate_data->param4);
    }
    
}

/**
 * @brief Encode a irrigate_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_data_t* irrigate_data)
{
    return _ttalink_irrigate_data_encode(dst_addr, src_addr, msg, irrigate_data, false);
}

/**
 * @brief Encode a irrigate_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_data_t* irrigate_data)
{
    return _ttalink_irrigate_data_encode(dst_addr, src_addr, msg, irrigate_data, true);
}


static inline uint16_t _ttalink_irrigate_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_data_t* irrigate_data, bool nocrc)
{
    if(nocrc){
        return ttalink_irrigate_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, irrigate_data->update_time, irrigate_data->type, irrigate_data->ctrl, irrigate_data->data, irrigate_data->state, irrigate_data->flow_speed, irrigate_data->liquid, irrigate_data->spray_area, irrigate_data->volume_usage, irrigate_data->param3, irrigate_data->param4);
    }else{
        return ttalink_irrigate_data_pack_chan(dst_addr, src_addr, chan, msg, irrigate_data->update_time, irrigate_data->type, irrigate_data->ctrl, irrigate_data->data, irrigate_data->state, irrigate_data->flow_speed, irrigate_data->liquid, irrigate_data->spray_area, irrigate_data->volume_usage, irrigate_data->param3, irrigate_data->param4);
    }
}

/**
 * @brief Encode a irrigate_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_data_t* irrigate_data)
{
    return _ttalink_irrigate_data_encode_chan(dst_addr, src_addr, chan, msg, irrigate_data, false);
}

/**
 * @brief Encode a irrigate_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_data_t* irrigate_data)
{
    return _ttalink_irrigate_data_encode_chan(dst_addr, src_addr, chan, msg, irrigate_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_irrigate_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_float(buf, 6, data);
    _tta_put_uint8_t(buf, 10, state);
    _tta_put_float(buf, 11, flow_speed);
    _tta_put_uint8_t(buf, 15, liquid);
    _tta_put_float(buf, 16, spray_area);
    _tta_put_float(buf, 20, volume_usage);
    _tta_put_float(buf, 24, param3);
    _tta_put_float(buf, 28, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_DATA, buf, TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_CRC, nocrc);
#else
    ttalink_irrigate_data_t packet;
    packet.update_time = update_time;
    packet.type = type;
    packet.ctrl = ctrl;
    packet.data = data;
    packet.state = state;
    packet.flow_speed = flow_speed;
    packet.liquid = liquid;
    packet.spray_area = spray_area;
    packet.volume_usage = volume_usage;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_DATA, (const char *)&packet, TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a irrigate_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . 
 * @param ctrl   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 
 * @param data   根据ctrl控制的数值 
 * @param state   0=关闭 1=开启.		//开启状态 
 * @param flow_speed   流量计返回的数据，流速数据. 
 * @param liquid   液位计返回的数据，只有开关量. 
 * @param spray_area  有效喷洒面积. 
 * @param volume_usage   已使用容量. 
 * @param param3   . 
 * @param param4   . 
 */
static inline void ttalink_irrigate_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4)
{
    _ttalink_irrigate_data_send(dst_addr, src_addr, chan, update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, false);
}

/**
 * @brief Send a irrigate_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . 
 * @param ctrl   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 
 * @param data   根据ctrl控制的数值 
 * @param state   0=关闭 1=开启.		//开启状态 
 * @param flow_speed   流量计返回的数据，流速数据. 
 * @param liquid   液位计返回的数据，只有开关量. 
 * @param spray_area  有效喷洒面积. 
 * @param volume_usage   已使用容量. 
 * @param param3   . 
 * @param param4   . 
 */
static inline void ttalink_irrigate_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4)
{
    _ttalink_irrigate_data_send(dst_addr, src_addr, chan, update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, true);
}


static inline void _ttalink_irrigate_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_data_t* irrigate_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_irrigate_data_send_nocrc(dst_addr, src_addr, chan, irrigate_data->update_time, irrigate_data->type, irrigate_data->ctrl, irrigate_data->data, irrigate_data->state, irrigate_data->flow_speed, irrigate_data->liquid, irrigate_data->spray_area, irrigate_data->volume_usage, irrigate_data->param3, irrigate_data->param4);
    }else{
        ttalink_irrigate_data_send(dst_addr, src_addr, chan, irrigate_data->update_time, irrigate_data->type, irrigate_data->ctrl, irrigate_data->data, irrigate_data->state, irrigate_data->flow_speed, irrigate_data->liquid, irrigate_data->spray_area, irrigate_data->volume_usage, irrigate_data->param3, irrigate_data->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_DATA, (const char *)irrigate_data, TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a irrigate_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_irrigate_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_data_t* irrigate_data)
{
    _ttalink_irrigate_data_send_struct(dst_addr, src_addr, chan, irrigate_data, false);
}

/**
 * @brief Send a irrigate_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_irrigate_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_data_t* irrigate_data)
{
    _ttalink_irrigate_data_send_struct(dst_addr, src_addr, chan, irrigate_data, true);
}

#if TTALINK_MSG_ID_IRRIGATE_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_irrigate_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_float(buf, 6, data);
    _tta_put_uint8_t(buf, 10, state);
    _tta_put_float(buf, 11, flow_speed);
    _tta_put_uint8_t(buf, 15, liquid);
    _tta_put_float(buf, 16, spray_area);
    _tta_put_float(buf, 20, volume_usage);
    _tta_put_float(buf, 24, param3);
    _tta_put_float(buf, 28, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_DATA, buf, TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_CRC, nocrc);
#else
    ttalink_irrigate_data_t *packet = (ttalink_irrigate_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->type = type;
    packet->ctrl = ctrl;
    packet->data = data;
    packet->state = state;
    packet->flow_speed = flow_speed;
    packet->liquid = liquid;
    packet->spray_area = spray_area;
    packet->volume_usage = volume_usage;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_DATA, (const char *)packet, TTALINK_MSG_ID_IRRIGATE_DATA_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_LEN, TTALINK_MSG_ID_IRRIGATE_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_irrigate_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4)
{
    _ttalink_irrigate_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_irrigate_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t type, uint8_t ctrl, float data, uint8_t state, float flow_speed, uint8_t liquid, float spray_area, float volume_usage, float param3, float param4)
{
    _ttalink_irrigate_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, type, ctrl, data, state, flow_speed, liquid, spray_area, volume_usage, param3, param4, true);
}
#endif

#endif

// MESSAGE IRRIGATE_DATA UNPACKING


/**
 * @brief Get field update_time from irrigate_data message
 *
 * @return .
 */
static inline uint32_t ttalink_irrigate_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field type from irrigate_data message
 *
 * @return   0=NULL  1=水泵流量计 2=水泵液位计 3=播撒机  //不需要解析判断 . 
 */
static inline uint8_t ttalink_irrigate_data_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ctrl from irrigate_data message
 *
 * @return   0=NULL  1=PWM  2=流速  3=亩喷量  4=待速//控制方式 
 */
static inline uint8_t ttalink_irrigate_data_get_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field data from irrigate_data message
 *
 * @return   根据ctrl控制的数值 
 */
static inline float ttalink_irrigate_data_get_data(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  6);
}

/**
 * @brief Get field state from irrigate_data message
 *
 * @return   0=关闭 1=开启.		//开启状态 
 */
static inline uint8_t ttalink_irrigate_data_get_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field flow_speed from irrigate_data message
 *
 * @return   流量计返回的数据，流速数据. 
 */
static inline float ttalink_irrigate_data_get_flow_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  11);
}

/**
 * @brief Get field liquid from irrigate_data message
 *
 * @return   液位计返回的数据，只有开关量. 
 */
static inline uint8_t ttalink_irrigate_data_get_liquid(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field spray_area from irrigate_data message
 *
 * @return  有效喷洒面积. 
 */
static inline float ttalink_irrigate_data_get_spray_area(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field volume_usage from irrigate_data message
 *
 * @return   已使用容量. 
 */
static inline float ttalink_irrigate_data_get_volume_usage(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field param3 from irrigate_data message
 *
 * @return   . 
 */
static inline float ttalink_irrigate_data_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field param4 from irrigate_data message
 *
 * @return   . 
 */
static inline float ttalink_irrigate_data_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Decode a irrigate_data message into a struct
 *
 * @param msg The message to decode
 * @param irrigate_data C-struct to decode the message contents into
 */
static inline void ttalink_irrigate_data_decode(const ttalink_message_t* msg, ttalink_irrigate_data_t* irrigate_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    irrigate_data->update_time = ttalink_irrigate_data_get_update_time(msg);
    irrigate_data->type = ttalink_irrigate_data_get_type(msg);
    irrigate_data->ctrl = ttalink_irrigate_data_get_ctrl(msg);
    irrigate_data->data = ttalink_irrigate_data_get_data(msg);
    irrigate_data->state = ttalink_irrigate_data_get_state(msg);
    irrigate_data->flow_speed = ttalink_irrigate_data_get_flow_speed(msg);
    irrigate_data->liquid = ttalink_irrigate_data_get_liquid(msg);
    irrigate_data->spray_area = ttalink_irrigate_data_get_spray_area(msg);
    irrigate_data->volume_usage = ttalink_irrigate_data_get_volume_usage(msg);
    irrigate_data->param3 = ttalink_irrigate_data_get_param3(msg);
    irrigate_data->param4 = ttalink_irrigate_data_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_IRRIGATE_DATA_LEN? msg->len : TTALINK_MSG_ID_IRRIGATE_DATA_LEN;
        memset(irrigate_data, 0, TTALINK_MSG_ID_IRRIGATE_DATA_LEN);
    memcpy(irrigate_data, _TTA_PAYLOAD(msg), len);
#endif
}
