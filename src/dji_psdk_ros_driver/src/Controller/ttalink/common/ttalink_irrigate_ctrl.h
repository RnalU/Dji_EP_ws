#pragma once
// MESSAGE IRRIGATE_CTRL PACKING

#define TTALINK_MSG_ID_IRRIGATE_CTRL 2160

TTAPACKED(
typedef struct __ttalink_irrigate_ctrl_t {
 uint32_t update_time; /*< .*/
 uint8_t type; /*<   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 */
 uint8_t ctrl; /*<   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速*/
 uint16_t pwm; /*<  PWM控制. */
 float flow_speed; /*<   流速. */
 float flow_mu; /*<   亩喷量. */
 float uav_speed; /*<   无人机速度. */
 float width; /*<    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. */
 float calc_width; /*<  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . */
 float param1; /*<   . */
 float param2; /*<   . */
}) ttalink_irrigate_ctrl_t;

#define TTALINK_MSG_ID_IRRIGATE_CTRL_LEN 36
#define TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN 36
#define TTALINK_MSG_ID_2160_LEN 36
#define TTALINK_MSG_ID_2160_MIN_LEN 36

#define TTALINK_MSG_ID_IRRIGATE_CTRL_CRC 67
#define TTALINK_MSG_ID_2160_CRC 67



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_IRRIGATE_CTRL { \
    2160, \
    "IRRIGATE_CTRL", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_irrigate_ctrl_t, update_time) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_irrigate_ctrl_t, type) }, \
         { "ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_irrigate_ctrl_t, ctrl) }, \
         { "pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_irrigate_ctrl_t, pwm) }, \
         { "flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_irrigate_ctrl_t, flow_speed) }, \
         { "flow_mu", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_irrigate_ctrl_t, flow_mu) }, \
         { "uav_speed", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_irrigate_ctrl_t, uav_speed) }, \
         { "width", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_irrigate_ctrl_t, width) }, \
         { "calc_width", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_irrigate_ctrl_t, calc_width) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_irrigate_ctrl_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_irrigate_ctrl_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_IRRIGATE_CTRL { \
    "IRRIGATE_CTRL", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_irrigate_ctrl_t, update_time) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_irrigate_ctrl_t, type) }, \
         { "ctrl", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_irrigate_ctrl_t, ctrl) }, \
         { "pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 6, offsetof(ttalink_irrigate_ctrl_t, pwm) }, \
         { "flow_speed", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_irrigate_ctrl_t, flow_speed) }, \
         { "flow_mu", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_irrigate_ctrl_t, flow_mu) }, \
         { "uav_speed", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_irrigate_ctrl_t, uav_speed) }, \
         { "width", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_irrigate_ctrl_t, width) }, \
         { "calc_width", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_irrigate_ctrl_t, calc_width) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_irrigate_ctrl_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_irrigate_ctrl_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_irrigate_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_CTRL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_uint16_t(buf, 6, pwm);
    _tta_put_float(buf, 8, flow_speed);
    _tta_put_float(buf, 12, flow_mu);
    _tta_put_float(buf, 16, uav_speed);
    _tta_put_float(buf, 20, width);
    _tta_put_float(buf, 24, calc_width);
    _tta_put_float(buf, 28, param1);
    _tta_put_float(buf, 32, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN);
#else
    ttalink_irrigate_ctrl_t packet;
    packet.update_time = update_time;
    packet.type = type;
    packet.ctrl = ctrl;
    packet.pwm = pwm;
    packet.flow_speed = flow_speed;
    packet.flow_mu = flow_mu;
    packet.uav_speed = uav_speed;
    packet.width = width;
    packet.calc_width = calc_width;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IRRIGATE_CTRL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a irrigate_ctrl message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 
 * @param ctrl   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速
 * @param pwm  PWM控制. 
 * @param flow_speed   流速. 
 * @param flow_mu   亩喷量. 
 * @param uav_speed   无人机速度. 
 * @param width    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. 
 * @param calc_width  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2)
{
    return _ttalink_irrigate_ctrl_pack(dst_addr, src_addr, msg,  update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, false);
}

/**
 * @brief Pack a irrigate_ctrl message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 
 * @param ctrl   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速
 * @param pwm  PWM控制. 
 * @param flow_speed   流速. 
 * @param flow_mu   亩喷量. 
 * @param uav_speed   无人机速度. 
 * @param width    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. 
 * @param calc_width  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_ctrl_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2)
{
    return _ttalink_irrigate_ctrl_pack(dst_addr, src_addr, msg,  update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, true);
}


static inline uint16_t _ttalink_irrigate_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t type,uint8_t ctrl,uint16_t pwm,float flow_speed,float flow_mu,float uav_speed,float width,float calc_width,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_CTRL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_uint16_t(buf, 6, pwm);
    _tta_put_float(buf, 8, flow_speed);
    _tta_put_float(buf, 12, flow_mu);
    _tta_put_float(buf, 16, uav_speed);
    _tta_put_float(buf, 20, width);
    _tta_put_float(buf, 24, calc_width);
    _tta_put_float(buf, 28, param1);
    _tta_put_float(buf, 32, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN);
#else
    ttalink_irrigate_ctrl_t packet;
    packet.update_time = update_time;
    packet.type = type;
    packet.ctrl = ctrl;
    packet.pwm = pwm;
    packet.flow_speed = flow_speed;
    packet.flow_mu = flow_mu;
    packet.uav_speed = uav_speed;
    packet.width = width;
    packet.calc_width = calc_width;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IRRIGATE_CTRL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a irrigate_ctrl message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 
 * @param ctrl   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速
 * @param pwm  PWM控制. 
 * @param flow_speed   流速. 
 * @param flow_mu   亩喷量. 
 * @param uav_speed   无人机速度. 
 * @param width    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. 
 * @param calc_width  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t type,uint8_t ctrl,uint16_t pwm,float flow_speed,float flow_mu,float uav_speed,float width,float calc_width,float param1,float param2)
{
    return _ttalink_irrigate_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, false);
}

/**
 * @brief Pack a irrigate_ctrl message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 
 * @param ctrl   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速
 * @param pwm  PWM控制. 
 * @param flow_speed   流速. 
 * @param flow_mu   亩喷量. 
 * @param uav_speed   无人机速度. 
 * @param width    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. 
 * @param calc_width  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_irrigate_ctrl_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t type,uint8_t ctrl,uint16_t pwm,float flow_speed,float flow_mu,float uav_speed,float width,float calc_width,float param1,float param2)
{
    return _ttalink_irrigate_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, true);
}


static inline uint16_t _ttalink_irrigate_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_ctrl_t* irrigate_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_irrigate_ctrl_pack_nocrc(dst_addr, src_addr, msg, irrigate_ctrl->update_time, irrigate_ctrl->type, irrigate_ctrl->ctrl, irrigate_ctrl->pwm, irrigate_ctrl->flow_speed, irrigate_ctrl->flow_mu, irrigate_ctrl->uav_speed, irrigate_ctrl->width, irrigate_ctrl->calc_width, irrigate_ctrl->param1, irrigate_ctrl->param2);
    }else{
        return ttalink_irrigate_ctrl_pack(dst_addr, src_addr, msg, irrigate_ctrl->update_time, irrigate_ctrl->type, irrigate_ctrl->ctrl, irrigate_ctrl->pwm, irrigate_ctrl->flow_speed, irrigate_ctrl->flow_mu, irrigate_ctrl->uav_speed, irrigate_ctrl->width, irrigate_ctrl->calc_width, irrigate_ctrl->param1, irrigate_ctrl->param2);
    }
    
}

/**
 * @brief Encode a irrigate_ctrl struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_ctrl_t* irrigate_ctrl)
{
    return _ttalink_irrigate_ctrl_encode(dst_addr, src_addr, msg, irrigate_ctrl, false);
}

/**
 * @brief Encode a irrigate_ctrl struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_ctrl_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_irrigate_ctrl_t* irrigate_ctrl)
{
    return _ttalink_irrigate_ctrl_encode(dst_addr, src_addr, msg, irrigate_ctrl, true);
}


static inline uint16_t _ttalink_irrigate_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_ctrl_t* irrigate_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_irrigate_ctrl_pack_chan_nocrc(dst_addr, src_addr, chan, msg, irrigate_ctrl->update_time, irrigate_ctrl->type, irrigate_ctrl->ctrl, irrigate_ctrl->pwm, irrigate_ctrl->flow_speed, irrigate_ctrl->flow_mu, irrigate_ctrl->uav_speed, irrigate_ctrl->width, irrigate_ctrl->calc_width, irrigate_ctrl->param1, irrigate_ctrl->param2);
    }else{
        return ttalink_irrigate_ctrl_pack_chan(dst_addr, src_addr, chan, msg, irrigate_ctrl->update_time, irrigate_ctrl->type, irrigate_ctrl->ctrl, irrigate_ctrl->pwm, irrigate_ctrl->flow_speed, irrigate_ctrl->flow_mu, irrigate_ctrl->uav_speed, irrigate_ctrl->width, irrigate_ctrl->calc_width, irrigate_ctrl->param1, irrigate_ctrl->param2);
    }
}

/**
 * @brief Encode a irrigate_ctrl struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_ctrl_t* irrigate_ctrl)
{
    return _ttalink_irrigate_ctrl_encode_chan(dst_addr, src_addr, chan, msg, irrigate_ctrl, false);
}

/**
 * @brief Encode a irrigate_ctrl struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param irrigate_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_irrigate_ctrl_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_irrigate_ctrl_t* irrigate_ctrl)
{
    return _ttalink_irrigate_ctrl_encode_chan(dst_addr, src_addr, chan, msg, irrigate_ctrl, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_irrigate_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IRRIGATE_CTRL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_uint16_t(buf, 6, pwm);
    _tta_put_float(buf, 8, flow_speed);
    _tta_put_float(buf, 12, flow_mu);
    _tta_put_float(buf, 16, uav_speed);
    _tta_put_float(buf, 20, width);
    _tta_put_float(buf, 24, calc_width);
    _tta_put_float(buf, 28, param1);
    _tta_put_float(buf, 32, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_CTRL, buf, TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_CRC, nocrc);
#else
    ttalink_irrigate_ctrl_t packet;
    packet.update_time = update_time;
    packet.type = type;
    packet.ctrl = ctrl;
    packet.pwm = pwm;
    packet.flow_speed = flow_speed;
    packet.flow_mu = flow_mu;
    packet.uav_speed = uav_speed;
    packet.width = width;
    packet.calc_width = calc_width;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_CTRL, (const char *)&packet, TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a irrigate_ctrl message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 
 * @param ctrl   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速
 * @param pwm  PWM控制. 
 * @param flow_speed   流速. 
 * @param flow_mu   亩喷量. 
 * @param uav_speed   无人机速度. 
 * @param width    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. 
 * @param calc_width  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . 
 * @param param1   . 
 * @param param2   . 
 */
static inline void ttalink_irrigate_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2)
{
    _ttalink_irrigate_ctrl_send(dst_addr, src_addr, chan, update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, false);
}

/**
 * @brief Send a irrigate_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param type   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 
 * @param ctrl   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速
 * @param pwm  PWM控制. 
 * @param flow_speed   流速. 
 * @param flow_mu   亩喷量. 
 * @param uav_speed   无人机速度. 
 * @param width    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. 
 * @param calc_width  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . 
 * @param param1   . 
 * @param param2   . 
 */
static inline void ttalink_irrigate_ctrl_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2)
{
    _ttalink_irrigate_ctrl_send(dst_addr, src_addr, chan, update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, true);
}


static inline void _ttalink_irrigate_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_ctrl_t* irrigate_ctrl, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_irrigate_ctrl_send_nocrc(dst_addr, src_addr, chan, irrigate_ctrl->update_time, irrigate_ctrl->type, irrigate_ctrl->ctrl, irrigate_ctrl->pwm, irrigate_ctrl->flow_speed, irrigate_ctrl->flow_mu, irrigate_ctrl->uav_speed, irrigate_ctrl->width, irrigate_ctrl->calc_width, irrigate_ctrl->param1, irrigate_ctrl->param2);
    }else{
        ttalink_irrigate_ctrl_send(dst_addr, src_addr, chan, irrigate_ctrl->update_time, irrigate_ctrl->type, irrigate_ctrl->ctrl, irrigate_ctrl->pwm, irrigate_ctrl->flow_speed, irrigate_ctrl->flow_mu, irrigate_ctrl->uav_speed, irrigate_ctrl->width, irrigate_ctrl->calc_width, irrigate_ctrl->param1, irrigate_ctrl->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_CTRL, (const char *)irrigate_ctrl, TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a irrigate_ctrl message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_irrigate_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_ctrl_t* irrigate_ctrl)
{
    _ttalink_irrigate_ctrl_send_struct(dst_addr, src_addr, chan, irrigate_ctrl, false);
}

/**
 * @brief Send a irrigate_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_irrigate_ctrl_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_irrigate_ctrl_t* irrigate_ctrl)
{
    _ttalink_irrigate_ctrl_send_struct(dst_addr, src_addr, chan, irrigate_ctrl, true);
}

#if TTALINK_MSG_ID_IRRIGATE_CTRL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_irrigate_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, type);
    _tta_put_uint8_t(buf, 5, ctrl);
    _tta_put_uint16_t(buf, 6, pwm);
    _tta_put_float(buf, 8, flow_speed);
    _tta_put_float(buf, 12, flow_mu);
    _tta_put_float(buf, 16, uav_speed);
    _tta_put_float(buf, 20, width);
    _tta_put_float(buf, 24, calc_width);
    _tta_put_float(buf, 28, param1);
    _tta_put_float(buf, 32, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_CTRL, buf, TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_CRC, nocrc);
#else
    ttalink_irrigate_ctrl_t *packet = (ttalink_irrigate_ctrl_t *)msgbuf;
    packet->update_time = update_time;
    packet->type = type;
    packet->ctrl = ctrl;
    packet->pwm = pwm;
    packet->flow_speed = flow_speed;
    packet->flow_mu = flow_mu;
    packet->uav_speed = uav_speed;
    packet->width = width;
    packet->calc_width = calc_width;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IRRIGATE_CTRL, (const char *)packet, TTALINK_MSG_ID_IRRIGATE_CTRL_MIN_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN, TTALINK_MSG_ID_IRRIGATE_CTRL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_irrigate_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2)
{
    _ttalink_irrigate_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_irrigate_ctrl_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t type, uint8_t ctrl, uint16_t pwm, float flow_speed, float flow_mu, float uav_speed, float width, float calc_width, float param1, float param2)
{
    _ttalink_irrigate_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, type, ctrl, pwm, flow_speed, flow_mu, uav_speed, width, calc_width, param1, param2, true);
}
#endif

#endif

// MESSAGE IRRIGATE_CTRL UNPACKING


/**
 * @brief Get field update_time from irrigate_ctrl message
 *
 * @return .
 */
static inline uint32_t ttalink_irrigate_ctrl_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field type from irrigate_ctrl message
 *
 * @return   0=NULL  1=水泵-左  2=水泵-右 3=水泵全 4=雾化器 5=播撒机 
 */
static inline uint8_t ttalink_irrigate_ctrl_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ctrl from irrigate_ctrl message
 *
 * @return   0=NULL  1=PWM  	2=流速  3=亩喷量  4=待速
 */
static inline uint8_t ttalink_irrigate_ctrl_get_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field pwm from irrigate_ctrl message
 *
 * @return  PWM控制. 
 */
static inline uint16_t ttalink_irrigate_ctrl_get_pwm(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field flow_speed from irrigate_ctrl message
 *
 * @return   流速. 
 */
static inline float ttalink_irrigate_ctrl_get_flow_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field flow_mu from irrigate_ctrl message
 *
 * @return   亩喷量. 
 */
static inline float ttalink_irrigate_ctrl_get_flow_mu(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field uav_speed from irrigate_ctrl message
 *
 * @return   无人机速度. 
 */
static inline float ttalink_irrigate_ctrl_get_uav_speed(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field width from irrigate_ctrl message
 *
 * @return    流速:spary_width（机架固定，无人机喷幅参数即固定） 播撒（可修改）:spread_width. 
 */
static inline float ttalink_irrigate_ctrl_get_width(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field calc_width from irrigate_ctrl message
 *
 * @return  计算宽度，有航线时候用航线间距，没有航线 固定值=spary_width . 
 */
static inline float ttalink_irrigate_ctrl_get_calc_width(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field param1 from irrigate_ctrl message
 *
 * @return   . 
 */
static inline float ttalink_irrigate_ctrl_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field param2 from irrigate_ctrl message
 *
 * @return   . 
 */
static inline float ttalink_irrigate_ctrl_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Decode a irrigate_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param irrigate_ctrl C-struct to decode the message contents into
 */
static inline void ttalink_irrigate_ctrl_decode(const ttalink_message_t* msg, ttalink_irrigate_ctrl_t* irrigate_ctrl)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    irrigate_ctrl->update_time = ttalink_irrigate_ctrl_get_update_time(msg);
    irrigate_ctrl->type = ttalink_irrigate_ctrl_get_type(msg);
    irrigate_ctrl->ctrl = ttalink_irrigate_ctrl_get_ctrl(msg);
    irrigate_ctrl->pwm = ttalink_irrigate_ctrl_get_pwm(msg);
    irrigate_ctrl->flow_speed = ttalink_irrigate_ctrl_get_flow_speed(msg);
    irrigate_ctrl->flow_mu = ttalink_irrigate_ctrl_get_flow_mu(msg);
    irrigate_ctrl->uav_speed = ttalink_irrigate_ctrl_get_uav_speed(msg);
    irrigate_ctrl->width = ttalink_irrigate_ctrl_get_width(msg);
    irrigate_ctrl->calc_width = ttalink_irrigate_ctrl_get_calc_width(msg);
    irrigate_ctrl->param1 = ttalink_irrigate_ctrl_get_param1(msg);
    irrigate_ctrl->param2 = ttalink_irrigate_ctrl_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_IRRIGATE_CTRL_LEN? msg->len : TTALINK_MSG_ID_IRRIGATE_CTRL_LEN;
        memset(irrigate_ctrl, 0, TTALINK_MSG_ID_IRRIGATE_CTRL_LEN);
    memcpy(irrigate_ctrl, _TTA_PAYLOAD(msg), len);
#endif
}
