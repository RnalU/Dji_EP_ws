#pragma once
// MESSAGE SINGLE_CHANNEL_FUNCTION_CONFIG PACKING

#define TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG 2259

TTAPACKED(
typedef struct __ttalink_single_channel_function_config_t {
 uint8_t chan_type; /*< 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT*/
 uint8_t chan_idx; /*< 通道索引*/
 uint8_t reverse; /*< 反向标志： 0：不反向， 1：反向*/
 uint8_t type; /*< 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）*/
 uint8_t count; /*< 离散档位数量*/
 uint16_t action; /*< 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作*/
 uint16_t action_2; /*< 离散档位2的动作*/
 uint16_t action_3; /*< 离散档位3的动作*/
 uint8_t dev_idx; /*< 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号*/
 uint8_t dev_idx_2; /*< 离散档位2的设备号*/
 uint8_t dev_idx_3; /*< 离散档位3的设备号*/
 uint16_t min_pwm; /*< 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM*/
 uint16_t pwm; /*< 离散档位2的PWM*/
 uint16_t max_pwm; /*< 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM*/
 float min_value; /*< 线性最小值*/
 float max_value; /*< 线性最大值*/
}) ttalink_single_channel_function_config_t;

#define TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN 28
#define TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN 28
#define TTALINK_MSG_ID_2259_LEN 28
#define TTALINK_MSG_ID_2259_MIN_LEN 28

#define TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC 216
#define TTALINK_MSG_ID_2259_CRC 216



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SINGLE_CHANNEL_FUNCTION_CONFIG { \
    2259, \
    "SINGLE_CHANNEL_FUNCTION_CONFIG", \
    16, \
    {  { "chan_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_single_channel_function_config_t, chan_type) }, \
         { "chan_idx", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_single_channel_function_config_t, chan_idx) }, \
         { "reverse", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_single_channel_function_config_t, reverse) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_single_channel_function_config_t, type) }, \
         { "count", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_single_channel_function_config_t, count) }, \
         { "action", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_single_channel_function_config_t, action) }, \
         { "action_2", NULL, TTALINK_TYPE_UINT16_T, 0, 7, offsetof(ttalink_single_channel_function_config_t, action_2) }, \
         { "action_3", NULL, TTALINK_TYPE_UINT16_T, 0, 9, offsetof(ttalink_single_channel_function_config_t, action_3) }, \
         { "dev_idx", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_single_channel_function_config_t, dev_idx) }, \
         { "dev_idx_2", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_single_channel_function_config_t, dev_idx_2) }, \
         { "dev_idx_3", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_single_channel_function_config_t, dev_idx_3) }, \
         { "min_pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 14, offsetof(ttalink_single_channel_function_config_t, min_pwm) }, \
         { "pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 16, offsetof(ttalink_single_channel_function_config_t, pwm) }, \
         { "max_pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 18, offsetof(ttalink_single_channel_function_config_t, max_pwm) }, \
         { "min_value", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_single_channel_function_config_t, min_value) }, \
         { "max_value", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_single_channel_function_config_t, max_value) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SINGLE_CHANNEL_FUNCTION_CONFIG { \
    "SINGLE_CHANNEL_FUNCTION_CONFIG", \
    16, \
    {  { "chan_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_single_channel_function_config_t, chan_type) }, \
         { "chan_idx", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_single_channel_function_config_t, chan_idx) }, \
         { "reverse", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_single_channel_function_config_t, reverse) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_single_channel_function_config_t, type) }, \
         { "count", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_single_channel_function_config_t, count) }, \
         { "action", NULL, TTALINK_TYPE_UINT16_T, 0, 5, offsetof(ttalink_single_channel_function_config_t, action) }, \
         { "action_2", NULL, TTALINK_TYPE_UINT16_T, 0, 7, offsetof(ttalink_single_channel_function_config_t, action_2) }, \
         { "action_3", NULL, TTALINK_TYPE_UINT16_T, 0, 9, offsetof(ttalink_single_channel_function_config_t, action_3) }, \
         { "dev_idx", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_single_channel_function_config_t, dev_idx) }, \
         { "dev_idx_2", NULL, TTALINK_TYPE_UINT8_T, 0, 12, offsetof(ttalink_single_channel_function_config_t, dev_idx_2) }, \
         { "dev_idx_3", NULL, TTALINK_TYPE_UINT8_T, 0, 13, offsetof(ttalink_single_channel_function_config_t, dev_idx_3) }, \
         { "min_pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 14, offsetof(ttalink_single_channel_function_config_t, min_pwm) }, \
         { "pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 16, offsetof(ttalink_single_channel_function_config_t, pwm) }, \
         { "max_pwm", NULL, TTALINK_TYPE_UINT16_T, 0, 18, offsetof(ttalink_single_channel_function_config_t, max_pwm) }, \
         { "min_value", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_single_channel_function_config_t, min_value) }, \
         { "max_value", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_single_channel_function_config_t, max_value) }, \
         } \
}
#endif


static inline uint16_t _ttalink_single_channel_function_config_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN];
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_idx);
    _tta_put_uint8_t(buf, 2, reverse);
    _tta_put_uint8_t(buf, 3, type);
    _tta_put_uint8_t(buf, 4, count);
    _tta_put_uint16_t(buf, 5, action);
    _tta_put_uint16_t(buf, 7, action_2);
    _tta_put_uint16_t(buf, 9, action_3);
    _tta_put_uint8_t(buf, 11, dev_idx);
    _tta_put_uint8_t(buf, 12, dev_idx_2);
    _tta_put_uint8_t(buf, 13, dev_idx_3);
    _tta_put_uint16_t(buf, 14, min_pwm);
    _tta_put_uint16_t(buf, 16, pwm);
    _tta_put_uint16_t(buf, 18, max_pwm);
    _tta_put_float(buf, 20, min_value);
    _tta_put_float(buf, 24, max_value);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN);
#else
    ttalink_single_channel_function_config_t packet;
    packet.chan_type = chan_type;
    packet.chan_idx = chan_idx;
    packet.reverse = reverse;
    packet.type = type;
    packet.count = count;
    packet.action = action;
    packet.action_2 = action_2;
    packet.action_3 = action_3;
    packet.dev_idx = dev_idx;
    packet.dev_idx_2 = dev_idx_2;
    packet.dev_idx_3 = dev_idx_3;
    packet.min_pwm = min_pwm;
    packet.pwm = pwm;
    packet.max_pwm = max_pwm;
    packet.min_value = min_value;
    packet.max_value = max_value;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
}

/**
 * @brief Pack a single_channel_function_config message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @param reverse 反向标志： 0：不反向， 1：反向
 * @param type 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 * @param count 离散档位数量
 * @param action 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 * @param action_2 离散档位2的动作
 * @param action_3 离散档位3的动作
 * @param dev_idx 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 * @param dev_idx_2 离散档位2的设备号
 * @param dev_idx_3 离散档位3的设备号
 * @param min_pwm 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 * @param pwm 离散档位2的PWM
 * @param max_pwm 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 * @param min_value 线性最小值
 * @param max_value 线性最大值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_single_channel_function_config_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value)
{
    return _ttalink_single_channel_function_config_pack(dst_addr, src_addr, msg,  chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/**
 * @brief Pack a single_channel_function_config message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @param reverse 反向标志： 0：不反向， 1：反向
 * @param type 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 * @param count 离散档位数量
 * @param action 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 * @param action_2 离散档位2的动作
 * @param action_3 离散档位3的动作
 * @param dev_idx 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 * @param dev_idx_2 离散档位2的设备号
 * @param dev_idx_3 离散档位3的设备号
 * @param min_pwm 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 * @param pwm 离散档位2的PWM
 * @param max_pwm 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 * @param min_value 线性最小值
 * @param max_value 线性最大值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_single_channel_function_config_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value)
{
    return _ttalink_single_channel_function_config_pack(dst_addr, src_addr, msg,  chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}


static inline uint16_t _ttalink_single_channel_function_config_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t chan_type,uint8_t chan_idx,uint8_t reverse,uint8_t type,uint8_t count,uint16_t action,uint16_t action_2,uint16_t action_3,uint8_t dev_idx,uint8_t dev_idx_2,uint8_t dev_idx_3,uint16_t min_pwm,uint16_t pwm,uint16_t max_pwm,float min_value,float max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN];
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_idx);
    _tta_put_uint8_t(buf, 2, reverse);
    _tta_put_uint8_t(buf, 3, type);
    _tta_put_uint8_t(buf, 4, count);
    _tta_put_uint16_t(buf, 5, action);
    _tta_put_uint16_t(buf, 7, action_2);
    _tta_put_uint16_t(buf, 9, action_3);
    _tta_put_uint8_t(buf, 11, dev_idx);
    _tta_put_uint8_t(buf, 12, dev_idx_2);
    _tta_put_uint8_t(buf, 13, dev_idx_3);
    _tta_put_uint16_t(buf, 14, min_pwm);
    _tta_put_uint16_t(buf, 16, pwm);
    _tta_put_uint16_t(buf, 18, max_pwm);
    _tta_put_float(buf, 20, min_value);
    _tta_put_float(buf, 24, max_value);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN);
#else
    ttalink_single_channel_function_config_t packet;
    packet.chan_type = chan_type;
    packet.chan_idx = chan_idx;
    packet.reverse = reverse;
    packet.type = type;
    packet.count = count;
    packet.action = action;
    packet.action_2 = action_2;
    packet.action_3 = action_3;
    packet.dev_idx = dev_idx;
    packet.dev_idx_2 = dev_idx_2;
    packet.dev_idx_3 = dev_idx_3;
    packet.min_pwm = min_pwm;
    packet.pwm = pwm;
    packet.max_pwm = max_pwm;
    packet.min_value = min_value;
    packet.max_value = max_value;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
}

/**
 * @brief Pack a single_channel_function_config message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @param reverse 反向标志： 0：不反向， 1：反向
 * @param type 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 * @param count 离散档位数量
 * @param action 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 * @param action_2 离散档位2的动作
 * @param action_3 离散档位3的动作
 * @param dev_idx 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 * @param dev_idx_2 离散档位2的设备号
 * @param dev_idx_3 离散档位3的设备号
 * @param min_pwm 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 * @param pwm 离散档位2的PWM
 * @param max_pwm 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 * @param min_value 线性最小值
 * @param max_value 线性最大值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_single_channel_function_config_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t chan_type,uint8_t chan_idx,uint8_t reverse,uint8_t type,uint8_t count,uint16_t action,uint16_t action_2,uint16_t action_3,uint8_t dev_idx,uint8_t dev_idx_2,uint8_t dev_idx_3,uint16_t min_pwm,uint16_t pwm,uint16_t max_pwm,float min_value,float max_value)
{
    return _ttalink_single_channel_function_config_pack_chan(dst_addr, src_addr, chan, msg,  chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/**
 * @brief Pack a single_channel_function_config message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @param reverse 反向标志： 0：不反向， 1：反向
 * @param type 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 * @param count 离散档位数量
 * @param action 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 * @param action_2 离散档位2的动作
 * @param action_3 离散档位3的动作
 * @param dev_idx 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 * @param dev_idx_2 离散档位2的设备号
 * @param dev_idx_3 离散档位3的设备号
 * @param min_pwm 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 * @param pwm 离散档位2的PWM
 * @param max_pwm 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 * @param min_value 线性最小值
 * @param max_value 线性最大值
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_single_channel_function_config_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t chan_type,uint8_t chan_idx,uint8_t reverse,uint8_t type,uint8_t count,uint16_t action,uint16_t action_2,uint16_t action_3,uint8_t dev_idx,uint8_t dev_idx_2,uint8_t dev_idx_3,uint16_t min_pwm,uint16_t pwm,uint16_t max_pwm,float min_value,float max_value)
{
    return _ttalink_single_channel_function_config_pack_chan(dst_addr, src_addr, chan, msg,  chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}


static inline uint16_t _ttalink_single_channel_function_config_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_single_channel_function_config_t* single_channel_function_config, bool nocrc)
{
    if(nocrc){
        return ttalink_single_channel_function_config_pack_nocrc(dst_addr, src_addr, msg, single_channel_function_config->chan_type, single_channel_function_config->chan_idx, single_channel_function_config->reverse, single_channel_function_config->type, single_channel_function_config->count, single_channel_function_config->action, single_channel_function_config->action_2, single_channel_function_config->action_3, single_channel_function_config->dev_idx, single_channel_function_config->dev_idx_2, single_channel_function_config->dev_idx_3, single_channel_function_config->min_pwm, single_channel_function_config->pwm, single_channel_function_config->max_pwm, single_channel_function_config->min_value, single_channel_function_config->max_value);
    }else{
        return ttalink_single_channel_function_config_pack(dst_addr, src_addr, msg, single_channel_function_config->chan_type, single_channel_function_config->chan_idx, single_channel_function_config->reverse, single_channel_function_config->type, single_channel_function_config->count, single_channel_function_config->action, single_channel_function_config->action_2, single_channel_function_config->action_3, single_channel_function_config->dev_idx, single_channel_function_config->dev_idx_2, single_channel_function_config->dev_idx_3, single_channel_function_config->min_pwm, single_channel_function_config->pwm, single_channel_function_config->max_pwm, single_channel_function_config->min_value, single_channel_function_config->max_value);
    }
    
}

/**
 * @brief Encode a single_channel_function_config struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param single_channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_single_channel_function_config_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_single_channel_function_config_t* single_channel_function_config)
{
    return _ttalink_single_channel_function_config_encode(dst_addr, src_addr, msg, single_channel_function_config, false);
}

/**
 * @brief Encode a single_channel_function_config struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param single_channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_single_channel_function_config_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_single_channel_function_config_t* single_channel_function_config)
{
    return _ttalink_single_channel_function_config_encode(dst_addr, src_addr, msg, single_channel_function_config, true);
}


static inline uint16_t _ttalink_single_channel_function_config_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_single_channel_function_config_t* single_channel_function_config, bool nocrc)
{
    if(nocrc){
        return ttalink_single_channel_function_config_pack_chan_nocrc(dst_addr, src_addr, chan, msg, single_channel_function_config->chan_type, single_channel_function_config->chan_idx, single_channel_function_config->reverse, single_channel_function_config->type, single_channel_function_config->count, single_channel_function_config->action, single_channel_function_config->action_2, single_channel_function_config->action_3, single_channel_function_config->dev_idx, single_channel_function_config->dev_idx_2, single_channel_function_config->dev_idx_3, single_channel_function_config->min_pwm, single_channel_function_config->pwm, single_channel_function_config->max_pwm, single_channel_function_config->min_value, single_channel_function_config->max_value);
    }else{
        return ttalink_single_channel_function_config_pack_chan(dst_addr, src_addr, chan, msg, single_channel_function_config->chan_type, single_channel_function_config->chan_idx, single_channel_function_config->reverse, single_channel_function_config->type, single_channel_function_config->count, single_channel_function_config->action, single_channel_function_config->action_2, single_channel_function_config->action_3, single_channel_function_config->dev_idx, single_channel_function_config->dev_idx_2, single_channel_function_config->dev_idx_3, single_channel_function_config->min_pwm, single_channel_function_config->pwm, single_channel_function_config->max_pwm, single_channel_function_config->min_value, single_channel_function_config->max_value);
    }
}

/**
 * @brief Encode a single_channel_function_config struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param single_channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_single_channel_function_config_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_single_channel_function_config_t* single_channel_function_config)
{
    return _ttalink_single_channel_function_config_encode_chan(dst_addr, src_addr, chan, msg, single_channel_function_config, false);
}

/**
 * @brief Encode a single_channel_function_config struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param single_channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_single_channel_function_config_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_single_channel_function_config_t* single_channel_function_config)
{
    return _ttalink_single_channel_function_config_encode_chan(dst_addr, src_addr, chan, msg, single_channel_function_config, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_single_channel_function_config_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN];
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_idx);
    _tta_put_uint8_t(buf, 2, reverse);
    _tta_put_uint8_t(buf, 3, type);
    _tta_put_uint8_t(buf, 4, count);
    _tta_put_uint16_t(buf, 5, action);
    _tta_put_uint16_t(buf, 7, action_2);
    _tta_put_uint16_t(buf, 9, action_3);
    _tta_put_uint8_t(buf, 11, dev_idx);
    _tta_put_uint8_t(buf, 12, dev_idx_2);
    _tta_put_uint8_t(buf, 13, dev_idx_3);
    _tta_put_uint16_t(buf, 14, min_pwm);
    _tta_put_uint16_t(buf, 16, pwm);
    _tta_put_uint16_t(buf, 18, max_pwm);
    _tta_put_float(buf, 20, min_value);
    _tta_put_float(buf, 24, max_value);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG, buf, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#else
    ttalink_single_channel_function_config_t packet;
    packet.chan_type = chan_type;
    packet.chan_idx = chan_idx;
    packet.reverse = reverse;
    packet.type = type;
    packet.count = count;
    packet.action = action;
    packet.action_2 = action_2;
    packet.action_3 = action_3;
    packet.dev_idx = dev_idx;
    packet.dev_idx_2 = dev_idx_2;
    packet.dev_idx_3 = dev_idx_3;
    packet.min_pwm = min_pwm;
    packet.pwm = pwm;
    packet.max_pwm = max_pwm;
    packet.min_value = min_value;
    packet.max_value = max_value;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG, (const char *)&packet, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#endif
}

/**
 * @brief Send a single_channel_function_config message
 * @param chan TTAlink channel to send the message
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @param reverse 反向标志： 0：不反向， 1：反向
 * @param type 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 * @param count 离散档位数量
 * @param action 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 * @param action_2 离散档位2的动作
 * @param action_3 离散档位3的动作
 * @param dev_idx 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 * @param dev_idx_2 离散档位2的设备号
 * @param dev_idx_3 离散档位3的设备号
 * @param min_pwm 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 * @param pwm 离散档位2的PWM
 * @param max_pwm 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 * @param min_value 线性最小值
 * @param max_value 线性最大值
 */
static inline void ttalink_single_channel_function_config_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value)
{
    _ttalink_single_channel_function_config_send(dst_addr, src_addr, chan, chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/**
 * @brief Send a single_channel_function_config message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @param reverse 反向标志： 0：不反向， 1：反向
 * @param type 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 * @param count 离散档位数量
 * @param action 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 * @param action_2 离散档位2的动作
 * @param action_3 离散档位3的动作
 * @param dev_idx 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 * @param dev_idx_2 离散档位2的设备号
 * @param dev_idx_3 离散档位3的设备号
 * @param min_pwm 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 * @param pwm 离散档位2的PWM
 * @param max_pwm 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 * @param min_value 线性最小值
 * @param max_value 线性最大值
 */
static inline void ttalink_single_channel_function_config_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value)
{
    _ttalink_single_channel_function_config_send(dst_addr, src_addr, chan, chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}


static inline void _ttalink_single_channel_function_config_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_single_channel_function_config_t* single_channel_function_config, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_single_channel_function_config_send_nocrc(dst_addr, src_addr, chan, single_channel_function_config->chan_type, single_channel_function_config->chan_idx, single_channel_function_config->reverse, single_channel_function_config->type, single_channel_function_config->count, single_channel_function_config->action, single_channel_function_config->action_2, single_channel_function_config->action_3, single_channel_function_config->dev_idx, single_channel_function_config->dev_idx_2, single_channel_function_config->dev_idx_3, single_channel_function_config->min_pwm, single_channel_function_config->pwm, single_channel_function_config->max_pwm, single_channel_function_config->min_value, single_channel_function_config->max_value);
    }else{
        ttalink_single_channel_function_config_send(dst_addr, src_addr, chan, single_channel_function_config->chan_type, single_channel_function_config->chan_idx, single_channel_function_config->reverse, single_channel_function_config->type, single_channel_function_config->count, single_channel_function_config->action, single_channel_function_config->action_2, single_channel_function_config->action_3, single_channel_function_config->dev_idx, single_channel_function_config->dev_idx_2, single_channel_function_config->dev_idx_3, single_channel_function_config->min_pwm, single_channel_function_config->pwm, single_channel_function_config->max_pwm, single_channel_function_config->min_value, single_channel_function_config->max_value);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG, (const char *)single_channel_function_config, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#endif
}

/**
 * @brief Send a single_channel_function_config message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_single_channel_function_config_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_single_channel_function_config_t* single_channel_function_config)
{
    _ttalink_single_channel_function_config_send_struct(dst_addr, src_addr, chan, single_channel_function_config, false);
}

/**
 * @brief Send a single_channel_function_config message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_single_channel_function_config_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_single_channel_function_config_t* single_channel_function_config)
{
    _ttalink_single_channel_function_config_send_struct(dst_addr, src_addr, chan, single_channel_function_config, true);
}

#if TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_single_channel_function_config_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_idx);
    _tta_put_uint8_t(buf, 2, reverse);
    _tta_put_uint8_t(buf, 3, type);
    _tta_put_uint8_t(buf, 4, count);
    _tta_put_uint16_t(buf, 5, action);
    _tta_put_uint16_t(buf, 7, action_2);
    _tta_put_uint16_t(buf, 9, action_3);
    _tta_put_uint8_t(buf, 11, dev_idx);
    _tta_put_uint8_t(buf, 12, dev_idx_2);
    _tta_put_uint8_t(buf, 13, dev_idx_3);
    _tta_put_uint16_t(buf, 14, min_pwm);
    _tta_put_uint16_t(buf, 16, pwm);
    _tta_put_uint16_t(buf, 18, max_pwm);
    _tta_put_float(buf, 20, min_value);
    _tta_put_float(buf, 24, max_value);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG, buf, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#else
    ttalink_single_channel_function_config_t *packet = (ttalink_single_channel_function_config_t *)msgbuf;
    packet->chan_type = chan_type;
    packet->chan_idx = chan_idx;
    packet->reverse = reverse;
    packet->type = type;
    packet->count = count;
    packet->action = action;
    packet->action_2 = action_2;
    packet->action_3 = action_3;
    packet->dev_idx = dev_idx;
    packet->dev_idx_2 = dev_idx_2;
    packet->dev_idx_3 = dev_idx_3;
    packet->min_pwm = min_pwm;
    packet->pwm = pwm;
    packet->max_pwm = max_pwm;
    packet->min_value = min_value;
    packet->max_value = max_value;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG, (const char *)packet, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_single_channel_function_config_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value)
{
    _ttalink_single_channel_function_config_send_buf(dst_addr, src_addr, msgbuf, chan, chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_single_channel_function_config_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t chan_type, uint8_t chan_idx, uint8_t reverse, uint8_t type, uint8_t count, uint16_t action, uint16_t action_2, uint16_t action_3, uint8_t dev_idx, uint8_t dev_idx_2, uint8_t dev_idx_3, uint16_t min_pwm, uint16_t pwm, uint16_t max_pwm, float min_value, float max_value)
{
    _ttalink_single_channel_function_config_send_buf(dst_addr, src_addr, msgbuf, chan, chan_type, chan_idx, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}
#endif

#endif

// MESSAGE SINGLE_CHANNEL_FUNCTION_CONFIG UNPACKING


/**
 * @brief Get field chan_type from single_channel_function_config message
 *
 * @return 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 */
static inline uint8_t ttalink_single_channel_function_config_get_chan_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field chan_idx from single_channel_function_config message
 *
 * @return 通道索引
 */
static inline uint8_t ttalink_single_channel_function_config_get_chan_idx(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field reverse from single_channel_function_config message
 *
 * @return 反向标志： 0：不反向， 1：反向
 */
static inline uint8_t ttalink_single_channel_function_config_get_reverse(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field type from single_channel_function_config message
 *
 * @return 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 */
static inline uint8_t ttalink_single_channel_function_config_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field count from single_channel_function_config message
 *
 * @return 离散档位数量
 */
static inline uint8_t ttalink_single_channel_function_config_get_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field action from single_channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 */
static inline uint16_t ttalink_single_channel_function_config_get_action(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  5);
}

/**
 * @brief Get field action_2 from single_channel_function_config message
 *
 * @return 离散档位2的动作
 */
static inline uint16_t ttalink_single_channel_function_config_get_action_2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  7);
}

/**
 * @brief Get field action_3 from single_channel_function_config message
 *
 * @return 离散档位3的动作
 */
static inline uint16_t ttalink_single_channel_function_config_get_action_3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  9);
}

/**
 * @brief Get field dev_idx from single_channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 */
static inline uint8_t ttalink_single_channel_function_config_get_dev_idx(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field dev_idx_2 from single_channel_function_config message
 *
 * @return 离散档位2的设备号
 */
static inline uint8_t ttalink_single_channel_function_config_get_dev_idx_2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field dev_idx_3 from single_channel_function_config message
 *
 * @return 离散档位3的设备号
 */
static inline uint8_t ttalink_single_channel_function_config_get_dev_idx_3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field min_pwm from single_channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 */
static inline uint16_t ttalink_single_channel_function_config_get_min_pwm(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field pwm from single_channel_function_config message
 *
 * @return 离散档位2的PWM
 */
static inline uint16_t ttalink_single_channel_function_config_get_pwm(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field max_pwm from single_channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 */
static inline uint16_t ttalink_single_channel_function_config_get_max_pwm(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field min_value from single_channel_function_config message
 *
 * @return 线性最小值
 */
static inline float ttalink_single_channel_function_config_get_min_value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field max_value from single_channel_function_config message
 *
 * @return 线性最大值
 */
static inline float ttalink_single_channel_function_config_get_max_value(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Decode a single_channel_function_config message into a struct
 *
 * @param msg The message to decode
 * @param single_channel_function_config C-struct to decode the message contents into
 */
static inline void ttalink_single_channel_function_config_decode(const ttalink_message_t* msg, ttalink_single_channel_function_config_t* single_channel_function_config)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    single_channel_function_config->chan_type = ttalink_single_channel_function_config_get_chan_type(msg);
    single_channel_function_config->chan_idx = ttalink_single_channel_function_config_get_chan_idx(msg);
    single_channel_function_config->reverse = ttalink_single_channel_function_config_get_reverse(msg);
    single_channel_function_config->type = ttalink_single_channel_function_config_get_type(msg);
    single_channel_function_config->count = ttalink_single_channel_function_config_get_count(msg);
    single_channel_function_config->action = ttalink_single_channel_function_config_get_action(msg);
    single_channel_function_config->action_2 = ttalink_single_channel_function_config_get_action_2(msg);
    single_channel_function_config->action_3 = ttalink_single_channel_function_config_get_action_3(msg);
    single_channel_function_config->dev_idx = ttalink_single_channel_function_config_get_dev_idx(msg);
    single_channel_function_config->dev_idx_2 = ttalink_single_channel_function_config_get_dev_idx_2(msg);
    single_channel_function_config->dev_idx_3 = ttalink_single_channel_function_config_get_dev_idx_3(msg);
    single_channel_function_config->min_pwm = ttalink_single_channel_function_config_get_min_pwm(msg);
    single_channel_function_config->pwm = ttalink_single_channel_function_config_get_pwm(msg);
    single_channel_function_config->max_pwm = ttalink_single_channel_function_config_get_max_pwm(msg);
    single_channel_function_config->min_value = ttalink_single_channel_function_config_get_min_value(msg);
    single_channel_function_config->max_value = ttalink_single_channel_function_config_get_max_value(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN? msg->len : TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN;
        memset(single_channel_function_config, 0, TTALINK_MSG_ID_SINGLE_CHANNEL_FUNCTION_CONFIG_LEN);
    memcpy(single_channel_function_config, _TTA_PAYLOAD(msg), len);
#endif
}
