#pragma once
// MESSAGE CHANNEL_FUNCTION_CONFIG PACKING

#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG 2258

TTAPACKED(
typedef struct __ttalink_channel_function_config_t {
 uint8_t chan_type; /*< 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)*/
 uint8_t chan_count; /*< 通道数量*/
 uint8_t reverse[16]; /*< 反向标志： 0：不反向， 1：反向*/
 uint8_t type[16]; /*< 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）*/
 uint8_t count[16]; /*< 离散档位数量*/
 uint16_t action[16]; /*< 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作*/
 uint16_t action_2[16]; /*< 离散档位2的动作*/
 uint16_t action_3[16]; /*< 离散档位3的动作*/
 uint8_t dev_idx[16]; /*< 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号*/
 uint8_t dev_idx_2[16]; /*< 离散档位2的设备号*/
 uint8_t dev_idx_3[16]; /*< 离散档位3的设备号*/
 uint16_t min_pwm[16]; /*< 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM*/
 uint16_t pwm[16]; /*< 离散档位2的PWM*/
 uint16_t max_pwm[16]; /*< 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM*/
 float min_value[16]; /*< 线性最小值*/
 float max_value[16]; /*< 线性最大值*/
}) ttalink_channel_function_config_t;

#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN 418
#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN 418
#define TTALINK_MSG_ID_2258_LEN 418
#define TTALINK_MSG_ID_2258_MIN_LEN 418

#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC 201
#define TTALINK_MSG_ID_2258_CRC 201

#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_REVERSE_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_TYPE_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_COUNT_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_ACTION_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_ACTION_2_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_ACTION_3_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_DEV_IDX_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_DEV_IDX_2_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_DEV_IDX_3_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_MIN_PWM_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_PWM_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_MAX_PWM_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_MIN_VALUE_LEN 16
#define TTALINK_MSG_CHANNEL_FUNCTION_CONFIG_FIELD_MAX_VALUE_LEN 16

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_CHANNEL_FUNCTION_CONFIG { \
    2258, \
    "CHANNEL_FUNCTION_CONFIG", \
    16, \
    {  { "chan_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_channel_function_config_t, chan_type) }, \
         { "chan_count", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_channel_function_config_t, chan_count) }, \
         { "reverse", NULL, TTALINK_TYPE_UINT8_T, 16, 2, offsetof(ttalink_channel_function_config_t, reverse) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 16, 18, offsetof(ttalink_channel_function_config_t, type) }, \
         { "count", NULL, TTALINK_TYPE_UINT8_T, 16, 34, offsetof(ttalink_channel_function_config_t, count) }, \
         { "action", NULL, TTALINK_TYPE_UINT16_T, 16, 50, offsetof(ttalink_channel_function_config_t, action) }, \
         { "action_2", NULL, TTALINK_TYPE_UINT16_T, 16, 82, offsetof(ttalink_channel_function_config_t, action_2) }, \
         { "action_3", NULL, TTALINK_TYPE_UINT16_T, 16, 114, offsetof(ttalink_channel_function_config_t, action_3) }, \
         { "dev_idx", NULL, TTALINK_TYPE_UINT8_T, 16, 146, offsetof(ttalink_channel_function_config_t, dev_idx) }, \
         { "dev_idx_2", NULL, TTALINK_TYPE_UINT8_T, 16, 162, offsetof(ttalink_channel_function_config_t, dev_idx_2) }, \
         { "dev_idx_3", NULL, TTALINK_TYPE_UINT8_T, 16, 178, offsetof(ttalink_channel_function_config_t, dev_idx_3) }, \
         { "min_pwm", NULL, TTALINK_TYPE_UINT16_T, 16, 194, offsetof(ttalink_channel_function_config_t, min_pwm) }, \
         { "pwm", NULL, TTALINK_TYPE_UINT16_T, 16, 226, offsetof(ttalink_channel_function_config_t, pwm) }, \
         { "max_pwm", NULL, TTALINK_TYPE_UINT16_T, 16, 258, offsetof(ttalink_channel_function_config_t, max_pwm) }, \
         { "min_value", NULL, TTALINK_TYPE_FLOAT, 16, 290, offsetof(ttalink_channel_function_config_t, min_value) }, \
         { "max_value", NULL, TTALINK_TYPE_FLOAT, 16, 354, offsetof(ttalink_channel_function_config_t, max_value) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_CHANNEL_FUNCTION_CONFIG { \
    "CHANNEL_FUNCTION_CONFIG", \
    16, \
    {  { "chan_type", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_channel_function_config_t, chan_type) }, \
         { "chan_count", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_channel_function_config_t, chan_count) }, \
         { "reverse", NULL, TTALINK_TYPE_UINT8_T, 16, 2, offsetof(ttalink_channel_function_config_t, reverse) }, \
         { "type", NULL, TTALINK_TYPE_UINT8_T, 16, 18, offsetof(ttalink_channel_function_config_t, type) }, \
         { "count", NULL, TTALINK_TYPE_UINT8_T, 16, 34, offsetof(ttalink_channel_function_config_t, count) }, \
         { "action", NULL, TTALINK_TYPE_UINT16_T, 16, 50, offsetof(ttalink_channel_function_config_t, action) }, \
         { "action_2", NULL, TTALINK_TYPE_UINT16_T, 16, 82, offsetof(ttalink_channel_function_config_t, action_2) }, \
         { "action_3", NULL, TTALINK_TYPE_UINT16_T, 16, 114, offsetof(ttalink_channel_function_config_t, action_3) }, \
         { "dev_idx", NULL, TTALINK_TYPE_UINT8_T, 16, 146, offsetof(ttalink_channel_function_config_t, dev_idx) }, \
         { "dev_idx_2", NULL, TTALINK_TYPE_UINT8_T, 16, 162, offsetof(ttalink_channel_function_config_t, dev_idx_2) }, \
         { "dev_idx_3", NULL, TTALINK_TYPE_UINT8_T, 16, 178, offsetof(ttalink_channel_function_config_t, dev_idx_3) }, \
         { "min_pwm", NULL, TTALINK_TYPE_UINT16_T, 16, 194, offsetof(ttalink_channel_function_config_t, min_pwm) }, \
         { "pwm", NULL, TTALINK_TYPE_UINT16_T, 16, 226, offsetof(ttalink_channel_function_config_t, pwm) }, \
         { "max_pwm", NULL, TTALINK_TYPE_UINT16_T, 16, 258, offsetof(ttalink_channel_function_config_t, max_pwm) }, \
         { "min_value", NULL, TTALINK_TYPE_FLOAT, 16, 290, offsetof(ttalink_channel_function_config_t, min_value) }, \
         { "max_value", NULL, TTALINK_TYPE_FLOAT, 16, 354, offsetof(ttalink_channel_function_config_t, max_value) }, \
         } \
}
#endif


static inline uint16_t _ttalink_channel_function_config_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN];
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_count);
    _tta_put_uint8_t_array(buf, 2, reverse, 16);
    _tta_put_uint8_t_array(buf, 18, type, 16);
    _tta_put_uint8_t_array(buf, 34, count, 16);
    _tta_put_uint16_t_array(buf, 50, action, 16);
    _tta_put_uint16_t_array(buf, 82, action_2, 16);
    _tta_put_uint16_t_array(buf, 114, action_3, 16);
    _tta_put_uint8_t_array(buf, 146, dev_idx, 16);
    _tta_put_uint8_t_array(buf, 162, dev_idx_2, 16);
    _tta_put_uint8_t_array(buf, 178, dev_idx_3, 16);
    _tta_put_uint16_t_array(buf, 194, min_pwm, 16);
    _tta_put_uint16_t_array(buf, 226, pwm, 16);
    _tta_put_uint16_t_array(buf, 258, max_pwm, 16);
    _tta_put_float_array(buf, 290, min_value, 16);
    _tta_put_float_array(buf, 354, max_value, 16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN);
#else
    ttalink_channel_function_config_t packet;
    packet.chan_type = chan_type;
    packet.chan_count = chan_count;
    tta_array_memcpy(packet.reverse, reverse, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.type, type, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.count, count, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.action, action, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.action_2, action_2, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.action_3, action_3, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.dev_idx, dev_idx, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.dev_idx_2, dev_idx_2, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.dev_idx_3, dev_idx_3, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.min_pwm, min_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.pwm, pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.max_pwm, max_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.min_value, min_value, sizeof(float)*16);
    tta_array_memcpy(packet.max_value, max_value, sizeof(float)*16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
}

/**
 * @brief Pack a channel_function_config message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)
 * @param chan_count 通道数量
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
static inline uint16_t ttalink_channel_function_config_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value)
{
    return _ttalink_channel_function_config_pack(dst_addr, src_addr, msg,  chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/**
 * @brief Pack a channel_function_config message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)
 * @param chan_count 通道数量
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
static inline uint16_t ttalink_channel_function_config_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value)
{
    return _ttalink_channel_function_config_pack(dst_addr, src_addr, msg,  chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}


static inline uint16_t _ttalink_channel_function_config_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t chan_type,uint8_t chan_count,const uint8_t *reverse,const uint8_t *type,const uint8_t *count,const uint16_t *action,const uint16_t *action_2,const uint16_t *action_3,const uint8_t *dev_idx,const uint8_t *dev_idx_2,const uint8_t *dev_idx_3,const uint16_t *min_pwm,const uint16_t *pwm,const uint16_t *max_pwm,const float *min_value,const float *max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN];
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_count);
    _tta_put_uint8_t_array(buf, 2, reverse, 16);
    _tta_put_uint8_t_array(buf, 18, type, 16);
    _tta_put_uint8_t_array(buf, 34, count, 16);
    _tta_put_uint16_t_array(buf, 50, action, 16);
    _tta_put_uint16_t_array(buf, 82, action_2, 16);
    _tta_put_uint16_t_array(buf, 114, action_3, 16);
    _tta_put_uint8_t_array(buf, 146, dev_idx, 16);
    _tta_put_uint8_t_array(buf, 162, dev_idx_2, 16);
    _tta_put_uint8_t_array(buf, 178, dev_idx_3, 16);
    _tta_put_uint16_t_array(buf, 194, min_pwm, 16);
    _tta_put_uint16_t_array(buf, 226, pwm, 16);
    _tta_put_uint16_t_array(buf, 258, max_pwm, 16);
    _tta_put_float_array(buf, 290, min_value, 16);
    _tta_put_float_array(buf, 354, max_value, 16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN);
#else
    ttalink_channel_function_config_t packet;
    packet.chan_type = chan_type;
    packet.chan_count = chan_count;
    tta_array_memcpy(packet.reverse, reverse, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.type, type, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.count, count, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.action, action, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.action_2, action_2, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.action_3, action_3, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.dev_idx, dev_idx, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.dev_idx_2, dev_idx_2, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.dev_idx_3, dev_idx_3, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.min_pwm, min_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.pwm, pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.max_pwm, max_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.min_value, min_value, sizeof(float)*16);
    tta_array_memcpy(packet.max_value, max_value, sizeof(float)*16);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
}

/**
 * @brief Pack a channel_function_config message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)
 * @param chan_count 通道数量
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
static inline uint16_t ttalink_channel_function_config_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t chan_type,uint8_t chan_count,const uint8_t *reverse,const uint8_t *type,const uint8_t *count,const uint16_t *action,const uint16_t *action_2,const uint16_t *action_3,const uint8_t *dev_idx,const uint8_t *dev_idx_2,const uint8_t *dev_idx_3,const uint16_t *min_pwm,const uint16_t *pwm,const uint16_t *max_pwm,const float *min_value,const float *max_value)
{
    return _ttalink_channel_function_config_pack_chan(dst_addr, src_addr, chan, msg,  chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/**
 * @brief Pack a channel_function_config message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)
 * @param chan_count 通道数量
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
static inline uint16_t ttalink_channel_function_config_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t chan_type,uint8_t chan_count,const uint8_t *reverse,const uint8_t *type,const uint8_t *count,const uint16_t *action,const uint16_t *action_2,const uint16_t *action_3,const uint8_t *dev_idx,const uint8_t *dev_idx_2,const uint8_t *dev_idx_3,const uint16_t *min_pwm,const uint16_t *pwm,const uint16_t *max_pwm,const float *min_value,const float *max_value)
{
    return _ttalink_channel_function_config_pack_chan(dst_addr, src_addr, chan, msg,  chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}


static inline uint16_t _ttalink_channel_function_config_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_channel_function_config_t* channel_function_config, bool nocrc)
{
    if(nocrc){
        return ttalink_channel_function_config_pack_nocrc(dst_addr, src_addr, msg, channel_function_config->chan_type, channel_function_config->chan_count, channel_function_config->reverse, channel_function_config->type, channel_function_config->count, channel_function_config->action, channel_function_config->action_2, channel_function_config->action_3, channel_function_config->dev_idx, channel_function_config->dev_idx_2, channel_function_config->dev_idx_3, channel_function_config->min_pwm, channel_function_config->pwm, channel_function_config->max_pwm, channel_function_config->min_value, channel_function_config->max_value);
    }else{
        return ttalink_channel_function_config_pack(dst_addr, src_addr, msg, channel_function_config->chan_type, channel_function_config->chan_count, channel_function_config->reverse, channel_function_config->type, channel_function_config->count, channel_function_config->action, channel_function_config->action_2, channel_function_config->action_3, channel_function_config->dev_idx, channel_function_config->dev_idx_2, channel_function_config->dev_idx_3, channel_function_config->min_pwm, channel_function_config->pwm, channel_function_config->max_pwm, channel_function_config->min_value, channel_function_config->max_value);
    }
    
}

/**
 * @brief Encode a channel_function_config struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_channel_function_config_t* channel_function_config)
{
    return _ttalink_channel_function_config_encode(dst_addr, src_addr, msg, channel_function_config, false);
}

/**
 * @brief Encode a channel_function_config struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_channel_function_config_t* channel_function_config)
{
    return _ttalink_channel_function_config_encode(dst_addr, src_addr, msg, channel_function_config, true);
}


static inline uint16_t _ttalink_channel_function_config_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_channel_function_config_t* channel_function_config, bool nocrc)
{
    if(nocrc){
        return ttalink_channel_function_config_pack_chan_nocrc(dst_addr, src_addr, chan, msg, channel_function_config->chan_type, channel_function_config->chan_count, channel_function_config->reverse, channel_function_config->type, channel_function_config->count, channel_function_config->action, channel_function_config->action_2, channel_function_config->action_3, channel_function_config->dev_idx, channel_function_config->dev_idx_2, channel_function_config->dev_idx_3, channel_function_config->min_pwm, channel_function_config->pwm, channel_function_config->max_pwm, channel_function_config->min_value, channel_function_config->max_value);
    }else{
        return ttalink_channel_function_config_pack_chan(dst_addr, src_addr, chan, msg, channel_function_config->chan_type, channel_function_config->chan_count, channel_function_config->reverse, channel_function_config->type, channel_function_config->count, channel_function_config->action, channel_function_config->action_2, channel_function_config->action_3, channel_function_config->dev_idx, channel_function_config->dev_idx_2, channel_function_config->dev_idx_3, channel_function_config->min_pwm, channel_function_config->pwm, channel_function_config->max_pwm, channel_function_config->min_value, channel_function_config->max_value);
    }
}

/**
 * @brief Encode a channel_function_config struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_channel_function_config_t* channel_function_config)
{
    return _ttalink_channel_function_config_encode_chan(dst_addr, src_addr, chan, msg, channel_function_config, false);
}

/**
 * @brief Encode a channel_function_config struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_channel_function_config_t* channel_function_config)
{
    return _ttalink_channel_function_config_encode_chan(dst_addr, src_addr, chan, msg, channel_function_config, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_channel_function_config_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN];
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_count);
    _tta_put_uint8_t_array(buf, 2, reverse, 16);
    _tta_put_uint8_t_array(buf, 18, type, 16);
    _tta_put_uint8_t_array(buf, 34, count, 16);
    _tta_put_uint16_t_array(buf, 50, action, 16);
    _tta_put_uint16_t_array(buf, 82, action_2, 16);
    _tta_put_uint16_t_array(buf, 114, action_3, 16);
    _tta_put_uint8_t_array(buf, 146, dev_idx, 16);
    _tta_put_uint8_t_array(buf, 162, dev_idx_2, 16);
    _tta_put_uint8_t_array(buf, 178, dev_idx_3, 16);
    _tta_put_uint16_t_array(buf, 194, min_pwm, 16);
    _tta_put_uint16_t_array(buf, 226, pwm, 16);
    _tta_put_uint16_t_array(buf, 258, max_pwm, 16);
    _tta_put_float_array(buf, 290, min_value, 16);
    _tta_put_float_array(buf, 354, max_value, 16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG, buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#else
    ttalink_channel_function_config_t packet;
    packet.chan_type = chan_type;
    packet.chan_count = chan_count;
    tta_array_memcpy(packet.reverse, reverse, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.type, type, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.count, count, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.action, action, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.action_2, action_2, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.action_3, action_3, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.dev_idx, dev_idx, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.dev_idx_2, dev_idx_2, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.dev_idx_3, dev_idx_3, sizeof(uint8_t)*16);
    tta_array_memcpy(packet.min_pwm, min_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.pwm, pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.max_pwm, max_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet.min_value, min_value, sizeof(float)*16);
    tta_array_memcpy(packet.max_value, max_value, sizeof(float)*16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG, (const char *)&packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#endif
}

/**
 * @brief Send a channel_function_config message
 * @param chan TTAlink channel to send the message
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)
 * @param chan_count 通道数量
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
static inline void ttalink_channel_function_config_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value)
{
    _ttalink_channel_function_config_send(dst_addr, src_addr, chan, chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/**
 * @brief Send a channel_function_config message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)
 * @param chan_count 通道数量
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
static inline void ttalink_channel_function_config_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value)
{
    _ttalink_channel_function_config_send(dst_addr, src_addr, chan, chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}


static inline void _ttalink_channel_function_config_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_channel_function_config_t* channel_function_config, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_channel_function_config_send_nocrc(dst_addr, src_addr, chan, channel_function_config->chan_type, channel_function_config->chan_count, channel_function_config->reverse, channel_function_config->type, channel_function_config->count, channel_function_config->action, channel_function_config->action_2, channel_function_config->action_3, channel_function_config->dev_idx, channel_function_config->dev_idx_2, channel_function_config->dev_idx_3, channel_function_config->min_pwm, channel_function_config->pwm, channel_function_config->max_pwm, channel_function_config->min_value, channel_function_config->max_value);
    }else{
        ttalink_channel_function_config_send(dst_addr, src_addr, chan, channel_function_config->chan_type, channel_function_config->chan_count, channel_function_config->reverse, channel_function_config->type, channel_function_config->count, channel_function_config->action, channel_function_config->action_2, channel_function_config->action_3, channel_function_config->dev_idx, channel_function_config->dev_idx_2, channel_function_config->dev_idx_3, channel_function_config->min_pwm, channel_function_config->pwm, channel_function_config->max_pwm, channel_function_config->min_value, channel_function_config->max_value);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG, (const char *)channel_function_config, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#endif
}

/**
 * @brief Send a channel_function_config message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_channel_function_config_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_channel_function_config_t* channel_function_config)
{
    _ttalink_channel_function_config_send_struct(dst_addr, src_addr, chan, channel_function_config, false);
}

/**
 * @brief Send a channel_function_config message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_channel_function_config_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_channel_function_config_t* channel_function_config)
{
    _ttalink_channel_function_config_send_struct(dst_addr, src_addr, chan, channel_function_config, true);
}

#if TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_channel_function_config_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, chan_type);
    _tta_put_uint8_t(buf, 1, chan_count);
    _tta_put_uint8_t_array(buf, 2, reverse, 16);
    _tta_put_uint8_t_array(buf, 18, type, 16);
    _tta_put_uint8_t_array(buf, 34, count, 16);
    _tta_put_uint16_t_array(buf, 50, action, 16);
    _tta_put_uint16_t_array(buf, 82, action_2, 16);
    _tta_put_uint16_t_array(buf, 114, action_3, 16);
    _tta_put_uint8_t_array(buf, 146, dev_idx, 16);
    _tta_put_uint8_t_array(buf, 162, dev_idx_2, 16);
    _tta_put_uint8_t_array(buf, 178, dev_idx_3, 16);
    _tta_put_uint16_t_array(buf, 194, min_pwm, 16);
    _tta_put_uint16_t_array(buf, 226, pwm, 16);
    _tta_put_uint16_t_array(buf, 258, max_pwm, 16);
    _tta_put_float_array(buf, 290, min_value, 16);
    _tta_put_float_array(buf, 354, max_value, 16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG, buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#else
    ttalink_channel_function_config_t *packet = (ttalink_channel_function_config_t *)msgbuf;
    packet->chan_type = chan_type;
    packet->chan_count = chan_count;
    tta_array_memcpy(packet->reverse, reverse, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->type, type, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->count, count, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->action, action, sizeof(uint16_t)*16);
    tta_array_memcpy(packet->action_2, action_2, sizeof(uint16_t)*16);
    tta_array_memcpy(packet->action_3, action_3, sizeof(uint16_t)*16);
    tta_array_memcpy(packet->dev_idx, dev_idx, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->dev_idx_2, dev_idx_2, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->dev_idx_3, dev_idx_3, sizeof(uint8_t)*16);
    tta_array_memcpy(packet->min_pwm, min_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet->pwm, pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet->max_pwm, max_pwm, sizeof(uint16_t)*16);
    tta_array_memcpy(packet->min_value, min_value, sizeof(float)*16);
    tta_array_memcpy(packet->max_value, max_value, sizeof(float)*16);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG, (const char *)packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_channel_function_config_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value)
{
    _ttalink_channel_function_config_send_buf(dst_addr, src_addr, msgbuf, chan, chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_channel_function_config_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t chan_type, uint8_t chan_count, const uint8_t *reverse, const uint8_t *type, const uint8_t *count, const uint16_t *action, const uint16_t *action_2, const uint16_t *action_3, const uint8_t *dev_idx, const uint8_t *dev_idx_2, const uint8_t *dev_idx_3, const uint16_t *min_pwm, const uint16_t *pwm, const uint16_t *max_pwm, const float *min_value, const float *max_value)
{
    _ttalink_channel_function_config_send_buf(dst_addr, src_addr, msgbuf, chan, chan_type, chan_count, reverse, type, count, action, action_2, action_3, dev_idx, dev_idx_2, dev_idx_3, min_pwm, pwm, max_pwm, min_value, max_value, true);
}
#endif

#endif

// MESSAGE CHANNEL_FUNCTION_CONFIG UNPACKING


/**
 * @brief Get field chan_type from channel_function_config message
 *
 * @return 通道类型： 0：无； 1：RC_INPUT（FC）； 2：PWM_OUTPUT(TC)
 */
static inline uint8_t ttalink_channel_function_config_get_chan_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field chan_count from channel_function_config message
 *
 * @return 通道数量
 */
static inline uint8_t ttalink_channel_function_config_get_chan_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field reverse from channel_function_config message
 *
 * @return 反向标志： 0：不反向， 1：反向
 */
static inline uint16_t ttalink_channel_function_config_get_reverse(const ttalink_message_t* msg, uint8_t *reverse)
{
    return _TTA_RETURN_uint8_t_array(msg, reverse, 16,  2);
}

/**
 * @brief Get field type from channel_function_config message
 *
 * @return 功能控制类型： 0：无； 1：线性参数（同一个动作，不同值）； 2：离散档位（不同动作）
 */
static inline uint16_t ttalink_channel_function_config_get_type(const ttalink_message_t* msg, uint8_t *type)
{
    return _TTA_RETURN_uint8_t_array(msg, type, 16,  18);
}

/**
 * @brief Get field count from channel_function_config message
 *
 * @return 离散档位数量
 */
static inline uint16_t ttalink_channel_function_config_get_count(const ttalink_message_t* msg, uint8_t *count)
{
    return _TTA_RETURN_uint8_t_array(msg, count, 16,  34);
}

/**
 * @brief Get field action from channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性动作；类型是2离散档位时，表示离散档位1的动作
 */
static inline uint16_t ttalink_channel_function_config_get_action(const ttalink_message_t* msg, uint16_t *action)
{
    return _TTA_RETURN_uint16_t_array(msg, action, 16,  50);
}

/**
 * @brief Get field action_2 from channel_function_config message
 *
 * @return 离散档位2的动作
 */
static inline uint16_t ttalink_channel_function_config_get_action_2(const ttalink_message_t* msg, uint16_t *action_2)
{
    return _TTA_RETURN_uint16_t_array(msg, action_2, 16,  82);
}

/**
 * @brief Get field action_3 from channel_function_config message
 *
 * @return 离散档位3的动作
 */
static inline uint16_t ttalink_channel_function_config_get_action_3(const ttalink_message_t* msg, uint16_t *action_3)
{
    return _TTA_RETURN_uint16_t_array(msg, action_3, 16,  114);
}

/**
 * @brief Get field dev_idx from channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性设备号；类型是2离散档位时，表示离散档位1的设备号
 */
static inline uint16_t ttalink_channel_function_config_get_dev_idx(const ttalink_message_t* msg, uint8_t *dev_idx)
{
    return _TTA_RETURN_uint8_t_array(msg, dev_idx, 16,  146);
}

/**
 * @brief Get field dev_idx_2 from channel_function_config message
 *
 * @return 离散档位2的设备号
 */
static inline uint16_t ttalink_channel_function_config_get_dev_idx_2(const ttalink_message_t* msg, uint8_t *dev_idx_2)
{
    return _TTA_RETURN_uint8_t_array(msg, dev_idx_2, 16,  162);
}

/**
 * @brief Get field dev_idx_3 from channel_function_config message
 *
 * @return 离散档位3的设备号
 */
static inline uint16_t ttalink_channel_function_config_get_dev_idx_3(const ttalink_message_t* msg, uint8_t *dev_idx_3)
{
    return _TTA_RETURN_uint8_t_array(msg, dev_idx_3, 16,  178);
}

/**
 * @brief Get field min_pwm from channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性最小PWM；类型是2离散档位时，表示离散档位1的PWM
 */
static inline uint16_t ttalink_channel_function_config_get_min_pwm(const ttalink_message_t* msg, uint16_t *min_pwm)
{
    return _TTA_RETURN_uint16_t_array(msg, min_pwm, 16,  194);
}

/**
 * @brief Get field pwm from channel_function_config message
 *
 * @return 离散档位2的PWM
 */
static inline uint16_t ttalink_channel_function_config_get_pwm(const ttalink_message_t* msg, uint16_t *pwm)
{
    return _TTA_RETURN_uint16_t_array(msg, pwm, 16,  226);
}

/**
 * @brief Get field max_pwm from channel_function_config message
 *
 * @return 类型是1线性参数时，表示线性最大PWM；类型是2离散档位时，表示离散档位3的PWM
 */
static inline uint16_t ttalink_channel_function_config_get_max_pwm(const ttalink_message_t* msg, uint16_t *max_pwm)
{
    return _TTA_RETURN_uint16_t_array(msg, max_pwm, 16,  258);
}

/**
 * @brief Get field min_value from channel_function_config message
 *
 * @return 线性最小值
 */
static inline uint16_t ttalink_channel_function_config_get_min_value(const ttalink_message_t* msg, float *min_value)
{
    return _TTA_RETURN_float_array(msg, min_value, 16,  290);
}

/**
 * @brief Get field max_value from channel_function_config message
 *
 * @return 线性最大值
 */
static inline uint16_t ttalink_channel_function_config_get_max_value(const ttalink_message_t* msg, float *max_value)
{
    return _TTA_RETURN_float_array(msg, max_value, 16,  354);
}

/**
 * @brief Decode a channel_function_config message into a struct
 *
 * @param msg The message to decode
 * @param channel_function_config C-struct to decode the message contents into
 */
static inline void ttalink_channel_function_config_decode(const ttalink_message_t* msg, ttalink_channel_function_config_t* channel_function_config)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    channel_function_config->chan_type = ttalink_channel_function_config_get_chan_type(msg);
    channel_function_config->chan_count = ttalink_channel_function_config_get_chan_count(msg);
    ttalink_channel_function_config_get_reverse(msg, channel_function_config->reverse);
    ttalink_channel_function_config_get_type(msg, channel_function_config->type);
    ttalink_channel_function_config_get_count(msg, channel_function_config->count);
    ttalink_channel_function_config_get_action(msg, channel_function_config->action);
    ttalink_channel_function_config_get_action_2(msg, channel_function_config->action_2);
    ttalink_channel_function_config_get_action_3(msg, channel_function_config->action_3);
    ttalink_channel_function_config_get_dev_idx(msg, channel_function_config->dev_idx);
    ttalink_channel_function_config_get_dev_idx_2(msg, channel_function_config->dev_idx_2);
    ttalink_channel_function_config_get_dev_idx_3(msg, channel_function_config->dev_idx_3);
    ttalink_channel_function_config_get_min_pwm(msg, channel_function_config->min_pwm);
    ttalink_channel_function_config_get_pwm(msg, channel_function_config->pwm);
    ttalink_channel_function_config_get_max_pwm(msg, channel_function_config->max_pwm);
    ttalink_channel_function_config_get_min_value(msg, channel_function_config->min_value);
    ttalink_channel_function_config_get_max_value(msg, channel_function_config->max_value);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN? msg->len : TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN;
        memset(channel_function_config, 0, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_LEN);
    memcpy(channel_function_config, _TTA_PAYLOAD(msg), len);
#endif
}
