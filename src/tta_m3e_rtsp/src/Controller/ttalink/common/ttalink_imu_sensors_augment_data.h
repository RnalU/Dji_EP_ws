#pragma once
// MESSAGE IMU_SENSORS_AUGMENT_DATA PACKING

#define TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA 2179

TTAPACKED(
typedef struct __ttalink_imu_sensors_augment_data_t {
 uint32_t update_time; /*<  .*/
 float switch_gyro[3]; /*<  滤波校准后的角速度.*/
 float switch_gyro_var[3]; /*< .*/
 float switch_gyro_e[3]; /*< .*/
 float switch_acc[3]; /*<  滤波校准后的加速度.*/
 float switch_acc_var[3]; /*< .*/
 float switch_acc_e[3]; /*< .*/
 float switch_acc_root; /*< .*/
 float param[4]; /*<  .*/
}) ttalink_imu_sensors_augment_data_t;

#define TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN 96
#define TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN 96
#define TTALINK_MSG_ID_2179_LEN 96
#define TTALINK_MSG_ID_2179_MIN_LEN 96

#define TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC 253
#define TTALINK_MSG_ID_2179_CRC 253

#define TTALINK_MSG_IMU_SENSORS_AUGMENT_DATA_FIELD_SWITCH_GYRO_LEN 3
#define TTALINK_MSG_IMU_SENSORS_AUGMENT_DATA_FIELD_SWITCH_GYRO_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_AUGMENT_DATA_FIELD_SWITCH_GYRO_E_LEN 3
#define TTALINK_MSG_IMU_SENSORS_AUGMENT_DATA_FIELD_SWITCH_ACC_LEN 3
#define TTALINK_MSG_IMU_SENSORS_AUGMENT_DATA_FIELD_SWITCH_ACC_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_AUGMENT_DATA_FIELD_SWITCH_ACC_E_LEN 3
#define TTALINK_MSG_IMU_SENSORS_AUGMENT_DATA_FIELD_PARAM_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_IMU_SENSORS_AUGMENT_DATA { \
    2179, \
    "IMU_SENSORS_AUGMENT_DATA", \
    9, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_imu_sensors_augment_data_t, update_time) }, \
         { "switch_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_imu_sensors_augment_data_t, switch_gyro) }, \
         { "switch_gyro_var", NULL, TTALINK_TYPE_FLOAT, 3, 16, offsetof(ttalink_imu_sensors_augment_data_t, switch_gyro_var) }, \
         { "switch_gyro_e", NULL, TTALINK_TYPE_FLOAT, 3, 28, offsetof(ttalink_imu_sensors_augment_data_t, switch_gyro_e) }, \
         { "switch_acc", NULL, TTALINK_TYPE_FLOAT, 3, 40, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc) }, \
         { "switch_acc_var", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc_var) }, \
         { "switch_acc_e", NULL, TTALINK_TYPE_FLOAT, 3, 64, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc_e) }, \
         { "switch_acc_root", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc_root) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 80, offsetof(ttalink_imu_sensors_augment_data_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_IMU_SENSORS_AUGMENT_DATA { \
    "IMU_SENSORS_AUGMENT_DATA", \
    9, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_imu_sensors_augment_data_t, update_time) }, \
         { "switch_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_imu_sensors_augment_data_t, switch_gyro) }, \
         { "switch_gyro_var", NULL, TTALINK_TYPE_FLOAT, 3, 16, offsetof(ttalink_imu_sensors_augment_data_t, switch_gyro_var) }, \
         { "switch_gyro_e", NULL, TTALINK_TYPE_FLOAT, 3, 28, offsetof(ttalink_imu_sensors_augment_data_t, switch_gyro_e) }, \
         { "switch_acc", NULL, TTALINK_TYPE_FLOAT, 3, 40, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc) }, \
         { "switch_acc_var", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc_var) }, \
         { "switch_acc_e", NULL, TTALINK_TYPE_FLOAT, 3, 64, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc_e) }, \
         { "switch_acc_root", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_imu_sensors_augment_data_t, switch_acc_root) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 80, offsetof(ttalink_imu_sensors_augment_data_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_imu_sensors_augment_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 76, switch_acc_root);
    _tta_put_float_array(buf, 4, switch_gyro, 3);
    _tta_put_float_array(buf, 16, switch_gyro_var, 3);
    _tta_put_float_array(buf, 28, switch_gyro_e, 3);
    _tta_put_float_array(buf, 40, switch_acc, 3);
    _tta_put_float_array(buf, 52, switch_acc_var, 3);
    _tta_put_float_array(buf, 64, switch_acc_e, 3);
    _tta_put_float_array(buf, 80, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN);
#else
    ttalink_imu_sensors_augment_data_t packet;
    packet.update_time = update_time;
    packet.switch_acc_root = switch_acc_root;
    tta_array_memcpy(packet.switch_gyro, switch_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro_var, switch_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro_e, switch_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc_var, switch_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc_e, switch_acc_e, sizeof(float)*3);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC, nocrc);
}

/**
 * @brief Pack a imu_sensors_augment_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param switch_gyro  滤波校准后的角速度.
 * @param switch_gyro_var .
 * @param switch_gyro_e .
 * @param switch_acc  滤波校准后的加速度.
 * @param switch_acc_var .
 * @param switch_acc_e .
 * @param switch_acc_root .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_augment_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param)
{
    return _ttalink_imu_sensors_augment_data_pack(dst_addr, src_addr, msg,  update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, false);
}

/**
 * @brief Pack a imu_sensors_augment_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param switch_gyro  滤波校准后的角速度.
 * @param switch_gyro_var .
 * @param switch_gyro_e .
 * @param switch_acc  滤波校准后的加速度.
 * @param switch_acc_var .
 * @param switch_acc_e .
 * @param switch_acc_root .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_augment_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param)
{
    return _ttalink_imu_sensors_augment_data_pack(dst_addr, src_addr, msg,  update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, true);
}


static inline uint16_t _ttalink_imu_sensors_augment_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const float *switch_gyro,const float *switch_gyro_var,const float *switch_gyro_e,const float *switch_acc,const float *switch_acc_var,const float *switch_acc_e,float switch_acc_root,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 76, switch_acc_root);
    _tta_put_float_array(buf, 4, switch_gyro, 3);
    _tta_put_float_array(buf, 16, switch_gyro_var, 3);
    _tta_put_float_array(buf, 28, switch_gyro_e, 3);
    _tta_put_float_array(buf, 40, switch_acc, 3);
    _tta_put_float_array(buf, 52, switch_acc_var, 3);
    _tta_put_float_array(buf, 64, switch_acc_e, 3);
    _tta_put_float_array(buf, 80, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN);
#else
    ttalink_imu_sensors_augment_data_t packet;
    packet.update_time = update_time;
    packet.switch_acc_root = switch_acc_root;
    tta_array_memcpy(packet.switch_gyro, switch_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro_var, switch_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro_e, switch_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc_var, switch_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc_e, switch_acc_e, sizeof(float)*3);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC, nocrc);
}

/**
 * @brief Pack a imu_sensors_augment_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param switch_gyro  滤波校准后的角速度.
 * @param switch_gyro_var .
 * @param switch_gyro_e .
 * @param switch_acc  滤波校准后的加速度.
 * @param switch_acc_var .
 * @param switch_acc_e .
 * @param switch_acc_root .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_augment_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const float *switch_gyro,const float *switch_gyro_var,const float *switch_gyro_e,const float *switch_acc,const float *switch_acc_var,const float *switch_acc_e,float switch_acc_root,const float *param)
{
    return _ttalink_imu_sensors_augment_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, false);
}

/**
 * @brief Pack a imu_sensors_augment_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param switch_gyro  滤波校准后的角速度.
 * @param switch_gyro_var .
 * @param switch_gyro_e .
 * @param switch_acc  滤波校准后的加速度.
 * @param switch_acc_var .
 * @param switch_acc_e .
 * @param switch_acc_root .
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_augment_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,const float *switch_gyro,const float *switch_gyro_var,const float *switch_gyro_e,const float *switch_acc,const float *switch_acc_var,const float *switch_acc_e,float switch_acc_root,const float *param)
{
    return _ttalink_imu_sensors_augment_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, true);
}


static inline uint16_t _ttalink_imu_sensors_augment_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data, bool nocrc)
{
    if(nocrc){
        return ttalink_imu_sensors_augment_data_pack_nocrc(dst_addr, src_addr, msg, imu_sensors_augment_data->update_time, imu_sensors_augment_data->switch_gyro, imu_sensors_augment_data->switch_gyro_var, imu_sensors_augment_data->switch_gyro_e, imu_sensors_augment_data->switch_acc, imu_sensors_augment_data->switch_acc_var, imu_sensors_augment_data->switch_acc_e, imu_sensors_augment_data->switch_acc_root, imu_sensors_augment_data->param);
    }else{
        return ttalink_imu_sensors_augment_data_pack(dst_addr, src_addr, msg, imu_sensors_augment_data->update_time, imu_sensors_augment_data->switch_gyro, imu_sensors_augment_data->switch_gyro_var, imu_sensors_augment_data->switch_gyro_e, imu_sensors_augment_data->switch_acc, imu_sensors_augment_data->switch_acc_var, imu_sensors_augment_data->switch_acc_e, imu_sensors_augment_data->switch_acc_root, imu_sensors_augment_data->param);
    }
    
}

/**
 * @brief Encode a imu_sensors_augment_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_augment_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data)
{
    return _ttalink_imu_sensors_augment_data_encode(dst_addr, src_addr, msg, imu_sensors_augment_data, false);
}

/**
 * @brief Encode a imu_sensors_augment_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_augment_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data)
{
    return _ttalink_imu_sensors_augment_data_encode(dst_addr, src_addr, msg, imu_sensors_augment_data, true);
}


static inline uint16_t _ttalink_imu_sensors_augment_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data, bool nocrc)
{
    if(nocrc){
        return ttalink_imu_sensors_augment_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, imu_sensors_augment_data->update_time, imu_sensors_augment_data->switch_gyro, imu_sensors_augment_data->switch_gyro_var, imu_sensors_augment_data->switch_gyro_e, imu_sensors_augment_data->switch_acc, imu_sensors_augment_data->switch_acc_var, imu_sensors_augment_data->switch_acc_e, imu_sensors_augment_data->switch_acc_root, imu_sensors_augment_data->param);
    }else{
        return ttalink_imu_sensors_augment_data_pack_chan(dst_addr, src_addr, chan, msg, imu_sensors_augment_data->update_time, imu_sensors_augment_data->switch_gyro, imu_sensors_augment_data->switch_gyro_var, imu_sensors_augment_data->switch_gyro_e, imu_sensors_augment_data->switch_acc, imu_sensors_augment_data->switch_acc_var, imu_sensors_augment_data->switch_acc_e, imu_sensors_augment_data->switch_acc_root, imu_sensors_augment_data->param);
    }
}

/**
 * @brief Encode a imu_sensors_augment_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_augment_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data)
{
    return _ttalink_imu_sensors_augment_data_encode_chan(dst_addr, src_addr, chan, msg, imu_sensors_augment_data, false);
}

/**
 * @brief Encode a imu_sensors_augment_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_augment_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_augment_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data)
{
    return _ttalink_imu_sensors_augment_data_encode_chan(dst_addr, src_addr, chan, msg, imu_sensors_augment_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_imu_sensors_augment_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 76, switch_acc_root);
    _tta_put_float_array(buf, 4, switch_gyro, 3);
    _tta_put_float_array(buf, 16, switch_gyro_var, 3);
    _tta_put_float_array(buf, 28, switch_gyro_e, 3);
    _tta_put_float_array(buf, 40, switch_acc, 3);
    _tta_put_float_array(buf, 52, switch_acc_var, 3);
    _tta_put_float_array(buf, 64, switch_acc_e, 3);
    _tta_put_float_array(buf, 80, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA, buf, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC, nocrc);
#else
    ttalink_imu_sensors_augment_data_t packet;
    packet.update_time = update_time;
    packet.switch_acc_root = switch_acc_root;
    tta_array_memcpy(packet.switch_gyro, switch_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro_var, switch_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro_e, switch_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc_var, switch_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc_e, switch_acc_e, sizeof(float)*3);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA, (const char *)&packet, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a imu_sensors_augment_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param switch_gyro  滤波校准后的角速度.
 * @param switch_gyro_var .
 * @param switch_gyro_e .
 * @param switch_acc  滤波校准后的加速度.
 * @param switch_acc_var .
 * @param switch_acc_e .
 * @param switch_acc_root .
 * @param param  .
 */
static inline void ttalink_imu_sensors_augment_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param)
{
    _ttalink_imu_sensors_augment_data_send(dst_addr, src_addr, chan, update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, false);
}

/**
 * @brief Send a imu_sensors_augment_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param switch_gyro  滤波校准后的角速度.
 * @param switch_gyro_var .
 * @param switch_gyro_e .
 * @param switch_acc  滤波校准后的加速度.
 * @param switch_acc_var .
 * @param switch_acc_e .
 * @param switch_acc_root .
 * @param param  .
 */
static inline void ttalink_imu_sensors_augment_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param)
{
    _ttalink_imu_sensors_augment_data_send(dst_addr, src_addr, chan, update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, true);
}


static inline void _ttalink_imu_sensors_augment_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_imu_sensors_augment_data_send_nocrc(dst_addr, src_addr, chan, imu_sensors_augment_data->update_time, imu_sensors_augment_data->switch_gyro, imu_sensors_augment_data->switch_gyro_var, imu_sensors_augment_data->switch_gyro_e, imu_sensors_augment_data->switch_acc, imu_sensors_augment_data->switch_acc_var, imu_sensors_augment_data->switch_acc_e, imu_sensors_augment_data->switch_acc_root, imu_sensors_augment_data->param);
    }else{
        ttalink_imu_sensors_augment_data_send(dst_addr, src_addr, chan, imu_sensors_augment_data->update_time, imu_sensors_augment_data->switch_gyro, imu_sensors_augment_data->switch_gyro_var, imu_sensors_augment_data->switch_gyro_e, imu_sensors_augment_data->switch_acc, imu_sensors_augment_data->switch_acc_var, imu_sensors_augment_data->switch_acc_e, imu_sensors_augment_data->switch_acc_root, imu_sensors_augment_data->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA, (const char *)imu_sensors_augment_data, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a imu_sensors_augment_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_imu_sensors_augment_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data)
{
    _ttalink_imu_sensors_augment_data_send_struct(dst_addr, src_addr, chan, imu_sensors_augment_data, false);
}

/**
 * @brief Send a imu_sensors_augment_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_imu_sensors_augment_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data)
{
    _ttalink_imu_sensors_augment_data_send_struct(dst_addr, src_addr, chan, imu_sensors_augment_data, true);
}

#if TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_imu_sensors_augment_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 76, switch_acc_root);
    _tta_put_float_array(buf, 4, switch_gyro, 3);
    _tta_put_float_array(buf, 16, switch_gyro_var, 3);
    _tta_put_float_array(buf, 28, switch_gyro_e, 3);
    _tta_put_float_array(buf, 40, switch_acc, 3);
    _tta_put_float_array(buf, 52, switch_acc_var, 3);
    _tta_put_float_array(buf, 64, switch_acc_e, 3);
    _tta_put_float_array(buf, 80, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA, buf, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC, nocrc);
#else
    ttalink_imu_sensors_augment_data_t *packet = (ttalink_imu_sensors_augment_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->switch_acc_root = switch_acc_root;
    tta_array_memcpy(packet->switch_gyro, switch_gyro, sizeof(float)*3);
    tta_array_memcpy(packet->switch_gyro_var, switch_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet->switch_gyro_e, switch_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet->switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet->switch_acc_var, switch_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet->switch_acc_e, switch_acc_e, sizeof(float)*3);
    tta_array_memcpy(packet->param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA, (const char *)packet, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_imu_sensors_augment_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param)
{
    _ttalink_imu_sensors_augment_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_imu_sensors_augment_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, const float *switch_gyro, const float *switch_gyro_var, const float *switch_gyro_e, const float *switch_acc, const float *switch_acc_var, const float *switch_acc_e, float switch_acc_root, const float *param)
{
    _ttalink_imu_sensors_augment_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, switch_gyro, switch_gyro_var, switch_gyro_e, switch_acc, switch_acc_var, switch_acc_e, switch_acc_root, param, true);
}
#endif

#endif

// MESSAGE IMU_SENSORS_AUGMENT_DATA UNPACKING


/**
 * @brief Get field update_time from imu_sensors_augment_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_imu_sensors_augment_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field switch_gyro from imu_sensors_augment_data message
 *
 * @return  滤波校准后的角速度.
 */
static inline uint16_t ttalink_imu_sensors_augment_data_get_switch_gyro(const ttalink_message_t* msg, float *switch_gyro)
{
    return _TTA_RETURN_float_array(msg, switch_gyro, 3,  4);
}

/**
 * @brief Get field switch_gyro_var from imu_sensors_augment_data message
 *
 * @return .
 */
static inline uint16_t ttalink_imu_sensors_augment_data_get_switch_gyro_var(const ttalink_message_t* msg, float *switch_gyro_var)
{
    return _TTA_RETURN_float_array(msg, switch_gyro_var, 3,  16);
}

/**
 * @brief Get field switch_gyro_e from imu_sensors_augment_data message
 *
 * @return .
 */
static inline uint16_t ttalink_imu_sensors_augment_data_get_switch_gyro_e(const ttalink_message_t* msg, float *switch_gyro_e)
{
    return _TTA_RETURN_float_array(msg, switch_gyro_e, 3,  28);
}

/**
 * @brief Get field switch_acc from imu_sensors_augment_data message
 *
 * @return  滤波校准后的加速度.
 */
static inline uint16_t ttalink_imu_sensors_augment_data_get_switch_acc(const ttalink_message_t* msg, float *switch_acc)
{
    return _TTA_RETURN_float_array(msg, switch_acc, 3,  40);
}

/**
 * @brief Get field switch_acc_var from imu_sensors_augment_data message
 *
 * @return .
 */
static inline uint16_t ttalink_imu_sensors_augment_data_get_switch_acc_var(const ttalink_message_t* msg, float *switch_acc_var)
{
    return _TTA_RETURN_float_array(msg, switch_acc_var, 3,  52);
}

/**
 * @brief Get field switch_acc_e from imu_sensors_augment_data message
 *
 * @return .
 */
static inline uint16_t ttalink_imu_sensors_augment_data_get_switch_acc_e(const ttalink_message_t* msg, float *switch_acc_e)
{
    return _TTA_RETURN_float_array(msg, switch_acc_e, 3,  64);
}

/**
 * @brief Get field switch_acc_root from imu_sensors_augment_data message
 *
 * @return .
 */
static inline float ttalink_imu_sensors_augment_data_get_switch_acc_root(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  76);
}

/**
 * @brief Get field param from imu_sensors_augment_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_augment_data_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 4,  80);
}

/**
 * @brief Decode a imu_sensors_augment_data message into a struct
 *
 * @param msg The message to decode
 * @param imu_sensors_augment_data C-struct to decode the message contents into
 */
static inline void ttalink_imu_sensors_augment_data_decode(const ttalink_message_t* msg, ttalink_imu_sensors_augment_data_t* imu_sensors_augment_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    imu_sensors_augment_data->update_time = ttalink_imu_sensors_augment_data_get_update_time(msg);
    ttalink_imu_sensors_augment_data_get_switch_gyro(msg, imu_sensors_augment_data->switch_gyro);
    ttalink_imu_sensors_augment_data_get_switch_gyro_var(msg, imu_sensors_augment_data->switch_gyro_var);
    ttalink_imu_sensors_augment_data_get_switch_gyro_e(msg, imu_sensors_augment_data->switch_gyro_e);
    ttalink_imu_sensors_augment_data_get_switch_acc(msg, imu_sensors_augment_data->switch_acc);
    ttalink_imu_sensors_augment_data_get_switch_acc_var(msg, imu_sensors_augment_data->switch_acc_var);
    ttalink_imu_sensors_augment_data_get_switch_acc_e(msg, imu_sensors_augment_data->switch_acc_e);
    imu_sensors_augment_data->switch_acc_root = ttalink_imu_sensors_augment_data_get_switch_acc_root(msg);
    ttalink_imu_sensors_augment_data_get_param(msg, imu_sensors_augment_data->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN? msg->len : TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN;
        memset(imu_sensors_augment_data, 0, TTALINK_MSG_ID_IMU_SENSORS_AUGMENT_DATA_LEN);
    memcpy(imu_sensors_augment_data, _TTA_PAYLOAD(msg), len);
#endif
}
