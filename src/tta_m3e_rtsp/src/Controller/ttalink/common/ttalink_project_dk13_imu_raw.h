#pragma once
// MESSAGE PROJECT_DK13_IMU_RAW PACKING

#define TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW 5100

TTAPACKED(
typedef struct __ttalink_project_dk13_imu_raw_t {
 float origin_gyro_data[3]; /*< 输出角速度，xyz*/
 float origin_acc_data[3]; /*< 输出加速度，xyz*/
 float origin_gyro_temp[3]; /*< 温度.*/
 float origin_q_out_x[3]; /*< 正交幅度.*/
 float origin_dramp_out_x[3]; /*< 驱动幅度.*/
 float imu_param; /*< 预留参数.*/
}) ttalink_project_dk13_imu_raw_t;

#define TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN 64
#define TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN 64
#define TTALINK_MSG_ID_5100_LEN 64
#define TTALINK_MSG_ID_5100_MIN_LEN 64

#define TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC 178
#define TTALINK_MSG_ID_5100_CRC 178

#define TTALINK_MSG_PROJECT_DK13_IMU_RAW_FIELD_ORIGIN_GYRO_DATA_LEN 3
#define TTALINK_MSG_PROJECT_DK13_IMU_RAW_FIELD_ORIGIN_ACC_DATA_LEN 3
#define TTALINK_MSG_PROJECT_DK13_IMU_RAW_FIELD_ORIGIN_GYRO_TEMP_LEN 3
#define TTALINK_MSG_PROJECT_DK13_IMU_RAW_FIELD_ORIGIN_Q_OUT_X_LEN 3
#define TTALINK_MSG_PROJECT_DK13_IMU_RAW_FIELD_ORIGIN_DRAMP_OUT_X_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_PROJECT_DK13_IMU_RAW { \
    5100, \
    "PROJECT_DK13_IMU_RAW", \
    6, \
    {  { "origin_gyro_data", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_project_dk13_imu_raw_t, origin_gyro_data) }, \
         { "origin_acc_data", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_project_dk13_imu_raw_t, origin_acc_data) }, \
         { "origin_gyro_temp", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_project_dk13_imu_raw_t, origin_gyro_temp) }, \
         { "origin_q_out_x", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_project_dk13_imu_raw_t, origin_q_out_x) }, \
         { "origin_dramp_out_x", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_project_dk13_imu_raw_t, origin_dramp_out_x) }, \
         { "imu_param", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_project_dk13_imu_raw_t, imu_param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_PROJECT_DK13_IMU_RAW { \
    "PROJECT_DK13_IMU_RAW", \
    6, \
    {  { "origin_gyro_data", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_project_dk13_imu_raw_t, origin_gyro_data) }, \
         { "origin_acc_data", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_project_dk13_imu_raw_t, origin_acc_data) }, \
         { "origin_gyro_temp", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_project_dk13_imu_raw_t, origin_gyro_temp) }, \
         { "origin_q_out_x", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_project_dk13_imu_raw_t, origin_q_out_x) }, \
         { "origin_dramp_out_x", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_project_dk13_imu_raw_t, origin_dramp_out_x) }, \
         { "imu_param", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_project_dk13_imu_raw_t, imu_param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_project_dk13_imu_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN];
    _tta_put_float(buf, 60, imu_param);
    _tta_put_float_array(buf, 0, origin_gyro_data, 3);
    _tta_put_float_array(buf, 12, origin_acc_data, 3);
    _tta_put_float_array(buf, 24, origin_gyro_temp, 3);
    _tta_put_float_array(buf, 36, origin_q_out_x, 3);
    _tta_put_float_array(buf, 48, origin_dramp_out_x, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN);
#else
    ttalink_project_dk13_imu_raw_t packet;
    packet.imu_param = imu_param;
    tta_array_memcpy(packet.origin_gyro_data, origin_gyro_data, sizeof(float)*3);
    tta_array_memcpy(packet.origin_acc_data, origin_acc_data, sizeof(float)*3);
    tta_array_memcpy(packet.origin_gyro_temp, origin_gyro_temp, sizeof(float)*3);
    tta_array_memcpy(packet.origin_q_out_x, origin_q_out_x, sizeof(float)*3);
    tta_array_memcpy(packet.origin_dramp_out_x, origin_dramp_out_x, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC, nocrc);
}

/**
 * @brief Pack a project_dk13_imu_raw message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param origin_gyro_data 输出角速度，xyz
 * @param origin_acc_data 输出加速度，xyz
 * @param origin_gyro_temp 温度.
 * @param origin_q_out_x 正交幅度.
 * @param origin_dramp_out_x 驱动幅度.
 * @param imu_param 预留参数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_project_dk13_imu_raw_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param)
{
    return _ttalink_project_dk13_imu_raw_pack(dst_addr, src_addr, msg,  origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, false);
}

/**
 * @brief Pack a project_dk13_imu_raw message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param origin_gyro_data 输出角速度，xyz
 * @param origin_acc_data 输出加速度，xyz
 * @param origin_gyro_temp 温度.
 * @param origin_q_out_x 正交幅度.
 * @param origin_dramp_out_x 驱动幅度.
 * @param imu_param 预留参数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_project_dk13_imu_raw_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param)
{
    return _ttalink_project_dk13_imu_raw_pack(dst_addr, src_addr, msg,  origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, true);
}


static inline uint16_t _ttalink_project_dk13_imu_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *origin_gyro_data,const float *origin_acc_data,const float *origin_gyro_temp,const float *origin_q_out_x,const float *origin_dramp_out_x,float imu_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN];
    _tta_put_float(buf, 60, imu_param);
    _tta_put_float_array(buf, 0, origin_gyro_data, 3);
    _tta_put_float_array(buf, 12, origin_acc_data, 3);
    _tta_put_float_array(buf, 24, origin_gyro_temp, 3);
    _tta_put_float_array(buf, 36, origin_q_out_x, 3);
    _tta_put_float_array(buf, 48, origin_dramp_out_x, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN);
#else
    ttalink_project_dk13_imu_raw_t packet;
    packet.imu_param = imu_param;
    tta_array_memcpy(packet.origin_gyro_data, origin_gyro_data, sizeof(float)*3);
    tta_array_memcpy(packet.origin_acc_data, origin_acc_data, sizeof(float)*3);
    tta_array_memcpy(packet.origin_gyro_temp, origin_gyro_temp, sizeof(float)*3);
    tta_array_memcpy(packet.origin_q_out_x, origin_q_out_x, sizeof(float)*3);
    tta_array_memcpy(packet.origin_dramp_out_x, origin_dramp_out_x, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC, nocrc);
}

/**
 * @brief Pack a project_dk13_imu_raw message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param origin_gyro_data 输出角速度，xyz
 * @param origin_acc_data 输出加速度，xyz
 * @param origin_gyro_temp 温度.
 * @param origin_q_out_x 正交幅度.
 * @param origin_dramp_out_x 驱动幅度.
 * @param imu_param 预留参数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_project_dk13_imu_raw_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *origin_gyro_data,const float *origin_acc_data,const float *origin_gyro_temp,const float *origin_q_out_x,const float *origin_dramp_out_x,float imu_param)
{
    return _ttalink_project_dk13_imu_raw_pack_chan(dst_addr, src_addr, chan, msg,  origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, false);
}

/**
 * @brief Pack a project_dk13_imu_raw message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param origin_gyro_data 输出角速度，xyz
 * @param origin_acc_data 输出加速度，xyz
 * @param origin_gyro_temp 温度.
 * @param origin_q_out_x 正交幅度.
 * @param origin_dramp_out_x 驱动幅度.
 * @param imu_param 预留参数.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_project_dk13_imu_raw_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *origin_gyro_data,const float *origin_acc_data,const float *origin_gyro_temp,const float *origin_q_out_x,const float *origin_dramp_out_x,float imu_param)
{
    return _ttalink_project_dk13_imu_raw_pack_chan(dst_addr, src_addr, chan, msg,  origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, true);
}


static inline uint16_t _ttalink_project_dk13_imu_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_project_dk13_imu_raw_pack_nocrc(dst_addr, src_addr, msg, project_dk13_imu_raw->origin_gyro_data, project_dk13_imu_raw->origin_acc_data, project_dk13_imu_raw->origin_gyro_temp, project_dk13_imu_raw->origin_q_out_x, project_dk13_imu_raw->origin_dramp_out_x, project_dk13_imu_raw->imu_param);
    }else{
        return ttalink_project_dk13_imu_raw_pack(dst_addr, src_addr, msg, project_dk13_imu_raw->origin_gyro_data, project_dk13_imu_raw->origin_acc_data, project_dk13_imu_raw->origin_gyro_temp, project_dk13_imu_raw->origin_q_out_x, project_dk13_imu_raw->origin_dramp_out_x, project_dk13_imu_raw->imu_param);
    }
    
}

/**
 * @brief Encode a project_dk13_imu_raw struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param project_dk13_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_project_dk13_imu_raw_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw)
{
    return _ttalink_project_dk13_imu_raw_encode(dst_addr, src_addr, msg, project_dk13_imu_raw, false);
}

/**
 * @brief Encode a project_dk13_imu_raw struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param project_dk13_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_project_dk13_imu_raw_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw)
{
    return _ttalink_project_dk13_imu_raw_encode(dst_addr, src_addr, msg, project_dk13_imu_raw, true);
}


static inline uint16_t _ttalink_project_dk13_imu_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw, bool nocrc)
{
    if(nocrc){
        return ttalink_project_dk13_imu_raw_pack_chan_nocrc(dst_addr, src_addr, chan, msg, project_dk13_imu_raw->origin_gyro_data, project_dk13_imu_raw->origin_acc_data, project_dk13_imu_raw->origin_gyro_temp, project_dk13_imu_raw->origin_q_out_x, project_dk13_imu_raw->origin_dramp_out_x, project_dk13_imu_raw->imu_param);
    }else{
        return ttalink_project_dk13_imu_raw_pack_chan(dst_addr, src_addr, chan, msg, project_dk13_imu_raw->origin_gyro_data, project_dk13_imu_raw->origin_acc_data, project_dk13_imu_raw->origin_gyro_temp, project_dk13_imu_raw->origin_q_out_x, project_dk13_imu_raw->origin_dramp_out_x, project_dk13_imu_raw->imu_param);
    }
}

/**
 * @brief Encode a project_dk13_imu_raw struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param project_dk13_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_project_dk13_imu_raw_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw)
{
    return _ttalink_project_dk13_imu_raw_encode_chan(dst_addr, src_addr, chan, msg, project_dk13_imu_raw, false);
}

/**
 * @brief Encode a project_dk13_imu_raw struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param project_dk13_imu_raw C-struct to read the message contents from
 */
static inline uint16_t ttalink_project_dk13_imu_raw_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw)
{
    return _ttalink_project_dk13_imu_raw_encode_chan(dst_addr, src_addr, chan, msg, project_dk13_imu_raw, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_project_dk13_imu_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN];
    _tta_put_float(buf, 60, imu_param);
    _tta_put_float_array(buf, 0, origin_gyro_data, 3);
    _tta_put_float_array(buf, 12, origin_acc_data, 3);
    _tta_put_float_array(buf, 24, origin_gyro_temp, 3);
    _tta_put_float_array(buf, 36, origin_q_out_x, 3);
    _tta_put_float_array(buf, 48, origin_dramp_out_x, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW, buf, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC, nocrc);
#else
    ttalink_project_dk13_imu_raw_t packet;
    packet.imu_param = imu_param;
    tta_array_memcpy(packet.origin_gyro_data, origin_gyro_data, sizeof(float)*3);
    tta_array_memcpy(packet.origin_acc_data, origin_acc_data, sizeof(float)*3);
    tta_array_memcpy(packet.origin_gyro_temp, origin_gyro_temp, sizeof(float)*3);
    tta_array_memcpy(packet.origin_q_out_x, origin_q_out_x, sizeof(float)*3);
    tta_array_memcpy(packet.origin_dramp_out_x, origin_dramp_out_x, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW, (const char *)&packet, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a project_dk13_imu_raw message
 * @param chan TTAlink channel to send the message
 *
 * @param origin_gyro_data 输出角速度，xyz
 * @param origin_acc_data 输出加速度，xyz
 * @param origin_gyro_temp 温度.
 * @param origin_q_out_x 正交幅度.
 * @param origin_dramp_out_x 驱动幅度.
 * @param imu_param 预留参数.
 */
static inline void ttalink_project_dk13_imu_raw_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param)
{
    _ttalink_project_dk13_imu_raw_send(dst_addr, src_addr, chan, origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, false);
}

/**
 * @brief Send a project_dk13_imu_raw message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param origin_gyro_data 输出角速度，xyz
 * @param origin_acc_data 输出加速度，xyz
 * @param origin_gyro_temp 温度.
 * @param origin_q_out_x 正交幅度.
 * @param origin_dramp_out_x 驱动幅度.
 * @param imu_param 预留参数.
 */
static inline void ttalink_project_dk13_imu_raw_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param)
{
    _ttalink_project_dk13_imu_raw_send(dst_addr, src_addr, chan, origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, true);
}


static inline void _ttalink_project_dk13_imu_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_project_dk13_imu_raw_send_nocrc(dst_addr, src_addr, chan, project_dk13_imu_raw->origin_gyro_data, project_dk13_imu_raw->origin_acc_data, project_dk13_imu_raw->origin_gyro_temp, project_dk13_imu_raw->origin_q_out_x, project_dk13_imu_raw->origin_dramp_out_x, project_dk13_imu_raw->imu_param);
    }else{
        ttalink_project_dk13_imu_raw_send(dst_addr, src_addr, chan, project_dk13_imu_raw->origin_gyro_data, project_dk13_imu_raw->origin_acc_data, project_dk13_imu_raw->origin_gyro_temp, project_dk13_imu_raw->origin_q_out_x, project_dk13_imu_raw->origin_dramp_out_x, project_dk13_imu_raw->imu_param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW, (const char *)project_dk13_imu_raw, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC, nocrc);
#endif
}

/**
 * @brief Send a project_dk13_imu_raw message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_project_dk13_imu_raw_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw)
{
    _ttalink_project_dk13_imu_raw_send_struct(dst_addr, src_addr, chan, project_dk13_imu_raw, false);
}

/**
 * @brief Send a project_dk13_imu_raw message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_project_dk13_imu_raw_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw)
{
    _ttalink_project_dk13_imu_raw_send_struct(dst_addr, src_addr, chan, project_dk13_imu_raw, true);
}

#if TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_project_dk13_imu_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 60, imu_param);
    _tta_put_float_array(buf, 0, origin_gyro_data, 3);
    _tta_put_float_array(buf, 12, origin_acc_data, 3);
    _tta_put_float_array(buf, 24, origin_gyro_temp, 3);
    _tta_put_float_array(buf, 36, origin_q_out_x, 3);
    _tta_put_float_array(buf, 48, origin_dramp_out_x, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW, buf, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC, nocrc);
#else
    ttalink_project_dk13_imu_raw_t *packet = (ttalink_project_dk13_imu_raw_t *)msgbuf;
    packet->imu_param = imu_param;
    tta_array_memcpy(packet->origin_gyro_data, origin_gyro_data, sizeof(float)*3);
    tta_array_memcpy(packet->origin_acc_data, origin_acc_data, sizeof(float)*3);
    tta_array_memcpy(packet->origin_gyro_temp, origin_gyro_temp, sizeof(float)*3);
    tta_array_memcpy(packet->origin_q_out_x, origin_q_out_x, sizeof(float)*3);
    tta_array_memcpy(packet->origin_dramp_out_x, origin_dramp_out_x, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW, (const char *)packet, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_MIN_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_project_dk13_imu_raw_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param)
{
    _ttalink_project_dk13_imu_raw_send_buf(dst_addr, src_addr, msgbuf, chan, origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_project_dk13_imu_raw_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *origin_gyro_data, const float *origin_acc_data, const float *origin_gyro_temp, const float *origin_q_out_x, const float *origin_dramp_out_x, float imu_param)
{
    _ttalink_project_dk13_imu_raw_send_buf(dst_addr, src_addr, msgbuf, chan, origin_gyro_data, origin_acc_data, origin_gyro_temp, origin_q_out_x, origin_dramp_out_x, imu_param, true);
}
#endif

#endif

// MESSAGE PROJECT_DK13_IMU_RAW UNPACKING


/**
 * @brief Get field origin_gyro_data from project_dk13_imu_raw message
 *
 * @return 输出角速度，xyz
 */
static inline uint16_t ttalink_project_dk13_imu_raw_get_origin_gyro_data(const ttalink_message_t* msg, float *origin_gyro_data)
{
    return _TTA_RETURN_float_array(msg, origin_gyro_data, 3,  0);
}

/**
 * @brief Get field origin_acc_data from project_dk13_imu_raw message
 *
 * @return 输出加速度，xyz
 */
static inline uint16_t ttalink_project_dk13_imu_raw_get_origin_acc_data(const ttalink_message_t* msg, float *origin_acc_data)
{
    return _TTA_RETURN_float_array(msg, origin_acc_data, 3,  12);
}

/**
 * @brief Get field origin_gyro_temp from project_dk13_imu_raw message
 *
 * @return 温度.
 */
static inline uint16_t ttalink_project_dk13_imu_raw_get_origin_gyro_temp(const ttalink_message_t* msg, float *origin_gyro_temp)
{
    return _TTA_RETURN_float_array(msg, origin_gyro_temp, 3,  24);
}

/**
 * @brief Get field origin_q_out_x from project_dk13_imu_raw message
 *
 * @return 正交幅度.
 */
static inline uint16_t ttalink_project_dk13_imu_raw_get_origin_q_out_x(const ttalink_message_t* msg, float *origin_q_out_x)
{
    return _TTA_RETURN_float_array(msg, origin_q_out_x, 3,  36);
}

/**
 * @brief Get field origin_dramp_out_x from project_dk13_imu_raw message
 *
 * @return 驱动幅度.
 */
static inline uint16_t ttalink_project_dk13_imu_raw_get_origin_dramp_out_x(const ttalink_message_t* msg, float *origin_dramp_out_x)
{
    return _TTA_RETURN_float_array(msg, origin_dramp_out_x, 3,  48);
}

/**
 * @brief Get field imu_param from project_dk13_imu_raw message
 *
 * @return 预留参数.
 */
static inline float ttalink_project_dk13_imu_raw_get_imu_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  60);
}

/**
 * @brief Decode a project_dk13_imu_raw message into a struct
 *
 * @param msg The message to decode
 * @param project_dk13_imu_raw C-struct to decode the message contents into
 */
static inline void ttalink_project_dk13_imu_raw_decode(const ttalink_message_t* msg, ttalink_project_dk13_imu_raw_t* project_dk13_imu_raw)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_project_dk13_imu_raw_get_origin_gyro_data(msg, project_dk13_imu_raw->origin_gyro_data);
    ttalink_project_dk13_imu_raw_get_origin_acc_data(msg, project_dk13_imu_raw->origin_acc_data);
    ttalink_project_dk13_imu_raw_get_origin_gyro_temp(msg, project_dk13_imu_raw->origin_gyro_temp);
    ttalink_project_dk13_imu_raw_get_origin_q_out_x(msg, project_dk13_imu_raw->origin_q_out_x);
    ttalink_project_dk13_imu_raw_get_origin_dramp_out_x(msg, project_dk13_imu_raw->origin_dramp_out_x);
    project_dk13_imu_raw->imu_param = ttalink_project_dk13_imu_raw_get_imu_param(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN? msg->len : TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN;
        memset(project_dk13_imu_raw, 0, TTALINK_MSG_ID_PROJECT_DK13_IMU_RAW_LEN);
    memcpy(project_dk13_imu_raw, _TTA_PAYLOAD(msg), len);
#endif
}
