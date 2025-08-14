#pragma once
// MESSAGE POD_1360_UPDATE_PTZ_NAVI_DATA PACKING

#define TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA 11360

TTAPACKED(
typedef struct __ttalink_pod_1360_update_ptz_navi_data_t {
 uint8_t ack; /*< .*/
 float imu_temperature; /*< .*/
 float gyro_x; /*< .*/
 float gyro_y; /*< .*/
 float gyro_z; /*< .*/
 float atti_pitch; /*< .*/
 float atti_roll; /*< .*/
 float atti_yaw; /*< .*/
}) ttalink_pod_1360_update_ptz_navi_data_t;

#define TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN 29
#define TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN 29
#define TTALINK_MSG_ID_11360_LEN 29
#define TTALINK_MSG_ID_11360_MIN_LEN 29

#define TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC 217
#define TTALINK_MSG_ID_11360_CRC 217



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1360_UPDATE_PTZ_NAVI_DATA { \
    11360, \
    "POD_1360_UPDATE_PTZ_NAVI_DATA", \
    8, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, ack) }, \
         { "imu_temperature", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, imu_temperature) }, \
         { "gyro_x", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, gyro_x) }, \
         { "gyro_y", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, gyro_y) }, \
         { "gyro_z", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, gyro_z) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, atti_yaw) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1360_UPDATE_PTZ_NAVI_DATA { \
    "POD_1360_UPDATE_PTZ_NAVI_DATA", \
    8, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, ack) }, \
         { "imu_temperature", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, imu_temperature) }, \
         { "gyro_x", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, gyro_x) }, \
         { "gyro_y", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, gyro_y) }, \
         { "gyro_z", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, gyro_z) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pod_1360_update_ptz_navi_data_t, atti_yaw) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1360_update_ptz_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, imu_temperature);
    _tta_put_float(buf, 5, gyro_x);
    _tta_put_float(buf, 9, gyro_y);
    _tta_put_float(buf, 13, gyro_z);
    _tta_put_float(buf, 17, atti_pitch);
    _tta_put_float(buf, 21, atti_roll);
    _tta_put_float(buf, 25, atti_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN);
#else
    ttalink_pod_1360_update_ptz_navi_data_t packet;
    packet.ack = ack;
    packet.imu_temperature = imu_temperature;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1360_update_ptz_navi_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param imu_temperature .
 * @param gyro_x .
 * @param gyro_y .
 * @param gyro_z .
 * @param atti_pitch .
 * @param atti_roll .
 * @param atti_yaw .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw)
{
    return _ttalink_pod_1360_update_ptz_navi_data_pack(dst_addr, src_addr, msg,  ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, false);
}

/**
 * @brief Pack a pod_1360_update_ptz_navi_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param imu_temperature .
 * @param gyro_x .
 * @param gyro_y .
 * @param gyro_z .
 * @param atti_pitch .
 * @param atti_roll .
 * @param atti_yaw .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw)
{
    return _ttalink_pod_1360_update_ptz_navi_data_pack(dst_addr, src_addr, msg,  ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, true);
}


static inline uint16_t _ttalink_pod_1360_update_ptz_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float imu_temperature,float gyro_x,float gyro_y,float gyro_z,float atti_pitch,float atti_roll,float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, imu_temperature);
    _tta_put_float(buf, 5, gyro_x);
    _tta_put_float(buf, 9, gyro_y);
    _tta_put_float(buf, 13, gyro_z);
    _tta_put_float(buf, 17, atti_pitch);
    _tta_put_float(buf, 21, atti_roll);
    _tta_put_float(buf, 25, atti_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN);
#else
    ttalink_pod_1360_update_ptz_navi_data_t packet;
    packet.ack = ack;
    packet.imu_temperature = imu_temperature;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1360_update_ptz_navi_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param imu_temperature .
 * @param gyro_x .
 * @param gyro_y .
 * @param gyro_z .
 * @param atti_pitch .
 * @param atti_roll .
 * @param atti_yaw .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float imu_temperature,float gyro_x,float gyro_y,float gyro_z,float atti_pitch,float atti_roll,float atti_yaw)
{
    return _ttalink_pod_1360_update_ptz_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, false);
}

/**
 * @brief Pack a pod_1360_update_ptz_navi_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param imu_temperature .
 * @param gyro_x .
 * @param gyro_y .
 * @param gyro_z .
 * @param atti_pitch .
 * @param atti_roll .
 * @param atti_yaw .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,float imu_temperature,float gyro_x,float gyro_y,float gyro_z,float atti_pitch,float atti_roll,float atti_yaw)
{
    return _ttalink_pod_1360_update_ptz_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, true);
}


static inline uint16_t _ttalink_pod_1360_update_ptz_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1360_update_ptz_navi_data_pack_nocrc(dst_addr, src_addr, msg, pod_1360_update_ptz_navi_data->ack, pod_1360_update_ptz_navi_data->imu_temperature, pod_1360_update_ptz_navi_data->gyro_x, pod_1360_update_ptz_navi_data->gyro_y, pod_1360_update_ptz_navi_data->gyro_z, pod_1360_update_ptz_navi_data->atti_pitch, pod_1360_update_ptz_navi_data->atti_roll, pod_1360_update_ptz_navi_data->atti_yaw);
    }else{
        return ttalink_pod_1360_update_ptz_navi_data_pack(dst_addr, src_addr, msg, pod_1360_update_ptz_navi_data->ack, pod_1360_update_ptz_navi_data->imu_temperature, pod_1360_update_ptz_navi_data->gyro_x, pod_1360_update_ptz_navi_data->gyro_y, pod_1360_update_ptz_navi_data->gyro_z, pod_1360_update_ptz_navi_data->atti_pitch, pod_1360_update_ptz_navi_data->atti_roll, pod_1360_update_ptz_navi_data->atti_yaw);
    }
    
}

/**
 * @brief Encode a pod_1360_update_ptz_navi_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1360_update_ptz_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data)
{
    return _ttalink_pod_1360_update_ptz_navi_data_encode(dst_addr, src_addr, msg, pod_1360_update_ptz_navi_data, false);
}

/**
 * @brief Encode a pod_1360_update_ptz_navi_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1360_update_ptz_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data)
{
    return _ttalink_pod_1360_update_ptz_navi_data_encode(dst_addr, src_addr, msg, pod_1360_update_ptz_navi_data, true);
}


static inline uint16_t _ttalink_pod_1360_update_ptz_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1360_update_ptz_navi_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1360_update_ptz_navi_data->ack, pod_1360_update_ptz_navi_data->imu_temperature, pod_1360_update_ptz_navi_data->gyro_x, pod_1360_update_ptz_navi_data->gyro_y, pod_1360_update_ptz_navi_data->gyro_z, pod_1360_update_ptz_navi_data->atti_pitch, pod_1360_update_ptz_navi_data->atti_roll, pod_1360_update_ptz_navi_data->atti_yaw);
    }else{
        return ttalink_pod_1360_update_ptz_navi_data_pack_chan(dst_addr, src_addr, chan, msg, pod_1360_update_ptz_navi_data->ack, pod_1360_update_ptz_navi_data->imu_temperature, pod_1360_update_ptz_navi_data->gyro_x, pod_1360_update_ptz_navi_data->gyro_y, pod_1360_update_ptz_navi_data->gyro_z, pod_1360_update_ptz_navi_data->atti_pitch, pod_1360_update_ptz_navi_data->atti_roll, pod_1360_update_ptz_navi_data->atti_yaw);
    }
}

/**
 * @brief Encode a pod_1360_update_ptz_navi_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1360_update_ptz_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data)
{
    return _ttalink_pod_1360_update_ptz_navi_data_encode_chan(dst_addr, src_addr, chan, msg, pod_1360_update_ptz_navi_data, false);
}

/**
 * @brief Encode a pod_1360_update_ptz_navi_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1360_update_ptz_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1360_update_ptz_navi_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data)
{
    return _ttalink_pod_1360_update_ptz_navi_data_encode_chan(dst_addr, src_addr, chan, msg, pod_1360_update_ptz_navi_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1360_update_ptz_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, imu_temperature);
    _tta_put_float(buf, 5, gyro_x);
    _tta_put_float(buf, 9, gyro_y);
    _tta_put_float(buf, 13, gyro_z);
    _tta_put_float(buf, 17, atti_pitch);
    _tta_put_float(buf, 21, atti_roll);
    _tta_put_float(buf, 25, atti_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA, buf, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC, nocrc);
#else
    ttalink_pod_1360_update_ptz_navi_data_t packet;
    packet.ack = ack;
    packet.imu_temperature = imu_temperature;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA, (const char *)&packet, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1360_update_ptz_navi_data message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param imu_temperature .
 * @param gyro_x .
 * @param gyro_y .
 * @param gyro_z .
 * @param atti_pitch .
 * @param atti_roll .
 * @param atti_yaw .
 */
static inline void ttalink_pod_1360_update_ptz_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_1360_update_ptz_navi_data_send(dst_addr, src_addr, chan, ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, false);
}

/**
 * @brief Send a pod_1360_update_ptz_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param imu_temperature .
 * @param gyro_x .
 * @param gyro_y .
 * @param gyro_z .
 * @param atti_pitch .
 * @param atti_roll .
 * @param atti_yaw .
 */
static inline void ttalink_pod_1360_update_ptz_navi_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_1360_update_ptz_navi_data_send(dst_addr, src_addr, chan, ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, true);
}


static inline void _ttalink_pod_1360_update_ptz_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1360_update_ptz_navi_data_send_nocrc(dst_addr, src_addr, chan, pod_1360_update_ptz_navi_data->ack, pod_1360_update_ptz_navi_data->imu_temperature, pod_1360_update_ptz_navi_data->gyro_x, pod_1360_update_ptz_navi_data->gyro_y, pod_1360_update_ptz_navi_data->gyro_z, pod_1360_update_ptz_navi_data->atti_pitch, pod_1360_update_ptz_navi_data->atti_roll, pod_1360_update_ptz_navi_data->atti_yaw);
    }else{
        ttalink_pod_1360_update_ptz_navi_data_send(dst_addr, src_addr, chan, pod_1360_update_ptz_navi_data->ack, pod_1360_update_ptz_navi_data->imu_temperature, pod_1360_update_ptz_navi_data->gyro_x, pod_1360_update_ptz_navi_data->gyro_y, pod_1360_update_ptz_navi_data->gyro_z, pod_1360_update_ptz_navi_data->atti_pitch, pod_1360_update_ptz_navi_data->atti_roll, pod_1360_update_ptz_navi_data->atti_yaw);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA, (const char *)pod_1360_update_ptz_navi_data, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1360_update_ptz_navi_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1360_update_ptz_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data)
{
    _ttalink_pod_1360_update_ptz_navi_data_send_struct(dst_addr, src_addr, chan, pod_1360_update_ptz_navi_data, false);
}

/**
 * @brief Send a pod_1360_update_ptz_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1360_update_ptz_navi_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data)
{
    _ttalink_pod_1360_update_ptz_navi_data_send_struct(dst_addr, src_addr, chan, pod_1360_update_ptz_navi_data, true);
}

#if TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1360_update_ptz_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_float(buf, 1, imu_temperature);
    _tta_put_float(buf, 5, gyro_x);
    _tta_put_float(buf, 9, gyro_y);
    _tta_put_float(buf, 13, gyro_z);
    _tta_put_float(buf, 17, atti_pitch);
    _tta_put_float(buf, 21, atti_roll);
    _tta_put_float(buf, 25, atti_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA, buf, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC, nocrc);
#else
    ttalink_pod_1360_update_ptz_navi_data_t *packet = (ttalink_pod_1360_update_ptz_navi_data_t *)msgbuf;
    packet->ack = ack;
    packet->imu_temperature = imu_temperature;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->atti_pitch = atti_pitch;
    packet->atti_roll = atti_roll;
    packet->atti_yaw = atti_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA, (const char *)packet, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1360_update_ptz_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_1360_update_ptz_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1360_update_ptz_navi_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, float imu_temperature, float gyro_x, float gyro_y, float gyro_z, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_1360_update_ptz_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, ack, imu_temperature, gyro_x, gyro_y, gyro_z, atti_pitch, atti_roll, atti_yaw, true);
}
#endif

#endif

// MESSAGE POD_1360_UPDATE_PTZ_NAVI_DATA UNPACKING


/**
 * @brief Get field ack from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1360_update_ptz_navi_data_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field imu_temperature from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1360_update_ptz_navi_data_get_imu_temperature(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  1);
}

/**
 * @brief Get field gyro_x from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1360_update_ptz_navi_data_get_gyro_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Get field gyro_y from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1360_update_ptz_navi_data_get_gyro_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field gyro_z from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1360_update_ptz_navi_data_get_gyro_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field atti_pitch from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1360_update_ptz_navi_data_get_atti_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Get field atti_roll from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1360_update_ptz_navi_data_get_atti_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field atti_yaw from pod_1360_update_ptz_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1360_update_ptz_navi_data_get_atti_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Decode a pod_1360_update_ptz_navi_data message into a struct
 *
 * @param msg The message to decode
 * @param pod_1360_update_ptz_navi_data C-struct to decode the message contents into
 */
static inline void ttalink_pod_1360_update_ptz_navi_data_decode(const ttalink_message_t* msg, ttalink_pod_1360_update_ptz_navi_data_t* pod_1360_update_ptz_navi_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1360_update_ptz_navi_data->ack = ttalink_pod_1360_update_ptz_navi_data_get_ack(msg);
    pod_1360_update_ptz_navi_data->imu_temperature = ttalink_pod_1360_update_ptz_navi_data_get_imu_temperature(msg);
    pod_1360_update_ptz_navi_data->gyro_x = ttalink_pod_1360_update_ptz_navi_data_get_gyro_x(msg);
    pod_1360_update_ptz_navi_data->gyro_y = ttalink_pod_1360_update_ptz_navi_data_get_gyro_y(msg);
    pod_1360_update_ptz_navi_data->gyro_z = ttalink_pod_1360_update_ptz_navi_data_get_gyro_z(msg);
    pod_1360_update_ptz_navi_data->atti_pitch = ttalink_pod_1360_update_ptz_navi_data_get_atti_pitch(msg);
    pod_1360_update_ptz_navi_data->atti_roll = ttalink_pod_1360_update_ptz_navi_data_get_atti_roll(msg);
    pod_1360_update_ptz_navi_data->atti_yaw = ttalink_pod_1360_update_ptz_navi_data_get_atti_yaw(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN? msg->len : TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN;
        memset(pod_1360_update_ptz_navi_data, 0, TTALINK_MSG_ID_POD_1360_UPDATE_PTZ_NAVI_DATA_LEN);
    memcpy(pod_1360_update_ptz_navi_data, _TTA_PAYLOAD(msg), len);
#endif
}
