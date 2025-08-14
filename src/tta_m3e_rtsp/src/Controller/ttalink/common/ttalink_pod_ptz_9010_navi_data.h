#pragma once
// MESSAGE POD_PTZ_9010_NAVI_DATA PACKING

#define TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA 9010

TTAPACKED(
typedef struct __ttalink_pod_ptz_9010_navi_data_t {
 uint32_t update_time; /*<  update_time*/
 float accN; /*<  acc-n*/
 float accE; /*<  acc-e*/
 float accD; /*<  acc-d*/
 float gyro_pitch; /*<  Euler angular velocity-Pitch*/
 float gyro_roll; /*<  Euler angular velocity-Roll*/
 float gyro_yaw; /*<  Euler angular velocity-Yaw*/
 float atti_pitch; /*<  Euler angular Pitch*/
 float atti_roll; /*<  Euler angular Roll*/
 float atti_yaw; /*<  Euler angular Yaw*/
}) ttalink_pod_ptz_9010_navi_data_t;

#define TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN 40
#define TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN 40
#define TTALINK_MSG_ID_9010_LEN 40
#define TTALINK_MSG_ID_9010_MIN_LEN 40

#define TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC 43
#define TTALINK_MSG_ID_9010_CRC 43



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_PTZ_9010_NAVI_DATA { \
    9010, \
    "POD_PTZ_9010_NAVI_DATA", \
    10, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_ptz_9010_navi_data_t, update_time) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_pod_ptz_9010_navi_data_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_pod_ptz_9010_navi_data_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_pod_ptz_9010_navi_data_t, accD) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_pod_ptz_9010_navi_data_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_pod_ptz_9010_navi_data_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_pod_ptz_9010_navi_data_t, gyro_yaw) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_pod_ptz_9010_navi_data_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_pod_ptz_9010_navi_data_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_pod_ptz_9010_navi_data_t, atti_yaw) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_PTZ_9010_NAVI_DATA { \
    "POD_PTZ_9010_NAVI_DATA", \
    10, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_pod_ptz_9010_navi_data_t, update_time) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_pod_ptz_9010_navi_data_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_pod_ptz_9010_navi_data_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_pod_ptz_9010_navi_data_t, accD) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_pod_ptz_9010_navi_data_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_pod_ptz_9010_navi_data_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_pod_ptz_9010_navi_data_t, gyro_yaw) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_pod_ptz_9010_navi_data_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_pod_ptz_9010_navi_data_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_pod_ptz_9010_navi_data_t, atti_yaw) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_ptz_9010_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, accN);
    _tta_put_float(buf, 8, accE);
    _tta_put_float(buf, 12, accD);
    _tta_put_float(buf, 16, gyro_pitch);
    _tta_put_float(buf, 20, gyro_roll);
    _tta_put_float(buf, 24, gyro_yaw);
    _tta_put_float(buf, 28, atti_pitch);
    _tta_put_float(buf, 32, atti_roll);
    _tta_put_float(buf, 36, atti_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN);
#else
    ttalink_pod_ptz_9010_navi_data_t packet;
    packet.update_time = update_time;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_ptz_9010_navi_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param accN  acc-n
 * @param accE  acc-e
 * @param accD  acc-d
 * @param gyro_pitch  Euler angular velocity-Pitch
 * @param gyro_roll  Euler angular velocity-Roll
 * @param gyro_yaw  Euler angular velocity-Yaw
 * @param atti_pitch  Euler angular Pitch
 * @param atti_roll  Euler angular Roll
 * @param atti_yaw  Euler angular Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw)
{
    return _ttalink_pod_ptz_9010_navi_data_pack(dst_addr, src_addr, msg,  update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, false);
}

/**
 * @brief Pack a pod_ptz_9010_navi_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
 * @param accN  acc-n
 * @param accE  acc-e
 * @param accD  acc-d
 * @param gyro_pitch  Euler angular velocity-Pitch
 * @param gyro_roll  Euler angular velocity-Roll
 * @param gyro_yaw  Euler angular velocity-Yaw
 * @param atti_pitch  Euler angular Pitch
 * @param atti_roll  Euler angular Roll
 * @param atti_yaw  Euler angular Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw)
{
    return _ttalink_pod_ptz_9010_navi_data_pack(dst_addr, src_addr, msg,  update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, true);
}


static inline uint16_t _ttalink_pod_ptz_9010_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float accN,float accE,float accD,float gyro_pitch,float gyro_roll,float gyro_yaw,float atti_pitch,float atti_roll,float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, accN);
    _tta_put_float(buf, 8, accE);
    _tta_put_float(buf, 12, accD);
    _tta_put_float(buf, 16, gyro_pitch);
    _tta_put_float(buf, 20, gyro_roll);
    _tta_put_float(buf, 24, gyro_yaw);
    _tta_put_float(buf, 28, atti_pitch);
    _tta_put_float(buf, 32, atti_roll);
    _tta_put_float(buf, 36, atti_yaw);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN);
#else
    ttalink_pod_ptz_9010_navi_data_t packet;
    packet.update_time = update_time;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_ptz_9010_navi_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param accN  acc-n
 * @param accE  acc-e
 * @param accD  acc-d
 * @param gyro_pitch  Euler angular velocity-Pitch
 * @param gyro_roll  Euler angular velocity-Roll
 * @param gyro_yaw  Euler angular velocity-Yaw
 * @param atti_pitch  Euler angular Pitch
 * @param atti_roll  Euler angular Roll
 * @param atti_yaw  Euler angular Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float accN,float accE,float accD,float gyro_pitch,float gyro_roll,float gyro_yaw,float atti_pitch,float atti_roll,float atti_yaw)
{
    return _ttalink_pod_ptz_9010_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, false);
}

/**
 * @brief Pack a pod_ptz_9010_navi_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
 * @param accN  acc-n
 * @param accE  acc-e
 * @param accD  acc-d
 * @param gyro_pitch  Euler angular velocity-Pitch
 * @param gyro_roll  Euler angular velocity-Roll
 * @param gyro_yaw  Euler angular velocity-Yaw
 * @param atti_pitch  Euler angular Pitch
 * @param atti_roll  Euler angular Roll
 * @param atti_yaw  Euler angular Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,float accN,float accE,float accD,float gyro_pitch,float gyro_roll,float gyro_yaw,float atti_pitch,float atti_roll,float atti_yaw)
{
    return _ttalink_pod_ptz_9010_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, true);
}


static inline uint16_t _ttalink_pod_ptz_9010_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_ptz_9010_navi_data_pack_nocrc(dst_addr, src_addr, msg, pod_ptz_9010_navi_data->update_time, pod_ptz_9010_navi_data->accN, pod_ptz_9010_navi_data->accE, pod_ptz_9010_navi_data->accD, pod_ptz_9010_navi_data->gyro_pitch, pod_ptz_9010_navi_data->gyro_roll, pod_ptz_9010_navi_data->gyro_yaw, pod_ptz_9010_navi_data->atti_pitch, pod_ptz_9010_navi_data->atti_roll, pod_ptz_9010_navi_data->atti_yaw);
    }else{
        return ttalink_pod_ptz_9010_navi_data_pack(dst_addr, src_addr, msg, pod_ptz_9010_navi_data->update_time, pod_ptz_9010_navi_data->accN, pod_ptz_9010_navi_data->accE, pod_ptz_9010_navi_data->accD, pod_ptz_9010_navi_data->gyro_pitch, pod_ptz_9010_navi_data->gyro_roll, pod_ptz_9010_navi_data->gyro_yaw, pod_ptz_9010_navi_data->atti_pitch, pod_ptz_9010_navi_data->atti_roll, pod_ptz_9010_navi_data->atti_yaw);
    }
    
}

/**
 * @brief Encode a pod_ptz_9010_navi_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9010_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data)
{
    return _ttalink_pod_ptz_9010_navi_data_encode(dst_addr, src_addr, msg, pod_ptz_9010_navi_data, false);
}

/**
 * @brief Encode a pod_ptz_9010_navi_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9010_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data)
{
    return _ttalink_pod_ptz_9010_navi_data_encode(dst_addr, src_addr, msg, pod_ptz_9010_navi_data, true);
}


static inline uint16_t _ttalink_pod_ptz_9010_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_ptz_9010_navi_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_ptz_9010_navi_data->update_time, pod_ptz_9010_navi_data->accN, pod_ptz_9010_navi_data->accE, pod_ptz_9010_navi_data->accD, pod_ptz_9010_navi_data->gyro_pitch, pod_ptz_9010_navi_data->gyro_roll, pod_ptz_9010_navi_data->gyro_yaw, pod_ptz_9010_navi_data->atti_pitch, pod_ptz_9010_navi_data->atti_roll, pod_ptz_9010_navi_data->atti_yaw);
    }else{
        return ttalink_pod_ptz_9010_navi_data_pack_chan(dst_addr, src_addr, chan, msg, pod_ptz_9010_navi_data->update_time, pod_ptz_9010_navi_data->accN, pod_ptz_9010_navi_data->accE, pod_ptz_9010_navi_data->accD, pod_ptz_9010_navi_data->gyro_pitch, pod_ptz_9010_navi_data->gyro_roll, pod_ptz_9010_navi_data->gyro_yaw, pod_ptz_9010_navi_data->atti_pitch, pod_ptz_9010_navi_data->atti_roll, pod_ptz_9010_navi_data->atti_yaw);
    }
}

/**
 * @brief Encode a pod_ptz_9010_navi_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9010_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data)
{
    return _ttalink_pod_ptz_9010_navi_data_encode_chan(dst_addr, src_addr, chan, msg, pod_ptz_9010_navi_data, false);
}

/**
 * @brief Encode a pod_ptz_9010_navi_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_ptz_9010_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_ptz_9010_navi_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data)
{
    return _ttalink_pod_ptz_9010_navi_data_encode_chan(dst_addr, src_addr, chan, msg, pod_ptz_9010_navi_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_ptz_9010_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, accN);
    _tta_put_float(buf, 8, accE);
    _tta_put_float(buf, 12, accD);
    _tta_put_float(buf, 16, gyro_pitch);
    _tta_put_float(buf, 20, gyro_roll);
    _tta_put_float(buf, 24, gyro_yaw);
    _tta_put_float(buf, 28, atti_pitch);
    _tta_put_float(buf, 32, atti_roll);
    _tta_put_float(buf, 36, atti_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA, buf, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC, nocrc);
#else
    ttalink_pod_ptz_9010_navi_data_t packet;
    packet.update_time = update_time;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA, (const char *)&packet, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_ptz_9010_navi_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param accN  acc-n
 * @param accE  acc-e
 * @param accD  acc-d
 * @param gyro_pitch  Euler angular velocity-Pitch
 * @param gyro_roll  Euler angular velocity-Roll
 * @param gyro_yaw  Euler angular velocity-Yaw
 * @param atti_pitch  Euler angular Pitch
 * @param atti_roll  Euler angular Roll
 * @param atti_yaw  Euler angular Yaw
 */
static inline void ttalink_pod_ptz_9010_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_ptz_9010_navi_data_send(dst_addr, src_addr, chan, update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, false);
}

/**
 * @brief Send a pod_ptz_9010_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
 * @param accN  acc-n
 * @param accE  acc-e
 * @param accD  acc-d
 * @param gyro_pitch  Euler angular velocity-Pitch
 * @param gyro_roll  Euler angular velocity-Roll
 * @param gyro_yaw  Euler angular velocity-Yaw
 * @param atti_pitch  Euler angular Pitch
 * @param atti_roll  Euler angular Roll
 * @param atti_yaw  Euler angular Yaw
 */
static inline void ttalink_pod_ptz_9010_navi_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_ptz_9010_navi_data_send(dst_addr, src_addr, chan, update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, true);
}


static inline void _ttalink_pod_ptz_9010_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_ptz_9010_navi_data_send_nocrc(dst_addr, src_addr, chan, pod_ptz_9010_navi_data->update_time, pod_ptz_9010_navi_data->accN, pod_ptz_9010_navi_data->accE, pod_ptz_9010_navi_data->accD, pod_ptz_9010_navi_data->gyro_pitch, pod_ptz_9010_navi_data->gyro_roll, pod_ptz_9010_navi_data->gyro_yaw, pod_ptz_9010_navi_data->atti_pitch, pod_ptz_9010_navi_data->atti_roll, pod_ptz_9010_navi_data->atti_yaw);
    }else{
        ttalink_pod_ptz_9010_navi_data_send(dst_addr, src_addr, chan, pod_ptz_9010_navi_data->update_time, pod_ptz_9010_navi_data->accN, pod_ptz_9010_navi_data->accE, pod_ptz_9010_navi_data->accD, pod_ptz_9010_navi_data->gyro_pitch, pod_ptz_9010_navi_data->gyro_roll, pod_ptz_9010_navi_data->gyro_yaw, pod_ptz_9010_navi_data->atti_pitch, pod_ptz_9010_navi_data->atti_roll, pod_ptz_9010_navi_data->atti_yaw);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA, (const char *)pod_ptz_9010_navi_data, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_ptz_9010_navi_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_ptz_9010_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data)
{
    _ttalink_pod_ptz_9010_navi_data_send_struct(dst_addr, src_addr, chan, pod_ptz_9010_navi_data, false);
}

/**
 * @brief Send a pod_ptz_9010_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_ptz_9010_navi_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data)
{
    _ttalink_pod_ptz_9010_navi_data_send_struct(dst_addr, src_addr, chan, pod_ptz_9010_navi_data, true);
}

#if TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_ptz_9010_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_float(buf, 4, accN);
    _tta_put_float(buf, 8, accE);
    _tta_put_float(buf, 12, accD);
    _tta_put_float(buf, 16, gyro_pitch);
    _tta_put_float(buf, 20, gyro_roll);
    _tta_put_float(buf, 24, gyro_yaw);
    _tta_put_float(buf, 28, atti_pitch);
    _tta_put_float(buf, 32, atti_roll);
    _tta_put_float(buf, 36, atti_yaw);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA, buf, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC, nocrc);
#else
    ttalink_pod_ptz_9010_navi_data_t *packet = (ttalink_pod_ptz_9010_navi_data_t *)msgbuf;
    packet->update_time = update_time;
    packet->accN = accN;
    packet->accE = accE;
    packet->accD = accD;
    packet->gyro_pitch = gyro_pitch;
    packet->gyro_roll = gyro_roll;
    packet->gyro_yaw = gyro_yaw;
    packet->atti_pitch = atti_pitch;
    packet->atti_roll = atti_roll;
    packet->atti_yaw = atti_yaw;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA, (const char *)packet, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_ptz_9010_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_ptz_9010_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_ptz_9010_navi_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, float accN, float accE, float accD, float gyro_pitch, float gyro_roll, float gyro_yaw, float atti_pitch, float atti_roll, float atti_yaw)
{
    _ttalink_pod_ptz_9010_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, accN, accE, accD, gyro_pitch, gyro_roll, gyro_yaw, atti_pitch, atti_roll, atti_yaw, true);
}
#endif

#endif

// MESSAGE POD_PTZ_9010_NAVI_DATA UNPACKING


/**
 * @brief Get field update_time from pod_ptz_9010_navi_data message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_pod_ptz_9010_navi_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field accN from pod_ptz_9010_navi_data message
 *
 * @return  acc-n
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_accN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field accE from pod_ptz_9010_navi_data message
 *
 * @return  acc-e
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_accE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field accD from pod_ptz_9010_navi_data message
 *
 * @return  acc-d
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_accD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_pitch from pod_ptz_9010_navi_data message
 *
 * @return  Euler angular velocity-Pitch
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_gyro_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_roll from pod_ptz_9010_navi_data message
 *
 * @return  Euler angular velocity-Roll
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_gyro_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro_yaw from pod_ptz_9010_navi_data message
 *
 * @return  Euler angular velocity-Yaw
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_gyro_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field atti_pitch from pod_ptz_9010_navi_data message
 *
 * @return  Euler angular Pitch
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_atti_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field atti_roll from pod_ptz_9010_navi_data message
 *
 * @return  Euler angular Roll
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_atti_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field atti_yaw from pod_ptz_9010_navi_data message
 *
 * @return  Euler angular Yaw
 */
static inline float ttalink_pod_ptz_9010_navi_data_get_atti_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Decode a pod_ptz_9010_navi_data message into a struct
 *
 * @param msg The message to decode
 * @param pod_ptz_9010_navi_data C-struct to decode the message contents into
 */
static inline void ttalink_pod_ptz_9010_navi_data_decode(const ttalink_message_t* msg, ttalink_pod_ptz_9010_navi_data_t* pod_ptz_9010_navi_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_ptz_9010_navi_data->update_time = ttalink_pod_ptz_9010_navi_data_get_update_time(msg);
    pod_ptz_9010_navi_data->accN = ttalink_pod_ptz_9010_navi_data_get_accN(msg);
    pod_ptz_9010_navi_data->accE = ttalink_pod_ptz_9010_navi_data_get_accE(msg);
    pod_ptz_9010_navi_data->accD = ttalink_pod_ptz_9010_navi_data_get_accD(msg);
    pod_ptz_9010_navi_data->gyro_pitch = ttalink_pod_ptz_9010_navi_data_get_gyro_pitch(msg);
    pod_ptz_9010_navi_data->gyro_roll = ttalink_pod_ptz_9010_navi_data_get_gyro_roll(msg);
    pod_ptz_9010_navi_data->gyro_yaw = ttalink_pod_ptz_9010_navi_data_get_gyro_yaw(msg);
    pod_ptz_9010_navi_data->atti_pitch = ttalink_pod_ptz_9010_navi_data_get_atti_pitch(msg);
    pod_ptz_9010_navi_data->atti_roll = ttalink_pod_ptz_9010_navi_data_get_atti_roll(msg);
    pod_ptz_9010_navi_data->atti_yaw = ttalink_pod_ptz_9010_navi_data_get_atti_yaw(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN? msg->len : TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN;
        memset(pod_ptz_9010_navi_data, 0, TTALINK_MSG_ID_POD_PTZ_9010_NAVI_DATA_LEN);
    memcpy(pod_ptz_9010_navi_data, _TTA_PAYLOAD(msg), len);
#endif
}
