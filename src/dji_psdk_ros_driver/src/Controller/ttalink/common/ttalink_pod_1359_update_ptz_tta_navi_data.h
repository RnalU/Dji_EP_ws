#pragma once
// MESSAGE POD_1359_UPDATE_PTZ_TTA_NAVI_DATA PACKING

#define TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA 11359

TTAPACKED(
typedef struct __ttalink_pod_1359_update_ptz_tta_navi_data_t {
 uint8_t ack; /*< .*/
 uint32_t update_time; /*< .*/
 double longi; /*< .*/
 double latit; /*< .*/
 float altit; /*< .*/
 float velN; /*< .*/
 float velE; /*< .*/
 float velD; /*< .*/
 float pitch; /*< .*/
 float roll; /*< .*/
 float yaw; /*< .*/
 float param1; /*< .*/
 float param2; /*< .*/
}) ttalink_pod_1359_update_ptz_tta_navi_data_t;

#define TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN 57
#define TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN 57
#define TTALINK_MSG_ID_11359_LEN 57
#define TTALINK_MSG_ID_11359_MIN_LEN 57

#define TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC 15
#define TTALINK_MSG_ID_11359_CRC 15



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA { \
    11359, \
    "POD_1359_UPDATE_PTZ_TTA_NAVI_DATA", \
    13, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, ack) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, update_time) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 5, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 13, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, velD) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, yaw) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 49, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 53, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA { \
    "POD_1359_UPDATE_PTZ_TTA_NAVI_DATA", \
    13, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, ack) }, \
         { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 1, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, update_time) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 5, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 13, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, velD) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, yaw) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 49, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 53, offsetof(ttalink_pod_1359_update_ptz_tta_navi_data_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1359_update_ptz_tta_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_double(buf, 5, longi);
    _tta_put_double(buf, 13, latit);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, pitch);
    _tta_put_float(buf, 41, roll);
    _tta_put_float(buf, 45, yaw);
    _tta_put_float(buf, 49, param1);
    _tta_put_float(buf, 53, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN);
#else
    ttalink_pod_1359_update_ptz_tta_navi_data_t packet;
    packet.ack = ack;
    packet.update_time = update_time;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1359_update_ptz_tta_navi_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param update_time .
 * @param longi .
 * @param latit .
 * @param altit .
 * @param velN .
 * @param velE .
 * @param velD .
 * @param pitch .
 * @param roll .
 * @param yaw .
 * @param param1 .
 * @param param2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_pack(dst_addr, src_addr, msg,  ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, false);
}

/**
 * @brief Pack a pod_1359_update_ptz_tta_navi_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param update_time .
 * @param longi .
 * @param latit .
 * @param altit .
 * @param velN .
 * @param velE .
 * @param velD .
 * @param pitch .
 * @param roll .
 * @param yaw .
 * @param param1 .
 * @param param2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_pack(dst_addr, src_addr, msg,  ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, true);
}


static inline uint16_t _ttalink_pod_1359_update_ptz_tta_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint32_t update_time,double longi,double latit,float altit,float velN,float velE,float velD,float pitch,float roll,float yaw,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_double(buf, 5, longi);
    _tta_put_double(buf, 13, latit);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, pitch);
    _tta_put_float(buf, 41, roll);
    _tta_put_float(buf, 45, yaw);
    _tta_put_float(buf, 49, param1);
    _tta_put_float(buf, 53, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN);
#else
    ttalink_pod_1359_update_ptz_tta_navi_data_t packet;
    packet.ack = ack;
    packet.update_time = update_time;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1359_update_ptz_tta_navi_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param update_time .
 * @param longi .
 * @param latit .
 * @param altit .
 * @param velN .
 * @param velE .
 * @param velD .
 * @param pitch .
 * @param roll .
 * @param yaw .
 * @param param1 .
 * @param param2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint32_t update_time,double longi,double latit,float altit,float velN,float velE,float velD,float pitch,float roll,float yaw,float param1,float param2)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, false);
}

/**
 * @brief Pack a pod_1359_update_ptz_tta_navi_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param update_time .
 * @param longi .
 * @param latit .
 * @param altit .
 * @param velN .
 * @param velE .
 * @param velD .
 * @param pitch .
 * @param roll .
 * @param yaw .
 * @param param1 .
 * @param param2 .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint32_t update_time,double longi,double latit,float altit,float velN,float velE,float velD,float pitch,float roll,float yaw,float param1,float param2)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, true);
}


static inline uint16_t _ttalink_pod_1359_update_ptz_tta_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1359_update_ptz_tta_navi_data_pack_nocrc(dst_addr, src_addr, msg, pod_1359_update_ptz_tta_navi_data->ack, pod_1359_update_ptz_tta_navi_data->update_time, pod_1359_update_ptz_tta_navi_data->longi, pod_1359_update_ptz_tta_navi_data->latit, pod_1359_update_ptz_tta_navi_data->altit, pod_1359_update_ptz_tta_navi_data->velN, pod_1359_update_ptz_tta_navi_data->velE, pod_1359_update_ptz_tta_navi_data->velD, pod_1359_update_ptz_tta_navi_data->pitch, pod_1359_update_ptz_tta_navi_data->roll, pod_1359_update_ptz_tta_navi_data->yaw, pod_1359_update_ptz_tta_navi_data->param1, pod_1359_update_ptz_tta_navi_data->param2);
    }else{
        return ttalink_pod_1359_update_ptz_tta_navi_data_pack(dst_addr, src_addr, msg, pod_1359_update_ptz_tta_navi_data->ack, pod_1359_update_ptz_tta_navi_data->update_time, pod_1359_update_ptz_tta_navi_data->longi, pod_1359_update_ptz_tta_navi_data->latit, pod_1359_update_ptz_tta_navi_data->altit, pod_1359_update_ptz_tta_navi_data->velN, pod_1359_update_ptz_tta_navi_data->velE, pod_1359_update_ptz_tta_navi_data->velD, pod_1359_update_ptz_tta_navi_data->pitch, pod_1359_update_ptz_tta_navi_data->roll, pod_1359_update_ptz_tta_navi_data->yaw, pod_1359_update_ptz_tta_navi_data->param1, pod_1359_update_ptz_tta_navi_data->param2);
    }
    
}

/**
 * @brief Encode a pod_1359_update_ptz_tta_navi_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1359_update_ptz_tta_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_encode(dst_addr, src_addr, msg, pod_1359_update_ptz_tta_navi_data, false);
}

/**
 * @brief Encode a pod_1359_update_ptz_tta_navi_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1359_update_ptz_tta_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_encode(dst_addr, src_addr, msg, pod_1359_update_ptz_tta_navi_data, true);
}


static inline uint16_t _ttalink_pod_1359_update_ptz_tta_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1359_update_ptz_tta_navi_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1359_update_ptz_tta_navi_data->ack, pod_1359_update_ptz_tta_navi_data->update_time, pod_1359_update_ptz_tta_navi_data->longi, pod_1359_update_ptz_tta_navi_data->latit, pod_1359_update_ptz_tta_navi_data->altit, pod_1359_update_ptz_tta_navi_data->velN, pod_1359_update_ptz_tta_navi_data->velE, pod_1359_update_ptz_tta_navi_data->velD, pod_1359_update_ptz_tta_navi_data->pitch, pod_1359_update_ptz_tta_navi_data->roll, pod_1359_update_ptz_tta_navi_data->yaw, pod_1359_update_ptz_tta_navi_data->param1, pod_1359_update_ptz_tta_navi_data->param2);
    }else{
        return ttalink_pod_1359_update_ptz_tta_navi_data_pack_chan(dst_addr, src_addr, chan, msg, pod_1359_update_ptz_tta_navi_data->ack, pod_1359_update_ptz_tta_navi_data->update_time, pod_1359_update_ptz_tta_navi_data->longi, pod_1359_update_ptz_tta_navi_data->latit, pod_1359_update_ptz_tta_navi_data->altit, pod_1359_update_ptz_tta_navi_data->velN, pod_1359_update_ptz_tta_navi_data->velE, pod_1359_update_ptz_tta_navi_data->velD, pod_1359_update_ptz_tta_navi_data->pitch, pod_1359_update_ptz_tta_navi_data->roll, pod_1359_update_ptz_tta_navi_data->yaw, pod_1359_update_ptz_tta_navi_data->param1, pod_1359_update_ptz_tta_navi_data->param2);
    }
}

/**
 * @brief Encode a pod_1359_update_ptz_tta_navi_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1359_update_ptz_tta_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_encode_chan(dst_addr, src_addr, chan, msg, pod_1359_update_ptz_tta_navi_data, false);
}

/**
 * @brief Encode a pod_1359_update_ptz_tta_navi_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1359_update_ptz_tta_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1359_update_ptz_tta_navi_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data)
{
    return _ttalink_pod_1359_update_ptz_tta_navi_data_encode_chan(dst_addr, src_addr, chan, msg, pod_1359_update_ptz_tta_navi_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1359_update_ptz_tta_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_double(buf, 5, longi);
    _tta_put_double(buf, 13, latit);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, pitch);
    _tta_put_float(buf, 41, roll);
    _tta_put_float(buf, 45, yaw);
    _tta_put_float(buf, 49, param1);
    _tta_put_float(buf, 53, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA, buf, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC, nocrc);
#else
    ttalink_pod_1359_update_ptz_tta_navi_data_t packet;
    packet.ack = ack;
    packet.update_time = update_time;
    packet.longi = longi;
    packet.latit = latit;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA, (const char *)&packet, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1359_update_ptz_tta_navi_data message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param update_time .
 * @param longi .
 * @param latit .
 * @param altit .
 * @param velN .
 * @param velE .
 * @param velD .
 * @param pitch .
 * @param roll .
 * @param yaw .
 * @param param1 .
 * @param param2 .
 */
static inline void ttalink_pod_1359_update_ptz_tta_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2)
{
    _ttalink_pod_1359_update_ptz_tta_navi_data_send(dst_addr, src_addr, chan, ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, false);
}

/**
 * @brief Send a pod_1359_update_ptz_tta_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param update_time .
 * @param longi .
 * @param latit .
 * @param altit .
 * @param velN .
 * @param velE .
 * @param velD .
 * @param pitch .
 * @param roll .
 * @param yaw .
 * @param param1 .
 * @param param2 .
 */
static inline void ttalink_pod_1359_update_ptz_tta_navi_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2)
{
    _ttalink_pod_1359_update_ptz_tta_navi_data_send(dst_addr, src_addr, chan, ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, true);
}


static inline void _ttalink_pod_1359_update_ptz_tta_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1359_update_ptz_tta_navi_data_send_nocrc(dst_addr, src_addr, chan, pod_1359_update_ptz_tta_navi_data->ack, pod_1359_update_ptz_tta_navi_data->update_time, pod_1359_update_ptz_tta_navi_data->longi, pod_1359_update_ptz_tta_navi_data->latit, pod_1359_update_ptz_tta_navi_data->altit, pod_1359_update_ptz_tta_navi_data->velN, pod_1359_update_ptz_tta_navi_data->velE, pod_1359_update_ptz_tta_navi_data->velD, pod_1359_update_ptz_tta_navi_data->pitch, pod_1359_update_ptz_tta_navi_data->roll, pod_1359_update_ptz_tta_navi_data->yaw, pod_1359_update_ptz_tta_navi_data->param1, pod_1359_update_ptz_tta_navi_data->param2);
    }else{
        ttalink_pod_1359_update_ptz_tta_navi_data_send(dst_addr, src_addr, chan, pod_1359_update_ptz_tta_navi_data->ack, pod_1359_update_ptz_tta_navi_data->update_time, pod_1359_update_ptz_tta_navi_data->longi, pod_1359_update_ptz_tta_navi_data->latit, pod_1359_update_ptz_tta_navi_data->altit, pod_1359_update_ptz_tta_navi_data->velN, pod_1359_update_ptz_tta_navi_data->velE, pod_1359_update_ptz_tta_navi_data->velD, pod_1359_update_ptz_tta_navi_data->pitch, pod_1359_update_ptz_tta_navi_data->roll, pod_1359_update_ptz_tta_navi_data->yaw, pod_1359_update_ptz_tta_navi_data->param1, pod_1359_update_ptz_tta_navi_data->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA, (const char *)pod_1359_update_ptz_tta_navi_data, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1359_update_ptz_tta_navi_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1359_update_ptz_tta_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data)
{
    _ttalink_pod_1359_update_ptz_tta_navi_data_send_struct(dst_addr, src_addr, chan, pod_1359_update_ptz_tta_navi_data, false);
}

/**
 * @brief Send a pod_1359_update_ptz_tta_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1359_update_ptz_tta_navi_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data)
{
    _ttalink_pod_1359_update_ptz_tta_navi_data_send_struct(dst_addr, src_addr, chan, pod_1359_update_ptz_tta_navi_data, true);
}

#if TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1359_update_ptz_tta_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint32_t(buf, 1, update_time);
    _tta_put_double(buf, 5, longi);
    _tta_put_double(buf, 13, latit);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, pitch);
    _tta_put_float(buf, 41, roll);
    _tta_put_float(buf, 45, yaw);
    _tta_put_float(buf, 49, param1);
    _tta_put_float(buf, 53, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA, buf, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC, nocrc);
#else
    ttalink_pod_1359_update_ptz_tta_navi_data_t *packet = (ttalink_pod_1359_update_ptz_tta_navi_data_t *)msgbuf;
    packet->ack = ack;
    packet->update_time = update_time;
    packet->longi = longi;
    packet->latit = latit;
    packet->altit = altit;
    packet->velN = velN;
    packet->velE = velE;
    packet->velD = velD;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->yaw = yaw;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA, (const char *)packet, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1359_update_ptz_tta_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2)
{
    _ttalink_pod_1359_update_ptz_tta_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1359_update_ptz_tta_navi_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint32_t update_time, double longi, double latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2)
{
    _ttalink_pod_1359_update_ptz_tta_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, ack, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, true);
}
#endif

#endif

// MESSAGE POD_1359_UPDATE_PTZ_TTA_NAVI_DATA UNPACKING


/**
 * @brief Get field ack from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1359_update_ptz_tta_navi_data_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field update_time from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline uint32_t ttalink_pod_1359_update_ptz_tta_navi_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  1);
}

/**
 * @brief Get field longi from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline double ttalink_pod_1359_update_ptz_tta_navi_data_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  5);
}

/**
 * @brief Get field latit from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline double ttalink_pod_1359_update_ptz_tta_navi_data_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  13);
}

/**
 * @brief Get field altit from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field velN from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field velE from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Get field velD from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_velD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  33);
}

/**
 * @brief Get field pitch from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  37);
}

/**
 * @brief Get field roll from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  41);
}

/**
 * @brief Get field yaw from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  45);
}

/**
 * @brief Get field param1 from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  49);
}

/**
 * @brief Get field param2 from pod_1359_update_ptz_tta_navi_data message
 *
 * @return .
 */
static inline float ttalink_pod_1359_update_ptz_tta_navi_data_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  53);
}

/**
 * @brief Decode a pod_1359_update_ptz_tta_navi_data message into a struct
 *
 * @param msg The message to decode
 * @param pod_1359_update_ptz_tta_navi_data C-struct to decode the message contents into
 */
static inline void ttalink_pod_1359_update_ptz_tta_navi_data_decode(const ttalink_message_t* msg, ttalink_pod_1359_update_ptz_tta_navi_data_t* pod_1359_update_ptz_tta_navi_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1359_update_ptz_tta_navi_data->ack = ttalink_pod_1359_update_ptz_tta_navi_data_get_ack(msg);
    pod_1359_update_ptz_tta_navi_data->update_time = ttalink_pod_1359_update_ptz_tta_navi_data_get_update_time(msg);
    pod_1359_update_ptz_tta_navi_data->longi = ttalink_pod_1359_update_ptz_tta_navi_data_get_longi(msg);
    pod_1359_update_ptz_tta_navi_data->latit = ttalink_pod_1359_update_ptz_tta_navi_data_get_latit(msg);
    pod_1359_update_ptz_tta_navi_data->altit = ttalink_pod_1359_update_ptz_tta_navi_data_get_altit(msg);
    pod_1359_update_ptz_tta_navi_data->velN = ttalink_pod_1359_update_ptz_tta_navi_data_get_velN(msg);
    pod_1359_update_ptz_tta_navi_data->velE = ttalink_pod_1359_update_ptz_tta_navi_data_get_velE(msg);
    pod_1359_update_ptz_tta_navi_data->velD = ttalink_pod_1359_update_ptz_tta_navi_data_get_velD(msg);
    pod_1359_update_ptz_tta_navi_data->pitch = ttalink_pod_1359_update_ptz_tta_navi_data_get_pitch(msg);
    pod_1359_update_ptz_tta_navi_data->roll = ttalink_pod_1359_update_ptz_tta_navi_data_get_roll(msg);
    pod_1359_update_ptz_tta_navi_data->yaw = ttalink_pod_1359_update_ptz_tta_navi_data_get_yaw(msg);
    pod_1359_update_ptz_tta_navi_data->param1 = ttalink_pod_1359_update_ptz_tta_navi_data_get_param1(msg);
    pod_1359_update_ptz_tta_navi_data->param2 = ttalink_pod_1359_update_ptz_tta_navi_data_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN? msg->len : TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN;
        memset(pod_1359_update_ptz_tta_navi_data, 0, TTALINK_MSG_ID_POD_1359_UPDATE_PTZ_TTA_NAVI_DATA_LEN);
    memcpy(pod_1359_update_ptz_tta_navi_data, _TTA_PAYLOAD(msg), len);
#endif
}
