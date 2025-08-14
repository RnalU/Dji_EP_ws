#pragma once
// MESSAGE TC_9002_NAVI_DATA PACKING

#define TTALINK_MSG_ID_TC_9002_NAVI_DATA 9002

TTAPACKED(
typedef struct __ttalink_tc_9002_navi_data_t {
 uint32_t update_time; /*<  update_time*/
 int32_t longi; /*< .*/
 int32_t latit; /*< .*/
 float altit; /*< .*/
 float velN; /*< .*/
 float velE; /*< .*/
 float velD; /*< .*/
 float pitch; /*< .*/
 float roll; /*< .*/
 float yaw; /*< .*/
 float param1; /*< .*/
 float param2; /*< .*/
 float param3; /*<  预留参数*/
 float param4; /*<  预留参数*/
}) ttalink_tc_9002_navi_data_t;

#define TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN 56
#define TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN 56
#define TTALINK_MSG_ID_9002_LEN 56
#define TTALINK_MSG_ID_9002_MIN_LEN 56

#define TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC 57
#define TTALINK_MSG_ID_9002_CRC 57



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_TC_9002_NAVI_DATA { \
    9002, \
    "TC_9002_NAVI_DATA", \
    14, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9002_navi_data_t, update_time) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_tc_9002_navi_data_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_tc_9002_navi_data_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_tc_9002_navi_data_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_tc_9002_navi_data_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_tc_9002_navi_data_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_tc_9002_navi_data_t, velD) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_tc_9002_navi_data_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_tc_9002_navi_data_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_tc_9002_navi_data_t, yaw) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_tc_9002_navi_data_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_tc_9002_navi_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_tc_9002_navi_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_tc_9002_navi_data_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_TC_9002_NAVI_DATA { \
    "TC_9002_NAVI_DATA", \
    14, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_tc_9002_navi_data_t, update_time) }, \
         { "longi", NULL, TTALINK_TYPE_INT32_T, 0, 4, offsetof(ttalink_tc_9002_navi_data_t, longi) }, \
         { "latit", NULL, TTALINK_TYPE_INT32_T, 0, 8, offsetof(ttalink_tc_9002_navi_data_t, latit) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_tc_9002_navi_data_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_tc_9002_navi_data_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_tc_9002_navi_data_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_tc_9002_navi_data_t, velD) }, \
         { "pitch", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_tc_9002_navi_data_t, pitch) }, \
         { "roll", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_tc_9002_navi_data_t, roll) }, \
         { "yaw", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_tc_9002_navi_data_t, yaw) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_tc_9002_navi_data_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_tc_9002_navi_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_tc_9002_navi_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_tc_9002_navi_data_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_tc_9002_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_int32_t(buf, 8, latit);
    _tta_put_float(buf, 12, altit);
    _tta_put_float(buf, 16, velN);
    _tta_put_float(buf, 20, velE);
    _tta_put_float(buf, 24, velD);
    _tta_put_float(buf, 28, pitch);
    _tta_put_float(buf, 32, roll);
    _tta_put_float(buf, 36, yaw);
    _tta_put_float(buf, 40, param1);
    _tta_put_float(buf, 44, param2);
    _tta_put_float(buf, 48, param3);
    _tta_put_float(buf, 52, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN);
#else
    ttalink_tc_9002_navi_data_t packet;
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
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9002_NAVI_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9002_navi_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
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
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9002_navi_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4)
{
    return _ttalink_tc_9002_navi_data_pack(dst_addr, src_addr, msg,  update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a tc_9002_navi_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  update_time
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
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9002_navi_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4)
{
    return _ttalink_tc_9002_navi_data_pack(dst_addr, src_addr, msg,  update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_tc_9002_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t longi,int32_t latit,float altit,float velN,float velE,float velD,float pitch,float roll,float yaw,float param1,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_int32_t(buf, 8, latit);
    _tta_put_float(buf, 12, altit);
    _tta_put_float(buf, 16, velN);
    _tta_put_float(buf, 20, velE);
    _tta_put_float(buf, 24, velD);
    _tta_put_float(buf, 28, pitch);
    _tta_put_float(buf, 32, roll);
    _tta_put_float(buf, 36, yaw);
    _tta_put_float(buf, 40, param1);
    _tta_put_float(buf, 44, param2);
    _tta_put_float(buf, 48, param3);
    _tta_put_float(buf, 52, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN);
#else
    ttalink_tc_9002_navi_data_t packet;
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
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_TC_9002_NAVI_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC, nocrc);
}

/**
 * @brief Pack a tc_9002_navi_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
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
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9002_navi_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t longi,int32_t latit,float altit,float velN,float velE,float velD,float pitch,float roll,float yaw,float param1,float param2,float param3,float param4)
{
    return _ttalink_tc_9002_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a tc_9002_navi_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  update_time
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
 * @param param3  预留参数
 * @param param4  预留参数
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_tc_9002_navi_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,int32_t longi,int32_t latit,float altit,float velN,float velE,float velD,float pitch,float roll,float yaw,float param1,float param2,float param3,float param4)
{
    return _ttalink_tc_9002_navi_data_pack_chan(dst_addr, src_addr, chan, msg,  update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_tc_9002_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9002_navi_data_pack_nocrc(dst_addr, src_addr, msg, tc_9002_navi_data->update_time, tc_9002_navi_data->longi, tc_9002_navi_data->latit, tc_9002_navi_data->altit, tc_9002_navi_data->velN, tc_9002_navi_data->velE, tc_9002_navi_data->velD, tc_9002_navi_data->pitch, tc_9002_navi_data->roll, tc_9002_navi_data->yaw, tc_9002_navi_data->param1, tc_9002_navi_data->param2, tc_9002_navi_data->param3, tc_9002_navi_data->param4);
    }else{
        return ttalink_tc_9002_navi_data_pack(dst_addr, src_addr, msg, tc_9002_navi_data->update_time, tc_9002_navi_data->longi, tc_9002_navi_data->latit, tc_9002_navi_data->altit, tc_9002_navi_data->velN, tc_9002_navi_data->velE, tc_9002_navi_data->velD, tc_9002_navi_data->pitch, tc_9002_navi_data->roll, tc_9002_navi_data->yaw, tc_9002_navi_data->param1, tc_9002_navi_data->param2, tc_9002_navi_data->param3, tc_9002_navi_data->param4);
    }
    
}

/**
 * @brief Encode a tc_9002_navi_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9002_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9002_navi_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data)
{
    return _ttalink_tc_9002_navi_data_encode(dst_addr, src_addr, msg, tc_9002_navi_data, false);
}

/**
 * @brief Encode a tc_9002_navi_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param tc_9002_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9002_navi_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data)
{
    return _ttalink_tc_9002_navi_data_encode(dst_addr, src_addr, msg, tc_9002_navi_data, true);
}


static inline uint16_t _ttalink_tc_9002_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data, bool nocrc)
{
    if(nocrc){
        return ttalink_tc_9002_navi_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, tc_9002_navi_data->update_time, tc_9002_navi_data->longi, tc_9002_navi_data->latit, tc_9002_navi_data->altit, tc_9002_navi_data->velN, tc_9002_navi_data->velE, tc_9002_navi_data->velD, tc_9002_navi_data->pitch, tc_9002_navi_data->roll, tc_9002_navi_data->yaw, tc_9002_navi_data->param1, tc_9002_navi_data->param2, tc_9002_navi_data->param3, tc_9002_navi_data->param4);
    }else{
        return ttalink_tc_9002_navi_data_pack_chan(dst_addr, src_addr, chan, msg, tc_9002_navi_data->update_time, tc_9002_navi_data->longi, tc_9002_navi_data->latit, tc_9002_navi_data->altit, tc_9002_navi_data->velN, tc_9002_navi_data->velE, tc_9002_navi_data->velD, tc_9002_navi_data->pitch, tc_9002_navi_data->roll, tc_9002_navi_data->yaw, tc_9002_navi_data->param1, tc_9002_navi_data->param2, tc_9002_navi_data->param3, tc_9002_navi_data->param4);
    }
}

/**
 * @brief Encode a tc_9002_navi_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9002_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9002_navi_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data)
{
    return _ttalink_tc_9002_navi_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9002_navi_data, false);
}

/**
 * @brief Encode a tc_9002_navi_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param tc_9002_navi_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_tc_9002_navi_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data)
{
    return _ttalink_tc_9002_navi_data_encode_chan(dst_addr, src_addr, chan, msg, tc_9002_navi_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_tc_9002_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_int32_t(buf, 8, latit);
    _tta_put_float(buf, 12, altit);
    _tta_put_float(buf, 16, velN);
    _tta_put_float(buf, 20, velE);
    _tta_put_float(buf, 24, velD);
    _tta_put_float(buf, 28, pitch);
    _tta_put_float(buf, 32, roll);
    _tta_put_float(buf, 36, yaw);
    _tta_put_float(buf, 40, param1);
    _tta_put_float(buf, 44, param2);
    _tta_put_float(buf, 48, param3);
    _tta_put_float(buf, 52, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9002_NAVI_DATA, buf, TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC, nocrc);
#else
    ttalink_tc_9002_navi_data_t packet;
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
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9002_NAVI_DATA, (const char *)&packet, TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9002_navi_data message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
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
 * @param param3  预留参数
 * @param param4  预留参数
 */
static inline void ttalink_tc_9002_navi_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9002_navi_data_send(dst_addr, src_addr, chan, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, false);
}

/**
 * @brief Send a tc_9002_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  update_time
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
 * @param param3  预留参数
 * @param param4  预留参数
 */
static inline void ttalink_tc_9002_navi_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9002_navi_data_send(dst_addr, src_addr, chan, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, true);
}


static inline void _ttalink_tc_9002_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_tc_9002_navi_data_send_nocrc(dst_addr, src_addr, chan, tc_9002_navi_data->update_time, tc_9002_navi_data->longi, tc_9002_navi_data->latit, tc_9002_navi_data->altit, tc_9002_navi_data->velN, tc_9002_navi_data->velE, tc_9002_navi_data->velD, tc_9002_navi_data->pitch, tc_9002_navi_data->roll, tc_9002_navi_data->yaw, tc_9002_navi_data->param1, tc_9002_navi_data->param2, tc_9002_navi_data->param3, tc_9002_navi_data->param4);
    }else{
        ttalink_tc_9002_navi_data_send(dst_addr, src_addr, chan, tc_9002_navi_data->update_time, tc_9002_navi_data->longi, tc_9002_navi_data->latit, tc_9002_navi_data->altit, tc_9002_navi_data->velN, tc_9002_navi_data->velE, tc_9002_navi_data->velD, tc_9002_navi_data->pitch, tc_9002_navi_data->roll, tc_9002_navi_data->yaw, tc_9002_navi_data->param1, tc_9002_navi_data->param2, tc_9002_navi_data->param3, tc_9002_navi_data->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9002_NAVI_DATA, (const char *)tc_9002_navi_data, TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a tc_9002_navi_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9002_navi_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data)
{
    _ttalink_tc_9002_navi_data_send_struct(dst_addr, src_addr, chan, tc_9002_navi_data, false);
}

/**
 * @brief Send a tc_9002_navi_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_tc_9002_navi_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_tc_9002_navi_data_t* tc_9002_navi_data)
{
    _ttalink_tc_9002_navi_data_send_struct(dst_addr, src_addr, chan, tc_9002_navi_data, true);
}

#if TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_tc_9002_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_int32_t(buf, 4, longi);
    _tta_put_int32_t(buf, 8, latit);
    _tta_put_float(buf, 12, altit);
    _tta_put_float(buf, 16, velN);
    _tta_put_float(buf, 20, velE);
    _tta_put_float(buf, 24, velD);
    _tta_put_float(buf, 28, pitch);
    _tta_put_float(buf, 32, roll);
    _tta_put_float(buf, 36, yaw);
    _tta_put_float(buf, 40, param1);
    _tta_put_float(buf, 44, param2);
    _tta_put_float(buf, 48, param3);
    _tta_put_float(buf, 52, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9002_NAVI_DATA, buf, TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC, nocrc);
#else
    ttalink_tc_9002_navi_data_t *packet = (ttalink_tc_9002_navi_data_t *)msgbuf;
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
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_TC_9002_NAVI_DATA, (const char *)packet, TTALINK_MSG_ID_TC_9002_NAVI_DATA_MIN_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN, TTALINK_MSG_ID_TC_9002_NAVI_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_tc_9002_navi_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9002_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_tc_9002_navi_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, int32_t longi, int32_t latit, float altit, float velN, float velE, float velD, float pitch, float roll, float yaw, float param1, float param2, float param3, float param4)
{
    _ttalink_tc_9002_navi_data_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, longi, latit, altit, velN, velE, velD, pitch, roll, yaw, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE TC_9002_NAVI_DATA UNPACKING


/**
 * @brief Get field update_time from tc_9002_navi_data message
 *
 * @return  update_time
 */
static inline uint32_t ttalink_tc_9002_navi_data_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field longi from tc_9002_navi_data message
 *
 * @return .
 */
static inline int32_t ttalink_tc_9002_navi_data_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field latit from tc_9002_navi_data message
 *
 * @return .
 */
static inline int32_t ttalink_tc_9002_navi_data_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altit from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field velN from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field velE from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field velD from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_velD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitch from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field roll from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field yaw from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field param1 from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field param2 from tc_9002_navi_data message
 *
 * @return .
 */
static inline float ttalink_tc_9002_navi_data_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field param3 from tc_9002_navi_data message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9002_navi_data_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field param4 from tc_9002_navi_data message
 *
 * @return  预留参数
 */
static inline float ttalink_tc_9002_navi_data_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  52);
}

/**
 * @brief Decode a tc_9002_navi_data message into a struct
 *
 * @param msg The message to decode
 * @param tc_9002_navi_data C-struct to decode the message contents into
 */
static inline void ttalink_tc_9002_navi_data_decode(const ttalink_message_t* msg, ttalink_tc_9002_navi_data_t* tc_9002_navi_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    tc_9002_navi_data->update_time = ttalink_tc_9002_navi_data_get_update_time(msg);
    tc_9002_navi_data->longi = ttalink_tc_9002_navi_data_get_longi(msg);
    tc_9002_navi_data->latit = ttalink_tc_9002_navi_data_get_latit(msg);
    tc_9002_navi_data->altit = ttalink_tc_9002_navi_data_get_altit(msg);
    tc_9002_navi_data->velN = ttalink_tc_9002_navi_data_get_velN(msg);
    tc_9002_navi_data->velE = ttalink_tc_9002_navi_data_get_velE(msg);
    tc_9002_navi_data->velD = ttalink_tc_9002_navi_data_get_velD(msg);
    tc_9002_navi_data->pitch = ttalink_tc_9002_navi_data_get_pitch(msg);
    tc_9002_navi_data->roll = ttalink_tc_9002_navi_data_get_roll(msg);
    tc_9002_navi_data->yaw = ttalink_tc_9002_navi_data_get_yaw(msg);
    tc_9002_navi_data->param1 = ttalink_tc_9002_navi_data_get_param1(msg);
    tc_9002_navi_data->param2 = ttalink_tc_9002_navi_data_get_param2(msg);
    tc_9002_navi_data->param3 = ttalink_tc_9002_navi_data_get_param3(msg);
    tc_9002_navi_data->param4 = ttalink_tc_9002_navi_data_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN? msg->len : TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN;
        memset(tc_9002_navi_data, 0, TTALINK_MSG_ID_TC_9002_NAVI_DATA_LEN);
    memcpy(tc_9002_navi_data, _TTA_PAYLOAD(msg), len);
#endif
}
