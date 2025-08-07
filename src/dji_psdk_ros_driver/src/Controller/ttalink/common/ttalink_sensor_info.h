#pragma once
// MESSAGE SENSOR_INFO PACKING

#define TTALINK_MSG_ID_SENSOR_INFO 2129

TTAPACKED(
typedef struct __ttalink_sensor_info_t {
 float accX; /*<  .*/
 float accY; /*<  .*/
 float accZ; /*<  .*/
 float accVar; /*<  .*/
 float gyroX; /*<  .*/
 float gyroY; /*<  .*/
 float gyroZ; /*<  .*/
 float gyroVar; /*<  .*/
 float baro; /*<  .*/
 float baroH; /*<  .*/
 float magX; /*<  .*/
 float magY; /*<  .*/
 float magZ; /*<  .*/
 float magRoot; /*<  .*/
 int32_t gps_latit; /*<  .*/
 int32_t gps_lonig; /*<  .*/
 float gps_altit; /*<  .*/
 int32_t rtk_latit; /*<  .*/
 int32_t rtk_lonig; /*<  .*/
 float rtk_altit; /*<  .*/
 float posX; /*<  水平位置X.*/
 float posY; /*<  水平位置Y.*/
 float posZ; /*<  水平位置Z 向下为正.*/
 float velX; /*<  机体速度X.*/
 float velY; /*<  机体速度Y.*/
 float velZ; /*<  机体速度Z 向下为正.*/
 float param[4]; /*<  .*/
}) ttalink_sensor_info_t;

#define TTALINK_MSG_ID_SENSOR_INFO_LEN 120
#define TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN 120
#define TTALINK_MSG_ID_2129_LEN 120
#define TTALINK_MSG_ID_2129_MIN_LEN 120

#define TTALINK_MSG_ID_SENSOR_INFO_CRC 231
#define TTALINK_MSG_ID_2129_CRC 231

#define TTALINK_MSG_SENSOR_INFO_FIELD_PARAM_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SENSOR_INFO { \
    2129, \
    "SENSOR_INFO", \
    27, \
    {  { "accX", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_sensor_info_t, accX) }, \
         { "accY", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_sensor_info_t, accY) }, \
         { "accZ", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_sensor_info_t, accZ) }, \
         { "accVar", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_sensor_info_t, accVar) }, \
         { "gyroX", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_sensor_info_t, gyroX) }, \
         { "gyroY", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_sensor_info_t, gyroY) }, \
         { "gyroZ", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_sensor_info_t, gyroZ) }, \
         { "gyroVar", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_sensor_info_t, gyroVar) }, \
         { "baro", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_sensor_info_t, baro) }, \
         { "baroH", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_sensor_info_t, baroH) }, \
         { "magX", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_sensor_info_t, magX) }, \
         { "magY", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_sensor_info_t, magY) }, \
         { "magZ", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_sensor_info_t, magZ) }, \
         { "magRoot", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_sensor_info_t, magRoot) }, \
         { "gps_latit", NULL, TTALINK_TYPE_INT32_T, 0, 56, offsetof(ttalink_sensor_info_t, gps_latit) }, \
         { "gps_lonig", NULL, TTALINK_TYPE_INT32_T, 0, 60, offsetof(ttalink_sensor_info_t, gps_lonig) }, \
         { "gps_altit", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_sensor_info_t, gps_altit) }, \
         { "rtk_latit", NULL, TTALINK_TYPE_INT32_T, 0, 68, offsetof(ttalink_sensor_info_t, rtk_latit) }, \
         { "rtk_lonig", NULL, TTALINK_TYPE_INT32_T, 0, 72, offsetof(ttalink_sensor_info_t, rtk_lonig) }, \
         { "rtk_altit", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_sensor_info_t, rtk_altit) }, \
         { "posX", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_sensor_info_t, posX) }, \
         { "posY", NULL, TTALINK_TYPE_FLOAT, 0, 84, offsetof(ttalink_sensor_info_t, posY) }, \
         { "posZ", NULL, TTALINK_TYPE_FLOAT, 0, 88, offsetof(ttalink_sensor_info_t, posZ) }, \
         { "velX", NULL, TTALINK_TYPE_FLOAT, 0, 92, offsetof(ttalink_sensor_info_t, velX) }, \
         { "velY", NULL, TTALINK_TYPE_FLOAT, 0, 96, offsetof(ttalink_sensor_info_t, velY) }, \
         { "velZ", NULL, TTALINK_TYPE_FLOAT, 0, 100, offsetof(ttalink_sensor_info_t, velZ) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 104, offsetof(ttalink_sensor_info_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SENSOR_INFO { \
    "SENSOR_INFO", \
    27, \
    {  { "accX", NULL, TTALINK_TYPE_FLOAT, 0, 0, offsetof(ttalink_sensor_info_t, accX) }, \
         { "accY", NULL, TTALINK_TYPE_FLOAT, 0, 4, offsetof(ttalink_sensor_info_t, accY) }, \
         { "accZ", NULL, TTALINK_TYPE_FLOAT, 0, 8, offsetof(ttalink_sensor_info_t, accZ) }, \
         { "accVar", NULL, TTALINK_TYPE_FLOAT, 0, 12, offsetof(ttalink_sensor_info_t, accVar) }, \
         { "gyroX", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_sensor_info_t, gyroX) }, \
         { "gyroY", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_sensor_info_t, gyroY) }, \
         { "gyroZ", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_sensor_info_t, gyroZ) }, \
         { "gyroVar", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_sensor_info_t, gyroVar) }, \
         { "baro", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_sensor_info_t, baro) }, \
         { "baroH", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_sensor_info_t, baroH) }, \
         { "magX", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_sensor_info_t, magX) }, \
         { "magY", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_sensor_info_t, magY) }, \
         { "magZ", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_sensor_info_t, magZ) }, \
         { "magRoot", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_sensor_info_t, magRoot) }, \
         { "gps_latit", NULL, TTALINK_TYPE_INT32_T, 0, 56, offsetof(ttalink_sensor_info_t, gps_latit) }, \
         { "gps_lonig", NULL, TTALINK_TYPE_INT32_T, 0, 60, offsetof(ttalink_sensor_info_t, gps_lonig) }, \
         { "gps_altit", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_sensor_info_t, gps_altit) }, \
         { "rtk_latit", NULL, TTALINK_TYPE_INT32_T, 0, 68, offsetof(ttalink_sensor_info_t, rtk_latit) }, \
         { "rtk_lonig", NULL, TTALINK_TYPE_INT32_T, 0, 72, offsetof(ttalink_sensor_info_t, rtk_lonig) }, \
         { "rtk_altit", NULL, TTALINK_TYPE_FLOAT, 0, 76, offsetof(ttalink_sensor_info_t, rtk_altit) }, \
         { "posX", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_sensor_info_t, posX) }, \
         { "posY", NULL, TTALINK_TYPE_FLOAT, 0, 84, offsetof(ttalink_sensor_info_t, posY) }, \
         { "posZ", NULL, TTALINK_TYPE_FLOAT, 0, 88, offsetof(ttalink_sensor_info_t, posZ) }, \
         { "velX", NULL, TTALINK_TYPE_FLOAT, 0, 92, offsetof(ttalink_sensor_info_t, velX) }, \
         { "velY", NULL, TTALINK_TYPE_FLOAT, 0, 96, offsetof(ttalink_sensor_info_t, velY) }, \
         { "velZ", NULL, TTALINK_TYPE_FLOAT, 0, 100, offsetof(ttalink_sensor_info_t, velZ) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 104, offsetof(ttalink_sensor_info_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sensor_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_INFO_LEN];
    _tta_put_float(buf, 0, accX);
    _tta_put_float(buf, 4, accY);
    _tta_put_float(buf, 8, accZ);
    _tta_put_float(buf, 12, accVar);
    _tta_put_float(buf, 16, gyroX);
    _tta_put_float(buf, 20, gyroY);
    _tta_put_float(buf, 24, gyroZ);
    _tta_put_float(buf, 28, gyroVar);
    _tta_put_float(buf, 32, baro);
    _tta_put_float(buf, 36, baroH);
    _tta_put_float(buf, 40, magX);
    _tta_put_float(buf, 44, magY);
    _tta_put_float(buf, 48, magZ);
    _tta_put_float(buf, 52, magRoot);
    _tta_put_int32_t(buf, 56, gps_latit);
    _tta_put_int32_t(buf, 60, gps_lonig);
    _tta_put_float(buf, 64, gps_altit);
    _tta_put_int32_t(buf, 68, rtk_latit);
    _tta_put_int32_t(buf, 72, rtk_lonig);
    _tta_put_float(buf, 76, rtk_altit);
    _tta_put_float(buf, 80, posX);
    _tta_put_float(buf, 84, posY);
    _tta_put_float(buf, 88, posZ);
    _tta_put_float(buf, 92, velX);
    _tta_put_float(buf, 96, velY);
    _tta_put_float(buf, 100, velZ);
    _tta_put_float_array(buf, 104, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_INFO_LEN);
#else
    ttalink_sensor_info_t packet;
    packet.accX = accX;
    packet.accY = accY;
    packet.accZ = accZ;
    packet.accVar = accVar;
    packet.gyroX = gyroX;
    packet.gyroY = gyroY;
    packet.gyroZ = gyroZ;
    packet.gyroVar = gyroVar;
    packet.baro = baro;
    packet.baroH = baroH;
    packet.magX = magX;
    packet.magY = magY;
    packet.magZ = magZ;
    packet.magRoot = magRoot;
    packet.gps_latit = gps_latit;
    packet.gps_lonig = gps_lonig;
    packet.gps_altit = gps_altit;
    packet.rtk_latit = rtk_latit;
    packet.rtk_lonig = rtk_lonig;
    packet.rtk_altit = rtk_altit;
    packet.posX = posX;
    packet.posY = posY;
    packet.posZ = posZ;
    packet.velX = velX;
    packet.velY = velY;
    packet.velZ = velZ;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN, TTALINK_MSG_ID_SENSOR_INFO_LEN, TTALINK_MSG_ID_SENSOR_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sensor_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param accX  .
 * @param accY  .
 * @param accZ  .
 * @param accVar  .
 * @param gyroX  .
 * @param gyroY  .
 * @param gyroZ  .
 * @param gyroVar  .
 * @param baro  .
 * @param baroH  .
 * @param magX  .
 * @param magY  .
 * @param magZ  .
 * @param magRoot  .
 * @param gps_latit  .
 * @param gps_lonig  .
 * @param gps_altit  .
 * @param rtk_latit  .
 * @param rtk_lonig  .
 * @param rtk_altit  .
 * @param posX  水平位置X.
 * @param posY  水平位置Y.
 * @param posZ  水平位置Z 向下为正.
 * @param velX  机体速度X.
 * @param velY  机体速度Y.
 * @param velZ  机体速度Z 向下为正.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param)
{
    return _ttalink_sensor_info_pack(dst_addr, src_addr, msg,  accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, false);
}

/**
 * @brief Pack a sensor_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param accX  .
 * @param accY  .
 * @param accZ  .
 * @param accVar  .
 * @param gyroX  .
 * @param gyroY  .
 * @param gyroZ  .
 * @param gyroVar  .
 * @param baro  .
 * @param baroH  .
 * @param magX  .
 * @param magY  .
 * @param magZ  .
 * @param magRoot  .
 * @param gps_latit  .
 * @param gps_lonig  .
 * @param gps_altit  .
 * @param rtk_latit  .
 * @param rtk_lonig  .
 * @param rtk_altit  .
 * @param posX  水平位置X.
 * @param posY  水平位置Y.
 * @param posZ  水平位置Z 向下为正.
 * @param velX  机体速度X.
 * @param velY  机体速度Y.
 * @param velZ  机体速度Z 向下为正.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param)
{
    return _ttalink_sensor_info_pack(dst_addr, src_addr, msg,  accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, true);
}


static inline uint16_t _ttalink_sensor_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float accX,float accY,float accZ,float accVar,float gyroX,float gyroY,float gyroZ,float gyroVar,float baro,float baroH,float magX,float magY,float magZ,float magRoot,int32_t gps_latit,int32_t gps_lonig,float gps_altit,int32_t rtk_latit,int32_t rtk_lonig,float rtk_altit,float posX,float posY,float posZ,float velX,float velY,float velZ,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_INFO_LEN];
    _tta_put_float(buf, 0, accX);
    _tta_put_float(buf, 4, accY);
    _tta_put_float(buf, 8, accZ);
    _tta_put_float(buf, 12, accVar);
    _tta_put_float(buf, 16, gyroX);
    _tta_put_float(buf, 20, gyroY);
    _tta_put_float(buf, 24, gyroZ);
    _tta_put_float(buf, 28, gyroVar);
    _tta_put_float(buf, 32, baro);
    _tta_put_float(buf, 36, baroH);
    _tta_put_float(buf, 40, magX);
    _tta_put_float(buf, 44, magY);
    _tta_put_float(buf, 48, magZ);
    _tta_put_float(buf, 52, magRoot);
    _tta_put_int32_t(buf, 56, gps_latit);
    _tta_put_int32_t(buf, 60, gps_lonig);
    _tta_put_float(buf, 64, gps_altit);
    _tta_put_int32_t(buf, 68, rtk_latit);
    _tta_put_int32_t(buf, 72, rtk_lonig);
    _tta_put_float(buf, 76, rtk_altit);
    _tta_put_float(buf, 80, posX);
    _tta_put_float(buf, 84, posY);
    _tta_put_float(buf, 88, posZ);
    _tta_put_float(buf, 92, velX);
    _tta_put_float(buf, 96, velY);
    _tta_put_float(buf, 100, velZ);
    _tta_put_float_array(buf, 104, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SENSOR_INFO_LEN);
#else
    ttalink_sensor_info_t packet;
    packet.accX = accX;
    packet.accY = accY;
    packet.accZ = accZ;
    packet.accVar = accVar;
    packet.gyroX = gyroX;
    packet.gyroY = gyroY;
    packet.gyroZ = gyroZ;
    packet.gyroVar = gyroVar;
    packet.baro = baro;
    packet.baroH = baroH;
    packet.magX = magX;
    packet.magY = magY;
    packet.magZ = magZ;
    packet.magRoot = magRoot;
    packet.gps_latit = gps_latit;
    packet.gps_lonig = gps_lonig;
    packet.gps_altit = gps_altit;
    packet.rtk_latit = rtk_latit;
    packet.rtk_lonig = rtk_lonig;
    packet.rtk_altit = rtk_altit;
    packet.posX = posX;
    packet.posY = posY;
    packet.posZ = posZ;
    packet.velX = velX;
    packet.velY = velY;
    packet.velZ = velZ;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SENSOR_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SENSOR_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN, TTALINK_MSG_ID_SENSOR_INFO_LEN, TTALINK_MSG_ID_SENSOR_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sensor_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param accX  .
 * @param accY  .
 * @param accZ  .
 * @param accVar  .
 * @param gyroX  .
 * @param gyroY  .
 * @param gyroZ  .
 * @param gyroVar  .
 * @param baro  .
 * @param baroH  .
 * @param magX  .
 * @param magY  .
 * @param magZ  .
 * @param magRoot  .
 * @param gps_latit  .
 * @param gps_lonig  .
 * @param gps_altit  .
 * @param rtk_latit  .
 * @param rtk_lonig  .
 * @param rtk_altit  .
 * @param posX  水平位置X.
 * @param posY  水平位置Y.
 * @param posZ  水平位置Z 向下为正.
 * @param velX  机体速度X.
 * @param velY  机体速度Y.
 * @param velZ  机体速度Z 向下为正.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float accX,float accY,float accZ,float accVar,float gyroX,float gyroY,float gyroZ,float gyroVar,float baro,float baroH,float magX,float magY,float magZ,float magRoot,int32_t gps_latit,int32_t gps_lonig,float gps_altit,int32_t rtk_latit,int32_t rtk_lonig,float rtk_altit,float posX,float posY,float posZ,float velX,float velY,float velZ,const float *param)
{
    return _ttalink_sensor_info_pack_chan(dst_addr, src_addr, chan, msg,  accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, false);
}

/**
 * @brief Pack a sensor_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param accX  .
 * @param accY  .
 * @param accZ  .
 * @param accVar  .
 * @param gyroX  .
 * @param gyroY  .
 * @param gyroZ  .
 * @param gyroVar  .
 * @param baro  .
 * @param baroH  .
 * @param magX  .
 * @param magY  .
 * @param magZ  .
 * @param magRoot  .
 * @param gps_latit  .
 * @param gps_lonig  .
 * @param gps_altit  .
 * @param rtk_latit  .
 * @param rtk_lonig  .
 * @param rtk_altit  .
 * @param posX  水平位置X.
 * @param posY  水平位置Y.
 * @param posZ  水平位置Z 向下为正.
 * @param velX  机体速度X.
 * @param velY  机体速度Y.
 * @param velZ  机体速度Z 向下为正.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sensor_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   float accX,float accY,float accZ,float accVar,float gyroX,float gyroY,float gyroZ,float gyroVar,float baro,float baroH,float magX,float magY,float magZ,float magRoot,int32_t gps_latit,int32_t gps_lonig,float gps_altit,int32_t rtk_latit,int32_t rtk_lonig,float rtk_altit,float posX,float posY,float posZ,float velX,float velY,float velZ,const float *param)
{
    return _ttalink_sensor_info_pack_chan(dst_addr, src_addr, chan, msg,  accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, true);
}


static inline uint16_t _ttalink_sensor_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_info_t* sensor_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_info_pack_nocrc(dst_addr, src_addr, msg, sensor_info->accX, sensor_info->accY, sensor_info->accZ, sensor_info->accVar, sensor_info->gyroX, sensor_info->gyroY, sensor_info->gyroZ, sensor_info->gyroVar, sensor_info->baro, sensor_info->baroH, sensor_info->magX, sensor_info->magY, sensor_info->magZ, sensor_info->magRoot, sensor_info->gps_latit, sensor_info->gps_lonig, sensor_info->gps_altit, sensor_info->rtk_latit, sensor_info->rtk_lonig, sensor_info->rtk_altit, sensor_info->posX, sensor_info->posY, sensor_info->posZ, sensor_info->velX, sensor_info->velY, sensor_info->velZ, sensor_info->param);
    }else{
        return ttalink_sensor_info_pack(dst_addr, src_addr, msg, sensor_info->accX, sensor_info->accY, sensor_info->accZ, sensor_info->accVar, sensor_info->gyroX, sensor_info->gyroY, sensor_info->gyroZ, sensor_info->gyroVar, sensor_info->baro, sensor_info->baroH, sensor_info->magX, sensor_info->magY, sensor_info->magZ, sensor_info->magRoot, sensor_info->gps_latit, sensor_info->gps_lonig, sensor_info->gps_altit, sensor_info->rtk_latit, sensor_info->rtk_lonig, sensor_info->rtk_altit, sensor_info->posX, sensor_info->posY, sensor_info->posZ, sensor_info->velX, sensor_info->velY, sensor_info->velZ, sensor_info->param);
    }
    
}

/**
 * @brief Encode a sensor_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_info_t* sensor_info)
{
    return _ttalink_sensor_info_encode(dst_addr, src_addr, msg, sensor_info, false);
}

/**
 * @brief Encode a sensor_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sensor_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sensor_info_t* sensor_info)
{
    return _ttalink_sensor_info_encode(dst_addr, src_addr, msg, sensor_info, true);
}


static inline uint16_t _ttalink_sensor_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_info_t* sensor_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sensor_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sensor_info->accX, sensor_info->accY, sensor_info->accZ, sensor_info->accVar, sensor_info->gyroX, sensor_info->gyroY, sensor_info->gyroZ, sensor_info->gyroVar, sensor_info->baro, sensor_info->baroH, sensor_info->magX, sensor_info->magY, sensor_info->magZ, sensor_info->magRoot, sensor_info->gps_latit, sensor_info->gps_lonig, sensor_info->gps_altit, sensor_info->rtk_latit, sensor_info->rtk_lonig, sensor_info->rtk_altit, sensor_info->posX, sensor_info->posY, sensor_info->posZ, sensor_info->velX, sensor_info->velY, sensor_info->velZ, sensor_info->param);
    }else{
        return ttalink_sensor_info_pack_chan(dst_addr, src_addr, chan, msg, sensor_info->accX, sensor_info->accY, sensor_info->accZ, sensor_info->accVar, sensor_info->gyroX, sensor_info->gyroY, sensor_info->gyroZ, sensor_info->gyroVar, sensor_info->baro, sensor_info->baroH, sensor_info->magX, sensor_info->magY, sensor_info->magZ, sensor_info->magRoot, sensor_info->gps_latit, sensor_info->gps_lonig, sensor_info->gps_altit, sensor_info->rtk_latit, sensor_info->rtk_lonig, sensor_info->rtk_altit, sensor_info->posX, sensor_info->posY, sensor_info->posZ, sensor_info->velX, sensor_info->velY, sensor_info->velZ, sensor_info->param);
    }
}

/**
 * @brief Encode a sensor_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_info_t* sensor_info)
{
    return _ttalink_sensor_info_encode_chan(dst_addr, src_addr, chan, msg, sensor_info, false);
}

/**
 * @brief Encode a sensor_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sensor_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sensor_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sensor_info_t* sensor_info)
{
    return _ttalink_sensor_info_encode_chan(dst_addr, src_addr, chan, msg, sensor_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sensor_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SENSOR_INFO_LEN];
    _tta_put_float(buf, 0, accX);
    _tta_put_float(buf, 4, accY);
    _tta_put_float(buf, 8, accZ);
    _tta_put_float(buf, 12, accVar);
    _tta_put_float(buf, 16, gyroX);
    _tta_put_float(buf, 20, gyroY);
    _tta_put_float(buf, 24, gyroZ);
    _tta_put_float(buf, 28, gyroVar);
    _tta_put_float(buf, 32, baro);
    _tta_put_float(buf, 36, baroH);
    _tta_put_float(buf, 40, magX);
    _tta_put_float(buf, 44, magY);
    _tta_put_float(buf, 48, magZ);
    _tta_put_float(buf, 52, magRoot);
    _tta_put_int32_t(buf, 56, gps_latit);
    _tta_put_int32_t(buf, 60, gps_lonig);
    _tta_put_float(buf, 64, gps_altit);
    _tta_put_int32_t(buf, 68, rtk_latit);
    _tta_put_int32_t(buf, 72, rtk_lonig);
    _tta_put_float(buf, 76, rtk_altit);
    _tta_put_float(buf, 80, posX);
    _tta_put_float(buf, 84, posY);
    _tta_put_float(buf, 88, posZ);
    _tta_put_float(buf, 92, velX);
    _tta_put_float(buf, 96, velY);
    _tta_put_float(buf, 100, velZ);
    _tta_put_float_array(buf, 104, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_INFO, buf, TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN, TTALINK_MSG_ID_SENSOR_INFO_LEN, TTALINK_MSG_ID_SENSOR_INFO_CRC, nocrc);
#else
    ttalink_sensor_info_t packet;
    packet.accX = accX;
    packet.accY = accY;
    packet.accZ = accZ;
    packet.accVar = accVar;
    packet.gyroX = gyroX;
    packet.gyroY = gyroY;
    packet.gyroZ = gyroZ;
    packet.gyroVar = gyroVar;
    packet.baro = baro;
    packet.baroH = baroH;
    packet.magX = magX;
    packet.magY = magY;
    packet.magZ = magZ;
    packet.magRoot = magRoot;
    packet.gps_latit = gps_latit;
    packet.gps_lonig = gps_lonig;
    packet.gps_altit = gps_altit;
    packet.rtk_latit = rtk_latit;
    packet.rtk_lonig = rtk_lonig;
    packet.rtk_altit = rtk_altit;
    packet.posX = posX;
    packet.posY = posY;
    packet.posZ = posZ;
    packet.velX = velX;
    packet.velY = velY;
    packet.velZ = velZ;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_INFO, (const char *)&packet, TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN, TTALINK_MSG_ID_SENSOR_INFO_LEN, TTALINK_MSG_ID_SENSOR_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_info message
 * @param chan TTAlink channel to send the message
 *
 * @param accX  .
 * @param accY  .
 * @param accZ  .
 * @param accVar  .
 * @param gyroX  .
 * @param gyroY  .
 * @param gyroZ  .
 * @param gyroVar  .
 * @param baro  .
 * @param baroH  .
 * @param magX  .
 * @param magY  .
 * @param magZ  .
 * @param magRoot  .
 * @param gps_latit  .
 * @param gps_lonig  .
 * @param gps_altit  .
 * @param rtk_latit  .
 * @param rtk_lonig  .
 * @param rtk_altit  .
 * @param posX  水平位置X.
 * @param posY  水平位置Y.
 * @param posZ  水平位置Z 向下为正.
 * @param velX  机体速度X.
 * @param velY  机体速度Y.
 * @param velZ  机体速度Z 向下为正.
 * @param param  .
 */
static inline void ttalink_sensor_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param)
{
    _ttalink_sensor_info_send(dst_addr, src_addr, chan, accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, false);
}

/**
 * @brief Send a sensor_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param accX  .
 * @param accY  .
 * @param accZ  .
 * @param accVar  .
 * @param gyroX  .
 * @param gyroY  .
 * @param gyroZ  .
 * @param gyroVar  .
 * @param baro  .
 * @param baroH  .
 * @param magX  .
 * @param magY  .
 * @param magZ  .
 * @param magRoot  .
 * @param gps_latit  .
 * @param gps_lonig  .
 * @param gps_altit  .
 * @param rtk_latit  .
 * @param rtk_lonig  .
 * @param rtk_altit  .
 * @param posX  水平位置X.
 * @param posY  水平位置Y.
 * @param posZ  水平位置Z 向下为正.
 * @param velX  机体速度X.
 * @param velY  机体速度Y.
 * @param velZ  机体速度Z 向下为正.
 * @param param  .
 */
static inline void ttalink_sensor_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param)
{
    _ttalink_sensor_info_send(dst_addr, src_addr, chan, accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, true);
}


static inline void _ttalink_sensor_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_info_t* sensor_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sensor_info_send_nocrc(dst_addr, src_addr, chan, sensor_info->accX, sensor_info->accY, sensor_info->accZ, sensor_info->accVar, sensor_info->gyroX, sensor_info->gyroY, sensor_info->gyroZ, sensor_info->gyroVar, sensor_info->baro, sensor_info->baroH, sensor_info->magX, sensor_info->magY, sensor_info->magZ, sensor_info->magRoot, sensor_info->gps_latit, sensor_info->gps_lonig, sensor_info->gps_altit, sensor_info->rtk_latit, sensor_info->rtk_lonig, sensor_info->rtk_altit, sensor_info->posX, sensor_info->posY, sensor_info->posZ, sensor_info->velX, sensor_info->velY, sensor_info->velZ, sensor_info->param);
    }else{
        ttalink_sensor_info_send(dst_addr, src_addr, chan, sensor_info->accX, sensor_info->accY, sensor_info->accZ, sensor_info->accVar, sensor_info->gyroX, sensor_info->gyroY, sensor_info->gyroZ, sensor_info->gyroVar, sensor_info->baro, sensor_info->baroH, sensor_info->magX, sensor_info->magY, sensor_info->magZ, sensor_info->magRoot, sensor_info->gps_latit, sensor_info->gps_lonig, sensor_info->gps_altit, sensor_info->rtk_latit, sensor_info->rtk_lonig, sensor_info->rtk_altit, sensor_info->posX, sensor_info->posY, sensor_info->posZ, sensor_info->velX, sensor_info->velY, sensor_info->velZ, sensor_info->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_INFO, (const char *)sensor_info, TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN, TTALINK_MSG_ID_SENSOR_INFO_LEN, TTALINK_MSG_ID_SENSOR_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sensor_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_info_t* sensor_info)
{
    _ttalink_sensor_info_send_struct(dst_addr, src_addr, chan, sensor_info, false);
}

/**
 * @brief Send a sensor_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sensor_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sensor_info_t* sensor_info)
{
    _ttalink_sensor_info_send_struct(dst_addr, src_addr, chan, sensor_info, true);
}

#if TTALINK_MSG_ID_SENSOR_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sensor_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_float(buf, 0, accX);
    _tta_put_float(buf, 4, accY);
    _tta_put_float(buf, 8, accZ);
    _tta_put_float(buf, 12, accVar);
    _tta_put_float(buf, 16, gyroX);
    _tta_put_float(buf, 20, gyroY);
    _tta_put_float(buf, 24, gyroZ);
    _tta_put_float(buf, 28, gyroVar);
    _tta_put_float(buf, 32, baro);
    _tta_put_float(buf, 36, baroH);
    _tta_put_float(buf, 40, magX);
    _tta_put_float(buf, 44, magY);
    _tta_put_float(buf, 48, magZ);
    _tta_put_float(buf, 52, magRoot);
    _tta_put_int32_t(buf, 56, gps_latit);
    _tta_put_int32_t(buf, 60, gps_lonig);
    _tta_put_float(buf, 64, gps_altit);
    _tta_put_int32_t(buf, 68, rtk_latit);
    _tta_put_int32_t(buf, 72, rtk_lonig);
    _tta_put_float(buf, 76, rtk_altit);
    _tta_put_float(buf, 80, posX);
    _tta_put_float(buf, 84, posY);
    _tta_put_float(buf, 88, posZ);
    _tta_put_float(buf, 92, velX);
    _tta_put_float(buf, 96, velY);
    _tta_put_float(buf, 100, velZ);
    _tta_put_float_array(buf, 104, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_INFO, buf, TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN, TTALINK_MSG_ID_SENSOR_INFO_LEN, TTALINK_MSG_ID_SENSOR_INFO_CRC, nocrc);
#else
    ttalink_sensor_info_t *packet = (ttalink_sensor_info_t *)msgbuf;
    packet->accX = accX;
    packet->accY = accY;
    packet->accZ = accZ;
    packet->accVar = accVar;
    packet->gyroX = gyroX;
    packet->gyroY = gyroY;
    packet->gyroZ = gyroZ;
    packet->gyroVar = gyroVar;
    packet->baro = baro;
    packet->baroH = baroH;
    packet->magX = magX;
    packet->magY = magY;
    packet->magZ = magZ;
    packet->magRoot = magRoot;
    packet->gps_latit = gps_latit;
    packet->gps_lonig = gps_lonig;
    packet->gps_altit = gps_altit;
    packet->rtk_latit = rtk_latit;
    packet->rtk_lonig = rtk_lonig;
    packet->rtk_altit = rtk_altit;
    packet->posX = posX;
    packet->posY = posY;
    packet->posZ = posZ;
    packet->velX = velX;
    packet->velY = velY;
    packet->velZ = velZ;
    tta_array_memcpy(packet->param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SENSOR_INFO, (const char *)packet, TTALINK_MSG_ID_SENSOR_INFO_MIN_LEN, TTALINK_MSG_ID_SENSOR_INFO_LEN, TTALINK_MSG_ID_SENSOR_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sensor_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param)
{
    _ttalink_sensor_info_send_buf(dst_addr, src_addr, msgbuf, chan, accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sensor_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  float accX, float accY, float accZ, float accVar, float gyroX, float gyroY, float gyroZ, float gyroVar, float baro, float baroH, float magX, float magY, float magZ, float magRoot, int32_t gps_latit, int32_t gps_lonig, float gps_altit, int32_t rtk_latit, int32_t rtk_lonig, float rtk_altit, float posX, float posY, float posZ, float velX, float velY, float velZ, const float *param)
{
    _ttalink_sensor_info_send_buf(dst_addr, src_addr, msgbuf, chan, accX, accY, accZ, accVar, gyroX, gyroY, gyroZ, gyroVar, baro, baroH, magX, magY, magZ, magRoot, gps_latit, gps_lonig, gps_altit, rtk_latit, rtk_lonig, rtk_altit, posX, posY, posZ, velX, velY, velZ, param, true);
}
#endif

#endif

// MESSAGE SENSOR_INFO UNPACKING


/**
 * @brief Get field accX from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_accX(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  0);
}

/**
 * @brief Get field accY from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_accY(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  4);
}

/**
 * @brief Get field accZ from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_accZ(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  8);
}

/**
 * @brief Get field accVar from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_accVar(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyroX from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_gyroX(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyroY from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_gyroY(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyroZ from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_gyroZ(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field gyroVar from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_gyroVar(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field baro from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_baro(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field baroH from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_baroH(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field magX from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_magX(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field magY from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_magY(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field magZ from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_magZ(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field magRoot from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_magRoot(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  52);
}

/**
 * @brief Get field gps_latit from sensor_info message
 *
 * @return  .
 */
static inline int32_t ttalink_sensor_info_get_gps_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  56);
}

/**
 * @brief Get field gps_lonig from sensor_info message
 *
 * @return  .
 */
static inline int32_t ttalink_sensor_info_get_gps_lonig(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  60);
}

/**
 * @brief Get field gps_altit from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_gps_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  64);
}

/**
 * @brief Get field rtk_latit from sensor_info message
 *
 * @return  .
 */
static inline int32_t ttalink_sensor_info_get_rtk_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  68);
}

/**
 * @brief Get field rtk_lonig from sensor_info message
 *
 * @return  .
 */
static inline int32_t ttalink_sensor_info_get_rtk_lonig(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  72);
}

/**
 * @brief Get field rtk_altit from sensor_info message
 *
 * @return  .
 */
static inline float ttalink_sensor_info_get_rtk_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  76);
}

/**
 * @brief Get field posX from sensor_info message
 *
 * @return  水平位置X.
 */
static inline float ttalink_sensor_info_get_posX(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  80);
}

/**
 * @brief Get field posY from sensor_info message
 *
 * @return  水平位置Y.
 */
static inline float ttalink_sensor_info_get_posY(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  84);
}

/**
 * @brief Get field posZ from sensor_info message
 *
 * @return  水平位置Z 向下为正.
 */
static inline float ttalink_sensor_info_get_posZ(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  88);
}

/**
 * @brief Get field velX from sensor_info message
 *
 * @return  机体速度X.
 */
static inline float ttalink_sensor_info_get_velX(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  92);
}

/**
 * @brief Get field velY from sensor_info message
 *
 * @return  机体速度Y.
 */
static inline float ttalink_sensor_info_get_velY(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  96);
}

/**
 * @brief Get field velZ from sensor_info message
 *
 * @return  机体速度Z 向下为正.
 */
static inline float ttalink_sensor_info_get_velZ(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  100);
}

/**
 * @brief Get field param from sensor_info message
 *
 * @return  .
 */
static inline uint16_t ttalink_sensor_info_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 4,  104);
}

/**
 * @brief Decode a sensor_info message into a struct
 *
 * @param msg The message to decode
 * @param sensor_info C-struct to decode the message contents into
 */
static inline void ttalink_sensor_info_decode(const ttalink_message_t* msg, ttalink_sensor_info_t* sensor_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sensor_info->accX = ttalink_sensor_info_get_accX(msg);
    sensor_info->accY = ttalink_sensor_info_get_accY(msg);
    sensor_info->accZ = ttalink_sensor_info_get_accZ(msg);
    sensor_info->accVar = ttalink_sensor_info_get_accVar(msg);
    sensor_info->gyroX = ttalink_sensor_info_get_gyroX(msg);
    sensor_info->gyroY = ttalink_sensor_info_get_gyroY(msg);
    sensor_info->gyroZ = ttalink_sensor_info_get_gyroZ(msg);
    sensor_info->gyroVar = ttalink_sensor_info_get_gyroVar(msg);
    sensor_info->baro = ttalink_sensor_info_get_baro(msg);
    sensor_info->baroH = ttalink_sensor_info_get_baroH(msg);
    sensor_info->magX = ttalink_sensor_info_get_magX(msg);
    sensor_info->magY = ttalink_sensor_info_get_magY(msg);
    sensor_info->magZ = ttalink_sensor_info_get_magZ(msg);
    sensor_info->magRoot = ttalink_sensor_info_get_magRoot(msg);
    sensor_info->gps_latit = ttalink_sensor_info_get_gps_latit(msg);
    sensor_info->gps_lonig = ttalink_sensor_info_get_gps_lonig(msg);
    sensor_info->gps_altit = ttalink_sensor_info_get_gps_altit(msg);
    sensor_info->rtk_latit = ttalink_sensor_info_get_rtk_latit(msg);
    sensor_info->rtk_lonig = ttalink_sensor_info_get_rtk_lonig(msg);
    sensor_info->rtk_altit = ttalink_sensor_info_get_rtk_altit(msg);
    sensor_info->posX = ttalink_sensor_info_get_posX(msg);
    sensor_info->posY = ttalink_sensor_info_get_posY(msg);
    sensor_info->posZ = ttalink_sensor_info_get_posZ(msg);
    sensor_info->velX = ttalink_sensor_info_get_velX(msg);
    sensor_info->velY = ttalink_sensor_info_get_velY(msg);
    sensor_info->velZ = ttalink_sensor_info_get_velZ(msg);
    ttalink_sensor_info_get_param(msg, sensor_info->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SENSOR_INFO_LEN? msg->len : TTALINK_MSG_ID_SENSOR_INFO_LEN;
        memset(sensor_info, 0, TTALINK_MSG_ID_SENSOR_INFO_LEN);
    memcpy(sensor_info, _TTA_PAYLOAD(msg), len);
#endif
}
