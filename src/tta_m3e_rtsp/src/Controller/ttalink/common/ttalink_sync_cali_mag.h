#pragma once
// MESSAGE SYNC_CALI_MAG PACKING

#define TTALINK_MSG_ID_SYNC_CALI_MAG 2121

TTAPACKED(
typedef struct __ttalink_sync_cali_mag_t {
 uint32_t simple_time; /*<  .*/
 float Mag1CaliX[3]; /*<  .*/
 float Mag1CaliY[3]; /*<  .*/
 float Mag1CaliZ[3]; /*<  .*/
 float Mag1Offset[3]; /*<  .*/
 float Mag2CaliX[3]; /*<  .*/
 float Mag2CaliY[3]; /*<  .*/
 float Mag2CaliZ[3]; /*<  .*/
 float Mag2Offset[3]; /*<  .*/
}) ttalink_sync_cali_mag_t;

#define TTALINK_MSG_ID_SYNC_CALI_MAG_LEN 100
#define TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN 100
#define TTALINK_MSG_ID_2121_LEN 100
#define TTALINK_MSG_ID_2121_MIN_LEN 100

#define TTALINK_MSG_ID_SYNC_CALI_MAG_CRC 228
#define TTALINK_MSG_ID_2121_CRC 228

#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG1CALIX_LEN 3
#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG1CALIY_LEN 3
#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG1CALIZ_LEN 3
#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG1OFFSET_LEN 3
#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG2CALIX_LEN 3
#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG2CALIY_LEN 3
#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG2CALIZ_LEN 3
#define TTALINK_MSG_SYNC_CALI_MAG_FIELD_MAG2OFFSET_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CALI_MAG { \
    2121, \
    "SYNC_CALI_MAG", \
    9, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_mag_t, simple_time) }, \
         { "Mag1CaliX", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_sync_cali_mag_t, Mag1CaliX) }, \
         { "Mag1CaliY", NULL, TTALINK_TYPE_FLOAT, 3, 16, offsetof(ttalink_sync_cali_mag_t, Mag1CaliY) }, \
         { "Mag1CaliZ", NULL, TTALINK_TYPE_FLOAT, 3, 28, offsetof(ttalink_sync_cali_mag_t, Mag1CaliZ) }, \
         { "Mag1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 40, offsetof(ttalink_sync_cali_mag_t, Mag1Offset) }, \
         { "Mag2CaliX", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_sync_cali_mag_t, Mag2CaliX) }, \
         { "Mag2CaliY", NULL, TTALINK_TYPE_FLOAT, 3, 64, offsetof(ttalink_sync_cali_mag_t, Mag2CaliY) }, \
         { "Mag2CaliZ", NULL, TTALINK_TYPE_FLOAT, 3, 76, offsetof(ttalink_sync_cali_mag_t, Mag2CaliZ) }, \
         { "Mag2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 88, offsetof(ttalink_sync_cali_mag_t, Mag2Offset) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CALI_MAG { \
    "SYNC_CALI_MAG", \
    9, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_mag_t, simple_time) }, \
         { "Mag1CaliX", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_sync_cali_mag_t, Mag1CaliX) }, \
         { "Mag1CaliY", NULL, TTALINK_TYPE_FLOAT, 3, 16, offsetof(ttalink_sync_cali_mag_t, Mag1CaliY) }, \
         { "Mag1CaliZ", NULL, TTALINK_TYPE_FLOAT, 3, 28, offsetof(ttalink_sync_cali_mag_t, Mag1CaliZ) }, \
         { "Mag1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 40, offsetof(ttalink_sync_cali_mag_t, Mag1Offset) }, \
         { "Mag2CaliX", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_sync_cali_mag_t, Mag2CaliX) }, \
         { "Mag2CaliY", NULL, TTALINK_TYPE_FLOAT, 3, 64, offsetof(ttalink_sync_cali_mag_t, Mag2CaliY) }, \
         { "Mag2CaliZ", NULL, TTALINK_TYPE_FLOAT, 3, 76, offsetof(ttalink_sync_cali_mag_t, Mag2CaliZ) }, \
         { "Mag2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 88, offsetof(ttalink_sync_cali_mag_t, Mag2Offset) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_cali_mag_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_MAG_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, Mag1CaliX, 3);
    _tta_put_float_array(buf, 16, Mag1CaliY, 3);
    _tta_put_float_array(buf, 28, Mag1CaliZ, 3);
    _tta_put_float_array(buf, 40, Mag1Offset, 3);
    _tta_put_float_array(buf, 52, Mag2CaliX, 3);
    _tta_put_float_array(buf, 64, Mag2CaliY, 3);
    _tta_put_float_array(buf, 76, Mag2CaliZ, 3);
    _tta_put_float_array(buf, 88, Mag2Offset, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN);
#else
    ttalink_sync_cali_mag_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.Mag1CaliX, Mag1CaliX, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1CaliY, Mag1CaliY, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1CaliZ, Mag1CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1Offset, Mag1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliX, Mag2CaliX, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliY, Mag2CaliY, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliZ, Mag2CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2Offset, Mag2Offset, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_MAG;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_mag message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param Mag1CaliX  .
 * @param Mag1CaliY  .
 * @param Mag1CaliZ  .
 * @param Mag1Offset  .
 * @param Mag2CaliX  .
 * @param Mag2CaliY  .
 * @param Mag2CaliZ  .
 * @param Mag2Offset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_mag_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset)
{
    return _ttalink_sync_cali_mag_pack(dst_addr, src_addr, msg,  simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, false);
}

/**
 * @brief Pack a sync_cali_mag message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param Mag1CaliX  .
 * @param Mag1CaliY  .
 * @param Mag1CaliZ  .
 * @param Mag1Offset  .
 * @param Mag2CaliX  .
 * @param Mag2CaliY  .
 * @param Mag2CaliZ  .
 * @param Mag2Offset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_mag_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset)
{
    return _ttalink_sync_cali_mag_pack(dst_addr, src_addr, msg,  simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, true);
}


static inline uint16_t _ttalink_sync_cali_mag_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *Mag1CaliX,const float *Mag1CaliY,const float *Mag1CaliZ,const float *Mag1Offset,const float *Mag2CaliX,const float *Mag2CaliY,const float *Mag2CaliZ,const float *Mag2Offset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_MAG_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, Mag1CaliX, 3);
    _tta_put_float_array(buf, 16, Mag1CaliY, 3);
    _tta_put_float_array(buf, 28, Mag1CaliZ, 3);
    _tta_put_float_array(buf, 40, Mag1Offset, 3);
    _tta_put_float_array(buf, 52, Mag2CaliX, 3);
    _tta_put_float_array(buf, 64, Mag2CaliY, 3);
    _tta_put_float_array(buf, 76, Mag2CaliZ, 3);
    _tta_put_float_array(buf, 88, Mag2Offset, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN);
#else
    ttalink_sync_cali_mag_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.Mag1CaliX, Mag1CaliX, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1CaliY, Mag1CaliY, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1CaliZ, Mag1CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1Offset, Mag1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliX, Mag2CaliX, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliY, Mag2CaliY, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliZ, Mag2CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2Offset, Mag2Offset, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_MAG;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_mag message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param Mag1CaliX  .
 * @param Mag1CaliY  .
 * @param Mag1CaliZ  .
 * @param Mag1Offset  .
 * @param Mag2CaliX  .
 * @param Mag2CaliY  .
 * @param Mag2CaliZ  .
 * @param Mag2Offset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_mag_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *Mag1CaliX,const float *Mag1CaliY,const float *Mag1CaliZ,const float *Mag1Offset,const float *Mag2CaliX,const float *Mag2CaliY,const float *Mag2CaliZ,const float *Mag2Offset)
{
    return _ttalink_sync_cali_mag_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, false);
}

/**
 * @brief Pack a sync_cali_mag message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param Mag1CaliX  .
 * @param Mag1CaliY  .
 * @param Mag1CaliZ  .
 * @param Mag1Offset  .
 * @param Mag2CaliX  .
 * @param Mag2CaliY  .
 * @param Mag2CaliZ  .
 * @param Mag2Offset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_mag_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *Mag1CaliX,const float *Mag1CaliY,const float *Mag1CaliZ,const float *Mag1Offset,const float *Mag2CaliX,const float *Mag2CaliY,const float *Mag2CaliZ,const float *Mag2Offset)
{
    return _ttalink_sync_cali_mag_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, true);
}


static inline uint16_t _ttalink_sync_cali_mag_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_mag_t* sync_cali_mag, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_mag_pack_nocrc(dst_addr, src_addr, msg, sync_cali_mag->simple_time, sync_cali_mag->Mag1CaliX, sync_cali_mag->Mag1CaliY, sync_cali_mag->Mag1CaliZ, sync_cali_mag->Mag1Offset, sync_cali_mag->Mag2CaliX, sync_cali_mag->Mag2CaliY, sync_cali_mag->Mag2CaliZ, sync_cali_mag->Mag2Offset);
    }else{
        return ttalink_sync_cali_mag_pack(dst_addr, src_addr, msg, sync_cali_mag->simple_time, sync_cali_mag->Mag1CaliX, sync_cali_mag->Mag1CaliY, sync_cali_mag->Mag1CaliZ, sync_cali_mag->Mag1Offset, sync_cali_mag->Mag2CaliX, sync_cali_mag->Mag2CaliY, sync_cali_mag->Mag2CaliZ, sync_cali_mag->Mag2Offset);
    }
    
}

/**
 * @brief Encode a sync_cali_mag struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_mag_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_mag_t* sync_cali_mag)
{
    return _ttalink_sync_cali_mag_encode(dst_addr, src_addr, msg, sync_cali_mag, false);
}

/**
 * @brief Encode a sync_cali_mag struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_mag_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_mag_t* sync_cali_mag)
{
    return _ttalink_sync_cali_mag_encode(dst_addr, src_addr, msg, sync_cali_mag, true);
}


static inline uint16_t _ttalink_sync_cali_mag_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_mag_t* sync_cali_mag, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_mag_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_cali_mag->simple_time, sync_cali_mag->Mag1CaliX, sync_cali_mag->Mag1CaliY, sync_cali_mag->Mag1CaliZ, sync_cali_mag->Mag1Offset, sync_cali_mag->Mag2CaliX, sync_cali_mag->Mag2CaliY, sync_cali_mag->Mag2CaliZ, sync_cali_mag->Mag2Offset);
    }else{
        return ttalink_sync_cali_mag_pack_chan(dst_addr, src_addr, chan, msg, sync_cali_mag->simple_time, sync_cali_mag->Mag1CaliX, sync_cali_mag->Mag1CaliY, sync_cali_mag->Mag1CaliZ, sync_cali_mag->Mag1Offset, sync_cali_mag->Mag2CaliX, sync_cali_mag->Mag2CaliY, sync_cali_mag->Mag2CaliZ, sync_cali_mag->Mag2Offset);
    }
}

/**
 * @brief Encode a sync_cali_mag struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_mag_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_mag_t* sync_cali_mag)
{
    return _ttalink_sync_cali_mag_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_mag, false);
}

/**
 * @brief Encode a sync_cali_mag struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_mag C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_mag_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_mag_t* sync_cali_mag)
{
    return _ttalink_sync_cali_mag_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_mag, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_cali_mag_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_MAG_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, Mag1CaliX, 3);
    _tta_put_float_array(buf, 16, Mag1CaliY, 3);
    _tta_put_float_array(buf, 28, Mag1CaliZ, 3);
    _tta_put_float_array(buf, 40, Mag1Offset, 3);
    _tta_put_float_array(buf, 52, Mag2CaliX, 3);
    _tta_put_float_array(buf, 64, Mag2CaliY, 3);
    _tta_put_float_array(buf, 76, Mag2CaliZ, 3);
    _tta_put_float_array(buf, 88, Mag2Offset, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_MAG, buf, TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_CRC, nocrc);
#else
    ttalink_sync_cali_mag_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.Mag1CaliX, Mag1CaliX, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1CaliY, Mag1CaliY, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1CaliZ, Mag1CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet.Mag1Offset, Mag1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliX, Mag2CaliX, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliY, Mag2CaliY, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2CaliZ, Mag2CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet.Mag2Offset, Mag2Offset, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_MAG, (const char *)&packet, TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_mag message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param Mag1CaliX  .
 * @param Mag1CaliY  .
 * @param Mag1CaliZ  .
 * @param Mag1Offset  .
 * @param Mag2CaliX  .
 * @param Mag2CaliY  .
 * @param Mag2CaliZ  .
 * @param Mag2Offset  .
 */
static inline void ttalink_sync_cali_mag_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset)
{
    _ttalink_sync_cali_mag_send(dst_addr, src_addr, chan, simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, false);
}

/**
 * @brief Send a sync_cali_mag message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param Mag1CaliX  .
 * @param Mag1CaliY  .
 * @param Mag1CaliZ  .
 * @param Mag1Offset  .
 * @param Mag2CaliX  .
 * @param Mag2CaliY  .
 * @param Mag2CaliZ  .
 * @param Mag2Offset  .
 */
static inline void ttalink_sync_cali_mag_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset)
{
    _ttalink_sync_cali_mag_send(dst_addr, src_addr, chan, simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, true);
}


static inline void _ttalink_sync_cali_mag_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_mag_t* sync_cali_mag, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_cali_mag_send_nocrc(dst_addr, src_addr, chan, sync_cali_mag->simple_time, sync_cali_mag->Mag1CaliX, sync_cali_mag->Mag1CaliY, sync_cali_mag->Mag1CaliZ, sync_cali_mag->Mag1Offset, sync_cali_mag->Mag2CaliX, sync_cali_mag->Mag2CaliY, sync_cali_mag->Mag2CaliZ, sync_cali_mag->Mag2Offset);
    }else{
        ttalink_sync_cali_mag_send(dst_addr, src_addr, chan, sync_cali_mag->simple_time, sync_cali_mag->Mag1CaliX, sync_cali_mag->Mag1CaliY, sync_cali_mag->Mag1CaliZ, sync_cali_mag->Mag1Offset, sync_cali_mag->Mag2CaliX, sync_cali_mag->Mag2CaliY, sync_cali_mag->Mag2CaliZ, sync_cali_mag->Mag2Offset);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_MAG, (const char *)sync_cali_mag, TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_mag message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_mag_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_mag_t* sync_cali_mag)
{
    _ttalink_sync_cali_mag_send_struct(dst_addr, src_addr, chan, sync_cali_mag, false);
}

/**
 * @brief Send a sync_cali_mag message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_mag_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_mag_t* sync_cali_mag)
{
    _ttalink_sync_cali_mag_send_struct(dst_addr, src_addr, chan, sync_cali_mag, true);
}

#if TTALINK_MSG_ID_SYNC_CALI_MAG_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_cali_mag_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, Mag1CaliX, 3);
    _tta_put_float_array(buf, 16, Mag1CaliY, 3);
    _tta_put_float_array(buf, 28, Mag1CaliZ, 3);
    _tta_put_float_array(buf, 40, Mag1Offset, 3);
    _tta_put_float_array(buf, 52, Mag2CaliX, 3);
    _tta_put_float_array(buf, 64, Mag2CaliY, 3);
    _tta_put_float_array(buf, 76, Mag2CaliZ, 3);
    _tta_put_float_array(buf, 88, Mag2Offset, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_MAG, buf, TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_CRC, nocrc);
#else
    ttalink_sync_cali_mag_t *packet = (ttalink_sync_cali_mag_t *)msgbuf;
    packet->simple_time = simple_time;
    tta_array_memcpy(packet->Mag1CaliX, Mag1CaliX, sizeof(float)*3);
    tta_array_memcpy(packet->Mag1CaliY, Mag1CaliY, sizeof(float)*3);
    tta_array_memcpy(packet->Mag1CaliZ, Mag1CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet->Mag1Offset, Mag1Offset, sizeof(float)*3);
    tta_array_memcpy(packet->Mag2CaliX, Mag2CaliX, sizeof(float)*3);
    tta_array_memcpy(packet->Mag2CaliY, Mag2CaliY, sizeof(float)*3);
    tta_array_memcpy(packet->Mag2CaliZ, Mag2CaliZ, sizeof(float)*3);
    tta_array_memcpy(packet->Mag2Offset, Mag2Offset, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_MAG, (const char *)packet, TTALINK_MSG_ID_SYNC_CALI_MAG_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN, TTALINK_MSG_ID_SYNC_CALI_MAG_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_cali_mag_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset)
{
    _ttalink_sync_cali_mag_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_cali_mag_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *Mag1CaliX, const float *Mag1CaliY, const float *Mag1CaliZ, const float *Mag1Offset, const float *Mag2CaliX, const float *Mag2CaliY, const float *Mag2CaliZ, const float *Mag2Offset)
{
    _ttalink_sync_cali_mag_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, Mag1CaliX, Mag1CaliY, Mag1CaliZ, Mag1Offset, Mag2CaliX, Mag2CaliY, Mag2CaliZ, Mag2Offset, true);
}
#endif

#endif

// MESSAGE SYNC_CALI_MAG UNPACKING


/**
 * @brief Get field simple_time from sync_cali_mag message
 *
 * @return  .
 */
static inline uint32_t ttalink_sync_cali_mag_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field Mag1CaliX from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag1CaliX(const ttalink_message_t* msg, float *Mag1CaliX)
{
    return _TTA_RETURN_float_array(msg, Mag1CaliX, 3,  4);
}

/**
 * @brief Get field Mag1CaliY from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag1CaliY(const ttalink_message_t* msg, float *Mag1CaliY)
{
    return _TTA_RETURN_float_array(msg, Mag1CaliY, 3,  16);
}

/**
 * @brief Get field Mag1CaliZ from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag1CaliZ(const ttalink_message_t* msg, float *Mag1CaliZ)
{
    return _TTA_RETURN_float_array(msg, Mag1CaliZ, 3,  28);
}

/**
 * @brief Get field Mag1Offset from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag1Offset(const ttalink_message_t* msg, float *Mag1Offset)
{
    return _TTA_RETURN_float_array(msg, Mag1Offset, 3,  40);
}

/**
 * @brief Get field Mag2CaliX from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag2CaliX(const ttalink_message_t* msg, float *Mag2CaliX)
{
    return _TTA_RETURN_float_array(msg, Mag2CaliX, 3,  52);
}

/**
 * @brief Get field Mag2CaliY from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag2CaliY(const ttalink_message_t* msg, float *Mag2CaliY)
{
    return _TTA_RETURN_float_array(msg, Mag2CaliY, 3,  64);
}

/**
 * @brief Get field Mag2CaliZ from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag2CaliZ(const ttalink_message_t* msg, float *Mag2CaliZ)
{
    return _TTA_RETURN_float_array(msg, Mag2CaliZ, 3,  76);
}

/**
 * @brief Get field Mag2Offset from sync_cali_mag message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_mag_get_Mag2Offset(const ttalink_message_t* msg, float *Mag2Offset)
{
    return _TTA_RETURN_float_array(msg, Mag2Offset, 3,  88);
}

/**
 * @brief Decode a sync_cali_mag message into a struct
 *
 * @param msg The message to decode
 * @param sync_cali_mag C-struct to decode the message contents into
 */
static inline void ttalink_sync_cali_mag_decode(const ttalink_message_t* msg, ttalink_sync_cali_mag_t* sync_cali_mag)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_cali_mag->simple_time = ttalink_sync_cali_mag_get_simple_time(msg);
    ttalink_sync_cali_mag_get_Mag1CaliX(msg, sync_cali_mag->Mag1CaliX);
    ttalink_sync_cali_mag_get_Mag1CaliY(msg, sync_cali_mag->Mag1CaliY);
    ttalink_sync_cali_mag_get_Mag1CaliZ(msg, sync_cali_mag->Mag1CaliZ);
    ttalink_sync_cali_mag_get_Mag1Offset(msg, sync_cali_mag->Mag1Offset);
    ttalink_sync_cali_mag_get_Mag2CaliX(msg, sync_cali_mag->Mag2CaliX);
    ttalink_sync_cali_mag_get_Mag2CaliY(msg, sync_cali_mag->Mag2CaliY);
    ttalink_sync_cali_mag_get_Mag2CaliZ(msg, sync_cali_mag->Mag2CaliZ);
    ttalink_sync_cali_mag_get_Mag2Offset(msg, sync_cali_mag->Mag2Offset);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CALI_MAG_LEN? msg->len : TTALINK_MSG_ID_SYNC_CALI_MAG_LEN;
        memset(sync_cali_mag, 0, TTALINK_MSG_ID_SYNC_CALI_MAG_LEN);
    memcpy(sync_cali_mag, _TTA_PAYLOAD(msg), len);
#endif
}
