#pragma once
// MESSAGE SYNC_CALI_ACC PACKING

#define TTALINK_MSG_ID_SYNC_CALI_ACC 2120

TTAPACKED(
typedef struct __ttalink_sync_cali_acc_t {
 uint32_t simple_time; /*<  .*/
 float MainAcc1Gain[3]; /*<  .*/
 float MainAcc2Gain[3]; /*<  .*/
 float BackUpAccGain[3]; /*<  .*/
 float MainAcc1Offset[3]; /*<  .*/
 float MainAcc2Offset[3]; /*<  .*/
 float BackUpAccOffset[3]; /*<  .*/
}) ttalink_sync_cali_acc_t;

#define TTALINK_MSG_ID_SYNC_CALI_ACC_LEN 76
#define TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN 76
#define TTALINK_MSG_ID_2120_LEN 76
#define TTALINK_MSG_ID_2120_MIN_LEN 76

#define TTALINK_MSG_ID_SYNC_CALI_ACC_CRC 209
#define TTALINK_MSG_ID_2120_CRC 209

#define TTALINK_MSG_SYNC_CALI_ACC_FIELD_MAINACC1GAIN_LEN 3
#define TTALINK_MSG_SYNC_CALI_ACC_FIELD_MAINACC2GAIN_LEN 3
#define TTALINK_MSG_SYNC_CALI_ACC_FIELD_BACKUPACCGAIN_LEN 3
#define TTALINK_MSG_SYNC_CALI_ACC_FIELD_MAINACC1OFFSET_LEN 3
#define TTALINK_MSG_SYNC_CALI_ACC_FIELD_MAINACC2OFFSET_LEN 3
#define TTALINK_MSG_SYNC_CALI_ACC_FIELD_BACKUPACCOFFSET_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_CALI_ACC { \
    2120, \
    "SYNC_CALI_ACC", \
    7, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_acc_t, simple_time) }, \
         { "MainAcc1Gain", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_sync_cali_acc_t, MainAcc1Gain) }, \
         { "MainAcc2Gain", NULL, TTALINK_TYPE_FLOAT, 3, 16, offsetof(ttalink_sync_cali_acc_t, MainAcc2Gain) }, \
         { "BackUpAccGain", NULL, TTALINK_TYPE_FLOAT, 3, 28, offsetof(ttalink_sync_cali_acc_t, BackUpAccGain) }, \
         { "MainAcc1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 40, offsetof(ttalink_sync_cali_acc_t, MainAcc1Offset) }, \
         { "MainAcc2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_sync_cali_acc_t, MainAcc2Offset) }, \
         { "BackUpAccOffset", NULL, TTALINK_TYPE_FLOAT, 3, 64, offsetof(ttalink_sync_cali_acc_t, BackUpAccOffset) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_CALI_ACC { \
    "SYNC_CALI_ACC", \
    7, \
    {  { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_sync_cali_acc_t, simple_time) }, \
         { "MainAcc1Gain", NULL, TTALINK_TYPE_FLOAT, 3, 4, offsetof(ttalink_sync_cali_acc_t, MainAcc1Gain) }, \
         { "MainAcc2Gain", NULL, TTALINK_TYPE_FLOAT, 3, 16, offsetof(ttalink_sync_cali_acc_t, MainAcc2Gain) }, \
         { "BackUpAccGain", NULL, TTALINK_TYPE_FLOAT, 3, 28, offsetof(ttalink_sync_cali_acc_t, BackUpAccGain) }, \
         { "MainAcc1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 40, offsetof(ttalink_sync_cali_acc_t, MainAcc1Offset) }, \
         { "MainAcc2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 52, offsetof(ttalink_sync_cali_acc_t, MainAcc2Offset) }, \
         { "BackUpAccOffset", NULL, TTALINK_TYPE_FLOAT, 3, 64, offsetof(ttalink_sync_cali_acc_t, BackUpAccOffset) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_cali_acc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_ACC_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, MainAcc1Gain, 3);
    _tta_put_float_array(buf, 16, MainAcc2Gain, 3);
    _tta_put_float_array(buf, 28, BackUpAccGain, 3);
    _tta_put_float_array(buf, 40, MainAcc1Offset, 3);
    _tta_put_float_array(buf, 52, MainAcc2Offset, 3);
    _tta_put_float_array(buf, 64, BackUpAccOffset, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN);
#else
    ttalink_sync_cali_acc_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.MainAcc1Gain, MainAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc2Gain, MainAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAccGain, BackUpAccGain, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc1Offset, MainAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc2Offset, MainAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAccOffset, BackUpAccOffset, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_ACC;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_acc message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param MainAcc1Gain  .
 * @param MainAcc2Gain  .
 * @param BackUpAccGain  .
 * @param MainAcc1Offset  .
 * @param MainAcc2Offset  .
 * @param BackUpAccOffset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_acc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset)
{
    return _ttalink_sync_cali_acc_pack(dst_addr, src_addr, msg,  simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, false);
}

/**
 * @brief Pack a sync_cali_acc message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param simple_time  .
 * @param MainAcc1Gain  .
 * @param MainAcc2Gain  .
 * @param BackUpAccGain  .
 * @param MainAcc1Offset  .
 * @param MainAcc2Offset  .
 * @param BackUpAccOffset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_acc_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset)
{
    return _ttalink_sync_cali_acc_pack(dst_addr, src_addr, msg,  simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, true);
}


static inline uint16_t _ttalink_sync_cali_acc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *MainAcc1Gain,const float *MainAcc2Gain,const float *BackUpAccGain,const float *MainAcc1Offset,const float *MainAcc2Offset,const float *BackUpAccOffset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_ACC_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, MainAcc1Gain, 3);
    _tta_put_float_array(buf, 16, MainAcc2Gain, 3);
    _tta_put_float_array(buf, 28, BackUpAccGain, 3);
    _tta_put_float_array(buf, 40, MainAcc1Offset, 3);
    _tta_put_float_array(buf, 52, MainAcc2Offset, 3);
    _tta_put_float_array(buf, 64, BackUpAccOffset, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN);
#else
    ttalink_sync_cali_acc_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.MainAcc1Gain, MainAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc2Gain, MainAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAccGain, BackUpAccGain, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc1Offset, MainAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc2Offset, MainAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAccOffset, BackUpAccOffset, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_CALI_ACC;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_CRC, nocrc);
}

/**
 * @brief Pack a sync_cali_acc message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param MainAcc1Gain  .
 * @param MainAcc2Gain  .
 * @param BackUpAccGain  .
 * @param MainAcc1Offset  .
 * @param MainAcc2Offset  .
 * @param BackUpAccOffset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_acc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *MainAcc1Gain,const float *MainAcc2Gain,const float *BackUpAccGain,const float *MainAcc1Offset,const float *MainAcc2Offset,const float *BackUpAccOffset)
{
    return _ttalink_sync_cali_acc_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, false);
}

/**
 * @brief Pack a sync_cali_acc message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param simple_time  .
 * @param MainAcc1Gain  .
 * @param MainAcc2Gain  .
 * @param BackUpAccGain  .
 * @param MainAcc1Offset  .
 * @param MainAcc2Offset  .
 * @param BackUpAccOffset  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_cali_acc_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t simple_time,const float *MainAcc1Gain,const float *MainAcc2Gain,const float *BackUpAccGain,const float *MainAcc1Offset,const float *MainAcc2Offset,const float *BackUpAccOffset)
{
    return _ttalink_sync_cali_acc_pack_chan(dst_addr, src_addr, chan, msg,  simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, true);
}


static inline uint16_t _ttalink_sync_cali_acc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_acc_t* sync_cali_acc, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_acc_pack_nocrc(dst_addr, src_addr, msg, sync_cali_acc->simple_time, sync_cali_acc->MainAcc1Gain, sync_cali_acc->MainAcc2Gain, sync_cali_acc->BackUpAccGain, sync_cali_acc->MainAcc1Offset, sync_cali_acc->MainAcc2Offset, sync_cali_acc->BackUpAccOffset);
    }else{
        return ttalink_sync_cali_acc_pack(dst_addr, src_addr, msg, sync_cali_acc->simple_time, sync_cali_acc->MainAcc1Gain, sync_cali_acc->MainAcc2Gain, sync_cali_acc->BackUpAccGain, sync_cali_acc->MainAcc1Offset, sync_cali_acc->MainAcc2Offset, sync_cali_acc->BackUpAccOffset);
    }
    
}

/**
 * @brief Encode a sync_cali_acc struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_acc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_acc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_acc_t* sync_cali_acc)
{
    return _ttalink_sync_cali_acc_encode(dst_addr, src_addr, msg, sync_cali_acc, false);
}

/**
 * @brief Encode a sync_cali_acc struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_acc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_acc_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_cali_acc_t* sync_cali_acc)
{
    return _ttalink_sync_cali_acc_encode(dst_addr, src_addr, msg, sync_cali_acc, true);
}


static inline uint16_t _ttalink_sync_cali_acc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_acc_t* sync_cali_acc, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_cali_acc_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_cali_acc->simple_time, sync_cali_acc->MainAcc1Gain, sync_cali_acc->MainAcc2Gain, sync_cali_acc->BackUpAccGain, sync_cali_acc->MainAcc1Offset, sync_cali_acc->MainAcc2Offset, sync_cali_acc->BackUpAccOffset);
    }else{
        return ttalink_sync_cali_acc_pack_chan(dst_addr, src_addr, chan, msg, sync_cali_acc->simple_time, sync_cali_acc->MainAcc1Gain, sync_cali_acc->MainAcc2Gain, sync_cali_acc->BackUpAccGain, sync_cali_acc->MainAcc1Offset, sync_cali_acc->MainAcc2Offset, sync_cali_acc->BackUpAccOffset);
    }
}

/**
 * @brief Encode a sync_cali_acc struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_acc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_acc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_acc_t* sync_cali_acc)
{
    return _ttalink_sync_cali_acc_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_acc, false);
}

/**
 * @brief Encode a sync_cali_acc struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_cali_acc C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_cali_acc_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_cali_acc_t* sync_cali_acc)
{
    return _ttalink_sync_cali_acc_encode_chan(dst_addr, src_addr, chan, msg, sync_cali_acc, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_cali_acc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_CALI_ACC_LEN];
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, MainAcc1Gain, 3);
    _tta_put_float_array(buf, 16, MainAcc2Gain, 3);
    _tta_put_float_array(buf, 28, BackUpAccGain, 3);
    _tta_put_float_array(buf, 40, MainAcc1Offset, 3);
    _tta_put_float_array(buf, 52, MainAcc2Offset, 3);
    _tta_put_float_array(buf, 64, BackUpAccOffset, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_ACC, buf, TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_CRC, nocrc);
#else
    ttalink_sync_cali_acc_t packet;
    packet.simple_time = simple_time;
    tta_array_memcpy(packet.MainAcc1Gain, MainAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc2Gain, MainAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAccGain, BackUpAccGain, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc1Offset, MainAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.MainAcc2Offset, MainAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAccOffset, BackUpAccOffset, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_ACC, (const char *)&packet, TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_acc message
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param MainAcc1Gain  .
 * @param MainAcc2Gain  .
 * @param BackUpAccGain  .
 * @param MainAcc1Offset  .
 * @param MainAcc2Offset  .
 * @param BackUpAccOffset  .
 */
static inline void ttalink_sync_cali_acc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset)
{
    _ttalink_sync_cali_acc_send(dst_addr, src_addr, chan, simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, false);
}

/**
 * @brief Send a sync_cali_acc message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param simple_time  .
 * @param MainAcc1Gain  .
 * @param MainAcc2Gain  .
 * @param BackUpAccGain  .
 * @param MainAcc1Offset  .
 * @param MainAcc2Offset  .
 * @param BackUpAccOffset  .
 */
static inline void ttalink_sync_cali_acc_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset)
{
    _ttalink_sync_cali_acc_send(dst_addr, src_addr, chan, simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, true);
}


static inline void _ttalink_sync_cali_acc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_acc_t* sync_cali_acc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_cali_acc_send_nocrc(dst_addr, src_addr, chan, sync_cali_acc->simple_time, sync_cali_acc->MainAcc1Gain, sync_cali_acc->MainAcc2Gain, sync_cali_acc->BackUpAccGain, sync_cali_acc->MainAcc1Offset, sync_cali_acc->MainAcc2Offset, sync_cali_acc->BackUpAccOffset);
    }else{
        ttalink_sync_cali_acc_send(dst_addr, src_addr, chan, sync_cali_acc->simple_time, sync_cali_acc->MainAcc1Gain, sync_cali_acc->MainAcc2Gain, sync_cali_acc->BackUpAccGain, sync_cali_acc->MainAcc1Offset, sync_cali_acc->MainAcc2Offset, sync_cali_acc->BackUpAccOffset);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_ACC, (const char *)sync_cali_acc, TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_cali_acc message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_acc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_acc_t* sync_cali_acc)
{
    _ttalink_sync_cali_acc_send_struct(dst_addr, src_addr, chan, sync_cali_acc, false);
}

/**
 * @brief Send a sync_cali_acc message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_cali_acc_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_cali_acc_t* sync_cali_acc)
{
    _ttalink_sync_cali_acc_send_struct(dst_addr, src_addr, chan, sync_cali_acc, true);
}

#if TTALINK_MSG_ID_SYNC_CALI_ACC_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_cali_acc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, simple_time);
    _tta_put_float_array(buf, 4, MainAcc1Gain, 3);
    _tta_put_float_array(buf, 16, MainAcc2Gain, 3);
    _tta_put_float_array(buf, 28, BackUpAccGain, 3);
    _tta_put_float_array(buf, 40, MainAcc1Offset, 3);
    _tta_put_float_array(buf, 52, MainAcc2Offset, 3);
    _tta_put_float_array(buf, 64, BackUpAccOffset, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_ACC, buf, TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_CRC, nocrc);
#else
    ttalink_sync_cali_acc_t *packet = (ttalink_sync_cali_acc_t *)msgbuf;
    packet->simple_time = simple_time;
    tta_array_memcpy(packet->MainAcc1Gain, MainAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet->MainAcc2Gain, MainAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAccGain, BackUpAccGain, sizeof(float)*3);
    tta_array_memcpy(packet->MainAcc1Offset, MainAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet->MainAcc2Offset, MainAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAccOffset, BackUpAccOffset, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_CALI_ACC, (const char *)packet, TTALINK_MSG_ID_SYNC_CALI_ACC_MIN_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN, TTALINK_MSG_ID_SYNC_CALI_ACC_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_cali_acc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset)
{
    _ttalink_sync_cali_acc_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_cali_acc_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t simple_time, const float *MainAcc1Gain, const float *MainAcc2Gain, const float *BackUpAccGain, const float *MainAcc1Offset, const float *MainAcc2Offset, const float *BackUpAccOffset)
{
    _ttalink_sync_cali_acc_send_buf(dst_addr, src_addr, msgbuf, chan, simple_time, MainAcc1Gain, MainAcc2Gain, BackUpAccGain, MainAcc1Offset, MainAcc2Offset, BackUpAccOffset, true);
}
#endif

#endif

// MESSAGE SYNC_CALI_ACC UNPACKING


/**
 * @brief Get field simple_time from sync_cali_acc message
 *
 * @return  .
 */
static inline uint32_t ttalink_sync_cali_acc_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field MainAcc1Gain from sync_cali_acc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_acc_get_MainAcc1Gain(const ttalink_message_t* msg, float *MainAcc1Gain)
{
    return _TTA_RETURN_float_array(msg, MainAcc1Gain, 3,  4);
}

/**
 * @brief Get field MainAcc2Gain from sync_cali_acc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_acc_get_MainAcc2Gain(const ttalink_message_t* msg, float *MainAcc2Gain)
{
    return _TTA_RETURN_float_array(msg, MainAcc2Gain, 3,  16);
}

/**
 * @brief Get field BackUpAccGain from sync_cali_acc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_acc_get_BackUpAccGain(const ttalink_message_t* msg, float *BackUpAccGain)
{
    return _TTA_RETURN_float_array(msg, BackUpAccGain, 3,  28);
}

/**
 * @brief Get field MainAcc1Offset from sync_cali_acc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_acc_get_MainAcc1Offset(const ttalink_message_t* msg, float *MainAcc1Offset)
{
    return _TTA_RETURN_float_array(msg, MainAcc1Offset, 3,  40);
}

/**
 * @brief Get field MainAcc2Offset from sync_cali_acc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_acc_get_MainAcc2Offset(const ttalink_message_t* msg, float *MainAcc2Offset)
{
    return _TTA_RETURN_float_array(msg, MainAcc2Offset, 3,  52);
}

/**
 * @brief Get field BackUpAccOffset from sync_cali_acc message
 *
 * @return  .
 */
static inline uint16_t ttalink_sync_cali_acc_get_BackUpAccOffset(const ttalink_message_t* msg, float *BackUpAccOffset)
{
    return _TTA_RETURN_float_array(msg, BackUpAccOffset, 3,  64);
}

/**
 * @brief Decode a sync_cali_acc message into a struct
 *
 * @param msg The message to decode
 * @param sync_cali_acc C-struct to decode the message contents into
 */
static inline void ttalink_sync_cali_acc_decode(const ttalink_message_t* msg, ttalink_sync_cali_acc_t* sync_cali_acc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_cali_acc->simple_time = ttalink_sync_cali_acc_get_simple_time(msg);
    ttalink_sync_cali_acc_get_MainAcc1Gain(msg, sync_cali_acc->MainAcc1Gain);
    ttalink_sync_cali_acc_get_MainAcc2Gain(msg, sync_cali_acc->MainAcc2Gain);
    ttalink_sync_cali_acc_get_BackUpAccGain(msg, sync_cali_acc->BackUpAccGain);
    ttalink_sync_cali_acc_get_MainAcc1Offset(msg, sync_cali_acc->MainAcc1Offset);
    ttalink_sync_cali_acc_get_MainAcc2Offset(msg, sync_cali_acc->MainAcc2Offset);
    ttalink_sync_cali_acc_get_BackUpAccOffset(msg, sync_cali_acc->BackUpAccOffset);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_CALI_ACC_LEN? msg->len : TTALINK_MSG_ID_SYNC_CALI_ACC_LEN;
        memset(sync_cali_acc, 0, TTALINK_MSG_ID_SYNC_CALI_ACC_LEN);
    memcpy(sync_cali_acc, _TTA_PAYLOAD(msg), len);
#endif
}
