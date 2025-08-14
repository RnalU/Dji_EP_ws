#pragma once
// MESSAGE CALIBRATE_DATA PACKING

#define TTALINK_MSG_ID_CALIBRATE_DATA 2127

TTAPACKED(
typedef struct __ttalink_calibrate_data_t {
 float BackUpGyro1Offset[3]; /*< This is the offset of backup gyro1.*/
 float BackUpGyro1Gain[3]; /*< This is the gain of backup gyro1.*/
 float BackUpAcc1Offset[3]; /*< This is the offset of backup acc1.*/
 float BackUpAcc1Gain[3]; /*< This is the gain of backup acc1.*/
 float BackUpGyro2Offset[3]; /*< This is the offset of backup gyro2.*/
 float BackUpGyro2Gain[3]; /*< This is the gain of backup gyro2.*/
 float BackUpAcc2Offset[3]; /*< This is the offset of backup acc2.*/
 float BackUpAcc2Gain[3]; /*< This is the gain of backup acc2.*/
 float BackUpGyro3Offset[3]; /*< This is the offset of backup gyro3.*/
 float BackUpGyro3Gain[3]; /*< This is the gain of backup gyro3.*/
 float BackUpAcc3Offset[3]; /*< This is the offset of backup acc3.*/
 float BackUpAcc3Gain[3]; /*< This is the gain of backup acc3.*/
}) ttalink_calibrate_data_t;

#define TTALINK_MSG_ID_CALIBRATE_DATA_LEN 144
#define TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN 144
#define TTALINK_MSG_ID_2127_LEN 144
#define TTALINK_MSG_ID_2127_MIN_LEN 144

#define TTALINK_MSG_ID_CALIBRATE_DATA_CRC 58
#define TTALINK_MSG_ID_2127_CRC 58

#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPGYRO1OFFSET_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPGYRO1GAIN_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPACC1OFFSET_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPACC1GAIN_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPGYRO2OFFSET_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPGYRO2GAIN_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPACC2OFFSET_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPACC2GAIN_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPGYRO3OFFSET_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPGYRO3GAIN_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPACC3OFFSET_LEN 3
#define TTALINK_MSG_CALIBRATE_DATA_FIELD_BACKUPACC3GAIN_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_CALIBRATE_DATA { \
    2127, \
    "CALIBRATE_DATA", \
    12, \
    {  { "BackUpGyro1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_calibrate_data_t, BackUpGyro1Offset) }, \
         { "BackUpGyro1Gain", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_calibrate_data_t, BackUpGyro1Gain) }, \
         { "BackUpAcc1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_calibrate_data_t, BackUpAcc1Offset) }, \
         { "BackUpAcc1Gain", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_calibrate_data_t, BackUpAcc1Gain) }, \
         { "BackUpGyro2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_calibrate_data_t, BackUpGyro2Offset) }, \
         { "BackUpGyro2Gain", NULL, TTALINK_TYPE_FLOAT, 3, 60, offsetof(ttalink_calibrate_data_t, BackUpGyro2Gain) }, \
         { "BackUpAcc2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 72, offsetof(ttalink_calibrate_data_t, BackUpAcc2Offset) }, \
         { "BackUpAcc2Gain", NULL, TTALINK_TYPE_FLOAT, 3, 84, offsetof(ttalink_calibrate_data_t, BackUpAcc2Gain) }, \
         { "BackUpGyro3Offset", NULL, TTALINK_TYPE_FLOAT, 3, 96, offsetof(ttalink_calibrate_data_t, BackUpGyro3Offset) }, \
         { "BackUpGyro3Gain", NULL, TTALINK_TYPE_FLOAT, 3, 108, offsetof(ttalink_calibrate_data_t, BackUpGyro3Gain) }, \
         { "BackUpAcc3Offset", NULL, TTALINK_TYPE_FLOAT, 3, 120, offsetof(ttalink_calibrate_data_t, BackUpAcc3Offset) }, \
         { "BackUpAcc3Gain", NULL, TTALINK_TYPE_FLOAT, 3, 132, offsetof(ttalink_calibrate_data_t, BackUpAcc3Gain) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_CALIBRATE_DATA { \
    "CALIBRATE_DATA", \
    12, \
    {  { "BackUpGyro1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_calibrate_data_t, BackUpGyro1Offset) }, \
         { "BackUpGyro1Gain", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_calibrate_data_t, BackUpGyro1Gain) }, \
         { "BackUpAcc1Offset", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_calibrate_data_t, BackUpAcc1Offset) }, \
         { "BackUpAcc1Gain", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_calibrate_data_t, BackUpAcc1Gain) }, \
         { "BackUpGyro2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_calibrate_data_t, BackUpGyro2Offset) }, \
         { "BackUpGyro2Gain", NULL, TTALINK_TYPE_FLOAT, 3, 60, offsetof(ttalink_calibrate_data_t, BackUpGyro2Gain) }, \
         { "BackUpAcc2Offset", NULL, TTALINK_TYPE_FLOAT, 3, 72, offsetof(ttalink_calibrate_data_t, BackUpAcc2Offset) }, \
         { "BackUpAcc2Gain", NULL, TTALINK_TYPE_FLOAT, 3, 84, offsetof(ttalink_calibrate_data_t, BackUpAcc2Gain) }, \
         { "BackUpGyro3Offset", NULL, TTALINK_TYPE_FLOAT, 3, 96, offsetof(ttalink_calibrate_data_t, BackUpGyro3Offset) }, \
         { "BackUpGyro3Gain", NULL, TTALINK_TYPE_FLOAT, 3, 108, offsetof(ttalink_calibrate_data_t, BackUpGyro3Gain) }, \
         { "BackUpAcc3Offset", NULL, TTALINK_TYPE_FLOAT, 3, 120, offsetof(ttalink_calibrate_data_t, BackUpAcc3Offset) }, \
         { "BackUpAcc3Gain", NULL, TTALINK_TYPE_FLOAT, 3, 132, offsetof(ttalink_calibrate_data_t, BackUpAcc3Gain) }, \
         } \
}
#endif


static inline uint16_t _ttalink_calibrate_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CALIBRATE_DATA_LEN];

    _tta_put_float_array(buf, 0, BackUpGyro1Offset, 3);
    _tta_put_float_array(buf, 12, BackUpGyro1Gain, 3);
    _tta_put_float_array(buf, 24, BackUpAcc1Offset, 3);
    _tta_put_float_array(buf, 36, BackUpAcc1Gain, 3);
    _tta_put_float_array(buf, 48, BackUpGyro2Offset, 3);
    _tta_put_float_array(buf, 60, BackUpGyro2Gain, 3);
    _tta_put_float_array(buf, 72, BackUpAcc2Offset, 3);
    _tta_put_float_array(buf, 84, BackUpAcc2Gain, 3);
    _tta_put_float_array(buf, 96, BackUpGyro3Offset, 3);
    _tta_put_float_array(buf, 108, BackUpGyro3Gain, 3);
    _tta_put_float_array(buf, 120, BackUpAcc3Offset, 3);
    _tta_put_float_array(buf, 132, BackUpAcc3Gain, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CALIBRATE_DATA_LEN);
#else
    ttalink_calibrate_data_t packet;

    tta_array_memcpy(packet.BackUpGyro1Offset, BackUpGyro1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro1Gain, BackUpGyro1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc1Offset, BackUpAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc1Gain, BackUpAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro2Offset, BackUpGyro2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro2Gain, BackUpGyro2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc2Offset, BackUpAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc2Gain, BackUpAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro3Offset, BackUpGyro3Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro3Gain, BackUpGyro3Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc3Offset, BackUpAcc3Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc3Gain, BackUpAcc3Gain, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CALIBRATE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CALIBRATE_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a calibrate_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param BackUpGyro1Offset This is the offset of backup gyro1.
 * @param BackUpGyro1Gain This is the gain of backup gyro1.
 * @param BackUpAcc1Offset This is the offset of backup acc1.
 * @param BackUpAcc1Gain This is the gain of backup acc1.
 * @param BackUpGyro2Offset This is the offset of backup gyro2.
 * @param BackUpGyro2Gain This is the gain of backup gyro2.
 * @param BackUpAcc2Offset This is the offset of backup acc2.
 * @param BackUpAcc2Gain This is the gain of backup acc2.
 * @param BackUpGyro3Offset This is the offset of backup gyro3.
 * @param BackUpGyro3Gain This is the gain of backup gyro3.
 * @param BackUpAcc3Offset This is the offset of backup acc3.
 * @param BackUpAcc3Gain This is the gain of backup acc3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain)
{
    return _ttalink_calibrate_data_pack(dst_addr, src_addr, msg,  BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, false);
}

/**
 * @brief Pack a calibrate_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param BackUpGyro1Offset This is the offset of backup gyro1.
 * @param BackUpGyro1Gain This is the gain of backup gyro1.
 * @param BackUpAcc1Offset This is the offset of backup acc1.
 * @param BackUpAcc1Gain This is the gain of backup acc1.
 * @param BackUpGyro2Offset This is the offset of backup gyro2.
 * @param BackUpGyro2Gain This is the gain of backup gyro2.
 * @param BackUpAcc2Offset This is the offset of backup acc2.
 * @param BackUpAcc2Gain This is the gain of backup acc2.
 * @param BackUpGyro3Offset This is the offset of backup gyro3.
 * @param BackUpGyro3Gain This is the gain of backup gyro3.
 * @param BackUpAcc3Offset This is the offset of backup acc3.
 * @param BackUpAcc3Gain This is the gain of backup acc3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain)
{
    return _ttalink_calibrate_data_pack(dst_addr, src_addr, msg,  BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, true);
}


static inline uint16_t _ttalink_calibrate_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *BackUpGyro1Offset,const float *BackUpGyro1Gain,const float *BackUpAcc1Offset,const float *BackUpAcc1Gain,const float *BackUpGyro2Offset,const float *BackUpGyro2Gain,const float *BackUpAcc2Offset,const float *BackUpAcc2Gain,const float *BackUpGyro3Offset,const float *BackUpGyro3Gain,const float *BackUpAcc3Offset,const float *BackUpAcc3Gain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CALIBRATE_DATA_LEN];

    _tta_put_float_array(buf, 0, BackUpGyro1Offset, 3);
    _tta_put_float_array(buf, 12, BackUpGyro1Gain, 3);
    _tta_put_float_array(buf, 24, BackUpAcc1Offset, 3);
    _tta_put_float_array(buf, 36, BackUpAcc1Gain, 3);
    _tta_put_float_array(buf, 48, BackUpGyro2Offset, 3);
    _tta_put_float_array(buf, 60, BackUpGyro2Gain, 3);
    _tta_put_float_array(buf, 72, BackUpAcc2Offset, 3);
    _tta_put_float_array(buf, 84, BackUpAcc2Gain, 3);
    _tta_put_float_array(buf, 96, BackUpGyro3Offset, 3);
    _tta_put_float_array(buf, 108, BackUpGyro3Gain, 3);
    _tta_put_float_array(buf, 120, BackUpAcc3Offset, 3);
    _tta_put_float_array(buf, 132, BackUpAcc3Gain, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CALIBRATE_DATA_LEN);
#else
    ttalink_calibrate_data_t packet;

    tta_array_memcpy(packet.BackUpGyro1Offset, BackUpGyro1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro1Gain, BackUpGyro1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc1Offset, BackUpAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc1Gain, BackUpAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro2Offset, BackUpGyro2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro2Gain, BackUpGyro2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc2Offset, BackUpAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc2Gain, BackUpAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro3Offset, BackUpGyro3Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro3Gain, BackUpGyro3Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc3Offset, BackUpAcc3Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc3Gain, BackUpAcc3Gain, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CALIBRATE_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CALIBRATE_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_CRC, nocrc);
}

/**
 * @brief Pack a calibrate_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param BackUpGyro1Offset This is the offset of backup gyro1.
 * @param BackUpGyro1Gain This is the gain of backup gyro1.
 * @param BackUpAcc1Offset This is the offset of backup acc1.
 * @param BackUpAcc1Gain This is the gain of backup acc1.
 * @param BackUpGyro2Offset This is the offset of backup gyro2.
 * @param BackUpGyro2Gain This is the gain of backup gyro2.
 * @param BackUpAcc2Offset This is the offset of backup acc2.
 * @param BackUpAcc2Gain This is the gain of backup acc2.
 * @param BackUpGyro3Offset This is the offset of backup gyro3.
 * @param BackUpGyro3Gain This is the gain of backup gyro3.
 * @param BackUpAcc3Offset This is the offset of backup acc3.
 * @param BackUpAcc3Gain This is the gain of backup acc3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *BackUpGyro1Offset,const float *BackUpGyro1Gain,const float *BackUpAcc1Offset,const float *BackUpAcc1Gain,const float *BackUpGyro2Offset,const float *BackUpGyro2Gain,const float *BackUpAcc2Offset,const float *BackUpAcc2Gain,const float *BackUpGyro3Offset,const float *BackUpGyro3Gain,const float *BackUpAcc3Offset,const float *BackUpAcc3Gain)
{
    return _ttalink_calibrate_data_pack_chan(dst_addr, src_addr, chan, msg,  BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, false);
}

/**
 * @brief Pack a calibrate_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param BackUpGyro1Offset This is the offset of backup gyro1.
 * @param BackUpGyro1Gain This is the gain of backup gyro1.
 * @param BackUpAcc1Offset This is the offset of backup acc1.
 * @param BackUpAcc1Gain This is the gain of backup acc1.
 * @param BackUpGyro2Offset This is the offset of backup gyro2.
 * @param BackUpGyro2Gain This is the gain of backup gyro2.
 * @param BackUpAcc2Offset This is the offset of backup acc2.
 * @param BackUpAcc2Gain This is the gain of backup acc2.
 * @param BackUpGyro3Offset This is the offset of backup gyro3.
 * @param BackUpGyro3Gain This is the gain of backup gyro3.
 * @param BackUpAcc3Offset This is the offset of backup acc3.
 * @param BackUpAcc3Gain This is the gain of backup acc3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_calibrate_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *BackUpGyro1Offset,const float *BackUpGyro1Gain,const float *BackUpAcc1Offset,const float *BackUpAcc1Gain,const float *BackUpGyro2Offset,const float *BackUpGyro2Gain,const float *BackUpAcc2Offset,const float *BackUpAcc2Gain,const float *BackUpGyro3Offset,const float *BackUpGyro3Gain,const float *BackUpAcc3Offset,const float *BackUpAcc3Gain)
{
    return _ttalink_calibrate_data_pack_chan(dst_addr, src_addr, chan, msg,  BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, true);
}


static inline uint16_t _ttalink_calibrate_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_calibrate_data_t* calibrate_data, bool nocrc)
{
    if(nocrc){
        return ttalink_calibrate_data_pack_nocrc(dst_addr, src_addr, msg, calibrate_data->BackUpGyro1Offset, calibrate_data->BackUpGyro1Gain, calibrate_data->BackUpAcc1Offset, calibrate_data->BackUpAcc1Gain, calibrate_data->BackUpGyro2Offset, calibrate_data->BackUpGyro2Gain, calibrate_data->BackUpAcc2Offset, calibrate_data->BackUpAcc2Gain, calibrate_data->BackUpGyro3Offset, calibrate_data->BackUpGyro3Gain, calibrate_data->BackUpAcc3Offset, calibrate_data->BackUpAcc3Gain);
    }else{
        return ttalink_calibrate_data_pack(dst_addr, src_addr, msg, calibrate_data->BackUpGyro1Offset, calibrate_data->BackUpGyro1Gain, calibrate_data->BackUpAcc1Offset, calibrate_data->BackUpAcc1Gain, calibrate_data->BackUpGyro2Offset, calibrate_data->BackUpGyro2Gain, calibrate_data->BackUpAcc2Offset, calibrate_data->BackUpAcc2Gain, calibrate_data->BackUpGyro3Offset, calibrate_data->BackUpGyro3Gain, calibrate_data->BackUpAcc3Offset, calibrate_data->BackUpAcc3Gain);
    }
    
}

/**
 * @brief Encode a calibrate_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_calibrate_data_t* calibrate_data)
{
    return _ttalink_calibrate_data_encode(dst_addr, src_addr, msg, calibrate_data, false);
}

/**
 * @brief Encode a calibrate_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_calibrate_data_t* calibrate_data)
{
    return _ttalink_calibrate_data_encode(dst_addr, src_addr, msg, calibrate_data, true);
}


static inline uint16_t _ttalink_calibrate_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_calibrate_data_t* calibrate_data, bool nocrc)
{
    if(nocrc){
        return ttalink_calibrate_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, calibrate_data->BackUpGyro1Offset, calibrate_data->BackUpGyro1Gain, calibrate_data->BackUpAcc1Offset, calibrate_data->BackUpAcc1Gain, calibrate_data->BackUpGyro2Offset, calibrate_data->BackUpGyro2Gain, calibrate_data->BackUpAcc2Offset, calibrate_data->BackUpAcc2Gain, calibrate_data->BackUpGyro3Offset, calibrate_data->BackUpGyro3Gain, calibrate_data->BackUpAcc3Offset, calibrate_data->BackUpAcc3Gain);
    }else{
        return ttalink_calibrate_data_pack_chan(dst_addr, src_addr, chan, msg, calibrate_data->BackUpGyro1Offset, calibrate_data->BackUpGyro1Gain, calibrate_data->BackUpAcc1Offset, calibrate_data->BackUpAcc1Gain, calibrate_data->BackUpGyro2Offset, calibrate_data->BackUpGyro2Gain, calibrate_data->BackUpAcc2Offset, calibrate_data->BackUpAcc2Gain, calibrate_data->BackUpGyro3Offset, calibrate_data->BackUpGyro3Gain, calibrate_data->BackUpAcc3Offset, calibrate_data->BackUpAcc3Gain);
    }
}

/**
 * @brief Encode a calibrate_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_calibrate_data_t* calibrate_data)
{
    return _ttalink_calibrate_data_encode_chan(dst_addr, src_addr, chan, msg, calibrate_data, false);
}

/**
 * @brief Encode a calibrate_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param calibrate_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_calibrate_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_calibrate_data_t* calibrate_data)
{
    return _ttalink_calibrate_data_encode_chan(dst_addr, src_addr, chan, msg, calibrate_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_calibrate_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CALIBRATE_DATA_LEN];

    _tta_put_float_array(buf, 0, BackUpGyro1Offset, 3);
    _tta_put_float_array(buf, 12, BackUpGyro1Gain, 3);
    _tta_put_float_array(buf, 24, BackUpAcc1Offset, 3);
    _tta_put_float_array(buf, 36, BackUpAcc1Gain, 3);
    _tta_put_float_array(buf, 48, BackUpGyro2Offset, 3);
    _tta_put_float_array(buf, 60, BackUpGyro2Gain, 3);
    _tta_put_float_array(buf, 72, BackUpAcc2Offset, 3);
    _tta_put_float_array(buf, 84, BackUpAcc2Gain, 3);
    _tta_put_float_array(buf, 96, BackUpGyro3Offset, 3);
    _tta_put_float_array(buf, 108, BackUpGyro3Gain, 3);
    _tta_put_float_array(buf, 120, BackUpAcc3Offset, 3);
    _tta_put_float_array(buf, 132, BackUpAcc3Gain, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA, buf, TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_CRC, nocrc);
#else
    ttalink_calibrate_data_t packet;

    tta_array_memcpy(packet.BackUpGyro1Offset, BackUpGyro1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro1Gain, BackUpGyro1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc1Offset, BackUpAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc1Gain, BackUpAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro2Offset, BackUpGyro2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro2Gain, BackUpGyro2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc2Offset, BackUpAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc2Gain, BackUpAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro3Offset, BackUpGyro3Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpGyro3Gain, BackUpGyro3Gain, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc3Offset, BackUpAcc3Offset, sizeof(float)*3);
    tta_array_memcpy(packet.BackUpAcc3Gain, BackUpAcc3Gain, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA, (const char *)&packet, TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a calibrate_data message
 * @param chan TTAlink channel to send the message
 *
 * @param BackUpGyro1Offset This is the offset of backup gyro1.
 * @param BackUpGyro1Gain This is the gain of backup gyro1.
 * @param BackUpAcc1Offset This is the offset of backup acc1.
 * @param BackUpAcc1Gain This is the gain of backup acc1.
 * @param BackUpGyro2Offset This is the offset of backup gyro2.
 * @param BackUpGyro2Gain This is the gain of backup gyro2.
 * @param BackUpAcc2Offset This is the offset of backup acc2.
 * @param BackUpAcc2Gain This is the gain of backup acc2.
 * @param BackUpGyro3Offset This is the offset of backup gyro3.
 * @param BackUpGyro3Gain This is the gain of backup gyro3.
 * @param BackUpAcc3Offset This is the offset of backup acc3.
 * @param BackUpAcc3Gain This is the gain of backup acc3.
 */
static inline void ttalink_calibrate_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain)
{
    _ttalink_calibrate_data_send(dst_addr, src_addr, chan, BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, false);
}

/**
 * @brief Send a calibrate_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param BackUpGyro1Offset This is the offset of backup gyro1.
 * @param BackUpGyro1Gain This is the gain of backup gyro1.
 * @param BackUpAcc1Offset This is the offset of backup acc1.
 * @param BackUpAcc1Gain This is the gain of backup acc1.
 * @param BackUpGyro2Offset This is the offset of backup gyro2.
 * @param BackUpGyro2Gain This is the gain of backup gyro2.
 * @param BackUpAcc2Offset This is the offset of backup acc2.
 * @param BackUpAcc2Gain This is the gain of backup acc2.
 * @param BackUpGyro3Offset This is the offset of backup gyro3.
 * @param BackUpGyro3Gain This is the gain of backup gyro3.
 * @param BackUpAcc3Offset This is the offset of backup acc3.
 * @param BackUpAcc3Gain This is the gain of backup acc3.
 */
static inline void ttalink_calibrate_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain)
{
    _ttalink_calibrate_data_send(dst_addr, src_addr, chan, BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, true);
}


static inline void _ttalink_calibrate_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_calibrate_data_t* calibrate_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_calibrate_data_send_nocrc(dst_addr, src_addr, chan, calibrate_data->BackUpGyro1Offset, calibrate_data->BackUpGyro1Gain, calibrate_data->BackUpAcc1Offset, calibrate_data->BackUpAcc1Gain, calibrate_data->BackUpGyro2Offset, calibrate_data->BackUpGyro2Gain, calibrate_data->BackUpAcc2Offset, calibrate_data->BackUpAcc2Gain, calibrate_data->BackUpGyro3Offset, calibrate_data->BackUpGyro3Gain, calibrate_data->BackUpAcc3Offset, calibrate_data->BackUpAcc3Gain);
    }else{
        ttalink_calibrate_data_send(dst_addr, src_addr, chan, calibrate_data->BackUpGyro1Offset, calibrate_data->BackUpGyro1Gain, calibrate_data->BackUpAcc1Offset, calibrate_data->BackUpAcc1Gain, calibrate_data->BackUpGyro2Offset, calibrate_data->BackUpGyro2Gain, calibrate_data->BackUpAcc2Offset, calibrate_data->BackUpAcc2Gain, calibrate_data->BackUpGyro3Offset, calibrate_data->BackUpGyro3Gain, calibrate_data->BackUpAcc3Offset, calibrate_data->BackUpAcc3Gain);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA, (const char *)calibrate_data, TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a calibrate_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_calibrate_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_calibrate_data_t* calibrate_data)
{
    _ttalink_calibrate_data_send_struct(dst_addr, src_addr, chan, calibrate_data, false);
}

/**
 * @brief Send a calibrate_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_calibrate_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_calibrate_data_t* calibrate_data)
{
    _ttalink_calibrate_data_send_struct(dst_addr, src_addr, chan, calibrate_data, true);
}

#if TTALINK_MSG_ID_CALIBRATE_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_calibrate_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_float_array(buf, 0, BackUpGyro1Offset, 3);
    _tta_put_float_array(buf, 12, BackUpGyro1Gain, 3);
    _tta_put_float_array(buf, 24, BackUpAcc1Offset, 3);
    _tta_put_float_array(buf, 36, BackUpAcc1Gain, 3);
    _tta_put_float_array(buf, 48, BackUpGyro2Offset, 3);
    _tta_put_float_array(buf, 60, BackUpGyro2Gain, 3);
    _tta_put_float_array(buf, 72, BackUpAcc2Offset, 3);
    _tta_put_float_array(buf, 84, BackUpAcc2Gain, 3);
    _tta_put_float_array(buf, 96, BackUpGyro3Offset, 3);
    _tta_put_float_array(buf, 108, BackUpGyro3Gain, 3);
    _tta_put_float_array(buf, 120, BackUpAcc3Offset, 3);
    _tta_put_float_array(buf, 132, BackUpAcc3Gain, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA, buf, TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_CRC, nocrc);
#else
    ttalink_calibrate_data_t *packet = (ttalink_calibrate_data_t *)msgbuf;

    tta_array_memcpy(packet->BackUpGyro1Offset, BackUpGyro1Offset, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpGyro1Gain, BackUpGyro1Gain, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAcc1Offset, BackUpAcc1Offset, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAcc1Gain, BackUpAcc1Gain, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpGyro2Offset, BackUpGyro2Offset, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpGyro2Gain, BackUpGyro2Gain, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAcc2Offset, BackUpAcc2Offset, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAcc2Gain, BackUpAcc2Gain, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpGyro3Offset, BackUpGyro3Offset, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpGyro3Gain, BackUpGyro3Gain, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAcc3Offset, BackUpAcc3Offset, sizeof(float)*3);
    tta_array_memcpy(packet->BackUpAcc3Gain, BackUpAcc3Gain, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CALIBRATE_DATA, (const char *)packet, TTALINK_MSG_ID_CALIBRATE_DATA_MIN_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_LEN, TTALINK_MSG_ID_CALIBRATE_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_calibrate_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain)
{
    _ttalink_calibrate_data_send_buf(dst_addr, src_addr, msgbuf, chan, BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_calibrate_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *BackUpGyro1Offset, const float *BackUpGyro1Gain, const float *BackUpAcc1Offset, const float *BackUpAcc1Gain, const float *BackUpGyro2Offset, const float *BackUpGyro2Gain, const float *BackUpAcc2Offset, const float *BackUpAcc2Gain, const float *BackUpGyro3Offset, const float *BackUpGyro3Gain, const float *BackUpAcc3Offset, const float *BackUpAcc3Gain)
{
    _ttalink_calibrate_data_send_buf(dst_addr, src_addr, msgbuf, chan, BackUpGyro1Offset, BackUpGyro1Gain, BackUpAcc1Offset, BackUpAcc1Gain, BackUpGyro2Offset, BackUpGyro2Gain, BackUpAcc2Offset, BackUpAcc2Gain, BackUpGyro3Offset, BackUpGyro3Gain, BackUpAcc3Offset, BackUpAcc3Gain, true);
}
#endif

#endif

// MESSAGE CALIBRATE_DATA UNPACKING


/**
 * @brief Get field BackUpGyro1Offset from calibrate_data message
 *
 * @return This is the offset of backup gyro1.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpGyro1Offset(const ttalink_message_t* msg, float *BackUpGyro1Offset)
{
    return _TTA_RETURN_float_array(msg, BackUpGyro1Offset, 3,  0);
}

/**
 * @brief Get field BackUpGyro1Gain from calibrate_data message
 *
 * @return This is the gain of backup gyro1.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpGyro1Gain(const ttalink_message_t* msg, float *BackUpGyro1Gain)
{
    return _TTA_RETURN_float_array(msg, BackUpGyro1Gain, 3,  12);
}

/**
 * @brief Get field BackUpAcc1Offset from calibrate_data message
 *
 * @return This is the offset of backup acc1.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpAcc1Offset(const ttalink_message_t* msg, float *BackUpAcc1Offset)
{
    return _TTA_RETURN_float_array(msg, BackUpAcc1Offset, 3,  24);
}

/**
 * @brief Get field BackUpAcc1Gain from calibrate_data message
 *
 * @return This is the gain of backup acc1.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpAcc1Gain(const ttalink_message_t* msg, float *BackUpAcc1Gain)
{
    return _TTA_RETURN_float_array(msg, BackUpAcc1Gain, 3,  36);
}

/**
 * @brief Get field BackUpGyro2Offset from calibrate_data message
 *
 * @return This is the offset of backup gyro2.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpGyro2Offset(const ttalink_message_t* msg, float *BackUpGyro2Offset)
{
    return _TTA_RETURN_float_array(msg, BackUpGyro2Offset, 3,  48);
}

/**
 * @brief Get field BackUpGyro2Gain from calibrate_data message
 *
 * @return This is the gain of backup gyro2.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpGyro2Gain(const ttalink_message_t* msg, float *BackUpGyro2Gain)
{
    return _TTA_RETURN_float_array(msg, BackUpGyro2Gain, 3,  60);
}

/**
 * @brief Get field BackUpAcc2Offset from calibrate_data message
 *
 * @return This is the offset of backup acc2.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpAcc2Offset(const ttalink_message_t* msg, float *BackUpAcc2Offset)
{
    return _TTA_RETURN_float_array(msg, BackUpAcc2Offset, 3,  72);
}

/**
 * @brief Get field BackUpAcc2Gain from calibrate_data message
 *
 * @return This is the gain of backup acc2.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpAcc2Gain(const ttalink_message_t* msg, float *BackUpAcc2Gain)
{
    return _TTA_RETURN_float_array(msg, BackUpAcc2Gain, 3,  84);
}

/**
 * @brief Get field BackUpGyro3Offset from calibrate_data message
 *
 * @return This is the offset of backup gyro3.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpGyro3Offset(const ttalink_message_t* msg, float *BackUpGyro3Offset)
{
    return _TTA_RETURN_float_array(msg, BackUpGyro3Offset, 3,  96);
}

/**
 * @brief Get field BackUpGyro3Gain from calibrate_data message
 *
 * @return This is the gain of backup gyro3.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpGyro3Gain(const ttalink_message_t* msg, float *BackUpGyro3Gain)
{
    return _TTA_RETURN_float_array(msg, BackUpGyro3Gain, 3,  108);
}

/**
 * @brief Get field BackUpAcc3Offset from calibrate_data message
 *
 * @return This is the offset of backup acc3.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpAcc3Offset(const ttalink_message_t* msg, float *BackUpAcc3Offset)
{
    return _TTA_RETURN_float_array(msg, BackUpAcc3Offset, 3,  120);
}

/**
 * @brief Get field BackUpAcc3Gain from calibrate_data message
 *
 * @return This is the gain of backup acc3.
 */
static inline uint16_t ttalink_calibrate_data_get_BackUpAcc3Gain(const ttalink_message_t* msg, float *BackUpAcc3Gain)
{
    return _TTA_RETURN_float_array(msg, BackUpAcc3Gain, 3,  132);
}

/**
 * @brief Decode a calibrate_data message into a struct
 *
 * @param msg The message to decode
 * @param calibrate_data C-struct to decode the message contents into
 */
static inline void ttalink_calibrate_data_decode(const ttalink_message_t* msg, ttalink_calibrate_data_t* calibrate_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_calibrate_data_get_BackUpGyro1Offset(msg, calibrate_data->BackUpGyro1Offset);
    ttalink_calibrate_data_get_BackUpGyro1Gain(msg, calibrate_data->BackUpGyro1Gain);
    ttalink_calibrate_data_get_BackUpAcc1Offset(msg, calibrate_data->BackUpAcc1Offset);
    ttalink_calibrate_data_get_BackUpAcc1Gain(msg, calibrate_data->BackUpAcc1Gain);
    ttalink_calibrate_data_get_BackUpGyro2Offset(msg, calibrate_data->BackUpGyro2Offset);
    ttalink_calibrate_data_get_BackUpGyro2Gain(msg, calibrate_data->BackUpGyro2Gain);
    ttalink_calibrate_data_get_BackUpAcc2Offset(msg, calibrate_data->BackUpAcc2Offset);
    ttalink_calibrate_data_get_BackUpAcc2Gain(msg, calibrate_data->BackUpAcc2Gain);
    ttalink_calibrate_data_get_BackUpGyro3Offset(msg, calibrate_data->BackUpGyro3Offset);
    ttalink_calibrate_data_get_BackUpGyro3Gain(msg, calibrate_data->BackUpGyro3Gain);
    ttalink_calibrate_data_get_BackUpAcc3Offset(msg, calibrate_data->BackUpAcc3Offset);
    ttalink_calibrate_data_get_BackUpAcc3Gain(msg, calibrate_data->BackUpAcc3Gain);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_CALIBRATE_DATA_LEN? msg->len : TTALINK_MSG_ID_CALIBRATE_DATA_LEN;
        memset(calibrate_data, 0, TTALINK_MSG_ID_CALIBRATE_DATA_LEN);
    memcpy(calibrate_data, _TTA_PAYLOAD(msg), len);
#endif
}
