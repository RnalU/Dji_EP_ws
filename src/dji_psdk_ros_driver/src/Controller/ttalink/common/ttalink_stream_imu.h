#pragma once
// MESSAGE STREAM_IMU PACKING

#define TTALINK_MSG_ID_STREAM_IMU 2145

TTAPACKED(
typedef struct __ttalink_stream_imu_t {
 float main_acc1[3]; /*<  .*/
 float main_acc2[3]; /*<  .*/
 float backup_acc[3]; /*<  .*/
 float main_gyro1[3]; /*<  .*/
 float main_gyro2[3]; /*<  .*/
 float backup_gyro[3]; /*<  .*/
 float switch_acc[3]; /*<  .*/
 float switch_gyro[3]; /*<  .*/
}) ttalink_stream_imu_t;

#define TTALINK_MSG_ID_STREAM_IMU_LEN 96
#define TTALINK_MSG_ID_STREAM_IMU_MIN_LEN 96
#define TTALINK_MSG_ID_2145_LEN 96
#define TTALINK_MSG_ID_2145_MIN_LEN 96

#define TTALINK_MSG_ID_STREAM_IMU_CRC 248
#define TTALINK_MSG_ID_2145_CRC 248

#define TTALINK_MSG_STREAM_IMU_FIELD_MAIN_ACC1_LEN 3
#define TTALINK_MSG_STREAM_IMU_FIELD_MAIN_ACC2_LEN 3
#define TTALINK_MSG_STREAM_IMU_FIELD_BACKUP_ACC_LEN 3
#define TTALINK_MSG_STREAM_IMU_FIELD_MAIN_GYRO1_LEN 3
#define TTALINK_MSG_STREAM_IMU_FIELD_MAIN_GYRO2_LEN 3
#define TTALINK_MSG_STREAM_IMU_FIELD_BACKUP_GYRO_LEN 3
#define TTALINK_MSG_STREAM_IMU_FIELD_SWITCH_ACC_LEN 3
#define TTALINK_MSG_STREAM_IMU_FIELD_SWITCH_GYRO_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_STREAM_IMU { \
    2145, \
    "STREAM_IMU", \
    8, \
    {  { "main_acc1", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_stream_imu_t, main_acc1) }, \
         { "main_acc2", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_stream_imu_t, main_acc2) }, \
         { "backup_acc", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_stream_imu_t, backup_acc) }, \
         { "main_gyro1", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_stream_imu_t, main_gyro1) }, \
         { "main_gyro2", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_stream_imu_t, main_gyro2) }, \
         { "backup_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 60, offsetof(ttalink_stream_imu_t, backup_gyro) }, \
         { "switch_acc", NULL, TTALINK_TYPE_FLOAT, 3, 72, offsetof(ttalink_stream_imu_t, switch_acc) }, \
         { "switch_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 84, offsetof(ttalink_stream_imu_t, switch_gyro) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_STREAM_IMU { \
    "STREAM_IMU", \
    8, \
    {  { "main_acc1", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_stream_imu_t, main_acc1) }, \
         { "main_acc2", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_stream_imu_t, main_acc2) }, \
         { "backup_acc", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_stream_imu_t, backup_acc) }, \
         { "main_gyro1", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_stream_imu_t, main_gyro1) }, \
         { "main_gyro2", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_stream_imu_t, main_gyro2) }, \
         { "backup_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 60, offsetof(ttalink_stream_imu_t, backup_gyro) }, \
         { "switch_acc", NULL, TTALINK_TYPE_FLOAT, 3, 72, offsetof(ttalink_stream_imu_t, switch_acc) }, \
         { "switch_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 84, offsetof(ttalink_stream_imu_t, switch_gyro) }, \
         } \
}
#endif


static inline uint16_t _ttalink_stream_imu_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_IMU_LEN];

    _tta_put_float_array(buf, 0, main_acc1, 3);
    _tta_put_float_array(buf, 12, main_acc2, 3);
    _tta_put_float_array(buf, 24, backup_acc, 3);
    _tta_put_float_array(buf, 36, main_gyro1, 3);
    _tta_put_float_array(buf, 48, main_gyro2, 3);
    _tta_put_float_array(buf, 60, backup_gyro, 3);
    _tta_put_float_array(buf, 72, switch_acc, 3);
    _tta_put_float_array(buf, 84, switch_gyro, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_IMU_LEN);
#else
    ttalink_stream_imu_t packet;

    tta_array_memcpy(packet.main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro, switch_gyro, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_IMU_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_IMU;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_STREAM_IMU_MIN_LEN, TTALINK_MSG_ID_STREAM_IMU_LEN, TTALINK_MSG_ID_STREAM_IMU_CRC, nocrc);
}

/**
 * @brief Pack a stream_imu message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param switch_acc  .
 * @param switch_gyro  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_imu_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro)
{
    return _ttalink_stream_imu_pack(dst_addr, src_addr, msg,  main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, false);
}

/**
 * @brief Pack a stream_imu message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param switch_acc  .
 * @param switch_gyro  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_imu_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro)
{
    return _ttalink_stream_imu_pack(dst_addr, src_addr, msg,  main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, true);
}


static inline uint16_t _ttalink_stream_imu_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *main_acc1,const float *main_acc2,const float *backup_acc,const float *main_gyro1,const float *main_gyro2,const float *backup_gyro,const float *switch_acc,const float *switch_gyro, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_IMU_LEN];

    _tta_put_float_array(buf, 0, main_acc1, 3);
    _tta_put_float_array(buf, 12, main_acc2, 3);
    _tta_put_float_array(buf, 24, backup_acc, 3);
    _tta_put_float_array(buf, 36, main_gyro1, 3);
    _tta_put_float_array(buf, 48, main_gyro2, 3);
    _tta_put_float_array(buf, 60, backup_gyro, 3);
    _tta_put_float_array(buf, 72, switch_acc, 3);
    _tta_put_float_array(buf, 84, switch_gyro, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_STREAM_IMU_LEN);
#else
    ttalink_stream_imu_t packet;

    tta_array_memcpy(packet.main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro, switch_gyro, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_STREAM_IMU_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_STREAM_IMU;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_IMU_MIN_LEN, TTALINK_MSG_ID_STREAM_IMU_LEN, TTALINK_MSG_ID_STREAM_IMU_CRC, nocrc);
}

/**
 * @brief Pack a stream_imu message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param switch_acc  .
 * @param switch_gyro  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_imu_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *main_acc1,const float *main_acc2,const float *backup_acc,const float *main_gyro1,const float *main_gyro2,const float *backup_gyro,const float *switch_acc,const float *switch_gyro)
{
    return _ttalink_stream_imu_pack_chan(dst_addr, src_addr, chan, msg,  main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, false);
}

/**
 * @brief Pack a stream_imu message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param switch_acc  .
 * @param switch_gyro  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_stream_imu_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *main_acc1,const float *main_acc2,const float *backup_acc,const float *main_gyro1,const float *main_gyro2,const float *backup_gyro,const float *switch_acc,const float *switch_gyro)
{
    return _ttalink_stream_imu_pack_chan(dst_addr, src_addr, chan, msg,  main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, true);
}


static inline uint16_t _ttalink_stream_imu_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_imu_t* stream_imu, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_imu_pack_nocrc(dst_addr, src_addr, msg, stream_imu->main_acc1, stream_imu->main_acc2, stream_imu->backup_acc, stream_imu->main_gyro1, stream_imu->main_gyro2, stream_imu->backup_gyro, stream_imu->switch_acc, stream_imu->switch_gyro);
    }else{
        return ttalink_stream_imu_pack(dst_addr, src_addr, msg, stream_imu->main_acc1, stream_imu->main_acc2, stream_imu->backup_acc, stream_imu->main_gyro1, stream_imu->main_gyro2, stream_imu->backup_gyro, stream_imu->switch_acc, stream_imu->switch_gyro);
    }
    
}

/**
 * @brief Encode a stream_imu struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_imu C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_imu_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_imu_t* stream_imu)
{
    return _ttalink_stream_imu_encode(dst_addr, src_addr, msg, stream_imu, false);
}

/**
 * @brief Encode a stream_imu struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param stream_imu C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_imu_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_stream_imu_t* stream_imu)
{
    return _ttalink_stream_imu_encode(dst_addr, src_addr, msg, stream_imu, true);
}


static inline uint16_t _ttalink_stream_imu_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_imu_t* stream_imu, bool nocrc)
{
    if(nocrc){
        return ttalink_stream_imu_pack_chan_nocrc(dst_addr, src_addr, chan, msg, stream_imu->main_acc1, stream_imu->main_acc2, stream_imu->backup_acc, stream_imu->main_gyro1, stream_imu->main_gyro2, stream_imu->backup_gyro, stream_imu->switch_acc, stream_imu->switch_gyro);
    }else{
        return ttalink_stream_imu_pack_chan(dst_addr, src_addr, chan, msg, stream_imu->main_acc1, stream_imu->main_acc2, stream_imu->backup_acc, stream_imu->main_gyro1, stream_imu->main_gyro2, stream_imu->backup_gyro, stream_imu->switch_acc, stream_imu->switch_gyro);
    }
}

/**
 * @brief Encode a stream_imu struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_imu C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_imu_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_imu_t* stream_imu)
{
    return _ttalink_stream_imu_encode_chan(dst_addr, src_addr, chan, msg, stream_imu, false);
}

/**
 * @brief Encode a stream_imu struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param stream_imu C-struct to read the message contents from
 */
static inline uint16_t ttalink_stream_imu_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_stream_imu_t* stream_imu)
{
    return _ttalink_stream_imu_encode_chan(dst_addr, src_addr, chan, msg, stream_imu, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_stream_imu_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_STREAM_IMU_LEN];

    _tta_put_float_array(buf, 0, main_acc1, 3);
    _tta_put_float_array(buf, 12, main_acc2, 3);
    _tta_put_float_array(buf, 24, backup_acc, 3);
    _tta_put_float_array(buf, 36, main_gyro1, 3);
    _tta_put_float_array(buf, 48, main_gyro2, 3);
    _tta_put_float_array(buf, 60, backup_gyro, 3);
    _tta_put_float_array(buf, 72, switch_acc, 3);
    _tta_put_float_array(buf, 84, switch_gyro, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_IMU, buf, TTALINK_MSG_ID_STREAM_IMU_MIN_LEN, TTALINK_MSG_ID_STREAM_IMU_LEN, TTALINK_MSG_ID_STREAM_IMU_CRC, nocrc);
#else
    ttalink_stream_imu_t packet;

    tta_array_memcpy(packet.main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet.switch_gyro, switch_gyro, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_IMU, (const char *)&packet, TTALINK_MSG_ID_STREAM_IMU_MIN_LEN, TTALINK_MSG_ID_STREAM_IMU_LEN, TTALINK_MSG_ID_STREAM_IMU_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_imu message
 * @param chan TTAlink channel to send the message
 *
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param switch_acc  .
 * @param switch_gyro  .
 */
static inline void ttalink_stream_imu_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro)
{
    _ttalink_stream_imu_send(dst_addr, src_addr, chan, main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, false);
}

/**
 * @brief Send a stream_imu message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param switch_acc  .
 * @param switch_gyro  .
 */
static inline void ttalink_stream_imu_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro)
{
    _ttalink_stream_imu_send(dst_addr, src_addr, chan, main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, true);
}


static inline void _ttalink_stream_imu_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_imu_t* stream_imu, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_stream_imu_send_nocrc(dst_addr, src_addr, chan, stream_imu->main_acc1, stream_imu->main_acc2, stream_imu->backup_acc, stream_imu->main_gyro1, stream_imu->main_gyro2, stream_imu->backup_gyro, stream_imu->switch_acc, stream_imu->switch_gyro);
    }else{
        ttalink_stream_imu_send(dst_addr, src_addr, chan, stream_imu->main_acc1, stream_imu->main_acc2, stream_imu->backup_acc, stream_imu->main_gyro1, stream_imu->main_gyro2, stream_imu->backup_gyro, stream_imu->switch_acc, stream_imu->switch_gyro);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_IMU, (const char *)stream_imu, TTALINK_MSG_ID_STREAM_IMU_MIN_LEN, TTALINK_MSG_ID_STREAM_IMU_LEN, TTALINK_MSG_ID_STREAM_IMU_CRC, nocrc);
#endif
}

/**
 * @brief Send a stream_imu message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_imu_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_imu_t* stream_imu)
{
    _ttalink_stream_imu_send_struct(dst_addr, src_addr, chan, stream_imu, false);
}

/**
 * @brief Send a stream_imu message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_stream_imu_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_stream_imu_t* stream_imu)
{
    _ttalink_stream_imu_send_struct(dst_addr, src_addr, chan, stream_imu, true);
}

#if TTALINK_MSG_ID_STREAM_IMU_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_stream_imu_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _tta_put_float_array(buf, 0, main_acc1, 3);
    _tta_put_float_array(buf, 12, main_acc2, 3);
    _tta_put_float_array(buf, 24, backup_acc, 3);
    _tta_put_float_array(buf, 36, main_gyro1, 3);
    _tta_put_float_array(buf, 48, main_gyro2, 3);
    _tta_put_float_array(buf, 60, backup_gyro, 3);
    _tta_put_float_array(buf, 72, switch_acc, 3);
    _tta_put_float_array(buf, 84, switch_gyro, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_IMU, buf, TTALINK_MSG_ID_STREAM_IMU_MIN_LEN, TTALINK_MSG_ID_STREAM_IMU_LEN, TTALINK_MSG_ID_STREAM_IMU_CRC, nocrc);
#else
    ttalink_stream_imu_t *packet = (ttalink_stream_imu_t *)msgbuf;

    tta_array_memcpy(packet->main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet->main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet->backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet->main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet->main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet->backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet->switch_acc, switch_acc, sizeof(float)*3);
    tta_array_memcpy(packet->switch_gyro, switch_gyro, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_STREAM_IMU, (const char *)packet, TTALINK_MSG_ID_STREAM_IMU_MIN_LEN, TTALINK_MSG_ID_STREAM_IMU_LEN, TTALINK_MSG_ID_STREAM_IMU_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_stream_imu_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro)
{
    _ttalink_stream_imu_send_buf(dst_addr, src_addr, msgbuf, chan, main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_stream_imu_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *switch_acc, const float *switch_gyro)
{
    _ttalink_stream_imu_send_buf(dst_addr, src_addr, msgbuf, chan, main_acc1, main_acc2, backup_acc, main_gyro1, main_gyro2, backup_gyro, switch_acc, switch_gyro, true);
}
#endif

#endif

// MESSAGE STREAM_IMU UNPACKING


/**
 * @brief Get field main_acc1 from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_main_acc1(const ttalink_message_t* msg, float *main_acc1)
{
    return _TTA_RETURN_float_array(msg, main_acc1, 3,  0);
}

/**
 * @brief Get field main_acc2 from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_main_acc2(const ttalink_message_t* msg, float *main_acc2)
{
    return _TTA_RETURN_float_array(msg, main_acc2, 3,  12);
}

/**
 * @brief Get field backup_acc from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_backup_acc(const ttalink_message_t* msg, float *backup_acc)
{
    return _TTA_RETURN_float_array(msg, backup_acc, 3,  24);
}

/**
 * @brief Get field main_gyro1 from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_main_gyro1(const ttalink_message_t* msg, float *main_gyro1)
{
    return _TTA_RETURN_float_array(msg, main_gyro1, 3,  36);
}

/**
 * @brief Get field main_gyro2 from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_main_gyro2(const ttalink_message_t* msg, float *main_gyro2)
{
    return _TTA_RETURN_float_array(msg, main_gyro2, 3,  48);
}

/**
 * @brief Get field backup_gyro from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_backup_gyro(const ttalink_message_t* msg, float *backup_gyro)
{
    return _TTA_RETURN_float_array(msg, backup_gyro, 3,  60);
}

/**
 * @brief Get field switch_acc from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_switch_acc(const ttalink_message_t* msg, float *switch_acc)
{
    return _TTA_RETURN_float_array(msg, switch_acc, 3,  72);
}

/**
 * @brief Get field switch_gyro from stream_imu message
 *
 * @return  .
 */
static inline uint16_t ttalink_stream_imu_get_switch_gyro(const ttalink_message_t* msg, float *switch_gyro)
{
    return _TTA_RETURN_float_array(msg, switch_gyro, 3,  84);
}

/**
 * @brief Decode a stream_imu message into a struct
 *
 * @param msg The message to decode
 * @param stream_imu C-struct to decode the message contents into
 */
static inline void ttalink_stream_imu_decode(const ttalink_message_t* msg, ttalink_stream_imu_t* stream_imu)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_stream_imu_get_main_acc1(msg, stream_imu->main_acc1);
    ttalink_stream_imu_get_main_acc2(msg, stream_imu->main_acc2);
    ttalink_stream_imu_get_backup_acc(msg, stream_imu->backup_acc);
    ttalink_stream_imu_get_main_gyro1(msg, stream_imu->main_gyro1);
    ttalink_stream_imu_get_main_gyro2(msg, stream_imu->main_gyro2);
    ttalink_stream_imu_get_backup_gyro(msg, stream_imu->backup_gyro);
    ttalink_stream_imu_get_switch_acc(msg, stream_imu->switch_acc);
    ttalink_stream_imu_get_switch_gyro(msg, stream_imu->switch_gyro);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_STREAM_IMU_LEN? msg->len : TTALINK_MSG_ID_STREAM_IMU_LEN;
        memset(stream_imu, 0, TTALINK_MSG_ID_STREAM_IMU_LEN);
    memcpy(stream_imu, _TTA_PAYLOAD(msg), len);
#endif
}
