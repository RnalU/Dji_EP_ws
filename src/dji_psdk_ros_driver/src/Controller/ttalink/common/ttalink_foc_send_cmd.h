#pragma once
// MESSAGE FOC_SEND_CMD PACKING

#define TTALINK_MSG_ID_FOC_SEND_CMD 2010

TTAPACKED(
typedef struct __ttalink_foc_send_cmd_t {
 uint32_t update_time; /*< system time*/
 uint8_t foc_cmd; /*< control command*/
 int16_t speed_ctrl_data[8]; /*< motor speed ctrl target input*/
 int16_t torque_ctrl_data[8]; /*< motor torque ctrl target input*/
 int8_t led_ctrl_data[3]; /*< motor led ctrl target input(RGB)*/
}) ttalink_foc_send_cmd_t;

#define TTALINK_MSG_ID_FOC_SEND_CMD_LEN 40
#define TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN 40
#define TTALINK_MSG_ID_2010_LEN 40
#define TTALINK_MSG_ID_2010_MIN_LEN 40

#define TTALINK_MSG_ID_FOC_SEND_CMD_CRC 88
#define TTALINK_MSG_ID_2010_CRC 88

#define TTALINK_MSG_FOC_SEND_CMD_FIELD_SPEED_CTRL_DATA_LEN 8
#define TTALINK_MSG_FOC_SEND_CMD_FIELD_TORQUE_CTRL_DATA_LEN 8
#define TTALINK_MSG_FOC_SEND_CMD_FIELD_LED_CTRL_DATA_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_FOC_SEND_CMD { \
    2010, \
    "FOC_SEND_CMD", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_send_cmd_t, update_time) }, \
         { "foc_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_send_cmd_t, foc_cmd) }, \
         { "speed_ctrl_data", NULL, TTALINK_TYPE_INT16_T, 8, 5, offsetof(ttalink_foc_send_cmd_t, speed_ctrl_data) }, \
         { "torque_ctrl_data", NULL, TTALINK_TYPE_INT16_T, 8, 21, offsetof(ttalink_foc_send_cmd_t, torque_ctrl_data) }, \
         { "led_ctrl_data", NULL, TTALINK_TYPE_INT8_T, 3, 37, offsetof(ttalink_foc_send_cmd_t, led_ctrl_data) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_FOC_SEND_CMD { \
    "FOC_SEND_CMD", \
    5, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_foc_send_cmd_t, update_time) }, \
         { "foc_cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_foc_send_cmd_t, foc_cmd) }, \
         { "speed_ctrl_data", NULL, TTALINK_TYPE_INT16_T, 8, 5, offsetof(ttalink_foc_send_cmd_t, speed_ctrl_data) }, \
         { "torque_ctrl_data", NULL, TTALINK_TYPE_INT16_T, 8, 21, offsetof(ttalink_foc_send_cmd_t, torque_ctrl_data) }, \
         { "led_ctrl_data", NULL, TTALINK_TYPE_INT8_T, 3, 37, offsetof(ttalink_foc_send_cmd_t, led_ctrl_data) }, \
         } \
}
#endif


static inline uint16_t _ttalink_foc_send_cmd_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_SEND_CMD_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, foc_cmd);
    _tta_put_int16_t_array(buf, 5, speed_ctrl_data, 8);
    _tta_put_int16_t_array(buf, 21, torque_ctrl_data, 8);
    _tta_put_int8_t_array(buf, 37, led_ctrl_data, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_SEND_CMD_LEN);
#else
    ttalink_foc_send_cmd_t packet;
    packet.update_time = update_time;
    packet.foc_cmd = foc_cmd;
    tta_array_memcpy(packet.speed_ctrl_data, speed_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet.torque_ctrl_data, torque_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet.led_ctrl_data, led_ctrl_data, sizeof(int8_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_SEND_CMD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_SEND_CMD;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_CRC, nocrc);
}

/**
 * @brief Pack a foc_send_cmd message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param foc_cmd control command
 * @param speed_ctrl_data motor speed ctrl target input
 * @param torque_ctrl_data motor torque ctrl target input
 * @param led_ctrl_data motor led ctrl target input(RGB)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_send_cmd_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data)
{
    return _ttalink_foc_send_cmd_pack(dst_addr, src_addr, msg,  update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, false);
}

/**
 * @brief Pack a foc_send_cmd message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time system time
 * @param foc_cmd control command
 * @param speed_ctrl_data motor speed ctrl target input
 * @param torque_ctrl_data motor torque ctrl target input
 * @param led_ctrl_data motor led ctrl target input(RGB)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_send_cmd_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data)
{
    return _ttalink_foc_send_cmd_pack(dst_addr, src_addr, msg,  update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, true);
}


static inline uint16_t _ttalink_foc_send_cmd_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t foc_cmd,const int16_t *speed_ctrl_data,const int16_t *torque_ctrl_data,const int8_t *led_ctrl_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_SEND_CMD_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, foc_cmd);
    _tta_put_int16_t_array(buf, 5, speed_ctrl_data, 8);
    _tta_put_int16_t_array(buf, 21, torque_ctrl_data, 8);
    _tta_put_int8_t_array(buf, 37, led_ctrl_data, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_FOC_SEND_CMD_LEN);
#else
    ttalink_foc_send_cmd_t packet;
    packet.update_time = update_time;
    packet.foc_cmd = foc_cmd;
    tta_array_memcpy(packet.speed_ctrl_data, speed_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet.torque_ctrl_data, torque_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet.led_ctrl_data, led_ctrl_data, sizeof(int8_t)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_FOC_SEND_CMD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_FOC_SEND_CMD;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_CRC, nocrc);
}

/**
 * @brief Pack a foc_send_cmd message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param foc_cmd control command
 * @param speed_ctrl_data motor speed ctrl target input
 * @param torque_ctrl_data motor torque ctrl target input
 * @param led_ctrl_data motor led ctrl target input(RGB)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_send_cmd_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t foc_cmd,const int16_t *speed_ctrl_data,const int16_t *torque_ctrl_data,const int8_t *led_ctrl_data)
{
    return _ttalink_foc_send_cmd_pack_chan(dst_addr, src_addr, chan, msg,  update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, false);
}

/**
 * @brief Pack a foc_send_cmd message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time system time
 * @param foc_cmd control command
 * @param speed_ctrl_data motor speed ctrl target input
 * @param torque_ctrl_data motor torque ctrl target input
 * @param led_ctrl_data motor led ctrl target input(RGB)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_foc_send_cmd_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t foc_cmd,const int16_t *speed_ctrl_data,const int16_t *torque_ctrl_data,const int8_t *led_ctrl_data)
{
    return _ttalink_foc_send_cmd_pack_chan(dst_addr, src_addr, chan, msg,  update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, true);
}


static inline uint16_t _ttalink_foc_send_cmd_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_send_cmd_t* foc_send_cmd, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_send_cmd_pack_nocrc(dst_addr, src_addr, msg, foc_send_cmd->update_time, foc_send_cmd->foc_cmd, foc_send_cmd->speed_ctrl_data, foc_send_cmd->torque_ctrl_data, foc_send_cmd->led_ctrl_data);
    }else{
        return ttalink_foc_send_cmd_pack(dst_addr, src_addr, msg, foc_send_cmd->update_time, foc_send_cmd->foc_cmd, foc_send_cmd->speed_ctrl_data, foc_send_cmd->torque_ctrl_data, foc_send_cmd->led_ctrl_data);
    }
    
}

/**
 * @brief Encode a foc_send_cmd struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_send_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_send_cmd_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_send_cmd_t* foc_send_cmd)
{
    return _ttalink_foc_send_cmd_encode(dst_addr, src_addr, msg, foc_send_cmd, false);
}

/**
 * @brief Encode a foc_send_cmd struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param foc_send_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_send_cmd_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_foc_send_cmd_t* foc_send_cmd)
{
    return _ttalink_foc_send_cmd_encode(dst_addr, src_addr, msg, foc_send_cmd, true);
}


static inline uint16_t _ttalink_foc_send_cmd_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_send_cmd_t* foc_send_cmd, bool nocrc)
{
    if(nocrc){
        return ttalink_foc_send_cmd_pack_chan_nocrc(dst_addr, src_addr, chan, msg, foc_send_cmd->update_time, foc_send_cmd->foc_cmd, foc_send_cmd->speed_ctrl_data, foc_send_cmd->torque_ctrl_data, foc_send_cmd->led_ctrl_data);
    }else{
        return ttalink_foc_send_cmd_pack_chan(dst_addr, src_addr, chan, msg, foc_send_cmd->update_time, foc_send_cmd->foc_cmd, foc_send_cmd->speed_ctrl_data, foc_send_cmd->torque_ctrl_data, foc_send_cmd->led_ctrl_data);
    }
}

/**
 * @brief Encode a foc_send_cmd struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_send_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_send_cmd_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_send_cmd_t* foc_send_cmd)
{
    return _ttalink_foc_send_cmd_encode_chan(dst_addr, src_addr, chan, msg, foc_send_cmd, false);
}

/**
 * @brief Encode a foc_send_cmd struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param foc_send_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_foc_send_cmd_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_foc_send_cmd_t* foc_send_cmd)
{
    return _ttalink_foc_send_cmd_encode_chan(dst_addr, src_addr, chan, msg, foc_send_cmd, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_foc_send_cmd_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_FOC_SEND_CMD_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, foc_cmd);
    _tta_put_int16_t_array(buf, 5, speed_ctrl_data, 8);
    _tta_put_int16_t_array(buf, 21, torque_ctrl_data, 8);
    _tta_put_int8_t_array(buf, 37, led_ctrl_data, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SEND_CMD, buf, TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_CRC, nocrc);
#else
    ttalink_foc_send_cmd_t packet;
    packet.update_time = update_time;
    packet.foc_cmd = foc_cmd;
    tta_array_memcpy(packet.speed_ctrl_data, speed_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet.torque_ctrl_data, torque_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet.led_ctrl_data, led_ctrl_data, sizeof(int8_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SEND_CMD, (const char *)&packet, TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_send_cmd message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param foc_cmd control command
 * @param speed_ctrl_data motor speed ctrl target input
 * @param torque_ctrl_data motor torque ctrl target input
 * @param led_ctrl_data motor led ctrl target input(RGB)
 */
static inline void ttalink_foc_send_cmd_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data)
{
    _ttalink_foc_send_cmd_send(dst_addr, src_addr, chan, update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, false);
}

/**
 * @brief Send a foc_send_cmd message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time system time
 * @param foc_cmd control command
 * @param speed_ctrl_data motor speed ctrl target input
 * @param torque_ctrl_data motor torque ctrl target input
 * @param led_ctrl_data motor led ctrl target input(RGB)
 */
static inline void ttalink_foc_send_cmd_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data)
{
    _ttalink_foc_send_cmd_send(dst_addr, src_addr, chan, update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, true);
}


static inline void _ttalink_foc_send_cmd_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_send_cmd_t* foc_send_cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_foc_send_cmd_send_nocrc(dst_addr, src_addr, chan, foc_send_cmd->update_time, foc_send_cmd->foc_cmd, foc_send_cmd->speed_ctrl_data, foc_send_cmd->torque_ctrl_data, foc_send_cmd->led_ctrl_data);
    }else{
        ttalink_foc_send_cmd_send(dst_addr, src_addr, chan, foc_send_cmd->update_time, foc_send_cmd->foc_cmd, foc_send_cmd->speed_ctrl_data, foc_send_cmd->torque_ctrl_data, foc_send_cmd->led_ctrl_data);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SEND_CMD, (const char *)foc_send_cmd, TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_CRC, nocrc);
#endif
}

/**
 * @brief Send a foc_send_cmd message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_send_cmd_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_send_cmd_t* foc_send_cmd)
{
    _ttalink_foc_send_cmd_send_struct(dst_addr, src_addr, chan, foc_send_cmd, false);
}

/**
 * @brief Send a foc_send_cmd message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_foc_send_cmd_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_foc_send_cmd_t* foc_send_cmd)
{
    _ttalink_foc_send_cmd_send_struct(dst_addr, src_addr, chan, foc_send_cmd, true);
}

#if TTALINK_MSG_ID_FOC_SEND_CMD_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_foc_send_cmd_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, foc_cmd);
    _tta_put_int16_t_array(buf, 5, speed_ctrl_data, 8);
    _tta_put_int16_t_array(buf, 21, torque_ctrl_data, 8);
    _tta_put_int8_t_array(buf, 37, led_ctrl_data, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SEND_CMD, buf, TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_CRC, nocrc);
#else
    ttalink_foc_send_cmd_t *packet = (ttalink_foc_send_cmd_t *)msgbuf;
    packet->update_time = update_time;
    packet->foc_cmd = foc_cmd;
    tta_array_memcpy(packet->speed_ctrl_data, speed_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet->torque_ctrl_data, torque_ctrl_data, sizeof(int16_t)*8);
    tta_array_memcpy(packet->led_ctrl_data, led_ctrl_data, sizeof(int8_t)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_FOC_SEND_CMD, (const char *)packet, TTALINK_MSG_ID_FOC_SEND_CMD_MIN_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_LEN, TTALINK_MSG_ID_FOC_SEND_CMD_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_foc_send_cmd_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data)
{
    _ttalink_foc_send_cmd_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_foc_send_cmd_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t foc_cmd, const int16_t *speed_ctrl_data, const int16_t *torque_ctrl_data, const int8_t *led_ctrl_data)
{
    _ttalink_foc_send_cmd_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, foc_cmd, speed_ctrl_data, torque_ctrl_data, led_ctrl_data, true);
}
#endif

#endif

// MESSAGE FOC_SEND_CMD UNPACKING


/**
 * @brief Get field update_time from foc_send_cmd message
 *
 * @return system time
 */
static inline uint32_t ttalink_foc_send_cmd_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field foc_cmd from foc_send_cmd message
 *
 * @return control command
 */
static inline uint8_t ttalink_foc_send_cmd_get_foc_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field speed_ctrl_data from foc_send_cmd message
 *
 * @return motor speed ctrl target input
 */
static inline uint16_t ttalink_foc_send_cmd_get_speed_ctrl_data(const ttalink_message_t* msg, int16_t *speed_ctrl_data)
{
    return _TTA_RETURN_int16_t_array(msg, speed_ctrl_data, 8,  5);
}

/**
 * @brief Get field torque_ctrl_data from foc_send_cmd message
 *
 * @return motor torque ctrl target input
 */
static inline uint16_t ttalink_foc_send_cmd_get_torque_ctrl_data(const ttalink_message_t* msg, int16_t *torque_ctrl_data)
{
    return _TTA_RETURN_int16_t_array(msg, torque_ctrl_data, 8,  21);
}

/**
 * @brief Get field led_ctrl_data from foc_send_cmd message
 *
 * @return motor led ctrl target input(RGB)
 */
static inline uint16_t ttalink_foc_send_cmd_get_led_ctrl_data(const ttalink_message_t* msg, int8_t *led_ctrl_data)
{
    return _TTA_RETURN_int8_t_array(msg, led_ctrl_data, 3,  37);
}

/**
 * @brief Decode a foc_send_cmd message into a struct
 *
 * @param msg The message to decode
 * @param foc_send_cmd C-struct to decode the message contents into
 */
static inline void ttalink_foc_send_cmd_decode(const ttalink_message_t* msg, ttalink_foc_send_cmd_t* foc_send_cmd)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    foc_send_cmd->update_time = ttalink_foc_send_cmd_get_update_time(msg);
    foc_send_cmd->foc_cmd = ttalink_foc_send_cmd_get_foc_cmd(msg);
    ttalink_foc_send_cmd_get_speed_ctrl_data(msg, foc_send_cmd->speed_ctrl_data);
    ttalink_foc_send_cmd_get_torque_ctrl_data(msg, foc_send_cmd->torque_ctrl_data);
    ttalink_foc_send_cmd_get_led_ctrl_data(msg, foc_send_cmd->led_ctrl_data);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_FOC_SEND_CMD_LEN? msg->len : TTALINK_MSG_ID_FOC_SEND_CMD_LEN;
        memset(foc_send_cmd, 0, TTALINK_MSG_ID_FOC_SEND_CMD_LEN);
    memcpy(foc_send_cmd, _TTA_PAYLOAD(msg), len);
#endif
}
