#pragma once
// MESSAGE GENERAL_POINTS_COMMAND PACKING

#define TTALINK_MSG_ID_GENERAL_POINTS_COMMAND 4112

TTAPACKED(
typedef struct __ttalink_general_points_command_t {
 uint8_t ack; /*< 回应*/
 int32_t type; /*< 类型*/
 int32_t num; /*< 编号*/
 int32_t cmd; /*< 命令*/
 int32_t index; /*< 索引*/
 int32_t count; /*< 数量*/
 double x; /*< x*/
 double y; /*< y*/
 double z; /*< z*/
 int32_t reserve1; /*< Reserve1*/
 int32_t reserve2; /*< Reserve2*/
 int64_t reserve3; /*< Reserve3*/
 int64_t reserve4; /*< Reserve4*/
 double reserve5; /*< Reserve5*/
 double reserve6; /*< Reserve6*/
 double reserve7; /*< Reserve7*/
 double reserve8; /*< Reserve8*/
}) ttalink_general_points_command_t;

#define TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN 101
#define TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN 101
#define TTALINK_MSG_ID_4112_LEN 101
#define TTALINK_MSG_ID_4112_MIN_LEN 101

#define TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC 163
#define TTALINK_MSG_ID_4112_CRC 163



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_POINTS_COMMAND { \
    4112, \
    "GENERAL_POINTS_COMMAND", \
    17, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_points_command_t, ack) }, \
         { "type", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_general_points_command_t, type) }, \
         { "num", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_general_points_command_t, num) }, \
         { "cmd", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_general_points_command_t, cmd) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 13, offsetof(ttalink_general_points_command_t, index) }, \
         { "count", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_general_points_command_t, count) }, \
         { "x", NULL, TTALINK_TYPE_DOUBLE, 0, 21, offsetof(ttalink_general_points_command_t, x) }, \
         { "y", NULL, TTALINK_TYPE_DOUBLE, 0, 29, offsetof(ttalink_general_points_command_t, y) }, \
         { "z", NULL, TTALINK_TYPE_DOUBLE, 0, 37, offsetof(ttalink_general_points_command_t, z) }, \
         { "reserve1", NULL, TTALINK_TYPE_INT32_T, 0, 45, offsetof(ttalink_general_points_command_t, reserve1) }, \
         { "reserve2", NULL, TTALINK_TYPE_INT32_T, 0, 49, offsetof(ttalink_general_points_command_t, reserve2) }, \
         { "reserve3", NULL, TTALINK_TYPE_INT64_T, 0, 53, offsetof(ttalink_general_points_command_t, reserve3) }, \
         { "reserve4", NULL, TTALINK_TYPE_INT64_T, 0, 61, offsetof(ttalink_general_points_command_t, reserve4) }, \
         { "reserve5", NULL, TTALINK_TYPE_DOUBLE, 0, 69, offsetof(ttalink_general_points_command_t, reserve5) }, \
         { "reserve6", NULL, TTALINK_TYPE_DOUBLE, 0, 77, offsetof(ttalink_general_points_command_t, reserve6) }, \
         { "reserve7", NULL, TTALINK_TYPE_DOUBLE, 0, 85, offsetof(ttalink_general_points_command_t, reserve7) }, \
         { "reserve8", NULL, TTALINK_TYPE_DOUBLE, 0, 93, offsetof(ttalink_general_points_command_t, reserve8) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_POINTS_COMMAND { \
    "GENERAL_POINTS_COMMAND", \
    17, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_points_command_t, ack) }, \
         { "type", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_general_points_command_t, type) }, \
         { "num", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_general_points_command_t, num) }, \
         { "cmd", NULL, TTALINK_TYPE_INT32_T, 0, 9, offsetof(ttalink_general_points_command_t, cmd) }, \
         { "index", NULL, TTALINK_TYPE_INT32_T, 0, 13, offsetof(ttalink_general_points_command_t, index) }, \
         { "count", NULL, TTALINK_TYPE_INT32_T, 0, 17, offsetof(ttalink_general_points_command_t, count) }, \
         { "x", NULL, TTALINK_TYPE_DOUBLE, 0, 21, offsetof(ttalink_general_points_command_t, x) }, \
         { "y", NULL, TTALINK_TYPE_DOUBLE, 0, 29, offsetof(ttalink_general_points_command_t, y) }, \
         { "z", NULL, TTALINK_TYPE_DOUBLE, 0, 37, offsetof(ttalink_general_points_command_t, z) }, \
         { "reserve1", NULL, TTALINK_TYPE_INT32_T, 0, 45, offsetof(ttalink_general_points_command_t, reserve1) }, \
         { "reserve2", NULL, TTALINK_TYPE_INT32_T, 0, 49, offsetof(ttalink_general_points_command_t, reserve2) }, \
         { "reserve3", NULL, TTALINK_TYPE_INT64_T, 0, 53, offsetof(ttalink_general_points_command_t, reserve3) }, \
         { "reserve4", NULL, TTALINK_TYPE_INT64_T, 0, 61, offsetof(ttalink_general_points_command_t, reserve4) }, \
         { "reserve5", NULL, TTALINK_TYPE_DOUBLE, 0, 69, offsetof(ttalink_general_points_command_t, reserve5) }, \
         { "reserve6", NULL, TTALINK_TYPE_DOUBLE, 0, 77, offsetof(ttalink_general_points_command_t, reserve6) }, \
         { "reserve7", NULL, TTALINK_TYPE_DOUBLE, 0, 85, offsetof(ttalink_general_points_command_t, reserve7) }, \
         { "reserve8", NULL, TTALINK_TYPE_DOUBLE, 0, 93, offsetof(ttalink_general_points_command_t, reserve8) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_points_command_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int32_t(buf, 5, num);
    _tta_put_int32_t(buf, 9, cmd);
    _tta_put_int32_t(buf, 13, index);
    _tta_put_int32_t(buf, 17, count);
    _tta_put_double(buf, 21, x);
    _tta_put_double(buf, 29, y);
    _tta_put_double(buf, 37, z);
    _tta_put_int32_t(buf, 45, reserve1);
    _tta_put_int32_t(buf, 49, reserve2);
    _tta_put_int64_t(buf, 53, reserve3);
    _tta_put_int64_t(buf, 61, reserve4);
    _tta_put_double(buf, 69, reserve5);
    _tta_put_double(buf, 77, reserve6);
    _tta_put_double(buf, 85, reserve7);
    _tta_put_double(buf, 93, reserve8);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN);
#else
    ttalink_general_points_command_t packet;
    packet.ack = ack;
    packet.type = type;
    packet.num = num;
    packet.cmd = cmd;
    packet.index = index;
    packet.count = count;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    packet.reserve4 = reserve4;
    packet.reserve5 = reserve5;
    packet.reserve6 = reserve6;
    packet.reserve7 = reserve7;
    packet.reserve8 = reserve8;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_POINTS_COMMAND;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC, nocrc);
}

/**
 * @brief Pack a general_points_command message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack 回应
 * @param type 类型
 * @param num 编号
 * @param cmd 命令
 * @param index 索引
 * @param count 数量
 * @param x x
 * @param y y
 * @param z z
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_points_command_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    return _ttalink_general_points_command_pack(dst_addr, src_addr, msg,  ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/**
 * @brief Pack a general_points_command message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack 回应
 * @param type 类型
 * @param num 编号
 * @param cmd 命令
 * @param index 索引
 * @param count 数量
 * @param x x
 * @param y y
 * @param z z
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_points_command_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    return _ttalink_general_points_command_pack(dst_addr, src_addr, msg,  ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}


static inline uint16_t _ttalink_general_points_command_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,int32_t type,int32_t num,int32_t cmd,int32_t index,int32_t count,double x,double y,double z,int32_t reserve1,int32_t reserve2,int64_t reserve3,int64_t reserve4,double reserve5,double reserve6,double reserve7,double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int32_t(buf, 5, num);
    _tta_put_int32_t(buf, 9, cmd);
    _tta_put_int32_t(buf, 13, index);
    _tta_put_int32_t(buf, 17, count);
    _tta_put_double(buf, 21, x);
    _tta_put_double(buf, 29, y);
    _tta_put_double(buf, 37, z);
    _tta_put_int32_t(buf, 45, reserve1);
    _tta_put_int32_t(buf, 49, reserve2);
    _tta_put_int64_t(buf, 53, reserve3);
    _tta_put_int64_t(buf, 61, reserve4);
    _tta_put_double(buf, 69, reserve5);
    _tta_put_double(buf, 77, reserve6);
    _tta_put_double(buf, 85, reserve7);
    _tta_put_double(buf, 93, reserve8);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN);
#else
    ttalink_general_points_command_t packet;
    packet.ack = ack;
    packet.type = type;
    packet.num = num;
    packet.cmd = cmd;
    packet.index = index;
    packet.count = count;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    packet.reserve4 = reserve4;
    packet.reserve5 = reserve5;
    packet.reserve6 = reserve6;
    packet.reserve7 = reserve7;
    packet.reserve8 = reserve8;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_POINTS_COMMAND;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC, nocrc);
}

/**
 * @brief Pack a general_points_command message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack 回应
 * @param type 类型
 * @param num 编号
 * @param cmd 命令
 * @param index 索引
 * @param count 数量
 * @param x x
 * @param y y
 * @param z z
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_points_command_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,int32_t type,int32_t num,int32_t cmd,int32_t index,int32_t count,double x,double y,double z,int32_t reserve1,int32_t reserve2,int64_t reserve3,int64_t reserve4,double reserve5,double reserve6,double reserve7,double reserve8)
{
    return _ttalink_general_points_command_pack_chan(dst_addr, src_addr, chan, msg,  ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/**
 * @brief Pack a general_points_command message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack 回应
 * @param type 类型
 * @param num 编号
 * @param cmd 命令
 * @param index 索引
 * @param count 数量
 * @param x x
 * @param y y
 * @param z z
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_points_command_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,int32_t type,int32_t num,int32_t cmd,int32_t index,int32_t count,double x,double y,double z,int32_t reserve1,int32_t reserve2,int64_t reserve3,int64_t reserve4,double reserve5,double reserve6,double reserve7,double reserve8)
{
    return _ttalink_general_points_command_pack_chan(dst_addr, src_addr, chan, msg,  ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}


static inline uint16_t _ttalink_general_points_command_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_points_command_t* general_points_command, bool nocrc)
{
    if(nocrc){
        return ttalink_general_points_command_pack_nocrc(dst_addr, src_addr, msg, general_points_command->ack, general_points_command->type, general_points_command->num, general_points_command->cmd, general_points_command->index, general_points_command->count, general_points_command->x, general_points_command->y, general_points_command->z, general_points_command->reserve1, general_points_command->reserve2, general_points_command->reserve3, general_points_command->reserve4, general_points_command->reserve5, general_points_command->reserve6, general_points_command->reserve7, general_points_command->reserve8);
    }else{
        return ttalink_general_points_command_pack(dst_addr, src_addr, msg, general_points_command->ack, general_points_command->type, general_points_command->num, general_points_command->cmd, general_points_command->index, general_points_command->count, general_points_command->x, general_points_command->y, general_points_command->z, general_points_command->reserve1, general_points_command->reserve2, general_points_command->reserve3, general_points_command->reserve4, general_points_command->reserve5, general_points_command->reserve6, general_points_command->reserve7, general_points_command->reserve8);
    }
    
}

/**
 * @brief Encode a general_points_command struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_points_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_points_command_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_points_command_t* general_points_command)
{
    return _ttalink_general_points_command_encode(dst_addr, src_addr, msg, general_points_command, false);
}

/**
 * @brief Encode a general_points_command struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_points_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_points_command_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_points_command_t* general_points_command)
{
    return _ttalink_general_points_command_encode(dst_addr, src_addr, msg, general_points_command, true);
}


static inline uint16_t _ttalink_general_points_command_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_points_command_t* general_points_command, bool nocrc)
{
    if(nocrc){
        return ttalink_general_points_command_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_points_command->ack, general_points_command->type, general_points_command->num, general_points_command->cmd, general_points_command->index, general_points_command->count, general_points_command->x, general_points_command->y, general_points_command->z, general_points_command->reserve1, general_points_command->reserve2, general_points_command->reserve3, general_points_command->reserve4, general_points_command->reserve5, general_points_command->reserve6, general_points_command->reserve7, general_points_command->reserve8);
    }else{
        return ttalink_general_points_command_pack_chan(dst_addr, src_addr, chan, msg, general_points_command->ack, general_points_command->type, general_points_command->num, general_points_command->cmd, general_points_command->index, general_points_command->count, general_points_command->x, general_points_command->y, general_points_command->z, general_points_command->reserve1, general_points_command->reserve2, general_points_command->reserve3, general_points_command->reserve4, general_points_command->reserve5, general_points_command->reserve6, general_points_command->reserve7, general_points_command->reserve8);
    }
}

/**
 * @brief Encode a general_points_command struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_points_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_points_command_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_points_command_t* general_points_command)
{
    return _ttalink_general_points_command_encode_chan(dst_addr, src_addr, chan, msg, general_points_command, false);
}

/**
 * @brief Encode a general_points_command struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_points_command C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_points_command_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_points_command_t* general_points_command)
{
    return _ttalink_general_points_command_encode_chan(dst_addr, src_addr, chan, msg, general_points_command, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_points_command_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int32_t(buf, 5, num);
    _tta_put_int32_t(buf, 9, cmd);
    _tta_put_int32_t(buf, 13, index);
    _tta_put_int32_t(buf, 17, count);
    _tta_put_double(buf, 21, x);
    _tta_put_double(buf, 29, y);
    _tta_put_double(buf, 37, z);
    _tta_put_int32_t(buf, 45, reserve1);
    _tta_put_int32_t(buf, 49, reserve2);
    _tta_put_int64_t(buf, 53, reserve3);
    _tta_put_int64_t(buf, 61, reserve4);
    _tta_put_double(buf, 69, reserve5);
    _tta_put_double(buf, 77, reserve6);
    _tta_put_double(buf, 85, reserve7);
    _tta_put_double(buf, 93, reserve8);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND, buf, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC, nocrc);
#else
    ttalink_general_points_command_t packet;
    packet.ack = ack;
    packet.type = type;
    packet.num = num;
    packet.cmd = cmd;
    packet.index = index;
    packet.count = count;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    packet.reserve4 = reserve4;
    packet.reserve5 = reserve5;
    packet.reserve6 = reserve6;
    packet.reserve7 = reserve7;
    packet.reserve8 = reserve8;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND, (const char *)&packet, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_points_command message
 * @param chan TTAlink channel to send the message
 *
 * @param ack 回应
 * @param type 类型
 * @param num 编号
 * @param cmd 命令
 * @param index 索引
 * @param count 数量
 * @param x x
 * @param y y
 * @param z z
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 */
static inline void ttalink_general_points_command_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_points_command_send(dst_addr, src_addr, chan, ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/**
 * @brief Send a general_points_command message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack 回应
 * @param type 类型
 * @param num 编号
 * @param cmd 命令
 * @param index 索引
 * @param count 数量
 * @param x x
 * @param y y
 * @param z z
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 */
static inline void ttalink_general_points_command_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_points_command_send(dst_addr, src_addr, chan, ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}


static inline void _ttalink_general_points_command_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_points_command_t* general_points_command, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_points_command_send_nocrc(dst_addr, src_addr, chan, general_points_command->ack, general_points_command->type, general_points_command->num, general_points_command->cmd, general_points_command->index, general_points_command->count, general_points_command->x, general_points_command->y, general_points_command->z, general_points_command->reserve1, general_points_command->reserve2, general_points_command->reserve3, general_points_command->reserve4, general_points_command->reserve5, general_points_command->reserve6, general_points_command->reserve7, general_points_command->reserve8);
    }else{
        ttalink_general_points_command_send(dst_addr, src_addr, chan, general_points_command->ack, general_points_command->type, general_points_command->num, general_points_command->cmd, general_points_command->index, general_points_command->count, general_points_command->x, general_points_command->y, general_points_command->z, general_points_command->reserve1, general_points_command->reserve2, general_points_command->reserve3, general_points_command->reserve4, general_points_command->reserve5, general_points_command->reserve6, general_points_command->reserve7, general_points_command->reserve8);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND, (const char *)general_points_command, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_points_command message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_points_command_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_points_command_t* general_points_command)
{
    _ttalink_general_points_command_send_struct(dst_addr, src_addr, chan, general_points_command, false);
}

/**
 * @brief Send a general_points_command message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_points_command_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_points_command_t* general_points_command)
{
    _ttalink_general_points_command_send_struct(dst_addr, src_addr, chan, general_points_command, true);
}

#if TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_points_command_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int32_t(buf, 5, num);
    _tta_put_int32_t(buf, 9, cmd);
    _tta_put_int32_t(buf, 13, index);
    _tta_put_int32_t(buf, 17, count);
    _tta_put_double(buf, 21, x);
    _tta_put_double(buf, 29, y);
    _tta_put_double(buf, 37, z);
    _tta_put_int32_t(buf, 45, reserve1);
    _tta_put_int32_t(buf, 49, reserve2);
    _tta_put_int64_t(buf, 53, reserve3);
    _tta_put_int64_t(buf, 61, reserve4);
    _tta_put_double(buf, 69, reserve5);
    _tta_put_double(buf, 77, reserve6);
    _tta_put_double(buf, 85, reserve7);
    _tta_put_double(buf, 93, reserve8);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND, buf, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC, nocrc);
#else
    ttalink_general_points_command_t *packet = (ttalink_general_points_command_t *)msgbuf;
    packet->ack = ack;
    packet->type = type;
    packet->num = num;
    packet->cmd = cmd;
    packet->index = index;
    packet->count = count;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->reserve1 = reserve1;
    packet->reserve2 = reserve2;
    packet->reserve3 = reserve3;
    packet->reserve4 = reserve4;
    packet->reserve5 = reserve5;
    packet->reserve6 = reserve6;
    packet->reserve7 = reserve7;
    packet->reserve8 = reserve8;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND, (const char *)packet, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_MIN_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_points_command_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_points_command_send_buf(dst_addr, src_addr, msgbuf, chan, ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_points_command_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, int32_t type, int32_t num, int32_t cmd, int32_t index, int32_t count, double x, double y, double z, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_points_command_send_buf(dst_addr, src_addr, msgbuf, chan, ack, type, num, cmd, index, count, x, y, z, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}
#endif

#endif

// MESSAGE GENERAL_POINTS_COMMAND UNPACKING


/**
 * @brief Get field ack from general_points_command message
 *
 * @return 回应
 */
static inline uint8_t ttalink_general_points_command_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field type from general_points_command message
 *
 * @return 类型
 */
static inline int32_t ttalink_general_points_command_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  1);
}

/**
 * @brief Get field num from general_points_command message
 *
 * @return 编号
 */
static inline int32_t ttalink_general_points_command_get_num(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  5);
}

/**
 * @brief Get field cmd from general_points_command message
 *
 * @return 命令
 */
static inline int32_t ttalink_general_points_command_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  9);
}

/**
 * @brief Get field index from general_points_command message
 *
 * @return 索引
 */
static inline int32_t ttalink_general_points_command_get_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  13);
}

/**
 * @brief Get field count from general_points_command message
 *
 * @return 数量
 */
static inline int32_t ttalink_general_points_command_get_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  17);
}

/**
 * @brief Get field x from general_points_command message
 *
 * @return x
 */
static inline double ttalink_general_points_command_get_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  21);
}

/**
 * @brief Get field y from general_points_command message
 *
 * @return y
 */
static inline double ttalink_general_points_command_get_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  29);
}

/**
 * @brief Get field z from general_points_command message
 *
 * @return z
 */
static inline double ttalink_general_points_command_get_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  37);
}

/**
 * @brief Get field reserve1 from general_points_command message
 *
 * @return Reserve1
 */
static inline int32_t ttalink_general_points_command_get_reserve1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  45);
}

/**
 * @brief Get field reserve2 from general_points_command message
 *
 * @return Reserve2
 */
static inline int32_t ttalink_general_points_command_get_reserve2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  49);
}

/**
 * @brief Get field reserve3 from general_points_command message
 *
 * @return Reserve3
 */
static inline int64_t ttalink_general_points_command_get_reserve3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  53);
}

/**
 * @brief Get field reserve4 from general_points_command message
 *
 * @return Reserve4
 */
static inline int64_t ttalink_general_points_command_get_reserve4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  61);
}

/**
 * @brief Get field reserve5 from general_points_command message
 *
 * @return Reserve5
 */
static inline double ttalink_general_points_command_get_reserve5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  69);
}

/**
 * @brief Get field reserve6 from general_points_command message
 *
 * @return Reserve6
 */
static inline double ttalink_general_points_command_get_reserve6(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  77);
}

/**
 * @brief Get field reserve7 from general_points_command message
 *
 * @return Reserve7
 */
static inline double ttalink_general_points_command_get_reserve7(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  85);
}

/**
 * @brief Get field reserve8 from general_points_command message
 *
 * @return Reserve8
 */
static inline double ttalink_general_points_command_get_reserve8(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  93);
}

/**
 * @brief Decode a general_points_command message into a struct
 *
 * @param msg The message to decode
 * @param general_points_command C-struct to decode the message contents into
 */
static inline void ttalink_general_points_command_decode(const ttalink_message_t* msg, ttalink_general_points_command_t* general_points_command)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_points_command->ack = ttalink_general_points_command_get_ack(msg);
    general_points_command->type = ttalink_general_points_command_get_type(msg);
    general_points_command->num = ttalink_general_points_command_get_num(msg);
    general_points_command->cmd = ttalink_general_points_command_get_cmd(msg);
    general_points_command->index = ttalink_general_points_command_get_index(msg);
    general_points_command->count = ttalink_general_points_command_get_count(msg);
    general_points_command->x = ttalink_general_points_command_get_x(msg);
    general_points_command->y = ttalink_general_points_command_get_y(msg);
    general_points_command->z = ttalink_general_points_command_get_z(msg);
    general_points_command->reserve1 = ttalink_general_points_command_get_reserve1(msg);
    general_points_command->reserve2 = ttalink_general_points_command_get_reserve2(msg);
    general_points_command->reserve3 = ttalink_general_points_command_get_reserve3(msg);
    general_points_command->reserve4 = ttalink_general_points_command_get_reserve4(msg);
    general_points_command->reserve5 = ttalink_general_points_command_get_reserve5(msg);
    general_points_command->reserve6 = ttalink_general_points_command_get_reserve6(msg);
    general_points_command->reserve7 = ttalink_general_points_command_get_reserve7(msg);
    general_points_command->reserve8 = ttalink_general_points_command_get_reserve8(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN? msg->len : TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN;
        memset(general_points_command, 0, TTALINK_MSG_ID_GENERAL_POINTS_COMMAND_LEN);
    memcpy(general_points_command, _TTA_PAYLOAD(msg), len);
#endif
}
