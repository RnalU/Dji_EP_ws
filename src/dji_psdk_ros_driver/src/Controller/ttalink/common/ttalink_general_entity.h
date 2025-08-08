#pragma once
// MESSAGE GENERAL_ENTITY PACKING

#define TTALINK_MSG_ID_GENERAL_ENTITY 4111

TTAPACKED(
typedef struct __ttalink_general_entity_t {
 uint8_t ack; /*< 回应*/
 int32_t type; /*< 类型*/
 int64_t sn; /*< SN*/
 int64_t ts; /*< TS*/
 int32_t reserve1; /*< Reserve1*/
 int32_t reserve2; /*< Reserve2*/
 int64_t reserve3; /*< Reserve3*/
 int64_t reserve4; /*< Reserve4*/
 double reserve5; /*< Reserve5*/
 double reserve6; /*< Reserve6*/
 double reserve7; /*< Reserve7*/
 double reserve8; /*< Reserve8*/
}) ttalink_general_entity_t;

#define TTALINK_MSG_ID_GENERAL_ENTITY_LEN 77
#define TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN 77
#define TTALINK_MSG_ID_4111_LEN 77
#define TTALINK_MSG_ID_4111_MIN_LEN 77

#define TTALINK_MSG_ID_GENERAL_ENTITY_CRC 105
#define TTALINK_MSG_ID_4111_CRC 105



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_ENTITY { \
    4111, \
    "GENERAL_ENTITY", \
    12, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_entity_t, ack) }, \
         { "type", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_general_entity_t, type) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 5, offsetof(ttalink_general_entity_t, sn) }, \
         { "ts", NULL, TTALINK_TYPE_INT64_T, 0, 13, offsetof(ttalink_general_entity_t, ts) }, \
         { "reserve1", NULL, TTALINK_TYPE_INT32_T, 0, 21, offsetof(ttalink_general_entity_t, reserve1) }, \
         { "reserve2", NULL, TTALINK_TYPE_INT32_T, 0, 25, offsetof(ttalink_general_entity_t, reserve2) }, \
         { "reserve3", NULL, TTALINK_TYPE_INT64_T, 0, 29, offsetof(ttalink_general_entity_t, reserve3) }, \
         { "reserve4", NULL, TTALINK_TYPE_INT64_T, 0, 37, offsetof(ttalink_general_entity_t, reserve4) }, \
         { "reserve5", NULL, TTALINK_TYPE_DOUBLE, 0, 45, offsetof(ttalink_general_entity_t, reserve5) }, \
         { "reserve6", NULL, TTALINK_TYPE_DOUBLE, 0, 53, offsetof(ttalink_general_entity_t, reserve6) }, \
         { "reserve7", NULL, TTALINK_TYPE_DOUBLE, 0, 61, offsetof(ttalink_general_entity_t, reserve7) }, \
         { "reserve8", NULL, TTALINK_TYPE_DOUBLE, 0, 69, offsetof(ttalink_general_entity_t, reserve8) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_ENTITY { \
    "GENERAL_ENTITY", \
    12, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_general_entity_t, ack) }, \
         { "type", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_general_entity_t, type) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 5, offsetof(ttalink_general_entity_t, sn) }, \
         { "ts", NULL, TTALINK_TYPE_INT64_T, 0, 13, offsetof(ttalink_general_entity_t, ts) }, \
         { "reserve1", NULL, TTALINK_TYPE_INT32_T, 0, 21, offsetof(ttalink_general_entity_t, reserve1) }, \
         { "reserve2", NULL, TTALINK_TYPE_INT32_T, 0, 25, offsetof(ttalink_general_entity_t, reserve2) }, \
         { "reserve3", NULL, TTALINK_TYPE_INT64_T, 0, 29, offsetof(ttalink_general_entity_t, reserve3) }, \
         { "reserve4", NULL, TTALINK_TYPE_INT64_T, 0, 37, offsetof(ttalink_general_entity_t, reserve4) }, \
         { "reserve5", NULL, TTALINK_TYPE_DOUBLE, 0, 45, offsetof(ttalink_general_entity_t, reserve5) }, \
         { "reserve6", NULL, TTALINK_TYPE_DOUBLE, 0, 53, offsetof(ttalink_general_entity_t, reserve6) }, \
         { "reserve7", NULL, TTALINK_TYPE_DOUBLE, 0, 61, offsetof(ttalink_general_entity_t, reserve7) }, \
         { "reserve8", NULL, TTALINK_TYPE_DOUBLE, 0, 69, offsetof(ttalink_general_entity_t, reserve8) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_entity_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_ENTITY_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_int64_t(buf, 13, ts);
    _tta_put_int32_t(buf, 21, reserve1);
    _tta_put_int32_t(buf, 25, reserve2);
    _tta_put_int64_t(buf, 29, reserve3);
    _tta_put_int64_t(buf, 37, reserve4);
    _tta_put_double(buf, 45, reserve5);
    _tta_put_double(buf, 53, reserve6);
    _tta_put_double(buf, 61, reserve7);
    _tta_put_double(buf, 69, reserve8);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_ENTITY_LEN);
#else
    ttalink_general_entity_t packet;
    packet.ack = ack;
    packet.type = type;
    packet.sn = sn;
    packet.ts = ts;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    packet.reserve4 = reserve4;
    packet.reserve5 = reserve5;
    packet.reserve6 = reserve6;
    packet.reserve7 = reserve7;
    packet.reserve8 = reserve8;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_ENTITY_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_ENTITY;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_CRC, nocrc);
}

/**
 * @brief Pack a general_entity message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack 回应
 * @param type 类型
 * @param sn SN
 * @param ts TS
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
static inline uint16_t ttalink_general_entity_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    return _ttalink_general_entity_pack(dst_addr, src_addr, msg,  ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/**
 * @brief Pack a general_entity message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack 回应
 * @param type 类型
 * @param sn SN
 * @param ts TS
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
static inline uint16_t ttalink_general_entity_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    return _ttalink_general_entity_pack(dst_addr, src_addr, msg,  ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}


static inline uint16_t _ttalink_general_entity_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,int32_t type,int64_t sn,int64_t ts,int32_t reserve1,int32_t reserve2,int64_t reserve3,int64_t reserve4,double reserve5,double reserve6,double reserve7,double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_ENTITY_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_int64_t(buf, 13, ts);
    _tta_put_int32_t(buf, 21, reserve1);
    _tta_put_int32_t(buf, 25, reserve2);
    _tta_put_int64_t(buf, 29, reserve3);
    _tta_put_int64_t(buf, 37, reserve4);
    _tta_put_double(buf, 45, reserve5);
    _tta_put_double(buf, 53, reserve6);
    _tta_put_double(buf, 61, reserve7);
    _tta_put_double(buf, 69, reserve8);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_ENTITY_LEN);
#else
    ttalink_general_entity_t packet;
    packet.ack = ack;
    packet.type = type;
    packet.sn = sn;
    packet.ts = ts;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    packet.reserve4 = reserve4;
    packet.reserve5 = reserve5;
    packet.reserve6 = reserve6;
    packet.reserve7 = reserve7;
    packet.reserve8 = reserve8;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_ENTITY_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_ENTITY;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_CRC, nocrc);
}

/**
 * @brief Pack a general_entity message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack 回应
 * @param type 类型
 * @param sn SN
 * @param ts TS
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
static inline uint16_t ttalink_general_entity_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,int32_t type,int64_t sn,int64_t ts,int32_t reserve1,int32_t reserve2,int64_t reserve3,int64_t reserve4,double reserve5,double reserve6,double reserve7,double reserve8)
{
    return _ttalink_general_entity_pack_chan(dst_addr, src_addr, chan, msg,  ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/**
 * @brief Pack a general_entity message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack 回应
 * @param type 类型
 * @param sn SN
 * @param ts TS
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
static inline uint16_t ttalink_general_entity_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,int32_t type,int64_t sn,int64_t ts,int32_t reserve1,int32_t reserve2,int64_t reserve3,int64_t reserve4,double reserve5,double reserve6,double reserve7,double reserve8)
{
    return _ttalink_general_entity_pack_chan(dst_addr, src_addr, chan, msg,  ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}


static inline uint16_t _ttalink_general_entity_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_entity_t* general_entity, bool nocrc)
{
    if(nocrc){
        return ttalink_general_entity_pack_nocrc(dst_addr, src_addr, msg, general_entity->ack, general_entity->type, general_entity->sn, general_entity->ts, general_entity->reserve1, general_entity->reserve2, general_entity->reserve3, general_entity->reserve4, general_entity->reserve5, general_entity->reserve6, general_entity->reserve7, general_entity->reserve8);
    }else{
        return ttalink_general_entity_pack(dst_addr, src_addr, msg, general_entity->ack, general_entity->type, general_entity->sn, general_entity->ts, general_entity->reserve1, general_entity->reserve2, general_entity->reserve3, general_entity->reserve4, general_entity->reserve5, general_entity->reserve6, general_entity->reserve7, general_entity->reserve8);
    }
    
}

/**
 * @brief Encode a general_entity struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_entity C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_entity_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_entity_t* general_entity)
{
    return _ttalink_general_entity_encode(dst_addr, src_addr, msg, general_entity, false);
}

/**
 * @brief Encode a general_entity struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_entity C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_entity_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_entity_t* general_entity)
{
    return _ttalink_general_entity_encode(dst_addr, src_addr, msg, general_entity, true);
}


static inline uint16_t _ttalink_general_entity_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_entity_t* general_entity, bool nocrc)
{
    if(nocrc){
        return ttalink_general_entity_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_entity->ack, general_entity->type, general_entity->sn, general_entity->ts, general_entity->reserve1, general_entity->reserve2, general_entity->reserve3, general_entity->reserve4, general_entity->reserve5, general_entity->reserve6, general_entity->reserve7, general_entity->reserve8);
    }else{
        return ttalink_general_entity_pack_chan(dst_addr, src_addr, chan, msg, general_entity->ack, general_entity->type, general_entity->sn, general_entity->ts, general_entity->reserve1, general_entity->reserve2, general_entity->reserve3, general_entity->reserve4, general_entity->reserve5, general_entity->reserve6, general_entity->reserve7, general_entity->reserve8);
    }
}

/**
 * @brief Encode a general_entity struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_entity C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_entity_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_entity_t* general_entity)
{
    return _ttalink_general_entity_encode_chan(dst_addr, src_addr, chan, msg, general_entity, false);
}

/**
 * @brief Encode a general_entity struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_entity C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_entity_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_entity_t* general_entity)
{
    return _ttalink_general_entity_encode_chan(dst_addr, src_addr, chan, msg, general_entity, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_entity_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_ENTITY_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_int64_t(buf, 13, ts);
    _tta_put_int32_t(buf, 21, reserve1);
    _tta_put_int32_t(buf, 25, reserve2);
    _tta_put_int64_t(buf, 29, reserve3);
    _tta_put_int64_t(buf, 37, reserve4);
    _tta_put_double(buf, 45, reserve5);
    _tta_put_double(buf, 53, reserve6);
    _tta_put_double(buf, 61, reserve7);
    _tta_put_double(buf, 69, reserve8);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_ENTITY, buf, TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_CRC, nocrc);
#else
    ttalink_general_entity_t packet;
    packet.ack = ack;
    packet.type = type;
    packet.sn = sn;
    packet.ts = ts;
    packet.reserve1 = reserve1;
    packet.reserve2 = reserve2;
    packet.reserve3 = reserve3;
    packet.reserve4 = reserve4;
    packet.reserve5 = reserve5;
    packet.reserve6 = reserve6;
    packet.reserve7 = reserve7;
    packet.reserve8 = reserve8;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_ENTITY, (const char *)&packet, TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_entity message
 * @param chan TTAlink channel to send the message
 *
 * @param ack 回应
 * @param type 类型
 * @param sn SN
 * @param ts TS
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 */
static inline void ttalink_general_entity_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_entity_send(dst_addr, src_addr, chan, ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/**
 * @brief Send a general_entity message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack 回应
 * @param type 类型
 * @param sn SN
 * @param ts TS
 * @param reserve1 Reserve1
 * @param reserve2 Reserve2
 * @param reserve3 Reserve3
 * @param reserve4 Reserve4
 * @param reserve5 Reserve5
 * @param reserve6 Reserve6
 * @param reserve7 Reserve7
 * @param reserve8 Reserve8
 */
static inline void ttalink_general_entity_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_entity_send(dst_addr, src_addr, chan, ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}


static inline void _ttalink_general_entity_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_entity_t* general_entity, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_entity_send_nocrc(dst_addr, src_addr, chan, general_entity->ack, general_entity->type, general_entity->sn, general_entity->ts, general_entity->reserve1, general_entity->reserve2, general_entity->reserve3, general_entity->reserve4, general_entity->reserve5, general_entity->reserve6, general_entity->reserve7, general_entity->reserve8);
    }else{
        ttalink_general_entity_send(dst_addr, src_addr, chan, general_entity->ack, general_entity->type, general_entity->sn, general_entity->ts, general_entity->reserve1, general_entity->reserve2, general_entity->reserve3, general_entity->reserve4, general_entity->reserve5, general_entity->reserve6, general_entity->reserve7, general_entity->reserve8);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_ENTITY, (const char *)general_entity, TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_entity message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_entity_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_entity_t* general_entity)
{
    _ttalink_general_entity_send_struct(dst_addr, src_addr, chan, general_entity, false);
}

/**
 * @brief Send a general_entity message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_entity_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_entity_t* general_entity)
{
    _ttalink_general_entity_send_struct(dst_addr, src_addr, chan, general_entity, true);
}

#if TTALINK_MSG_ID_GENERAL_ENTITY_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_entity_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_int32_t(buf, 1, type);
    _tta_put_int64_t(buf, 5, sn);
    _tta_put_int64_t(buf, 13, ts);
    _tta_put_int32_t(buf, 21, reserve1);
    _tta_put_int32_t(buf, 25, reserve2);
    _tta_put_int64_t(buf, 29, reserve3);
    _tta_put_int64_t(buf, 37, reserve4);
    _tta_put_double(buf, 45, reserve5);
    _tta_put_double(buf, 53, reserve6);
    _tta_put_double(buf, 61, reserve7);
    _tta_put_double(buf, 69, reserve8);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_ENTITY, buf, TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_CRC, nocrc);
#else
    ttalink_general_entity_t *packet = (ttalink_general_entity_t *)msgbuf;
    packet->ack = ack;
    packet->type = type;
    packet->sn = sn;
    packet->ts = ts;
    packet->reserve1 = reserve1;
    packet->reserve2 = reserve2;
    packet->reserve3 = reserve3;
    packet->reserve4 = reserve4;
    packet->reserve5 = reserve5;
    packet->reserve6 = reserve6;
    packet->reserve7 = reserve7;
    packet->reserve8 = reserve8;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_ENTITY, (const char *)packet, TTALINK_MSG_ID_GENERAL_ENTITY_MIN_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_LEN, TTALINK_MSG_ID_GENERAL_ENTITY_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_entity_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_entity_send_buf(dst_addr, src_addr, msgbuf, chan, ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_entity_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, int32_t type, int64_t sn, int64_t ts, int32_t reserve1, int32_t reserve2, int64_t reserve3, int64_t reserve4, double reserve5, double reserve6, double reserve7, double reserve8)
{
    _ttalink_general_entity_send_buf(dst_addr, src_addr, msgbuf, chan, ack, type, sn, ts, reserve1, reserve2, reserve3, reserve4, reserve5, reserve6, reserve7, reserve8, true);
}
#endif

#endif

// MESSAGE GENERAL_ENTITY UNPACKING


/**
 * @brief Get field ack from general_entity message
 *
 * @return 回应
 */
static inline uint8_t ttalink_general_entity_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field type from general_entity message
 *
 * @return 类型
 */
static inline int32_t ttalink_general_entity_get_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  1);
}

/**
 * @brief Get field sn from general_entity message
 *
 * @return SN
 */
static inline int64_t ttalink_general_entity_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  5);
}

/**
 * @brief Get field ts from general_entity message
 *
 * @return TS
 */
static inline int64_t ttalink_general_entity_get_ts(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  13);
}

/**
 * @brief Get field reserve1 from general_entity message
 *
 * @return Reserve1
 */
static inline int32_t ttalink_general_entity_get_reserve1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  21);
}

/**
 * @brief Get field reserve2 from general_entity message
 *
 * @return Reserve2
 */
static inline int32_t ttalink_general_entity_get_reserve2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  25);
}

/**
 * @brief Get field reserve3 from general_entity message
 *
 * @return Reserve3
 */
static inline int64_t ttalink_general_entity_get_reserve3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  29);
}

/**
 * @brief Get field reserve4 from general_entity message
 *
 * @return Reserve4
 */
static inline int64_t ttalink_general_entity_get_reserve4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  37);
}

/**
 * @brief Get field reserve5 from general_entity message
 *
 * @return Reserve5
 */
static inline double ttalink_general_entity_get_reserve5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  45);
}

/**
 * @brief Get field reserve6 from general_entity message
 *
 * @return Reserve6
 */
static inline double ttalink_general_entity_get_reserve6(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  53);
}

/**
 * @brief Get field reserve7 from general_entity message
 *
 * @return Reserve7
 */
static inline double ttalink_general_entity_get_reserve7(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  61);
}

/**
 * @brief Get field reserve8 from general_entity message
 *
 * @return Reserve8
 */
static inline double ttalink_general_entity_get_reserve8(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  69);
}

/**
 * @brief Decode a general_entity message into a struct
 *
 * @param msg The message to decode
 * @param general_entity C-struct to decode the message contents into
 */
static inline void ttalink_general_entity_decode(const ttalink_message_t* msg, ttalink_general_entity_t* general_entity)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_entity->ack = ttalink_general_entity_get_ack(msg);
    general_entity->type = ttalink_general_entity_get_type(msg);
    general_entity->sn = ttalink_general_entity_get_sn(msg);
    general_entity->ts = ttalink_general_entity_get_ts(msg);
    general_entity->reserve1 = ttalink_general_entity_get_reserve1(msg);
    general_entity->reserve2 = ttalink_general_entity_get_reserve2(msg);
    general_entity->reserve3 = ttalink_general_entity_get_reserve3(msg);
    general_entity->reserve4 = ttalink_general_entity_get_reserve4(msg);
    general_entity->reserve5 = ttalink_general_entity_get_reserve5(msg);
    general_entity->reserve6 = ttalink_general_entity_get_reserve6(msg);
    general_entity->reserve7 = ttalink_general_entity_get_reserve7(msg);
    general_entity->reserve8 = ttalink_general_entity_get_reserve8(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_ENTITY_LEN? msg->len : TTALINK_MSG_ID_GENERAL_ENTITY_LEN;
        memset(general_entity, 0, TTALINK_MSG_ID_GENERAL_ENTITY_LEN);
    memcpy(general_entity, _TTA_PAYLOAD(msg), len);
#endif
}
