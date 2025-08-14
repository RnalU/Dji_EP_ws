#pragma once
// MESSAGE WP_PROCESS_INDEX PACKING

#define TTALINK_MSG_ID_WP_PROCESS_INDEX 2231

TTAPACKED(
typedef struct __ttalink_wp_process_index_t {
 uint8_t action_wp_index; /*<  当前执行航线为第几条航线.*/
 int32_t total_wp_count; /*<  总的航点数目.*/
 int32_t target_wp_index; /*<  目标航点数目.*/
 uint8_t total_cycle_count; /*<  总的循环次数.*/
 uint8_t remain_cycle_count; /*<  剩余航线循环次数.*/
 uint8_t break_state; /*<  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.*/
 uint32_t wp_code; /*<  航线码.*/
 uint8_t wp_done_action; /*<  航线完成动作.*/
 uint8_t param3; /*<  预留.*/
 uint8_t param4; /*<  预留.*/
 uint8_t param5; /*<  预留.*/
 float param2; /*<  预留.*/
}) ttalink_wp_process_index_t;

#define TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN 24
#define TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN 24
#define TTALINK_MSG_ID_2231_LEN 24
#define TTALINK_MSG_ID_2231_MIN_LEN 24

#define TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC 194
#define TTALINK_MSG_ID_2231_CRC 194



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_WP_PROCESS_INDEX { \
    2231, \
    "WP_PROCESS_INDEX", \
    12, \
    {  { "action_wp_index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_wp_process_index_t, action_wp_index) }, \
         { "total_wp_count", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_wp_process_index_t, total_wp_count) }, \
         { "target_wp_index", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_wp_process_index_t, target_wp_index) }, \
         { "total_cycle_count", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_wp_process_index_t, total_cycle_count) }, \
         { "remain_cycle_count", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_wp_process_index_t, remain_cycle_count) }, \
         { "break_state", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_wp_process_index_t, break_state) }, \
         { "wp_code", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_wp_process_index_t, wp_code) }, \
         { "wp_done_action", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_wp_process_index_t, wp_done_action) }, \
         { "param3", NULL, TTALINK_TYPE_UINT8_T, 0, 17, offsetof(ttalink_wp_process_index_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_UINT8_T, 0, 18, offsetof(ttalink_wp_process_index_t, param4) }, \
         { "param5", NULL, TTALINK_TYPE_UINT8_T, 0, 19, offsetof(ttalink_wp_process_index_t, param5) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_wp_process_index_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_WP_PROCESS_INDEX { \
    "WP_PROCESS_INDEX", \
    12, \
    {  { "action_wp_index", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_wp_process_index_t, action_wp_index) }, \
         { "total_wp_count", NULL, TTALINK_TYPE_INT32_T, 0, 1, offsetof(ttalink_wp_process_index_t, total_wp_count) }, \
         { "target_wp_index", NULL, TTALINK_TYPE_INT32_T, 0, 5, offsetof(ttalink_wp_process_index_t, target_wp_index) }, \
         { "total_cycle_count", NULL, TTALINK_TYPE_UINT8_T, 0, 9, offsetof(ttalink_wp_process_index_t, total_cycle_count) }, \
         { "remain_cycle_count", NULL, TTALINK_TYPE_UINT8_T, 0, 10, offsetof(ttalink_wp_process_index_t, remain_cycle_count) }, \
         { "break_state", NULL, TTALINK_TYPE_UINT8_T, 0, 11, offsetof(ttalink_wp_process_index_t, break_state) }, \
         { "wp_code", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_wp_process_index_t, wp_code) }, \
         { "wp_done_action", NULL, TTALINK_TYPE_UINT8_T, 0, 16, offsetof(ttalink_wp_process_index_t, wp_done_action) }, \
         { "param3", NULL, TTALINK_TYPE_UINT8_T, 0, 17, offsetof(ttalink_wp_process_index_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_UINT8_T, 0, 18, offsetof(ttalink_wp_process_index_t, param4) }, \
         { "param5", NULL, TTALINK_TYPE_UINT8_T, 0, 19, offsetof(ttalink_wp_process_index_t, param5) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_wp_process_index_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_wp_process_index_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN];
    _tta_put_uint8_t(buf, 0, action_wp_index);
    _tta_put_int32_t(buf, 1, total_wp_count);
    _tta_put_int32_t(buf, 5, target_wp_index);
    _tta_put_uint8_t(buf, 9, total_cycle_count);
    _tta_put_uint8_t(buf, 10, remain_cycle_count);
    _tta_put_uint8_t(buf, 11, break_state);
    _tta_put_uint32_t(buf, 12, wp_code);
    _tta_put_uint8_t(buf, 16, wp_done_action);
    _tta_put_uint8_t(buf, 17, param3);
    _tta_put_uint8_t(buf, 18, param4);
    _tta_put_uint8_t(buf, 19, param5);
    _tta_put_float(buf, 20, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN);
#else
    ttalink_wp_process_index_t packet;
    packet.action_wp_index = action_wp_index;
    packet.total_wp_count = total_wp_count;
    packet.target_wp_index = target_wp_index;
    packet.total_cycle_count = total_cycle_count;
    packet.remain_cycle_count = remain_cycle_count;
    packet.break_state = break_state;
    packet.wp_code = wp_code;
    packet.wp_done_action = wp_done_action;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WP_PROCESS_INDEX;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC, nocrc);
}

/**
 * @brief Pack a wp_process_index message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param action_wp_index  当前执行航线为第几条航线.
 * @param total_wp_count  总的航点数目.
 * @param target_wp_index  目标航点数目.
 * @param total_cycle_count  总的循环次数.
 * @param remain_cycle_count  剩余航线循环次数.
 * @param break_state  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.
 * @param wp_code  航线码.
 * @param wp_done_action  航线完成动作.
 * @param param3  预留.
 * @param param4  预留.
 * @param param5  预留.
 * @param param2  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_process_index_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2)
{
    return _ttalink_wp_process_index_pack(dst_addr, src_addr, msg,  action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, false);
}

/**
 * @brief Pack a wp_process_index message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param action_wp_index  当前执行航线为第几条航线.
 * @param total_wp_count  总的航点数目.
 * @param target_wp_index  目标航点数目.
 * @param total_cycle_count  总的循环次数.
 * @param remain_cycle_count  剩余航线循环次数.
 * @param break_state  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.
 * @param wp_code  航线码.
 * @param wp_done_action  航线完成动作.
 * @param param3  预留.
 * @param param4  预留.
 * @param param5  预留.
 * @param param2  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_process_index_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2)
{
    return _ttalink_wp_process_index_pack(dst_addr, src_addr, msg,  action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, true);
}


static inline uint16_t _ttalink_wp_process_index_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t action_wp_index,int32_t total_wp_count,int32_t target_wp_index,uint8_t total_cycle_count,uint8_t remain_cycle_count,uint8_t break_state,uint32_t wp_code,uint8_t wp_done_action,uint8_t param3,uint8_t param4,uint8_t param5,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN];
    _tta_put_uint8_t(buf, 0, action_wp_index);
    _tta_put_int32_t(buf, 1, total_wp_count);
    _tta_put_int32_t(buf, 5, target_wp_index);
    _tta_put_uint8_t(buf, 9, total_cycle_count);
    _tta_put_uint8_t(buf, 10, remain_cycle_count);
    _tta_put_uint8_t(buf, 11, break_state);
    _tta_put_uint32_t(buf, 12, wp_code);
    _tta_put_uint8_t(buf, 16, wp_done_action);
    _tta_put_uint8_t(buf, 17, param3);
    _tta_put_uint8_t(buf, 18, param4);
    _tta_put_uint8_t(buf, 19, param5);
    _tta_put_float(buf, 20, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN);
#else
    ttalink_wp_process_index_t packet;
    packet.action_wp_index = action_wp_index;
    packet.total_wp_count = total_wp_count;
    packet.target_wp_index = target_wp_index;
    packet.total_cycle_count = total_cycle_count;
    packet.remain_cycle_count = remain_cycle_count;
    packet.break_state = break_state;
    packet.wp_code = wp_code;
    packet.wp_done_action = wp_done_action;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WP_PROCESS_INDEX;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC, nocrc);
}

/**
 * @brief Pack a wp_process_index message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param action_wp_index  当前执行航线为第几条航线.
 * @param total_wp_count  总的航点数目.
 * @param target_wp_index  目标航点数目.
 * @param total_cycle_count  总的循环次数.
 * @param remain_cycle_count  剩余航线循环次数.
 * @param break_state  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.
 * @param wp_code  航线码.
 * @param wp_done_action  航线完成动作.
 * @param param3  预留.
 * @param param4  预留.
 * @param param5  预留.
 * @param param2  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_process_index_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t action_wp_index,int32_t total_wp_count,int32_t target_wp_index,uint8_t total_cycle_count,uint8_t remain_cycle_count,uint8_t break_state,uint32_t wp_code,uint8_t wp_done_action,uint8_t param3,uint8_t param4,uint8_t param5,float param2)
{
    return _ttalink_wp_process_index_pack_chan(dst_addr, src_addr, chan, msg,  action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, false);
}

/**
 * @brief Pack a wp_process_index message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param action_wp_index  当前执行航线为第几条航线.
 * @param total_wp_count  总的航点数目.
 * @param target_wp_index  目标航点数目.
 * @param total_cycle_count  总的循环次数.
 * @param remain_cycle_count  剩余航线循环次数.
 * @param break_state  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.
 * @param wp_code  航线码.
 * @param wp_done_action  航线完成动作.
 * @param param3  预留.
 * @param param4  预留.
 * @param param5  预留.
 * @param param2  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_process_index_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t action_wp_index,int32_t total_wp_count,int32_t target_wp_index,uint8_t total_cycle_count,uint8_t remain_cycle_count,uint8_t break_state,uint32_t wp_code,uint8_t wp_done_action,uint8_t param3,uint8_t param4,uint8_t param5,float param2)
{
    return _ttalink_wp_process_index_pack_chan(dst_addr, src_addr, chan, msg,  action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, true);
}


static inline uint16_t _ttalink_wp_process_index_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_process_index_t* wp_process_index, bool nocrc)
{
    if(nocrc){
        return ttalink_wp_process_index_pack_nocrc(dst_addr, src_addr, msg, wp_process_index->action_wp_index, wp_process_index->total_wp_count, wp_process_index->target_wp_index, wp_process_index->total_cycle_count, wp_process_index->remain_cycle_count, wp_process_index->break_state, wp_process_index->wp_code, wp_process_index->wp_done_action, wp_process_index->param3, wp_process_index->param4, wp_process_index->param5, wp_process_index->param2);
    }else{
        return ttalink_wp_process_index_pack(dst_addr, src_addr, msg, wp_process_index->action_wp_index, wp_process_index->total_wp_count, wp_process_index->target_wp_index, wp_process_index->total_cycle_count, wp_process_index->remain_cycle_count, wp_process_index->break_state, wp_process_index->wp_code, wp_process_index->wp_done_action, wp_process_index->param3, wp_process_index->param4, wp_process_index->param5, wp_process_index->param2);
    }
    
}

/**
 * @brief Encode a wp_process_index struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param wp_process_index C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_process_index_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_process_index_t* wp_process_index)
{
    return _ttalink_wp_process_index_encode(dst_addr, src_addr, msg, wp_process_index, false);
}

/**
 * @brief Encode a wp_process_index struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param wp_process_index C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_process_index_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_process_index_t* wp_process_index)
{
    return _ttalink_wp_process_index_encode(dst_addr, src_addr, msg, wp_process_index, true);
}


static inline uint16_t _ttalink_wp_process_index_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_process_index_t* wp_process_index, bool nocrc)
{
    if(nocrc){
        return ttalink_wp_process_index_pack_chan_nocrc(dst_addr, src_addr, chan, msg, wp_process_index->action_wp_index, wp_process_index->total_wp_count, wp_process_index->target_wp_index, wp_process_index->total_cycle_count, wp_process_index->remain_cycle_count, wp_process_index->break_state, wp_process_index->wp_code, wp_process_index->wp_done_action, wp_process_index->param3, wp_process_index->param4, wp_process_index->param5, wp_process_index->param2);
    }else{
        return ttalink_wp_process_index_pack_chan(dst_addr, src_addr, chan, msg, wp_process_index->action_wp_index, wp_process_index->total_wp_count, wp_process_index->target_wp_index, wp_process_index->total_cycle_count, wp_process_index->remain_cycle_count, wp_process_index->break_state, wp_process_index->wp_code, wp_process_index->wp_done_action, wp_process_index->param3, wp_process_index->param4, wp_process_index->param5, wp_process_index->param2);
    }
}

/**
 * @brief Encode a wp_process_index struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param wp_process_index C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_process_index_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_process_index_t* wp_process_index)
{
    return _ttalink_wp_process_index_encode_chan(dst_addr, src_addr, chan, msg, wp_process_index, false);
}

/**
 * @brief Encode a wp_process_index struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param wp_process_index C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_process_index_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_process_index_t* wp_process_index)
{
    return _ttalink_wp_process_index_encode_chan(dst_addr, src_addr, chan, msg, wp_process_index, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_wp_process_index_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN];
    _tta_put_uint8_t(buf, 0, action_wp_index);
    _tta_put_int32_t(buf, 1, total_wp_count);
    _tta_put_int32_t(buf, 5, target_wp_index);
    _tta_put_uint8_t(buf, 9, total_cycle_count);
    _tta_put_uint8_t(buf, 10, remain_cycle_count);
    _tta_put_uint8_t(buf, 11, break_state);
    _tta_put_uint32_t(buf, 12, wp_code);
    _tta_put_uint8_t(buf, 16, wp_done_action);
    _tta_put_uint8_t(buf, 17, param3);
    _tta_put_uint8_t(buf, 18, param4);
    _tta_put_uint8_t(buf, 19, param5);
    _tta_put_float(buf, 20, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_PROCESS_INDEX, buf, TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC, nocrc);
#else
    ttalink_wp_process_index_t packet;
    packet.action_wp_index = action_wp_index;
    packet.total_wp_count = total_wp_count;
    packet.target_wp_index = target_wp_index;
    packet.total_cycle_count = total_cycle_count;
    packet.remain_cycle_count = remain_cycle_count;
    packet.break_state = break_state;
    packet.wp_code = wp_code;
    packet.wp_done_action = wp_done_action;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_PROCESS_INDEX, (const char *)&packet, TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC, nocrc);
#endif
}

/**
 * @brief Send a wp_process_index message
 * @param chan TTAlink channel to send the message
 *
 * @param action_wp_index  当前执行航线为第几条航线.
 * @param total_wp_count  总的航点数目.
 * @param target_wp_index  目标航点数目.
 * @param total_cycle_count  总的循环次数.
 * @param remain_cycle_count  剩余航线循环次数.
 * @param break_state  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.
 * @param wp_code  航线码.
 * @param wp_done_action  航线完成动作.
 * @param param3  预留.
 * @param param4  预留.
 * @param param5  预留.
 * @param param2  预留.
 */
static inline void ttalink_wp_process_index_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2)
{
    _ttalink_wp_process_index_send(dst_addr, src_addr, chan, action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, false);
}

/**
 * @brief Send a wp_process_index message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param action_wp_index  当前执行航线为第几条航线.
 * @param total_wp_count  总的航点数目.
 * @param target_wp_index  目标航点数目.
 * @param total_cycle_count  总的循环次数.
 * @param remain_cycle_count  剩余航线循环次数.
 * @param break_state  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.
 * @param wp_code  航线码.
 * @param wp_done_action  航线完成动作.
 * @param param3  预留.
 * @param param4  预留.
 * @param param5  预留.
 * @param param2  预留.
 */
static inline void ttalink_wp_process_index_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2)
{
    _ttalink_wp_process_index_send(dst_addr, src_addr, chan, action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, true);
}


static inline void _ttalink_wp_process_index_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_process_index_t* wp_process_index, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_wp_process_index_send_nocrc(dst_addr, src_addr, chan, wp_process_index->action_wp_index, wp_process_index->total_wp_count, wp_process_index->target_wp_index, wp_process_index->total_cycle_count, wp_process_index->remain_cycle_count, wp_process_index->break_state, wp_process_index->wp_code, wp_process_index->wp_done_action, wp_process_index->param3, wp_process_index->param4, wp_process_index->param5, wp_process_index->param2);
    }else{
        ttalink_wp_process_index_send(dst_addr, src_addr, chan, wp_process_index->action_wp_index, wp_process_index->total_wp_count, wp_process_index->target_wp_index, wp_process_index->total_cycle_count, wp_process_index->remain_cycle_count, wp_process_index->break_state, wp_process_index->wp_code, wp_process_index->wp_done_action, wp_process_index->param3, wp_process_index->param4, wp_process_index->param5, wp_process_index->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_PROCESS_INDEX, (const char *)wp_process_index, TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC, nocrc);
#endif
}

/**
 * @brief Send a wp_process_index message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_wp_process_index_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_process_index_t* wp_process_index)
{
    _ttalink_wp_process_index_send_struct(dst_addr, src_addr, chan, wp_process_index, false);
}

/**
 * @brief Send a wp_process_index message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_wp_process_index_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_process_index_t* wp_process_index)
{
    _ttalink_wp_process_index_send_struct(dst_addr, src_addr, chan, wp_process_index, true);
}

#if TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_wp_process_index_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, action_wp_index);
    _tta_put_int32_t(buf, 1, total_wp_count);
    _tta_put_int32_t(buf, 5, target_wp_index);
    _tta_put_uint8_t(buf, 9, total_cycle_count);
    _tta_put_uint8_t(buf, 10, remain_cycle_count);
    _tta_put_uint8_t(buf, 11, break_state);
    _tta_put_uint32_t(buf, 12, wp_code);
    _tta_put_uint8_t(buf, 16, wp_done_action);
    _tta_put_uint8_t(buf, 17, param3);
    _tta_put_uint8_t(buf, 18, param4);
    _tta_put_uint8_t(buf, 19, param5);
    _tta_put_float(buf, 20, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_PROCESS_INDEX, buf, TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC, nocrc);
#else
    ttalink_wp_process_index_t *packet = (ttalink_wp_process_index_t *)msgbuf;
    packet->action_wp_index = action_wp_index;
    packet->total_wp_count = total_wp_count;
    packet->target_wp_index = target_wp_index;
    packet->total_cycle_count = total_cycle_count;
    packet->remain_cycle_count = remain_cycle_count;
    packet->break_state = break_state;
    packet->wp_code = wp_code;
    packet->wp_done_action = wp_done_action;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_PROCESS_INDEX, (const char *)packet, TTALINK_MSG_ID_WP_PROCESS_INDEX_MIN_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN, TTALINK_MSG_ID_WP_PROCESS_INDEX_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_wp_process_index_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2)
{
    _ttalink_wp_process_index_send_buf(dst_addr, src_addr, msgbuf, chan, action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_wp_process_index_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t action_wp_index, int32_t total_wp_count, int32_t target_wp_index, uint8_t total_cycle_count, uint8_t remain_cycle_count, uint8_t break_state, uint32_t wp_code, uint8_t wp_done_action, uint8_t param3, uint8_t param4, uint8_t param5, float param2)
{
    _ttalink_wp_process_index_send_buf(dst_addr, src_addr, msgbuf, chan, action_wp_index, total_wp_count, target_wp_index, total_cycle_count, remain_cycle_count, break_state, wp_code, wp_done_action, param3, param4, param5, param2, true);
}
#endif

#endif

// MESSAGE WP_PROCESS_INDEX UNPACKING


/**
 * @brief Get field action_wp_index from wp_process_index message
 *
 * @return  当前执行航线为第几条航线.
 */
static inline uint8_t ttalink_wp_process_index_get_action_wp_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field total_wp_count from wp_process_index message
 *
 * @return  总的航点数目.
 */
static inline int32_t ttalink_wp_process_index_get_total_wp_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  1);
}

/**
 * @brief Get field target_wp_index from wp_process_index message
 *
 * @return  目标航点数目.
 */
static inline int32_t ttalink_wp_process_index_get_target_wp_index(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int32_t(msg,  5);
}

/**
 * @brief Get field total_cycle_count from wp_process_index message
 *
 * @return  总的循环次数.
 */
static inline uint8_t ttalink_wp_process_index_get_total_cycle_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field remain_cycle_count from wp_process_index message
 *
 * @return  剩余航线循环次数.
 */
static inline uint8_t ttalink_wp_process_index_get_remain_cycle_count(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field break_state from wp_process_index message
 *
 * @return  是否存在航线断点 0：不存在断点  1：存在航线断点  2：存在AB点航线断点.
 */
static inline uint8_t ttalink_wp_process_index_get_break_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field wp_code from wp_process_index message
 *
 * @return  航线码.
 */
static inline uint32_t ttalink_wp_process_index_get_wp_code(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field wp_done_action from wp_process_index message
 *
 * @return  航线完成动作.
 */
static inline uint8_t ttalink_wp_process_index_get_wp_done_action(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field param3 from wp_process_index message
 *
 * @return  预留.
 */
static inline uint8_t ttalink_wp_process_index_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field param4 from wp_process_index message
 *
 * @return  预留.
 */
static inline uint8_t ttalink_wp_process_index_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field param5 from wp_process_index message
 *
 * @return  预留.
 */
static inline uint8_t ttalink_wp_process_index_get_param5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field param2 from wp_process_index message
 *
 * @return  预留.
 */
static inline float ttalink_wp_process_index_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Decode a wp_process_index message into a struct
 *
 * @param msg The message to decode
 * @param wp_process_index C-struct to decode the message contents into
 */
static inline void ttalink_wp_process_index_decode(const ttalink_message_t* msg, ttalink_wp_process_index_t* wp_process_index)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    wp_process_index->action_wp_index = ttalink_wp_process_index_get_action_wp_index(msg);
    wp_process_index->total_wp_count = ttalink_wp_process_index_get_total_wp_count(msg);
    wp_process_index->target_wp_index = ttalink_wp_process_index_get_target_wp_index(msg);
    wp_process_index->total_cycle_count = ttalink_wp_process_index_get_total_cycle_count(msg);
    wp_process_index->remain_cycle_count = ttalink_wp_process_index_get_remain_cycle_count(msg);
    wp_process_index->break_state = ttalink_wp_process_index_get_break_state(msg);
    wp_process_index->wp_code = ttalink_wp_process_index_get_wp_code(msg);
    wp_process_index->wp_done_action = ttalink_wp_process_index_get_wp_done_action(msg);
    wp_process_index->param3 = ttalink_wp_process_index_get_param3(msg);
    wp_process_index->param4 = ttalink_wp_process_index_get_param4(msg);
    wp_process_index->param5 = ttalink_wp_process_index_get_param5(msg);
    wp_process_index->param2 = ttalink_wp_process_index_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN? msg->len : TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN;
        memset(wp_process_index, 0, TTALINK_MSG_ID_WP_PROCESS_INDEX_LEN);
    memcpy(wp_process_index, _TTA_PAYLOAD(msg), len);
#endif
}
