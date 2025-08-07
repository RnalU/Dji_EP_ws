#pragma once
// MESSAGE WP_EVENT_CTRL PACKING

#define TTALINK_MSG_ID_WP_EVENT_CTRL 2158

TTAPACKED(
typedef struct __ttalink_wp_event_ctrl_t {
 uint32_t update_time; /*< .*/
 uint32_t ctrl_cmd; /*<   */
 uint8_t ctrl_action; /*<   0=NULL  */
 uint32_t ctrl_time; /*<  预留控制时间*/
 float ctrl_param1; /*<  控制参数1. */
 float ctrl_param2; /*<  控制参数2. */
 float ctrl_param3; /*<  控制参数3. */
 float ctrl_param4; /*<  控制参数4. */
 float param1; /*<   . */
 float param2; /*<   . */
}) ttalink_wp_event_ctrl_t;

#define TTALINK_MSG_ID_WP_EVENT_CTRL_LEN 37
#define TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN 37
#define TTALINK_MSG_ID_2158_LEN 37
#define TTALINK_MSG_ID_2158_MIN_LEN 37

#define TTALINK_MSG_ID_WP_EVENT_CTRL_CRC 126
#define TTALINK_MSG_ID_2158_CRC 126



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_WP_EVENT_CTRL { \
    2158, \
    "WP_EVENT_CTRL", \
    10, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_wp_event_ctrl_t, update_time) }, \
         { "ctrl_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_wp_event_ctrl_t, ctrl_cmd) }, \
         { "ctrl_action", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_wp_event_ctrl_t, ctrl_action) }, \
         { "ctrl_time", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_wp_event_ctrl_t, ctrl_time) }, \
         { "ctrl_param1", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_wp_event_ctrl_t, ctrl_param1) }, \
         { "ctrl_param2", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_wp_event_ctrl_t, ctrl_param2) }, \
         { "ctrl_param3", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_wp_event_ctrl_t, ctrl_param3) }, \
         { "ctrl_param4", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_wp_event_ctrl_t, ctrl_param4) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_wp_event_ctrl_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_wp_event_ctrl_t, param2) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_WP_EVENT_CTRL { \
    "WP_EVENT_CTRL", \
    10, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_wp_event_ctrl_t, update_time) }, \
         { "ctrl_cmd", NULL, TTALINK_TYPE_UINT32_T, 0, 4, offsetof(ttalink_wp_event_ctrl_t, ctrl_cmd) }, \
         { "ctrl_action", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_wp_event_ctrl_t, ctrl_action) }, \
         { "ctrl_time", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_wp_event_ctrl_t, ctrl_time) }, \
         { "ctrl_param1", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_wp_event_ctrl_t, ctrl_param1) }, \
         { "ctrl_param2", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_wp_event_ctrl_t, ctrl_param2) }, \
         { "ctrl_param3", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_wp_event_ctrl_t, ctrl_param3) }, \
         { "ctrl_param4", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_wp_event_ctrl_t, ctrl_param4) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_wp_event_ctrl_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_wp_event_ctrl_t, param2) }, \
         } \
}
#endif


static inline uint16_t _ttalink_wp_event_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_EVENT_CTRL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_float(buf, 13, ctrl_param1);
    _tta_put_float(buf, 17, ctrl_param2);
    _tta_put_float(buf, 21, ctrl_param3);
    _tta_put_float(buf, 25, ctrl_param4);
    _tta_put_float(buf, 29, param1);
    _tta_put_float(buf, 33, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN);
#else
    ttalink_wp_event_ctrl_t packet;
    packet.update_time = update_time;
    packet.ctrl_cmd = ctrl_cmd;
    packet.ctrl_action = ctrl_action;
    packet.ctrl_time = ctrl_time;
    packet.ctrl_param1 = ctrl_param1;
    packet.ctrl_param2 = ctrl_param2;
    packet.ctrl_param3 = ctrl_param3;
    packet.ctrl_param4 = ctrl_param4;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WP_EVENT_CTRL;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a wp_event_ctrl message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param ctrl_cmd   
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ctrl_param1  控制参数1. 
 * @param ctrl_param2  控制参数2. 
 * @param ctrl_param3  控制参数3. 
 * @param ctrl_param4  控制参数4. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2)
{
    return _ttalink_wp_event_ctrl_pack(dst_addr, src_addr, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, false);
}

/**
 * @brief Pack a wp_event_ctrl message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time .
 * @param ctrl_cmd   
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ctrl_param1  控制参数1. 
 * @param ctrl_param2  控制参数2. 
 * @param ctrl_param3  控制参数3. 
 * @param ctrl_param4  控制参数4. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2)
{
    return _ttalink_wp_event_ctrl_pack(dst_addr, src_addr, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, true);
}


static inline uint16_t _ttalink_wp_event_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ctrl_cmd,uint8_t ctrl_action,uint32_t ctrl_time,float ctrl_param1,float ctrl_param2,float ctrl_param3,float ctrl_param4,float param1,float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_EVENT_CTRL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_float(buf, 13, ctrl_param1);
    _tta_put_float(buf, 17, ctrl_param2);
    _tta_put_float(buf, 21, ctrl_param3);
    _tta_put_float(buf, 25, ctrl_param4);
    _tta_put_float(buf, 29, param1);
    _tta_put_float(buf, 33, param2);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN);
#else
    ttalink_wp_event_ctrl_t packet;
    packet.update_time = update_time;
    packet.ctrl_cmd = ctrl_cmd;
    packet.ctrl_action = ctrl_action;
    packet.ctrl_time = ctrl_time;
    packet.ctrl_param1 = ctrl_param1;
    packet.ctrl_param2 = ctrl_param2;
    packet.ctrl_param3 = ctrl_param3;
    packet.ctrl_param4 = ctrl_param4;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_WP_EVENT_CTRL;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_CRC, nocrc);
}

/**
 * @brief Pack a wp_event_ctrl message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param ctrl_cmd   
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ctrl_param1  控制参数1. 
 * @param ctrl_param2  控制参数2. 
 * @param ctrl_param3  控制参数3. 
 * @param ctrl_param4  控制参数4. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ctrl_cmd,uint8_t ctrl_action,uint32_t ctrl_time,float ctrl_param1,float ctrl_param2,float ctrl_param3,float ctrl_param4,float param1,float param2)
{
    return _ttalink_wp_event_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, false);
}

/**
 * @brief Pack a wp_event_ctrl message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time .
 * @param ctrl_cmd   
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ctrl_param1  控制参数1. 
 * @param ctrl_param2  控制参数2. 
 * @param ctrl_param3  控制参数3. 
 * @param ctrl_param4  控制参数4. 
 * @param param1   . 
 * @param param2   . 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_wp_event_ctrl_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint32_t ctrl_cmd,uint8_t ctrl_action,uint32_t ctrl_time,float ctrl_param1,float ctrl_param2,float ctrl_param3,float ctrl_param4,float param1,float param2)
{
    return _ttalink_wp_event_ctrl_pack_chan(dst_addr, src_addr, chan, msg,  update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, true);
}


static inline uint16_t _ttalink_wp_event_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_event_ctrl_t* wp_event_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_wp_event_ctrl_pack_nocrc(dst_addr, src_addr, msg, wp_event_ctrl->update_time, wp_event_ctrl->ctrl_cmd, wp_event_ctrl->ctrl_action, wp_event_ctrl->ctrl_time, wp_event_ctrl->ctrl_param1, wp_event_ctrl->ctrl_param2, wp_event_ctrl->ctrl_param3, wp_event_ctrl->ctrl_param4, wp_event_ctrl->param1, wp_event_ctrl->param2);
    }else{
        return ttalink_wp_event_ctrl_pack(dst_addr, src_addr, msg, wp_event_ctrl->update_time, wp_event_ctrl->ctrl_cmd, wp_event_ctrl->ctrl_action, wp_event_ctrl->ctrl_time, wp_event_ctrl->ctrl_param1, wp_event_ctrl->ctrl_param2, wp_event_ctrl->ctrl_param3, wp_event_ctrl->ctrl_param4, wp_event_ctrl->param1, wp_event_ctrl->param2);
    }
    
}

/**
 * @brief Encode a wp_event_ctrl struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_event_ctrl_t* wp_event_ctrl)
{
    return _ttalink_wp_event_ctrl_encode(dst_addr, src_addr, msg, wp_event_ctrl, false);
}

/**
 * @brief Encode a wp_event_ctrl struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_wp_event_ctrl_t* wp_event_ctrl)
{
    return _ttalink_wp_event_ctrl_encode(dst_addr, src_addr, msg, wp_event_ctrl, true);
}


static inline uint16_t _ttalink_wp_event_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_event_ctrl_t* wp_event_ctrl, bool nocrc)
{
    if(nocrc){
        return ttalink_wp_event_ctrl_pack_chan_nocrc(dst_addr, src_addr, chan, msg, wp_event_ctrl->update_time, wp_event_ctrl->ctrl_cmd, wp_event_ctrl->ctrl_action, wp_event_ctrl->ctrl_time, wp_event_ctrl->ctrl_param1, wp_event_ctrl->ctrl_param2, wp_event_ctrl->ctrl_param3, wp_event_ctrl->ctrl_param4, wp_event_ctrl->param1, wp_event_ctrl->param2);
    }else{
        return ttalink_wp_event_ctrl_pack_chan(dst_addr, src_addr, chan, msg, wp_event_ctrl->update_time, wp_event_ctrl->ctrl_cmd, wp_event_ctrl->ctrl_action, wp_event_ctrl->ctrl_time, wp_event_ctrl->ctrl_param1, wp_event_ctrl->ctrl_param2, wp_event_ctrl->ctrl_param3, wp_event_ctrl->ctrl_param4, wp_event_ctrl->param1, wp_event_ctrl->param2);
    }
}

/**
 * @brief Encode a wp_event_ctrl struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_event_ctrl_t* wp_event_ctrl)
{
    return _ttalink_wp_event_ctrl_encode_chan(dst_addr, src_addr, chan, msg, wp_event_ctrl, false);
}

/**
 * @brief Encode a wp_event_ctrl struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param wp_event_ctrl C-struct to read the message contents from
 */
static inline uint16_t ttalink_wp_event_ctrl_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_wp_event_ctrl_t* wp_event_ctrl)
{
    return _ttalink_wp_event_ctrl_encode_chan(dst_addr, src_addr, chan, msg, wp_event_ctrl, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_wp_event_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_WP_EVENT_CTRL_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_float(buf, 13, ctrl_param1);
    _tta_put_float(buf, 17, ctrl_param2);
    _tta_put_float(buf, 21, ctrl_param3);
    _tta_put_float(buf, 25, ctrl_param4);
    _tta_put_float(buf, 29, param1);
    _tta_put_float(buf, 33, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL, buf, TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_CRC, nocrc);
#else
    ttalink_wp_event_ctrl_t packet;
    packet.update_time = update_time;
    packet.ctrl_cmd = ctrl_cmd;
    packet.ctrl_action = ctrl_action;
    packet.ctrl_time = ctrl_time;
    packet.ctrl_param1 = ctrl_param1;
    packet.ctrl_param2 = ctrl_param2;
    packet.ctrl_param3 = ctrl_param3;
    packet.ctrl_param4 = ctrl_param4;
    packet.param1 = param1;
    packet.param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL, (const char *)&packet, TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a wp_event_ctrl message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param ctrl_cmd   
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ctrl_param1  控制参数1. 
 * @param ctrl_param2  控制参数2. 
 * @param ctrl_param3  控制参数3. 
 * @param ctrl_param4  控制参数4. 
 * @param param1   . 
 * @param param2   . 
 */
static inline void ttalink_wp_event_ctrl_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2)
{
    _ttalink_wp_event_ctrl_send(dst_addr, src_addr, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, false);
}

/**
 * @brief Send a wp_event_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time .
 * @param ctrl_cmd   
 * @param ctrl_action   0=NULL  
 * @param ctrl_time  预留控制时间
 * @param ctrl_param1  控制参数1. 
 * @param ctrl_param2  控制参数2. 
 * @param ctrl_param3  控制参数3. 
 * @param ctrl_param4  控制参数4. 
 * @param param1   . 
 * @param param2   . 
 */
static inline void ttalink_wp_event_ctrl_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2)
{
    _ttalink_wp_event_ctrl_send(dst_addr, src_addr, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, true);
}


static inline void _ttalink_wp_event_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_event_ctrl_t* wp_event_ctrl, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_wp_event_ctrl_send_nocrc(dst_addr, src_addr, chan, wp_event_ctrl->update_time, wp_event_ctrl->ctrl_cmd, wp_event_ctrl->ctrl_action, wp_event_ctrl->ctrl_time, wp_event_ctrl->ctrl_param1, wp_event_ctrl->ctrl_param2, wp_event_ctrl->ctrl_param3, wp_event_ctrl->ctrl_param4, wp_event_ctrl->param1, wp_event_ctrl->param2);
    }else{
        ttalink_wp_event_ctrl_send(dst_addr, src_addr, chan, wp_event_ctrl->update_time, wp_event_ctrl->ctrl_cmd, wp_event_ctrl->ctrl_action, wp_event_ctrl->ctrl_time, wp_event_ctrl->ctrl_param1, wp_event_ctrl->ctrl_param2, wp_event_ctrl->ctrl_param3, wp_event_ctrl->ctrl_param4, wp_event_ctrl->param1, wp_event_ctrl->param2);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL, (const char *)wp_event_ctrl, TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_CRC, nocrc);
#endif
}

/**
 * @brief Send a wp_event_ctrl message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_wp_event_ctrl_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_event_ctrl_t* wp_event_ctrl)
{
    _ttalink_wp_event_ctrl_send_struct(dst_addr, src_addr, chan, wp_event_ctrl, false);
}

/**
 * @brief Send a wp_event_ctrl message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_wp_event_ctrl_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_wp_event_ctrl_t* wp_event_ctrl)
{
    _ttalink_wp_event_ctrl_send_struct(dst_addr, src_addr, chan, wp_event_ctrl, true);
}

#if TTALINK_MSG_ID_WP_EVENT_CTRL_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_wp_event_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint32_t(buf, 4, ctrl_cmd);
    _tta_put_uint8_t(buf, 8, ctrl_action);
    _tta_put_uint32_t(buf, 9, ctrl_time);
    _tta_put_float(buf, 13, ctrl_param1);
    _tta_put_float(buf, 17, ctrl_param2);
    _tta_put_float(buf, 21, ctrl_param3);
    _tta_put_float(buf, 25, ctrl_param4);
    _tta_put_float(buf, 29, param1);
    _tta_put_float(buf, 33, param2);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL, buf, TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_CRC, nocrc);
#else
    ttalink_wp_event_ctrl_t *packet = (ttalink_wp_event_ctrl_t *)msgbuf;
    packet->update_time = update_time;
    packet->ctrl_cmd = ctrl_cmd;
    packet->ctrl_action = ctrl_action;
    packet->ctrl_time = ctrl_time;
    packet->ctrl_param1 = ctrl_param1;
    packet->ctrl_param2 = ctrl_param2;
    packet->ctrl_param3 = ctrl_param3;
    packet->ctrl_param4 = ctrl_param4;
    packet->param1 = param1;
    packet->param2 = param2;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_WP_EVENT_CTRL, (const char *)packet, TTALINK_MSG_ID_WP_EVENT_CTRL_MIN_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN, TTALINK_MSG_ID_WP_EVENT_CTRL_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_wp_event_ctrl_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2)
{
    _ttalink_wp_event_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_wp_event_ctrl_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint32_t ctrl_cmd, uint8_t ctrl_action, uint32_t ctrl_time, float ctrl_param1, float ctrl_param2, float ctrl_param3, float ctrl_param4, float param1, float param2)
{
    _ttalink_wp_event_ctrl_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, ctrl_cmd, ctrl_action, ctrl_time, ctrl_param1, ctrl_param2, ctrl_param3, ctrl_param4, param1, param2, true);
}
#endif

#endif

// MESSAGE WP_EVENT_CTRL UNPACKING


/**
 * @brief Get field update_time from wp_event_ctrl message
 *
 * @return .
 */
static inline uint32_t ttalink_wp_event_ctrl_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ctrl_cmd from wp_event_ctrl message
 *
 * @return   
 */
static inline uint32_t ttalink_wp_event_ctrl_get_ctrl_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field ctrl_action from wp_event_ctrl message
 *
 * @return   0=NULL  
 */
static inline uint8_t ttalink_wp_event_ctrl_get_ctrl_action(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field ctrl_time from wp_event_ctrl message
 *
 * @return  预留控制时间
 */
static inline uint32_t ttalink_wp_event_ctrl_get_ctrl_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Get field ctrl_param1 from wp_event_ctrl message
 *
 * @return  控制参数1. 
 */
static inline float ttalink_wp_event_ctrl_get_ctrl_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field ctrl_param2 from wp_event_ctrl message
 *
 * @return  控制参数2. 
 */
static inline float ttalink_wp_event_ctrl_get_ctrl_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Get field ctrl_param3 from wp_event_ctrl message
 *
 * @return  控制参数3. 
 */
static inline float ttalink_wp_event_ctrl_get_ctrl_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field ctrl_param4 from wp_event_ctrl message
 *
 * @return  控制参数4. 
 */
static inline float ttalink_wp_event_ctrl_get_ctrl_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field param1 from wp_event_ctrl message
 *
 * @return   . 
 */
static inline float ttalink_wp_event_ctrl_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Get field param2 from wp_event_ctrl message
 *
 * @return   . 
 */
static inline float ttalink_wp_event_ctrl_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  33);
}

/**
 * @brief Decode a wp_event_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param wp_event_ctrl C-struct to decode the message contents into
 */
static inline void ttalink_wp_event_ctrl_decode(const ttalink_message_t* msg, ttalink_wp_event_ctrl_t* wp_event_ctrl)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    wp_event_ctrl->update_time = ttalink_wp_event_ctrl_get_update_time(msg);
    wp_event_ctrl->ctrl_cmd = ttalink_wp_event_ctrl_get_ctrl_cmd(msg);
    wp_event_ctrl->ctrl_action = ttalink_wp_event_ctrl_get_ctrl_action(msg);
    wp_event_ctrl->ctrl_time = ttalink_wp_event_ctrl_get_ctrl_time(msg);
    wp_event_ctrl->ctrl_param1 = ttalink_wp_event_ctrl_get_ctrl_param1(msg);
    wp_event_ctrl->ctrl_param2 = ttalink_wp_event_ctrl_get_ctrl_param2(msg);
    wp_event_ctrl->ctrl_param3 = ttalink_wp_event_ctrl_get_ctrl_param3(msg);
    wp_event_ctrl->ctrl_param4 = ttalink_wp_event_ctrl_get_ctrl_param4(msg);
    wp_event_ctrl->param1 = ttalink_wp_event_ctrl_get_param1(msg);
    wp_event_ctrl->param2 = ttalink_wp_event_ctrl_get_param2(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_WP_EVENT_CTRL_LEN? msg->len : TTALINK_MSG_ID_WP_EVENT_CTRL_LEN;
        memset(wp_event_ctrl, 0, TTALINK_MSG_ID_WP_EVENT_CTRL_LEN);
    memcpy(wp_event_ctrl, _TTA_PAYLOAD(msg), len);
#endif
}
