#pragma once
// MESSAGE HEARTBEAT PACKING

#define TTALINK_MSG_ID_HEARTBEAT 2001

TTAPACKED(
typedef struct __ttalink_heartbeat_t {
 uint8_t mode; /*< 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块*/
 uint8_t device_type; /*< 设备类型.*/
 uint8_t flight_mode; /*< 飞行模式.*/
 uint8_t flight_mode_state; /*< 飞行模式的状态.*/
 uint8_t ctrl_state; /*< 控制权状态.*/
 uint32_t flight_time; /*< 飞行时间.*/
 uint32_t simple_time; /*< 系统时间.*/
 int64_t sn; /*< 设备序列号.*/
 uint8_t upload_state; /*< 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.*/
 uint8_t air_born; /*< 飞行状态 0:未飞行，1：已飞行，需要加入文档.*/
 uint16_t device_reserve; /*< 设备预留*/
 uint32_t reserve; /*< 预留.*/
}) ttalink_heartbeat_t;

#define TTALINK_MSG_ID_HEARTBEAT_LEN 29
#define TTALINK_MSG_ID_HEARTBEAT_MIN_LEN 29
#define TTALINK_MSG_ID_2001_LEN 29
#define TTALINK_MSG_ID_2001_MIN_LEN 29

#define TTALINK_MSG_ID_HEARTBEAT_CRC 208
#define TTALINK_MSG_ID_2001_CRC 208



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_HEARTBEAT { \
    2001, \
    "HEARTBEAT", \
    12, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_heartbeat_t, mode) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_heartbeat_t, device_type) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_heartbeat_t, flight_mode) }, \
         { "flight_mode_state", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_heartbeat_t, flight_mode_state) }, \
         { "ctrl_state", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_heartbeat_t, ctrl_state) }, \
         { "flight_time", NULL, TTALINK_TYPE_UINT32_T, 0, 5, offsetof(ttalink_heartbeat_t, flight_time) }, \
         { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_heartbeat_t, simple_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 13, offsetof(ttalink_heartbeat_t, sn) }, \
         { "upload_state", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_heartbeat_t, upload_state) }, \
         { "air_born", NULL, TTALINK_TYPE_UINT8_T, 0, 22, offsetof(ttalink_heartbeat_t, air_born) }, \
         { "device_reserve", NULL, TTALINK_TYPE_UINT16_T, 0, 23, offsetof(ttalink_heartbeat_t, device_reserve) }, \
         { "reserve", NULL, TTALINK_TYPE_UINT32_T, 0, 25, offsetof(ttalink_heartbeat_t, reserve) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_HEARTBEAT { \
    "HEARTBEAT", \
    12, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_heartbeat_t, mode) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_heartbeat_t, device_type) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_heartbeat_t, flight_mode) }, \
         { "flight_mode_state", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_heartbeat_t, flight_mode_state) }, \
         { "ctrl_state", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_heartbeat_t, ctrl_state) }, \
         { "flight_time", NULL, TTALINK_TYPE_UINT32_T, 0, 5, offsetof(ttalink_heartbeat_t, flight_time) }, \
         { "simple_time", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_heartbeat_t, simple_time) }, \
         { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 13, offsetof(ttalink_heartbeat_t, sn) }, \
         { "upload_state", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_heartbeat_t, upload_state) }, \
         { "air_born", NULL, TTALINK_TYPE_UINT8_T, 0, 22, offsetof(ttalink_heartbeat_t, air_born) }, \
         { "device_reserve", NULL, TTALINK_TYPE_UINT16_T, 0, 23, offsetof(ttalink_heartbeat_t, device_reserve) }, \
         { "reserve", NULL, TTALINK_TYPE_UINT32_T, 0, 25, offsetof(ttalink_heartbeat_t, reserve) }, \
         } \
}
#endif


static inline uint16_t _ttalink_heartbeat_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_HEARTBEAT_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, device_type);
    _tta_put_uint8_t(buf, 2, flight_mode);
    _tta_put_uint8_t(buf, 3, flight_mode_state);
    _tta_put_uint8_t(buf, 4, ctrl_state);
    _tta_put_uint32_t(buf, 5, flight_time);
    _tta_put_uint32_t(buf, 9, simple_time);
    _tta_put_int64_t(buf, 13, sn);
    _tta_put_uint8_t(buf, 21, upload_state);
    _tta_put_uint8_t(buf, 22, air_born);
    _tta_put_uint16_t(buf, 23, device_reserve);
    _tta_put_uint32_t(buf, 25, reserve);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_HEARTBEAT_LEN);
#else
    ttalink_heartbeat_t packet;
    packet.mode = mode;
    packet.device_type = device_type;
    packet.flight_mode = flight_mode;
    packet.flight_mode_state = flight_mode_state;
    packet.ctrl_state = ctrl_state;
    packet.flight_time = flight_time;
    packet.simple_time = simple_time;
    packet.sn = sn;
    packet.upload_state = upload_state;
    packet.air_born = air_born;
    packet.device_reserve = device_reserve;
    packet.reserve = reserve;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_HEARTBEAT;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_HEARTBEAT_MIN_LEN, TTALINK_MSG_ID_HEARTBEAT_LEN, TTALINK_MSG_ID_HEARTBEAT_CRC, nocrc);
}

/**
 * @brief Pack a heartbeat message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块
 * @param device_type 设备类型.
 * @param flight_mode 飞行模式.
 * @param flight_mode_state 飞行模式的状态.
 * @param ctrl_state 控制权状态.
 * @param flight_time 飞行时间.
 * @param simple_time 系统时间.
 * @param sn 设备序列号.
 * @param upload_state 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.
 * @param air_born 飞行状态 0:未飞行，1：已飞行，需要加入文档.
 * @param device_reserve 设备预留
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_heartbeat_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve)
{
    return _ttalink_heartbeat_pack(dst_addr, src_addr, msg,  mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, false);
}

/**
 * @brief Pack a heartbeat message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块
 * @param device_type 设备类型.
 * @param flight_mode 飞行模式.
 * @param flight_mode_state 飞行模式的状态.
 * @param ctrl_state 控制权状态.
 * @param flight_time 飞行时间.
 * @param simple_time 系统时间.
 * @param sn 设备序列号.
 * @param upload_state 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.
 * @param air_born 飞行状态 0:未飞行，1：已飞行，需要加入文档.
 * @param device_reserve 设备预留
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_heartbeat_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve)
{
    return _ttalink_heartbeat_pack(dst_addr, src_addr, msg,  mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, true);
}


static inline uint16_t _ttalink_heartbeat_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t device_type,uint8_t flight_mode,uint8_t flight_mode_state,uint8_t ctrl_state,uint32_t flight_time,uint32_t simple_time,int64_t sn,uint8_t upload_state,uint8_t air_born,uint16_t device_reserve,uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_HEARTBEAT_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, device_type);
    _tta_put_uint8_t(buf, 2, flight_mode);
    _tta_put_uint8_t(buf, 3, flight_mode_state);
    _tta_put_uint8_t(buf, 4, ctrl_state);
    _tta_put_uint32_t(buf, 5, flight_time);
    _tta_put_uint32_t(buf, 9, simple_time);
    _tta_put_int64_t(buf, 13, sn);
    _tta_put_uint8_t(buf, 21, upload_state);
    _tta_put_uint8_t(buf, 22, air_born);
    _tta_put_uint16_t(buf, 23, device_reserve);
    _tta_put_uint32_t(buf, 25, reserve);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_HEARTBEAT_LEN);
#else
    ttalink_heartbeat_t packet;
    packet.mode = mode;
    packet.device_type = device_type;
    packet.flight_mode = flight_mode;
    packet.flight_mode_state = flight_mode_state;
    packet.ctrl_state = ctrl_state;
    packet.flight_time = flight_time;
    packet.simple_time = simple_time;
    packet.sn = sn;
    packet.upload_state = upload_state;
    packet.air_born = air_born;
    packet.device_reserve = device_reserve;
    packet.reserve = reserve;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_HEARTBEAT;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_HEARTBEAT_MIN_LEN, TTALINK_MSG_ID_HEARTBEAT_LEN, TTALINK_MSG_ID_HEARTBEAT_CRC, nocrc);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块
 * @param device_type 设备类型.
 * @param flight_mode 飞行模式.
 * @param flight_mode_state 飞行模式的状态.
 * @param ctrl_state 控制权状态.
 * @param flight_time 飞行时间.
 * @param simple_time 系统时间.
 * @param sn 设备序列号.
 * @param upload_state 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.
 * @param air_born 飞行状态 0:未飞行，1：已飞行，需要加入文档.
 * @param device_reserve 设备预留
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_heartbeat_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t device_type,uint8_t flight_mode,uint8_t flight_mode_state,uint8_t ctrl_state,uint32_t flight_time,uint32_t simple_time,int64_t sn,uint8_t upload_state,uint8_t air_born,uint16_t device_reserve,uint32_t reserve)
{
    return _ttalink_heartbeat_pack_chan(dst_addr, src_addr, chan, msg,  mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, false);
}

/**
 * @brief Pack a heartbeat message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块
 * @param device_type 设备类型.
 * @param flight_mode 飞行模式.
 * @param flight_mode_state 飞行模式的状态.
 * @param ctrl_state 控制权状态.
 * @param flight_time 飞行时间.
 * @param simple_time 系统时间.
 * @param sn 设备序列号.
 * @param upload_state 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.
 * @param air_born 飞行状态 0:未飞行，1：已飞行，需要加入文档.
 * @param device_reserve 设备预留
 * @param reserve 预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_heartbeat_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t device_type,uint8_t flight_mode,uint8_t flight_mode_state,uint8_t ctrl_state,uint32_t flight_time,uint32_t simple_time,int64_t sn,uint8_t upload_state,uint8_t air_born,uint16_t device_reserve,uint32_t reserve)
{
    return _ttalink_heartbeat_pack_chan(dst_addr, src_addr, chan, msg,  mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, true);
}


static inline uint16_t _ttalink_heartbeat_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_heartbeat_t* heartbeat, bool nocrc)
{
    if(nocrc){
        return ttalink_heartbeat_pack_nocrc(dst_addr, src_addr, msg, heartbeat->mode, heartbeat->device_type, heartbeat->flight_mode, heartbeat->flight_mode_state, heartbeat->ctrl_state, heartbeat->flight_time, heartbeat->simple_time, heartbeat->sn, heartbeat->upload_state, heartbeat->air_born, heartbeat->device_reserve, heartbeat->reserve);
    }else{
        return ttalink_heartbeat_pack(dst_addr, src_addr, msg, heartbeat->mode, heartbeat->device_type, heartbeat->flight_mode, heartbeat->flight_mode_state, heartbeat->ctrl_state, heartbeat->flight_time, heartbeat->simple_time, heartbeat->sn, heartbeat->upload_state, heartbeat->air_born, heartbeat->device_reserve, heartbeat->reserve);
    }
    
}

/**
 * @brief Encode a heartbeat struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t ttalink_heartbeat_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_heartbeat_t* heartbeat)
{
    return _ttalink_heartbeat_encode(dst_addr, src_addr, msg, heartbeat, false);
}

/**
 * @brief Encode a heartbeat struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t ttalink_heartbeat_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_heartbeat_t* heartbeat)
{
    return _ttalink_heartbeat_encode(dst_addr, src_addr, msg, heartbeat, true);
}


static inline uint16_t _ttalink_heartbeat_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_heartbeat_t* heartbeat, bool nocrc)
{
    if(nocrc){
        return ttalink_heartbeat_pack_chan_nocrc(dst_addr, src_addr, chan, msg, heartbeat->mode, heartbeat->device_type, heartbeat->flight_mode, heartbeat->flight_mode_state, heartbeat->ctrl_state, heartbeat->flight_time, heartbeat->simple_time, heartbeat->sn, heartbeat->upload_state, heartbeat->air_born, heartbeat->device_reserve, heartbeat->reserve);
    }else{
        return ttalink_heartbeat_pack_chan(dst_addr, src_addr, chan, msg, heartbeat->mode, heartbeat->device_type, heartbeat->flight_mode, heartbeat->flight_mode_state, heartbeat->ctrl_state, heartbeat->flight_time, heartbeat->simple_time, heartbeat->sn, heartbeat->upload_state, heartbeat->air_born, heartbeat->device_reserve, heartbeat->reserve);
    }
}

/**
 * @brief Encode a heartbeat struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t ttalink_heartbeat_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_heartbeat_t* heartbeat)
{
    return _ttalink_heartbeat_encode_chan(dst_addr, src_addr, chan, msg, heartbeat, false);
}

/**
 * @brief Encode a heartbeat struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t ttalink_heartbeat_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_heartbeat_t* heartbeat)
{
    return _ttalink_heartbeat_encode_chan(dst_addr, src_addr, chan, msg, heartbeat, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_heartbeat_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_HEARTBEAT_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, device_type);
    _tta_put_uint8_t(buf, 2, flight_mode);
    _tta_put_uint8_t(buf, 3, flight_mode_state);
    _tta_put_uint8_t(buf, 4, ctrl_state);
    _tta_put_uint32_t(buf, 5, flight_time);
    _tta_put_uint32_t(buf, 9, simple_time);
    _tta_put_int64_t(buf, 13, sn);
    _tta_put_uint8_t(buf, 21, upload_state);
    _tta_put_uint8_t(buf, 22, air_born);
    _tta_put_uint16_t(buf, 23, device_reserve);
    _tta_put_uint32_t(buf, 25, reserve);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_HEARTBEAT, buf, TTALINK_MSG_ID_HEARTBEAT_MIN_LEN, TTALINK_MSG_ID_HEARTBEAT_LEN, TTALINK_MSG_ID_HEARTBEAT_CRC, nocrc);
#else
    ttalink_heartbeat_t packet;
    packet.mode = mode;
    packet.device_type = device_type;
    packet.flight_mode = flight_mode;
    packet.flight_mode_state = flight_mode_state;
    packet.ctrl_state = ctrl_state;
    packet.flight_time = flight_time;
    packet.simple_time = simple_time;
    packet.sn = sn;
    packet.upload_state = upload_state;
    packet.air_born = air_born;
    packet.device_reserve = device_reserve;
    packet.reserve = reserve;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_HEARTBEAT, (const char *)&packet, TTALINK_MSG_ID_HEARTBEAT_MIN_LEN, TTALINK_MSG_ID_HEARTBEAT_LEN, TTALINK_MSG_ID_HEARTBEAT_CRC, nocrc);
#endif
}

/**
 * @brief Send a heartbeat message
 * @param chan TTAlink channel to send the message
 *
 * @param mode 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块
 * @param device_type 设备类型.
 * @param flight_mode 飞行模式.
 * @param flight_mode_state 飞行模式的状态.
 * @param ctrl_state 控制权状态.
 * @param flight_time 飞行时间.
 * @param simple_time 系统时间.
 * @param sn 设备序列号.
 * @param upload_state 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.
 * @param air_born 飞行状态 0:未飞行，1：已飞行，需要加入文档.
 * @param device_reserve 设备预留
 * @param reserve 预留.
 */
static inline void ttalink_heartbeat_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve)
{
    _ttalink_heartbeat_send(dst_addr, src_addr, chan, mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, false);
}

/**
 * @brief Send a heartbeat message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mode 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块
 * @param device_type 设备类型.
 * @param flight_mode 飞行模式.
 * @param flight_mode_state 飞行模式的状态.
 * @param ctrl_state 控制权状态.
 * @param flight_time 飞行时间.
 * @param simple_time 系统时间.
 * @param sn 设备序列号.
 * @param upload_state 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.
 * @param air_born 飞行状态 0:未飞行，1：已飞行，需要加入文档.
 * @param device_reserve 设备预留
 * @param reserve 预留.
 */
static inline void ttalink_heartbeat_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve)
{
    _ttalink_heartbeat_send(dst_addr, src_addr, chan, mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, true);
}


static inline void _ttalink_heartbeat_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_heartbeat_t* heartbeat, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_heartbeat_send_nocrc(dst_addr, src_addr, chan, heartbeat->mode, heartbeat->device_type, heartbeat->flight_mode, heartbeat->flight_mode_state, heartbeat->ctrl_state, heartbeat->flight_time, heartbeat->simple_time, heartbeat->sn, heartbeat->upload_state, heartbeat->air_born, heartbeat->device_reserve, heartbeat->reserve);
    }else{
        ttalink_heartbeat_send(dst_addr, src_addr, chan, heartbeat->mode, heartbeat->device_type, heartbeat->flight_mode, heartbeat->flight_mode_state, heartbeat->ctrl_state, heartbeat->flight_time, heartbeat->simple_time, heartbeat->sn, heartbeat->upload_state, heartbeat->air_born, heartbeat->device_reserve, heartbeat->reserve);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_HEARTBEAT, (const char *)heartbeat, TTALINK_MSG_ID_HEARTBEAT_MIN_LEN, TTALINK_MSG_ID_HEARTBEAT_LEN, TTALINK_MSG_ID_HEARTBEAT_CRC, nocrc);
#endif
}

/**
 * @brief Send a heartbeat message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_heartbeat_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_heartbeat_t* heartbeat)
{
    _ttalink_heartbeat_send_struct(dst_addr, src_addr, chan, heartbeat, false);
}

/**
 * @brief Send a heartbeat message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_heartbeat_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_heartbeat_t* heartbeat)
{
    _ttalink_heartbeat_send_struct(dst_addr, src_addr, chan, heartbeat, true);
}

#if TTALINK_MSG_ID_HEARTBEAT_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_heartbeat_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, device_type);
    _tta_put_uint8_t(buf, 2, flight_mode);
    _tta_put_uint8_t(buf, 3, flight_mode_state);
    _tta_put_uint8_t(buf, 4, ctrl_state);
    _tta_put_uint32_t(buf, 5, flight_time);
    _tta_put_uint32_t(buf, 9, simple_time);
    _tta_put_int64_t(buf, 13, sn);
    _tta_put_uint8_t(buf, 21, upload_state);
    _tta_put_uint8_t(buf, 22, air_born);
    _tta_put_uint16_t(buf, 23, device_reserve);
    _tta_put_uint32_t(buf, 25, reserve);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_HEARTBEAT, buf, TTALINK_MSG_ID_HEARTBEAT_MIN_LEN, TTALINK_MSG_ID_HEARTBEAT_LEN, TTALINK_MSG_ID_HEARTBEAT_CRC, nocrc);
#else
    ttalink_heartbeat_t *packet = (ttalink_heartbeat_t *)msgbuf;
    packet->mode = mode;
    packet->device_type = device_type;
    packet->flight_mode = flight_mode;
    packet->flight_mode_state = flight_mode_state;
    packet->ctrl_state = ctrl_state;
    packet->flight_time = flight_time;
    packet->simple_time = simple_time;
    packet->sn = sn;
    packet->upload_state = upload_state;
    packet->air_born = air_born;
    packet->device_reserve = device_reserve;
    packet->reserve = reserve;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_HEARTBEAT, (const char *)packet, TTALINK_MSG_ID_HEARTBEAT_MIN_LEN, TTALINK_MSG_ID_HEARTBEAT_LEN, TTALINK_MSG_ID_HEARTBEAT_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_heartbeat_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve)
{
    _ttalink_heartbeat_send_buf(dst_addr, src_addr, msgbuf, chan, mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_heartbeat_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t device_type, uint8_t flight_mode, uint8_t flight_mode_state, uint8_t ctrl_state, uint32_t flight_time, uint32_t simple_time, int64_t sn, uint8_t upload_state, uint8_t air_born, uint16_t device_reserve, uint32_t reserve)
{
    _ttalink_heartbeat_send_buf(dst_addr, src_addr, msgbuf, chan, mode, device_type, flight_mode, flight_mode_state, ctrl_state, flight_time, simple_time, sn, upload_state, air_born, device_reserve, reserve, true);
}
#endif

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field mode from heartbeat message
 *
 * @return 板卡模式 0=单机 1-主机 2=从机 200=多旋翼模块 201=固定翼模块 254=外部模块
 */
static inline uint8_t ttalink_heartbeat_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field device_type from heartbeat message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_heartbeat_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field flight_mode from heartbeat message
 *
 * @return 飞行模式.
 */
static inline uint8_t ttalink_heartbeat_get_flight_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field flight_mode_state from heartbeat message
 *
 * @return 飞行模式的状态.
 */
static inline uint8_t ttalink_heartbeat_get_flight_mode_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field ctrl_state from heartbeat message
 *
 * @return 控制权状态.
 */
static inline uint8_t ttalink_heartbeat_get_ctrl_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field flight_time from heartbeat message
 *
 * @return 飞行时间.
 */
static inline uint32_t ttalink_heartbeat_get_flight_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  5);
}

/**
 * @brief Get field simple_time from heartbeat message
 *
 * @return 系统时间.
 */
static inline uint32_t ttalink_heartbeat_get_simple_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Get field sn from heartbeat message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_heartbeat_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  13);
}

/**
 * @brief Get field upload_state from heartbeat message
 *
 * @return 模块升级状态 0 - 不用升级 1-正在升级 2-升级完成.
 */
static inline uint8_t ttalink_heartbeat_get_upload_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field air_born from heartbeat message
 *
 * @return 飞行状态 0:未飞行，1：已飞行，需要加入文档.
 */
static inline uint8_t ttalink_heartbeat_get_air_born(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field device_reserve from heartbeat message
 *
 * @return 设备预留
 */
static inline uint16_t ttalink_heartbeat_get_device_reserve(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  23);
}

/**
 * @brief Get field reserve from heartbeat message
 *
 * @return 预留.
 */
static inline uint32_t ttalink_heartbeat_get_reserve(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  25);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void ttalink_heartbeat_decode(const ttalink_message_t* msg, ttalink_heartbeat_t* heartbeat)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    heartbeat->mode = ttalink_heartbeat_get_mode(msg);
    heartbeat->device_type = ttalink_heartbeat_get_device_type(msg);
    heartbeat->flight_mode = ttalink_heartbeat_get_flight_mode(msg);
    heartbeat->flight_mode_state = ttalink_heartbeat_get_flight_mode_state(msg);
    heartbeat->ctrl_state = ttalink_heartbeat_get_ctrl_state(msg);
    heartbeat->flight_time = ttalink_heartbeat_get_flight_time(msg);
    heartbeat->simple_time = ttalink_heartbeat_get_simple_time(msg);
    heartbeat->sn = ttalink_heartbeat_get_sn(msg);
    heartbeat->upload_state = ttalink_heartbeat_get_upload_state(msg);
    heartbeat->air_born = ttalink_heartbeat_get_air_born(msg);
    heartbeat->device_reserve = ttalink_heartbeat_get_device_reserve(msg);
    heartbeat->reserve = ttalink_heartbeat_get_reserve(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_HEARTBEAT_LEN? msg->len : TTALINK_MSG_ID_HEARTBEAT_LEN;
        memset(heartbeat, 0, TTALINK_MSG_ID_HEARTBEAT_LEN);
    memcpy(heartbeat, _TTA_PAYLOAD(msg), len);
#endif
}
