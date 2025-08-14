#pragma once
// MESSAGE UPDATE_VR_FROM_FC PACKING

#define TTALINK_MSG_ID_UPDATE_VR_FROM_FC 2227

TTAPACKED(
typedef struct __ttalink_update_vr_from_fc_t {
 uint32_t update_time; /*<  .*/
 uint8_t flight_mode; /*< 0：模拟航线飞行  1：模拟遥控器飞行.*/
 uint8_t ctrl_mode; /*<   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.*/
 float acc_x; /*<  （符合右手定责，前右下坐标系）（m/s²）.*/
 float acc_y; /*< （符合右手定责，前右下坐标系）（m/s²） .*/
 float acc_u; /*<  （NED坐标系）（m/s²）.*/
 float yaw_ctrl; /*<  （符合右手定责，前右下坐标系）（转矩）.*/
 float param1; /*<  预留.*/
 float param2; /*<  预留.*/
 float param3; /*<  预留.*/
 float param4; /*<  预留.*/
}) ttalink_update_vr_from_fc_t;

#define TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN 38
#define TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN 38
#define TTALINK_MSG_ID_2227_LEN 38
#define TTALINK_MSG_ID_2227_MIN_LEN 38

#define TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC 205
#define TTALINK_MSG_ID_2227_CRC 205



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_UPDATE_VR_FROM_FC { \
    2227, \
    "UPDATE_VR_FROM_FC", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_update_vr_from_fc_t, update_time) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_update_vr_from_fc_t, flight_mode) }, \
         { "ctrl_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_update_vr_from_fc_t, ctrl_mode) }, \
         { "acc_x", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_update_vr_from_fc_t, acc_x) }, \
         { "acc_y", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_update_vr_from_fc_t, acc_y) }, \
         { "acc_u", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_update_vr_from_fc_t, acc_u) }, \
         { "yaw_ctrl", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_update_vr_from_fc_t, yaw_ctrl) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_update_vr_from_fc_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_update_vr_from_fc_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_update_vr_from_fc_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_update_vr_from_fc_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_UPDATE_VR_FROM_FC { \
    "UPDATE_VR_FROM_FC", \
    11, \
    {  { "update_time", NULL, TTALINK_TYPE_UINT32_T, 0, 0, offsetof(ttalink_update_vr_from_fc_t, update_time) }, \
         { "flight_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_update_vr_from_fc_t, flight_mode) }, \
         { "ctrl_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 5, offsetof(ttalink_update_vr_from_fc_t, ctrl_mode) }, \
         { "acc_x", NULL, TTALINK_TYPE_FLOAT, 0, 6, offsetof(ttalink_update_vr_from_fc_t, acc_x) }, \
         { "acc_y", NULL, TTALINK_TYPE_FLOAT, 0, 10, offsetof(ttalink_update_vr_from_fc_t, acc_y) }, \
         { "acc_u", NULL, TTALINK_TYPE_FLOAT, 0, 14, offsetof(ttalink_update_vr_from_fc_t, acc_u) }, \
         { "yaw_ctrl", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_update_vr_from_fc_t, yaw_ctrl) }, \
         { "param1", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_update_vr_from_fc_t, param1) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_update_vr_from_fc_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_update_vr_from_fc_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_update_vr_from_fc_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_update_vr_from_fc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, ctrl_mode);
    _tta_put_float(buf, 6, acc_x);
    _tta_put_float(buf, 10, acc_y);
    _tta_put_float(buf, 14, acc_u);
    _tta_put_float(buf, 18, yaw_ctrl);
    _tta_put_float(buf, 22, param1);
    _tta_put_float(buf, 26, param2);
    _tta_put_float(buf, 30, param3);
    _tta_put_float(buf, 34, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN);
#else
    ttalink_update_vr_from_fc_t packet;
    packet.update_time = update_time;
    packet.flight_mode = flight_mode;
    packet.ctrl_mode = ctrl_mode;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_u = acc_u;
    packet.yaw_ctrl = yaw_ctrl;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPDATE_VR_FROM_FC;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC, nocrc);
}

/**
 * @brief Pack a update_vr_from_fc message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param flight_mode 0：模拟航线飞行  1：模拟遥控器飞行.
 * @param ctrl_mode   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.
 * @param acc_x  （符合右手定责，前右下坐标系）（m/s²）.
 * @param acc_y （符合右手定责，前右下坐标系）（m/s²） .
 * @param acc_u  （NED坐标系）（m/s²）.
 * @param yaw_ctrl  （符合右手定责，前右下坐标系）（转矩）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_from_fc_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4)
{
    return _ttalink_update_vr_from_fc_pack(dst_addr, src_addr, msg,  update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a update_vr_from_fc message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param update_time  .
 * @param flight_mode 0：模拟航线飞行  1：模拟遥控器飞行.
 * @param ctrl_mode   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.
 * @param acc_x  （符合右手定责，前右下坐标系）（m/s²）.
 * @param acc_y （符合右手定责，前右下坐标系）（m/s²） .
 * @param acc_u  （NED坐标系）（m/s²）.
 * @param yaw_ctrl  （符合右手定责，前右下坐标系）（转矩）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_from_fc_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4)
{
    return _ttalink_update_vr_from_fc_pack(dst_addr, src_addr, msg,  update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_update_vr_from_fc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t flight_mode,uint8_t ctrl_mode,float acc_x,float acc_y,float acc_u,float yaw_ctrl,float param1,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, ctrl_mode);
    _tta_put_float(buf, 6, acc_x);
    _tta_put_float(buf, 10, acc_y);
    _tta_put_float(buf, 14, acc_u);
    _tta_put_float(buf, 18, yaw_ctrl);
    _tta_put_float(buf, 22, param1);
    _tta_put_float(buf, 26, param2);
    _tta_put_float(buf, 30, param3);
    _tta_put_float(buf, 34, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN);
#else
    ttalink_update_vr_from_fc_t packet;
    packet.update_time = update_time;
    packet.flight_mode = flight_mode;
    packet.ctrl_mode = ctrl_mode;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_u = acc_u;
    packet.yaw_ctrl = yaw_ctrl;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_UPDATE_VR_FROM_FC;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC, nocrc);
}

/**
 * @brief Pack a update_vr_from_fc message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param flight_mode 0：模拟航线飞行  1：模拟遥控器飞行.
 * @param ctrl_mode   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.
 * @param acc_x  （符合右手定责，前右下坐标系）（m/s²）.
 * @param acc_y （符合右手定责，前右下坐标系）（m/s²） .
 * @param acc_u  （NED坐标系）（m/s²）.
 * @param yaw_ctrl  （符合右手定责，前右下坐标系）（转矩）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_from_fc_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t flight_mode,uint8_t ctrl_mode,float acc_x,float acc_y,float acc_u,float yaw_ctrl,float param1,float param2,float param3,float param4)
{
    return _ttalink_update_vr_from_fc_pack_chan(dst_addr, src_addr, chan, msg,  update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, false);
}

/**
 * @brief Pack a update_vr_from_fc message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_time  .
 * @param flight_mode 0：模拟航线飞行  1：模拟遥控器飞行.
 * @param ctrl_mode   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.
 * @param acc_x  （符合右手定责，前右下坐标系）（m/s²）.
 * @param acc_y （符合右手定责，前右下坐标系）（m/s²） .
 * @param acc_u  （NED坐标系）（m/s²）.
 * @param yaw_ctrl  （符合右手定责，前右下坐标系）（转矩）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_update_vr_from_fc_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint32_t update_time,uint8_t flight_mode,uint8_t ctrl_mode,float acc_x,float acc_y,float acc_u,float yaw_ctrl,float param1,float param2,float param3,float param4)
{
    return _ttalink_update_vr_from_fc_pack_chan(dst_addr, src_addr, chan, msg,  update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, true);
}


static inline uint16_t _ttalink_update_vr_from_fc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_from_fc_t* update_vr_from_fc, bool nocrc)
{
    if(nocrc){
        return ttalink_update_vr_from_fc_pack_nocrc(dst_addr, src_addr, msg, update_vr_from_fc->update_time, update_vr_from_fc->flight_mode, update_vr_from_fc->ctrl_mode, update_vr_from_fc->acc_x, update_vr_from_fc->acc_y, update_vr_from_fc->acc_u, update_vr_from_fc->yaw_ctrl, update_vr_from_fc->param1, update_vr_from_fc->param2, update_vr_from_fc->param3, update_vr_from_fc->param4);
    }else{
        return ttalink_update_vr_from_fc_pack(dst_addr, src_addr, msg, update_vr_from_fc->update_time, update_vr_from_fc->flight_mode, update_vr_from_fc->ctrl_mode, update_vr_from_fc->acc_x, update_vr_from_fc->acc_y, update_vr_from_fc->acc_u, update_vr_from_fc->yaw_ctrl, update_vr_from_fc->param1, update_vr_from_fc->param2, update_vr_from_fc->param3, update_vr_from_fc->param4);
    }
    
}

/**
 * @brief Encode a update_vr_from_fc struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_from_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_from_fc_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_from_fc_t* update_vr_from_fc)
{
    return _ttalink_update_vr_from_fc_encode(dst_addr, src_addr, msg, update_vr_from_fc, false);
}

/**
 * @brief Encode a update_vr_from_fc struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_from_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_from_fc_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_update_vr_from_fc_t* update_vr_from_fc)
{
    return _ttalink_update_vr_from_fc_encode(dst_addr, src_addr, msg, update_vr_from_fc, true);
}


static inline uint16_t _ttalink_update_vr_from_fc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_from_fc_t* update_vr_from_fc, bool nocrc)
{
    if(nocrc){
        return ttalink_update_vr_from_fc_pack_chan_nocrc(dst_addr, src_addr, chan, msg, update_vr_from_fc->update_time, update_vr_from_fc->flight_mode, update_vr_from_fc->ctrl_mode, update_vr_from_fc->acc_x, update_vr_from_fc->acc_y, update_vr_from_fc->acc_u, update_vr_from_fc->yaw_ctrl, update_vr_from_fc->param1, update_vr_from_fc->param2, update_vr_from_fc->param3, update_vr_from_fc->param4);
    }else{
        return ttalink_update_vr_from_fc_pack_chan(dst_addr, src_addr, chan, msg, update_vr_from_fc->update_time, update_vr_from_fc->flight_mode, update_vr_from_fc->ctrl_mode, update_vr_from_fc->acc_x, update_vr_from_fc->acc_y, update_vr_from_fc->acc_u, update_vr_from_fc->yaw_ctrl, update_vr_from_fc->param1, update_vr_from_fc->param2, update_vr_from_fc->param3, update_vr_from_fc->param4);
    }
}

/**
 * @brief Encode a update_vr_from_fc struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_from_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_from_fc_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_from_fc_t* update_vr_from_fc)
{
    return _ttalink_update_vr_from_fc_encode_chan(dst_addr, src_addr, chan, msg, update_vr_from_fc, false);
}

/**
 * @brief Encode a update_vr_from_fc struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param update_vr_from_fc C-struct to read the message contents from
 */
static inline uint16_t ttalink_update_vr_from_fc_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_update_vr_from_fc_t* update_vr_from_fc)
{
    return _ttalink_update_vr_from_fc_encode_chan(dst_addr, src_addr, chan, msg, update_vr_from_fc, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_update_vr_from_fc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN];
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, ctrl_mode);
    _tta_put_float(buf, 6, acc_x);
    _tta_put_float(buf, 10, acc_y);
    _tta_put_float(buf, 14, acc_u);
    _tta_put_float(buf, 18, yaw_ctrl);
    _tta_put_float(buf, 22, param1);
    _tta_put_float(buf, 26, param2);
    _tta_put_float(buf, 30, param3);
    _tta_put_float(buf, 34, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_FROM_FC, buf, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC, nocrc);
#else
    ttalink_update_vr_from_fc_t packet;
    packet.update_time = update_time;
    packet.flight_mode = flight_mode;
    packet.ctrl_mode = ctrl_mode;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_u = acc_u;
    packet.yaw_ctrl = yaw_ctrl;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_FROM_FC, (const char *)&packet, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC, nocrc);
#endif
}

/**
 * @brief Send a update_vr_from_fc message
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param flight_mode 0：模拟航线飞行  1：模拟遥控器飞行.
 * @param ctrl_mode   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.
 * @param acc_x  （符合右手定责，前右下坐标系）（m/s²）.
 * @param acc_y （符合右手定责，前右下坐标系）（m/s²） .
 * @param acc_u  （NED坐标系）（m/s²）.
 * @param yaw_ctrl  （符合右手定责，前右下坐标系）（转矩）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_update_vr_from_fc_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_from_fc_send(dst_addr, src_addr, chan, update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, false);
}

/**
 * @brief Send a update_vr_from_fc message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param update_time  .
 * @param flight_mode 0：模拟航线飞行  1：模拟遥控器飞行.
 * @param ctrl_mode   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.
 * @param acc_x  （符合右手定责，前右下坐标系）（m/s²）.
 * @param acc_y （符合右手定责，前右下坐标系）（m/s²） .
 * @param acc_u  （NED坐标系）（m/s²）.
 * @param yaw_ctrl  （符合右手定责，前右下坐标系）（转矩）.
 * @param param1  预留.
 * @param param2  预留.
 * @param param3  预留.
 * @param param4  预留.
 */
static inline void ttalink_update_vr_from_fc_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_from_fc_send(dst_addr, src_addr, chan, update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, true);
}


static inline void _ttalink_update_vr_from_fc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_from_fc_t* update_vr_from_fc, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_update_vr_from_fc_send_nocrc(dst_addr, src_addr, chan, update_vr_from_fc->update_time, update_vr_from_fc->flight_mode, update_vr_from_fc->ctrl_mode, update_vr_from_fc->acc_x, update_vr_from_fc->acc_y, update_vr_from_fc->acc_u, update_vr_from_fc->yaw_ctrl, update_vr_from_fc->param1, update_vr_from_fc->param2, update_vr_from_fc->param3, update_vr_from_fc->param4);
    }else{
        ttalink_update_vr_from_fc_send(dst_addr, src_addr, chan, update_vr_from_fc->update_time, update_vr_from_fc->flight_mode, update_vr_from_fc->ctrl_mode, update_vr_from_fc->acc_x, update_vr_from_fc->acc_y, update_vr_from_fc->acc_u, update_vr_from_fc->yaw_ctrl, update_vr_from_fc->param1, update_vr_from_fc->param2, update_vr_from_fc->param3, update_vr_from_fc->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_FROM_FC, (const char *)update_vr_from_fc, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC, nocrc);
#endif
}

/**
 * @brief Send a update_vr_from_fc message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_update_vr_from_fc_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_from_fc_t* update_vr_from_fc)
{
    _ttalink_update_vr_from_fc_send_struct(dst_addr, src_addr, chan, update_vr_from_fc, false);
}

/**
 * @brief Send a update_vr_from_fc message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_update_vr_from_fc_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_update_vr_from_fc_t* update_vr_from_fc)
{
    _ttalink_update_vr_from_fc_send_struct(dst_addr, src_addr, chan, update_vr_from_fc, true);
}

#if TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_update_vr_from_fc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint32_t(buf, 0, update_time);
    _tta_put_uint8_t(buf, 4, flight_mode);
    _tta_put_uint8_t(buf, 5, ctrl_mode);
    _tta_put_float(buf, 6, acc_x);
    _tta_put_float(buf, 10, acc_y);
    _tta_put_float(buf, 14, acc_u);
    _tta_put_float(buf, 18, yaw_ctrl);
    _tta_put_float(buf, 22, param1);
    _tta_put_float(buf, 26, param2);
    _tta_put_float(buf, 30, param3);
    _tta_put_float(buf, 34, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_FROM_FC, buf, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC, nocrc);
#else
    ttalink_update_vr_from_fc_t *packet = (ttalink_update_vr_from_fc_t *)msgbuf;
    packet->update_time = update_time;
    packet->flight_mode = flight_mode;
    packet->ctrl_mode = ctrl_mode;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_u = acc_u;
    packet->yaw_ctrl = yaw_ctrl;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_UPDATE_VR_FROM_FC, (const char *)packet, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_MIN_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_update_vr_from_fc_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_from_fc_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_update_vr_from_fc_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint32_t update_time, uint8_t flight_mode, uint8_t ctrl_mode, float acc_x, float acc_y, float acc_u, float yaw_ctrl, float param1, float param2, float param3, float param4)
{
    _ttalink_update_vr_from_fc_send_buf(dst_addr, src_addr, msgbuf, chan, update_time, flight_mode, ctrl_mode, acc_x, acc_y, acc_u, yaw_ctrl, param1, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE UPDATE_VR_FROM_FC UNPACKING


/**
 * @brief Get field update_time from update_vr_from_fc message
 *
 * @return  .
 */
static inline uint32_t ttalink_update_vr_from_fc_get_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flight_mode from update_vr_from_fc message
 *
 * @return 0：模拟航线飞行  1：模拟遥控器飞行.
 */
static inline uint8_t ttalink_update_vr_from_fc_get_flight_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ctrl_mode from update_vr_from_fc message
 *
 * @return   0：航线模式  1：手动模式  2：姿态模式 3：GPS模式.
 */
static inline uint8_t ttalink_update_vr_from_fc_get_ctrl_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field acc_x from update_vr_from_fc message
 *
 * @return  （符合右手定责，前右下坐标系）（m/s²）.
 */
static inline float ttalink_update_vr_from_fc_get_acc_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  6);
}

/**
 * @brief Get field acc_y from update_vr_from_fc message
 *
 * @return （符合右手定责，前右下坐标系）（m/s²） .
 */
static inline float ttalink_update_vr_from_fc_get_acc_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  10);
}

/**
 * @brief Get field acc_u from update_vr_from_fc message
 *
 * @return  （NED坐标系）（m/s²）.
 */
static inline float ttalink_update_vr_from_fc_get_acc_u(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  14);
}

/**
 * @brief Get field yaw_ctrl from update_vr_from_fc message
 *
 * @return  （符合右手定责，前右下坐标系）（转矩）.
 */
static inline float ttalink_update_vr_from_fc_get_yaw_ctrl(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  18);
}

/**
 * @brief Get field param1 from update_vr_from_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_from_fc_get_param1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  22);
}

/**
 * @brief Get field param2 from update_vr_from_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_from_fc_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Get field param3 from update_vr_from_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_from_fc_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  30);
}

/**
 * @brief Get field param4 from update_vr_from_fc message
 *
 * @return  预留.
 */
static inline float ttalink_update_vr_from_fc_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  34);
}

/**
 * @brief Decode a update_vr_from_fc message into a struct
 *
 * @param msg The message to decode
 * @param update_vr_from_fc C-struct to decode the message contents into
 */
static inline void ttalink_update_vr_from_fc_decode(const ttalink_message_t* msg, ttalink_update_vr_from_fc_t* update_vr_from_fc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    update_vr_from_fc->update_time = ttalink_update_vr_from_fc_get_update_time(msg);
    update_vr_from_fc->flight_mode = ttalink_update_vr_from_fc_get_flight_mode(msg);
    update_vr_from_fc->ctrl_mode = ttalink_update_vr_from_fc_get_ctrl_mode(msg);
    update_vr_from_fc->acc_x = ttalink_update_vr_from_fc_get_acc_x(msg);
    update_vr_from_fc->acc_y = ttalink_update_vr_from_fc_get_acc_y(msg);
    update_vr_from_fc->acc_u = ttalink_update_vr_from_fc_get_acc_u(msg);
    update_vr_from_fc->yaw_ctrl = ttalink_update_vr_from_fc_get_yaw_ctrl(msg);
    update_vr_from_fc->param1 = ttalink_update_vr_from_fc_get_param1(msg);
    update_vr_from_fc->param2 = ttalink_update_vr_from_fc_get_param2(msg);
    update_vr_from_fc->param3 = ttalink_update_vr_from_fc_get_param3(msg);
    update_vr_from_fc->param4 = ttalink_update_vr_from_fc_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN? msg->len : TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN;
        memset(update_vr_from_fc, 0, TTALINK_MSG_ID_UPDATE_VR_FROM_FC_LEN);
    memcpy(update_vr_from_fc, _TTA_PAYLOAD(msg), len);
#endif
}
