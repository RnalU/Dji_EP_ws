#pragma once
// MESSAGE POD_1376_SET_PTZ_PID_PARA PACKING

#define TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA 11376

TTAPACKED(
typedef struct __ttalink_pod_1376_set_ptz_pid_para_t {
 uint8_t switch_mode; /*< 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.*/
 float kp; /*< .*/
 float ki; /*< .*/
 float kd; /*< .*/
 float kp_limit; /*< .*/
 float ki_limit; /*< .*/
 float kd_limit; /*< .*/
 float out_limit; /*< .*/
 uint8_t ack; /*< .*/
}) ttalink_pod_1376_set_ptz_pid_para_t;

#define TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN 30
#define TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN 30
#define TTALINK_MSG_ID_11376_LEN 30
#define TTALINK_MSG_ID_11376_MIN_LEN 30

#define TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC 171
#define TTALINK_MSG_ID_11376_CRC 171



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1376_SET_PTZ_PID_PARA { \
    11376, \
    "POD_1376_SET_PTZ_PID_PARA", \
    9, \
    {  { "switch_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, switch_mode) }, \
         { "kp", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kp) }, \
         { "ki", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, ki) }, \
         { "kd", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kd) }, \
         { "kp_limit", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kp_limit) }, \
         { "ki_limit", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, ki_limit) }, \
         { "kd_limit", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kd_limit) }, \
         { "out_limit", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, out_limit) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 29, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, ack) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1376_SET_PTZ_PID_PARA { \
    "POD_1376_SET_PTZ_PID_PARA", \
    9, \
    {  { "switch_mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, switch_mode) }, \
         { "kp", NULL, TTALINK_TYPE_FLOAT, 0, 1, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kp) }, \
         { "ki", NULL, TTALINK_TYPE_FLOAT, 0, 5, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, ki) }, \
         { "kd", NULL, TTALINK_TYPE_FLOAT, 0, 9, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kd) }, \
         { "kp_limit", NULL, TTALINK_TYPE_FLOAT, 0, 13, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kp_limit) }, \
         { "ki_limit", NULL, TTALINK_TYPE_FLOAT, 0, 17, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, ki_limit) }, \
         { "kd_limit", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, kd_limit) }, \
         { "out_limit", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, out_limit) }, \
         { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 29, offsetof(ttalink_pod_1376_set_ptz_pid_para_t, ack) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1376_set_ptz_pid_para_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN];
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_float(buf, 1, kp);
    _tta_put_float(buf, 5, ki);
    _tta_put_float(buf, 9, kd);
    _tta_put_float(buf, 13, kp_limit);
    _tta_put_float(buf, 17, ki_limit);
    _tta_put_float(buf, 21, kd_limit);
    _tta_put_float(buf, 25, out_limit);
    _tta_put_uint8_t(buf, 29, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN);
#else
    ttalink_pod_1376_set_ptz_pid_para_t packet;
    packet.switch_mode = switch_mode;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.kp_limit = kp_limit;
    packet.ki_limit = ki_limit;
    packet.kd_limit = kd_limit;
    packet.out_limit = out_limit;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1376_set_ptz_pid_para message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_mode 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.
 * @param kp .
 * @param ki .
 * @param kd .
 * @param kp_limit .
 * @param ki_limit .
 * @param kd_limit .
 * @param out_limit .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack)
{
    return _ttalink_pod_1376_set_ptz_pid_para_pack(dst_addr, src_addr, msg,  switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, false);
}

/**
 * @brief Pack a pod_1376_set_ptz_pid_para message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param switch_mode 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.
 * @param kp .
 * @param ki .
 * @param kd .
 * @param kp_limit .
 * @param ki_limit .
 * @param kd_limit .
 * @param out_limit .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack)
{
    return _ttalink_pod_1376_set_ptz_pid_para_pack(dst_addr, src_addr, msg,  switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, true);
}


static inline uint16_t _ttalink_pod_1376_set_ptz_pid_para_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_mode,float kp,float ki,float kd,float kp_limit,float ki_limit,float kd_limit,float out_limit,uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN];
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_float(buf, 1, kp);
    _tta_put_float(buf, 5, ki);
    _tta_put_float(buf, 9, kd);
    _tta_put_float(buf, 13, kp_limit);
    _tta_put_float(buf, 17, ki_limit);
    _tta_put_float(buf, 21, kd_limit);
    _tta_put_float(buf, 25, out_limit);
    _tta_put_uint8_t(buf, 29, ack);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN);
#else
    ttalink_pod_1376_set_ptz_pid_para_t packet;
    packet.switch_mode = switch_mode;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.kp_limit = kp_limit;
    packet.ki_limit = ki_limit;
    packet.kd_limit = kd_limit;
    packet.out_limit = out_limit;
    packet.ack = ack;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC, nocrc);
}

/**
 * @brief Pack a pod_1376_set_ptz_pid_para message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_mode 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.
 * @param kp .
 * @param ki .
 * @param kd .
 * @param kp_limit .
 * @param ki_limit .
 * @param kd_limit .
 * @param out_limit .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_mode,float kp,float ki,float kd,float kp_limit,float ki_limit,float kd_limit,float out_limit,uint8_t ack)
{
    return _ttalink_pod_1376_set_ptz_pid_para_pack_chan(dst_addr, src_addr, chan, msg,  switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, false);
}

/**
 * @brief Pack a pod_1376_set_ptz_pid_para message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param switch_mode 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.
 * @param kp .
 * @param ki .
 * @param kd .
 * @param kp_limit .
 * @param ki_limit .
 * @param kd_limit .
 * @param out_limit .
 * @param ack .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t switch_mode,float kp,float ki,float kd,float kp_limit,float ki_limit,float kd_limit,float out_limit,uint8_t ack)
{
    return _ttalink_pod_1376_set_ptz_pid_para_pack_chan(dst_addr, src_addr, chan, msg,  switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, true);
}


static inline uint16_t _ttalink_pod_1376_set_ptz_pid_para_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1376_set_ptz_pid_para_pack_nocrc(dst_addr, src_addr, msg, pod_1376_set_ptz_pid_para->switch_mode, pod_1376_set_ptz_pid_para->kp, pod_1376_set_ptz_pid_para->ki, pod_1376_set_ptz_pid_para->kd, pod_1376_set_ptz_pid_para->kp_limit, pod_1376_set_ptz_pid_para->ki_limit, pod_1376_set_ptz_pid_para->kd_limit, pod_1376_set_ptz_pid_para->out_limit, pod_1376_set_ptz_pid_para->ack);
    }else{
        return ttalink_pod_1376_set_ptz_pid_para_pack(dst_addr, src_addr, msg, pod_1376_set_ptz_pid_para->switch_mode, pod_1376_set_ptz_pid_para->kp, pod_1376_set_ptz_pid_para->ki, pod_1376_set_ptz_pid_para->kd, pod_1376_set_ptz_pid_para->kp_limit, pod_1376_set_ptz_pid_para->ki_limit, pod_1376_set_ptz_pid_para->kd_limit, pod_1376_set_ptz_pid_para->out_limit, pod_1376_set_ptz_pid_para->ack);
    }
    
}

/**
 * @brief Encode a pod_1376_set_ptz_pid_para struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1376_set_ptz_pid_para C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para)
{
    return _ttalink_pod_1376_set_ptz_pid_para_encode(dst_addr, src_addr, msg, pod_1376_set_ptz_pid_para, false);
}

/**
 * @brief Encode a pod_1376_set_ptz_pid_para struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1376_set_ptz_pid_para C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para)
{
    return _ttalink_pod_1376_set_ptz_pid_para_encode(dst_addr, src_addr, msg, pod_1376_set_ptz_pid_para, true);
}


static inline uint16_t _ttalink_pod_1376_set_ptz_pid_para_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1376_set_ptz_pid_para_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1376_set_ptz_pid_para->switch_mode, pod_1376_set_ptz_pid_para->kp, pod_1376_set_ptz_pid_para->ki, pod_1376_set_ptz_pid_para->kd, pod_1376_set_ptz_pid_para->kp_limit, pod_1376_set_ptz_pid_para->ki_limit, pod_1376_set_ptz_pid_para->kd_limit, pod_1376_set_ptz_pid_para->out_limit, pod_1376_set_ptz_pid_para->ack);
    }else{
        return ttalink_pod_1376_set_ptz_pid_para_pack_chan(dst_addr, src_addr, chan, msg, pod_1376_set_ptz_pid_para->switch_mode, pod_1376_set_ptz_pid_para->kp, pod_1376_set_ptz_pid_para->ki, pod_1376_set_ptz_pid_para->kd, pod_1376_set_ptz_pid_para->kp_limit, pod_1376_set_ptz_pid_para->ki_limit, pod_1376_set_ptz_pid_para->kd_limit, pod_1376_set_ptz_pid_para->out_limit, pod_1376_set_ptz_pid_para->ack);
    }
}

/**
 * @brief Encode a pod_1376_set_ptz_pid_para struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1376_set_ptz_pid_para C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para)
{
    return _ttalink_pod_1376_set_ptz_pid_para_encode_chan(dst_addr, src_addr, chan, msg, pod_1376_set_ptz_pid_para, false);
}

/**
 * @brief Encode a pod_1376_set_ptz_pid_para struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1376_set_ptz_pid_para C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1376_set_ptz_pid_para_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para)
{
    return _ttalink_pod_1376_set_ptz_pid_para_encode_chan(dst_addr, src_addr, chan, msg, pod_1376_set_ptz_pid_para, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1376_set_ptz_pid_para_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN];
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_float(buf, 1, kp);
    _tta_put_float(buf, 5, ki);
    _tta_put_float(buf, 9, kd);
    _tta_put_float(buf, 13, kp_limit);
    _tta_put_float(buf, 17, ki_limit);
    _tta_put_float(buf, 21, kd_limit);
    _tta_put_float(buf, 25, out_limit);
    _tta_put_uint8_t(buf, 29, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA, buf, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC, nocrc);
#else
    ttalink_pod_1376_set_ptz_pid_para_t packet;
    packet.switch_mode = switch_mode;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.kp_limit = kp_limit;
    packet.ki_limit = ki_limit;
    packet.kd_limit = kd_limit;
    packet.out_limit = out_limit;
    packet.ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA, (const char *)&packet, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1376_set_ptz_pid_para message
 * @param chan TTAlink channel to send the message
 *
 * @param switch_mode 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.
 * @param kp .
 * @param ki .
 * @param kd .
 * @param kp_limit .
 * @param ki_limit .
 * @param kd_limit .
 * @param out_limit .
 * @param ack .
 */
static inline void ttalink_pod_1376_set_ptz_pid_para_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack)
{
    _ttalink_pod_1376_set_ptz_pid_para_send(dst_addr, src_addr, chan, switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, false);
}

/**
 * @brief Send a pod_1376_set_ptz_pid_para message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param switch_mode 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.
 * @param kp .
 * @param ki .
 * @param kd .
 * @param kp_limit .
 * @param ki_limit .
 * @param kd_limit .
 * @param out_limit .
 * @param ack .
 */
static inline void ttalink_pod_1376_set_ptz_pid_para_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack)
{
    _ttalink_pod_1376_set_ptz_pid_para_send(dst_addr, src_addr, chan, switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, true);
}


static inline void _ttalink_pod_1376_set_ptz_pid_para_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1376_set_ptz_pid_para_send_nocrc(dst_addr, src_addr, chan, pod_1376_set_ptz_pid_para->switch_mode, pod_1376_set_ptz_pid_para->kp, pod_1376_set_ptz_pid_para->ki, pod_1376_set_ptz_pid_para->kd, pod_1376_set_ptz_pid_para->kp_limit, pod_1376_set_ptz_pid_para->ki_limit, pod_1376_set_ptz_pid_para->kd_limit, pod_1376_set_ptz_pid_para->out_limit, pod_1376_set_ptz_pid_para->ack);
    }else{
        ttalink_pod_1376_set_ptz_pid_para_send(dst_addr, src_addr, chan, pod_1376_set_ptz_pid_para->switch_mode, pod_1376_set_ptz_pid_para->kp, pod_1376_set_ptz_pid_para->ki, pod_1376_set_ptz_pid_para->kd, pod_1376_set_ptz_pid_para->kp_limit, pod_1376_set_ptz_pid_para->ki_limit, pod_1376_set_ptz_pid_para->kd_limit, pod_1376_set_ptz_pid_para->out_limit, pod_1376_set_ptz_pid_para->ack);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA, (const char *)pod_1376_set_ptz_pid_para, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1376_set_ptz_pid_para message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1376_set_ptz_pid_para_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para)
{
    _ttalink_pod_1376_set_ptz_pid_para_send_struct(dst_addr, src_addr, chan, pod_1376_set_ptz_pid_para, false);
}

/**
 * @brief Send a pod_1376_set_ptz_pid_para message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1376_set_ptz_pid_para_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para)
{
    _ttalink_pod_1376_set_ptz_pid_para_send_struct(dst_addr, src_addr, chan, pod_1376_set_ptz_pid_para, true);
}

#if TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1376_set_ptz_pid_para_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, switch_mode);
    _tta_put_float(buf, 1, kp);
    _tta_put_float(buf, 5, ki);
    _tta_put_float(buf, 9, kd);
    _tta_put_float(buf, 13, kp_limit);
    _tta_put_float(buf, 17, ki_limit);
    _tta_put_float(buf, 21, kd_limit);
    _tta_put_float(buf, 25, out_limit);
    _tta_put_uint8_t(buf, 29, ack);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA, buf, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC, nocrc);
#else
    ttalink_pod_1376_set_ptz_pid_para_t *packet = (ttalink_pod_1376_set_ptz_pid_para_t *)msgbuf;
    packet->switch_mode = switch_mode;
    packet->kp = kp;
    packet->ki = ki;
    packet->kd = kd;
    packet->kp_limit = kp_limit;
    packet->ki_limit = ki_limit;
    packet->kd_limit = kd_limit;
    packet->out_limit = out_limit;
    packet->ack = ack;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA, (const char *)packet, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_MIN_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1376_set_ptz_pid_para_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack)
{
    _ttalink_pod_1376_set_ptz_pid_para_send_buf(dst_addr, src_addr, msgbuf, chan, switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1376_set_ptz_pid_para_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t switch_mode, float kp, float ki, float kd, float kp_limit, float ki_limit, float kd_limit, float out_limit, uint8_t ack)
{
    _ttalink_pod_1376_set_ptz_pid_para_send_buf(dst_addr, src_addr, msgbuf, chan, switch_mode, kp, ki, kd, kp_limit, ki_limit, kd_limit, out_limit, ack, true);
}
#endif

#endif

// MESSAGE POD_1376_SET_PTZ_PID_PARA UNPACKING


/**
 * @brief Get field switch_mode from pod_1376_set_ptz_pid_para message
 *
 * @return 0=NULL 1=GyroX 2=GyroY 3=GyroZ 
											4=Atti-Pitch 5=Atti-Roll 6=Atti-Yaw.
 */
static inline uint8_t ttalink_pod_1376_set_ptz_pid_para_get_switch_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field kp from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline float ttalink_pod_1376_set_ptz_pid_para_get_kp(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  1);
}

/**
 * @brief Get field ki from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline float ttalink_pod_1376_set_ptz_pid_para_get_ki(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  5);
}

/**
 * @brief Get field kd from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline float ttalink_pod_1376_set_ptz_pid_para_get_kd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  9);
}

/**
 * @brief Get field kp_limit from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline float ttalink_pod_1376_set_ptz_pid_para_get_kp_limit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  13);
}

/**
 * @brief Get field ki_limit from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline float ttalink_pod_1376_set_ptz_pid_para_get_ki_limit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  17);
}

/**
 * @brief Get field kd_limit from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline float ttalink_pod_1376_set_ptz_pid_para_get_kd_limit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field out_limit from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline float ttalink_pod_1376_set_ptz_pid_para_get_out_limit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field ack from pod_1376_set_ptz_pid_para message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1376_set_ptz_pid_para_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Decode a pod_1376_set_ptz_pid_para message into a struct
 *
 * @param msg The message to decode
 * @param pod_1376_set_ptz_pid_para C-struct to decode the message contents into
 */
static inline void ttalink_pod_1376_set_ptz_pid_para_decode(const ttalink_message_t* msg, ttalink_pod_1376_set_ptz_pid_para_t* pod_1376_set_ptz_pid_para)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1376_set_ptz_pid_para->switch_mode = ttalink_pod_1376_set_ptz_pid_para_get_switch_mode(msg);
    pod_1376_set_ptz_pid_para->kp = ttalink_pod_1376_set_ptz_pid_para_get_kp(msg);
    pod_1376_set_ptz_pid_para->ki = ttalink_pod_1376_set_ptz_pid_para_get_ki(msg);
    pod_1376_set_ptz_pid_para->kd = ttalink_pod_1376_set_ptz_pid_para_get_kd(msg);
    pod_1376_set_ptz_pid_para->kp_limit = ttalink_pod_1376_set_ptz_pid_para_get_kp_limit(msg);
    pod_1376_set_ptz_pid_para->ki_limit = ttalink_pod_1376_set_ptz_pid_para_get_ki_limit(msg);
    pod_1376_set_ptz_pid_para->kd_limit = ttalink_pod_1376_set_ptz_pid_para_get_kd_limit(msg);
    pod_1376_set_ptz_pid_para->out_limit = ttalink_pod_1376_set_ptz_pid_para_get_out_limit(msg);
    pod_1376_set_ptz_pid_para->ack = ttalink_pod_1376_set_ptz_pid_para_get_ack(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN? msg->len : TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN;
        memset(pod_1376_set_ptz_pid_para, 0, TTALINK_MSG_ID_POD_1376_SET_PTZ_PID_PARA_LEN);
    memcpy(pod_1376_set_ptz_pid_para, _TTA_PAYLOAD(msg), len);
#endif
}
