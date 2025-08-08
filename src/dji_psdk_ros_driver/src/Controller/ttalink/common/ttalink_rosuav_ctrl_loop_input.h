#pragma once
// MESSAGE ROSUAV_CTRL_LOOP_INPUT PACKING

#define TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT 8200

TTAPACKED(
typedef struct __ttalink_rosuav_ctrl_loop_input_t {
 uint8_t mode; /*<  预留模式.*/
 uint8_t mode_param; /*<  模式参数.*/
 uint8_t flight_ctrl_status; /*<  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量*/
 uint8_t ctrl_param; /*<  */
 uint8_t unlock_state; /*<  =0 上锁. 1=解锁*/
 double latit; /*<  .*/
 double longi; /*<  .*/
 float altit; /*<  .*/
 float velN; /*<  .*/
 float velE; /*<  .*/
 float velD; /*<  .*/
 float accN; /*<  N向加速度前馈.*/
 float accE; /*<  E向加速度前馈.*/
 float accD; /*<  D向加速度前馈.*/
 float atti_pitch; /*<  .*/
 float atti_roll; /*<  .*/
 float atti_yaw; /*<  .*/
 float gyro_pitch; /*<  .*/
 float gyro_roll; /*<  .*/
 float gyro_yaw; /*<  航向角速度.*/
 float thro_thrust; /*<  油门推力.*/
 float param[4]; /*<  .*/
}) ttalink_rosuav_ctrl_loop_input_t;

#define TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN 93
#define TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN 93
#define TTALINK_MSG_ID_8200_LEN 93
#define TTALINK_MSG_ID_8200_MIN_LEN 93

#define TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC 21
#define TTALINK_MSG_ID_8200_CRC 21

#define TTALINK_MSG_ROSUAV_CTRL_LOOP_INPUT_FIELD_PARAM_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_CTRL_LOOP_INPUT { \
    8200, \
    "ROSUAV_CTRL_LOOP_INPUT", \
    22, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rosuav_ctrl_loop_input_t, mode) }, \
         { "mode_param", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rosuav_ctrl_loop_input_t, mode_param) }, \
         { "flight_ctrl_status", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_rosuav_ctrl_loop_input_t, flight_ctrl_status) }, \
         { "ctrl_param", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_rosuav_ctrl_loop_input_t, ctrl_param) }, \
         { "unlock_state", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_rosuav_ctrl_loop_input_t, unlock_state) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 5, offsetof(ttalink_rosuav_ctrl_loop_input_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 13, offsetof(ttalink_rosuav_ctrl_loop_input_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_rosuav_ctrl_loop_input_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_rosuav_ctrl_loop_input_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_rosuav_ctrl_loop_input_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_rosuav_ctrl_loop_input_t, velD) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_rosuav_ctrl_loop_input_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_rosuav_ctrl_loop_input_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_rosuav_ctrl_loop_input_t, accD) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 49, offsetof(ttalink_rosuav_ctrl_loop_input_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 53, offsetof(ttalink_rosuav_ctrl_loop_input_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 57, offsetof(ttalink_rosuav_ctrl_loop_input_t, atti_yaw) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_rosuav_ctrl_loop_input_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 65, offsetof(ttalink_rosuav_ctrl_loop_input_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 69, offsetof(ttalink_rosuav_ctrl_loop_input_t, gyro_yaw) }, \
         { "thro_thrust", NULL, TTALINK_TYPE_FLOAT, 0, 73, offsetof(ttalink_rosuav_ctrl_loop_input_t, thro_thrust) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 77, offsetof(ttalink_rosuav_ctrl_loop_input_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_CTRL_LOOP_INPUT { \
    "ROSUAV_CTRL_LOOP_INPUT", \
    22, \
    {  { "mode", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rosuav_ctrl_loop_input_t, mode) }, \
         { "mode_param", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rosuav_ctrl_loop_input_t, mode_param) }, \
         { "flight_ctrl_status", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_rosuav_ctrl_loop_input_t, flight_ctrl_status) }, \
         { "ctrl_param", NULL, TTALINK_TYPE_UINT8_T, 0, 3, offsetof(ttalink_rosuav_ctrl_loop_input_t, ctrl_param) }, \
         { "unlock_state", NULL, TTALINK_TYPE_UINT8_T, 0, 4, offsetof(ttalink_rosuav_ctrl_loop_input_t, unlock_state) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 5, offsetof(ttalink_rosuav_ctrl_loop_input_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 13, offsetof(ttalink_rosuav_ctrl_loop_input_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 21, offsetof(ttalink_rosuav_ctrl_loop_input_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 25, offsetof(ttalink_rosuav_ctrl_loop_input_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 29, offsetof(ttalink_rosuav_ctrl_loop_input_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 33, offsetof(ttalink_rosuav_ctrl_loop_input_t, velD) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 37, offsetof(ttalink_rosuav_ctrl_loop_input_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 41, offsetof(ttalink_rosuav_ctrl_loop_input_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 45, offsetof(ttalink_rosuav_ctrl_loop_input_t, accD) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 49, offsetof(ttalink_rosuav_ctrl_loop_input_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 53, offsetof(ttalink_rosuav_ctrl_loop_input_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 57, offsetof(ttalink_rosuav_ctrl_loop_input_t, atti_yaw) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 61, offsetof(ttalink_rosuav_ctrl_loop_input_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 65, offsetof(ttalink_rosuav_ctrl_loop_input_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 69, offsetof(ttalink_rosuav_ctrl_loop_input_t, gyro_yaw) }, \
         { "thro_thrust", NULL, TTALINK_TYPE_FLOAT, 0, 73, offsetof(ttalink_rosuav_ctrl_loop_input_t, thro_thrust) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 77, offsetof(ttalink_rosuav_ctrl_loop_input_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_ctrl_loop_input_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, mode_param);
    _tta_put_uint8_t(buf, 2, flight_ctrl_status);
    _tta_put_uint8_t(buf, 3, ctrl_param);
    _tta_put_uint8_t(buf, 4, unlock_state);
    _tta_put_double(buf, 5, latit);
    _tta_put_double(buf, 13, longi);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, accN);
    _tta_put_float(buf, 41, accE);
    _tta_put_float(buf, 45, accD);
    _tta_put_float(buf, 49, atti_pitch);
    _tta_put_float(buf, 53, atti_roll);
    _tta_put_float(buf, 57, atti_yaw);
    _tta_put_float(buf, 61, gyro_pitch);
    _tta_put_float(buf, 65, gyro_roll);
    _tta_put_float(buf, 69, gyro_yaw);
    _tta_put_float(buf, 73, thro_thrust);
    _tta_put_float_array(buf, 77, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN);
#else
    ttalink_rosuav_ctrl_loop_input_t packet;
    packet.mode = mode;
    packet.mode_param = mode_param;
    packet.flight_ctrl_status = flight_ctrl_status;
    packet.ctrl_param = ctrl_param;
    packet.unlock_state = unlock_state;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.thro_thrust = thro_thrust;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_ctrl_loop_input message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  预留模式.
 * @param mode_param  模式参数.
 * @param flight_ctrl_status  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量
 * @param ctrl_param  
 * @param unlock_state  =0 上锁. 1=解锁
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度前馈.
 * @param accE  E向加速度前馈.
 * @param accD  D向加速度前馈.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  航向角速度.
 * @param thro_thrust  油门推力.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param)
{
    return _ttalink_rosuav_ctrl_loop_input_pack(dst_addr, src_addr, msg,  mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, false);
}

/**
 * @brief Pack a rosuav_ctrl_loop_input message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param mode  预留模式.
 * @param mode_param  模式参数.
 * @param flight_ctrl_status  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量
 * @param ctrl_param  
 * @param unlock_state  =0 上锁. 1=解锁
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度前馈.
 * @param accE  E向加速度前馈.
 * @param accD  D向加速度前馈.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  航向角速度.
 * @param thro_thrust  油门推力.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param)
{
    return _ttalink_rosuav_ctrl_loop_input_pack(dst_addr, src_addr, msg,  mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_loop_input_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t mode_param,uint8_t flight_ctrl_status,uint8_t ctrl_param,uint8_t unlock_state,double latit,double longi,float altit,float velN,float velE,float velD,float accN,float accE,float accD,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float thro_thrust,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, mode_param);
    _tta_put_uint8_t(buf, 2, flight_ctrl_status);
    _tta_put_uint8_t(buf, 3, ctrl_param);
    _tta_put_uint8_t(buf, 4, unlock_state);
    _tta_put_double(buf, 5, latit);
    _tta_put_double(buf, 13, longi);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, accN);
    _tta_put_float(buf, 41, accE);
    _tta_put_float(buf, 45, accD);
    _tta_put_float(buf, 49, atti_pitch);
    _tta_put_float(buf, 53, atti_roll);
    _tta_put_float(buf, 57, atti_yaw);
    _tta_put_float(buf, 61, gyro_pitch);
    _tta_put_float(buf, 65, gyro_roll);
    _tta_put_float(buf, 69, gyro_yaw);
    _tta_put_float(buf, 73, thro_thrust);
    _tta_put_float_array(buf, 77, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN);
#else
    ttalink_rosuav_ctrl_loop_input_t packet;
    packet.mode = mode;
    packet.mode_param = mode_param;
    packet.flight_ctrl_status = flight_ctrl_status;
    packet.ctrl_param = ctrl_param;
    packet.unlock_state = unlock_state;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.thro_thrust = thro_thrust;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_ctrl_loop_input message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  预留模式.
 * @param mode_param  模式参数.
 * @param flight_ctrl_status  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量
 * @param ctrl_param  
 * @param unlock_state  =0 上锁. 1=解锁
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度前馈.
 * @param accE  E向加速度前馈.
 * @param accD  D向加速度前馈.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  航向角速度.
 * @param thro_thrust  油门推力.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t mode_param,uint8_t flight_ctrl_status,uint8_t ctrl_param,uint8_t unlock_state,double latit,double longi,float altit,float velN,float velE,float velD,float accN,float accE,float accD,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float thro_thrust,const float *param)
{
    return _ttalink_rosuav_ctrl_loop_input_pack_chan(dst_addr, src_addr, chan, msg,  mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, false);
}

/**
 * @brief Pack a rosuav_ctrl_loop_input message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param mode  预留模式.
 * @param mode_param  模式参数.
 * @param flight_ctrl_status  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量
 * @param ctrl_param  
 * @param unlock_state  =0 上锁. 1=解锁
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度前馈.
 * @param accE  E向加速度前馈.
 * @param accD  D向加速度前馈.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  航向角速度.
 * @param thro_thrust  油门推力.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t mode,uint8_t mode_param,uint8_t flight_ctrl_status,uint8_t ctrl_param,uint8_t unlock_state,double latit,double longi,float altit,float velN,float velE,float velD,float accN,float accE,float accD,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float thro_thrust,const float *param)
{
    return _ttalink_rosuav_ctrl_loop_input_pack_chan(dst_addr, src_addr, chan, msg,  mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_loop_input_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_ctrl_loop_input_pack_nocrc(dst_addr, src_addr, msg, rosuav_ctrl_loop_input->mode, rosuav_ctrl_loop_input->mode_param, rosuav_ctrl_loop_input->flight_ctrl_status, rosuav_ctrl_loop_input->ctrl_param, rosuav_ctrl_loop_input->unlock_state, rosuav_ctrl_loop_input->latit, rosuav_ctrl_loop_input->longi, rosuav_ctrl_loop_input->altit, rosuav_ctrl_loop_input->velN, rosuav_ctrl_loop_input->velE, rosuav_ctrl_loop_input->velD, rosuav_ctrl_loop_input->accN, rosuav_ctrl_loop_input->accE, rosuav_ctrl_loop_input->accD, rosuav_ctrl_loop_input->atti_pitch, rosuav_ctrl_loop_input->atti_roll, rosuav_ctrl_loop_input->atti_yaw, rosuav_ctrl_loop_input->gyro_pitch, rosuav_ctrl_loop_input->gyro_roll, rosuav_ctrl_loop_input->gyro_yaw, rosuav_ctrl_loop_input->thro_thrust, rosuav_ctrl_loop_input->param);
    }else{
        return ttalink_rosuav_ctrl_loop_input_pack(dst_addr, src_addr, msg, rosuav_ctrl_loop_input->mode, rosuav_ctrl_loop_input->mode_param, rosuav_ctrl_loop_input->flight_ctrl_status, rosuav_ctrl_loop_input->ctrl_param, rosuav_ctrl_loop_input->unlock_state, rosuav_ctrl_loop_input->latit, rosuav_ctrl_loop_input->longi, rosuav_ctrl_loop_input->altit, rosuav_ctrl_loop_input->velN, rosuav_ctrl_loop_input->velE, rosuav_ctrl_loop_input->velD, rosuav_ctrl_loop_input->accN, rosuav_ctrl_loop_input->accE, rosuav_ctrl_loop_input->accD, rosuav_ctrl_loop_input->atti_pitch, rosuav_ctrl_loop_input->atti_roll, rosuav_ctrl_loop_input->atti_yaw, rosuav_ctrl_loop_input->gyro_pitch, rosuav_ctrl_loop_input->gyro_roll, rosuav_ctrl_loop_input->gyro_yaw, rosuav_ctrl_loop_input->thro_thrust, rosuav_ctrl_loop_input->param);
    }
    
}

/**
 * @brief Encode a rosuav_ctrl_loop_input struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_loop_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input)
{
    return _ttalink_rosuav_ctrl_loop_input_encode(dst_addr, src_addr, msg, rosuav_ctrl_loop_input, false);
}

/**
 * @brief Encode a rosuav_ctrl_loop_input struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_loop_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input)
{
    return _ttalink_rosuav_ctrl_loop_input_encode(dst_addr, src_addr, msg, rosuav_ctrl_loop_input, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_loop_input_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_ctrl_loop_input_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_ctrl_loop_input->mode, rosuav_ctrl_loop_input->mode_param, rosuav_ctrl_loop_input->flight_ctrl_status, rosuav_ctrl_loop_input->ctrl_param, rosuav_ctrl_loop_input->unlock_state, rosuav_ctrl_loop_input->latit, rosuav_ctrl_loop_input->longi, rosuav_ctrl_loop_input->altit, rosuav_ctrl_loop_input->velN, rosuav_ctrl_loop_input->velE, rosuav_ctrl_loop_input->velD, rosuav_ctrl_loop_input->accN, rosuav_ctrl_loop_input->accE, rosuav_ctrl_loop_input->accD, rosuav_ctrl_loop_input->atti_pitch, rosuav_ctrl_loop_input->atti_roll, rosuav_ctrl_loop_input->atti_yaw, rosuav_ctrl_loop_input->gyro_pitch, rosuav_ctrl_loop_input->gyro_roll, rosuav_ctrl_loop_input->gyro_yaw, rosuav_ctrl_loop_input->thro_thrust, rosuav_ctrl_loop_input->param);
    }else{
        return ttalink_rosuav_ctrl_loop_input_pack_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_loop_input->mode, rosuav_ctrl_loop_input->mode_param, rosuav_ctrl_loop_input->flight_ctrl_status, rosuav_ctrl_loop_input->ctrl_param, rosuav_ctrl_loop_input->unlock_state, rosuav_ctrl_loop_input->latit, rosuav_ctrl_loop_input->longi, rosuav_ctrl_loop_input->altit, rosuav_ctrl_loop_input->velN, rosuav_ctrl_loop_input->velE, rosuav_ctrl_loop_input->velD, rosuav_ctrl_loop_input->accN, rosuav_ctrl_loop_input->accE, rosuav_ctrl_loop_input->accD, rosuav_ctrl_loop_input->atti_pitch, rosuav_ctrl_loop_input->atti_roll, rosuav_ctrl_loop_input->atti_yaw, rosuav_ctrl_loop_input->gyro_pitch, rosuav_ctrl_loop_input->gyro_roll, rosuav_ctrl_loop_input->gyro_yaw, rosuav_ctrl_loop_input->thro_thrust, rosuav_ctrl_loop_input->param);
    }
}

/**
 * @brief Encode a rosuav_ctrl_loop_input struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_loop_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input)
{
    return _ttalink_rosuav_ctrl_loop_input_encode_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_loop_input, false);
}

/**
 * @brief Encode a rosuav_ctrl_loop_input struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_loop_input C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input)
{
    return _ttalink_rosuav_ctrl_loop_input_encode_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_loop_input, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_ctrl_loop_input_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN];
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, mode_param);
    _tta_put_uint8_t(buf, 2, flight_ctrl_status);
    _tta_put_uint8_t(buf, 3, ctrl_param);
    _tta_put_uint8_t(buf, 4, unlock_state);
    _tta_put_double(buf, 5, latit);
    _tta_put_double(buf, 13, longi);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, accN);
    _tta_put_float(buf, 41, accE);
    _tta_put_float(buf, 45, accD);
    _tta_put_float(buf, 49, atti_pitch);
    _tta_put_float(buf, 53, atti_roll);
    _tta_put_float(buf, 57, atti_yaw);
    _tta_put_float(buf, 61, gyro_pitch);
    _tta_put_float(buf, 65, gyro_roll);
    _tta_put_float(buf, 69, gyro_yaw);
    _tta_put_float(buf, 73, thro_thrust);
    _tta_put_float_array(buf, 77, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT, buf, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC, nocrc);
#else
    ttalink_rosuav_ctrl_loop_input_t packet;
    packet.mode = mode;
    packet.mode_param = mode_param;
    packet.flight_ctrl_status = flight_ctrl_status;
    packet.ctrl_param = ctrl_param;
    packet.unlock_state = unlock_state;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.thro_thrust = thro_thrust;
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_ctrl_loop_input message
 * @param chan TTAlink channel to send the message
 *
 * @param mode  预留模式.
 * @param mode_param  模式参数.
 * @param flight_ctrl_status  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量
 * @param ctrl_param  
 * @param unlock_state  =0 上锁. 1=解锁
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度前馈.
 * @param accE  E向加速度前馈.
 * @param accD  D向加速度前馈.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  航向角速度.
 * @param thro_thrust  油门推力.
 * @param param  .
 */
static inline void ttalink_rosuav_ctrl_loop_input_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param)
{
    _ttalink_rosuav_ctrl_loop_input_send(dst_addr, src_addr, chan, mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, false);
}

/**
 * @brief Send a rosuav_ctrl_loop_input message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param mode  预留模式.
 * @param mode_param  模式参数.
 * @param flight_ctrl_status  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量
 * @param ctrl_param  
 * @param unlock_state  =0 上锁. 1=解锁
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度前馈.
 * @param accE  E向加速度前馈.
 * @param accD  D向加速度前馈.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  航向角速度.
 * @param thro_thrust  油门推力.
 * @param param  .
 */
static inline void ttalink_rosuav_ctrl_loop_input_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param)
{
    _ttalink_rosuav_ctrl_loop_input_send(dst_addr, src_addr, chan, mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, true);
}


static inline void _ttalink_rosuav_ctrl_loop_input_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_ctrl_loop_input_send_nocrc(dst_addr, src_addr, chan, rosuav_ctrl_loop_input->mode, rosuav_ctrl_loop_input->mode_param, rosuav_ctrl_loop_input->flight_ctrl_status, rosuav_ctrl_loop_input->ctrl_param, rosuav_ctrl_loop_input->unlock_state, rosuav_ctrl_loop_input->latit, rosuav_ctrl_loop_input->longi, rosuav_ctrl_loop_input->altit, rosuav_ctrl_loop_input->velN, rosuav_ctrl_loop_input->velE, rosuav_ctrl_loop_input->velD, rosuav_ctrl_loop_input->accN, rosuav_ctrl_loop_input->accE, rosuav_ctrl_loop_input->accD, rosuav_ctrl_loop_input->atti_pitch, rosuav_ctrl_loop_input->atti_roll, rosuav_ctrl_loop_input->atti_yaw, rosuav_ctrl_loop_input->gyro_pitch, rosuav_ctrl_loop_input->gyro_roll, rosuav_ctrl_loop_input->gyro_yaw, rosuav_ctrl_loop_input->thro_thrust, rosuav_ctrl_loop_input->param);
    }else{
        ttalink_rosuav_ctrl_loop_input_send(dst_addr, src_addr, chan, rosuav_ctrl_loop_input->mode, rosuav_ctrl_loop_input->mode_param, rosuav_ctrl_loop_input->flight_ctrl_status, rosuav_ctrl_loop_input->ctrl_param, rosuav_ctrl_loop_input->unlock_state, rosuav_ctrl_loop_input->latit, rosuav_ctrl_loop_input->longi, rosuav_ctrl_loop_input->altit, rosuav_ctrl_loop_input->velN, rosuav_ctrl_loop_input->velE, rosuav_ctrl_loop_input->velD, rosuav_ctrl_loop_input->accN, rosuav_ctrl_loop_input->accE, rosuav_ctrl_loop_input->accD, rosuav_ctrl_loop_input->atti_pitch, rosuav_ctrl_loop_input->atti_roll, rosuav_ctrl_loop_input->atti_yaw, rosuav_ctrl_loop_input->gyro_pitch, rosuav_ctrl_loop_input->gyro_roll, rosuav_ctrl_loop_input->gyro_yaw, rosuav_ctrl_loop_input->thro_thrust, rosuav_ctrl_loop_input->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT, (const char *)rosuav_ctrl_loop_input, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_ctrl_loop_input message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_ctrl_loop_input_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input)
{
    _ttalink_rosuav_ctrl_loop_input_send_struct(dst_addr, src_addr, chan, rosuav_ctrl_loop_input, false);
}

/**
 * @brief Send a rosuav_ctrl_loop_input message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_ctrl_loop_input_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input)
{
    _ttalink_rosuav_ctrl_loop_input_send_struct(dst_addr, src_addr, chan, rosuav_ctrl_loop_input, true);
}

#if TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_ctrl_loop_input_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, mode);
    _tta_put_uint8_t(buf, 1, mode_param);
    _tta_put_uint8_t(buf, 2, flight_ctrl_status);
    _tta_put_uint8_t(buf, 3, ctrl_param);
    _tta_put_uint8_t(buf, 4, unlock_state);
    _tta_put_double(buf, 5, latit);
    _tta_put_double(buf, 13, longi);
    _tta_put_float(buf, 21, altit);
    _tta_put_float(buf, 25, velN);
    _tta_put_float(buf, 29, velE);
    _tta_put_float(buf, 33, velD);
    _tta_put_float(buf, 37, accN);
    _tta_put_float(buf, 41, accE);
    _tta_put_float(buf, 45, accD);
    _tta_put_float(buf, 49, atti_pitch);
    _tta_put_float(buf, 53, atti_roll);
    _tta_put_float(buf, 57, atti_yaw);
    _tta_put_float(buf, 61, gyro_pitch);
    _tta_put_float(buf, 65, gyro_roll);
    _tta_put_float(buf, 69, gyro_yaw);
    _tta_put_float(buf, 73, thro_thrust);
    _tta_put_float_array(buf, 77, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT, buf, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC, nocrc);
#else
    ttalink_rosuav_ctrl_loop_input_t *packet = (ttalink_rosuav_ctrl_loop_input_t *)msgbuf;
    packet->mode = mode;
    packet->mode_param = mode_param;
    packet->flight_ctrl_status = flight_ctrl_status;
    packet->ctrl_param = ctrl_param;
    packet->unlock_state = unlock_state;
    packet->latit = latit;
    packet->longi = longi;
    packet->altit = altit;
    packet->velN = velN;
    packet->velE = velE;
    packet->velD = velD;
    packet->accN = accN;
    packet->accE = accE;
    packet->accD = accD;
    packet->atti_pitch = atti_pitch;
    packet->atti_roll = atti_roll;
    packet->atti_yaw = atti_yaw;
    packet->gyro_pitch = gyro_pitch;
    packet->gyro_roll = gyro_roll;
    packet->gyro_yaw = gyro_yaw;
    packet->thro_thrust = thro_thrust;
    tta_array_memcpy(packet->param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT, (const char *)packet, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_ctrl_loop_input_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param)
{
    _ttalink_rosuav_ctrl_loop_input_send_buf(dst_addr, src_addr, msgbuf, chan, mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_ctrl_loop_input_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t mode, uint8_t mode_param, uint8_t flight_ctrl_status, uint8_t ctrl_param, uint8_t unlock_state, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float thro_thrust, const float *param)
{
    _ttalink_rosuav_ctrl_loop_input_send_buf(dst_addr, src_addr, msgbuf, chan, mode, mode_param, flight_ctrl_status, ctrl_param, unlock_state, latit, longi, altit, velN, velE, velD, accN, accE, accD, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, thro_thrust, param, true);
}
#endif

#endif

// MESSAGE ROSUAV_CTRL_LOOP_INPUT UNPACKING


/**
 * @brief Get field mode from rosuav_ctrl_loop_input message
 *
 * @return  预留模式.
 */
static inline uint8_t ttalink_rosuav_ctrl_loop_input_get_mode(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mode_param from rosuav_ctrl_loop_input message
 *
 * @return  模式参数.
 */
static inline uint8_t ttalink_rosuav_ctrl_loop_input_get_mode_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field flight_ctrl_status from rosuav_ctrl_loop_input message
 *
 * @return  控制细节模块，根据模块 参数可能意义不同，顶层控制为目标，下层为前馈量，例如速度控制，当前速度为目标量，姿态或加速度为前馈量
 */
static inline uint8_t ttalink_rosuav_ctrl_loop_input_get_flight_ctrl_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ctrl_param from rosuav_ctrl_loop_input message
 *
 * @return  
 */
static inline uint8_t ttalink_rosuav_ctrl_loop_input_get_ctrl_param(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field unlock_state from rosuav_ctrl_loop_input message
 *
 * @return  =0 上锁. 1=解锁
 */
static inline uint8_t ttalink_rosuav_ctrl_loop_input_get_unlock_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field latit from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline double ttalink_rosuav_ctrl_loop_input_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  5);
}

/**
 * @brief Get field longi from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline double ttalink_rosuav_ctrl_loop_input_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  13);
}

/**
 * @brief Get field altit from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  21);
}

/**
 * @brief Get field velN from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  25);
}

/**
 * @brief Get field velE from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  29);
}

/**
 * @brief Get field velD from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_velD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  33);
}

/**
 * @brief Get field accN from rosuav_ctrl_loop_input message
 *
 * @return  N向加速度前馈.
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_accN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  37);
}

/**
 * @brief Get field accE from rosuav_ctrl_loop_input message
 *
 * @return  E向加速度前馈.
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_accE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  41);
}

/**
 * @brief Get field accD from rosuav_ctrl_loop_input message
 *
 * @return  D向加速度前馈.
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_accD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  45);
}

/**
 * @brief Get field atti_pitch from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_atti_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  49);
}

/**
 * @brief Get field atti_roll from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_atti_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  53);
}

/**
 * @brief Get field atti_yaw from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_atti_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  57);
}

/**
 * @brief Get field gyro_pitch from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_gyro_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  61);
}

/**
 * @brief Get field gyro_roll from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_gyro_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  65);
}

/**
 * @brief Get field gyro_yaw from rosuav_ctrl_loop_input message
 *
 * @return  航向角速度.
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_gyro_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  69);
}

/**
 * @brief Get field thro_thrust from rosuav_ctrl_loop_input message
 *
 * @return  油门推力.
 */
static inline float ttalink_rosuav_ctrl_loop_input_get_thro_thrust(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  73);
}

/**
 * @brief Get field param from rosuav_ctrl_loop_input message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_ctrl_loop_input_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 4,  77);
}

/**
 * @brief Decode a rosuav_ctrl_loop_input message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_ctrl_loop_input C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_ctrl_loop_input_decode(const ttalink_message_t* msg, ttalink_rosuav_ctrl_loop_input_t* rosuav_ctrl_loop_input)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rosuav_ctrl_loop_input->mode = ttalink_rosuav_ctrl_loop_input_get_mode(msg);
    rosuav_ctrl_loop_input->mode_param = ttalink_rosuav_ctrl_loop_input_get_mode_param(msg);
    rosuav_ctrl_loop_input->flight_ctrl_status = ttalink_rosuav_ctrl_loop_input_get_flight_ctrl_status(msg);
    rosuav_ctrl_loop_input->ctrl_param = ttalink_rosuav_ctrl_loop_input_get_ctrl_param(msg);
    rosuav_ctrl_loop_input->unlock_state = ttalink_rosuav_ctrl_loop_input_get_unlock_state(msg);
    rosuav_ctrl_loop_input->latit = ttalink_rosuav_ctrl_loop_input_get_latit(msg);
    rosuav_ctrl_loop_input->longi = ttalink_rosuav_ctrl_loop_input_get_longi(msg);
    rosuav_ctrl_loop_input->altit = ttalink_rosuav_ctrl_loop_input_get_altit(msg);
    rosuav_ctrl_loop_input->velN = ttalink_rosuav_ctrl_loop_input_get_velN(msg);
    rosuav_ctrl_loop_input->velE = ttalink_rosuav_ctrl_loop_input_get_velE(msg);
    rosuav_ctrl_loop_input->velD = ttalink_rosuav_ctrl_loop_input_get_velD(msg);
    rosuav_ctrl_loop_input->accN = ttalink_rosuav_ctrl_loop_input_get_accN(msg);
    rosuav_ctrl_loop_input->accE = ttalink_rosuav_ctrl_loop_input_get_accE(msg);
    rosuav_ctrl_loop_input->accD = ttalink_rosuav_ctrl_loop_input_get_accD(msg);
    rosuav_ctrl_loop_input->atti_pitch = ttalink_rosuav_ctrl_loop_input_get_atti_pitch(msg);
    rosuav_ctrl_loop_input->atti_roll = ttalink_rosuav_ctrl_loop_input_get_atti_roll(msg);
    rosuav_ctrl_loop_input->atti_yaw = ttalink_rosuav_ctrl_loop_input_get_atti_yaw(msg);
    rosuav_ctrl_loop_input->gyro_pitch = ttalink_rosuav_ctrl_loop_input_get_gyro_pitch(msg);
    rosuav_ctrl_loop_input->gyro_roll = ttalink_rosuav_ctrl_loop_input_get_gyro_roll(msg);
    rosuav_ctrl_loop_input->gyro_yaw = ttalink_rosuav_ctrl_loop_input_get_gyro_yaw(msg);
    rosuav_ctrl_loop_input->thro_thrust = ttalink_rosuav_ctrl_loop_input_get_thro_thrust(msg);
    ttalink_rosuav_ctrl_loop_input_get_param(msg, rosuav_ctrl_loop_input->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN;
        memset(rosuav_ctrl_loop_input, 0, TTALINK_MSG_ID_ROSUAV_CTRL_LOOP_INPUT_LEN);
    memcpy(rosuav_ctrl_loop_input, _TTA_PAYLOAD(msg), len);
#endif
}
