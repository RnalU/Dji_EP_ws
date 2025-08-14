#pragma once
// MESSAGE ROSUAV_CTRL_TARGET_DATA PACKING

#define TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA 8202

TTAPACKED(
typedef struct __ttalink_rosuav_ctrl_target_data_t {
 uint8_t motor_flag; /*<   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达*/
 uint8_t flight_flag; /*<  飞行标志=0 未飞行. 1=已飞行*/
 double latit; /*<  .*/
 double longi; /*<  .*/
 float altit; /*<  .*/
 float velN; /*<  .*/
 float velE; /*<  .*/
 float velD; /*<  .*/
 float accN; /*<  N向加速度目标.*/
 float accE; /*<  E向加速度目标.*/
 float accD; /*<  D向加速度目标.*/
 float accX; /*<  X向加速度目标.*/
 float accY; /*<  Y向加速度目标.*/
 float accZ; /*<  Z向加速度目标.*/
 float atti_pitch; /*<  .*/
 float atti_roll; /*<  .*/
 float atti_yaw; /*<  .*/
 float gyro_pitch; /*<  .*/
 float gyro_roll; /*<  .*/
 float gyro_yaw; /*<  .*/
 float torque_x; /*<  X轴控制力矩（Roll） NM.*/
 float torque_y; /*<  Y轴控制力矩（Pitch） NM..*/
 float torque_z; /*<  Z轴控制力矩（Yaw） NM..*/
 float thrust; /*<  合推力输出（Thro） N.*/
 uint8_t matrix_step; /*<  马达分配矩阵当前状态。*/
 uint16_t motor_pwm[8]; /*<  马达输出值.*/
 float param[4]; /*<  .*/
}) ttalink_rosuav_ctrl_target_data_t;

#define TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN 131
#define TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN 131
#define TTALINK_MSG_ID_8202_LEN 131
#define TTALINK_MSG_ID_8202_MIN_LEN 131

#define TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC 240
#define TTALINK_MSG_ID_8202_CRC 240

#define TTALINK_MSG_ROSUAV_CTRL_TARGET_DATA_FIELD_MOTOR_PWM_LEN 8
#define TTALINK_MSG_ROSUAV_CTRL_TARGET_DATA_FIELD_PARAM_LEN 4

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_ROSUAV_CTRL_TARGET_DATA { \
    8202, \
    "ROSUAV_CTRL_TARGET_DATA", \
    27, \
    {  { "motor_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rosuav_ctrl_target_data_t, motor_flag) }, \
         { "flight_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rosuav_ctrl_target_data_t, flight_flag) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 2, offsetof(ttalink_rosuav_ctrl_target_data_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 10, offsetof(ttalink_rosuav_ctrl_target_data_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_rosuav_ctrl_target_data_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_rosuav_ctrl_target_data_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_rosuav_ctrl_target_data_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_rosuav_ctrl_target_data_t, velD) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_rosuav_ctrl_target_data_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 38, offsetof(ttalink_rosuav_ctrl_target_data_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 42, offsetof(ttalink_rosuav_ctrl_target_data_t, accD) }, \
         { "accX", NULL, TTALINK_TYPE_FLOAT, 0, 46, offsetof(ttalink_rosuav_ctrl_target_data_t, accX) }, \
         { "accY", NULL, TTALINK_TYPE_FLOAT, 0, 50, offsetof(ttalink_rosuav_ctrl_target_data_t, accY) }, \
         { "accZ", NULL, TTALINK_TYPE_FLOAT, 0, 54, offsetof(ttalink_rosuav_ctrl_target_data_t, accZ) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 58, offsetof(ttalink_rosuav_ctrl_target_data_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 62, offsetof(ttalink_rosuav_ctrl_target_data_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 66, offsetof(ttalink_rosuav_ctrl_target_data_t, atti_yaw) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 70, offsetof(ttalink_rosuav_ctrl_target_data_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 74, offsetof(ttalink_rosuav_ctrl_target_data_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 78, offsetof(ttalink_rosuav_ctrl_target_data_t, gyro_yaw) }, \
         { "torque_x", NULL, TTALINK_TYPE_FLOAT, 0, 82, offsetof(ttalink_rosuav_ctrl_target_data_t, torque_x) }, \
         { "torque_y", NULL, TTALINK_TYPE_FLOAT, 0, 86, offsetof(ttalink_rosuav_ctrl_target_data_t, torque_y) }, \
         { "torque_z", NULL, TTALINK_TYPE_FLOAT, 0, 90, offsetof(ttalink_rosuav_ctrl_target_data_t, torque_z) }, \
         { "thrust", NULL, TTALINK_TYPE_FLOAT, 0, 94, offsetof(ttalink_rosuav_ctrl_target_data_t, thrust) }, \
         { "matrix_step", NULL, TTALINK_TYPE_UINT8_T, 0, 98, offsetof(ttalink_rosuav_ctrl_target_data_t, matrix_step) }, \
         { "motor_pwm", NULL, TTALINK_TYPE_UINT16_T, 8, 99, offsetof(ttalink_rosuav_ctrl_target_data_t, motor_pwm) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 115, offsetof(ttalink_rosuav_ctrl_target_data_t, param) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_ROSUAV_CTRL_TARGET_DATA { \
    "ROSUAV_CTRL_TARGET_DATA", \
    27, \
    {  { "motor_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_rosuav_ctrl_target_data_t, motor_flag) }, \
         { "flight_flag", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_rosuav_ctrl_target_data_t, flight_flag) }, \
         { "latit", NULL, TTALINK_TYPE_DOUBLE, 0, 2, offsetof(ttalink_rosuav_ctrl_target_data_t, latit) }, \
         { "longi", NULL, TTALINK_TYPE_DOUBLE, 0, 10, offsetof(ttalink_rosuav_ctrl_target_data_t, longi) }, \
         { "altit", NULL, TTALINK_TYPE_FLOAT, 0, 18, offsetof(ttalink_rosuav_ctrl_target_data_t, altit) }, \
         { "velN", NULL, TTALINK_TYPE_FLOAT, 0, 22, offsetof(ttalink_rosuav_ctrl_target_data_t, velN) }, \
         { "velE", NULL, TTALINK_TYPE_FLOAT, 0, 26, offsetof(ttalink_rosuav_ctrl_target_data_t, velE) }, \
         { "velD", NULL, TTALINK_TYPE_FLOAT, 0, 30, offsetof(ttalink_rosuav_ctrl_target_data_t, velD) }, \
         { "accN", NULL, TTALINK_TYPE_FLOAT, 0, 34, offsetof(ttalink_rosuav_ctrl_target_data_t, accN) }, \
         { "accE", NULL, TTALINK_TYPE_FLOAT, 0, 38, offsetof(ttalink_rosuav_ctrl_target_data_t, accE) }, \
         { "accD", NULL, TTALINK_TYPE_FLOAT, 0, 42, offsetof(ttalink_rosuav_ctrl_target_data_t, accD) }, \
         { "accX", NULL, TTALINK_TYPE_FLOAT, 0, 46, offsetof(ttalink_rosuav_ctrl_target_data_t, accX) }, \
         { "accY", NULL, TTALINK_TYPE_FLOAT, 0, 50, offsetof(ttalink_rosuav_ctrl_target_data_t, accY) }, \
         { "accZ", NULL, TTALINK_TYPE_FLOAT, 0, 54, offsetof(ttalink_rosuav_ctrl_target_data_t, accZ) }, \
         { "atti_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 58, offsetof(ttalink_rosuav_ctrl_target_data_t, atti_pitch) }, \
         { "atti_roll", NULL, TTALINK_TYPE_FLOAT, 0, 62, offsetof(ttalink_rosuav_ctrl_target_data_t, atti_roll) }, \
         { "atti_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 66, offsetof(ttalink_rosuav_ctrl_target_data_t, atti_yaw) }, \
         { "gyro_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 70, offsetof(ttalink_rosuav_ctrl_target_data_t, gyro_pitch) }, \
         { "gyro_roll", NULL, TTALINK_TYPE_FLOAT, 0, 74, offsetof(ttalink_rosuav_ctrl_target_data_t, gyro_roll) }, \
         { "gyro_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 78, offsetof(ttalink_rosuav_ctrl_target_data_t, gyro_yaw) }, \
         { "torque_x", NULL, TTALINK_TYPE_FLOAT, 0, 82, offsetof(ttalink_rosuav_ctrl_target_data_t, torque_x) }, \
         { "torque_y", NULL, TTALINK_TYPE_FLOAT, 0, 86, offsetof(ttalink_rosuav_ctrl_target_data_t, torque_y) }, \
         { "torque_z", NULL, TTALINK_TYPE_FLOAT, 0, 90, offsetof(ttalink_rosuav_ctrl_target_data_t, torque_z) }, \
         { "thrust", NULL, TTALINK_TYPE_FLOAT, 0, 94, offsetof(ttalink_rosuav_ctrl_target_data_t, thrust) }, \
         { "matrix_step", NULL, TTALINK_TYPE_UINT8_T, 0, 98, offsetof(ttalink_rosuav_ctrl_target_data_t, matrix_step) }, \
         { "motor_pwm", NULL, TTALINK_TYPE_UINT16_T, 8, 99, offsetof(ttalink_rosuav_ctrl_target_data_t, motor_pwm) }, \
         { "param", NULL, TTALINK_TYPE_FLOAT, 4, 115, offsetof(ttalink_rosuav_ctrl_target_data_t, param) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rosuav_ctrl_target_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN];
    _tta_put_uint8_t(buf, 0, motor_flag);
    _tta_put_uint8_t(buf, 1, flight_flag);
    _tta_put_double(buf, 2, latit);
    _tta_put_double(buf, 10, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, velN);
    _tta_put_float(buf, 26, velE);
    _tta_put_float(buf, 30, velD);
    _tta_put_float(buf, 34, accN);
    _tta_put_float(buf, 38, accE);
    _tta_put_float(buf, 42, accD);
    _tta_put_float(buf, 46, accX);
    _tta_put_float(buf, 50, accY);
    _tta_put_float(buf, 54, accZ);
    _tta_put_float(buf, 58, atti_pitch);
    _tta_put_float(buf, 62, atti_roll);
    _tta_put_float(buf, 66, atti_yaw);
    _tta_put_float(buf, 70, gyro_pitch);
    _tta_put_float(buf, 74, gyro_roll);
    _tta_put_float(buf, 78, gyro_yaw);
    _tta_put_float(buf, 82, torque_x);
    _tta_put_float(buf, 86, torque_y);
    _tta_put_float(buf, 90, torque_z);
    _tta_put_float(buf, 94, thrust);
    _tta_put_uint8_t(buf, 98, matrix_step);
    _tta_put_uint16_t_array(buf, 99, motor_pwm, 8);
    _tta_put_float_array(buf, 115, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN);
#else
    ttalink_rosuav_ctrl_target_data_t packet;
    packet.motor_flag = motor_flag;
    packet.flight_flag = flight_flag;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.accX = accX;
    packet.accY = accY;
    packet.accZ = accZ;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.torque_x = torque_x;
    packet.torque_y = torque_y;
    packet.torque_z = torque_z;
    packet.thrust = thrust;
    packet.matrix_step = matrix_step;
    tta_array_memcpy(packet.motor_pwm, motor_pwm, sizeof(uint16_t)*8);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_ctrl_target_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param motor_flag   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达
 * @param flight_flag  飞行标志=0 未飞行. 1=已飞行
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度目标.
 * @param accE  E向加速度目标.
 * @param accD  D向加速度目标.
 * @param accX  X向加速度目标.
 * @param accY  Y向加速度目标.
 * @param accZ  Z向加速度目标.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param torque_x  X轴控制力矩（Roll） NM.
 * @param torque_y  Y轴控制力矩（Pitch） NM..
 * @param torque_z  Z轴控制力矩（Yaw） NM..
 * @param thrust  合推力输出（Thro） N.
 * @param matrix_step  马达分配矩阵当前状态。
 * @param motor_pwm  马达输出值.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param)
{
    return _ttalink_rosuav_ctrl_target_data_pack(dst_addr, src_addr, msg,  motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, false);
}

/**
 * @brief Pack a rosuav_ctrl_target_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param motor_flag   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达
 * @param flight_flag  飞行标志=0 未飞行. 1=已飞行
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度目标.
 * @param accE  E向加速度目标.
 * @param accD  D向加速度目标.
 * @param accX  X向加速度目标.
 * @param accY  Y向加速度目标.
 * @param accZ  Z向加速度目标.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param torque_x  X轴控制力矩（Roll） NM.
 * @param torque_y  Y轴控制力矩（Pitch） NM..
 * @param torque_z  Z轴控制力矩（Yaw） NM..
 * @param thrust  合推力输出（Thro） N.
 * @param matrix_step  马达分配矩阵当前状态。
 * @param motor_pwm  马达输出值.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param)
{
    return _ttalink_rosuav_ctrl_target_data_pack(dst_addr, src_addr, msg,  motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_target_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t motor_flag,uint8_t flight_flag,double latit,double longi,float altit,float velN,float velE,float velD,float accN,float accE,float accD,float accX,float accY,float accZ,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float torque_x,float torque_y,float torque_z,float thrust,uint8_t matrix_step,const uint16_t *motor_pwm,const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN];
    _tta_put_uint8_t(buf, 0, motor_flag);
    _tta_put_uint8_t(buf, 1, flight_flag);
    _tta_put_double(buf, 2, latit);
    _tta_put_double(buf, 10, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, velN);
    _tta_put_float(buf, 26, velE);
    _tta_put_float(buf, 30, velD);
    _tta_put_float(buf, 34, accN);
    _tta_put_float(buf, 38, accE);
    _tta_put_float(buf, 42, accD);
    _tta_put_float(buf, 46, accX);
    _tta_put_float(buf, 50, accY);
    _tta_put_float(buf, 54, accZ);
    _tta_put_float(buf, 58, atti_pitch);
    _tta_put_float(buf, 62, atti_roll);
    _tta_put_float(buf, 66, atti_yaw);
    _tta_put_float(buf, 70, gyro_pitch);
    _tta_put_float(buf, 74, gyro_roll);
    _tta_put_float(buf, 78, gyro_yaw);
    _tta_put_float(buf, 82, torque_x);
    _tta_put_float(buf, 86, torque_y);
    _tta_put_float(buf, 90, torque_z);
    _tta_put_float(buf, 94, thrust);
    _tta_put_uint8_t(buf, 98, matrix_step);
    _tta_put_uint16_t_array(buf, 99, motor_pwm, 8);
    _tta_put_float_array(buf, 115, param, 4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN);
#else
    ttalink_rosuav_ctrl_target_data_t packet;
    packet.motor_flag = motor_flag;
    packet.flight_flag = flight_flag;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.accX = accX;
    packet.accY = accY;
    packet.accZ = accZ;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.torque_x = torque_x;
    packet.torque_y = torque_y;
    packet.torque_z = torque_z;
    packet.thrust = thrust;
    packet.matrix_step = matrix_step;
    tta_array_memcpy(packet.motor_pwm, motor_pwm, sizeof(uint16_t)*8);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rosuav_ctrl_target_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param motor_flag   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达
 * @param flight_flag  飞行标志=0 未飞行. 1=已飞行
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度目标.
 * @param accE  E向加速度目标.
 * @param accD  D向加速度目标.
 * @param accX  X向加速度目标.
 * @param accY  Y向加速度目标.
 * @param accZ  Z向加速度目标.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param torque_x  X轴控制力矩（Roll） NM.
 * @param torque_y  Y轴控制力矩（Pitch） NM..
 * @param torque_z  Z轴控制力矩（Yaw） NM..
 * @param thrust  合推力输出（Thro） N.
 * @param matrix_step  马达分配矩阵当前状态。
 * @param motor_pwm  马达输出值.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t motor_flag,uint8_t flight_flag,double latit,double longi,float altit,float velN,float velE,float velD,float accN,float accE,float accD,float accX,float accY,float accZ,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float torque_x,float torque_y,float torque_z,float thrust,uint8_t matrix_step,const uint16_t *motor_pwm,const float *param)
{
    return _ttalink_rosuav_ctrl_target_data_pack_chan(dst_addr, src_addr, chan, msg,  motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, false);
}

/**
 * @brief Pack a rosuav_ctrl_target_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param motor_flag   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达
 * @param flight_flag  飞行标志=0 未飞行. 1=已飞行
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度目标.
 * @param accE  E向加速度目标.
 * @param accD  D向加速度目标.
 * @param accX  X向加速度目标.
 * @param accY  Y向加速度目标.
 * @param accZ  Z向加速度目标.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param torque_x  X轴控制力矩（Roll） NM.
 * @param torque_y  Y轴控制力矩（Pitch） NM..
 * @param torque_z  Z轴控制力矩（Yaw） NM..
 * @param thrust  合推力输出（Thro） N.
 * @param matrix_step  马达分配矩阵当前状态。
 * @param motor_pwm  马达输出值.
 * @param param  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t motor_flag,uint8_t flight_flag,double latit,double longi,float altit,float velN,float velE,float velD,float accN,float accE,float accD,float accX,float accY,float accZ,float atti_pitch,float atti_roll,float atti_yaw,float gyro_pitch,float gyro_roll,float gyro_yaw,float torque_x,float torque_y,float torque_z,float thrust,uint8_t matrix_step,const uint16_t *motor_pwm,const float *param)
{
    return _ttalink_rosuav_ctrl_target_data_pack_chan(dst_addr, src_addr, chan, msg,  motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_target_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_ctrl_target_data_pack_nocrc(dst_addr, src_addr, msg, rosuav_ctrl_target_data->motor_flag, rosuav_ctrl_target_data->flight_flag, rosuav_ctrl_target_data->latit, rosuav_ctrl_target_data->longi, rosuav_ctrl_target_data->altit, rosuav_ctrl_target_data->velN, rosuav_ctrl_target_data->velE, rosuav_ctrl_target_data->velD, rosuav_ctrl_target_data->accN, rosuav_ctrl_target_data->accE, rosuav_ctrl_target_data->accD, rosuav_ctrl_target_data->accX, rosuav_ctrl_target_data->accY, rosuav_ctrl_target_data->accZ, rosuav_ctrl_target_data->atti_pitch, rosuav_ctrl_target_data->atti_roll, rosuav_ctrl_target_data->atti_yaw, rosuav_ctrl_target_data->gyro_pitch, rosuav_ctrl_target_data->gyro_roll, rosuav_ctrl_target_data->gyro_yaw, rosuav_ctrl_target_data->torque_x, rosuav_ctrl_target_data->torque_y, rosuav_ctrl_target_data->torque_z, rosuav_ctrl_target_data->thrust, rosuav_ctrl_target_data->matrix_step, rosuav_ctrl_target_data->motor_pwm, rosuav_ctrl_target_data->param);
    }else{
        return ttalink_rosuav_ctrl_target_data_pack(dst_addr, src_addr, msg, rosuav_ctrl_target_data->motor_flag, rosuav_ctrl_target_data->flight_flag, rosuav_ctrl_target_data->latit, rosuav_ctrl_target_data->longi, rosuav_ctrl_target_data->altit, rosuav_ctrl_target_data->velN, rosuav_ctrl_target_data->velE, rosuav_ctrl_target_data->velD, rosuav_ctrl_target_data->accN, rosuav_ctrl_target_data->accE, rosuav_ctrl_target_data->accD, rosuav_ctrl_target_data->accX, rosuav_ctrl_target_data->accY, rosuav_ctrl_target_data->accZ, rosuav_ctrl_target_data->atti_pitch, rosuav_ctrl_target_data->atti_roll, rosuav_ctrl_target_data->atti_yaw, rosuav_ctrl_target_data->gyro_pitch, rosuav_ctrl_target_data->gyro_roll, rosuav_ctrl_target_data->gyro_yaw, rosuav_ctrl_target_data->torque_x, rosuav_ctrl_target_data->torque_y, rosuav_ctrl_target_data->torque_z, rosuav_ctrl_target_data->thrust, rosuav_ctrl_target_data->matrix_step, rosuav_ctrl_target_data->motor_pwm, rosuav_ctrl_target_data->param);
    }
    
}

/**
 * @brief Encode a rosuav_ctrl_target_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_target_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data)
{
    return _ttalink_rosuav_ctrl_target_data_encode(dst_addr, src_addr, msg, rosuav_ctrl_target_data, false);
}

/**
 * @brief Encode a rosuav_ctrl_target_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_target_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data)
{
    return _ttalink_rosuav_ctrl_target_data_encode(dst_addr, src_addr, msg, rosuav_ctrl_target_data, true);
}


static inline uint16_t _ttalink_rosuav_ctrl_target_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rosuav_ctrl_target_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rosuav_ctrl_target_data->motor_flag, rosuav_ctrl_target_data->flight_flag, rosuav_ctrl_target_data->latit, rosuav_ctrl_target_data->longi, rosuav_ctrl_target_data->altit, rosuav_ctrl_target_data->velN, rosuav_ctrl_target_data->velE, rosuav_ctrl_target_data->velD, rosuav_ctrl_target_data->accN, rosuav_ctrl_target_data->accE, rosuav_ctrl_target_data->accD, rosuav_ctrl_target_data->accX, rosuav_ctrl_target_data->accY, rosuav_ctrl_target_data->accZ, rosuav_ctrl_target_data->atti_pitch, rosuav_ctrl_target_data->atti_roll, rosuav_ctrl_target_data->atti_yaw, rosuav_ctrl_target_data->gyro_pitch, rosuav_ctrl_target_data->gyro_roll, rosuav_ctrl_target_data->gyro_yaw, rosuav_ctrl_target_data->torque_x, rosuav_ctrl_target_data->torque_y, rosuav_ctrl_target_data->torque_z, rosuav_ctrl_target_data->thrust, rosuav_ctrl_target_data->matrix_step, rosuav_ctrl_target_data->motor_pwm, rosuav_ctrl_target_data->param);
    }else{
        return ttalink_rosuav_ctrl_target_data_pack_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_target_data->motor_flag, rosuav_ctrl_target_data->flight_flag, rosuav_ctrl_target_data->latit, rosuav_ctrl_target_data->longi, rosuav_ctrl_target_data->altit, rosuav_ctrl_target_data->velN, rosuav_ctrl_target_data->velE, rosuav_ctrl_target_data->velD, rosuav_ctrl_target_data->accN, rosuav_ctrl_target_data->accE, rosuav_ctrl_target_data->accD, rosuav_ctrl_target_data->accX, rosuav_ctrl_target_data->accY, rosuav_ctrl_target_data->accZ, rosuav_ctrl_target_data->atti_pitch, rosuav_ctrl_target_data->atti_roll, rosuav_ctrl_target_data->atti_yaw, rosuav_ctrl_target_data->gyro_pitch, rosuav_ctrl_target_data->gyro_roll, rosuav_ctrl_target_data->gyro_yaw, rosuav_ctrl_target_data->torque_x, rosuav_ctrl_target_data->torque_y, rosuav_ctrl_target_data->torque_z, rosuav_ctrl_target_data->thrust, rosuav_ctrl_target_data->matrix_step, rosuav_ctrl_target_data->motor_pwm, rosuav_ctrl_target_data->param);
    }
}

/**
 * @brief Encode a rosuav_ctrl_target_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_target_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data)
{
    return _ttalink_rosuav_ctrl_target_data_encode_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_target_data, false);
}

/**
 * @brief Encode a rosuav_ctrl_target_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rosuav_ctrl_target_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data)
{
    return _ttalink_rosuav_ctrl_target_data_encode_chan(dst_addr, src_addr, chan, msg, rosuav_ctrl_target_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rosuav_ctrl_target_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN];
    _tta_put_uint8_t(buf, 0, motor_flag);
    _tta_put_uint8_t(buf, 1, flight_flag);
    _tta_put_double(buf, 2, latit);
    _tta_put_double(buf, 10, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, velN);
    _tta_put_float(buf, 26, velE);
    _tta_put_float(buf, 30, velD);
    _tta_put_float(buf, 34, accN);
    _tta_put_float(buf, 38, accE);
    _tta_put_float(buf, 42, accD);
    _tta_put_float(buf, 46, accX);
    _tta_put_float(buf, 50, accY);
    _tta_put_float(buf, 54, accZ);
    _tta_put_float(buf, 58, atti_pitch);
    _tta_put_float(buf, 62, atti_roll);
    _tta_put_float(buf, 66, atti_yaw);
    _tta_put_float(buf, 70, gyro_pitch);
    _tta_put_float(buf, 74, gyro_roll);
    _tta_put_float(buf, 78, gyro_yaw);
    _tta_put_float(buf, 82, torque_x);
    _tta_put_float(buf, 86, torque_y);
    _tta_put_float(buf, 90, torque_z);
    _tta_put_float(buf, 94, thrust);
    _tta_put_uint8_t(buf, 98, matrix_step);
    _tta_put_uint16_t_array(buf, 99, motor_pwm, 8);
    _tta_put_float_array(buf, 115, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA, buf, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC, nocrc);
#else
    ttalink_rosuav_ctrl_target_data_t packet;
    packet.motor_flag = motor_flag;
    packet.flight_flag = flight_flag;
    packet.latit = latit;
    packet.longi = longi;
    packet.altit = altit;
    packet.velN = velN;
    packet.velE = velE;
    packet.velD = velD;
    packet.accN = accN;
    packet.accE = accE;
    packet.accD = accD;
    packet.accX = accX;
    packet.accY = accY;
    packet.accZ = accZ;
    packet.atti_pitch = atti_pitch;
    packet.atti_roll = atti_roll;
    packet.atti_yaw = atti_yaw;
    packet.gyro_pitch = gyro_pitch;
    packet.gyro_roll = gyro_roll;
    packet.gyro_yaw = gyro_yaw;
    packet.torque_x = torque_x;
    packet.torque_y = torque_y;
    packet.torque_z = torque_z;
    packet.thrust = thrust;
    packet.matrix_step = matrix_step;
    tta_array_memcpy(packet.motor_pwm, motor_pwm, sizeof(uint16_t)*8);
    tta_array_memcpy(packet.param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA, (const char *)&packet, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_ctrl_target_data message
 * @param chan TTAlink channel to send the message
 *
 * @param motor_flag   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达
 * @param flight_flag  飞行标志=0 未飞行. 1=已飞行
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度目标.
 * @param accE  E向加速度目标.
 * @param accD  D向加速度目标.
 * @param accX  X向加速度目标.
 * @param accY  Y向加速度目标.
 * @param accZ  Z向加速度目标.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param torque_x  X轴控制力矩（Roll） NM.
 * @param torque_y  Y轴控制力矩（Pitch） NM..
 * @param torque_z  Z轴控制力矩（Yaw） NM..
 * @param thrust  合推力输出（Thro） N.
 * @param matrix_step  马达分配矩阵当前状态。
 * @param motor_pwm  马达输出值.
 * @param param  .
 */
static inline void ttalink_rosuav_ctrl_target_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param)
{
    _ttalink_rosuav_ctrl_target_data_send(dst_addr, src_addr, chan, motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, false);
}

/**
 * @brief Send a rosuav_ctrl_target_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param motor_flag   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达
 * @param flight_flag  飞行标志=0 未飞行. 1=已飞行
 * @param latit  .
 * @param longi  .
 * @param altit  .
 * @param velN  .
 * @param velE  .
 * @param velD  .
 * @param accN  N向加速度目标.
 * @param accE  E向加速度目标.
 * @param accD  D向加速度目标.
 * @param accX  X向加速度目标.
 * @param accY  Y向加速度目标.
 * @param accZ  Z向加速度目标.
 * @param atti_pitch  .
 * @param atti_roll  .
 * @param atti_yaw  .
 * @param gyro_pitch  .
 * @param gyro_roll  .
 * @param gyro_yaw  .
 * @param torque_x  X轴控制力矩（Roll） NM.
 * @param torque_y  Y轴控制力矩（Pitch） NM..
 * @param torque_z  Z轴控制力矩（Yaw） NM..
 * @param thrust  合推力输出（Thro） N.
 * @param matrix_step  马达分配矩阵当前状态。
 * @param motor_pwm  马达输出值.
 * @param param  .
 */
static inline void ttalink_rosuav_ctrl_target_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param)
{
    _ttalink_rosuav_ctrl_target_data_send(dst_addr, src_addr, chan, motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, true);
}


static inline void _ttalink_rosuav_ctrl_target_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rosuav_ctrl_target_data_send_nocrc(dst_addr, src_addr, chan, rosuav_ctrl_target_data->motor_flag, rosuav_ctrl_target_data->flight_flag, rosuav_ctrl_target_data->latit, rosuav_ctrl_target_data->longi, rosuav_ctrl_target_data->altit, rosuav_ctrl_target_data->velN, rosuav_ctrl_target_data->velE, rosuav_ctrl_target_data->velD, rosuav_ctrl_target_data->accN, rosuav_ctrl_target_data->accE, rosuav_ctrl_target_data->accD, rosuav_ctrl_target_data->accX, rosuav_ctrl_target_data->accY, rosuav_ctrl_target_data->accZ, rosuav_ctrl_target_data->atti_pitch, rosuav_ctrl_target_data->atti_roll, rosuav_ctrl_target_data->atti_yaw, rosuav_ctrl_target_data->gyro_pitch, rosuav_ctrl_target_data->gyro_roll, rosuav_ctrl_target_data->gyro_yaw, rosuav_ctrl_target_data->torque_x, rosuav_ctrl_target_data->torque_y, rosuav_ctrl_target_data->torque_z, rosuav_ctrl_target_data->thrust, rosuav_ctrl_target_data->matrix_step, rosuav_ctrl_target_data->motor_pwm, rosuav_ctrl_target_data->param);
    }else{
        ttalink_rosuav_ctrl_target_data_send(dst_addr, src_addr, chan, rosuav_ctrl_target_data->motor_flag, rosuav_ctrl_target_data->flight_flag, rosuav_ctrl_target_data->latit, rosuav_ctrl_target_data->longi, rosuav_ctrl_target_data->altit, rosuav_ctrl_target_data->velN, rosuav_ctrl_target_data->velE, rosuav_ctrl_target_data->velD, rosuav_ctrl_target_data->accN, rosuav_ctrl_target_data->accE, rosuav_ctrl_target_data->accD, rosuav_ctrl_target_data->accX, rosuav_ctrl_target_data->accY, rosuav_ctrl_target_data->accZ, rosuav_ctrl_target_data->atti_pitch, rosuav_ctrl_target_data->atti_roll, rosuav_ctrl_target_data->atti_yaw, rosuav_ctrl_target_data->gyro_pitch, rosuav_ctrl_target_data->gyro_roll, rosuav_ctrl_target_data->gyro_yaw, rosuav_ctrl_target_data->torque_x, rosuav_ctrl_target_data->torque_y, rosuav_ctrl_target_data->torque_z, rosuav_ctrl_target_data->thrust, rosuav_ctrl_target_data->matrix_step, rosuav_ctrl_target_data->motor_pwm, rosuav_ctrl_target_data->param);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA, (const char *)rosuav_ctrl_target_data, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rosuav_ctrl_target_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_ctrl_target_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data)
{
    _ttalink_rosuav_ctrl_target_data_send_struct(dst_addr, src_addr, chan, rosuav_ctrl_target_data, false);
}

/**
 * @brief Send a rosuav_ctrl_target_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rosuav_ctrl_target_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data)
{
    _ttalink_rosuav_ctrl_target_data_send_struct(dst_addr, src_addr, chan, rosuav_ctrl_target_data, true);
}

#if TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rosuav_ctrl_target_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, motor_flag);
    _tta_put_uint8_t(buf, 1, flight_flag);
    _tta_put_double(buf, 2, latit);
    _tta_put_double(buf, 10, longi);
    _tta_put_float(buf, 18, altit);
    _tta_put_float(buf, 22, velN);
    _tta_put_float(buf, 26, velE);
    _tta_put_float(buf, 30, velD);
    _tta_put_float(buf, 34, accN);
    _tta_put_float(buf, 38, accE);
    _tta_put_float(buf, 42, accD);
    _tta_put_float(buf, 46, accX);
    _tta_put_float(buf, 50, accY);
    _tta_put_float(buf, 54, accZ);
    _tta_put_float(buf, 58, atti_pitch);
    _tta_put_float(buf, 62, atti_roll);
    _tta_put_float(buf, 66, atti_yaw);
    _tta_put_float(buf, 70, gyro_pitch);
    _tta_put_float(buf, 74, gyro_roll);
    _tta_put_float(buf, 78, gyro_yaw);
    _tta_put_float(buf, 82, torque_x);
    _tta_put_float(buf, 86, torque_y);
    _tta_put_float(buf, 90, torque_z);
    _tta_put_float(buf, 94, thrust);
    _tta_put_uint8_t(buf, 98, matrix_step);
    _tta_put_uint16_t_array(buf, 99, motor_pwm, 8);
    _tta_put_float_array(buf, 115, param, 4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA, buf, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC, nocrc);
#else
    ttalink_rosuav_ctrl_target_data_t *packet = (ttalink_rosuav_ctrl_target_data_t *)msgbuf;
    packet->motor_flag = motor_flag;
    packet->flight_flag = flight_flag;
    packet->latit = latit;
    packet->longi = longi;
    packet->altit = altit;
    packet->velN = velN;
    packet->velE = velE;
    packet->velD = velD;
    packet->accN = accN;
    packet->accE = accE;
    packet->accD = accD;
    packet->accX = accX;
    packet->accY = accY;
    packet->accZ = accZ;
    packet->atti_pitch = atti_pitch;
    packet->atti_roll = atti_roll;
    packet->atti_yaw = atti_yaw;
    packet->gyro_pitch = gyro_pitch;
    packet->gyro_roll = gyro_roll;
    packet->gyro_yaw = gyro_yaw;
    packet->torque_x = torque_x;
    packet->torque_y = torque_y;
    packet->torque_z = torque_z;
    packet->thrust = thrust;
    packet->matrix_step = matrix_step;
    tta_array_memcpy(packet->motor_pwm, motor_pwm, sizeof(uint16_t)*8);
    tta_array_memcpy(packet->param, param, sizeof(float)*4);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA, (const char *)packet, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_MIN_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rosuav_ctrl_target_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param)
{
    _ttalink_rosuav_ctrl_target_data_send_buf(dst_addr, src_addr, msgbuf, chan, motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rosuav_ctrl_target_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t motor_flag, uint8_t flight_flag, double latit, double longi, float altit, float velN, float velE, float velD, float accN, float accE, float accD, float accX, float accY, float accZ, float atti_pitch, float atti_roll, float atti_yaw, float gyro_pitch, float gyro_roll, float gyro_yaw, float torque_x, float torque_y, float torque_z, float thrust, uint8_t matrix_step, const uint16_t *motor_pwm, const float *param)
{
    _ttalink_rosuav_ctrl_target_data_send_buf(dst_addr, src_addr, msgbuf, chan, motor_flag, flight_flag, latit, longi, altit, velN, velE, velD, accN, accE, accD, accX, accY, accZ, atti_pitch, atti_roll, atti_yaw, gyro_pitch, gyro_roll, gyro_yaw, torque_x, torque_y, torque_z, thrust, matrix_step, motor_pwm, param, true);
}
#endif

#endif

// MESSAGE ROSUAV_CTRL_TARGET_DATA UNPACKING


/**
 * @brief Get field motor_flag from rosuav_ctrl_target_data message
 *
 * @return   马达标志=0 未转动. 1=正常转动 2=加速中 3=减速中 4=需要停止马达
 */
static inline uint8_t ttalink_rosuav_ctrl_target_data_get_motor_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field flight_flag from rosuav_ctrl_target_data message
 *
 * @return  飞行标志=0 未飞行. 1=已飞行
 */
static inline uint8_t ttalink_rosuav_ctrl_target_data_get_flight_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field latit from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline double ttalink_rosuav_ctrl_target_data_get_latit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  2);
}

/**
 * @brief Get field longi from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline double ttalink_rosuav_ctrl_target_data_get_longi(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  10);
}

/**
 * @brief Get field altit from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_altit(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  18);
}

/**
 * @brief Get field velN from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  22);
}

/**
 * @brief Get field velE from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  26);
}

/**
 * @brief Get field velD from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_velD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  30);
}

/**
 * @brief Get field accN from rosuav_ctrl_target_data message
 *
 * @return  N向加速度目标.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_accN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  34);
}

/**
 * @brief Get field accE from rosuav_ctrl_target_data message
 *
 * @return  E向加速度目标.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_accE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  38);
}

/**
 * @brief Get field accD from rosuav_ctrl_target_data message
 *
 * @return  D向加速度目标.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_accD(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  42);
}

/**
 * @brief Get field accX from rosuav_ctrl_target_data message
 *
 * @return  X向加速度目标.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_accX(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  46);
}

/**
 * @brief Get field accY from rosuav_ctrl_target_data message
 *
 * @return  Y向加速度目标.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_accY(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  50);
}

/**
 * @brief Get field accZ from rosuav_ctrl_target_data message
 *
 * @return  Z向加速度目标.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_accZ(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  54);
}

/**
 * @brief Get field atti_pitch from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_atti_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  58);
}

/**
 * @brief Get field atti_roll from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_atti_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  62);
}

/**
 * @brief Get field atti_yaw from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_atti_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  66);
}

/**
 * @brief Get field gyro_pitch from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_gyro_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  70);
}

/**
 * @brief Get field gyro_roll from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_gyro_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  74);
}

/**
 * @brief Get field gyro_yaw from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline float ttalink_rosuav_ctrl_target_data_get_gyro_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  78);
}

/**
 * @brief Get field torque_x from rosuav_ctrl_target_data message
 *
 * @return  X轴控制力矩（Roll） NM.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_torque_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  82);
}

/**
 * @brief Get field torque_y from rosuav_ctrl_target_data message
 *
 * @return  Y轴控制力矩（Pitch） NM..
 */
static inline float ttalink_rosuav_ctrl_target_data_get_torque_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  86);
}

/**
 * @brief Get field torque_z from rosuav_ctrl_target_data message
 *
 * @return  Z轴控制力矩（Yaw） NM..
 */
static inline float ttalink_rosuav_ctrl_target_data_get_torque_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  90);
}

/**
 * @brief Get field thrust from rosuav_ctrl_target_data message
 *
 * @return  合推力输出（Thro） N.
 */
static inline float ttalink_rosuav_ctrl_target_data_get_thrust(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  94);
}

/**
 * @brief Get field matrix_step from rosuav_ctrl_target_data message
 *
 * @return  马达分配矩阵当前状态。
 */
static inline uint8_t ttalink_rosuav_ctrl_target_data_get_matrix_step(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  98);
}

/**
 * @brief Get field motor_pwm from rosuav_ctrl_target_data message
 *
 * @return  马达输出值.
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_get_motor_pwm(const ttalink_message_t* msg, uint16_t *motor_pwm)
{
    return _TTA_RETURN_uint16_t_array(msg, motor_pwm, 8,  99);
}

/**
 * @brief Get field param from rosuav_ctrl_target_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_rosuav_ctrl_target_data_get_param(const ttalink_message_t* msg, float *param)
{
    return _TTA_RETURN_float_array(msg, param, 4,  115);
}

/**
 * @brief Decode a rosuav_ctrl_target_data message into a struct
 *
 * @param msg The message to decode
 * @param rosuav_ctrl_target_data C-struct to decode the message contents into
 */
static inline void ttalink_rosuav_ctrl_target_data_decode(const ttalink_message_t* msg, ttalink_rosuav_ctrl_target_data_t* rosuav_ctrl_target_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rosuav_ctrl_target_data->motor_flag = ttalink_rosuav_ctrl_target_data_get_motor_flag(msg);
    rosuav_ctrl_target_data->flight_flag = ttalink_rosuav_ctrl_target_data_get_flight_flag(msg);
    rosuav_ctrl_target_data->latit = ttalink_rosuav_ctrl_target_data_get_latit(msg);
    rosuav_ctrl_target_data->longi = ttalink_rosuav_ctrl_target_data_get_longi(msg);
    rosuav_ctrl_target_data->altit = ttalink_rosuav_ctrl_target_data_get_altit(msg);
    rosuav_ctrl_target_data->velN = ttalink_rosuav_ctrl_target_data_get_velN(msg);
    rosuav_ctrl_target_data->velE = ttalink_rosuav_ctrl_target_data_get_velE(msg);
    rosuav_ctrl_target_data->velD = ttalink_rosuav_ctrl_target_data_get_velD(msg);
    rosuav_ctrl_target_data->accN = ttalink_rosuav_ctrl_target_data_get_accN(msg);
    rosuav_ctrl_target_data->accE = ttalink_rosuav_ctrl_target_data_get_accE(msg);
    rosuav_ctrl_target_data->accD = ttalink_rosuav_ctrl_target_data_get_accD(msg);
    rosuav_ctrl_target_data->accX = ttalink_rosuav_ctrl_target_data_get_accX(msg);
    rosuav_ctrl_target_data->accY = ttalink_rosuav_ctrl_target_data_get_accY(msg);
    rosuav_ctrl_target_data->accZ = ttalink_rosuav_ctrl_target_data_get_accZ(msg);
    rosuav_ctrl_target_data->atti_pitch = ttalink_rosuav_ctrl_target_data_get_atti_pitch(msg);
    rosuav_ctrl_target_data->atti_roll = ttalink_rosuav_ctrl_target_data_get_atti_roll(msg);
    rosuav_ctrl_target_data->atti_yaw = ttalink_rosuav_ctrl_target_data_get_atti_yaw(msg);
    rosuav_ctrl_target_data->gyro_pitch = ttalink_rosuav_ctrl_target_data_get_gyro_pitch(msg);
    rosuav_ctrl_target_data->gyro_roll = ttalink_rosuav_ctrl_target_data_get_gyro_roll(msg);
    rosuav_ctrl_target_data->gyro_yaw = ttalink_rosuav_ctrl_target_data_get_gyro_yaw(msg);
    rosuav_ctrl_target_data->torque_x = ttalink_rosuav_ctrl_target_data_get_torque_x(msg);
    rosuav_ctrl_target_data->torque_y = ttalink_rosuav_ctrl_target_data_get_torque_y(msg);
    rosuav_ctrl_target_data->torque_z = ttalink_rosuav_ctrl_target_data_get_torque_z(msg);
    rosuav_ctrl_target_data->thrust = ttalink_rosuav_ctrl_target_data_get_thrust(msg);
    rosuav_ctrl_target_data->matrix_step = ttalink_rosuav_ctrl_target_data_get_matrix_step(msg);
    ttalink_rosuav_ctrl_target_data_get_motor_pwm(msg, rosuav_ctrl_target_data->motor_pwm);
    ttalink_rosuav_ctrl_target_data_get_param(msg, rosuav_ctrl_target_data->param);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN? msg->len : TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN;
        memset(rosuav_ctrl_target_data, 0, TTALINK_MSG_ID_ROSUAV_CTRL_TARGET_DATA_LEN);
    memcpy(rosuav_ctrl_target_data, _TTA_PAYLOAD(msg), len);
#endif
}
