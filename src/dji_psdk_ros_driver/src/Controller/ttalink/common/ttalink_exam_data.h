#pragma once
// MESSAGE EXAM_DATA PACKING

#define TTALINK_MSG_ID_EXAM_DATA 194

TTAPACKED(
typedef struct __ttalink_exam_data_t {
 uint64_t time; /*<  utc_time(MS)*/
 uint64_t uav_id; /*<  uav_id.*/
 float target_pos_x; /*<  Relative home point */
 float target_pos_y; /*< Relative home point */
 float target_pos_z; /*< Relative home point */
 float current_pos_x; /*<  Relative home point*/
 float current_pos_y; /*< Relative home point */
 float current_pos_z; /*< Relative home point */
 float rf_filter_alt; /*< Height after filtering of radar */
 float GPS_RTK_pos_x; /*<  GPS-RTK Relative home point*/
 float GPS_RTK_pos_y; /*<  GPS-RTK Relative home point*/
 float GPS_RTK_pos_z; /*<  GPS-RTK Relative home point*/
 float baro_alt_meas; /*<  Barometer height*/
 float rf_alt_meas; /*<  Altimeter measurement */
 float fuse_hgt_meas; /*<  Fusion height measurement*/
 int16_t target_vel_x; /*<  target_vel*/
 int16_t target_vel_y; /*<  target_vel*/
 int16_t target_vel_z; /*<  target_vel*/
 int16_t current_vel_x; /*< current_vel */
 int16_t current_vel_y; /*< current_vel */
 int16_t current_vel_z; /*< current_vel */
 int16_t target_angle_x; /*<  target_angle*/
 int16_t target_angle_y; /*< target_angle */
 int16_t target_angle_z; /*<  target_angle*/
 int16_t current_angle_x; /*< current_angle */
 int16_t current_angle_y; /*< current_angle */
 int16_t current_angle_z; /*<  current_angle*/
 int16_t target_angle_rate_x; /*< target_angle_rate */
 int16_t target_angle_rate_y; /*<  target_angle_rate*/
 int16_t target_angle_rate_z; /*< target_angle_rate */
 int16_t current_angle_rate_x; /*< current_angle_rate */
 int16_t current_angle_rate_y; /*< current_angle_rate */
 int16_t current_angle_rate_z; /*< current_angle_rate */
 int16_t acc_roll_angle; /*< Sensor roll angle */
 int16_t acc_pitch_angle; /*<  Sensor pitch angle*/
 int16_t mag_yaw_angle; /*<  Magnetometer yaw angle*/
 int16_t rtk_yaw_angle; /*<  RTK yaw angle*/
 int16_t Reserve1; /*<  */
 int16_t Reserve2; /*<  */
 int16_t Reserve3; /*<  */
 int16_t Reserve4; /*<  */
 uint16_t flag; /*<  */
 uint8_t motor_chan1; /*< motor[0,100]%*/
 uint8_t motor_chan2; /*< motor[0,100]% */
 uint8_t motor_chan3; /*< motor[0,100]% */
 uint8_t motor_chan4; /*< motor[0,100]% */
 uint8_t motor_chan5; /*< motor[0,100]%*/
 uint8_t motor_chan6; /*< motor[0,100]%*/
 uint8_t motor_chan7; /*< motor[0,100]%*/
 uint8_t motor_chan8; /*< motor[0,100]%*/
 int8_t rc_chan1; /*< rc[0,100]%*/
 int8_t rc_chan2; /*< rc[0,100]%*/
 int8_t rc_chan3; /*< rc[0,100]%*/
 int8_t rc_chan4; /*< rc[0,100]%*/
 uint8_t rc_chan5_8; /*<  */
 uint8_t rc_chan9_12; /*<  */
 uint8_t rc_chan13_16; /*<  */
}) ttalink_exam_data_t;

#define TTALINK_MSG_ID_EXAM_DATA_LEN 137
#define TTALINK_MSG_ID_EXAM_DATA_MIN_LEN 137
#define TTALINK_MSG_ID_194_LEN 137
#define TTALINK_MSG_ID_194_MIN_LEN 137

#define TTALINK_MSG_ID_EXAM_DATA_CRC 37
#define TTALINK_MSG_ID_194_CRC 37



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_EXAM_DATA { \
    194, \
    "EXAM_DATA", \
    57, \
    {  { "time", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_exam_data_t, time) }, \
         { "uav_id", NULL, TTALINK_TYPE_UINT64_T, 0, 8, offsetof(ttalink_exam_data_t, uav_id) }, \
         { "target_pos_x", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_exam_data_t, target_pos_x) }, \
         { "target_pos_y", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_exam_data_t, target_pos_y) }, \
         { "target_pos_z", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_exam_data_t, target_pos_z) }, \
         { "current_pos_x", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_exam_data_t, current_pos_x) }, \
         { "current_pos_y", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_exam_data_t, current_pos_y) }, \
         { "current_pos_z", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_exam_data_t, current_pos_z) }, \
         { "rf_filter_alt", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_exam_data_t, rf_filter_alt) }, \
         { "GPS_RTK_pos_x", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_exam_data_t, GPS_RTK_pos_x) }, \
         { "GPS_RTK_pos_y", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_exam_data_t, GPS_RTK_pos_y) }, \
         { "GPS_RTK_pos_z", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_exam_data_t, GPS_RTK_pos_z) }, \
         { "baro_alt_meas", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_exam_data_t, baro_alt_meas) }, \
         { "rf_alt_meas", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_exam_data_t, rf_alt_meas) }, \
         { "fuse_hgt_meas", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_exam_data_t, fuse_hgt_meas) }, \
         { "target_vel_x", NULL, TTALINK_TYPE_INT16_T, 0, 68, offsetof(ttalink_exam_data_t, target_vel_x) }, \
         { "target_vel_y", NULL, TTALINK_TYPE_INT16_T, 0, 70, offsetof(ttalink_exam_data_t, target_vel_y) }, \
         { "target_vel_z", NULL, TTALINK_TYPE_INT16_T, 0, 72, offsetof(ttalink_exam_data_t, target_vel_z) }, \
         { "current_vel_x", NULL, TTALINK_TYPE_INT16_T, 0, 74, offsetof(ttalink_exam_data_t, current_vel_x) }, \
         { "current_vel_y", NULL, TTALINK_TYPE_INT16_T, 0, 76, offsetof(ttalink_exam_data_t, current_vel_y) }, \
         { "current_vel_z", NULL, TTALINK_TYPE_INT16_T, 0, 78, offsetof(ttalink_exam_data_t, current_vel_z) }, \
         { "target_angle_x", NULL, TTALINK_TYPE_INT16_T, 0, 80, offsetof(ttalink_exam_data_t, target_angle_x) }, \
         { "target_angle_y", NULL, TTALINK_TYPE_INT16_T, 0, 82, offsetof(ttalink_exam_data_t, target_angle_y) }, \
         { "target_angle_z", NULL, TTALINK_TYPE_INT16_T, 0, 84, offsetof(ttalink_exam_data_t, target_angle_z) }, \
         { "current_angle_x", NULL, TTALINK_TYPE_INT16_T, 0, 86, offsetof(ttalink_exam_data_t, current_angle_x) }, \
         { "current_angle_y", NULL, TTALINK_TYPE_INT16_T, 0, 88, offsetof(ttalink_exam_data_t, current_angle_y) }, \
         { "current_angle_z", NULL, TTALINK_TYPE_INT16_T, 0, 90, offsetof(ttalink_exam_data_t, current_angle_z) }, \
         { "target_angle_rate_x", NULL, TTALINK_TYPE_INT16_T, 0, 92, offsetof(ttalink_exam_data_t, target_angle_rate_x) }, \
         { "target_angle_rate_y", NULL, TTALINK_TYPE_INT16_T, 0, 94, offsetof(ttalink_exam_data_t, target_angle_rate_y) }, \
         { "target_angle_rate_z", NULL, TTALINK_TYPE_INT16_T, 0, 96, offsetof(ttalink_exam_data_t, target_angle_rate_z) }, \
         { "current_angle_rate_x", NULL, TTALINK_TYPE_INT16_T, 0, 98, offsetof(ttalink_exam_data_t, current_angle_rate_x) }, \
         { "current_angle_rate_y", NULL, TTALINK_TYPE_INT16_T, 0, 100, offsetof(ttalink_exam_data_t, current_angle_rate_y) }, \
         { "current_angle_rate_z", NULL, TTALINK_TYPE_INT16_T, 0, 102, offsetof(ttalink_exam_data_t, current_angle_rate_z) }, \
         { "acc_roll_angle", NULL, TTALINK_TYPE_INT16_T, 0, 104, offsetof(ttalink_exam_data_t, acc_roll_angle) }, \
         { "acc_pitch_angle", NULL, TTALINK_TYPE_INT16_T, 0, 106, offsetof(ttalink_exam_data_t, acc_pitch_angle) }, \
         { "mag_yaw_angle", NULL, TTALINK_TYPE_INT16_T, 0, 108, offsetof(ttalink_exam_data_t, mag_yaw_angle) }, \
         { "rtk_yaw_angle", NULL, TTALINK_TYPE_INT16_T, 0, 110, offsetof(ttalink_exam_data_t, rtk_yaw_angle) }, \
         { "Reserve1", NULL, TTALINK_TYPE_INT16_T, 0, 112, offsetof(ttalink_exam_data_t, Reserve1) }, \
         { "Reserve2", NULL, TTALINK_TYPE_INT16_T, 0, 114, offsetof(ttalink_exam_data_t, Reserve2) }, \
         { "Reserve3", NULL, TTALINK_TYPE_INT16_T, 0, 116, offsetof(ttalink_exam_data_t, Reserve3) }, \
         { "Reserve4", NULL, TTALINK_TYPE_INT16_T, 0, 118, offsetof(ttalink_exam_data_t, Reserve4) }, \
         { "flag", NULL, TTALINK_TYPE_UINT16_T, 0, 120, offsetof(ttalink_exam_data_t, flag) }, \
         { "motor_chan1", NULL, TTALINK_TYPE_UINT8_T, 0, 122, offsetof(ttalink_exam_data_t, motor_chan1) }, \
         { "motor_chan2", NULL, TTALINK_TYPE_UINT8_T, 0, 123, offsetof(ttalink_exam_data_t, motor_chan2) }, \
         { "motor_chan3", NULL, TTALINK_TYPE_UINT8_T, 0, 124, offsetof(ttalink_exam_data_t, motor_chan3) }, \
         { "motor_chan4", NULL, TTALINK_TYPE_UINT8_T, 0, 125, offsetof(ttalink_exam_data_t, motor_chan4) }, \
         { "motor_chan5", NULL, TTALINK_TYPE_UINT8_T, 0, 126, offsetof(ttalink_exam_data_t, motor_chan5) }, \
         { "motor_chan6", NULL, TTALINK_TYPE_UINT8_T, 0, 127, offsetof(ttalink_exam_data_t, motor_chan6) }, \
         { "motor_chan7", NULL, TTALINK_TYPE_UINT8_T, 0, 128, offsetof(ttalink_exam_data_t, motor_chan7) }, \
         { "motor_chan8", NULL, TTALINK_TYPE_UINT8_T, 0, 129, offsetof(ttalink_exam_data_t, motor_chan8) }, \
         { "rc_chan1", NULL, TTALINK_TYPE_INT8_T, 0, 130, offsetof(ttalink_exam_data_t, rc_chan1) }, \
         { "rc_chan2", NULL, TTALINK_TYPE_INT8_T, 0, 131, offsetof(ttalink_exam_data_t, rc_chan2) }, \
         { "rc_chan3", NULL, TTALINK_TYPE_INT8_T, 0, 132, offsetof(ttalink_exam_data_t, rc_chan3) }, \
         { "rc_chan4", NULL, TTALINK_TYPE_INT8_T, 0, 133, offsetof(ttalink_exam_data_t, rc_chan4) }, \
         { "rc_chan5_8", NULL, TTALINK_TYPE_UINT8_T, 0, 134, offsetof(ttalink_exam_data_t, rc_chan5_8) }, \
         { "rc_chan9_12", NULL, TTALINK_TYPE_UINT8_T, 0, 135, offsetof(ttalink_exam_data_t, rc_chan9_12) }, \
         { "rc_chan13_16", NULL, TTALINK_TYPE_UINT8_T, 0, 136, offsetof(ttalink_exam_data_t, rc_chan13_16) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_EXAM_DATA { \
    "EXAM_DATA", \
    57, \
    {  { "time", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_exam_data_t, time) }, \
         { "uav_id", NULL, TTALINK_TYPE_UINT64_T, 0, 8, offsetof(ttalink_exam_data_t, uav_id) }, \
         { "target_pos_x", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_exam_data_t, target_pos_x) }, \
         { "target_pos_y", NULL, TTALINK_TYPE_FLOAT, 0, 20, offsetof(ttalink_exam_data_t, target_pos_y) }, \
         { "target_pos_z", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_exam_data_t, target_pos_z) }, \
         { "current_pos_x", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_exam_data_t, current_pos_x) }, \
         { "current_pos_y", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_exam_data_t, current_pos_y) }, \
         { "current_pos_z", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_exam_data_t, current_pos_z) }, \
         { "rf_filter_alt", NULL, TTALINK_TYPE_FLOAT, 0, 40, offsetof(ttalink_exam_data_t, rf_filter_alt) }, \
         { "GPS_RTK_pos_x", NULL, TTALINK_TYPE_FLOAT, 0, 44, offsetof(ttalink_exam_data_t, GPS_RTK_pos_x) }, \
         { "GPS_RTK_pos_y", NULL, TTALINK_TYPE_FLOAT, 0, 48, offsetof(ttalink_exam_data_t, GPS_RTK_pos_y) }, \
         { "GPS_RTK_pos_z", NULL, TTALINK_TYPE_FLOAT, 0, 52, offsetof(ttalink_exam_data_t, GPS_RTK_pos_z) }, \
         { "baro_alt_meas", NULL, TTALINK_TYPE_FLOAT, 0, 56, offsetof(ttalink_exam_data_t, baro_alt_meas) }, \
         { "rf_alt_meas", NULL, TTALINK_TYPE_FLOAT, 0, 60, offsetof(ttalink_exam_data_t, rf_alt_meas) }, \
         { "fuse_hgt_meas", NULL, TTALINK_TYPE_FLOAT, 0, 64, offsetof(ttalink_exam_data_t, fuse_hgt_meas) }, \
         { "target_vel_x", NULL, TTALINK_TYPE_INT16_T, 0, 68, offsetof(ttalink_exam_data_t, target_vel_x) }, \
         { "target_vel_y", NULL, TTALINK_TYPE_INT16_T, 0, 70, offsetof(ttalink_exam_data_t, target_vel_y) }, \
         { "target_vel_z", NULL, TTALINK_TYPE_INT16_T, 0, 72, offsetof(ttalink_exam_data_t, target_vel_z) }, \
         { "current_vel_x", NULL, TTALINK_TYPE_INT16_T, 0, 74, offsetof(ttalink_exam_data_t, current_vel_x) }, \
         { "current_vel_y", NULL, TTALINK_TYPE_INT16_T, 0, 76, offsetof(ttalink_exam_data_t, current_vel_y) }, \
         { "current_vel_z", NULL, TTALINK_TYPE_INT16_T, 0, 78, offsetof(ttalink_exam_data_t, current_vel_z) }, \
         { "target_angle_x", NULL, TTALINK_TYPE_INT16_T, 0, 80, offsetof(ttalink_exam_data_t, target_angle_x) }, \
         { "target_angle_y", NULL, TTALINK_TYPE_INT16_T, 0, 82, offsetof(ttalink_exam_data_t, target_angle_y) }, \
         { "target_angle_z", NULL, TTALINK_TYPE_INT16_T, 0, 84, offsetof(ttalink_exam_data_t, target_angle_z) }, \
         { "current_angle_x", NULL, TTALINK_TYPE_INT16_T, 0, 86, offsetof(ttalink_exam_data_t, current_angle_x) }, \
         { "current_angle_y", NULL, TTALINK_TYPE_INT16_T, 0, 88, offsetof(ttalink_exam_data_t, current_angle_y) }, \
         { "current_angle_z", NULL, TTALINK_TYPE_INT16_T, 0, 90, offsetof(ttalink_exam_data_t, current_angle_z) }, \
         { "target_angle_rate_x", NULL, TTALINK_TYPE_INT16_T, 0, 92, offsetof(ttalink_exam_data_t, target_angle_rate_x) }, \
         { "target_angle_rate_y", NULL, TTALINK_TYPE_INT16_T, 0, 94, offsetof(ttalink_exam_data_t, target_angle_rate_y) }, \
         { "target_angle_rate_z", NULL, TTALINK_TYPE_INT16_T, 0, 96, offsetof(ttalink_exam_data_t, target_angle_rate_z) }, \
         { "current_angle_rate_x", NULL, TTALINK_TYPE_INT16_T, 0, 98, offsetof(ttalink_exam_data_t, current_angle_rate_x) }, \
         { "current_angle_rate_y", NULL, TTALINK_TYPE_INT16_T, 0, 100, offsetof(ttalink_exam_data_t, current_angle_rate_y) }, \
         { "current_angle_rate_z", NULL, TTALINK_TYPE_INT16_T, 0, 102, offsetof(ttalink_exam_data_t, current_angle_rate_z) }, \
         { "acc_roll_angle", NULL, TTALINK_TYPE_INT16_T, 0, 104, offsetof(ttalink_exam_data_t, acc_roll_angle) }, \
         { "acc_pitch_angle", NULL, TTALINK_TYPE_INT16_T, 0, 106, offsetof(ttalink_exam_data_t, acc_pitch_angle) }, \
         { "mag_yaw_angle", NULL, TTALINK_TYPE_INT16_T, 0, 108, offsetof(ttalink_exam_data_t, mag_yaw_angle) }, \
         { "rtk_yaw_angle", NULL, TTALINK_TYPE_INT16_T, 0, 110, offsetof(ttalink_exam_data_t, rtk_yaw_angle) }, \
         { "Reserve1", NULL, TTALINK_TYPE_INT16_T, 0, 112, offsetof(ttalink_exam_data_t, Reserve1) }, \
         { "Reserve2", NULL, TTALINK_TYPE_INT16_T, 0, 114, offsetof(ttalink_exam_data_t, Reserve2) }, \
         { "Reserve3", NULL, TTALINK_TYPE_INT16_T, 0, 116, offsetof(ttalink_exam_data_t, Reserve3) }, \
         { "Reserve4", NULL, TTALINK_TYPE_INT16_T, 0, 118, offsetof(ttalink_exam_data_t, Reserve4) }, \
         { "flag", NULL, TTALINK_TYPE_UINT16_T, 0, 120, offsetof(ttalink_exam_data_t, flag) }, \
         { "motor_chan1", NULL, TTALINK_TYPE_UINT8_T, 0, 122, offsetof(ttalink_exam_data_t, motor_chan1) }, \
         { "motor_chan2", NULL, TTALINK_TYPE_UINT8_T, 0, 123, offsetof(ttalink_exam_data_t, motor_chan2) }, \
         { "motor_chan3", NULL, TTALINK_TYPE_UINT8_T, 0, 124, offsetof(ttalink_exam_data_t, motor_chan3) }, \
         { "motor_chan4", NULL, TTALINK_TYPE_UINT8_T, 0, 125, offsetof(ttalink_exam_data_t, motor_chan4) }, \
         { "motor_chan5", NULL, TTALINK_TYPE_UINT8_T, 0, 126, offsetof(ttalink_exam_data_t, motor_chan5) }, \
         { "motor_chan6", NULL, TTALINK_TYPE_UINT8_T, 0, 127, offsetof(ttalink_exam_data_t, motor_chan6) }, \
         { "motor_chan7", NULL, TTALINK_TYPE_UINT8_T, 0, 128, offsetof(ttalink_exam_data_t, motor_chan7) }, \
         { "motor_chan8", NULL, TTALINK_TYPE_UINT8_T, 0, 129, offsetof(ttalink_exam_data_t, motor_chan8) }, \
         { "rc_chan1", NULL, TTALINK_TYPE_INT8_T, 0, 130, offsetof(ttalink_exam_data_t, rc_chan1) }, \
         { "rc_chan2", NULL, TTALINK_TYPE_INT8_T, 0, 131, offsetof(ttalink_exam_data_t, rc_chan2) }, \
         { "rc_chan3", NULL, TTALINK_TYPE_INT8_T, 0, 132, offsetof(ttalink_exam_data_t, rc_chan3) }, \
         { "rc_chan4", NULL, TTALINK_TYPE_INT8_T, 0, 133, offsetof(ttalink_exam_data_t, rc_chan4) }, \
         { "rc_chan5_8", NULL, TTALINK_TYPE_UINT8_T, 0, 134, offsetof(ttalink_exam_data_t, rc_chan5_8) }, \
         { "rc_chan9_12", NULL, TTALINK_TYPE_UINT8_T, 0, 135, offsetof(ttalink_exam_data_t, rc_chan9_12) }, \
         { "rc_chan13_16", NULL, TTALINK_TYPE_UINT8_T, 0, 136, offsetof(ttalink_exam_data_t, rc_chan13_16) }, \
         } \
}
#endif


static inline uint16_t _ttalink_exam_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXAM_DATA_LEN];
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_float(buf, 16, target_pos_x);
    _tta_put_float(buf, 20, target_pos_y);
    _tta_put_float(buf, 24, target_pos_z);
    _tta_put_float(buf, 28, current_pos_x);
    _tta_put_float(buf, 32, current_pos_y);
    _tta_put_float(buf, 36, current_pos_z);
    _tta_put_float(buf, 40, rf_filter_alt);
    _tta_put_float(buf, 44, GPS_RTK_pos_x);
    _tta_put_float(buf, 48, GPS_RTK_pos_y);
    _tta_put_float(buf, 52, GPS_RTK_pos_z);
    _tta_put_float(buf, 56, baro_alt_meas);
    _tta_put_float(buf, 60, rf_alt_meas);
    _tta_put_float(buf, 64, fuse_hgt_meas);
    _tta_put_int16_t(buf, 68, target_vel_x);
    _tta_put_int16_t(buf, 70, target_vel_y);
    _tta_put_int16_t(buf, 72, target_vel_z);
    _tta_put_int16_t(buf, 74, current_vel_x);
    _tta_put_int16_t(buf, 76, current_vel_y);
    _tta_put_int16_t(buf, 78, current_vel_z);
    _tta_put_int16_t(buf, 80, target_angle_x);
    _tta_put_int16_t(buf, 82, target_angle_y);
    _tta_put_int16_t(buf, 84, target_angle_z);
    _tta_put_int16_t(buf, 86, current_angle_x);
    _tta_put_int16_t(buf, 88, current_angle_y);
    _tta_put_int16_t(buf, 90, current_angle_z);
    _tta_put_int16_t(buf, 92, target_angle_rate_x);
    _tta_put_int16_t(buf, 94, target_angle_rate_y);
    _tta_put_int16_t(buf, 96, target_angle_rate_z);
    _tta_put_int16_t(buf, 98, current_angle_rate_x);
    _tta_put_int16_t(buf, 100, current_angle_rate_y);
    _tta_put_int16_t(buf, 102, current_angle_rate_z);
    _tta_put_int16_t(buf, 104, acc_roll_angle);
    _tta_put_int16_t(buf, 106, acc_pitch_angle);
    _tta_put_int16_t(buf, 108, mag_yaw_angle);
    _tta_put_int16_t(buf, 110, rtk_yaw_angle);
    _tta_put_int16_t(buf, 112, Reserve1);
    _tta_put_int16_t(buf, 114, Reserve2);
    _tta_put_int16_t(buf, 116, Reserve3);
    _tta_put_int16_t(buf, 118, Reserve4);
    _tta_put_uint16_t(buf, 120, flag);
    _tta_put_uint8_t(buf, 122, motor_chan1);
    _tta_put_uint8_t(buf, 123, motor_chan2);
    _tta_put_uint8_t(buf, 124, motor_chan3);
    _tta_put_uint8_t(buf, 125, motor_chan4);
    _tta_put_uint8_t(buf, 126, motor_chan5);
    _tta_put_uint8_t(buf, 127, motor_chan6);
    _tta_put_uint8_t(buf, 128, motor_chan7);
    _tta_put_uint8_t(buf, 129, motor_chan8);
    _tta_put_int8_t(buf, 130, rc_chan1);
    _tta_put_int8_t(buf, 131, rc_chan2);
    _tta_put_int8_t(buf, 132, rc_chan3);
    _tta_put_int8_t(buf, 133, rc_chan4);
    _tta_put_uint8_t(buf, 134, rc_chan5_8);
    _tta_put_uint8_t(buf, 135, rc_chan9_12);
    _tta_put_uint8_t(buf, 136, rc_chan13_16);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EXAM_DATA_LEN);
#else
    ttalink_exam_data_t packet;
    packet.time = time;
    packet.uav_id = uav_id;
    packet.target_pos_x = target_pos_x;
    packet.target_pos_y = target_pos_y;
    packet.target_pos_z = target_pos_z;
    packet.current_pos_x = current_pos_x;
    packet.current_pos_y = current_pos_y;
    packet.current_pos_z = current_pos_z;
    packet.rf_filter_alt = rf_filter_alt;
    packet.GPS_RTK_pos_x = GPS_RTK_pos_x;
    packet.GPS_RTK_pos_y = GPS_RTK_pos_y;
    packet.GPS_RTK_pos_z = GPS_RTK_pos_z;
    packet.baro_alt_meas = baro_alt_meas;
    packet.rf_alt_meas = rf_alt_meas;
    packet.fuse_hgt_meas = fuse_hgt_meas;
    packet.target_vel_x = target_vel_x;
    packet.target_vel_y = target_vel_y;
    packet.target_vel_z = target_vel_z;
    packet.current_vel_x = current_vel_x;
    packet.current_vel_y = current_vel_y;
    packet.current_vel_z = current_vel_z;
    packet.target_angle_x = target_angle_x;
    packet.target_angle_y = target_angle_y;
    packet.target_angle_z = target_angle_z;
    packet.current_angle_x = current_angle_x;
    packet.current_angle_y = current_angle_y;
    packet.current_angle_z = current_angle_z;
    packet.target_angle_rate_x = target_angle_rate_x;
    packet.target_angle_rate_y = target_angle_rate_y;
    packet.target_angle_rate_z = target_angle_rate_z;
    packet.current_angle_rate_x = current_angle_rate_x;
    packet.current_angle_rate_y = current_angle_rate_y;
    packet.current_angle_rate_z = current_angle_rate_z;
    packet.acc_roll_angle = acc_roll_angle;
    packet.acc_pitch_angle = acc_pitch_angle;
    packet.mag_yaw_angle = mag_yaw_angle;
    packet.rtk_yaw_angle = rtk_yaw_angle;
    packet.Reserve1 = Reserve1;
    packet.Reserve2 = Reserve2;
    packet.Reserve3 = Reserve3;
    packet.Reserve4 = Reserve4;
    packet.flag = flag;
    packet.motor_chan1 = motor_chan1;
    packet.motor_chan2 = motor_chan2;
    packet.motor_chan3 = motor_chan3;
    packet.motor_chan4 = motor_chan4;
    packet.motor_chan5 = motor_chan5;
    packet.motor_chan6 = motor_chan6;
    packet.motor_chan7 = motor_chan7;
    packet.motor_chan8 = motor_chan8;
    packet.rc_chan1 = rc_chan1;
    packet.rc_chan2 = rc_chan2;
    packet.rc_chan3 = rc_chan3;
    packet.rc_chan4 = rc_chan4;
    packet.rc_chan5_8 = rc_chan5_8;
    packet.rc_chan9_12 = rc_chan9_12;
    packet.rc_chan13_16 = rc_chan13_16;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EXAM_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EXAM_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_EXAM_DATA_MIN_LEN, TTALINK_MSG_ID_EXAM_DATA_LEN, TTALINK_MSG_ID_EXAM_DATA_CRC, nocrc);
}

/**
 * @brief Pack a exam_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param target_pos_x  Relative home point 
 * @param target_pos_y Relative home point 
 * @param target_pos_z Relative home point 
 * @param current_pos_x  Relative home point
 * @param current_pos_y Relative home point 
 * @param current_pos_z Relative home point 
 * @param rf_filter_alt Height after filtering of radar 
 * @param GPS_RTK_pos_x  GPS-RTK Relative home point
 * @param GPS_RTK_pos_y  GPS-RTK Relative home point
 * @param GPS_RTK_pos_z  GPS-RTK Relative home point
 * @param baro_alt_meas  Barometer height
 * @param rf_alt_meas  Altimeter measurement 
 * @param fuse_hgt_meas  Fusion height measurement
 * @param target_vel_x  target_vel
 * @param target_vel_y  target_vel
 * @param target_vel_z  target_vel
 * @param current_vel_x current_vel 
 * @param current_vel_y current_vel 
 * @param current_vel_z current_vel 
 * @param target_angle_x  target_angle
 * @param target_angle_y target_angle 
 * @param target_angle_z  target_angle
 * @param current_angle_x current_angle 
 * @param current_angle_y current_angle 
 * @param current_angle_z  current_angle
 * @param target_angle_rate_x target_angle_rate 
 * @param target_angle_rate_y  target_angle_rate
 * @param target_angle_rate_z target_angle_rate 
 * @param current_angle_rate_x current_angle_rate 
 * @param current_angle_rate_y current_angle_rate 
 * @param current_angle_rate_z current_angle_rate 
 * @param acc_roll_angle Sensor roll angle 
 * @param acc_pitch_angle  Sensor pitch angle
 * @param mag_yaw_angle  Magnetometer yaw angle
 * @param rtk_yaw_angle  RTK yaw angle
 * @param Reserve1  
 * @param Reserve2  
 * @param Reserve3  
 * @param Reserve4  
 * @param flag  
 * @param motor_chan1 motor[0,100]%
 * @param motor_chan2 motor[0,100]% 
 * @param motor_chan3 motor[0,100]% 
 * @param motor_chan4 motor[0,100]% 
 * @param motor_chan5 motor[0,100]%
 * @param motor_chan6 motor[0,100]%
 * @param motor_chan7 motor[0,100]%
 * @param motor_chan8 motor[0,100]%
 * @param rc_chan1 rc[0,100]%
 * @param rc_chan2 rc[0,100]%
 * @param rc_chan3 rc[0,100]%
 * @param rc_chan4 rc[0,100]%
 * @param rc_chan5_8  
 * @param rc_chan9_12  
 * @param rc_chan13_16  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16)
{
    return _ttalink_exam_data_pack(dst_addr, src_addr, msg,  time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, false);
}

/**
 * @brief Pack a exam_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param target_pos_x  Relative home point 
 * @param target_pos_y Relative home point 
 * @param target_pos_z Relative home point 
 * @param current_pos_x  Relative home point
 * @param current_pos_y Relative home point 
 * @param current_pos_z Relative home point 
 * @param rf_filter_alt Height after filtering of radar 
 * @param GPS_RTK_pos_x  GPS-RTK Relative home point
 * @param GPS_RTK_pos_y  GPS-RTK Relative home point
 * @param GPS_RTK_pos_z  GPS-RTK Relative home point
 * @param baro_alt_meas  Barometer height
 * @param rf_alt_meas  Altimeter measurement 
 * @param fuse_hgt_meas  Fusion height measurement
 * @param target_vel_x  target_vel
 * @param target_vel_y  target_vel
 * @param target_vel_z  target_vel
 * @param current_vel_x current_vel 
 * @param current_vel_y current_vel 
 * @param current_vel_z current_vel 
 * @param target_angle_x  target_angle
 * @param target_angle_y target_angle 
 * @param target_angle_z  target_angle
 * @param current_angle_x current_angle 
 * @param current_angle_y current_angle 
 * @param current_angle_z  current_angle
 * @param target_angle_rate_x target_angle_rate 
 * @param target_angle_rate_y  target_angle_rate
 * @param target_angle_rate_z target_angle_rate 
 * @param current_angle_rate_x current_angle_rate 
 * @param current_angle_rate_y current_angle_rate 
 * @param current_angle_rate_z current_angle_rate 
 * @param acc_roll_angle Sensor roll angle 
 * @param acc_pitch_angle  Sensor pitch angle
 * @param mag_yaw_angle  Magnetometer yaw angle
 * @param rtk_yaw_angle  RTK yaw angle
 * @param Reserve1  
 * @param Reserve2  
 * @param Reserve3  
 * @param Reserve4  
 * @param flag  
 * @param motor_chan1 motor[0,100]%
 * @param motor_chan2 motor[0,100]% 
 * @param motor_chan3 motor[0,100]% 
 * @param motor_chan4 motor[0,100]% 
 * @param motor_chan5 motor[0,100]%
 * @param motor_chan6 motor[0,100]%
 * @param motor_chan7 motor[0,100]%
 * @param motor_chan8 motor[0,100]%
 * @param rc_chan1 rc[0,100]%
 * @param rc_chan2 rc[0,100]%
 * @param rc_chan3 rc[0,100]%
 * @param rc_chan4 rc[0,100]%
 * @param rc_chan5_8  
 * @param rc_chan9_12  
 * @param rc_chan13_16  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16)
{
    return _ttalink_exam_data_pack(dst_addr, src_addr, msg,  time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, true);
}


static inline uint16_t _ttalink_exam_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time,uint64_t uav_id,float target_pos_x,float target_pos_y,float target_pos_z,float current_pos_x,float current_pos_y,float current_pos_z,float rf_filter_alt,float GPS_RTK_pos_x,float GPS_RTK_pos_y,float GPS_RTK_pos_z,float baro_alt_meas,float rf_alt_meas,float fuse_hgt_meas,int16_t target_vel_x,int16_t target_vel_y,int16_t target_vel_z,int16_t current_vel_x,int16_t current_vel_y,int16_t current_vel_z,int16_t target_angle_x,int16_t target_angle_y,int16_t target_angle_z,int16_t current_angle_x,int16_t current_angle_y,int16_t current_angle_z,int16_t target_angle_rate_x,int16_t target_angle_rate_y,int16_t target_angle_rate_z,int16_t current_angle_rate_x,int16_t current_angle_rate_y,int16_t current_angle_rate_z,int16_t acc_roll_angle,int16_t acc_pitch_angle,int16_t mag_yaw_angle,int16_t rtk_yaw_angle,int16_t Reserve1,int16_t Reserve2,int16_t Reserve3,int16_t Reserve4,uint16_t flag,uint8_t motor_chan1,uint8_t motor_chan2,uint8_t motor_chan3,uint8_t motor_chan4,uint8_t motor_chan5,uint8_t motor_chan6,uint8_t motor_chan7,uint8_t motor_chan8,int8_t rc_chan1,int8_t rc_chan2,int8_t rc_chan3,int8_t rc_chan4,uint8_t rc_chan5_8,uint8_t rc_chan9_12,uint8_t rc_chan13_16, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXAM_DATA_LEN];
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_float(buf, 16, target_pos_x);
    _tta_put_float(buf, 20, target_pos_y);
    _tta_put_float(buf, 24, target_pos_z);
    _tta_put_float(buf, 28, current_pos_x);
    _tta_put_float(buf, 32, current_pos_y);
    _tta_put_float(buf, 36, current_pos_z);
    _tta_put_float(buf, 40, rf_filter_alt);
    _tta_put_float(buf, 44, GPS_RTK_pos_x);
    _tta_put_float(buf, 48, GPS_RTK_pos_y);
    _tta_put_float(buf, 52, GPS_RTK_pos_z);
    _tta_put_float(buf, 56, baro_alt_meas);
    _tta_put_float(buf, 60, rf_alt_meas);
    _tta_put_float(buf, 64, fuse_hgt_meas);
    _tta_put_int16_t(buf, 68, target_vel_x);
    _tta_put_int16_t(buf, 70, target_vel_y);
    _tta_put_int16_t(buf, 72, target_vel_z);
    _tta_put_int16_t(buf, 74, current_vel_x);
    _tta_put_int16_t(buf, 76, current_vel_y);
    _tta_put_int16_t(buf, 78, current_vel_z);
    _tta_put_int16_t(buf, 80, target_angle_x);
    _tta_put_int16_t(buf, 82, target_angle_y);
    _tta_put_int16_t(buf, 84, target_angle_z);
    _tta_put_int16_t(buf, 86, current_angle_x);
    _tta_put_int16_t(buf, 88, current_angle_y);
    _tta_put_int16_t(buf, 90, current_angle_z);
    _tta_put_int16_t(buf, 92, target_angle_rate_x);
    _tta_put_int16_t(buf, 94, target_angle_rate_y);
    _tta_put_int16_t(buf, 96, target_angle_rate_z);
    _tta_put_int16_t(buf, 98, current_angle_rate_x);
    _tta_put_int16_t(buf, 100, current_angle_rate_y);
    _tta_put_int16_t(buf, 102, current_angle_rate_z);
    _tta_put_int16_t(buf, 104, acc_roll_angle);
    _tta_put_int16_t(buf, 106, acc_pitch_angle);
    _tta_put_int16_t(buf, 108, mag_yaw_angle);
    _tta_put_int16_t(buf, 110, rtk_yaw_angle);
    _tta_put_int16_t(buf, 112, Reserve1);
    _tta_put_int16_t(buf, 114, Reserve2);
    _tta_put_int16_t(buf, 116, Reserve3);
    _tta_put_int16_t(buf, 118, Reserve4);
    _tta_put_uint16_t(buf, 120, flag);
    _tta_put_uint8_t(buf, 122, motor_chan1);
    _tta_put_uint8_t(buf, 123, motor_chan2);
    _tta_put_uint8_t(buf, 124, motor_chan3);
    _tta_put_uint8_t(buf, 125, motor_chan4);
    _tta_put_uint8_t(buf, 126, motor_chan5);
    _tta_put_uint8_t(buf, 127, motor_chan6);
    _tta_put_uint8_t(buf, 128, motor_chan7);
    _tta_put_uint8_t(buf, 129, motor_chan8);
    _tta_put_int8_t(buf, 130, rc_chan1);
    _tta_put_int8_t(buf, 131, rc_chan2);
    _tta_put_int8_t(buf, 132, rc_chan3);
    _tta_put_int8_t(buf, 133, rc_chan4);
    _tta_put_uint8_t(buf, 134, rc_chan5_8);
    _tta_put_uint8_t(buf, 135, rc_chan9_12);
    _tta_put_uint8_t(buf, 136, rc_chan13_16);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_EXAM_DATA_LEN);
#else
    ttalink_exam_data_t packet;
    packet.time = time;
    packet.uav_id = uav_id;
    packet.target_pos_x = target_pos_x;
    packet.target_pos_y = target_pos_y;
    packet.target_pos_z = target_pos_z;
    packet.current_pos_x = current_pos_x;
    packet.current_pos_y = current_pos_y;
    packet.current_pos_z = current_pos_z;
    packet.rf_filter_alt = rf_filter_alt;
    packet.GPS_RTK_pos_x = GPS_RTK_pos_x;
    packet.GPS_RTK_pos_y = GPS_RTK_pos_y;
    packet.GPS_RTK_pos_z = GPS_RTK_pos_z;
    packet.baro_alt_meas = baro_alt_meas;
    packet.rf_alt_meas = rf_alt_meas;
    packet.fuse_hgt_meas = fuse_hgt_meas;
    packet.target_vel_x = target_vel_x;
    packet.target_vel_y = target_vel_y;
    packet.target_vel_z = target_vel_z;
    packet.current_vel_x = current_vel_x;
    packet.current_vel_y = current_vel_y;
    packet.current_vel_z = current_vel_z;
    packet.target_angle_x = target_angle_x;
    packet.target_angle_y = target_angle_y;
    packet.target_angle_z = target_angle_z;
    packet.current_angle_x = current_angle_x;
    packet.current_angle_y = current_angle_y;
    packet.current_angle_z = current_angle_z;
    packet.target_angle_rate_x = target_angle_rate_x;
    packet.target_angle_rate_y = target_angle_rate_y;
    packet.target_angle_rate_z = target_angle_rate_z;
    packet.current_angle_rate_x = current_angle_rate_x;
    packet.current_angle_rate_y = current_angle_rate_y;
    packet.current_angle_rate_z = current_angle_rate_z;
    packet.acc_roll_angle = acc_roll_angle;
    packet.acc_pitch_angle = acc_pitch_angle;
    packet.mag_yaw_angle = mag_yaw_angle;
    packet.rtk_yaw_angle = rtk_yaw_angle;
    packet.Reserve1 = Reserve1;
    packet.Reserve2 = Reserve2;
    packet.Reserve3 = Reserve3;
    packet.Reserve4 = Reserve4;
    packet.flag = flag;
    packet.motor_chan1 = motor_chan1;
    packet.motor_chan2 = motor_chan2;
    packet.motor_chan3 = motor_chan3;
    packet.motor_chan4 = motor_chan4;
    packet.motor_chan5 = motor_chan5;
    packet.motor_chan6 = motor_chan6;
    packet.motor_chan7 = motor_chan7;
    packet.motor_chan8 = motor_chan8;
    packet.rc_chan1 = rc_chan1;
    packet.rc_chan2 = rc_chan2;
    packet.rc_chan3 = rc_chan3;
    packet.rc_chan4 = rc_chan4;
    packet.rc_chan5_8 = rc_chan5_8;
    packet.rc_chan9_12 = rc_chan9_12;
    packet.rc_chan13_16 = rc_chan13_16;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_EXAM_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_EXAM_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_DATA_MIN_LEN, TTALINK_MSG_ID_EXAM_DATA_LEN, TTALINK_MSG_ID_EXAM_DATA_CRC, nocrc);
}

/**
 * @brief Pack a exam_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param target_pos_x  Relative home point 
 * @param target_pos_y Relative home point 
 * @param target_pos_z Relative home point 
 * @param current_pos_x  Relative home point
 * @param current_pos_y Relative home point 
 * @param current_pos_z Relative home point 
 * @param rf_filter_alt Height after filtering of radar 
 * @param GPS_RTK_pos_x  GPS-RTK Relative home point
 * @param GPS_RTK_pos_y  GPS-RTK Relative home point
 * @param GPS_RTK_pos_z  GPS-RTK Relative home point
 * @param baro_alt_meas  Barometer height
 * @param rf_alt_meas  Altimeter measurement 
 * @param fuse_hgt_meas  Fusion height measurement
 * @param target_vel_x  target_vel
 * @param target_vel_y  target_vel
 * @param target_vel_z  target_vel
 * @param current_vel_x current_vel 
 * @param current_vel_y current_vel 
 * @param current_vel_z current_vel 
 * @param target_angle_x  target_angle
 * @param target_angle_y target_angle 
 * @param target_angle_z  target_angle
 * @param current_angle_x current_angle 
 * @param current_angle_y current_angle 
 * @param current_angle_z  current_angle
 * @param target_angle_rate_x target_angle_rate 
 * @param target_angle_rate_y  target_angle_rate
 * @param target_angle_rate_z target_angle_rate 
 * @param current_angle_rate_x current_angle_rate 
 * @param current_angle_rate_y current_angle_rate 
 * @param current_angle_rate_z current_angle_rate 
 * @param acc_roll_angle Sensor roll angle 
 * @param acc_pitch_angle  Sensor pitch angle
 * @param mag_yaw_angle  Magnetometer yaw angle
 * @param rtk_yaw_angle  RTK yaw angle
 * @param Reserve1  
 * @param Reserve2  
 * @param Reserve3  
 * @param Reserve4  
 * @param flag  
 * @param motor_chan1 motor[0,100]%
 * @param motor_chan2 motor[0,100]% 
 * @param motor_chan3 motor[0,100]% 
 * @param motor_chan4 motor[0,100]% 
 * @param motor_chan5 motor[0,100]%
 * @param motor_chan6 motor[0,100]%
 * @param motor_chan7 motor[0,100]%
 * @param motor_chan8 motor[0,100]%
 * @param rc_chan1 rc[0,100]%
 * @param rc_chan2 rc[0,100]%
 * @param rc_chan3 rc[0,100]%
 * @param rc_chan4 rc[0,100]%
 * @param rc_chan5_8  
 * @param rc_chan9_12  
 * @param rc_chan13_16  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time,uint64_t uav_id,float target_pos_x,float target_pos_y,float target_pos_z,float current_pos_x,float current_pos_y,float current_pos_z,float rf_filter_alt,float GPS_RTK_pos_x,float GPS_RTK_pos_y,float GPS_RTK_pos_z,float baro_alt_meas,float rf_alt_meas,float fuse_hgt_meas,int16_t target_vel_x,int16_t target_vel_y,int16_t target_vel_z,int16_t current_vel_x,int16_t current_vel_y,int16_t current_vel_z,int16_t target_angle_x,int16_t target_angle_y,int16_t target_angle_z,int16_t current_angle_x,int16_t current_angle_y,int16_t current_angle_z,int16_t target_angle_rate_x,int16_t target_angle_rate_y,int16_t target_angle_rate_z,int16_t current_angle_rate_x,int16_t current_angle_rate_y,int16_t current_angle_rate_z,int16_t acc_roll_angle,int16_t acc_pitch_angle,int16_t mag_yaw_angle,int16_t rtk_yaw_angle,int16_t Reserve1,int16_t Reserve2,int16_t Reserve3,int16_t Reserve4,uint16_t flag,uint8_t motor_chan1,uint8_t motor_chan2,uint8_t motor_chan3,uint8_t motor_chan4,uint8_t motor_chan5,uint8_t motor_chan6,uint8_t motor_chan7,uint8_t motor_chan8,int8_t rc_chan1,int8_t rc_chan2,int8_t rc_chan3,int8_t rc_chan4,uint8_t rc_chan5_8,uint8_t rc_chan9_12,uint8_t rc_chan13_16)
{
    return _ttalink_exam_data_pack_chan(dst_addr, src_addr, chan, msg,  time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, false);
}

/**
 * @brief Pack a exam_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param target_pos_x  Relative home point 
 * @param target_pos_y Relative home point 
 * @param target_pos_z Relative home point 
 * @param current_pos_x  Relative home point
 * @param current_pos_y Relative home point 
 * @param current_pos_z Relative home point 
 * @param rf_filter_alt Height after filtering of radar 
 * @param GPS_RTK_pos_x  GPS-RTK Relative home point
 * @param GPS_RTK_pos_y  GPS-RTK Relative home point
 * @param GPS_RTK_pos_z  GPS-RTK Relative home point
 * @param baro_alt_meas  Barometer height
 * @param rf_alt_meas  Altimeter measurement 
 * @param fuse_hgt_meas  Fusion height measurement
 * @param target_vel_x  target_vel
 * @param target_vel_y  target_vel
 * @param target_vel_z  target_vel
 * @param current_vel_x current_vel 
 * @param current_vel_y current_vel 
 * @param current_vel_z current_vel 
 * @param target_angle_x  target_angle
 * @param target_angle_y target_angle 
 * @param target_angle_z  target_angle
 * @param current_angle_x current_angle 
 * @param current_angle_y current_angle 
 * @param current_angle_z  current_angle
 * @param target_angle_rate_x target_angle_rate 
 * @param target_angle_rate_y  target_angle_rate
 * @param target_angle_rate_z target_angle_rate 
 * @param current_angle_rate_x current_angle_rate 
 * @param current_angle_rate_y current_angle_rate 
 * @param current_angle_rate_z current_angle_rate 
 * @param acc_roll_angle Sensor roll angle 
 * @param acc_pitch_angle  Sensor pitch angle
 * @param mag_yaw_angle  Magnetometer yaw angle
 * @param rtk_yaw_angle  RTK yaw angle
 * @param Reserve1  
 * @param Reserve2  
 * @param Reserve3  
 * @param Reserve4  
 * @param flag  
 * @param motor_chan1 motor[0,100]%
 * @param motor_chan2 motor[0,100]% 
 * @param motor_chan3 motor[0,100]% 
 * @param motor_chan4 motor[0,100]% 
 * @param motor_chan5 motor[0,100]%
 * @param motor_chan6 motor[0,100]%
 * @param motor_chan7 motor[0,100]%
 * @param motor_chan8 motor[0,100]%
 * @param rc_chan1 rc[0,100]%
 * @param rc_chan2 rc[0,100]%
 * @param rc_chan3 rc[0,100]%
 * @param rc_chan4 rc[0,100]%
 * @param rc_chan5_8  
 * @param rc_chan9_12  
 * @param rc_chan13_16  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_exam_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t time,uint64_t uav_id,float target_pos_x,float target_pos_y,float target_pos_z,float current_pos_x,float current_pos_y,float current_pos_z,float rf_filter_alt,float GPS_RTK_pos_x,float GPS_RTK_pos_y,float GPS_RTK_pos_z,float baro_alt_meas,float rf_alt_meas,float fuse_hgt_meas,int16_t target_vel_x,int16_t target_vel_y,int16_t target_vel_z,int16_t current_vel_x,int16_t current_vel_y,int16_t current_vel_z,int16_t target_angle_x,int16_t target_angle_y,int16_t target_angle_z,int16_t current_angle_x,int16_t current_angle_y,int16_t current_angle_z,int16_t target_angle_rate_x,int16_t target_angle_rate_y,int16_t target_angle_rate_z,int16_t current_angle_rate_x,int16_t current_angle_rate_y,int16_t current_angle_rate_z,int16_t acc_roll_angle,int16_t acc_pitch_angle,int16_t mag_yaw_angle,int16_t rtk_yaw_angle,int16_t Reserve1,int16_t Reserve2,int16_t Reserve3,int16_t Reserve4,uint16_t flag,uint8_t motor_chan1,uint8_t motor_chan2,uint8_t motor_chan3,uint8_t motor_chan4,uint8_t motor_chan5,uint8_t motor_chan6,uint8_t motor_chan7,uint8_t motor_chan8,int8_t rc_chan1,int8_t rc_chan2,int8_t rc_chan3,int8_t rc_chan4,uint8_t rc_chan5_8,uint8_t rc_chan9_12,uint8_t rc_chan13_16)
{
    return _ttalink_exam_data_pack_chan(dst_addr, src_addr, chan, msg,  time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, true);
}


static inline uint16_t _ttalink_exam_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_exam_data_t* exam_data, bool nocrc)
{
    if(nocrc){
        return ttalink_exam_data_pack_nocrc(dst_addr, src_addr, msg, exam_data->time, exam_data->uav_id, exam_data->target_pos_x, exam_data->target_pos_y, exam_data->target_pos_z, exam_data->current_pos_x, exam_data->current_pos_y, exam_data->current_pos_z, exam_data->rf_filter_alt, exam_data->GPS_RTK_pos_x, exam_data->GPS_RTK_pos_y, exam_data->GPS_RTK_pos_z, exam_data->baro_alt_meas, exam_data->rf_alt_meas, exam_data->fuse_hgt_meas, exam_data->target_vel_x, exam_data->target_vel_y, exam_data->target_vel_z, exam_data->current_vel_x, exam_data->current_vel_y, exam_data->current_vel_z, exam_data->target_angle_x, exam_data->target_angle_y, exam_data->target_angle_z, exam_data->current_angle_x, exam_data->current_angle_y, exam_data->current_angle_z, exam_data->target_angle_rate_x, exam_data->target_angle_rate_y, exam_data->target_angle_rate_z, exam_data->current_angle_rate_x, exam_data->current_angle_rate_y, exam_data->current_angle_rate_z, exam_data->acc_roll_angle, exam_data->acc_pitch_angle, exam_data->mag_yaw_angle, exam_data->rtk_yaw_angle, exam_data->Reserve1, exam_data->Reserve2, exam_data->Reserve3, exam_data->Reserve4, exam_data->flag, exam_data->motor_chan1, exam_data->motor_chan2, exam_data->motor_chan3, exam_data->motor_chan4, exam_data->motor_chan5, exam_data->motor_chan6, exam_data->motor_chan7, exam_data->motor_chan8, exam_data->rc_chan1, exam_data->rc_chan2, exam_data->rc_chan3, exam_data->rc_chan4, exam_data->rc_chan5_8, exam_data->rc_chan9_12, exam_data->rc_chan13_16);
    }else{
        return ttalink_exam_data_pack(dst_addr, src_addr, msg, exam_data->time, exam_data->uav_id, exam_data->target_pos_x, exam_data->target_pos_y, exam_data->target_pos_z, exam_data->current_pos_x, exam_data->current_pos_y, exam_data->current_pos_z, exam_data->rf_filter_alt, exam_data->GPS_RTK_pos_x, exam_data->GPS_RTK_pos_y, exam_data->GPS_RTK_pos_z, exam_data->baro_alt_meas, exam_data->rf_alt_meas, exam_data->fuse_hgt_meas, exam_data->target_vel_x, exam_data->target_vel_y, exam_data->target_vel_z, exam_data->current_vel_x, exam_data->current_vel_y, exam_data->current_vel_z, exam_data->target_angle_x, exam_data->target_angle_y, exam_data->target_angle_z, exam_data->current_angle_x, exam_data->current_angle_y, exam_data->current_angle_z, exam_data->target_angle_rate_x, exam_data->target_angle_rate_y, exam_data->target_angle_rate_z, exam_data->current_angle_rate_x, exam_data->current_angle_rate_y, exam_data->current_angle_rate_z, exam_data->acc_roll_angle, exam_data->acc_pitch_angle, exam_data->mag_yaw_angle, exam_data->rtk_yaw_angle, exam_data->Reserve1, exam_data->Reserve2, exam_data->Reserve3, exam_data->Reserve4, exam_data->flag, exam_data->motor_chan1, exam_data->motor_chan2, exam_data->motor_chan3, exam_data->motor_chan4, exam_data->motor_chan5, exam_data->motor_chan6, exam_data->motor_chan7, exam_data->motor_chan8, exam_data->rc_chan1, exam_data->rc_chan2, exam_data->rc_chan3, exam_data->rc_chan4, exam_data->rc_chan5_8, exam_data->rc_chan9_12, exam_data->rc_chan13_16);
    }
    
}

/**
 * @brief Encode a exam_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param exam_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_exam_data_t* exam_data)
{
    return _ttalink_exam_data_encode(dst_addr, src_addr, msg, exam_data, false);
}

/**
 * @brief Encode a exam_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param exam_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_exam_data_t* exam_data)
{
    return _ttalink_exam_data_encode(dst_addr, src_addr, msg, exam_data, true);
}


static inline uint16_t _ttalink_exam_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_exam_data_t* exam_data, bool nocrc)
{
    if(nocrc){
        return ttalink_exam_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, exam_data->time, exam_data->uav_id, exam_data->target_pos_x, exam_data->target_pos_y, exam_data->target_pos_z, exam_data->current_pos_x, exam_data->current_pos_y, exam_data->current_pos_z, exam_data->rf_filter_alt, exam_data->GPS_RTK_pos_x, exam_data->GPS_RTK_pos_y, exam_data->GPS_RTK_pos_z, exam_data->baro_alt_meas, exam_data->rf_alt_meas, exam_data->fuse_hgt_meas, exam_data->target_vel_x, exam_data->target_vel_y, exam_data->target_vel_z, exam_data->current_vel_x, exam_data->current_vel_y, exam_data->current_vel_z, exam_data->target_angle_x, exam_data->target_angle_y, exam_data->target_angle_z, exam_data->current_angle_x, exam_data->current_angle_y, exam_data->current_angle_z, exam_data->target_angle_rate_x, exam_data->target_angle_rate_y, exam_data->target_angle_rate_z, exam_data->current_angle_rate_x, exam_data->current_angle_rate_y, exam_data->current_angle_rate_z, exam_data->acc_roll_angle, exam_data->acc_pitch_angle, exam_data->mag_yaw_angle, exam_data->rtk_yaw_angle, exam_data->Reserve1, exam_data->Reserve2, exam_data->Reserve3, exam_data->Reserve4, exam_data->flag, exam_data->motor_chan1, exam_data->motor_chan2, exam_data->motor_chan3, exam_data->motor_chan4, exam_data->motor_chan5, exam_data->motor_chan6, exam_data->motor_chan7, exam_data->motor_chan8, exam_data->rc_chan1, exam_data->rc_chan2, exam_data->rc_chan3, exam_data->rc_chan4, exam_data->rc_chan5_8, exam_data->rc_chan9_12, exam_data->rc_chan13_16);
    }else{
        return ttalink_exam_data_pack_chan(dst_addr, src_addr, chan, msg, exam_data->time, exam_data->uav_id, exam_data->target_pos_x, exam_data->target_pos_y, exam_data->target_pos_z, exam_data->current_pos_x, exam_data->current_pos_y, exam_data->current_pos_z, exam_data->rf_filter_alt, exam_data->GPS_RTK_pos_x, exam_data->GPS_RTK_pos_y, exam_data->GPS_RTK_pos_z, exam_data->baro_alt_meas, exam_data->rf_alt_meas, exam_data->fuse_hgt_meas, exam_data->target_vel_x, exam_data->target_vel_y, exam_data->target_vel_z, exam_data->current_vel_x, exam_data->current_vel_y, exam_data->current_vel_z, exam_data->target_angle_x, exam_data->target_angle_y, exam_data->target_angle_z, exam_data->current_angle_x, exam_data->current_angle_y, exam_data->current_angle_z, exam_data->target_angle_rate_x, exam_data->target_angle_rate_y, exam_data->target_angle_rate_z, exam_data->current_angle_rate_x, exam_data->current_angle_rate_y, exam_data->current_angle_rate_z, exam_data->acc_roll_angle, exam_data->acc_pitch_angle, exam_data->mag_yaw_angle, exam_data->rtk_yaw_angle, exam_data->Reserve1, exam_data->Reserve2, exam_data->Reserve3, exam_data->Reserve4, exam_data->flag, exam_data->motor_chan1, exam_data->motor_chan2, exam_data->motor_chan3, exam_data->motor_chan4, exam_data->motor_chan5, exam_data->motor_chan6, exam_data->motor_chan7, exam_data->motor_chan8, exam_data->rc_chan1, exam_data->rc_chan2, exam_data->rc_chan3, exam_data->rc_chan4, exam_data->rc_chan5_8, exam_data->rc_chan9_12, exam_data->rc_chan13_16);
    }
}

/**
 * @brief Encode a exam_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param exam_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_exam_data_t* exam_data)
{
    return _ttalink_exam_data_encode_chan(dst_addr, src_addr, chan, msg, exam_data, false);
}

/**
 * @brief Encode a exam_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param exam_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_exam_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_exam_data_t* exam_data)
{
    return _ttalink_exam_data_encode_chan(dst_addr, src_addr, chan, msg, exam_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_exam_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_EXAM_DATA_LEN];
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_float(buf, 16, target_pos_x);
    _tta_put_float(buf, 20, target_pos_y);
    _tta_put_float(buf, 24, target_pos_z);
    _tta_put_float(buf, 28, current_pos_x);
    _tta_put_float(buf, 32, current_pos_y);
    _tta_put_float(buf, 36, current_pos_z);
    _tta_put_float(buf, 40, rf_filter_alt);
    _tta_put_float(buf, 44, GPS_RTK_pos_x);
    _tta_put_float(buf, 48, GPS_RTK_pos_y);
    _tta_put_float(buf, 52, GPS_RTK_pos_z);
    _tta_put_float(buf, 56, baro_alt_meas);
    _tta_put_float(buf, 60, rf_alt_meas);
    _tta_put_float(buf, 64, fuse_hgt_meas);
    _tta_put_int16_t(buf, 68, target_vel_x);
    _tta_put_int16_t(buf, 70, target_vel_y);
    _tta_put_int16_t(buf, 72, target_vel_z);
    _tta_put_int16_t(buf, 74, current_vel_x);
    _tta_put_int16_t(buf, 76, current_vel_y);
    _tta_put_int16_t(buf, 78, current_vel_z);
    _tta_put_int16_t(buf, 80, target_angle_x);
    _tta_put_int16_t(buf, 82, target_angle_y);
    _tta_put_int16_t(buf, 84, target_angle_z);
    _tta_put_int16_t(buf, 86, current_angle_x);
    _tta_put_int16_t(buf, 88, current_angle_y);
    _tta_put_int16_t(buf, 90, current_angle_z);
    _tta_put_int16_t(buf, 92, target_angle_rate_x);
    _tta_put_int16_t(buf, 94, target_angle_rate_y);
    _tta_put_int16_t(buf, 96, target_angle_rate_z);
    _tta_put_int16_t(buf, 98, current_angle_rate_x);
    _tta_put_int16_t(buf, 100, current_angle_rate_y);
    _tta_put_int16_t(buf, 102, current_angle_rate_z);
    _tta_put_int16_t(buf, 104, acc_roll_angle);
    _tta_put_int16_t(buf, 106, acc_pitch_angle);
    _tta_put_int16_t(buf, 108, mag_yaw_angle);
    _tta_put_int16_t(buf, 110, rtk_yaw_angle);
    _tta_put_int16_t(buf, 112, Reserve1);
    _tta_put_int16_t(buf, 114, Reserve2);
    _tta_put_int16_t(buf, 116, Reserve3);
    _tta_put_int16_t(buf, 118, Reserve4);
    _tta_put_uint16_t(buf, 120, flag);
    _tta_put_uint8_t(buf, 122, motor_chan1);
    _tta_put_uint8_t(buf, 123, motor_chan2);
    _tta_put_uint8_t(buf, 124, motor_chan3);
    _tta_put_uint8_t(buf, 125, motor_chan4);
    _tta_put_uint8_t(buf, 126, motor_chan5);
    _tta_put_uint8_t(buf, 127, motor_chan6);
    _tta_put_uint8_t(buf, 128, motor_chan7);
    _tta_put_uint8_t(buf, 129, motor_chan8);
    _tta_put_int8_t(buf, 130, rc_chan1);
    _tta_put_int8_t(buf, 131, rc_chan2);
    _tta_put_int8_t(buf, 132, rc_chan3);
    _tta_put_int8_t(buf, 133, rc_chan4);
    _tta_put_uint8_t(buf, 134, rc_chan5_8);
    _tta_put_uint8_t(buf, 135, rc_chan9_12);
    _tta_put_uint8_t(buf, 136, rc_chan13_16);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_DATA, buf, TTALINK_MSG_ID_EXAM_DATA_MIN_LEN, TTALINK_MSG_ID_EXAM_DATA_LEN, TTALINK_MSG_ID_EXAM_DATA_CRC, nocrc);
#else
    ttalink_exam_data_t packet;
    packet.time = time;
    packet.uav_id = uav_id;
    packet.target_pos_x = target_pos_x;
    packet.target_pos_y = target_pos_y;
    packet.target_pos_z = target_pos_z;
    packet.current_pos_x = current_pos_x;
    packet.current_pos_y = current_pos_y;
    packet.current_pos_z = current_pos_z;
    packet.rf_filter_alt = rf_filter_alt;
    packet.GPS_RTK_pos_x = GPS_RTK_pos_x;
    packet.GPS_RTK_pos_y = GPS_RTK_pos_y;
    packet.GPS_RTK_pos_z = GPS_RTK_pos_z;
    packet.baro_alt_meas = baro_alt_meas;
    packet.rf_alt_meas = rf_alt_meas;
    packet.fuse_hgt_meas = fuse_hgt_meas;
    packet.target_vel_x = target_vel_x;
    packet.target_vel_y = target_vel_y;
    packet.target_vel_z = target_vel_z;
    packet.current_vel_x = current_vel_x;
    packet.current_vel_y = current_vel_y;
    packet.current_vel_z = current_vel_z;
    packet.target_angle_x = target_angle_x;
    packet.target_angle_y = target_angle_y;
    packet.target_angle_z = target_angle_z;
    packet.current_angle_x = current_angle_x;
    packet.current_angle_y = current_angle_y;
    packet.current_angle_z = current_angle_z;
    packet.target_angle_rate_x = target_angle_rate_x;
    packet.target_angle_rate_y = target_angle_rate_y;
    packet.target_angle_rate_z = target_angle_rate_z;
    packet.current_angle_rate_x = current_angle_rate_x;
    packet.current_angle_rate_y = current_angle_rate_y;
    packet.current_angle_rate_z = current_angle_rate_z;
    packet.acc_roll_angle = acc_roll_angle;
    packet.acc_pitch_angle = acc_pitch_angle;
    packet.mag_yaw_angle = mag_yaw_angle;
    packet.rtk_yaw_angle = rtk_yaw_angle;
    packet.Reserve1 = Reserve1;
    packet.Reserve2 = Reserve2;
    packet.Reserve3 = Reserve3;
    packet.Reserve4 = Reserve4;
    packet.flag = flag;
    packet.motor_chan1 = motor_chan1;
    packet.motor_chan2 = motor_chan2;
    packet.motor_chan3 = motor_chan3;
    packet.motor_chan4 = motor_chan4;
    packet.motor_chan5 = motor_chan5;
    packet.motor_chan6 = motor_chan6;
    packet.motor_chan7 = motor_chan7;
    packet.motor_chan8 = motor_chan8;
    packet.rc_chan1 = rc_chan1;
    packet.rc_chan2 = rc_chan2;
    packet.rc_chan3 = rc_chan3;
    packet.rc_chan4 = rc_chan4;
    packet.rc_chan5_8 = rc_chan5_8;
    packet.rc_chan9_12 = rc_chan9_12;
    packet.rc_chan13_16 = rc_chan13_16;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_DATA, (const char *)&packet, TTALINK_MSG_ID_EXAM_DATA_MIN_LEN, TTALINK_MSG_ID_EXAM_DATA_LEN, TTALINK_MSG_ID_EXAM_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a exam_data message
 * @param chan TTAlink channel to send the message
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param target_pos_x  Relative home point 
 * @param target_pos_y Relative home point 
 * @param target_pos_z Relative home point 
 * @param current_pos_x  Relative home point
 * @param current_pos_y Relative home point 
 * @param current_pos_z Relative home point 
 * @param rf_filter_alt Height after filtering of radar 
 * @param GPS_RTK_pos_x  GPS-RTK Relative home point
 * @param GPS_RTK_pos_y  GPS-RTK Relative home point
 * @param GPS_RTK_pos_z  GPS-RTK Relative home point
 * @param baro_alt_meas  Barometer height
 * @param rf_alt_meas  Altimeter measurement 
 * @param fuse_hgt_meas  Fusion height measurement
 * @param target_vel_x  target_vel
 * @param target_vel_y  target_vel
 * @param target_vel_z  target_vel
 * @param current_vel_x current_vel 
 * @param current_vel_y current_vel 
 * @param current_vel_z current_vel 
 * @param target_angle_x  target_angle
 * @param target_angle_y target_angle 
 * @param target_angle_z  target_angle
 * @param current_angle_x current_angle 
 * @param current_angle_y current_angle 
 * @param current_angle_z  current_angle
 * @param target_angle_rate_x target_angle_rate 
 * @param target_angle_rate_y  target_angle_rate
 * @param target_angle_rate_z target_angle_rate 
 * @param current_angle_rate_x current_angle_rate 
 * @param current_angle_rate_y current_angle_rate 
 * @param current_angle_rate_z current_angle_rate 
 * @param acc_roll_angle Sensor roll angle 
 * @param acc_pitch_angle  Sensor pitch angle
 * @param mag_yaw_angle  Magnetometer yaw angle
 * @param rtk_yaw_angle  RTK yaw angle
 * @param Reserve1  
 * @param Reserve2  
 * @param Reserve3  
 * @param Reserve4  
 * @param flag  
 * @param motor_chan1 motor[0,100]%
 * @param motor_chan2 motor[0,100]% 
 * @param motor_chan3 motor[0,100]% 
 * @param motor_chan4 motor[0,100]% 
 * @param motor_chan5 motor[0,100]%
 * @param motor_chan6 motor[0,100]%
 * @param motor_chan7 motor[0,100]%
 * @param motor_chan8 motor[0,100]%
 * @param rc_chan1 rc[0,100]%
 * @param rc_chan2 rc[0,100]%
 * @param rc_chan3 rc[0,100]%
 * @param rc_chan4 rc[0,100]%
 * @param rc_chan5_8  
 * @param rc_chan9_12  
 * @param rc_chan13_16  
 */
static inline void ttalink_exam_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16)
{
    _ttalink_exam_data_send(dst_addr, src_addr, chan, time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, false);
}

/**
 * @brief Send a exam_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param time  utc_time(MS)
 * @param uav_id  uav_id.
 * @param target_pos_x  Relative home point 
 * @param target_pos_y Relative home point 
 * @param target_pos_z Relative home point 
 * @param current_pos_x  Relative home point
 * @param current_pos_y Relative home point 
 * @param current_pos_z Relative home point 
 * @param rf_filter_alt Height after filtering of radar 
 * @param GPS_RTK_pos_x  GPS-RTK Relative home point
 * @param GPS_RTK_pos_y  GPS-RTK Relative home point
 * @param GPS_RTK_pos_z  GPS-RTK Relative home point
 * @param baro_alt_meas  Barometer height
 * @param rf_alt_meas  Altimeter measurement 
 * @param fuse_hgt_meas  Fusion height measurement
 * @param target_vel_x  target_vel
 * @param target_vel_y  target_vel
 * @param target_vel_z  target_vel
 * @param current_vel_x current_vel 
 * @param current_vel_y current_vel 
 * @param current_vel_z current_vel 
 * @param target_angle_x  target_angle
 * @param target_angle_y target_angle 
 * @param target_angle_z  target_angle
 * @param current_angle_x current_angle 
 * @param current_angle_y current_angle 
 * @param current_angle_z  current_angle
 * @param target_angle_rate_x target_angle_rate 
 * @param target_angle_rate_y  target_angle_rate
 * @param target_angle_rate_z target_angle_rate 
 * @param current_angle_rate_x current_angle_rate 
 * @param current_angle_rate_y current_angle_rate 
 * @param current_angle_rate_z current_angle_rate 
 * @param acc_roll_angle Sensor roll angle 
 * @param acc_pitch_angle  Sensor pitch angle
 * @param mag_yaw_angle  Magnetometer yaw angle
 * @param rtk_yaw_angle  RTK yaw angle
 * @param Reserve1  
 * @param Reserve2  
 * @param Reserve3  
 * @param Reserve4  
 * @param flag  
 * @param motor_chan1 motor[0,100]%
 * @param motor_chan2 motor[0,100]% 
 * @param motor_chan3 motor[0,100]% 
 * @param motor_chan4 motor[0,100]% 
 * @param motor_chan5 motor[0,100]%
 * @param motor_chan6 motor[0,100]%
 * @param motor_chan7 motor[0,100]%
 * @param motor_chan8 motor[0,100]%
 * @param rc_chan1 rc[0,100]%
 * @param rc_chan2 rc[0,100]%
 * @param rc_chan3 rc[0,100]%
 * @param rc_chan4 rc[0,100]%
 * @param rc_chan5_8  
 * @param rc_chan9_12  
 * @param rc_chan13_16  
 */
static inline void ttalink_exam_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16)
{
    _ttalink_exam_data_send(dst_addr, src_addr, chan, time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, true);
}


static inline void _ttalink_exam_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_exam_data_t* exam_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_exam_data_send_nocrc(dst_addr, src_addr, chan, exam_data->time, exam_data->uav_id, exam_data->target_pos_x, exam_data->target_pos_y, exam_data->target_pos_z, exam_data->current_pos_x, exam_data->current_pos_y, exam_data->current_pos_z, exam_data->rf_filter_alt, exam_data->GPS_RTK_pos_x, exam_data->GPS_RTK_pos_y, exam_data->GPS_RTK_pos_z, exam_data->baro_alt_meas, exam_data->rf_alt_meas, exam_data->fuse_hgt_meas, exam_data->target_vel_x, exam_data->target_vel_y, exam_data->target_vel_z, exam_data->current_vel_x, exam_data->current_vel_y, exam_data->current_vel_z, exam_data->target_angle_x, exam_data->target_angle_y, exam_data->target_angle_z, exam_data->current_angle_x, exam_data->current_angle_y, exam_data->current_angle_z, exam_data->target_angle_rate_x, exam_data->target_angle_rate_y, exam_data->target_angle_rate_z, exam_data->current_angle_rate_x, exam_data->current_angle_rate_y, exam_data->current_angle_rate_z, exam_data->acc_roll_angle, exam_data->acc_pitch_angle, exam_data->mag_yaw_angle, exam_data->rtk_yaw_angle, exam_data->Reserve1, exam_data->Reserve2, exam_data->Reserve3, exam_data->Reserve4, exam_data->flag, exam_data->motor_chan1, exam_data->motor_chan2, exam_data->motor_chan3, exam_data->motor_chan4, exam_data->motor_chan5, exam_data->motor_chan6, exam_data->motor_chan7, exam_data->motor_chan8, exam_data->rc_chan1, exam_data->rc_chan2, exam_data->rc_chan3, exam_data->rc_chan4, exam_data->rc_chan5_8, exam_data->rc_chan9_12, exam_data->rc_chan13_16);
    }else{
        ttalink_exam_data_send(dst_addr, src_addr, chan, exam_data->time, exam_data->uav_id, exam_data->target_pos_x, exam_data->target_pos_y, exam_data->target_pos_z, exam_data->current_pos_x, exam_data->current_pos_y, exam_data->current_pos_z, exam_data->rf_filter_alt, exam_data->GPS_RTK_pos_x, exam_data->GPS_RTK_pos_y, exam_data->GPS_RTK_pos_z, exam_data->baro_alt_meas, exam_data->rf_alt_meas, exam_data->fuse_hgt_meas, exam_data->target_vel_x, exam_data->target_vel_y, exam_data->target_vel_z, exam_data->current_vel_x, exam_data->current_vel_y, exam_data->current_vel_z, exam_data->target_angle_x, exam_data->target_angle_y, exam_data->target_angle_z, exam_data->current_angle_x, exam_data->current_angle_y, exam_data->current_angle_z, exam_data->target_angle_rate_x, exam_data->target_angle_rate_y, exam_data->target_angle_rate_z, exam_data->current_angle_rate_x, exam_data->current_angle_rate_y, exam_data->current_angle_rate_z, exam_data->acc_roll_angle, exam_data->acc_pitch_angle, exam_data->mag_yaw_angle, exam_data->rtk_yaw_angle, exam_data->Reserve1, exam_data->Reserve2, exam_data->Reserve3, exam_data->Reserve4, exam_data->flag, exam_data->motor_chan1, exam_data->motor_chan2, exam_data->motor_chan3, exam_data->motor_chan4, exam_data->motor_chan5, exam_data->motor_chan6, exam_data->motor_chan7, exam_data->motor_chan8, exam_data->rc_chan1, exam_data->rc_chan2, exam_data->rc_chan3, exam_data->rc_chan4, exam_data->rc_chan5_8, exam_data->rc_chan9_12, exam_data->rc_chan13_16);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_DATA, (const char *)exam_data, TTALINK_MSG_ID_EXAM_DATA_MIN_LEN, TTALINK_MSG_ID_EXAM_DATA_LEN, TTALINK_MSG_ID_EXAM_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a exam_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_exam_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_exam_data_t* exam_data)
{
    _ttalink_exam_data_send_struct(dst_addr, src_addr, chan, exam_data, false);
}

/**
 * @brief Send a exam_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_exam_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_exam_data_t* exam_data)
{
    _ttalink_exam_data_send_struct(dst_addr, src_addr, chan, exam_data, true);
}

#if TTALINK_MSG_ID_EXAM_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_exam_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, time);
    _tta_put_uint64_t(buf, 8, uav_id);
    _tta_put_float(buf, 16, target_pos_x);
    _tta_put_float(buf, 20, target_pos_y);
    _tta_put_float(buf, 24, target_pos_z);
    _tta_put_float(buf, 28, current_pos_x);
    _tta_put_float(buf, 32, current_pos_y);
    _tta_put_float(buf, 36, current_pos_z);
    _tta_put_float(buf, 40, rf_filter_alt);
    _tta_put_float(buf, 44, GPS_RTK_pos_x);
    _tta_put_float(buf, 48, GPS_RTK_pos_y);
    _tta_put_float(buf, 52, GPS_RTK_pos_z);
    _tta_put_float(buf, 56, baro_alt_meas);
    _tta_put_float(buf, 60, rf_alt_meas);
    _tta_put_float(buf, 64, fuse_hgt_meas);
    _tta_put_int16_t(buf, 68, target_vel_x);
    _tta_put_int16_t(buf, 70, target_vel_y);
    _tta_put_int16_t(buf, 72, target_vel_z);
    _tta_put_int16_t(buf, 74, current_vel_x);
    _tta_put_int16_t(buf, 76, current_vel_y);
    _tta_put_int16_t(buf, 78, current_vel_z);
    _tta_put_int16_t(buf, 80, target_angle_x);
    _tta_put_int16_t(buf, 82, target_angle_y);
    _tta_put_int16_t(buf, 84, target_angle_z);
    _tta_put_int16_t(buf, 86, current_angle_x);
    _tta_put_int16_t(buf, 88, current_angle_y);
    _tta_put_int16_t(buf, 90, current_angle_z);
    _tta_put_int16_t(buf, 92, target_angle_rate_x);
    _tta_put_int16_t(buf, 94, target_angle_rate_y);
    _tta_put_int16_t(buf, 96, target_angle_rate_z);
    _tta_put_int16_t(buf, 98, current_angle_rate_x);
    _tta_put_int16_t(buf, 100, current_angle_rate_y);
    _tta_put_int16_t(buf, 102, current_angle_rate_z);
    _tta_put_int16_t(buf, 104, acc_roll_angle);
    _tta_put_int16_t(buf, 106, acc_pitch_angle);
    _tta_put_int16_t(buf, 108, mag_yaw_angle);
    _tta_put_int16_t(buf, 110, rtk_yaw_angle);
    _tta_put_int16_t(buf, 112, Reserve1);
    _tta_put_int16_t(buf, 114, Reserve2);
    _tta_put_int16_t(buf, 116, Reserve3);
    _tta_put_int16_t(buf, 118, Reserve4);
    _tta_put_uint16_t(buf, 120, flag);
    _tta_put_uint8_t(buf, 122, motor_chan1);
    _tta_put_uint8_t(buf, 123, motor_chan2);
    _tta_put_uint8_t(buf, 124, motor_chan3);
    _tta_put_uint8_t(buf, 125, motor_chan4);
    _tta_put_uint8_t(buf, 126, motor_chan5);
    _tta_put_uint8_t(buf, 127, motor_chan6);
    _tta_put_uint8_t(buf, 128, motor_chan7);
    _tta_put_uint8_t(buf, 129, motor_chan8);
    _tta_put_int8_t(buf, 130, rc_chan1);
    _tta_put_int8_t(buf, 131, rc_chan2);
    _tta_put_int8_t(buf, 132, rc_chan3);
    _tta_put_int8_t(buf, 133, rc_chan4);
    _tta_put_uint8_t(buf, 134, rc_chan5_8);
    _tta_put_uint8_t(buf, 135, rc_chan9_12);
    _tta_put_uint8_t(buf, 136, rc_chan13_16);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_DATA, buf, TTALINK_MSG_ID_EXAM_DATA_MIN_LEN, TTALINK_MSG_ID_EXAM_DATA_LEN, TTALINK_MSG_ID_EXAM_DATA_CRC, nocrc);
#else
    ttalink_exam_data_t *packet = (ttalink_exam_data_t *)msgbuf;
    packet->time = time;
    packet->uav_id = uav_id;
    packet->target_pos_x = target_pos_x;
    packet->target_pos_y = target_pos_y;
    packet->target_pos_z = target_pos_z;
    packet->current_pos_x = current_pos_x;
    packet->current_pos_y = current_pos_y;
    packet->current_pos_z = current_pos_z;
    packet->rf_filter_alt = rf_filter_alt;
    packet->GPS_RTK_pos_x = GPS_RTK_pos_x;
    packet->GPS_RTK_pos_y = GPS_RTK_pos_y;
    packet->GPS_RTK_pos_z = GPS_RTK_pos_z;
    packet->baro_alt_meas = baro_alt_meas;
    packet->rf_alt_meas = rf_alt_meas;
    packet->fuse_hgt_meas = fuse_hgt_meas;
    packet->target_vel_x = target_vel_x;
    packet->target_vel_y = target_vel_y;
    packet->target_vel_z = target_vel_z;
    packet->current_vel_x = current_vel_x;
    packet->current_vel_y = current_vel_y;
    packet->current_vel_z = current_vel_z;
    packet->target_angle_x = target_angle_x;
    packet->target_angle_y = target_angle_y;
    packet->target_angle_z = target_angle_z;
    packet->current_angle_x = current_angle_x;
    packet->current_angle_y = current_angle_y;
    packet->current_angle_z = current_angle_z;
    packet->target_angle_rate_x = target_angle_rate_x;
    packet->target_angle_rate_y = target_angle_rate_y;
    packet->target_angle_rate_z = target_angle_rate_z;
    packet->current_angle_rate_x = current_angle_rate_x;
    packet->current_angle_rate_y = current_angle_rate_y;
    packet->current_angle_rate_z = current_angle_rate_z;
    packet->acc_roll_angle = acc_roll_angle;
    packet->acc_pitch_angle = acc_pitch_angle;
    packet->mag_yaw_angle = mag_yaw_angle;
    packet->rtk_yaw_angle = rtk_yaw_angle;
    packet->Reserve1 = Reserve1;
    packet->Reserve2 = Reserve2;
    packet->Reserve3 = Reserve3;
    packet->Reserve4 = Reserve4;
    packet->flag = flag;
    packet->motor_chan1 = motor_chan1;
    packet->motor_chan2 = motor_chan2;
    packet->motor_chan3 = motor_chan3;
    packet->motor_chan4 = motor_chan4;
    packet->motor_chan5 = motor_chan5;
    packet->motor_chan6 = motor_chan6;
    packet->motor_chan7 = motor_chan7;
    packet->motor_chan8 = motor_chan8;
    packet->rc_chan1 = rc_chan1;
    packet->rc_chan2 = rc_chan2;
    packet->rc_chan3 = rc_chan3;
    packet->rc_chan4 = rc_chan4;
    packet->rc_chan5_8 = rc_chan5_8;
    packet->rc_chan9_12 = rc_chan9_12;
    packet->rc_chan13_16 = rc_chan13_16;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_EXAM_DATA, (const char *)packet, TTALINK_MSG_ID_EXAM_DATA_MIN_LEN, TTALINK_MSG_ID_EXAM_DATA_LEN, TTALINK_MSG_ID_EXAM_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_exam_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16)
{
    _ttalink_exam_data_send_buf(dst_addr, src_addr, msgbuf, chan, time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_exam_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t time, uint64_t uav_id, float target_pos_x, float target_pos_y, float target_pos_z, float current_pos_x, float current_pos_y, float current_pos_z, float rf_filter_alt, float GPS_RTK_pos_x, float GPS_RTK_pos_y, float GPS_RTK_pos_z, float baro_alt_meas, float rf_alt_meas, float fuse_hgt_meas, int16_t target_vel_x, int16_t target_vel_y, int16_t target_vel_z, int16_t current_vel_x, int16_t current_vel_y, int16_t current_vel_z, int16_t target_angle_x, int16_t target_angle_y, int16_t target_angle_z, int16_t current_angle_x, int16_t current_angle_y, int16_t current_angle_z, int16_t target_angle_rate_x, int16_t target_angle_rate_y, int16_t target_angle_rate_z, int16_t current_angle_rate_x, int16_t current_angle_rate_y, int16_t current_angle_rate_z, int16_t acc_roll_angle, int16_t acc_pitch_angle, int16_t mag_yaw_angle, int16_t rtk_yaw_angle, int16_t Reserve1, int16_t Reserve2, int16_t Reserve3, int16_t Reserve4, uint16_t flag, uint8_t motor_chan1, uint8_t motor_chan2, uint8_t motor_chan3, uint8_t motor_chan4, uint8_t motor_chan5, uint8_t motor_chan6, uint8_t motor_chan7, uint8_t motor_chan8, int8_t rc_chan1, int8_t rc_chan2, int8_t rc_chan3, int8_t rc_chan4, uint8_t rc_chan5_8, uint8_t rc_chan9_12, uint8_t rc_chan13_16)
{
    _ttalink_exam_data_send_buf(dst_addr, src_addr, msgbuf, chan, time, uav_id, target_pos_x, target_pos_y, target_pos_z, current_pos_x, current_pos_y, current_pos_z, rf_filter_alt, GPS_RTK_pos_x, GPS_RTK_pos_y, GPS_RTK_pos_z, baro_alt_meas, rf_alt_meas, fuse_hgt_meas, target_vel_x, target_vel_y, target_vel_z, current_vel_x, current_vel_y, current_vel_z, target_angle_x, target_angle_y, target_angle_z, current_angle_x, current_angle_y, current_angle_z, target_angle_rate_x, target_angle_rate_y, target_angle_rate_z, current_angle_rate_x, current_angle_rate_y, current_angle_rate_z, acc_roll_angle, acc_pitch_angle, mag_yaw_angle, rtk_yaw_angle, Reserve1, Reserve2, Reserve3, Reserve4, flag, motor_chan1, motor_chan2, motor_chan3, motor_chan4, motor_chan5, motor_chan6, motor_chan7, motor_chan8, rc_chan1, rc_chan2, rc_chan3, rc_chan4, rc_chan5_8, rc_chan9_12, rc_chan13_16, true);
}
#endif

#endif

// MESSAGE EXAM_DATA UNPACKING


/**
 * @brief Get field time from exam_data message
 *
 * @return  utc_time(MS)
 */
static inline uint64_t ttalink_exam_data_get_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field uav_id from exam_data message
 *
 * @return  uav_id.
 */
static inline uint64_t ttalink_exam_data_get_uav_id(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field target_pos_x from exam_data message
 *
 * @return  Relative home point 
 */
static inline float ttalink_exam_data_get_target_pos_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field target_pos_y from exam_data message
 *
 * @return Relative home point 
 */
static inline float ttalink_exam_data_get_target_pos_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  20);
}

/**
 * @brief Get field target_pos_z from exam_data message
 *
 * @return Relative home point 
 */
static inline float ttalink_exam_data_get_target_pos_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field current_pos_x from exam_data message
 *
 * @return  Relative home point
 */
static inline float ttalink_exam_data_get_current_pos_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field current_pos_y from exam_data message
 *
 * @return Relative home point 
 */
static inline float ttalink_exam_data_get_current_pos_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field current_pos_z from exam_data message
 *
 * @return Relative home point 
 */
static inline float ttalink_exam_data_get_current_pos_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field rf_filter_alt from exam_data message
 *
 * @return Height after filtering of radar 
 */
static inline float ttalink_exam_data_get_rf_filter_alt(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  40);
}

/**
 * @brief Get field GPS_RTK_pos_x from exam_data message
 *
 * @return  GPS-RTK Relative home point
 */
static inline float ttalink_exam_data_get_GPS_RTK_pos_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  44);
}

/**
 * @brief Get field GPS_RTK_pos_y from exam_data message
 *
 * @return  GPS-RTK Relative home point
 */
static inline float ttalink_exam_data_get_GPS_RTK_pos_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  48);
}

/**
 * @brief Get field GPS_RTK_pos_z from exam_data message
 *
 * @return  GPS-RTK Relative home point
 */
static inline float ttalink_exam_data_get_GPS_RTK_pos_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  52);
}

/**
 * @brief Get field baro_alt_meas from exam_data message
 *
 * @return  Barometer height
 */
static inline float ttalink_exam_data_get_baro_alt_meas(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  56);
}

/**
 * @brief Get field rf_alt_meas from exam_data message
 *
 * @return  Altimeter measurement 
 */
static inline float ttalink_exam_data_get_rf_alt_meas(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  60);
}

/**
 * @brief Get field fuse_hgt_meas from exam_data message
 *
 * @return  Fusion height measurement
 */
static inline float ttalink_exam_data_get_fuse_hgt_meas(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  64);
}

/**
 * @brief Get field target_vel_x from exam_data message
 *
 * @return  target_vel
 */
static inline int16_t ttalink_exam_data_get_target_vel_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  68);
}

/**
 * @brief Get field target_vel_y from exam_data message
 *
 * @return  target_vel
 */
static inline int16_t ttalink_exam_data_get_target_vel_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  70);
}

/**
 * @brief Get field target_vel_z from exam_data message
 *
 * @return  target_vel
 */
static inline int16_t ttalink_exam_data_get_target_vel_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  72);
}

/**
 * @brief Get field current_vel_x from exam_data message
 *
 * @return current_vel 
 */
static inline int16_t ttalink_exam_data_get_current_vel_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  74);
}

/**
 * @brief Get field current_vel_y from exam_data message
 *
 * @return current_vel 
 */
static inline int16_t ttalink_exam_data_get_current_vel_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  76);
}

/**
 * @brief Get field current_vel_z from exam_data message
 *
 * @return current_vel 
 */
static inline int16_t ttalink_exam_data_get_current_vel_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  78);
}

/**
 * @brief Get field target_angle_x from exam_data message
 *
 * @return  target_angle
 */
static inline int16_t ttalink_exam_data_get_target_angle_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  80);
}

/**
 * @brief Get field target_angle_y from exam_data message
 *
 * @return target_angle 
 */
static inline int16_t ttalink_exam_data_get_target_angle_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  82);
}

/**
 * @brief Get field target_angle_z from exam_data message
 *
 * @return  target_angle
 */
static inline int16_t ttalink_exam_data_get_target_angle_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  84);
}

/**
 * @brief Get field current_angle_x from exam_data message
 *
 * @return current_angle 
 */
static inline int16_t ttalink_exam_data_get_current_angle_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  86);
}

/**
 * @brief Get field current_angle_y from exam_data message
 *
 * @return current_angle 
 */
static inline int16_t ttalink_exam_data_get_current_angle_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  88);
}

/**
 * @brief Get field current_angle_z from exam_data message
 *
 * @return  current_angle
 */
static inline int16_t ttalink_exam_data_get_current_angle_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  90);
}

/**
 * @brief Get field target_angle_rate_x from exam_data message
 *
 * @return target_angle_rate 
 */
static inline int16_t ttalink_exam_data_get_target_angle_rate_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  92);
}

/**
 * @brief Get field target_angle_rate_y from exam_data message
 *
 * @return  target_angle_rate
 */
static inline int16_t ttalink_exam_data_get_target_angle_rate_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  94);
}

/**
 * @brief Get field target_angle_rate_z from exam_data message
 *
 * @return target_angle_rate 
 */
static inline int16_t ttalink_exam_data_get_target_angle_rate_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  96);
}

/**
 * @brief Get field current_angle_rate_x from exam_data message
 *
 * @return current_angle_rate 
 */
static inline int16_t ttalink_exam_data_get_current_angle_rate_x(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  98);
}

/**
 * @brief Get field current_angle_rate_y from exam_data message
 *
 * @return current_angle_rate 
 */
static inline int16_t ttalink_exam_data_get_current_angle_rate_y(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  100);
}

/**
 * @brief Get field current_angle_rate_z from exam_data message
 *
 * @return current_angle_rate 
 */
static inline int16_t ttalink_exam_data_get_current_angle_rate_z(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  102);
}

/**
 * @brief Get field acc_roll_angle from exam_data message
 *
 * @return Sensor roll angle 
 */
static inline int16_t ttalink_exam_data_get_acc_roll_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  104);
}

/**
 * @brief Get field acc_pitch_angle from exam_data message
 *
 * @return  Sensor pitch angle
 */
static inline int16_t ttalink_exam_data_get_acc_pitch_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  106);
}

/**
 * @brief Get field mag_yaw_angle from exam_data message
 *
 * @return  Magnetometer yaw angle
 */
static inline int16_t ttalink_exam_data_get_mag_yaw_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  108);
}

/**
 * @brief Get field rtk_yaw_angle from exam_data message
 *
 * @return  RTK yaw angle
 */
static inline int16_t ttalink_exam_data_get_rtk_yaw_angle(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  110);
}

/**
 * @brief Get field Reserve1 from exam_data message
 *
 * @return  
 */
static inline int16_t ttalink_exam_data_get_Reserve1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  112);
}

/**
 * @brief Get field Reserve2 from exam_data message
 *
 * @return  
 */
static inline int16_t ttalink_exam_data_get_Reserve2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  114);
}

/**
 * @brief Get field Reserve3 from exam_data message
 *
 * @return  
 */
static inline int16_t ttalink_exam_data_get_Reserve3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  116);
}

/**
 * @brief Get field Reserve4 from exam_data message
 *
 * @return  
 */
static inline int16_t ttalink_exam_data_get_Reserve4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int16_t(msg,  118);
}

/**
 * @brief Get field flag from exam_data message
 *
 * @return  
 */
static inline uint16_t ttalink_exam_data_get_flag(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  120);
}

/**
 * @brief Get field motor_chan1 from exam_data message
 *
 * @return motor[0,100]%
 */
static inline uint8_t ttalink_exam_data_get_motor_chan1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  122);
}

/**
 * @brief Get field motor_chan2 from exam_data message
 *
 * @return motor[0,100]% 
 */
static inline uint8_t ttalink_exam_data_get_motor_chan2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  123);
}

/**
 * @brief Get field motor_chan3 from exam_data message
 *
 * @return motor[0,100]% 
 */
static inline uint8_t ttalink_exam_data_get_motor_chan3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  124);
}

/**
 * @brief Get field motor_chan4 from exam_data message
 *
 * @return motor[0,100]% 
 */
static inline uint8_t ttalink_exam_data_get_motor_chan4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  125);
}

/**
 * @brief Get field motor_chan5 from exam_data message
 *
 * @return motor[0,100]%
 */
static inline uint8_t ttalink_exam_data_get_motor_chan5(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  126);
}

/**
 * @brief Get field motor_chan6 from exam_data message
 *
 * @return motor[0,100]%
 */
static inline uint8_t ttalink_exam_data_get_motor_chan6(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  127);
}

/**
 * @brief Get field motor_chan7 from exam_data message
 *
 * @return motor[0,100]%
 */
static inline uint8_t ttalink_exam_data_get_motor_chan7(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  128);
}

/**
 * @brief Get field motor_chan8 from exam_data message
 *
 * @return motor[0,100]%
 */
static inline uint8_t ttalink_exam_data_get_motor_chan8(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  129);
}

/**
 * @brief Get field rc_chan1 from exam_data message
 *
 * @return rc[0,100]%
 */
static inline int8_t ttalink_exam_data_get_rc_chan1(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  130);
}

/**
 * @brief Get field rc_chan2 from exam_data message
 *
 * @return rc[0,100]%
 */
static inline int8_t ttalink_exam_data_get_rc_chan2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  131);
}

/**
 * @brief Get field rc_chan3 from exam_data message
 *
 * @return rc[0,100]%
 */
static inline int8_t ttalink_exam_data_get_rc_chan3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  132);
}

/**
 * @brief Get field rc_chan4 from exam_data message
 *
 * @return rc[0,100]%
 */
static inline int8_t ttalink_exam_data_get_rc_chan4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int8_t(msg,  133);
}

/**
 * @brief Get field rc_chan5_8 from exam_data message
 *
 * @return  
 */
static inline uint8_t ttalink_exam_data_get_rc_chan5_8(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  134);
}

/**
 * @brief Get field rc_chan9_12 from exam_data message
 *
 * @return  
 */
static inline uint8_t ttalink_exam_data_get_rc_chan9_12(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  135);
}

/**
 * @brief Get field rc_chan13_16 from exam_data message
 *
 * @return  
 */
static inline uint8_t ttalink_exam_data_get_rc_chan13_16(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  136);
}

/**
 * @brief Decode a exam_data message into a struct
 *
 * @param msg The message to decode
 * @param exam_data C-struct to decode the message contents into
 */
static inline void ttalink_exam_data_decode(const ttalink_message_t* msg, ttalink_exam_data_t* exam_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    exam_data->time = ttalink_exam_data_get_time(msg);
    exam_data->uav_id = ttalink_exam_data_get_uav_id(msg);
    exam_data->target_pos_x = ttalink_exam_data_get_target_pos_x(msg);
    exam_data->target_pos_y = ttalink_exam_data_get_target_pos_y(msg);
    exam_data->target_pos_z = ttalink_exam_data_get_target_pos_z(msg);
    exam_data->current_pos_x = ttalink_exam_data_get_current_pos_x(msg);
    exam_data->current_pos_y = ttalink_exam_data_get_current_pos_y(msg);
    exam_data->current_pos_z = ttalink_exam_data_get_current_pos_z(msg);
    exam_data->rf_filter_alt = ttalink_exam_data_get_rf_filter_alt(msg);
    exam_data->GPS_RTK_pos_x = ttalink_exam_data_get_GPS_RTK_pos_x(msg);
    exam_data->GPS_RTK_pos_y = ttalink_exam_data_get_GPS_RTK_pos_y(msg);
    exam_data->GPS_RTK_pos_z = ttalink_exam_data_get_GPS_RTK_pos_z(msg);
    exam_data->baro_alt_meas = ttalink_exam_data_get_baro_alt_meas(msg);
    exam_data->rf_alt_meas = ttalink_exam_data_get_rf_alt_meas(msg);
    exam_data->fuse_hgt_meas = ttalink_exam_data_get_fuse_hgt_meas(msg);
    exam_data->target_vel_x = ttalink_exam_data_get_target_vel_x(msg);
    exam_data->target_vel_y = ttalink_exam_data_get_target_vel_y(msg);
    exam_data->target_vel_z = ttalink_exam_data_get_target_vel_z(msg);
    exam_data->current_vel_x = ttalink_exam_data_get_current_vel_x(msg);
    exam_data->current_vel_y = ttalink_exam_data_get_current_vel_y(msg);
    exam_data->current_vel_z = ttalink_exam_data_get_current_vel_z(msg);
    exam_data->target_angle_x = ttalink_exam_data_get_target_angle_x(msg);
    exam_data->target_angle_y = ttalink_exam_data_get_target_angle_y(msg);
    exam_data->target_angle_z = ttalink_exam_data_get_target_angle_z(msg);
    exam_data->current_angle_x = ttalink_exam_data_get_current_angle_x(msg);
    exam_data->current_angle_y = ttalink_exam_data_get_current_angle_y(msg);
    exam_data->current_angle_z = ttalink_exam_data_get_current_angle_z(msg);
    exam_data->target_angle_rate_x = ttalink_exam_data_get_target_angle_rate_x(msg);
    exam_data->target_angle_rate_y = ttalink_exam_data_get_target_angle_rate_y(msg);
    exam_data->target_angle_rate_z = ttalink_exam_data_get_target_angle_rate_z(msg);
    exam_data->current_angle_rate_x = ttalink_exam_data_get_current_angle_rate_x(msg);
    exam_data->current_angle_rate_y = ttalink_exam_data_get_current_angle_rate_y(msg);
    exam_data->current_angle_rate_z = ttalink_exam_data_get_current_angle_rate_z(msg);
    exam_data->acc_roll_angle = ttalink_exam_data_get_acc_roll_angle(msg);
    exam_data->acc_pitch_angle = ttalink_exam_data_get_acc_pitch_angle(msg);
    exam_data->mag_yaw_angle = ttalink_exam_data_get_mag_yaw_angle(msg);
    exam_data->rtk_yaw_angle = ttalink_exam_data_get_rtk_yaw_angle(msg);
    exam_data->Reserve1 = ttalink_exam_data_get_Reserve1(msg);
    exam_data->Reserve2 = ttalink_exam_data_get_Reserve2(msg);
    exam_data->Reserve3 = ttalink_exam_data_get_Reserve3(msg);
    exam_data->Reserve4 = ttalink_exam_data_get_Reserve4(msg);
    exam_data->flag = ttalink_exam_data_get_flag(msg);
    exam_data->motor_chan1 = ttalink_exam_data_get_motor_chan1(msg);
    exam_data->motor_chan2 = ttalink_exam_data_get_motor_chan2(msg);
    exam_data->motor_chan3 = ttalink_exam_data_get_motor_chan3(msg);
    exam_data->motor_chan4 = ttalink_exam_data_get_motor_chan4(msg);
    exam_data->motor_chan5 = ttalink_exam_data_get_motor_chan5(msg);
    exam_data->motor_chan6 = ttalink_exam_data_get_motor_chan6(msg);
    exam_data->motor_chan7 = ttalink_exam_data_get_motor_chan7(msg);
    exam_data->motor_chan8 = ttalink_exam_data_get_motor_chan8(msg);
    exam_data->rc_chan1 = ttalink_exam_data_get_rc_chan1(msg);
    exam_data->rc_chan2 = ttalink_exam_data_get_rc_chan2(msg);
    exam_data->rc_chan3 = ttalink_exam_data_get_rc_chan3(msg);
    exam_data->rc_chan4 = ttalink_exam_data_get_rc_chan4(msg);
    exam_data->rc_chan5_8 = ttalink_exam_data_get_rc_chan5_8(msg);
    exam_data->rc_chan9_12 = ttalink_exam_data_get_rc_chan9_12(msg);
    exam_data->rc_chan13_16 = ttalink_exam_data_get_rc_chan13_16(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_EXAM_DATA_LEN? msg->len : TTALINK_MSG_ID_EXAM_DATA_LEN;
        memset(exam_data, 0, TTALINK_MSG_ID_EXAM_DATA_LEN);
    memcpy(exam_data, _TTA_PAYLOAD(msg), len);
#endif
}
