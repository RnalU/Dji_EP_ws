#pragma once
// MESSAGE RTK_SENSORS_P20_DATA PACKING

#define TTALINK_MSG_ID_RTK_SENSORS_P20_DATA 2178

TTAPACKED(
typedef struct __ttalink_rtk_sensors_p20_data_t {
 double gnss_latitude; /*<  .*/
 double gnss_longitude; /*<  .*/
 float gnss_altitude; /*<  .*/
 uint8_t stas_number_usedinsolution; /*<  使用的卫星数.*/
 uint8_t mode_type; /*<  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.*/
 uint8_t sol_status; /*<  .*/
 uint8_t pos_type; /*<  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.*/
 float hor_spd; /*<  地速.*/
 float trk_gnd; /*< 地速与北向夹角.*/
 float Magnetic_Variation; /*< 磁偏角.*/
 float Declination; /*< 磁偏角方向.*/
 uint16_t gps_time_week; /*<  整周.*/
 uint32_t gps_time_ms; /*<  周内秒.*/
 uint32_t utc_year; /*<  UTC年.*/
 uint8_t utc_month; /*<  UTC月.*/
 uint8_t utc_day; /*<  UTC日.*/
 uint8_t utc_hour; /*<  UTC时.*/
 uint8_t utc_min; /*<  UTC分.*/
 uint32_t utc_ms; /*<  UTC毫秒.*/
 uint32_t updateTime; /*<  .*/
 uint32_t vel_updateTime; /*<  .*/
 uint32_t pos_updateTime; /*<  .*/
 uint32_t dop_updateTime; /*<  .*/
 uint32_t heading_updateTime; /*<  .*/
 uint8_t heading_sol_status; /*<  .*/
 uint8_t heading_pos_type; /*<  .*/
 float GDOP; /*<  几何精度因子.*/
 float PDOP; /*<  位置精度.*/
 float HDOP; /*<  水平精度.*/
 double kalman_latitude; /*<  组合纬度.*/
 double kalman_longitude; /*<  组合经度.*/
 float kalman_altitude; /*<  组合高度.*/
 float lat_std_deviation; /*<  纬度误差.*/
 float lon_std_deviation; /*<  经度误差.*/
 float alt_std_deviation; /*<  高度误差.*/
 float kalman_velE; /*<  东向速度.*/
 float kalman_velN; /*<  北向速度.*/
 float kalman_velU; /*<  天向速度.*/
 float vel_std_g; /*<  速度水平误差.*/
 float kalman_yaw; /*<  航向.*/
 float kalman_pitch; /*<  俯仰.*/
 float kalman_roll; /*<  横滚.*/
 float yaw_std_deviation; /*<  航向误差.*/
 float pitch_std_deviation; /*<  俯仰误差.*/
 float roll_std_deviation; /*<  横滚误差.*/
 float base_line; /*<  基线距离.*/
 uint32_t rtk_version; /*<  .*/
 float param2; /*<  .*/
 float param3; /*<  .*/
 float param4; /*<  .*/
}) ttalink_rtk_sensors_p20_data_t;

#define TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN 184
#define TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN 184
#define TTALINK_MSG_ID_2178_LEN 184
#define TTALINK_MSG_ID_2178_MIN_LEN 184

#define TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC 241
#define TTALINK_MSG_ID_2178_CRC 241



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_RTK_SENSORS_P20_DATA { \
    2178, \
    "RTK_SENSORS_P20_DATA", \
    50, \
    {  { "gnss_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rtk_sensors_p20_data_t, gnss_latitude) }, \
         { "gnss_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rtk_sensors_p20_data_t, gnss_longitude) }, \
         { "gnss_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rtk_sensors_p20_data_t, gnss_altitude) }, \
         { "stas_number_usedinsolution", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_rtk_sensors_p20_data_t, stas_number_usedinsolution) }, \
         { "mode_type", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_rtk_sensors_p20_data_t, mode_type) }, \
         { "sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 22, offsetof(ttalink_rtk_sensors_p20_data_t, sol_status) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 23, offsetof(ttalink_rtk_sensors_p20_data_t, pos_type) }, \
         { "hor_spd", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_rtk_sensors_p20_data_t, hor_spd) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_rtk_sensors_p20_data_t, trk_gnd) }, \
         { "Magnetic_Variation", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_rtk_sensors_p20_data_t, Magnetic_Variation) }, \
         { "Declination", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_rtk_sensors_p20_data_t, Declination) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 40, offsetof(ttalink_rtk_sensors_p20_data_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 42, offsetof(ttalink_rtk_sensors_p20_data_t, gps_time_ms) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 46, offsetof(ttalink_rtk_sensors_p20_data_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 50, offsetof(ttalink_rtk_sensors_p20_data_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 51, offsetof(ttalink_rtk_sensors_p20_data_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 52, offsetof(ttalink_rtk_sensors_p20_data_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 53, offsetof(ttalink_rtk_sensors_p20_data_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 54, offsetof(ttalink_rtk_sensors_p20_data_t, utc_ms) }, \
         { "updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 58, offsetof(ttalink_rtk_sensors_p20_data_t, updateTime) }, \
         { "vel_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 62, offsetof(ttalink_rtk_sensors_p20_data_t, vel_updateTime) }, \
         { "pos_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 66, offsetof(ttalink_rtk_sensors_p20_data_t, pos_updateTime) }, \
         { "dop_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 70, offsetof(ttalink_rtk_sensors_p20_data_t, dop_updateTime) }, \
         { "heading_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 74, offsetof(ttalink_rtk_sensors_p20_data_t, heading_updateTime) }, \
         { "heading_sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 78, offsetof(ttalink_rtk_sensors_p20_data_t, heading_sol_status) }, \
         { "heading_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 79, offsetof(ttalink_rtk_sensors_p20_data_t, heading_pos_type) }, \
         { "GDOP", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_rtk_sensors_p20_data_t, GDOP) }, \
         { "PDOP", NULL, TTALINK_TYPE_FLOAT, 0, 84, offsetof(ttalink_rtk_sensors_p20_data_t, PDOP) }, \
         { "HDOP", NULL, TTALINK_TYPE_FLOAT, 0, 88, offsetof(ttalink_rtk_sensors_p20_data_t, HDOP) }, \
         { "kalman_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 92, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_latitude) }, \
         { "kalman_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 100, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_longitude) }, \
         { "kalman_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 108, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_altitude) }, \
         { "lat_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 112, offsetof(ttalink_rtk_sensors_p20_data_t, lat_std_deviation) }, \
         { "lon_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 116, offsetof(ttalink_rtk_sensors_p20_data_t, lon_std_deviation) }, \
         { "alt_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 120, offsetof(ttalink_rtk_sensors_p20_data_t, alt_std_deviation) }, \
         { "kalman_velE", NULL, TTALINK_TYPE_FLOAT, 0, 124, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_velE) }, \
         { "kalman_velN", NULL, TTALINK_TYPE_FLOAT, 0, 128, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_velN) }, \
         { "kalman_velU", NULL, TTALINK_TYPE_FLOAT, 0, 132, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_velU) }, \
         { "vel_std_g", NULL, TTALINK_TYPE_FLOAT, 0, 136, offsetof(ttalink_rtk_sensors_p20_data_t, vel_std_g) }, \
         { "kalman_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 140, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_yaw) }, \
         { "kalman_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 144, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_pitch) }, \
         { "kalman_roll", NULL, TTALINK_TYPE_FLOAT, 0, 148, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_roll) }, \
         { "yaw_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 152, offsetof(ttalink_rtk_sensors_p20_data_t, yaw_std_deviation) }, \
         { "pitch_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 156, offsetof(ttalink_rtk_sensors_p20_data_t, pitch_std_deviation) }, \
         { "roll_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 160, offsetof(ttalink_rtk_sensors_p20_data_t, roll_std_deviation) }, \
         { "base_line", NULL, TTALINK_TYPE_FLOAT, 0, 164, offsetof(ttalink_rtk_sensors_p20_data_t, base_line) }, \
         { "rtk_version", NULL, TTALINK_TYPE_UINT32_T, 0, 168, offsetof(ttalink_rtk_sensors_p20_data_t, rtk_version) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 172, offsetof(ttalink_rtk_sensors_p20_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 176, offsetof(ttalink_rtk_sensors_p20_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 180, offsetof(ttalink_rtk_sensors_p20_data_t, param4) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_RTK_SENSORS_P20_DATA { \
    "RTK_SENSORS_P20_DATA", \
    50, \
    {  { "gnss_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 0, offsetof(ttalink_rtk_sensors_p20_data_t, gnss_latitude) }, \
         { "gnss_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 8, offsetof(ttalink_rtk_sensors_p20_data_t, gnss_longitude) }, \
         { "gnss_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 16, offsetof(ttalink_rtk_sensors_p20_data_t, gnss_altitude) }, \
         { "stas_number_usedinsolution", NULL, TTALINK_TYPE_UINT8_T, 0, 20, offsetof(ttalink_rtk_sensors_p20_data_t, stas_number_usedinsolution) }, \
         { "mode_type", NULL, TTALINK_TYPE_UINT8_T, 0, 21, offsetof(ttalink_rtk_sensors_p20_data_t, mode_type) }, \
         { "sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 22, offsetof(ttalink_rtk_sensors_p20_data_t, sol_status) }, \
         { "pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 23, offsetof(ttalink_rtk_sensors_p20_data_t, pos_type) }, \
         { "hor_spd", NULL, TTALINK_TYPE_FLOAT, 0, 24, offsetof(ttalink_rtk_sensors_p20_data_t, hor_spd) }, \
         { "trk_gnd", NULL, TTALINK_TYPE_FLOAT, 0, 28, offsetof(ttalink_rtk_sensors_p20_data_t, trk_gnd) }, \
         { "Magnetic_Variation", NULL, TTALINK_TYPE_FLOAT, 0, 32, offsetof(ttalink_rtk_sensors_p20_data_t, Magnetic_Variation) }, \
         { "Declination", NULL, TTALINK_TYPE_FLOAT, 0, 36, offsetof(ttalink_rtk_sensors_p20_data_t, Declination) }, \
         { "gps_time_week", NULL, TTALINK_TYPE_UINT16_T, 0, 40, offsetof(ttalink_rtk_sensors_p20_data_t, gps_time_week) }, \
         { "gps_time_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 42, offsetof(ttalink_rtk_sensors_p20_data_t, gps_time_ms) }, \
         { "utc_year", NULL, TTALINK_TYPE_UINT32_T, 0, 46, offsetof(ttalink_rtk_sensors_p20_data_t, utc_year) }, \
         { "utc_month", NULL, TTALINK_TYPE_UINT8_T, 0, 50, offsetof(ttalink_rtk_sensors_p20_data_t, utc_month) }, \
         { "utc_day", NULL, TTALINK_TYPE_UINT8_T, 0, 51, offsetof(ttalink_rtk_sensors_p20_data_t, utc_day) }, \
         { "utc_hour", NULL, TTALINK_TYPE_UINT8_T, 0, 52, offsetof(ttalink_rtk_sensors_p20_data_t, utc_hour) }, \
         { "utc_min", NULL, TTALINK_TYPE_UINT8_T, 0, 53, offsetof(ttalink_rtk_sensors_p20_data_t, utc_min) }, \
         { "utc_ms", NULL, TTALINK_TYPE_UINT32_T, 0, 54, offsetof(ttalink_rtk_sensors_p20_data_t, utc_ms) }, \
         { "updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 58, offsetof(ttalink_rtk_sensors_p20_data_t, updateTime) }, \
         { "vel_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 62, offsetof(ttalink_rtk_sensors_p20_data_t, vel_updateTime) }, \
         { "pos_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 66, offsetof(ttalink_rtk_sensors_p20_data_t, pos_updateTime) }, \
         { "dop_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 70, offsetof(ttalink_rtk_sensors_p20_data_t, dop_updateTime) }, \
         { "heading_updateTime", NULL, TTALINK_TYPE_UINT32_T, 0, 74, offsetof(ttalink_rtk_sensors_p20_data_t, heading_updateTime) }, \
         { "heading_sol_status", NULL, TTALINK_TYPE_UINT8_T, 0, 78, offsetof(ttalink_rtk_sensors_p20_data_t, heading_sol_status) }, \
         { "heading_pos_type", NULL, TTALINK_TYPE_UINT8_T, 0, 79, offsetof(ttalink_rtk_sensors_p20_data_t, heading_pos_type) }, \
         { "GDOP", NULL, TTALINK_TYPE_FLOAT, 0, 80, offsetof(ttalink_rtk_sensors_p20_data_t, GDOP) }, \
         { "PDOP", NULL, TTALINK_TYPE_FLOAT, 0, 84, offsetof(ttalink_rtk_sensors_p20_data_t, PDOP) }, \
         { "HDOP", NULL, TTALINK_TYPE_FLOAT, 0, 88, offsetof(ttalink_rtk_sensors_p20_data_t, HDOP) }, \
         { "kalman_latitude", NULL, TTALINK_TYPE_DOUBLE, 0, 92, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_latitude) }, \
         { "kalman_longitude", NULL, TTALINK_TYPE_DOUBLE, 0, 100, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_longitude) }, \
         { "kalman_altitude", NULL, TTALINK_TYPE_FLOAT, 0, 108, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_altitude) }, \
         { "lat_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 112, offsetof(ttalink_rtk_sensors_p20_data_t, lat_std_deviation) }, \
         { "lon_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 116, offsetof(ttalink_rtk_sensors_p20_data_t, lon_std_deviation) }, \
         { "alt_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 120, offsetof(ttalink_rtk_sensors_p20_data_t, alt_std_deviation) }, \
         { "kalman_velE", NULL, TTALINK_TYPE_FLOAT, 0, 124, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_velE) }, \
         { "kalman_velN", NULL, TTALINK_TYPE_FLOAT, 0, 128, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_velN) }, \
         { "kalman_velU", NULL, TTALINK_TYPE_FLOAT, 0, 132, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_velU) }, \
         { "vel_std_g", NULL, TTALINK_TYPE_FLOAT, 0, 136, offsetof(ttalink_rtk_sensors_p20_data_t, vel_std_g) }, \
         { "kalman_yaw", NULL, TTALINK_TYPE_FLOAT, 0, 140, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_yaw) }, \
         { "kalman_pitch", NULL, TTALINK_TYPE_FLOAT, 0, 144, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_pitch) }, \
         { "kalman_roll", NULL, TTALINK_TYPE_FLOAT, 0, 148, offsetof(ttalink_rtk_sensors_p20_data_t, kalman_roll) }, \
         { "yaw_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 152, offsetof(ttalink_rtk_sensors_p20_data_t, yaw_std_deviation) }, \
         { "pitch_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 156, offsetof(ttalink_rtk_sensors_p20_data_t, pitch_std_deviation) }, \
         { "roll_std_deviation", NULL, TTALINK_TYPE_FLOAT, 0, 160, offsetof(ttalink_rtk_sensors_p20_data_t, roll_std_deviation) }, \
         { "base_line", NULL, TTALINK_TYPE_FLOAT, 0, 164, offsetof(ttalink_rtk_sensors_p20_data_t, base_line) }, \
         { "rtk_version", NULL, TTALINK_TYPE_UINT32_T, 0, 168, offsetof(ttalink_rtk_sensors_p20_data_t, rtk_version) }, \
         { "param2", NULL, TTALINK_TYPE_FLOAT, 0, 172, offsetof(ttalink_rtk_sensors_p20_data_t, param2) }, \
         { "param3", NULL, TTALINK_TYPE_FLOAT, 0, 176, offsetof(ttalink_rtk_sensors_p20_data_t, param3) }, \
         { "param4", NULL, TTALINK_TYPE_FLOAT, 0, 180, offsetof(ttalink_rtk_sensors_p20_data_t, param4) }, \
         } \
}
#endif


static inline uint16_t _ttalink_rtk_sensors_p20_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN];
    _tta_put_double(buf, 0, gnss_latitude);
    _tta_put_double(buf, 8, gnss_longitude);
    _tta_put_float(buf, 16, gnss_altitude);
    _tta_put_uint8_t(buf, 20, stas_number_usedinsolution);
    _tta_put_uint8_t(buf, 21, mode_type);
    _tta_put_uint8_t(buf, 22, sol_status);
    _tta_put_uint8_t(buf, 23, pos_type);
    _tta_put_float(buf, 24, hor_spd);
    _tta_put_float(buf, 28, trk_gnd);
    _tta_put_float(buf, 32, Magnetic_Variation);
    _tta_put_float(buf, 36, Declination);
    _tta_put_uint16_t(buf, 40, gps_time_week);
    _tta_put_uint32_t(buf, 42, gps_time_ms);
    _tta_put_uint32_t(buf, 46, utc_year);
    _tta_put_uint8_t(buf, 50, utc_month);
    _tta_put_uint8_t(buf, 51, utc_day);
    _tta_put_uint8_t(buf, 52, utc_hour);
    _tta_put_uint8_t(buf, 53, utc_min);
    _tta_put_uint32_t(buf, 54, utc_ms);
    _tta_put_uint32_t(buf, 58, updateTime);
    _tta_put_uint32_t(buf, 62, vel_updateTime);
    _tta_put_uint32_t(buf, 66, pos_updateTime);
    _tta_put_uint32_t(buf, 70, dop_updateTime);
    _tta_put_uint32_t(buf, 74, heading_updateTime);
    _tta_put_uint8_t(buf, 78, heading_sol_status);
    _tta_put_uint8_t(buf, 79, heading_pos_type);
    _tta_put_float(buf, 80, GDOP);
    _tta_put_float(buf, 84, PDOP);
    _tta_put_float(buf, 88, HDOP);
    _tta_put_double(buf, 92, kalman_latitude);
    _tta_put_double(buf, 100, kalman_longitude);
    _tta_put_float(buf, 108, kalman_altitude);
    _tta_put_float(buf, 112, lat_std_deviation);
    _tta_put_float(buf, 116, lon_std_deviation);
    _tta_put_float(buf, 120, alt_std_deviation);
    _tta_put_float(buf, 124, kalman_velE);
    _tta_put_float(buf, 128, kalman_velN);
    _tta_put_float(buf, 132, kalman_velU);
    _tta_put_float(buf, 136, vel_std_g);
    _tta_put_float(buf, 140, kalman_yaw);
    _tta_put_float(buf, 144, kalman_pitch);
    _tta_put_float(buf, 148, kalman_roll);
    _tta_put_float(buf, 152, yaw_std_deviation);
    _tta_put_float(buf, 156, pitch_std_deviation);
    _tta_put_float(buf, 160, roll_std_deviation);
    _tta_put_float(buf, 164, base_line);
    _tta_put_uint32_t(buf, 168, rtk_version);
    _tta_put_float(buf, 172, param2);
    _tta_put_float(buf, 176, param3);
    _tta_put_float(buf, 180, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN);
#else
    ttalink_rtk_sensors_p20_data_t packet;
    packet.gnss_latitude = gnss_latitude;
    packet.gnss_longitude = gnss_longitude;
    packet.gnss_altitude = gnss_altitude;
    packet.stas_number_usedinsolution = stas_number_usedinsolution;
    packet.mode_type = mode_type;
    packet.sol_status = sol_status;
    packet.pos_type = pos_type;
    packet.hor_spd = hor_spd;
    packet.trk_gnd = trk_gnd;
    packet.Magnetic_Variation = Magnetic_Variation;
    packet.Declination = Declination;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.updateTime = updateTime;
    packet.vel_updateTime = vel_updateTime;
    packet.pos_updateTime = pos_updateTime;
    packet.dop_updateTime = dop_updateTime;
    packet.heading_updateTime = heading_updateTime;
    packet.heading_sol_status = heading_sol_status;
    packet.heading_pos_type = heading_pos_type;
    packet.GDOP = GDOP;
    packet.PDOP = PDOP;
    packet.HDOP = HDOP;
    packet.kalman_latitude = kalman_latitude;
    packet.kalman_longitude = kalman_longitude;
    packet.kalman_altitude = kalman_altitude;
    packet.lat_std_deviation = lat_std_deviation;
    packet.lon_std_deviation = lon_std_deviation;
    packet.alt_std_deviation = alt_std_deviation;
    packet.kalman_velE = kalman_velE;
    packet.kalman_velN = kalman_velN;
    packet.kalman_velU = kalman_velU;
    packet.vel_std_g = vel_std_g;
    packet.kalman_yaw = kalman_yaw;
    packet.kalman_pitch = kalman_pitch;
    packet.kalman_roll = kalman_roll;
    packet.yaw_std_deviation = yaw_std_deviation;
    packet.pitch_std_deviation = pitch_std_deviation;
    packet.roll_std_deviation = roll_std_deviation;
    packet.base_line = base_line;
    packet.rtk_version = rtk_version;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RTK_SENSORS_P20_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rtk_sensors_p20_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param gnss_latitude  .
 * @param gnss_longitude  .
 * @param gnss_altitude  .
 * @param stas_number_usedinsolution  使用的卫星数.
 * @param mode_type  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.
 * @param sol_status  .
 * @param pos_type  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param Magnetic_Variation 磁偏角.
 * @param Declination 磁偏角方向.
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param GDOP  几何精度因子.
 * @param PDOP  位置精度.
 * @param HDOP  水平精度.
 * @param kalman_latitude  组合纬度.
 * @param kalman_longitude  组合经度.
 * @param kalman_altitude  组合高度.
 * @param lat_std_deviation  纬度误差.
 * @param lon_std_deviation  经度误差.
 * @param alt_std_deviation  高度误差.
 * @param kalman_velE  东向速度.
 * @param kalman_velN  北向速度.
 * @param kalman_velU  天向速度.
 * @param vel_std_g  速度水平误差.
 * @param kalman_yaw  航向.
 * @param kalman_pitch  俯仰.
 * @param kalman_roll  横滚.
 * @param yaw_std_deviation  航向误差.
 * @param pitch_std_deviation  俯仰误差.
 * @param roll_std_deviation  横滚误差.
 * @param base_line  基线距离.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4)
{
    return _ttalink_rtk_sensors_p20_data_pack(dst_addr, src_addr, msg,  gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, false);
}

/**
 * @brief Pack a rtk_sensors_p20_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param gnss_latitude  .
 * @param gnss_longitude  .
 * @param gnss_altitude  .
 * @param stas_number_usedinsolution  使用的卫星数.
 * @param mode_type  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.
 * @param sol_status  .
 * @param pos_type  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param Magnetic_Variation 磁偏角.
 * @param Declination 磁偏角方向.
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param GDOP  几何精度因子.
 * @param PDOP  位置精度.
 * @param HDOP  水平精度.
 * @param kalman_latitude  组合纬度.
 * @param kalman_longitude  组合经度.
 * @param kalman_altitude  组合高度.
 * @param lat_std_deviation  纬度误差.
 * @param lon_std_deviation  经度误差.
 * @param alt_std_deviation  高度误差.
 * @param kalman_velE  东向速度.
 * @param kalman_velN  北向速度.
 * @param kalman_velU  天向速度.
 * @param vel_std_g  速度水平误差.
 * @param kalman_yaw  航向.
 * @param kalman_pitch  俯仰.
 * @param kalman_roll  横滚.
 * @param yaw_std_deviation  航向误差.
 * @param pitch_std_deviation  俯仰误差.
 * @param roll_std_deviation  横滚误差.
 * @param base_line  基线距离.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4)
{
    return _ttalink_rtk_sensors_p20_data_pack(dst_addr, src_addr, msg,  gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, true);
}


static inline uint16_t _ttalink_rtk_sensors_p20_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double gnss_latitude,double gnss_longitude,float gnss_altitude,uint8_t stas_number_usedinsolution,uint8_t mode_type,uint8_t sol_status,uint8_t pos_type,float hor_spd,float trk_gnd,float Magnetic_Variation,float Declination,uint16_t gps_time_week,uint32_t gps_time_ms,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,float GDOP,float PDOP,float HDOP,double kalman_latitude,double kalman_longitude,float kalman_altitude,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float kalman_velE,float kalman_velN,float kalman_velU,float vel_std_g,float kalman_yaw,float kalman_pitch,float kalman_roll,float yaw_std_deviation,float pitch_std_deviation,float roll_std_deviation,float base_line,uint32_t rtk_version,float param2,float param3,float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN];
    _tta_put_double(buf, 0, gnss_latitude);
    _tta_put_double(buf, 8, gnss_longitude);
    _tta_put_float(buf, 16, gnss_altitude);
    _tta_put_uint8_t(buf, 20, stas_number_usedinsolution);
    _tta_put_uint8_t(buf, 21, mode_type);
    _tta_put_uint8_t(buf, 22, sol_status);
    _tta_put_uint8_t(buf, 23, pos_type);
    _tta_put_float(buf, 24, hor_spd);
    _tta_put_float(buf, 28, trk_gnd);
    _tta_put_float(buf, 32, Magnetic_Variation);
    _tta_put_float(buf, 36, Declination);
    _tta_put_uint16_t(buf, 40, gps_time_week);
    _tta_put_uint32_t(buf, 42, gps_time_ms);
    _tta_put_uint32_t(buf, 46, utc_year);
    _tta_put_uint8_t(buf, 50, utc_month);
    _tta_put_uint8_t(buf, 51, utc_day);
    _tta_put_uint8_t(buf, 52, utc_hour);
    _tta_put_uint8_t(buf, 53, utc_min);
    _tta_put_uint32_t(buf, 54, utc_ms);
    _tta_put_uint32_t(buf, 58, updateTime);
    _tta_put_uint32_t(buf, 62, vel_updateTime);
    _tta_put_uint32_t(buf, 66, pos_updateTime);
    _tta_put_uint32_t(buf, 70, dop_updateTime);
    _tta_put_uint32_t(buf, 74, heading_updateTime);
    _tta_put_uint8_t(buf, 78, heading_sol_status);
    _tta_put_uint8_t(buf, 79, heading_pos_type);
    _tta_put_float(buf, 80, GDOP);
    _tta_put_float(buf, 84, PDOP);
    _tta_put_float(buf, 88, HDOP);
    _tta_put_double(buf, 92, kalman_latitude);
    _tta_put_double(buf, 100, kalman_longitude);
    _tta_put_float(buf, 108, kalman_altitude);
    _tta_put_float(buf, 112, lat_std_deviation);
    _tta_put_float(buf, 116, lon_std_deviation);
    _tta_put_float(buf, 120, alt_std_deviation);
    _tta_put_float(buf, 124, kalman_velE);
    _tta_put_float(buf, 128, kalman_velN);
    _tta_put_float(buf, 132, kalman_velU);
    _tta_put_float(buf, 136, vel_std_g);
    _tta_put_float(buf, 140, kalman_yaw);
    _tta_put_float(buf, 144, kalman_pitch);
    _tta_put_float(buf, 148, kalman_roll);
    _tta_put_float(buf, 152, yaw_std_deviation);
    _tta_put_float(buf, 156, pitch_std_deviation);
    _tta_put_float(buf, 160, roll_std_deviation);
    _tta_put_float(buf, 164, base_line);
    _tta_put_uint32_t(buf, 168, rtk_version);
    _tta_put_float(buf, 172, param2);
    _tta_put_float(buf, 176, param3);
    _tta_put_float(buf, 180, param4);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN);
#else
    ttalink_rtk_sensors_p20_data_t packet;
    packet.gnss_latitude = gnss_latitude;
    packet.gnss_longitude = gnss_longitude;
    packet.gnss_altitude = gnss_altitude;
    packet.stas_number_usedinsolution = stas_number_usedinsolution;
    packet.mode_type = mode_type;
    packet.sol_status = sol_status;
    packet.pos_type = pos_type;
    packet.hor_spd = hor_spd;
    packet.trk_gnd = trk_gnd;
    packet.Magnetic_Variation = Magnetic_Variation;
    packet.Declination = Declination;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.updateTime = updateTime;
    packet.vel_updateTime = vel_updateTime;
    packet.pos_updateTime = pos_updateTime;
    packet.dop_updateTime = dop_updateTime;
    packet.heading_updateTime = heading_updateTime;
    packet.heading_sol_status = heading_sol_status;
    packet.heading_pos_type = heading_pos_type;
    packet.GDOP = GDOP;
    packet.PDOP = PDOP;
    packet.HDOP = HDOP;
    packet.kalman_latitude = kalman_latitude;
    packet.kalman_longitude = kalman_longitude;
    packet.kalman_altitude = kalman_altitude;
    packet.lat_std_deviation = lat_std_deviation;
    packet.lon_std_deviation = lon_std_deviation;
    packet.alt_std_deviation = alt_std_deviation;
    packet.kalman_velE = kalman_velE;
    packet.kalman_velN = kalman_velN;
    packet.kalman_velU = kalman_velU;
    packet.vel_std_g = vel_std_g;
    packet.kalman_yaw = kalman_yaw;
    packet.kalman_pitch = kalman_pitch;
    packet.kalman_roll = kalman_roll;
    packet.yaw_std_deviation = yaw_std_deviation;
    packet.pitch_std_deviation = pitch_std_deviation;
    packet.roll_std_deviation = roll_std_deviation;
    packet.base_line = base_line;
    packet.rtk_version = rtk_version;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_RTK_SENSORS_P20_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC, nocrc);
}

/**
 * @brief Pack a rtk_sensors_p20_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param gnss_latitude  .
 * @param gnss_longitude  .
 * @param gnss_altitude  .
 * @param stas_number_usedinsolution  使用的卫星数.
 * @param mode_type  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.
 * @param sol_status  .
 * @param pos_type  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param Magnetic_Variation 磁偏角.
 * @param Declination 磁偏角方向.
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param GDOP  几何精度因子.
 * @param PDOP  位置精度.
 * @param HDOP  水平精度.
 * @param kalman_latitude  组合纬度.
 * @param kalman_longitude  组合经度.
 * @param kalman_altitude  组合高度.
 * @param lat_std_deviation  纬度误差.
 * @param lon_std_deviation  经度误差.
 * @param alt_std_deviation  高度误差.
 * @param kalman_velE  东向速度.
 * @param kalman_velN  北向速度.
 * @param kalman_velU  天向速度.
 * @param vel_std_g  速度水平误差.
 * @param kalman_yaw  航向.
 * @param kalman_pitch  俯仰.
 * @param kalman_roll  横滚.
 * @param yaw_std_deviation  航向误差.
 * @param pitch_std_deviation  俯仰误差.
 * @param roll_std_deviation  横滚误差.
 * @param base_line  基线距离.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double gnss_latitude,double gnss_longitude,float gnss_altitude,uint8_t stas_number_usedinsolution,uint8_t mode_type,uint8_t sol_status,uint8_t pos_type,float hor_spd,float trk_gnd,float Magnetic_Variation,float Declination,uint16_t gps_time_week,uint32_t gps_time_ms,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,float GDOP,float PDOP,float HDOP,double kalman_latitude,double kalman_longitude,float kalman_altitude,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float kalman_velE,float kalman_velN,float kalman_velU,float vel_std_g,float kalman_yaw,float kalman_pitch,float kalman_roll,float yaw_std_deviation,float pitch_std_deviation,float roll_std_deviation,float base_line,uint32_t rtk_version,float param2,float param3,float param4)
{
    return _ttalink_rtk_sensors_p20_data_pack_chan(dst_addr, src_addr, chan, msg,  gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, false);
}

/**
 * @brief Pack a rtk_sensors_p20_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param gnss_latitude  .
 * @param gnss_longitude  .
 * @param gnss_altitude  .
 * @param stas_number_usedinsolution  使用的卫星数.
 * @param mode_type  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.
 * @param sol_status  .
 * @param pos_type  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param Magnetic_Variation 磁偏角.
 * @param Declination 磁偏角方向.
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param GDOP  几何精度因子.
 * @param PDOP  位置精度.
 * @param HDOP  水平精度.
 * @param kalman_latitude  组合纬度.
 * @param kalman_longitude  组合经度.
 * @param kalman_altitude  组合高度.
 * @param lat_std_deviation  纬度误差.
 * @param lon_std_deviation  经度误差.
 * @param alt_std_deviation  高度误差.
 * @param kalman_velE  东向速度.
 * @param kalman_velN  北向速度.
 * @param kalman_velU  天向速度.
 * @param vel_std_g  速度水平误差.
 * @param kalman_yaw  航向.
 * @param kalman_pitch  俯仰.
 * @param kalman_roll  横滚.
 * @param yaw_std_deviation  航向误差.
 * @param pitch_std_deviation  俯仰误差.
 * @param roll_std_deviation  横滚误差.
 * @param base_line  基线距离.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   double gnss_latitude,double gnss_longitude,float gnss_altitude,uint8_t stas_number_usedinsolution,uint8_t mode_type,uint8_t sol_status,uint8_t pos_type,float hor_spd,float trk_gnd,float Magnetic_Variation,float Declination,uint16_t gps_time_week,uint32_t gps_time_ms,uint32_t utc_year,uint8_t utc_month,uint8_t utc_day,uint8_t utc_hour,uint8_t utc_min,uint32_t utc_ms,uint32_t updateTime,uint32_t vel_updateTime,uint32_t pos_updateTime,uint32_t dop_updateTime,uint32_t heading_updateTime,uint8_t heading_sol_status,uint8_t heading_pos_type,float GDOP,float PDOP,float HDOP,double kalman_latitude,double kalman_longitude,float kalman_altitude,float lat_std_deviation,float lon_std_deviation,float alt_std_deviation,float kalman_velE,float kalman_velN,float kalman_velU,float vel_std_g,float kalman_yaw,float kalman_pitch,float kalman_roll,float yaw_std_deviation,float pitch_std_deviation,float roll_std_deviation,float base_line,uint32_t rtk_version,float param2,float param3,float param4)
{
    return _ttalink_rtk_sensors_p20_data_pack_chan(dst_addr, src_addr, chan, msg,  gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, true);
}


static inline uint16_t _ttalink_rtk_sensors_p20_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rtk_sensors_p20_data_pack_nocrc(dst_addr, src_addr, msg, rtk_sensors_p20_data->gnss_latitude, rtk_sensors_p20_data->gnss_longitude, rtk_sensors_p20_data->gnss_altitude, rtk_sensors_p20_data->stas_number_usedinsolution, rtk_sensors_p20_data->mode_type, rtk_sensors_p20_data->sol_status, rtk_sensors_p20_data->pos_type, rtk_sensors_p20_data->hor_spd, rtk_sensors_p20_data->trk_gnd, rtk_sensors_p20_data->Magnetic_Variation, rtk_sensors_p20_data->Declination, rtk_sensors_p20_data->gps_time_week, rtk_sensors_p20_data->gps_time_ms, rtk_sensors_p20_data->utc_year, rtk_sensors_p20_data->utc_month, rtk_sensors_p20_data->utc_day, rtk_sensors_p20_data->utc_hour, rtk_sensors_p20_data->utc_min, rtk_sensors_p20_data->utc_ms, rtk_sensors_p20_data->updateTime, rtk_sensors_p20_data->vel_updateTime, rtk_sensors_p20_data->pos_updateTime, rtk_sensors_p20_data->dop_updateTime, rtk_sensors_p20_data->heading_updateTime, rtk_sensors_p20_data->heading_sol_status, rtk_sensors_p20_data->heading_pos_type, rtk_sensors_p20_data->GDOP, rtk_sensors_p20_data->PDOP, rtk_sensors_p20_data->HDOP, rtk_sensors_p20_data->kalman_latitude, rtk_sensors_p20_data->kalman_longitude, rtk_sensors_p20_data->kalman_altitude, rtk_sensors_p20_data->lat_std_deviation, rtk_sensors_p20_data->lon_std_deviation, rtk_sensors_p20_data->alt_std_deviation, rtk_sensors_p20_data->kalman_velE, rtk_sensors_p20_data->kalman_velN, rtk_sensors_p20_data->kalman_velU, rtk_sensors_p20_data->vel_std_g, rtk_sensors_p20_data->kalman_yaw, rtk_sensors_p20_data->kalman_pitch, rtk_sensors_p20_data->kalman_roll, rtk_sensors_p20_data->yaw_std_deviation, rtk_sensors_p20_data->pitch_std_deviation, rtk_sensors_p20_data->roll_std_deviation, rtk_sensors_p20_data->base_line, rtk_sensors_p20_data->rtk_version, rtk_sensors_p20_data->param2, rtk_sensors_p20_data->param3, rtk_sensors_p20_data->param4);
    }else{
        return ttalink_rtk_sensors_p20_data_pack(dst_addr, src_addr, msg, rtk_sensors_p20_data->gnss_latitude, rtk_sensors_p20_data->gnss_longitude, rtk_sensors_p20_data->gnss_altitude, rtk_sensors_p20_data->stas_number_usedinsolution, rtk_sensors_p20_data->mode_type, rtk_sensors_p20_data->sol_status, rtk_sensors_p20_data->pos_type, rtk_sensors_p20_data->hor_spd, rtk_sensors_p20_data->trk_gnd, rtk_sensors_p20_data->Magnetic_Variation, rtk_sensors_p20_data->Declination, rtk_sensors_p20_data->gps_time_week, rtk_sensors_p20_data->gps_time_ms, rtk_sensors_p20_data->utc_year, rtk_sensors_p20_data->utc_month, rtk_sensors_p20_data->utc_day, rtk_sensors_p20_data->utc_hour, rtk_sensors_p20_data->utc_min, rtk_sensors_p20_data->utc_ms, rtk_sensors_p20_data->updateTime, rtk_sensors_p20_data->vel_updateTime, rtk_sensors_p20_data->pos_updateTime, rtk_sensors_p20_data->dop_updateTime, rtk_sensors_p20_data->heading_updateTime, rtk_sensors_p20_data->heading_sol_status, rtk_sensors_p20_data->heading_pos_type, rtk_sensors_p20_data->GDOP, rtk_sensors_p20_data->PDOP, rtk_sensors_p20_data->HDOP, rtk_sensors_p20_data->kalman_latitude, rtk_sensors_p20_data->kalman_longitude, rtk_sensors_p20_data->kalman_altitude, rtk_sensors_p20_data->lat_std_deviation, rtk_sensors_p20_data->lon_std_deviation, rtk_sensors_p20_data->alt_std_deviation, rtk_sensors_p20_data->kalman_velE, rtk_sensors_p20_data->kalman_velN, rtk_sensors_p20_data->kalman_velU, rtk_sensors_p20_data->vel_std_g, rtk_sensors_p20_data->kalman_yaw, rtk_sensors_p20_data->kalman_pitch, rtk_sensors_p20_data->kalman_roll, rtk_sensors_p20_data->yaw_std_deviation, rtk_sensors_p20_data->pitch_std_deviation, rtk_sensors_p20_data->roll_std_deviation, rtk_sensors_p20_data->base_line, rtk_sensors_p20_data->rtk_version, rtk_sensors_p20_data->param2, rtk_sensors_p20_data->param3, rtk_sensors_p20_data->param4);
    }
    
}

/**
 * @brief Encode a rtk_sensors_p20_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_p20_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data)
{
    return _ttalink_rtk_sensors_p20_data_encode(dst_addr, src_addr, msg, rtk_sensors_p20_data, false);
}

/**
 * @brief Encode a rtk_sensors_p20_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_p20_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data)
{
    return _ttalink_rtk_sensors_p20_data_encode(dst_addr, src_addr, msg, rtk_sensors_p20_data, true);
}


static inline uint16_t _ttalink_rtk_sensors_p20_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data, bool nocrc)
{
    if(nocrc){
        return ttalink_rtk_sensors_p20_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, rtk_sensors_p20_data->gnss_latitude, rtk_sensors_p20_data->gnss_longitude, rtk_sensors_p20_data->gnss_altitude, rtk_sensors_p20_data->stas_number_usedinsolution, rtk_sensors_p20_data->mode_type, rtk_sensors_p20_data->sol_status, rtk_sensors_p20_data->pos_type, rtk_sensors_p20_data->hor_spd, rtk_sensors_p20_data->trk_gnd, rtk_sensors_p20_data->Magnetic_Variation, rtk_sensors_p20_data->Declination, rtk_sensors_p20_data->gps_time_week, rtk_sensors_p20_data->gps_time_ms, rtk_sensors_p20_data->utc_year, rtk_sensors_p20_data->utc_month, rtk_sensors_p20_data->utc_day, rtk_sensors_p20_data->utc_hour, rtk_sensors_p20_data->utc_min, rtk_sensors_p20_data->utc_ms, rtk_sensors_p20_data->updateTime, rtk_sensors_p20_data->vel_updateTime, rtk_sensors_p20_data->pos_updateTime, rtk_sensors_p20_data->dop_updateTime, rtk_sensors_p20_data->heading_updateTime, rtk_sensors_p20_data->heading_sol_status, rtk_sensors_p20_data->heading_pos_type, rtk_sensors_p20_data->GDOP, rtk_sensors_p20_data->PDOP, rtk_sensors_p20_data->HDOP, rtk_sensors_p20_data->kalman_latitude, rtk_sensors_p20_data->kalman_longitude, rtk_sensors_p20_data->kalman_altitude, rtk_sensors_p20_data->lat_std_deviation, rtk_sensors_p20_data->lon_std_deviation, rtk_sensors_p20_data->alt_std_deviation, rtk_sensors_p20_data->kalman_velE, rtk_sensors_p20_data->kalman_velN, rtk_sensors_p20_data->kalman_velU, rtk_sensors_p20_data->vel_std_g, rtk_sensors_p20_data->kalman_yaw, rtk_sensors_p20_data->kalman_pitch, rtk_sensors_p20_data->kalman_roll, rtk_sensors_p20_data->yaw_std_deviation, rtk_sensors_p20_data->pitch_std_deviation, rtk_sensors_p20_data->roll_std_deviation, rtk_sensors_p20_data->base_line, rtk_sensors_p20_data->rtk_version, rtk_sensors_p20_data->param2, rtk_sensors_p20_data->param3, rtk_sensors_p20_data->param4);
    }else{
        return ttalink_rtk_sensors_p20_data_pack_chan(dst_addr, src_addr, chan, msg, rtk_sensors_p20_data->gnss_latitude, rtk_sensors_p20_data->gnss_longitude, rtk_sensors_p20_data->gnss_altitude, rtk_sensors_p20_data->stas_number_usedinsolution, rtk_sensors_p20_data->mode_type, rtk_sensors_p20_data->sol_status, rtk_sensors_p20_data->pos_type, rtk_sensors_p20_data->hor_spd, rtk_sensors_p20_data->trk_gnd, rtk_sensors_p20_data->Magnetic_Variation, rtk_sensors_p20_data->Declination, rtk_sensors_p20_data->gps_time_week, rtk_sensors_p20_data->gps_time_ms, rtk_sensors_p20_data->utc_year, rtk_sensors_p20_data->utc_month, rtk_sensors_p20_data->utc_day, rtk_sensors_p20_data->utc_hour, rtk_sensors_p20_data->utc_min, rtk_sensors_p20_data->utc_ms, rtk_sensors_p20_data->updateTime, rtk_sensors_p20_data->vel_updateTime, rtk_sensors_p20_data->pos_updateTime, rtk_sensors_p20_data->dop_updateTime, rtk_sensors_p20_data->heading_updateTime, rtk_sensors_p20_data->heading_sol_status, rtk_sensors_p20_data->heading_pos_type, rtk_sensors_p20_data->GDOP, rtk_sensors_p20_data->PDOP, rtk_sensors_p20_data->HDOP, rtk_sensors_p20_data->kalman_latitude, rtk_sensors_p20_data->kalman_longitude, rtk_sensors_p20_data->kalman_altitude, rtk_sensors_p20_data->lat_std_deviation, rtk_sensors_p20_data->lon_std_deviation, rtk_sensors_p20_data->alt_std_deviation, rtk_sensors_p20_data->kalman_velE, rtk_sensors_p20_data->kalman_velN, rtk_sensors_p20_data->kalman_velU, rtk_sensors_p20_data->vel_std_g, rtk_sensors_p20_data->kalman_yaw, rtk_sensors_p20_data->kalman_pitch, rtk_sensors_p20_data->kalman_roll, rtk_sensors_p20_data->yaw_std_deviation, rtk_sensors_p20_data->pitch_std_deviation, rtk_sensors_p20_data->roll_std_deviation, rtk_sensors_p20_data->base_line, rtk_sensors_p20_data->rtk_version, rtk_sensors_p20_data->param2, rtk_sensors_p20_data->param3, rtk_sensors_p20_data->param4);
    }
}

/**
 * @brief Encode a rtk_sensors_p20_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_p20_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data)
{
    return _ttalink_rtk_sensors_p20_data_encode_chan(dst_addr, src_addr, chan, msg, rtk_sensors_p20_data, false);
}

/**
 * @brief Encode a rtk_sensors_p20_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param rtk_sensors_p20_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data)
{
    return _ttalink_rtk_sensors_p20_data_encode_chan(dst_addr, src_addr, chan, msg, rtk_sensors_p20_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_rtk_sensors_p20_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN];
    _tta_put_double(buf, 0, gnss_latitude);
    _tta_put_double(buf, 8, gnss_longitude);
    _tta_put_float(buf, 16, gnss_altitude);
    _tta_put_uint8_t(buf, 20, stas_number_usedinsolution);
    _tta_put_uint8_t(buf, 21, mode_type);
    _tta_put_uint8_t(buf, 22, sol_status);
    _tta_put_uint8_t(buf, 23, pos_type);
    _tta_put_float(buf, 24, hor_spd);
    _tta_put_float(buf, 28, trk_gnd);
    _tta_put_float(buf, 32, Magnetic_Variation);
    _tta_put_float(buf, 36, Declination);
    _tta_put_uint16_t(buf, 40, gps_time_week);
    _tta_put_uint32_t(buf, 42, gps_time_ms);
    _tta_put_uint32_t(buf, 46, utc_year);
    _tta_put_uint8_t(buf, 50, utc_month);
    _tta_put_uint8_t(buf, 51, utc_day);
    _tta_put_uint8_t(buf, 52, utc_hour);
    _tta_put_uint8_t(buf, 53, utc_min);
    _tta_put_uint32_t(buf, 54, utc_ms);
    _tta_put_uint32_t(buf, 58, updateTime);
    _tta_put_uint32_t(buf, 62, vel_updateTime);
    _tta_put_uint32_t(buf, 66, pos_updateTime);
    _tta_put_uint32_t(buf, 70, dop_updateTime);
    _tta_put_uint32_t(buf, 74, heading_updateTime);
    _tta_put_uint8_t(buf, 78, heading_sol_status);
    _tta_put_uint8_t(buf, 79, heading_pos_type);
    _tta_put_float(buf, 80, GDOP);
    _tta_put_float(buf, 84, PDOP);
    _tta_put_float(buf, 88, HDOP);
    _tta_put_double(buf, 92, kalman_latitude);
    _tta_put_double(buf, 100, kalman_longitude);
    _tta_put_float(buf, 108, kalman_altitude);
    _tta_put_float(buf, 112, lat_std_deviation);
    _tta_put_float(buf, 116, lon_std_deviation);
    _tta_put_float(buf, 120, alt_std_deviation);
    _tta_put_float(buf, 124, kalman_velE);
    _tta_put_float(buf, 128, kalman_velN);
    _tta_put_float(buf, 132, kalman_velU);
    _tta_put_float(buf, 136, vel_std_g);
    _tta_put_float(buf, 140, kalman_yaw);
    _tta_put_float(buf, 144, kalman_pitch);
    _tta_put_float(buf, 148, kalman_roll);
    _tta_put_float(buf, 152, yaw_std_deviation);
    _tta_put_float(buf, 156, pitch_std_deviation);
    _tta_put_float(buf, 160, roll_std_deviation);
    _tta_put_float(buf, 164, base_line);
    _tta_put_uint32_t(buf, 168, rtk_version);
    _tta_put_float(buf, 172, param2);
    _tta_put_float(buf, 176, param3);
    _tta_put_float(buf, 180, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA, buf, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC, nocrc);
#else
    ttalink_rtk_sensors_p20_data_t packet;
    packet.gnss_latitude = gnss_latitude;
    packet.gnss_longitude = gnss_longitude;
    packet.gnss_altitude = gnss_altitude;
    packet.stas_number_usedinsolution = stas_number_usedinsolution;
    packet.mode_type = mode_type;
    packet.sol_status = sol_status;
    packet.pos_type = pos_type;
    packet.hor_spd = hor_spd;
    packet.trk_gnd = trk_gnd;
    packet.Magnetic_Variation = Magnetic_Variation;
    packet.Declination = Declination;
    packet.gps_time_week = gps_time_week;
    packet.gps_time_ms = gps_time_ms;
    packet.utc_year = utc_year;
    packet.utc_month = utc_month;
    packet.utc_day = utc_day;
    packet.utc_hour = utc_hour;
    packet.utc_min = utc_min;
    packet.utc_ms = utc_ms;
    packet.updateTime = updateTime;
    packet.vel_updateTime = vel_updateTime;
    packet.pos_updateTime = pos_updateTime;
    packet.dop_updateTime = dop_updateTime;
    packet.heading_updateTime = heading_updateTime;
    packet.heading_sol_status = heading_sol_status;
    packet.heading_pos_type = heading_pos_type;
    packet.GDOP = GDOP;
    packet.PDOP = PDOP;
    packet.HDOP = HDOP;
    packet.kalman_latitude = kalman_latitude;
    packet.kalman_longitude = kalman_longitude;
    packet.kalman_altitude = kalman_altitude;
    packet.lat_std_deviation = lat_std_deviation;
    packet.lon_std_deviation = lon_std_deviation;
    packet.alt_std_deviation = alt_std_deviation;
    packet.kalman_velE = kalman_velE;
    packet.kalman_velN = kalman_velN;
    packet.kalman_velU = kalman_velU;
    packet.vel_std_g = vel_std_g;
    packet.kalman_yaw = kalman_yaw;
    packet.kalman_pitch = kalman_pitch;
    packet.kalman_roll = kalman_roll;
    packet.yaw_std_deviation = yaw_std_deviation;
    packet.pitch_std_deviation = pitch_std_deviation;
    packet.roll_std_deviation = roll_std_deviation;
    packet.base_line = base_line;
    packet.rtk_version = rtk_version;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA, (const char *)&packet, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rtk_sensors_p20_data message
 * @param chan TTAlink channel to send the message
 *
 * @param gnss_latitude  .
 * @param gnss_longitude  .
 * @param gnss_altitude  .
 * @param stas_number_usedinsolution  使用的卫星数.
 * @param mode_type  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.
 * @param sol_status  .
 * @param pos_type  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param Magnetic_Variation 磁偏角.
 * @param Declination 磁偏角方向.
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param GDOP  几何精度因子.
 * @param PDOP  位置精度.
 * @param HDOP  水平精度.
 * @param kalman_latitude  组合纬度.
 * @param kalman_longitude  组合经度.
 * @param kalman_altitude  组合高度.
 * @param lat_std_deviation  纬度误差.
 * @param lon_std_deviation  经度误差.
 * @param alt_std_deviation  高度误差.
 * @param kalman_velE  东向速度.
 * @param kalman_velN  北向速度.
 * @param kalman_velU  天向速度.
 * @param vel_std_g  速度水平误差.
 * @param kalman_yaw  航向.
 * @param kalman_pitch  俯仰.
 * @param kalman_roll  横滚.
 * @param yaw_std_deviation  航向误差.
 * @param pitch_std_deviation  俯仰误差.
 * @param roll_std_deviation  横滚误差.
 * @param base_line  基线距离.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_rtk_sensors_p20_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_p20_data_send(dst_addr, src_addr, chan, gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, false);
}

/**
 * @brief Send a rtk_sensors_p20_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param gnss_latitude  .
 * @param gnss_longitude  .
 * @param gnss_altitude  .
 * @param stas_number_usedinsolution  使用的卫星数.
 * @param mode_type  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.
 * @param sol_status  .
 * @param pos_type  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.
 * @param hor_spd  地速.
 * @param trk_gnd 地速与北向夹角.
 * @param Magnetic_Variation 磁偏角.
 * @param Declination 磁偏角方向.
 * @param gps_time_week  整周.
 * @param gps_time_ms  周内秒.
 * @param utc_year  UTC年.
 * @param utc_month  UTC月.
 * @param utc_day  UTC日.
 * @param utc_hour  UTC时.
 * @param utc_min  UTC分.
 * @param utc_ms  UTC毫秒.
 * @param updateTime  .
 * @param vel_updateTime  .
 * @param pos_updateTime  .
 * @param dop_updateTime  .
 * @param heading_updateTime  .
 * @param heading_sol_status  .
 * @param heading_pos_type  .
 * @param GDOP  几何精度因子.
 * @param PDOP  位置精度.
 * @param HDOP  水平精度.
 * @param kalman_latitude  组合纬度.
 * @param kalman_longitude  组合经度.
 * @param kalman_altitude  组合高度.
 * @param lat_std_deviation  纬度误差.
 * @param lon_std_deviation  经度误差.
 * @param alt_std_deviation  高度误差.
 * @param kalman_velE  东向速度.
 * @param kalman_velN  北向速度.
 * @param kalman_velU  天向速度.
 * @param vel_std_g  速度水平误差.
 * @param kalman_yaw  航向.
 * @param kalman_pitch  俯仰.
 * @param kalman_roll  横滚.
 * @param yaw_std_deviation  航向误差.
 * @param pitch_std_deviation  俯仰误差.
 * @param roll_std_deviation  横滚误差.
 * @param base_line  基线距离.
 * @param rtk_version  .
 * @param param2  .
 * @param param3  .
 * @param param4  .
 */
static inline void ttalink_rtk_sensors_p20_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_p20_data_send(dst_addr, src_addr, chan, gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, true);
}


static inline void _ttalink_rtk_sensors_p20_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_rtk_sensors_p20_data_send_nocrc(dst_addr, src_addr, chan, rtk_sensors_p20_data->gnss_latitude, rtk_sensors_p20_data->gnss_longitude, rtk_sensors_p20_data->gnss_altitude, rtk_sensors_p20_data->stas_number_usedinsolution, rtk_sensors_p20_data->mode_type, rtk_sensors_p20_data->sol_status, rtk_sensors_p20_data->pos_type, rtk_sensors_p20_data->hor_spd, rtk_sensors_p20_data->trk_gnd, rtk_sensors_p20_data->Magnetic_Variation, rtk_sensors_p20_data->Declination, rtk_sensors_p20_data->gps_time_week, rtk_sensors_p20_data->gps_time_ms, rtk_sensors_p20_data->utc_year, rtk_sensors_p20_data->utc_month, rtk_sensors_p20_data->utc_day, rtk_sensors_p20_data->utc_hour, rtk_sensors_p20_data->utc_min, rtk_sensors_p20_data->utc_ms, rtk_sensors_p20_data->updateTime, rtk_sensors_p20_data->vel_updateTime, rtk_sensors_p20_data->pos_updateTime, rtk_sensors_p20_data->dop_updateTime, rtk_sensors_p20_data->heading_updateTime, rtk_sensors_p20_data->heading_sol_status, rtk_sensors_p20_data->heading_pos_type, rtk_sensors_p20_data->GDOP, rtk_sensors_p20_data->PDOP, rtk_sensors_p20_data->HDOP, rtk_sensors_p20_data->kalman_latitude, rtk_sensors_p20_data->kalman_longitude, rtk_sensors_p20_data->kalman_altitude, rtk_sensors_p20_data->lat_std_deviation, rtk_sensors_p20_data->lon_std_deviation, rtk_sensors_p20_data->alt_std_deviation, rtk_sensors_p20_data->kalman_velE, rtk_sensors_p20_data->kalman_velN, rtk_sensors_p20_data->kalman_velU, rtk_sensors_p20_data->vel_std_g, rtk_sensors_p20_data->kalman_yaw, rtk_sensors_p20_data->kalman_pitch, rtk_sensors_p20_data->kalman_roll, rtk_sensors_p20_data->yaw_std_deviation, rtk_sensors_p20_data->pitch_std_deviation, rtk_sensors_p20_data->roll_std_deviation, rtk_sensors_p20_data->base_line, rtk_sensors_p20_data->rtk_version, rtk_sensors_p20_data->param2, rtk_sensors_p20_data->param3, rtk_sensors_p20_data->param4);
    }else{
        ttalink_rtk_sensors_p20_data_send(dst_addr, src_addr, chan, rtk_sensors_p20_data->gnss_latitude, rtk_sensors_p20_data->gnss_longitude, rtk_sensors_p20_data->gnss_altitude, rtk_sensors_p20_data->stas_number_usedinsolution, rtk_sensors_p20_data->mode_type, rtk_sensors_p20_data->sol_status, rtk_sensors_p20_data->pos_type, rtk_sensors_p20_data->hor_spd, rtk_sensors_p20_data->trk_gnd, rtk_sensors_p20_data->Magnetic_Variation, rtk_sensors_p20_data->Declination, rtk_sensors_p20_data->gps_time_week, rtk_sensors_p20_data->gps_time_ms, rtk_sensors_p20_data->utc_year, rtk_sensors_p20_data->utc_month, rtk_sensors_p20_data->utc_day, rtk_sensors_p20_data->utc_hour, rtk_sensors_p20_data->utc_min, rtk_sensors_p20_data->utc_ms, rtk_sensors_p20_data->updateTime, rtk_sensors_p20_data->vel_updateTime, rtk_sensors_p20_data->pos_updateTime, rtk_sensors_p20_data->dop_updateTime, rtk_sensors_p20_data->heading_updateTime, rtk_sensors_p20_data->heading_sol_status, rtk_sensors_p20_data->heading_pos_type, rtk_sensors_p20_data->GDOP, rtk_sensors_p20_data->PDOP, rtk_sensors_p20_data->HDOP, rtk_sensors_p20_data->kalman_latitude, rtk_sensors_p20_data->kalman_longitude, rtk_sensors_p20_data->kalman_altitude, rtk_sensors_p20_data->lat_std_deviation, rtk_sensors_p20_data->lon_std_deviation, rtk_sensors_p20_data->alt_std_deviation, rtk_sensors_p20_data->kalman_velE, rtk_sensors_p20_data->kalman_velN, rtk_sensors_p20_data->kalman_velU, rtk_sensors_p20_data->vel_std_g, rtk_sensors_p20_data->kalman_yaw, rtk_sensors_p20_data->kalman_pitch, rtk_sensors_p20_data->kalman_roll, rtk_sensors_p20_data->yaw_std_deviation, rtk_sensors_p20_data->pitch_std_deviation, rtk_sensors_p20_data->roll_std_deviation, rtk_sensors_p20_data->base_line, rtk_sensors_p20_data->rtk_version, rtk_sensors_p20_data->param2, rtk_sensors_p20_data->param3, rtk_sensors_p20_data->param4);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA, (const char *)rtk_sensors_p20_data, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a rtk_sensors_p20_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rtk_sensors_p20_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data)
{
    _ttalink_rtk_sensors_p20_data_send_struct(dst_addr, src_addr, chan, rtk_sensors_p20_data, false);
}

/**
 * @brief Send a rtk_sensors_p20_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_rtk_sensors_p20_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data)
{
    _ttalink_rtk_sensors_p20_data_send_struct(dst_addr, src_addr, chan, rtk_sensors_p20_data, true);
}

#if TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_rtk_sensors_p20_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_double(buf, 0, gnss_latitude);
    _tta_put_double(buf, 8, gnss_longitude);
    _tta_put_float(buf, 16, gnss_altitude);
    _tta_put_uint8_t(buf, 20, stas_number_usedinsolution);
    _tta_put_uint8_t(buf, 21, mode_type);
    _tta_put_uint8_t(buf, 22, sol_status);
    _tta_put_uint8_t(buf, 23, pos_type);
    _tta_put_float(buf, 24, hor_spd);
    _tta_put_float(buf, 28, trk_gnd);
    _tta_put_float(buf, 32, Magnetic_Variation);
    _tta_put_float(buf, 36, Declination);
    _tta_put_uint16_t(buf, 40, gps_time_week);
    _tta_put_uint32_t(buf, 42, gps_time_ms);
    _tta_put_uint32_t(buf, 46, utc_year);
    _tta_put_uint8_t(buf, 50, utc_month);
    _tta_put_uint8_t(buf, 51, utc_day);
    _tta_put_uint8_t(buf, 52, utc_hour);
    _tta_put_uint8_t(buf, 53, utc_min);
    _tta_put_uint32_t(buf, 54, utc_ms);
    _tta_put_uint32_t(buf, 58, updateTime);
    _tta_put_uint32_t(buf, 62, vel_updateTime);
    _tta_put_uint32_t(buf, 66, pos_updateTime);
    _tta_put_uint32_t(buf, 70, dop_updateTime);
    _tta_put_uint32_t(buf, 74, heading_updateTime);
    _tta_put_uint8_t(buf, 78, heading_sol_status);
    _tta_put_uint8_t(buf, 79, heading_pos_type);
    _tta_put_float(buf, 80, GDOP);
    _tta_put_float(buf, 84, PDOP);
    _tta_put_float(buf, 88, HDOP);
    _tta_put_double(buf, 92, kalman_latitude);
    _tta_put_double(buf, 100, kalman_longitude);
    _tta_put_float(buf, 108, kalman_altitude);
    _tta_put_float(buf, 112, lat_std_deviation);
    _tta_put_float(buf, 116, lon_std_deviation);
    _tta_put_float(buf, 120, alt_std_deviation);
    _tta_put_float(buf, 124, kalman_velE);
    _tta_put_float(buf, 128, kalman_velN);
    _tta_put_float(buf, 132, kalman_velU);
    _tta_put_float(buf, 136, vel_std_g);
    _tta_put_float(buf, 140, kalman_yaw);
    _tta_put_float(buf, 144, kalman_pitch);
    _tta_put_float(buf, 148, kalman_roll);
    _tta_put_float(buf, 152, yaw_std_deviation);
    _tta_put_float(buf, 156, pitch_std_deviation);
    _tta_put_float(buf, 160, roll_std_deviation);
    _tta_put_float(buf, 164, base_line);
    _tta_put_uint32_t(buf, 168, rtk_version);
    _tta_put_float(buf, 172, param2);
    _tta_put_float(buf, 176, param3);
    _tta_put_float(buf, 180, param4);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA, buf, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC, nocrc);
#else
    ttalink_rtk_sensors_p20_data_t *packet = (ttalink_rtk_sensors_p20_data_t *)msgbuf;
    packet->gnss_latitude = gnss_latitude;
    packet->gnss_longitude = gnss_longitude;
    packet->gnss_altitude = gnss_altitude;
    packet->stas_number_usedinsolution = stas_number_usedinsolution;
    packet->mode_type = mode_type;
    packet->sol_status = sol_status;
    packet->pos_type = pos_type;
    packet->hor_spd = hor_spd;
    packet->trk_gnd = trk_gnd;
    packet->Magnetic_Variation = Magnetic_Variation;
    packet->Declination = Declination;
    packet->gps_time_week = gps_time_week;
    packet->gps_time_ms = gps_time_ms;
    packet->utc_year = utc_year;
    packet->utc_month = utc_month;
    packet->utc_day = utc_day;
    packet->utc_hour = utc_hour;
    packet->utc_min = utc_min;
    packet->utc_ms = utc_ms;
    packet->updateTime = updateTime;
    packet->vel_updateTime = vel_updateTime;
    packet->pos_updateTime = pos_updateTime;
    packet->dop_updateTime = dop_updateTime;
    packet->heading_updateTime = heading_updateTime;
    packet->heading_sol_status = heading_sol_status;
    packet->heading_pos_type = heading_pos_type;
    packet->GDOP = GDOP;
    packet->PDOP = PDOP;
    packet->HDOP = HDOP;
    packet->kalman_latitude = kalman_latitude;
    packet->kalman_longitude = kalman_longitude;
    packet->kalman_altitude = kalman_altitude;
    packet->lat_std_deviation = lat_std_deviation;
    packet->lon_std_deviation = lon_std_deviation;
    packet->alt_std_deviation = alt_std_deviation;
    packet->kalman_velE = kalman_velE;
    packet->kalman_velN = kalman_velN;
    packet->kalman_velU = kalman_velU;
    packet->vel_std_g = vel_std_g;
    packet->kalman_yaw = kalman_yaw;
    packet->kalman_pitch = kalman_pitch;
    packet->kalman_roll = kalman_roll;
    packet->yaw_std_deviation = yaw_std_deviation;
    packet->pitch_std_deviation = pitch_std_deviation;
    packet->roll_std_deviation = roll_std_deviation;
    packet->base_line = base_line;
    packet->rtk_version = rtk_version;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA, (const char *)packet, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_MIN_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_rtk_sensors_p20_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_p20_data_send_buf(dst_addr, src_addr, msgbuf, chan, gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_rtk_sensors_p20_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  double gnss_latitude, double gnss_longitude, float gnss_altitude, uint8_t stas_number_usedinsolution, uint8_t mode_type, uint8_t sol_status, uint8_t pos_type, float hor_spd, float trk_gnd, float Magnetic_Variation, float Declination, uint16_t gps_time_week, uint32_t gps_time_ms, uint32_t utc_year, uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, uint8_t utc_min, uint32_t utc_ms, uint32_t updateTime, uint32_t vel_updateTime, uint32_t pos_updateTime, uint32_t dop_updateTime, uint32_t heading_updateTime, uint8_t heading_sol_status, uint8_t heading_pos_type, float GDOP, float PDOP, float HDOP, double kalman_latitude, double kalman_longitude, float kalman_altitude, float lat_std_deviation, float lon_std_deviation, float alt_std_deviation, float kalman_velE, float kalman_velN, float kalman_velU, float vel_std_g, float kalman_yaw, float kalman_pitch, float kalman_roll, float yaw_std_deviation, float pitch_std_deviation, float roll_std_deviation, float base_line, uint32_t rtk_version, float param2, float param3, float param4)
{
    _ttalink_rtk_sensors_p20_data_send_buf(dst_addr, src_addr, msgbuf, chan, gnss_latitude, gnss_longitude, gnss_altitude, stas_number_usedinsolution, mode_type, sol_status, pos_type, hor_spd, trk_gnd, Magnetic_Variation, Declination, gps_time_week, gps_time_ms, utc_year, utc_month, utc_day, utc_hour, utc_min, utc_ms, updateTime, vel_updateTime, pos_updateTime, dop_updateTime, heading_updateTime, heading_sol_status, heading_pos_type, GDOP, PDOP, HDOP, kalman_latitude, kalman_longitude, kalman_altitude, lat_std_deviation, lon_std_deviation, alt_std_deviation, kalman_velE, kalman_velN, kalman_velU, vel_std_g, kalman_yaw, kalman_pitch, kalman_roll, yaw_std_deviation, pitch_std_deviation, roll_std_deviation, base_line, rtk_version, param2, param3, param4, true);
}
#endif

#endif

// MESSAGE RTK_SENSORS_P20_DATA UNPACKING


/**
 * @brief Get field gnss_latitude from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline double ttalink_rtk_sensors_p20_data_get_gnss_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  0);
}

/**
 * @brief Get field gnss_longitude from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline double ttalink_rtk_sensors_p20_data_get_gnss_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  8);
}

/**
 * @brief Get field gnss_altitude from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_p20_data_get_gnss_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  16);
}

/**
 * @brief Get field stas_number_usedinsolution from rtk_sensors_p20_data message
 *
 * @return  使用的卫星数.
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_stas_number_usedinsolution(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field mode_type from rtk_sensors_p20_data message
 *
 * @return  模式指示：A=自主定位，D=差分，E=估 算，N=数据无效.
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_mode_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field sol_status from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_sol_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field pos_type from rtk_sensors_p20_data message
 *
 * @return  定位质量：0=无效解，1=单点解，2=差分解，4=固定解，5=浮点解，6=惯导解.
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field hor_spd from rtk_sensors_p20_data message
 *
 * @return  地速.
 */
static inline float ttalink_rtk_sensors_p20_data_get_hor_spd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  24);
}

/**
 * @brief Get field trk_gnd from rtk_sensors_p20_data message
 *
 * @return 地速与北向夹角.
 */
static inline float ttalink_rtk_sensors_p20_data_get_trk_gnd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  28);
}

/**
 * @brief Get field Magnetic_Variation from rtk_sensors_p20_data message
 *
 * @return 磁偏角.
 */
static inline float ttalink_rtk_sensors_p20_data_get_Magnetic_Variation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  32);
}

/**
 * @brief Get field Declination from rtk_sensors_p20_data message
 *
 * @return 磁偏角方向.
 */
static inline float ttalink_rtk_sensors_p20_data_get_Declination(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  36);
}

/**
 * @brief Get field gps_time_week from rtk_sensors_p20_data message
 *
 * @return  整周.
 */
static inline uint16_t ttalink_rtk_sensors_p20_data_get_gps_time_week(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field gps_time_ms from rtk_sensors_p20_data message
 *
 * @return  周内秒.
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_gps_time_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  42);
}

/**
 * @brief Get field utc_year from rtk_sensors_p20_data message
 *
 * @return  UTC年.
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_utc_year(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  46);
}

/**
 * @brief Get field utc_month from rtk_sensors_p20_data message
 *
 * @return  UTC月.
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_utc_month(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field utc_day from rtk_sensors_p20_data message
 *
 * @return  UTC日.
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_utc_day(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field utc_hour from rtk_sensors_p20_data message
 *
 * @return  UTC时.
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_utc_hour(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field utc_min from rtk_sensors_p20_data message
 *
 * @return  UTC分.
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_utc_min(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field utc_ms from rtk_sensors_p20_data message
 *
 * @return  UTC毫秒.
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_utc_ms(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  54);
}

/**
 * @brief Get field updateTime from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  58);
}

/**
 * @brief Get field vel_updateTime from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_vel_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  62);
}

/**
 * @brief Get field pos_updateTime from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_pos_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  66);
}

/**
 * @brief Get field dop_updateTime from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_dop_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  70);
}

/**
 * @brief Get field heading_updateTime from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_heading_updateTime(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  74);
}

/**
 * @brief Get field heading_sol_status from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_heading_sol_status(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  78);
}

/**
 * @brief Get field heading_pos_type from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint8_t ttalink_rtk_sensors_p20_data_get_heading_pos_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  79);
}

/**
 * @brief Get field GDOP from rtk_sensors_p20_data message
 *
 * @return  几何精度因子.
 */
static inline float ttalink_rtk_sensors_p20_data_get_GDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  80);
}

/**
 * @brief Get field PDOP from rtk_sensors_p20_data message
 *
 * @return  位置精度.
 */
static inline float ttalink_rtk_sensors_p20_data_get_PDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  84);
}

/**
 * @brief Get field HDOP from rtk_sensors_p20_data message
 *
 * @return  水平精度.
 */
static inline float ttalink_rtk_sensors_p20_data_get_HDOP(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  88);
}

/**
 * @brief Get field kalman_latitude from rtk_sensors_p20_data message
 *
 * @return  组合纬度.
 */
static inline double ttalink_rtk_sensors_p20_data_get_kalman_latitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  92);
}

/**
 * @brief Get field kalman_longitude from rtk_sensors_p20_data message
 *
 * @return  组合经度.
 */
static inline double ttalink_rtk_sensors_p20_data_get_kalman_longitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_double(msg,  100);
}

/**
 * @brief Get field kalman_altitude from rtk_sensors_p20_data message
 *
 * @return  组合高度.
 */
static inline float ttalink_rtk_sensors_p20_data_get_kalman_altitude(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  108);
}

/**
 * @brief Get field lat_std_deviation from rtk_sensors_p20_data message
 *
 * @return  纬度误差.
 */
static inline float ttalink_rtk_sensors_p20_data_get_lat_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  112);
}

/**
 * @brief Get field lon_std_deviation from rtk_sensors_p20_data message
 *
 * @return  经度误差.
 */
static inline float ttalink_rtk_sensors_p20_data_get_lon_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  116);
}

/**
 * @brief Get field alt_std_deviation from rtk_sensors_p20_data message
 *
 * @return  高度误差.
 */
static inline float ttalink_rtk_sensors_p20_data_get_alt_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  120);
}

/**
 * @brief Get field kalman_velE from rtk_sensors_p20_data message
 *
 * @return  东向速度.
 */
static inline float ttalink_rtk_sensors_p20_data_get_kalman_velE(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  124);
}

/**
 * @brief Get field kalman_velN from rtk_sensors_p20_data message
 *
 * @return  北向速度.
 */
static inline float ttalink_rtk_sensors_p20_data_get_kalman_velN(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  128);
}

/**
 * @brief Get field kalman_velU from rtk_sensors_p20_data message
 *
 * @return  天向速度.
 */
static inline float ttalink_rtk_sensors_p20_data_get_kalman_velU(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  132);
}

/**
 * @brief Get field vel_std_g from rtk_sensors_p20_data message
 *
 * @return  速度水平误差.
 */
static inline float ttalink_rtk_sensors_p20_data_get_vel_std_g(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  136);
}

/**
 * @brief Get field kalman_yaw from rtk_sensors_p20_data message
 *
 * @return  航向.
 */
static inline float ttalink_rtk_sensors_p20_data_get_kalman_yaw(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  140);
}

/**
 * @brief Get field kalman_pitch from rtk_sensors_p20_data message
 *
 * @return  俯仰.
 */
static inline float ttalink_rtk_sensors_p20_data_get_kalman_pitch(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  144);
}

/**
 * @brief Get field kalman_roll from rtk_sensors_p20_data message
 *
 * @return  横滚.
 */
static inline float ttalink_rtk_sensors_p20_data_get_kalman_roll(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  148);
}

/**
 * @brief Get field yaw_std_deviation from rtk_sensors_p20_data message
 *
 * @return  航向误差.
 */
static inline float ttalink_rtk_sensors_p20_data_get_yaw_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  152);
}

/**
 * @brief Get field pitch_std_deviation from rtk_sensors_p20_data message
 *
 * @return  俯仰误差.
 */
static inline float ttalink_rtk_sensors_p20_data_get_pitch_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  156);
}

/**
 * @brief Get field roll_std_deviation from rtk_sensors_p20_data message
 *
 * @return  横滚误差.
 */
static inline float ttalink_rtk_sensors_p20_data_get_roll_std_deviation(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  160);
}

/**
 * @brief Get field base_line from rtk_sensors_p20_data message
 *
 * @return  基线距离.
 */
static inline float ttalink_rtk_sensors_p20_data_get_base_line(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  164);
}

/**
 * @brief Get field rtk_version from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline uint32_t ttalink_rtk_sensors_p20_data_get_rtk_version(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  168);
}

/**
 * @brief Get field param2 from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_p20_data_get_param2(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  172);
}

/**
 * @brief Get field param3 from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_p20_data_get_param3(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  176);
}

/**
 * @brief Get field param4 from rtk_sensors_p20_data message
 *
 * @return  .
 */
static inline float ttalink_rtk_sensors_p20_data_get_param4(const ttalink_message_t* msg)
{
    return _TTA_RETURN_float(msg,  180);
}

/**
 * @brief Decode a rtk_sensors_p20_data message into a struct
 *
 * @param msg The message to decode
 * @param rtk_sensors_p20_data C-struct to decode the message contents into
 */
static inline void ttalink_rtk_sensors_p20_data_decode(const ttalink_message_t* msg, ttalink_rtk_sensors_p20_data_t* rtk_sensors_p20_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    rtk_sensors_p20_data->gnss_latitude = ttalink_rtk_sensors_p20_data_get_gnss_latitude(msg);
    rtk_sensors_p20_data->gnss_longitude = ttalink_rtk_sensors_p20_data_get_gnss_longitude(msg);
    rtk_sensors_p20_data->gnss_altitude = ttalink_rtk_sensors_p20_data_get_gnss_altitude(msg);
    rtk_sensors_p20_data->stas_number_usedinsolution = ttalink_rtk_sensors_p20_data_get_stas_number_usedinsolution(msg);
    rtk_sensors_p20_data->mode_type = ttalink_rtk_sensors_p20_data_get_mode_type(msg);
    rtk_sensors_p20_data->sol_status = ttalink_rtk_sensors_p20_data_get_sol_status(msg);
    rtk_sensors_p20_data->pos_type = ttalink_rtk_sensors_p20_data_get_pos_type(msg);
    rtk_sensors_p20_data->hor_spd = ttalink_rtk_sensors_p20_data_get_hor_spd(msg);
    rtk_sensors_p20_data->trk_gnd = ttalink_rtk_sensors_p20_data_get_trk_gnd(msg);
    rtk_sensors_p20_data->Magnetic_Variation = ttalink_rtk_sensors_p20_data_get_Magnetic_Variation(msg);
    rtk_sensors_p20_data->Declination = ttalink_rtk_sensors_p20_data_get_Declination(msg);
    rtk_sensors_p20_data->gps_time_week = ttalink_rtk_sensors_p20_data_get_gps_time_week(msg);
    rtk_sensors_p20_data->gps_time_ms = ttalink_rtk_sensors_p20_data_get_gps_time_ms(msg);
    rtk_sensors_p20_data->utc_year = ttalink_rtk_sensors_p20_data_get_utc_year(msg);
    rtk_sensors_p20_data->utc_month = ttalink_rtk_sensors_p20_data_get_utc_month(msg);
    rtk_sensors_p20_data->utc_day = ttalink_rtk_sensors_p20_data_get_utc_day(msg);
    rtk_sensors_p20_data->utc_hour = ttalink_rtk_sensors_p20_data_get_utc_hour(msg);
    rtk_sensors_p20_data->utc_min = ttalink_rtk_sensors_p20_data_get_utc_min(msg);
    rtk_sensors_p20_data->utc_ms = ttalink_rtk_sensors_p20_data_get_utc_ms(msg);
    rtk_sensors_p20_data->updateTime = ttalink_rtk_sensors_p20_data_get_updateTime(msg);
    rtk_sensors_p20_data->vel_updateTime = ttalink_rtk_sensors_p20_data_get_vel_updateTime(msg);
    rtk_sensors_p20_data->pos_updateTime = ttalink_rtk_sensors_p20_data_get_pos_updateTime(msg);
    rtk_sensors_p20_data->dop_updateTime = ttalink_rtk_sensors_p20_data_get_dop_updateTime(msg);
    rtk_sensors_p20_data->heading_updateTime = ttalink_rtk_sensors_p20_data_get_heading_updateTime(msg);
    rtk_sensors_p20_data->heading_sol_status = ttalink_rtk_sensors_p20_data_get_heading_sol_status(msg);
    rtk_sensors_p20_data->heading_pos_type = ttalink_rtk_sensors_p20_data_get_heading_pos_type(msg);
    rtk_sensors_p20_data->GDOP = ttalink_rtk_sensors_p20_data_get_GDOP(msg);
    rtk_sensors_p20_data->PDOP = ttalink_rtk_sensors_p20_data_get_PDOP(msg);
    rtk_sensors_p20_data->HDOP = ttalink_rtk_sensors_p20_data_get_HDOP(msg);
    rtk_sensors_p20_data->kalman_latitude = ttalink_rtk_sensors_p20_data_get_kalman_latitude(msg);
    rtk_sensors_p20_data->kalman_longitude = ttalink_rtk_sensors_p20_data_get_kalman_longitude(msg);
    rtk_sensors_p20_data->kalman_altitude = ttalink_rtk_sensors_p20_data_get_kalman_altitude(msg);
    rtk_sensors_p20_data->lat_std_deviation = ttalink_rtk_sensors_p20_data_get_lat_std_deviation(msg);
    rtk_sensors_p20_data->lon_std_deviation = ttalink_rtk_sensors_p20_data_get_lon_std_deviation(msg);
    rtk_sensors_p20_data->alt_std_deviation = ttalink_rtk_sensors_p20_data_get_alt_std_deviation(msg);
    rtk_sensors_p20_data->kalman_velE = ttalink_rtk_sensors_p20_data_get_kalman_velE(msg);
    rtk_sensors_p20_data->kalman_velN = ttalink_rtk_sensors_p20_data_get_kalman_velN(msg);
    rtk_sensors_p20_data->kalman_velU = ttalink_rtk_sensors_p20_data_get_kalman_velU(msg);
    rtk_sensors_p20_data->vel_std_g = ttalink_rtk_sensors_p20_data_get_vel_std_g(msg);
    rtk_sensors_p20_data->kalman_yaw = ttalink_rtk_sensors_p20_data_get_kalman_yaw(msg);
    rtk_sensors_p20_data->kalman_pitch = ttalink_rtk_sensors_p20_data_get_kalman_pitch(msg);
    rtk_sensors_p20_data->kalman_roll = ttalink_rtk_sensors_p20_data_get_kalman_roll(msg);
    rtk_sensors_p20_data->yaw_std_deviation = ttalink_rtk_sensors_p20_data_get_yaw_std_deviation(msg);
    rtk_sensors_p20_data->pitch_std_deviation = ttalink_rtk_sensors_p20_data_get_pitch_std_deviation(msg);
    rtk_sensors_p20_data->roll_std_deviation = ttalink_rtk_sensors_p20_data_get_roll_std_deviation(msg);
    rtk_sensors_p20_data->base_line = ttalink_rtk_sensors_p20_data_get_base_line(msg);
    rtk_sensors_p20_data->rtk_version = ttalink_rtk_sensors_p20_data_get_rtk_version(msg);
    rtk_sensors_p20_data->param2 = ttalink_rtk_sensors_p20_data_get_param2(msg);
    rtk_sensors_p20_data->param3 = ttalink_rtk_sensors_p20_data_get_param3(msg);
    rtk_sensors_p20_data->param4 = ttalink_rtk_sensors_p20_data_get_param4(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN? msg->len : TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN;
        memset(rtk_sensors_p20_data, 0, TTALINK_MSG_ID_RTK_SENSORS_P20_DATA_LEN);
    memcpy(rtk_sensors_p20_data, _TTA_PAYLOAD(msg), len);
#endif
}
