#pragma once
// MESSAGE IMU_SENSORS_DATA PACKING

#define TTALINK_MSG_ID_IMU_SENSORS_DATA 2172

TTAPACKED(
typedef struct __ttalink_imu_sensors_data_t {
 float main_gyro1[3]; /*<  .*/
 float main_gyro2[3]; /*<  .*/
 float backup_gyro[3]; /*<  .*/
 float main_acc1[3]; /*<  .*/
 float main_acc2[3]; /*<  .*/
 float backup_acc[3]; /*<  .*/
 float main_gyro1_var[3]; /*<  .*/
 float main_gyro2_var[3]; /*<  .*/
 float backup_gyro_var[3]; /*<  .*/
 float main_acc1_var[3]; /*<  .*/
 float main_acc2_var[3]; /*<  .*/
 float backup_acc_var[3]; /*<  .*/
 float main_gyro1_e[3]; /*<  .*/
 float main_gyro2_e[3]; /*<  .*/
 float backup_gyro_e[3]; /*<  .*/
 float main_acc1_e[3]; /*<  .*/
 float main_acc2_e[3]; /*<  .*/
 float backup_acc_e[3]; /*<  .*/
 int64_t main_update_time; /*<  .*/
 int64_t backeup_update_time; /*<  .*/
}) ttalink_imu_sensors_data_t;

#define TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN 232
#define TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN 232
#define TTALINK_MSG_ID_2172_LEN 232
#define TTALINK_MSG_ID_2172_MIN_LEN 232

#define TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC 185
#define TTALINK_MSG_ID_2172_CRC 185

#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_GYRO1_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_GYRO2_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_BACKUP_GYRO_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_ACC1_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_ACC2_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_BACKUP_ACC_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_GYRO1_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_GYRO2_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_BACKUP_GYRO_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_ACC1_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_ACC2_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_BACKUP_ACC_VAR_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_GYRO1_E_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_GYRO2_E_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_BACKUP_GYRO_E_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_ACC1_E_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_MAIN_ACC2_E_LEN 3
#define TTALINK_MSG_IMU_SENSORS_DATA_FIELD_BACKUP_ACC_E_LEN 3

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_IMU_SENSORS_DATA { \
    2172, \
    "IMU_SENSORS_DATA", \
    20, \
    {  { "main_gyro1", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_imu_sensors_data_t, main_gyro1) }, \
         { "main_gyro2", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_imu_sensors_data_t, main_gyro2) }, \
         { "backup_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_imu_sensors_data_t, backup_gyro) }, \
         { "main_acc1", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_imu_sensors_data_t, main_acc1) }, \
         { "main_acc2", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_imu_sensors_data_t, main_acc2) }, \
         { "backup_acc", NULL, TTALINK_TYPE_FLOAT, 3, 60, offsetof(ttalink_imu_sensors_data_t, backup_acc) }, \
         { "main_gyro1_var", NULL, TTALINK_TYPE_FLOAT, 3, 72, offsetof(ttalink_imu_sensors_data_t, main_gyro1_var) }, \
         { "main_gyro2_var", NULL, TTALINK_TYPE_FLOAT, 3, 84, offsetof(ttalink_imu_sensors_data_t, main_gyro2_var) }, \
         { "backup_gyro_var", NULL, TTALINK_TYPE_FLOAT, 3, 96, offsetof(ttalink_imu_sensors_data_t, backup_gyro_var) }, \
         { "main_acc1_var", NULL, TTALINK_TYPE_FLOAT, 3, 108, offsetof(ttalink_imu_sensors_data_t, main_acc1_var) }, \
         { "main_acc2_var", NULL, TTALINK_TYPE_FLOAT, 3, 120, offsetof(ttalink_imu_sensors_data_t, main_acc2_var) }, \
         { "backup_acc_var", NULL, TTALINK_TYPE_FLOAT, 3, 132, offsetof(ttalink_imu_sensors_data_t, backup_acc_var) }, \
         { "main_gyro1_e", NULL, TTALINK_TYPE_FLOAT, 3, 144, offsetof(ttalink_imu_sensors_data_t, main_gyro1_e) }, \
         { "main_gyro2_e", NULL, TTALINK_TYPE_FLOAT, 3, 156, offsetof(ttalink_imu_sensors_data_t, main_gyro2_e) }, \
         { "backup_gyro_e", NULL, TTALINK_TYPE_FLOAT, 3, 168, offsetof(ttalink_imu_sensors_data_t, backup_gyro_e) }, \
         { "main_acc1_e", NULL, TTALINK_TYPE_FLOAT, 3, 180, offsetof(ttalink_imu_sensors_data_t, main_acc1_e) }, \
         { "main_acc2_e", NULL, TTALINK_TYPE_FLOAT, 3, 192, offsetof(ttalink_imu_sensors_data_t, main_acc2_e) }, \
         { "backup_acc_e", NULL, TTALINK_TYPE_FLOAT, 3, 204, offsetof(ttalink_imu_sensors_data_t, backup_acc_e) }, \
         { "main_update_time", NULL, TTALINK_TYPE_INT64_T, 0, 216, offsetof(ttalink_imu_sensors_data_t, main_update_time) }, \
         { "backeup_update_time", NULL, TTALINK_TYPE_INT64_T, 0, 224, offsetof(ttalink_imu_sensors_data_t, backeup_update_time) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_IMU_SENSORS_DATA { \
    "IMU_SENSORS_DATA", \
    20, \
    {  { "main_gyro1", NULL, TTALINK_TYPE_FLOAT, 3, 0, offsetof(ttalink_imu_sensors_data_t, main_gyro1) }, \
         { "main_gyro2", NULL, TTALINK_TYPE_FLOAT, 3, 12, offsetof(ttalink_imu_sensors_data_t, main_gyro2) }, \
         { "backup_gyro", NULL, TTALINK_TYPE_FLOAT, 3, 24, offsetof(ttalink_imu_sensors_data_t, backup_gyro) }, \
         { "main_acc1", NULL, TTALINK_TYPE_FLOAT, 3, 36, offsetof(ttalink_imu_sensors_data_t, main_acc1) }, \
         { "main_acc2", NULL, TTALINK_TYPE_FLOAT, 3, 48, offsetof(ttalink_imu_sensors_data_t, main_acc2) }, \
         { "backup_acc", NULL, TTALINK_TYPE_FLOAT, 3, 60, offsetof(ttalink_imu_sensors_data_t, backup_acc) }, \
         { "main_gyro1_var", NULL, TTALINK_TYPE_FLOAT, 3, 72, offsetof(ttalink_imu_sensors_data_t, main_gyro1_var) }, \
         { "main_gyro2_var", NULL, TTALINK_TYPE_FLOAT, 3, 84, offsetof(ttalink_imu_sensors_data_t, main_gyro2_var) }, \
         { "backup_gyro_var", NULL, TTALINK_TYPE_FLOAT, 3, 96, offsetof(ttalink_imu_sensors_data_t, backup_gyro_var) }, \
         { "main_acc1_var", NULL, TTALINK_TYPE_FLOAT, 3, 108, offsetof(ttalink_imu_sensors_data_t, main_acc1_var) }, \
         { "main_acc2_var", NULL, TTALINK_TYPE_FLOAT, 3, 120, offsetof(ttalink_imu_sensors_data_t, main_acc2_var) }, \
         { "backup_acc_var", NULL, TTALINK_TYPE_FLOAT, 3, 132, offsetof(ttalink_imu_sensors_data_t, backup_acc_var) }, \
         { "main_gyro1_e", NULL, TTALINK_TYPE_FLOAT, 3, 144, offsetof(ttalink_imu_sensors_data_t, main_gyro1_e) }, \
         { "main_gyro2_e", NULL, TTALINK_TYPE_FLOAT, 3, 156, offsetof(ttalink_imu_sensors_data_t, main_gyro2_e) }, \
         { "backup_gyro_e", NULL, TTALINK_TYPE_FLOAT, 3, 168, offsetof(ttalink_imu_sensors_data_t, backup_gyro_e) }, \
         { "main_acc1_e", NULL, TTALINK_TYPE_FLOAT, 3, 180, offsetof(ttalink_imu_sensors_data_t, main_acc1_e) }, \
         { "main_acc2_e", NULL, TTALINK_TYPE_FLOAT, 3, 192, offsetof(ttalink_imu_sensors_data_t, main_acc2_e) }, \
         { "backup_acc_e", NULL, TTALINK_TYPE_FLOAT, 3, 204, offsetof(ttalink_imu_sensors_data_t, backup_acc_e) }, \
         { "main_update_time", NULL, TTALINK_TYPE_INT64_T, 0, 216, offsetof(ttalink_imu_sensors_data_t, main_update_time) }, \
         { "backeup_update_time", NULL, TTALINK_TYPE_INT64_T, 0, 224, offsetof(ttalink_imu_sensors_data_t, backeup_update_time) }, \
         } \
}
#endif


static inline uint16_t _ttalink_imu_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN];
    _tta_put_int64_t(buf, 216, main_update_time);
    _tta_put_int64_t(buf, 224, backeup_update_time);
    _tta_put_float_array(buf, 0, main_gyro1, 3);
    _tta_put_float_array(buf, 12, main_gyro2, 3);
    _tta_put_float_array(buf, 24, backup_gyro, 3);
    _tta_put_float_array(buf, 36, main_acc1, 3);
    _tta_put_float_array(buf, 48, main_acc2, 3);
    _tta_put_float_array(buf, 60, backup_acc, 3);
    _tta_put_float_array(buf, 72, main_gyro1_var, 3);
    _tta_put_float_array(buf, 84, main_gyro2_var, 3);
    _tta_put_float_array(buf, 96, backup_gyro_var, 3);
    _tta_put_float_array(buf, 108, main_acc1_var, 3);
    _tta_put_float_array(buf, 120, main_acc2_var, 3);
    _tta_put_float_array(buf, 132, backup_acc_var, 3);
    _tta_put_float_array(buf, 144, main_gyro1_e, 3);
    _tta_put_float_array(buf, 156, main_gyro2_e, 3);
    _tta_put_float_array(buf, 168, backup_gyro_e, 3);
    _tta_put_float_array(buf, 180, main_acc1_e, 3);
    _tta_put_float_array(buf, 192, main_acc2_e, 3);
    _tta_put_float_array(buf, 204, backup_acc_e, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN);
#else
    ttalink_imu_sensors_data_t packet;
    packet.main_update_time = main_update_time;
    packet.backeup_update_time = backeup_update_time;
    tta_array_memcpy(packet.main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1_var, main_gyro1_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2_var, main_gyro2_var, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro_var, backup_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1_var, main_acc1_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2_var, main_acc2_var, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc_var, backup_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1_e, main_gyro1_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2_e, main_gyro2_e, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro_e, backup_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1_e, main_acc1_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2_e, main_acc2_e, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc_e, backup_acc_e, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IMU_SENSORS_DATA;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a imu_sensors_data message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1_var  .
 * @param main_gyro2_var  .
 * @param backup_gyro_var  .
 * @param main_acc1_var  .
 * @param main_acc2_var  .
 * @param backup_acc_var  .
 * @param main_gyro1_e  .
 * @param main_gyro2_e  .
 * @param backup_gyro_e  .
 * @param main_acc1_e  .
 * @param main_acc2_e  .
 * @param backup_acc_e  .
 * @param main_update_time  .
 * @param backeup_update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_data_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time)
{
    return _ttalink_imu_sensors_data_pack(dst_addr, src_addr, msg,  main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, false);
}

/**
 * @brief Pack a imu_sensors_data message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1_var  .
 * @param main_gyro2_var  .
 * @param backup_gyro_var  .
 * @param main_acc1_var  .
 * @param main_acc2_var  .
 * @param backup_acc_var  .
 * @param main_gyro1_e  .
 * @param main_gyro2_e  .
 * @param backup_gyro_e  .
 * @param main_acc1_e  .
 * @param main_acc2_e  .
 * @param backup_acc_e  .
 * @param main_update_time  .
 * @param backeup_update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_data_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time)
{
    return _ttalink_imu_sensors_data_pack(dst_addr, src_addr, msg,  main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, true);
}


static inline uint16_t _ttalink_imu_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *main_gyro1,const float *main_gyro2,const float *backup_gyro,const float *main_acc1,const float *main_acc2,const float *backup_acc,const float *main_gyro1_var,const float *main_gyro2_var,const float *backup_gyro_var,const float *main_acc1_var,const float *main_acc2_var,const float *backup_acc_var,const float *main_gyro1_e,const float *main_gyro2_e,const float *backup_gyro_e,const float *main_acc1_e,const float *main_acc2_e,const float *backup_acc_e,int64_t main_update_time,int64_t backeup_update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN];
    _tta_put_int64_t(buf, 216, main_update_time);
    _tta_put_int64_t(buf, 224, backeup_update_time);
    _tta_put_float_array(buf, 0, main_gyro1, 3);
    _tta_put_float_array(buf, 12, main_gyro2, 3);
    _tta_put_float_array(buf, 24, backup_gyro, 3);
    _tta_put_float_array(buf, 36, main_acc1, 3);
    _tta_put_float_array(buf, 48, main_acc2, 3);
    _tta_put_float_array(buf, 60, backup_acc, 3);
    _tta_put_float_array(buf, 72, main_gyro1_var, 3);
    _tta_put_float_array(buf, 84, main_gyro2_var, 3);
    _tta_put_float_array(buf, 96, backup_gyro_var, 3);
    _tta_put_float_array(buf, 108, main_acc1_var, 3);
    _tta_put_float_array(buf, 120, main_acc2_var, 3);
    _tta_put_float_array(buf, 132, backup_acc_var, 3);
    _tta_put_float_array(buf, 144, main_gyro1_e, 3);
    _tta_put_float_array(buf, 156, main_gyro2_e, 3);
    _tta_put_float_array(buf, 168, backup_gyro_e, 3);
    _tta_put_float_array(buf, 180, main_acc1_e, 3);
    _tta_put_float_array(buf, 192, main_acc2_e, 3);
    _tta_put_float_array(buf, 204, backup_acc_e, 3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN);
#else
    ttalink_imu_sensors_data_t packet;
    packet.main_update_time = main_update_time;
    packet.backeup_update_time = backeup_update_time;
    tta_array_memcpy(packet.main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1_var, main_gyro1_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2_var, main_gyro2_var, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro_var, backup_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1_var, main_acc1_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2_var, main_acc2_var, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc_var, backup_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1_e, main_gyro1_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2_e, main_gyro2_e, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro_e, backup_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1_e, main_acc1_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2_e, main_acc2_e, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc_e, backup_acc_e, sizeof(float)*3);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_IMU_SENSORS_DATA;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC, nocrc);
}

/**
 * @brief Pack a imu_sensors_data message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1_var  .
 * @param main_gyro2_var  .
 * @param backup_gyro_var  .
 * @param main_acc1_var  .
 * @param main_acc2_var  .
 * @param backup_acc_var  .
 * @param main_gyro1_e  .
 * @param main_gyro2_e  .
 * @param backup_gyro_e  .
 * @param main_acc1_e  .
 * @param main_acc2_e  .
 * @param backup_acc_e  .
 * @param main_update_time  .
 * @param backeup_update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_data_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *main_gyro1,const float *main_gyro2,const float *backup_gyro,const float *main_acc1,const float *main_acc2,const float *backup_acc,const float *main_gyro1_var,const float *main_gyro2_var,const float *backup_gyro_var,const float *main_acc1_var,const float *main_acc2_var,const float *backup_acc_var,const float *main_gyro1_e,const float *main_gyro2_e,const float *backup_gyro_e,const float *main_acc1_e,const float *main_acc2_e,const float *backup_acc_e,int64_t main_update_time,int64_t backeup_update_time)
{
    return _ttalink_imu_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, false);
}

/**
 * @brief Pack a imu_sensors_data message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1_var  .
 * @param main_gyro2_var  .
 * @param backup_gyro_var  .
 * @param main_acc1_var  .
 * @param main_acc2_var  .
 * @param backup_acc_var  .
 * @param main_gyro1_e  .
 * @param main_gyro2_e  .
 * @param backup_gyro_e  .
 * @param main_acc1_e  .
 * @param main_acc2_e  .
 * @param backup_acc_e  .
 * @param main_update_time  .
 * @param backeup_update_time  .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_imu_sensors_data_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   const float *main_gyro1,const float *main_gyro2,const float *backup_gyro,const float *main_acc1,const float *main_acc2,const float *backup_acc,const float *main_gyro1_var,const float *main_gyro2_var,const float *backup_gyro_var,const float *main_acc1_var,const float *main_acc2_var,const float *backup_acc_var,const float *main_gyro1_e,const float *main_gyro2_e,const float *backup_gyro_e,const float *main_acc1_e,const float *main_acc2_e,const float *backup_acc_e,int64_t main_update_time,int64_t backeup_update_time)
{
    return _ttalink_imu_sensors_data_pack_chan(dst_addr, src_addr, chan, msg,  main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, true);
}


static inline uint16_t _ttalink_imu_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_imu_sensors_data_t* imu_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_imu_sensors_data_pack_nocrc(dst_addr, src_addr, msg, imu_sensors_data->main_gyro1, imu_sensors_data->main_gyro2, imu_sensors_data->backup_gyro, imu_sensors_data->main_acc1, imu_sensors_data->main_acc2, imu_sensors_data->backup_acc, imu_sensors_data->main_gyro1_var, imu_sensors_data->main_gyro2_var, imu_sensors_data->backup_gyro_var, imu_sensors_data->main_acc1_var, imu_sensors_data->main_acc2_var, imu_sensors_data->backup_acc_var, imu_sensors_data->main_gyro1_e, imu_sensors_data->main_gyro2_e, imu_sensors_data->backup_gyro_e, imu_sensors_data->main_acc1_e, imu_sensors_data->main_acc2_e, imu_sensors_data->backup_acc_e, imu_sensors_data->main_update_time, imu_sensors_data->backeup_update_time);
    }else{
        return ttalink_imu_sensors_data_pack(dst_addr, src_addr, msg, imu_sensors_data->main_gyro1, imu_sensors_data->main_gyro2, imu_sensors_data->backup_gyro, imu_sensors_data->main_acc1, imu_sensors_data->main_acc2, imu_sensors_data->backup_acc, imu_sensors_data->main_gyro1_var, imu_sensors_data->main_gyro2_var, imu_sensors_data->backup_gyro_var, imu_sensors_data->main_acc1_var, imu_sensors_data->main_acc2_var, imu_sensors_data->backup_acc_var, imu_sensors_data->main_gyro1_e, imu_sensors_data->main_gyro2_e, imu_sensors_data->backup_gyro_e, imu_sensors_data->main_acc1_e, imu_sensors_data->main_acc2_e, imu_sensors_data->backup_acc_e, imu_sensors_data->main_update_time, imu_sensors_data->backeup_update_time);
    }
    
}

/**
 * @brief Encode a imu_sensors_data struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_data_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_imu_sensors_data_t* imu_sensors_data)
{
    return _ttalink_imu_sensors_data_encode(dst_addr, src_addr, msg, imu_sensors_data, false);
}

/**
 * @brief Encode a imu_sensors_data struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_data_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_imu_sensors_data_t* imu_sensors_data)
{
    return _ttalink_imu_sensors_data_encode(dst_addr, src_addr, msg, imu_sensors_data, true);
}


static inline uint16_t _ttalink_imu_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_imu_sensors_data_t* imu_sensors_data, bool nocrc)
{
    if(nocrc){
        return ttalink_imu_sensors_data_pack_chan_nocrc(dst_addr, src_addr, chan, msg, imu_sensors_data->main_gyro1, imu_sensors_data->main_gyro2, imu_sensors_data->backup_gyro, imu_sensors_data->main_acc1, imu_sensors_data->main_acc2, imu_sensors_data->backup_acc, imu_sensors_data->main_gyro1_var, imu_sensors_data->main_gyro2_var, imu_sensors_data->backup_gyro_var, imu_sensors_data->main_acc1_var, imu_sensors_data->main_acc2_var, imu_sensors_data->backup_acc_var, imu_sensors_data->main_gyro1_e, imu_sensors_data->main_gyro2_e, imu_sensors_data->backup_gyro_e, imu_sensors_data->main_acc1_e, imu_sensors_data->main_acc2_e, imu_sensors_data->backup_acc_e, imu_sensors_data->main_update_time, imu_sensors_data->backeup_update_time);
    }else{
        return ttalink_imu_sensors_data_pack_chan(dst_addr, src_addr, chan, msg, imu_sensors_data->main_gyro1, imu_sensors_data->main_gyro2, imu_sensors_data->backup_gyro, imu_sensors_data->main_acc1, imu_sensors_data->main_acc2, imu_sensors_data->backup_acc, imu_sensors_data->main_gyro1_var, imu_sensors_data->main_gyro2_var, imu_sensors_data->backup_gyro_var, imu_sensors_data->main_acc1_var, imu_sensors_data->main_acc2_var, imu_sensors_data->backup_acc_var, imu_sensors_data->main_gyro1_e, imu_sensors_data->main_gyro2_e, imu_sensors_data->backup_gyro_e, imu_sensors_data->main_acc1_e, imu_sensors_data->main_acc2_e, imu_sensors_data->backup_acc_e, imu_sensors_data->main_update_time, imu_sensors_data->backeup_update_time);
    }
}

/**
 * @brief Encode a imu_sensors_data struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_data_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_imu_sensors_data_t* imu_sensors_data)
{
    return _ttalink_imu_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, imu_sensors_data, false);
}

/**
 * @brief Encode a imu_sensors_data struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param imu_sensors_data C-struct to read the message contents from
 */
static inline uint16_t ttalink_imu_sensors_data_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_imu_sensors_data_t* imu_sensors_data)
{
    return _ttalink_imu_sensors_data_encode_chan(dst_addr, src_addr, chan, msg, imu_sensors_data, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_imu_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN];
    _tta_put_int64_t(buf, 216, main_update_time);
    _tta_put_int64_t(buf, 224, backeup_update_time);
    _tta_put_float_array(buf, 0, main_gyro1, 3);
    _tta_put_float_array(buf, 12, main_gyro2, 3);
    _tta_put_float_array(buf, 24, backup_gyro, 3);
    _tta_put_float_array(buf, 36, main_acc1, 3);
    _tta_put_float_array(buf, 48, main_acc2, 3);
    _tta_put_float_array(buf, 60, backup_acc, 3);
    _tta_put_float_array(buf, 72, main_gyro1_var, 3);
    _tta_put_float_array(buf, 84, main_gyro2_var, 3);
    _tta_put_float_array(buf, 96, backup_gyro_var, 3);
    _tta_put_float_array(buf, 108, main_acc1_var, 3);
    _tta_put_float_array(buf, 120, main_acc2_var, 3);
    _tta_put_float_array(buf, 132, backup_acc_var, 3);
    _tta_put_float_array(buf, 144, main_gyro1_e, 3);
    _tta_put_float_array(buf, 156, main_gyro2_e, 3);
    _tta_put_float_array(buf, 168, backup_gyro_e, 3);
    _tta_put_float_array(buf, 180, main_acc1_e, 3);
    _tta_put_float_array(buf, 192, main_acc2_e, 3);
    _tta_put_float_array(buf, 204, backup_acc_e, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_DATA, buf, TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_imu_sensors_data_t packet;
    packet.main_update_time = main_update_time;
    packet.backeup_update_time = backeup_update_time;
    tta_array_memcpy(packet.main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1_var, main_gyro1_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2_var, main_gyro2_var, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro_var, backup_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1_var, main_acc1_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2_var, main_acc2_var, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc_var, backup_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro1_e, main_gyro1_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_gyro2_e, main_gyro2_e, sizeof(float)*3);
    tta_array_memcpy(packet.backup_gyro_e, backup_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc1_e, main_acc1_e, sizeof(float)*3);
    tta_array_memcpy(packet.main_acc2_e, main_acc2_e, sizeof(float)*3);
    tta_array_memcpy(packet.backup_acc_e, backup_acc_e, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_DATA, (const char *)&packet, TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a imu_sensors_data message
 * @param chan TTAlink channel to send the message
 *
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1_var  .
 * @param main_gyro2_var  .
 * @param backup_gyro_var  .
 * @param main_acc1_var  .
 * @param main_acc2_var  .
 * @param backup_acc_var  .
 * @param main_gyro1_e  .
 * @param main_gyro2_e  .
 * @param backup_gyro_e  .
 * @param main_acc1_e  .
 * @param main_acc2_e  .
 * @param backup_acc_e  .
 * @param main_update_time  .
 * @param backeup_update_time  .
 */
static inline void ttalink_imu_sensors_data_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time)
{
    _ttalink_imu_sensors_data_send(dst_addr, src_addr, chan, main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, false);
}

/**
 * @brief Send a imu_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param main_gyro1  .
 * @param main_gyro2  .
 * @param backup_gyro  .
 * @param main_acc1  .
 * @param main_acc2  .
 * @param backup_acc  .
 * @param main_gyro1_var  .
 * @param main_gyro2_var  .
 * @param backup_gyro_var  .
 * @param main_acc1_var  .
 * @param main_acc2_var  .
 * @param backup_acc_var  .
 * @param main_gyro1_e  .
 * @param main_gyro2_e  .
 * @param backup_gyro_e  .
 * @param main_acc1_e  .
 * @param main_acc2_e  .
 * @param backup_acc_e  .
 * @param main_update_time  .
 * @param backeup_update_time  .
 */
static inline void ttalink_imu_sensors_data_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time)
{
    _ttalink_imu_sensors_data_send(dst_addr, src_addr, chan, main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, true);
}


static inline void _ttalink_imu_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_imu_sensors_data_t* imu_sensors_data, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_imu_sensors_data_send_nocrc(dst_addr, src_addr, chan, imu_sensors_data->main_gyro1, imu_sensors_data->main_gyro2, imu_sensors_data->backup_gyro, imu_sensors_data->main_acc1, imu_sensors_data->main_acc2, imu_sensors_data->backup_acc, imu_sensors_data->main_gyro1_var, imu_sensors_data->main_gyro2_var, imu_sensors_data->backup_gyro_var, imu_sensors_data->main_acc1_var, imu_sensors_data->main_acc2_var, imu_sensors_data->backup_acc_var, imu_sensors_data->main_gyro1_e, imu_sensors_data->main_gyro2_e, imu_sensors_data->backup_gyro_e, imu_sensors_data->main_acc1_e, imu_sensors_data->main_acc2_e, imu_sensors_data->backup_acc_e, imu_sensors_data->main_update_time, imu_sensors_data->backeup_update_time);
    }else{
        ttalink_imu_sensors_data_send(dst_addr, src_addr, chan, imu_sensors_data->main_gyro1, imu_sensors_data->main_gyro2, imu_sensors_data->backup_gyro, imu_sensors_data->main_acc1, imu_sensors_data->main_acc2, imu_sensors_data->backup_acc, imu_sensors_data->main_gyro1_var, imu_sensors_data->main_gyro2_var, imu_sensors_data->backup_gyro_var, imu_sensors_data->main_acc1_var, imu_sensors_data->main_acc2_var, imu_sensors_data->backup_acc_var, imu_sensors_data->main_gyro1_e, imu_sensors_data->main_gyro2_e, imu_sensors_data->backup_gyro_e, imu_sensors_data->main_acc1_e, imu_sensors_data->main_acc2_e, imu_sensors_data->backup_acc_e, imu_sensors_data->main_update_time, imu_sensors_data->backeup_update_time);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_DATA, (const char *)imu_sensors_data, TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC, nocrc);
#endif
}

/**
 * @brief Send a imu_sensors_data message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_imu_sensors_data_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_imu_sensors_data_t* imu_sensors_data)
{
    _ttalink_imu_sensors_data_send_struct(dst_addr, src_addr, chan, imu_sensors_data, false);
}

/**
 * @brief Send a imu_sensors_data message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_imu_sensors_data_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_imu_sensors_data_t* imu_sensors_data)
{
    _ttalink_imu_sensors_data_send_struct(dst_addr, src_addr, chan, imu_sensors_data, true);
}

#if TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_imu_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 216, main_update_time);
    _tta_put_int64_t(buf, 224, backeup_update_time);
    _tta_put_float_array(buf, 0, main_gyro1, 3);
    _tta_put_float_array(buf, 12, main_gyro2, 3);
    _tta_put_float_array(buf, 24, backup_gyro, 3);
    _tta_put_float_array(buf, 36, main_acc1, 3);
    _tta_put_float_array(buf, 48, main_acc2, 3);
    _tta_put_float_array(buf, 60, backup_acc, 3);
    _tta_put_float_array(buf, 72, main_gyro1_var, 3);
    _tta_put_float_array(buf, 84, main_gyro2_var, 3);
    _tta_put_float_array(buf, 96, backup_gyro_var, 3);
    _tta_put_float_array(buf, 108, main_acc1_var, 3);
    _tta_put_float_array(buf, 120, main_acc2_var, 3);
    _tta_put_float_array(buf, 132, backup_acc_var, 3);
    _tta_put_float_array(buf, 144, main_gyro1_e, 3);
    _tta_put_float_array(buf, 156, main_gyro2_e, 3);
    _tta_put_float_array(buf, 168, backup_gyro_e, 3);
    _tta_put_float_array(buf, 180, main_acc1_e, 3);
    _tta_put_float_array(buf, 192, main_acc2_e, 3);
    _tta_put_float_array(buf, 204, backup_acc_e, 3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_DATA, buf, TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC, nocrc);
#else
    ttalink_imu_sensors_data_t *packet = (ttalink_imu_sensors_data_t *)msgbuf;
    packet->main_update_time = main_update_time;
    packet->backeup_update_time = backeup_update_time;
    tta_array_memcpy(packet->main_gyro1, main_gyro1, sizeof(float)*3);
    tta_array_memcpy(packet->main_gyro2, main_gyro2, sizeof(float)*3);
    tta_array_memcpy(packet->backup_gyro, backup_gyro, sizeof(float)*3);
    tta_array_memcpy(packet->main_acc1, main_acc1, sizeof(float)*3);
    tta_array_memcpy(packet->main_acc2, main_acc2, sizeof(float)*3);
    tta_array_memcpy(packet->backup_acc, backup_acc, sizeof(float)*3);
    tta_array_memcpy(packet->main_gyro1_var, main_gyro1_var, sizeof(float)*3);
    tta_array_memcpy(packet->main_gyro2_var, main_gyro2_var, sizeof(float)*3);
    tta_array_memcpy(packet->backup_gyro_var, backup_gyro_var, sizeof(float)*3);
    tta_array_memcpy(packet->main_acc1_var, main_acc1_var, sizeof(float)*3);
    tta_array_memcpy(packet->main_acc2_var, main_acc2_var, sizeof(float)*3);
    tta_array_memcpy(packet->backup_acc_var, backup_acc_var, sizeof(float)*3);
    tta_array_memcpy(packet->main_gyro1_e, main_gyro1_e, sizeof(float)*3);
    tta_array_memcpy(packet->main_gyro2_e, main_gyro2_e, sizeof(float)*3);
    tta_array_memcpy(packet->backup_gyro_e, backup_gyro_e, sizeof(float)*3);
    tta_array_memcpy(packet->main_acc1_e, main_acc1_e, sizeof(float)*3);
    tta_array_memcpy(packet->main_acc2_e, main_acc2_e, sizeof(float)*3);
    tta_array_memcpy(packet->backup_acc_e, backup_acc_e, sizeof(float)*3);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_IMU_SENSORS_DATA, (const char *)packet, TTALINK_MSG_ID_IMU_SENSORS_DATA_MIN_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN, TTALINK_MSG_ID_IMU_SENSORS_DATA_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_imu_sensors_data_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time)
{
    _ttalink_imu_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_imu_sensors_data_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  const float *main_gyro1, const float *main_gyro2, const float *backup_gyro, const float *main_acc1, const float *main_acc2, const float *backup_acc, const float *main_gyro1_var, const float *main_gyro2_var, const float *backup_gyro_var, const float *main_acc1_var, const float *main_acc2_var, const float *backup_acc_var, const float *main_gyro1_e, const float *main_gyro2_e, const float *backup_gyro_e, const float *main_acc1_e, const float *main_acc2_e, const float *backup_acc_e, int64_t main_update_time, int64_t backeup_update_time)
{
    _ttalink_imu_sensors_data_send_buf(dst_addr, src_addr, msgbuf, chan, main_gyro1, main_gyro2, backup_gyro, main_acc1, main_acc2, backup_acc, main_gyro1_var, main_gyro2_var, backup_gyro_var, main_acc1_var, main_acc2_var, backup_acc_var, main_gyro1_e, main_gyro2_e, backup_gyro_e, main_acc1_e, main_acc2_e, backup_acc_e, main_update_time, backeup_update_time, true);
}
#endif

#endif

// MESSAGE IMU_SENSORS_DATA UNPACKING


/**
 * @brief Get field main_gyro1 from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_gyro1(const ttalink_message_t* msg, float *main_gyro1)
{
    return _TTA_RETURN_float_array(msg, main_gyro1, 3,  0);
}

/**
 * @brief Get field main_gyro2 from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_gyro2(const ttalink_message_t* msg, float *main_gyro2)
{
    return _TTA_RETURN_float_array(msg, main_gyro2, 3,  12);
}

/**
 * @brief Get field backup_gyro from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_backup_gyro(const ttalink_message_t* msg, float *backup_gyro)
{
    return _TTA_RETURN_float_array(msg, backup_gyro, 3,  24);
}

/**
 * @brief Get field main_acc1 from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_acc1(const ttalink_message_t* msg, float *main_acc1)
{
    return _TTA_RETURN_float_array(msg, main_acc1, 3,  36);
}

/**
 * @brief Get field main_acc2 from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_acc2(const ttalink_message_t* msg, float *main_acc2)
{
    return _TTA_RETURN_float_array(msg, main_acc2, 3,  48);
}

/**
 * @brief Get field backup_acc from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_backup_acc(const ttalink_message_t* msg, float *backup_acc)
{
    return _TTA_RETURN_float_array(msg, backup_acc, 3,  60);
}

/**
 * @brief Get field main_gyro1_var from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_gyro1_var(const ttalink_message_t* msg, float *main_gyro1_var)
{
    return _TTA_RETURN_float_array(msg, main_gyro1_var, 3,  72);
}

/**
 * @brief Get field main_gyro2_var from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_gyro2_var(const ttalink_message_t* msg, float *main_gyro2_var)
{
    return _TTA_RETURN_float_array(msg, main_gyro2_var, 3,  84);
}

/**
 * @brief Get field backup_gyro_var from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_backup_gyro_var(const ttalink_message_t* msg, float *backup_gyro_var)
{
    return _TTA_RETURN_float_array(msg, backup_gyro_var, 3,  96);
}

/**
 * @brief Get field main_acc1_var from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_acc1_var(const ttalink_message_t* msg, float *main_acc1_var)
{
    return _TTA_RETURN_float_array(msg, main_acc1_var, 3,  108);
}

/**
 * @brief Get field main_acc2_var from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_acc2_var(const ttalink_message_t* msg, float *main_acc2_var)
{
    return _TTA_RETURN_float_array(msg, main_acc2_var, 3,  120);
}

/**
 * @brief Get field backup_acc_var from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_backup_acc_var(const ttalink_message_t* msg, float *backup_acc_var)
{
    return _TTA_RETURN_float_array(msg, backup_acc_var, 3,  132);
}

/**
 * @brief Get field main_gyro1_e from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_gyro1_e(const ttalink_message_t* msg, float *main_gyro1_e)
{
    return _TTA_RETURN_float_array(msg, main_gyro1_e, 3,  144);
}

/**
 * @brief Get field main_gyro2_e from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_gyro2_e(const ttalink_message_t* msg, float *main_gyro2_e)
{
    return _TTA_RETURN_float_array(msg, main_gyro2_e, 3,  156);
}

/**
 * @brief Get field backup_gyro_e from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_backup_gyro_e(const ttalink_message_t* msg, float *backup_gyro_e)
{
    return _TTA_RETURN_float_array(msg, backup_gyro_e, 3,  168);
}

/**
 * @brief Get field main_acc1_e from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_acc1_e(const ttalink_message_t* msg, float *main_acc1_e)
{
    return _TTA_RETURN_float_array(msg, main_acc1_e, 3,  180);
}

/**
 * @brief Get field main_acc2_e from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_main_acc2_e(const ttalink_message_t* msg, float *main_acc2_e)
{
    return _TTA_RETURN_float_array(msg, main_acc2_e, 3,  192);
}

/**
 * @brief Get field backup_acc_e from imu_sensors_data message
 *
 * @return  .
 */
static inline uint16_t ttalink_imu_sensors_data_get_backup_acc_e(const ttalink_message_t* msg, float *backup_acc_e)
{
    return _TTA_RETURN_float_array(msg, backup_acc_e, 3,  204);
}

/**
 * @brief Get field main_update_time from imu_sensors_data message
 *
 * @return  .
 */
static inline int64_t ttalink_imu_sensors_data_get_main_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  216);
}

/**
 * @brief Get field backeup_update_time from imu_sensors_data message
 *
 * @return  .
 */
static inline int64_t ttalink_imu_sensors_data_get_backeup_update_time(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  224);
}

/**
 * @brief Decode a imu_sensors_data message into a struct
 *
 * @param msg The message to decode
 * @param imu_sensors_data C-struct to decode the message contents into
 */
static inline void ttalink_imu_sensors_data_decode(const ttalink_message_t* msg, ttalink_imu_sensors_data_t* imu_sensors_data)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    ttalink_imu_sensors_data_get_main_gyro1(msg, imu_sensors_data->main_gyro1);
    ttalink_imu_sensors_data_get_main_gyro2(msg, imu_sensors_data->main_gyro2);
    ttalink_imu_sensors_data_get_backup_gyro(msg, imu_sensors_data->backup_gyro);
    ttalink_imu_sensors_data_get_main_acc1(msg, imu_sensors_data->main_acc1);
    ttalink_imu_sensors_data_get_main_acc2(msg, imu_sensors_data->main_acc2);
    ttalink_imu_sensors_data_get_backup_acc(msg, imu_sensors_data->backup_acc);
    ttalink_imu_sensors_data_get_main_gyro1_var(msg, imu_sensors_data->main_gyro1_var);
    ttalink_imu_sensors_data_get_main_gyro2_var(msg, imu_sensors_data->main_gyro2_var);
    ttalink_imu_sensors_data_get_backup_gyro_var(msg, imu_sensors_data->backup_gyro_var);
    ttalink_imu_sensors_data_get_main_acc1_var(msg, imu_sensors_data->main_acc1_var);
    ttalink_imu_sensors_data_get_main_acc2_var(msg, imu_sensors_data->main_acc2_var);
    ttalink_imu_sensors_data_get_backup_acc_var(msg, imu_sensors_data->backup_acc_var);
    ttalink_imu_sensors_data_get_main_gyro1_e(msg, imu_sensors_data->main_gyro1_e);
    ttalink_imu_sensors_data_get_main_gyro2_e(msg, imu_sensors_data->main_gyro2_e);
    ttalink_imu_sensors_data_get_backup_gyro_e(msg, imu_sensors_data->backup_gyro_e);
    ttalink_imu_sensors_data_get_main_acc1_e(msg, imu_sensors_data->main_acc1_e);
    ttalink_imu_sensors_data_get_main_acc2_e(msg, imu_sensors_data->main_acc2_e);
    ttalink_imu_sensors_data_get_backup_acc_e(msg, imu_sensors_data->backup_acc_e);
    imu_sensors_data->main_update_time = ttalink_imu_sensors_data_get_main_update_time(msg);
    imu_sensors_data->backeup_update_time = ttalink_imu_sensors_data_get_backeup_update_time(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN? msg->len : TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN;
        memset(imu_sensors_data, 0, TTALINK_MSG_ID_IMU_SENSORS_DATA_LEN);
    memcpy(imu_sensors_data, _TTA_PAYLOAD(msg), len);
#endif
}
