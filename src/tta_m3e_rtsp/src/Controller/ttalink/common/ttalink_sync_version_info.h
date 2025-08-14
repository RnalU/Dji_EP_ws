#pragma once
// MESSAGE SYNC_VERSION_INFO PACKING

#define TTALINK_MSG_ID_SYNC_VERSION_INFO 2101

TTAPACKED(
typedef struct __ttalink_sync_version_info_t {
 uint64_t sn; /*<  . .*/
 uint32_t version_farame; /*<  . .*/
 uint32_t version_driver; /*<  . .*/
 uint32_t version_soft_a; /*<  . .*/
 uint32_t version_soft_b; /*<  . .*/
 uint32_t version_soft_c; /*<  . .*/
 uint32_t version_soft_svn; /*<  . .*/
 uint32_t version_release; /*<  . .*/
 uint8_t version[64]; /*<  . .*/
}) ttalink_sync_version_info_t;

#define TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN 100
#define TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN 100
#define TTALINK_MSG_ID_2101_LEN 100
#define TTALINK_MSG_ID_2101_MIN_LEN 100

#define TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC 148
#define TTALINK_MSG_ID_2101_CRC 148

#define TTALINK_MSG_SYNC_VERSION_INFO_FIELD_VERSION_LEN 64

#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_SYNC_VERSION_INFO { \
    2101, \
    "SYNC_VERSION_INFO", \
    9, \
    {  { "sn", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_sync_version_info_t, sn) }, \
         { "version_farame", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_sync_version_info_t, version_farame) }, \
         { "version_driver", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_sync_version_info_t, version_driver) }, \
         { "version_soft_a", NULL, TTALINK_TYPE_UINT32_T, 0, 16, offsetof(ttalink_sync_version_info_t, version_soft_a) }, \
         { "version_soft_b", NULL, TTALINK_TYPE_UINT32_T, 0, 20, offsetof(ttalink_sync_version_info_t, version_soft_b) }, \
         { "version_soft_c", NULL, TTALINK_TYPE_UINT32_T, 0, 24, offsetof(ttalink_sync_version_info_t, version_soft_c) }, \
         { "version_soft_svn", NULL, TTALINK_TYPE_UINT32_T, 0, 28, offsetof(ttalink_sync_version_info_t, version_soft_svn) }, \
         { "version_release", NULL, TTALINK_TYPE_UINT32_T, 0, 32, offsetof(ttalink_sync_version_info_t, version_release) }, \
         { "version", NULL, TTALINK_TYPE_UINT8_T, 64, 36, offsetof(ttalink_sync_version_info_t, version) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_SYNC_VERSION_INFO { \
    "SYNC_VERSION_INFO", \
    9, \
    {  { "sn", NULL, TTALINK_TYPE_UINT64_T, 0, 0, offsetof(ttalink_sync_version_info_t, sn) }, \
         { "version_farame", NULL, TTALINK_TYPE_UINT32_T, 0, 8, offsetof(ttalink_sync_version_info_t, version_farame) }, \
         { "version_driver", NULL, TTALINK_TYPE_UINT32_T, 0, 12, offsetof(ttalink_sync_version_info_t, version_driver) }, \
         { "version_soft_a", NULL, TTALINK_TYPE_UINT32_T, 0, 16, offsetof(ttalink_sync_version_info_t, version_soft_a) }, \
         { "version_soft_b", NULL, TTALINK_TYPE_UINT32_T, 0, 20, offsetof(ttalink_sync_version_info_t, version_soft_b) }, \
         { "version_soft_c", NULL, TTALINK_TYPE_UINT32_T, 0, 24, offsetof(ttalink_sync_version_info_t, version_soft_c) }, \
         { "version_soft_svn", NULL, TTALINK_TYPE_UINT32_T, 0, 28, offsetof(ttalink_sync_version_info_t, version_soft_svn) }, \
         { "version_release", NULL, TTALINK_TYPE_UINT32_T, 0, 32, offsetof(ttalink_sync_version_info_t, version_release) }, \
         { "version", NULL, TTALINK_TYPE_UINT8_T, 64, 36, offsetof(ttalink_sync_version_info_t, version) }, \
         } \
}
#endif


static inline uint16_t _ttalink_sync_version_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN];
    _tta_put_uint64_t(buf, 0, sn);
    _tta_put_uint32_t(buf, 8, version_farame);
    _tta_put_uint32_t(buf, 12, version_driver);
    _tta_put_uint32_t(buf, 16, version_soft_a);
    _tta_put_uint32_t(buf, 20, version_soft_b);
    _tta_put_uint32_t(buf, 24, version_soft_c);
    _tta_put_uint32_t(buf, 28, version_soft_svn);
    _tta_put_uint32_t(buf, 32, version_release);
    _tta_put_uint8_t_array(buf, 36, version, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN);
#else
    ttalink_sync_version_info_t packet;
    packet.sn = sn;
    packet.version_farame = version_farame;
    packet.version_driver = version_driver;
    packet.version_soft_a = version_soft_a;
    packet.version_soft_b = version_soft_b;
    packet.version_soft_c = version_soft_c;
    packet.version_soft_svn = version_soft_svn;
    packet.version_release = version_release;
    tta_array_memcpy(packet.version, version, sizeof(uint8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_VERSION_INFO;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_version_info message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn  . .
 * @param version_farame  . .
 * @param version_driver  . .
 * @param version_soft_a  . .
 * @param version_soft_b  . .
 * @param version_soft_c  . .
 * @param version_soft_svn  . .
 * @param version_release  . .
 * @param version  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_version_info_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version)
{
    return _ttalink_sync_version_info_pack(dst_addr, src_addr, msg,  sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, false);
}

/**
 * @brief Pack a sync_version_info message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn  . .
 * @param version_farame  . .
 * @param version_driver  . .
 * @param version_soft_a  . .
 * @param version_soft_b  . .
 * @param version_soft_c  . .
 * @param version_soft_svn  . .
 * @param version_release  . .
 * @param version  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_version_info_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version)
{
    return _ttalink_sync_version_info_pack(dst_addr, src_addr, msg,  sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, true);
}


static inline uint16_t _ttalink_sync_version_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t sn,uint32_t version_farame,uint32_t version_driver,uint32_t version_soft_a,uint32_t version_soft_b,uint32_t version_soft_c,uint32_t version_soft_svn,uint32_t version_release,const uint8_t *version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN];
    _tta_put_uint64_t(buf, 0, sn);
    _tta_put_uint32_t(buf, 8, version_farame);
    _tta_put_uint32_t(buf, 12, version_driver);
    _tta_put_uint32_t(buf, 16, version_soft_a);
    _tta_put_uint32_t(buf, 20, version_soft_b);
    _tta_put_uint32_t(buf, 24, version_soft_c);
    _tta_put_uint32_t(buf, 28, version_soft_svn);
    _tta_put_uint32_t(buf, 32, version_release);
    _tta_put_uint8_t_array(buf, 36, version, 64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN);
#else
    ttalink_sync_version_info_t packet;
    packet.sn = sn;
    packet.version_farame = version_farame;
    packet.version_driver = version_driver;
    packet.version_soft_a = version_soft_a;
    packet.version_soft_b = version_soft_b;
    packet.version_soft_c = version_soft_c;
    packet.version_soft_svn = version_soft_svn;
    packet.version_release = version_release;
    tta_array_memcpy(packet.version, version, sizeof(uint8_t)*64);
        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_SYNC_VERSION_INFO;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC, nocrc);
}

/**
 * @brief Pack a sync_version_info message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn  . .
 * @param version_farame  . .
 * @param version_driver  . .
 * @param version_soft_a  . .
 * @param version_soft_b  . .
 * @param version_soft_c  . .
 * @param version_soft_svn  . .
 * @param version_release  . .
 * @param version  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_version_info_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t sn,uint32_t version_farame,uint32_t version_driver,uint32_t version_soft_a,uint32_t version_soft_b,uint32_t version_soft_c,uint32_t version_soft_svn,uint32_t version_release,const uint8_t *version)
{
    return _ttalink_sync_version_info_pack_chan(dst_addr, src_addr, chan, msg,  sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, false);
}

/**
 * @brief Pack a sync_version_info message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn  . .
 * @param version_farame  . .
 * @param version_driver  . .
 * @param version_soft_a  . .
 * @param version_soft_b  . .
 * @param version_soft_c  . .
 * @param version_soft_svn  . .
 * @param version_release  . .
 * @param version  . .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_sync_version_info_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint64_t sn,uint32_t version_farame,uint32_t version_driver,uint32_t version_soft_a,uint32_t version_soft_b,uint32_t version_soft_c,uint32_t version_soft_svn,uint32_t version_release,const uint8_t *version)
{
    return _ttalink_sync_version_info_pack_chan(dst_addr, src_addr, chan, msg,  sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, true);
}


static inline uint16_t _ttalink_sync_version_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_version_info_t* sync_version_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_version_info_pack_nocrc(dst_addr, src_addr, msg, sync_version_info->sn, sync_version_info->version_farame, sync_version_info->version_driver, sync_version_info->version_soft_a, sync_version_info->version_soft_b, sync_version_info->version_soft_c, sync_version_info->version_soft_svn, sync_version_info->version_release, sync_version_info->version);
    }else{
        return ttalink_sync_version_info_pack(dst_addr, src_addr, msg, sync_version_info->sn, sync_version_info->version_farame, sync_version_info->version_driver, sync_version_info->version_soft_a, sync_version_info->version_soft_b, sync_version_info->version_soft_c, sync_version_info->version_soft_svn, sync_version_info->version_release, sync_version_info->version);
    }
    
}

/**
 * @brief Encode a sync_version_info struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_version_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_version_info_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_version_info_t* sync_version_info)
{
    return _ttalink_sync_version_info_encode(dst_addr, src_addr, msg, sync_version_info, false);
}

/**
 * @brief Encode a sync_version_info struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param sync_version_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_version_info_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_sync_version_info_t* sync_version_info)
{
    return _ttalink_sync_version_info_encode(dst_addr, src_addr, msg, sync_version_info, true);
}


static inline uint16_t _ttalink_sync_version_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_version_info_t* sync_version_info, bool nocrc)
{
    if(nocrc){
        return ttalink_sync_version_info_pack_chan_nocrc(dst_addr, src_addr, chan, msg, sync_version_info->sn, sync_version_info->version_farame, sync_version_info->version_driver, sync_version_info->version_soft_a, sync_version_info->version_soft_b, sync_version_info->version_soft_c, sync_version_info->version_soft_svn, sync_version_info->version_release, sync_version_info->version);
    }else{
        return ttalink_sync_version_info_pack_chan(dst_addr, src_addr, chan, msg, sync_version_info->sn, sync_version_info->version_farame, sync_version_info->version_driver, sync_version_info->version_soft_a, sync_version_info->version_soft_b, sync_version_info->version_soft_c, sync_version_info->version_soft_svn, sync_version_info->version_release, sync_version_info->version);
    }
}

/**
 * @brief Encode a sync_version_info struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_version_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_version_info_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_version_info_t* sync_version_info)
{
    return _ttalink_sync_version_info_encode_chan(dst_addr, src_addr, chan, msg, sync_version_info, false);
}

/**
 * @brief Encode a sync_version_info struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sync_version_info C-struct to read the message contents from
 */
static inline uint16_t ttalink_sync_version_info_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_sync_version_info_t* sync_version_info)
{
    return _ttalink_sync_version_info_encode_chan(dst_addr, src_addr, chan, msg, sync_version_info, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_sync_version_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN];
    _tta_put_uint64_t(buf, 0, sn);
    _tta_put_uint32_t(buf, 8, version_farame);
    _tta_put_uint32_t(buf, 12, version_driver);
    _tta_put_uint32_t(buf, 16, version_soft_a);
    _tta_put_uint32_t(buf, 20, version_soft_b);
    _tta_put_uint32_t(buf, 24, version_soft_c);
    _tta_put_uint32_t(buf, 28, version_soft_svn);
    _tta_put_uint32_t(buf, 32, version_release);
    _tta_put_uint8_t_array(buf, 36, version, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_VERSION_INFO, buf, TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC, nocrc);
#else
    ttalink_sync_version_info_t packet;
    packet.sn = sn;
    packet.version_farame = version_farame;
    packet.version_driver = version_driver;
    packet.version_soft_a = version_soft_a;
    packet.version_soft_b = version_soft_b;
    packet.version_soft_c = version_soft_c;
    packet.version_soft_svn = version_soft_svn;
    packet.version_release = version_release;
    tta_array_memcpy(packet.version, version, sizeof(uint8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_VERSION_INFO, (const char *)&packet, TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_version_info message
 * @param chan TTAlink channel to send the message
 *
 * @param sn  . .
 * @param version_farame  . .
 * @param version_driver  . .
 * @param version_soft_a  . .
 * @param version_soft_b  . .
 * @param version_soft_c  . .
 * @param version_soft_svn  . .
 * @param version_release  . .
 * @param version  . .
 */
static inline void ttalink_sync_version_info_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version)
{
    _ttalink_sync_version_info_send(dst_addr, src_addr, chan, sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, false);
}

/**
 * @brief Send a sync_version_info message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sn  . .
 * @param version_farame  . .
 * @param version_driver  . .
 * @param version_soft_a  . .
 * @param version_soft_b  . .
 * @param version_soft_c  . .
 * @param version_soft_svn  . .
 * @param version_release  . .
 * @param version  . .
 */
static inline void ttalink_sync_version_info_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version)
{
    _ttalink_sync_version_info_send(dst_addr, src_addr, chan, sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, true);
}


static inline void _ttalink_sync_version_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_version_info_t* sync_version_info, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_sync_version_info_send_nocrc(dst_addr, src_addr, chan, sync_version_info->sn, sync_version_info->version_farame, sync_version_info->version_driver, sync_version_info->version_soft_a, sync_version_info->version_soft_b, sync_version_info->version_soft_c, sync_version_info->version_soft_svn, sync_version_info->version_release, sync_version_info->version);
    }else{
        ttalink_sync_version_info_send(dst_addr, src_addr, chan, sync_version_info->sn, sync_version_info->version_farame, sync_version_info->version_driver, sync_version_info->version_soft_a, sync_version_info->version_soft_b, sync_version_info->version_soft_c, sync_version_info->version_soft_svn, sync_version_info->version_release, sync_version_info->version);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_VERSION_INFO, (const char *)sync_version_info, TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC, nocrc);
#endif
}

/**
 * @brief Send a sync_version_info message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_version_info_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_version_info_t* sync_version_info)
{
    _ttalink_sync_version_info_send_struct(dst_addr, src_addr, chan, sync_version_info, false);
}

/**
 * @brief Send a sync_version_info message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_sync_version_info_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_sync_version_info_t* sync_version_info)
{
    _ttalink_sync_version_info_send_struct(dst_addr, src_addr, chan, sync_version_info, true);
}

#if TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_sync_version_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint64_t(buf, 0, sn);
    _tta_put_uint32_t(buf, 8, version_farame);
    _tta_put_uint32_t(buf, 12, version_driver);
    _tta_put_uint32_t(buf, 16, version_soft_a);
    _tta_put_uint32_t(buf, 20, version_soft_b);
    _tta_put_uint32_t(buf, 24, version_soft_c);
    _tta_put_uint32_t(buf, 28, version_soft_svn);
    _tta_put_uint32_t(buf, 32, version_release);
    _tta_put_uint8_t_array(buf, 36, version, 64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_VERSION_INFO, buf, TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC, nocrc);
#else
    ttalink_sync_version_info_t *packet = (ttalink_sync_version_info_t *)msgbuf;
    packet->sn = sn;
    packet->version_farame = version_farame;
    packet->version_driver = version_driver;
    packet->version_soft_a = version_soft_a;
    packet->version_soft_b = version_soft_b;
    packet->version_soft_c = version_soft_c;
    packet->version_soft_svn = version_soft_svn;
    packet->version_release = version_release;
    tta_array_memcpy(packet->version, version, sizeof(uint8_t)*64);
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_SYNC_VERSION_INFO, (const char *)packet, TTALINK_MSG_ID_SYNC_VERSION_INFO_MIN_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN, TTALINK_MSG_ID_SYNC_VERSION_INFO_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_sync_version_info_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version)
{
    _ttalink_sync_version_info_send_buf(dst_addr, src_addr, msgbuf, chan, sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_sync_version_info_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint64_t sn, uint32_t version_farame, uint32_t version_driver, uint32_t version_soft_a, uint32_t version_soft_b, uint32_t version_soft_c, uint32_t version_soft_svn, uint32_t version_release, const uint8_t *version)
{
    _ttalink_sync_version_info_send_buf(dst_addr, src_addr, msgbuf, chan, sn, version_farame, version_driver, version_soft_a, version_soft_b, version_soft_c, version_soft_svn, version_release, version, true);
}
#endif

#endif

// MESSAGE SYNC_VERSION_INFO UNPACKING


/**
 * @brief Get field sn from sync_version_info message
 *
 * @return  . .
 */
static inline uint64_t ttalink_sync_version_info_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field version_farame from sync_version_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_version_info_get_version_farame(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field version_driver from sync_version_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_version_info_get_version_driver(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field version_soft_a from sync_version_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_version_info_get_version_soft_a(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field version_soft_b from sync_version_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_version_info_get_version_soft_b(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field version_soft_c from sync_version_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_version_info_get_version_soft_c(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field version_soft_svn from sync_version_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_version_info_get_version_soft_svn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field version_release from sync_version_info message
 *
 * @return  . .
 */
static inline uint32_t ttalink_sync_version_info_get_version_release(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field version from sync_version_info message
 *
 * @return  . .
 */
static inline uint16_t ttalink_sync_version_info_get_version(const ttalink_message_t* msg, uint8_t *version)
{
    return _TTA_RETURN_uint8_t_array(msg, version, 64,  36);
}

/**
 * @brief Decode a sync_version_info message into a struct
 *
 * @param msg The message to decode
 * @param sync_version_info C-struct to decode the message contents into
 */
static inline void ttalink_sync_version_info_decode(const ttalink_message_t* msg, ttalink_sync_version_info_t* sync_version_info)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    sync_version_info->sn = ttalink_sync_version_info_get_sn(msg);
    sync_version_info->version_farame = ttalink_sync_version_info_get_version_farame(msg);
    sync_version_info->version_driver = ttalink_sync_version_info_get_version_driver(msg);
    sync_version_info->version_soft_a = ttalink_sync_version_info_get_version_soft_a(msg);
    sync_version_info->version_soft_b = ttalink_sync_version_info_get_version_soft_b(msg);
    sync_version_info->version_soft_c = ttalink_sync_version_info_get_version_soft_c(msg);
    sync_version_info->version_soft_svn = ttalink_sync_version_info_get_version_soft_svn(msg);
    sync_version_info->version_release = ttalink_sync_version_info_get_version_release(msg);
    ttalink_sync_version_info_get_version(msg, sync_version_info->version);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN? msg->len : TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN;
        memset(sync_version_info, 0, TTALINK_MSG_ID_SYNC_VERSION_INFO_LEN);
    memcpy(sync_version_info, _TTA_PAYLOAD(msg), len);
#endif
}
