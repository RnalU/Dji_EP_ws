/** @file
 *  @brief TTAlink comm protocol built from common.xml
 *  @see http://ttalink.org
 */
#pragma once
 
#ifndef TTALINK_CHANNEL_H
#define TTALINK_CHANNEL_H

typedef enum {
    TTALINK_CHANNEL_DEFAULT = 0,
    TTALINK_CHANNEL_FC_EMBE = 1,
    TTALINK_CHANNEL_FC_SH = 2,
    TTALINK_CHANNEL_FC_SHARE = 3,
    TTALINK_CHANNEL_FC_TC = 4,
    TTALINK_CHANNEL_UAV_HK = 5,
    TTALINK_CHANNEL_SV_EMBE = 6,
    TTALINK_CHANNEL_TC_EMBE = 7,
    TTALINK_CHANNEL_EMBE_APP = 8,
    TTALINK_CHANNEL_TC_BMS = 9,
    TTALINK_CHANNEL_TC_POD_CAN = 10,
    TTALINK_CHANNEL_TC_POD_UART = 11,
    TTALINK_CHANNEL_TC_RADAR = 12,
    TTALINK_CHANNEL_TC_FOC = 13,
    TTALINK_CHANNEL_TC_SPREAD = 14,
    TTALINK_CHANNEL_TC_CAMERA = 15,
    TTALINK_CHANNEL_FC_VR = 16,
    TTALINK_CHANNEL_GW_DL = 17,
    TTALINK_CHANNEL_CAN_DEVICE = 18,
    TTALINK_CHANNEL_EXAM_WS = 19,
    TTALINK_CHANNEL_TC_MIST_SPRAYER = 20,
} ttalink_channel_t;

/*
 * applications can set TTALINK_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
#ifndef TTALINK_COMM_NUM_BUFFERS
# define TTALINK_COMM_NUM_BUFFERS 21
#endif

#endif // TTALINK_CHANNEL_H
