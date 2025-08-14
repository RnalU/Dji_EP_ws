#pragma once
#include <stdint.h>
#include <string>
#include "UdpDataPort.h"
#include "TcpDataPort.h"
#include "common/ttalink.h"

typedef struct __uavdata_feed_back_t {

    double latit;
    double longi;
    float altit;

    float velN;
    float velE;
    float velD;

    float atti_pitch;
    float atti_roll;
    float atti_yaw;

    float gyro_pitch;
    float gyro_roll;
    float gyro_yaw;

    float accN;
    float accE;
    float accD;

    float quat[4]; /*<  四元数.*/

}uav_data_feed_back_t;

class uavData : public Thread
{
public:
    uavData(int localport);
    virtual ~uavData();

private:
    int proxy();
    int ttalinkMsgHandle(ttalink_message_t *msg);
    int rosuavFeedbackHandle(ttalink_rosuav_ctrl_feed_back_t * feedback_data);

protected:
    int doProxy();
	void threadFunc();

public:
    DataPort * getDataPort();

private:
    DataPort *mFcConnction;

    int mlocalport;

public:
    int mDataCount;

    uav_data_feed_back_t  mFeedback_data;

};
