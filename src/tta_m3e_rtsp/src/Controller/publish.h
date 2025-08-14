#pragma once
#include <stdint.h>
#include <string>
#include "Thread.h"
#include "uavData.h"

#include "ros/ros.h"

class uavData_publish : public Thread
{
public:
    uavData_publish(ros::NodeHandle *nh, uavData *puavData);
    virtual ~uavData_publish();

public:
    int proxy();

protected:
    int doProxy();
	void threadFunc();

private:
    ros::NodeHandle *m_uavdata_nh;
    uavData *m_puavData;
    

};

