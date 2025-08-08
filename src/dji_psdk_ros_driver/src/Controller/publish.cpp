#include "uavData.h"
#include "publish.h"
#include "Utils.h"
#include <mutex>


extern std::mutex g_mutex;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "dji_psdk_ros_driver/uavdata.h"

uavData_publish::uavData_publish(ros::NodeHandle *nh, uavData *puavData)
{
    m_uavdata_nh = nh;
    m_puavData = puavData;
}

uavData_publish::~uavData_publish()
{

}

int uavData_publish::proxy()
{
	doProxy();

	return 0;
}

int uavData_publish::doProxy()
{
	BufferItem* item = NULL;
	int64_t now = Utils::getSystemRunTime();
	int64_t last_hb_ts = 0;

    ros::Publisher uavdata_pub = m_uavdata_nh->advertise<dji_psdk_ros_driver::uavdata>("uavdata", 1000);

    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok())
    {
        dji_psdk_ros_driver::uavdata uavdata_d;

        std::unique_lock<std::mutex> lock(g_mutex);

        uavdata_d.latit = m_puavData->mFeedback_data.latit;
        uavdata_d.longi = m_puavData->mFeedback_data.longi;
        uavdata_d.altit = m_puavData->mFeedback_data.altit;

        uavdata_d.velN = m_puavData->mFeedback_data.velN;
        uavdata_d.velE = m_puavData->mFeedback_data.velE;
        uavdata_d.velD = m_puavData->mFeedback_data.velD;

        uavdata_d.atti_pitch = m_puavData->mFeedback_data.atti_pitch;
        uavdata_d.atti_roll = m_puavData->mFeedback_data.atti_roll;
        uavdata_d.atti_yaw = m_puavData->mFeedback_data.atti_yaw;

        uavdata_d.gyro_pitch = m_puavData->mFeedback_data.gyro_pitch;
        uavdata_d.gyro_roll = m_puavData->mFeedback_data.gyro_roll;
        uavdata_d.gyro_yaw = m_puavData->mFeedback_data.gyro_yaw;

        uavdata_d.accN = m_puavData->mFeedback_data.accN;
        uavdata_d.accE = m_puavData->mFeedback_data.accE;
        uavdata_d.accD = m_puavData->mFeedback_data.accD;

        std::vector<float> arrp;

        arrp.push_back(m_puavData->mFeedback_data.quat[0]);
        arrp.push_back(m_puavData->mFeedback_data.quat[1]);
        arrp.push_back(m_puavData->mFeedback_data.quat[2]);
        arrp.push_back(m_puavData->mFeedback_data.quat[3]);

        uavdata_d.quat = arrp;

        uavdata_pub.publish(uavdata_d);

        lock.unlock();

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }

	return 0;
}

void uavData_publish::threadFunc(){
	proxy();
}



