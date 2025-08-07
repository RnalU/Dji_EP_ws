#include "uavData.h"
//#include "service.h"
#include "Utils.h"
#include "ttalinkUtils.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include "dji_psdk_ros_driver/gimbalControl.h"
#include "dji_psdk_ros_driver/takeoffOrLanding.h"
#include "dji_psdk_ros_driver/flightByVel.h"

#include "uavData.h"
#include "publish.h"

uavData *m_puavData = NULL;

bool takeoffOrLanding_serverCB(dji_psdk_ros_driver::takeoffOrLanding::Request  &req,
    dji_psdk_ros_driver::takeoffOrLanding::Response &res)
{
    res.ack = 1;

    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};

    if(req.takeoffOrLanding == 1)
    {
      rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_AUTO_TAKE_OFF;

      ROS_INFO("request: takeoff ");
    }
    else if(req.takeoffOrLanding == 2)
    {
      rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_NAVGATION_ALTI_VEL;

      ROS_INFO("request: landing ");
    }

    if(m_puavData)
    {
      ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
    }

  return true;
}

bool gimbalControl_serverCB(dji_psdk_ros_driver::gimbalControl::Request  &req,
    dji_psdk_ros_driver::gimbalControl::Response &res)
{
    res.ack = 1;

    ROS_INFO("request: pitch:%f , roll:%f, yaw:%f", req.pitch, req.roll, req.yaw);

    ttalinkUtils::SendGeneralCommand(m_puavData->getDataPort(), TTALINK_PTZ_ADDRESS, TTALINK_EMBE_ADDRESS
    , 11223344, 0, 109, req.pitch, req.roll, req.yaw,0,0,0,0);

    return true;
}

void flightByVel_messageCB(const dji_psdk_ros_driver::flightByVel::ConstPtr &msg)
{

    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};

    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_POS_VEL_ALTI_VEL;

    rosuav_ctrl_loop_input.velN = msg->vel_n;
    rosuav_ctrl_loop_input.velE = msg->vel_e;
    rosuav_ctrl_loop_input.velD = msg->vel_d;

    rosuav_ctrl_loop_input.atti_yaw = msg->targetYaw;

    rosuav_ctrl_loop_input.param[0] = msg->fly_time;

    ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "fcProxy");
    ros::NodeHandle n;

    m_puavData = new uavData(47142);
    m_puavData->start();

    uavData_publish *m_puavData_publish = new uavData_publish(&n, m_puavData);
    m_puavData_publish->start();

    ros::Subscriber subscriber1 = n.subscribe<dji_psdk_ros_driver::flightByVel>("flightByVel", 1, flightByVel_messageCB);
    ROS_INFO("Start flightByVel message");

    ros::ServiceServer service1 = n.advertiseService("takeoffOrLanding", takeoffOrLanding_serverCB);
    ROS_INFO("Start takeoffOrLanding server");

    ros::ServiceServer service2 = n.advertiseService("gimbalControl", gimbalControl_serverCB);
    ROS_INFO("Start gimbalControl server");


    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    delete m_puavData_publish;
    delete m_puavData;

	  return 0;
}




