/**
 * @author 梦狐(Dream_fox)
 * @brief 在新版机载盒子测试中，如果只用uavdata去做odom，进而推理实际位置不太现实，只能边走变矫正。主要是积分误差累计太大，虽然可以通过滤波器减少这种噪声，但是还是差强人意。
 *        本程序只不过是测试，并非主干程序，如果能用3TD或许就能实现精准定位。
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include "tta_m3e_rtsp/uavdata.h"

// 定义全局变量 uav_data
tta_m3e_rtsp::uavdata uav_data = {};

// 滤波系数
const float alpha = 0.2;

// 滤波后的速度
float filteredLinearVelocityX = 0;
float filteredLinearVelocityY = 0;
float filteredLinearVelocityZ = 0;
float filteredAngularVelocity = 0;

// 飞行器信息回调函数
void uavDataCallback(const tta_m3e_rtsp::uavdata& msg)
{
    uav_data = msg;
    // 指数加权移动平均滤波
    filteredLinearVelocityX = alpha * uav_data.velE + (1 - alpha) * filteredLinearVelocityX;
    filteredLinearVelocityY = alpha * uav_data.velN + (1 - alpha) * filteredLinearVelocityY;
    filteredLinearVelocityZ = alpha * uav_data.velD + (1 - alpha) * filteredLinearVelocityZ;
    filteredAngularVelocity = alpha * uav_data.gyro_yaw + (1 - alpha) * filteredAngularVelocity;
}

// 计算里程计信息的函数
nav_msgs::Odometry calculateOdom(const nav_msgs::Odometry& currentOdom, float linearVelocityX, float linearVelocityY, float linearVelocityZ, float angularVelocity, double dt)
{
    nav_msgs::Odometry newOdom = currentOdom;

    // 更新位置
    double deltaX = linearVelocityX * cos(tf::getYaw(currentOdom.pose.pose.orientation)) * dt;
    double deltaY = linearVelocityY * sin(tf::getYaw(currentOdom.pose.pose.orientation)) * dt;
    double deltaZ = linearVelocityZ * dt;

    newOdom.pose.pose.position.x += deltaX * cos(tf::getYaw(currentOdom.pose.pose.orientation)) - deltaY * sin(tf::getYaw(currentOdom.pose.pose.orientation));
    newOdom.pose.pose.position.y += deltaX * sin(tf::getYaw(currentOdom.pose.pose.orientation)) + deltaY * cos(tf::getYaw(currentOdom.pose.pose.orientation));
    newOdom.pose.pose.position.z += deltaZ;

    // 更新方向角
    double newTheta = tf::getYaw(currentOdom.pose.pose.orientation) + angularVelocity * dt;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(newTheta);
    newOdom.pose.pose.orientation = odom_quat;

    // 更新速度信息
    newOdom.twist.twist.linear.x = linearVelocityX * cos(tf::getYaw(currentOdom.pose.pose.orientation));
    newOdom.twist.twist.linear.y = linearVelocityY * sin(tf::getYaw(currentOdom.pose.pose.orientation));
    newOdom.twist.twist.linear.z = linearVelocityZ;
    newOdom.twist.twist.angular.z = angularVelocity;

    return newOdom;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;

    // 创建一个发布者，发布里程计消息
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    // 创建一个订阅者，订阅uavdata话题
    ros::Subscriber uav_data_sub = nh.subscribe("uavdata", 10, uavDataCallback);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // 初始化当前里程计信息
    nav_msgs::Odometry currentOdom;
    currentOdom.pose.pose.position.x = 0;
    currentOdom.pose.pose.position.y = 0;
    currentOdom.pose.pose.position.z = 0;
    currentOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    currentOdom.twist.twist.linear.x = 0;
    currentOdom.twist.twist.linear.y = 0;
    currentOdom.twist.twist.linear.z = 0;
    currentOdom.twist.twist.angular.z = 0;

    ros::Rate r(10.0);
    while (nh.ok()) {
        ros::spinOnce();               // 处理回调函数
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();

        // 计算新的里程计信息
        currentOdom = calculateOdom(currentOdom, filteredLinearVelocityX, filteredLinearVelocityY, filteredLinearVelocityZ, filteredAngularVelocity, dt);

        // 创建一个 tf 变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = currentOdom.pose.pose.position.x;
        odom_trans.transform.translation.y = currentOdom.pose.pose.position.y;
        odom_trans.transform.translation.z = currentOdom.pose.pose.position.z;
        odom_trans.transform.rotation = currentOdom.pose.pose.orientation;

        // 发布 tf 变换
        odom_broadcaster.sendTransform(odom_trans);

        // 设置里程计消息的时间戳和坐标系
        currentOdom.header.stamp = current_time;
        currentOdom.header.frame_id = "odom";
        currentOdom.child_frame_id = "base_link";

        // 发布里程计消息
        odom_pub.publish(currentOdom);

        last_time = current_time;
        r.sleep();
    }

    return 0;
}
