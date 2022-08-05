/**
 * @file t265_localization_wrapper.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief The header of T265 and map fusion's ROS wrapper
 * @version 0.1
 * @date 2021-07-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef T265_LOCALIZATION_WRAPPER_H
#define T265_LOCALIZATION_WRAPPER_H

#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "multiSensorsFusion.h"

using namespace std;
using namespace Eigen;

class t265_localization_wrapper
{
public:
    t265_localization_wrapper(ros::NodeHandle &nh);
    ~t265_localization_wrapper() = default;

private:
    // Synchronously process function
    void sync_process();

    void imuMsgProcess(const sensor_msgs::ImuConstPtr &msg);
    void t265MsgProcess(const nav_msgs::OdometryConstPtr &msg);
    void mapLocMsgProcess(const geometry_msgs::PoseStampedConstPtr &msg);

    // Callback functions
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void t265Callback(const nav_msgs::OdometryConstPtr &msg);
    void mapLocCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void fusionStateCallback(const ros::TimerEvent &event);

    // Publisher, Subscriber, Timer
    ros::Publisher pose_pub, odom_pub, path_pub, globalPose_pub, globalOdom_pub;
    ros::Subscriber imu_sub, t265_sub, mapLoc_sub;
    ros::Timer state_timer;

    nav_msgs::Path path;

    // message buffer
    queue<sensor_msgs::ImuConstPtr> imu_buf;
    queue<nav_msgs::OdometryConstPtr> t265_buf;
    queue<geometry_msgs::PoseStampedConstPtr> mapLoc_buf;

    multiSensorFusion::MSF vio_map_fusion;

    std::mutex m_buf;

    std::thread sync_processing;
};

#endif