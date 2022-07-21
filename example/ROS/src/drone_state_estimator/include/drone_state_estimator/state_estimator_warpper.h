/**
 * @file state_estimator_warpper.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief The header of T265 and map fusion's ROS wrapper
 * @version 0.1
 * @date 2021-10-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef STATE_ESTIMATOR_WARPPER_H
#define STATE_ESTIMATOR_WARPPER_H

#include <iostream>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include "msf_core.h"

class state_estimator_warpper
{
public:
    state_estimator_warpper(ros::NodeHandle &nh);
    ~state_estimator_warpper() = default;

private:
    // Callback function
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void t265Callback(const nav_msgs::OdometryConstPtr &msg);
    void mapLocCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void waypointCallback(const geometry_msgs::PointStampedConstPtr &msg);
    void stateCallback(const ros::TimerEvent &event);

    // Publisher, Subscriber, Timer
    ros::Publisher pose_pub, odom_pub, path_pub;
    ros::Subscriber imu_sub, t265_sub, mapLoc_sub, waypoint_sub;
    ros::Timer state_timer;

    nav_msgs::Path path;

    multiSensorFusion::msf_core msf;

    bool t265_get;
    Eigen::Isometry3d T_wv, T_vw, T_bc, T_cb;
};



#endif // STATE_ESTIMATOR_WARPPER_H