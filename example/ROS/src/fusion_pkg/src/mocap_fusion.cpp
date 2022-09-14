/**
 * @file vio_localization_fusion.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief fuse the pose from vio and map-based localization
 * @version 0.1
 * @date 2022-07-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "msf_core.h"
#include "msf_utils.h"

using namespace std;
using namespace Eigen;

shared_ptr<MSF::msf_core> pFusion;
ros::Publisher pose_pub, odom_pub, path_pub, globalPose_pub, globalOdom_pub, Tim_pub;
nav_msgs::Path path;

void publishCurrentPose()
{
    if(pFusion->isInitialized())
    {
        geometry_msgs::PoseStamped local_pose;
        nav_msgs::Odometry local_odom;

        MSF::baseState currentState = pFusion->outputCurrentState();

        ros::Time t = ros::Time(currentState.timestamp_);

        if(currentState.has_global_state_)
        {
            geometry_msgs::PoseStamped global_pose;
            nav_msgs::Odometry global_odom;
            geometry_msgs::PoseStamped Tim;

            global_odom.header.stamp = global_pose.header.stamp = Tim.header.stamp = t;
            global_odom.header.frame_id = global_pose.header.frame_id = "MSF_global_frame";
            global_odom.pose.pose.position.x = global_pose.pose.position.x = currentState.pos_in_global_[0];
            global_odom.pose.pose.position.y = global_pose.pose.position.y = currentState.pos_in_global_[1];
            global_odom.pose.pose.position.z = global_pose.pose.position.z = currentState.pos_in_global_[2];
            global_odom.pose.pose.orientation.w = global_pose.pose.orientation.w = currentState.q_in_global_.w();
            global_odom.pose.pose.orientation.x = global_pose.pose.orientation.x = currentState.q_in_global_.x();
            global_odom.pose.pose.orientation.y = global_pose.pose.orientation.y = currentState.q_in_global_.y();
            global_odom.pose.pose.orientation.z = global_pose.pose.orientation.z = currentState.q_in_global_.z();
            global_odom.twist.twist.linear.x = currentState.vel_in_global_[0];
            global_odom.twist.twist.linear.y = currentState.vel_in_global_[1];
            global_odom.twist.twist.linear.z = currentState.vel_in_global_[2];
            Tim.pose.position.x = currentState.local_p_global_[0];
            Tim.pose.position.y = currentState.local_p_global_[1];
            Tim.pose.position.z = currentState.local_p_global_[2];
            Tim.pose.orientation.w = currentState.local_q_global_.w();
            Tim.pose.orientation.x = currentState.local_q_global_.x();
            Tim.pose.orientation.y = currentState.local_q_global_.y();
            Tim.pose.orientation.z = currentState.local_q_global_.z();

            globalPose_pub.publish(global_pose);
            globalOdom_pub.publish(global_odom);
            Tim_pub.publish(Tim);
        }

        local_odom.header.stamp = local_pose.header.stamp = t;
        // local_odom.header.stamp = local_pose.header.stamp = ros::Time::now();
        local_odom.header.frame_id = local_pose.header.frame_id = "MSF_local_frame";
        local_odom.pose.pose.position.x = local_pose.pose.position.x = currentState.pos_[0];
        local_odom.pose.pose.position.y = local_pose.pose.position.y = currentState.pos_[1];
        local_odom.pose.pose.position.z = local_pose.pose.position.z = currentState.pos_[2];
        local_odom.pose.pose.orientation.w = local_pose.pose.orientation.w = currentState.q_.w();
        local_odom.pose.pose.orientation.x = local_pose.pose.orientation.x = currentState.q_.x();
        local_odom.pose.pose.orientation.y = local_pose.pose.orientation.y = currentState.q_.y();
        local_odom.pose.pose.orientation.z = local_pose.pose.orientation.z = currentState.q_.z();
        local_odom.twist.twist.linear.x = currentState.vel_[0];
        local_odom.twist.twist.linear.y = currentState.vel_[1];
        local_odom.twist.twist.linear.z = currentState.vel_[2];

        pose_pub.publish(local_pose);
        odom_pub.publish(local_odom);

        path.header.stamp = t;
        path.header.frame_id = "MSF_local_frame";
        path.poses.push_back(local_pose);
        path_pub.publish(path);
    }
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    auto data = std::make_shared<MSF::imuData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = MSF::IMU;
    data->acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    data->gyro_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    pFusion->inputIMU(data);

    publishCurrentPose();
}

void moCapCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    auto data = std::make_shared<MSF::poseData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = MSF::Pose;
    data->name_ = "mocap";
    data->pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    data->q_ = Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    pFusion->inputPose(data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mocap_t265_orb_fusion");
    ros::NodeHandle nh("~");

    if (argc != 2)
    {
        cerr << endl
             << "Usage: rosrun fusion_pkg mocap_fusion path_to_config" << endl;
        return 1;
    }

    pFusion = std::make_shared<MSF::msf_core>(argv[1]);

    cout << "Use motion capture system!!!" << endl;

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/MSF/pose/local", 100);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/MSF/odom/local", 100);
    path_pub = nh.advertise<nav_msgs::Path>("/MSF/path", 100);
    globalPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/MSF/pose/global", 100);
    globalOdom_pub = nh.advertise<nav_msgs::Odometry>("/MSF/odom/global", 100);
    Tim_pub = nh.advertise<geometry_msgs::PoseStamped>("/MSF/extrinsic", 100);

    ros::Subscriber imu_sub = nh.subscribe("/d400/imu", 100, imuCallback);
    ros::Subscriber moCap_sub = nh.subscribe("/mocap/pose", 100, moCapCallback);

    ros::spin();
    
    ros::shutdown();

    return 0;
}