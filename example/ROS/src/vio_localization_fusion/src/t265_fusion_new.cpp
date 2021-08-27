/**
 * @file vio_localization_fusion.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief fuse the pose from vio and map-based localization
 * @version 0.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "msf_core.h"

using namespace std;
using namespace Eigen;

multiSensorFusion::msf_core fusion;
ros::Publisher pose_pub, odom_pub, path_pub, globalPose_pub, globalOdom_pub;
nav_msgs::Path path;

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::imuData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->acc_ << msg->linear_acceleration.z, -msg->linear_acceleration.x, -msg->linear_acceleration.y;
    data->gyro_ << msg->angular_velocity.z, -msg->angular_velocity.x, -msg->angular_velocity.y;
    
    fusion.inputIMU(data);
}

void t265Callback(const nav_msgs::OdometryConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::vioData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    data->vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    data->q_ = Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Map<const Matrix<double, 6, 6> > poseCov(msg->pose.covariance.data());
    Eigen::Map<const Matrix<double, 6, 6> > twistCov(msg->twist.covariance.data());
    data->cov_.block<3, 3>(0, 0) = poseCov.block<3, 3>(0, 0);
    data->cov_.block<3, 3>(3, 3) = twistCov.block<3, 3>(0, 0);
    data->cov_.block<3, 3>(6, 6) = poseCov.block<3, 3>(3, 3);

    // get d400 pose from t265 pose
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(data->q_);
    T.pretranslate(data->pos_);
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    T1.pretranslate(Eigen::Vector3d(0, 0, 0.125));           
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T2.pretranslate(Eigen::Vector3d(0, 0, -0.125));
    Eigen::Isometry3d Twc_d400 = T1 * T * T2;
    data->pos_ = Twc_d400.translation();
    data->vel_ = data->q_.toRotationMatrix() * data->vel_;
    data->cov_.block<3, 3>(3, 3) = data->q_.toRotationMatrix() * data->cov_.block<3, 3>(3, 3) * data->q_.toRotationMatrix().transpose();

    fusion.inputVIO(data);
}

void mapLocCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::mapLocData>();

    data->timestamp_ = msg->header.stamp.toSec();
    Vector3d pos;
    pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    Eigen::Isometry3d Tmc = Eigen::Isometry3d::Identity();
    Tmc.rotate(q);
    Tmc.pretranslate(pos);
    Eigen::Isometry3d Tci = Eigen::Isometry3d::Identity();
    Tci.linear() << 0, -1, 0,
                    0, 0, -1,
                    1, 0, 0;
    Tci.pretranslate(Eigen::Vector3d(0.0361, -0.0055, -0.0205));
    Eigen::Isometry3d Tmi = Tmc * Tci;
    data->pos_ = Tmi.translation();
    
    data->q_ = Tmi.linear();

    fusion.inputMapLoc(data);
}

void fusionStateCallback(const ros::TimerEvent& event)
{
    if(fusion.isInitialized())
    {
        geometry_msgs::PoseStamped local_pose;
        nav_msgs::Odometry local_odom;

        multiSensorFusion::baseState currentState = fusion.outputCurrentState();

        ros::Time t = ros::Time(currentState.timestamp_);

        if(currentState.isWithMap_)
        {
            geometry_msgs::PoseStamped global_pose;
            nav_msgs::Odometry global_odom;

            global_odom.header.stamp = global_pose.header.stamp = t;
            // global_odom.header.stamp = global_pose.header.stamp = ros::Time::now();
            global_odom.header.frame_id = global_pose.header.frame_id = "global_frame";
            global_odom.pose.pose.position.x = global_pose.pose.position.x = currentState.posInMap_[0];
            global_odom.pose.pose.position.y = global_pose.pose.position.y = currentState.posInMap_[1];
            global_odom.pose.pose.position.z = global_pose.pose.position.z = currentState.posInMap_[2];
            global_odom.pose.pose.orientation.w = global_pose.pose.orientation.w = currentState.qInMap_.w();
            global_odom.pose.pose.orientation.x = global_pose.pose.orientation.x = currentState.qInMap_.x();
            global_odom.pose.pose.orientation.y = global_pose.pose.orientation.y = currentState.qInMap_.y();
            global_odom.pose.pose.orientation.z = global_pose.pose.orientation.z = currentState.qInMap_.z();
            global_odom.twist.twist.linear.x = currentState.velInMap_[0];
            global_odom.twist.twist.linear.y = currentState.velInMap_[1];
            global_odom.twist.twist.linear.z = currentState.velInMap_[2];   

            globalPose_pub.publish(global_pose);
            globalOdom_pub.publish(global_odom);
        }        

        local_odom.header.stamp = local_pose.header.stamp = t;
        // local_odom.header.stamp = local_pose.header.stamp = ros::Time::now();
        local_odom.header.frame_id = local_pose.header.frame_id = "t265_odom_frame";
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
        // path.header.stamp = ros::Time::now();
        path.header.frame_id = "t265_odom_frame";
        path.poses.push_back(local_pose);
        path_pub.publish(path);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vio_localization_fusion");
    ros::NodeHandle n("~");

    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/MSF/pose/local", 100);
    odom_pub = n.advertise<nav_msgs::Odometry>("/MSF/odom/local", 100);
    path_pub = n.advertise<nav_msgs::Path>("/MSF/path", 100);
    globalPose_pub = n.advertise<geometry_msgs::PoseStamped>("/MSF/pose/global", 100);
    globalOdom_pub = n.advertise<nav_msgs::Odometry>("/MSF/odom/global", 100);

    ros::Subscriber imu_sub = n.subscribe("/d400/imu", 100, imuCallback);
    ros::Subscriber t265_sub = n.subscribe("/t265/odom/sample", 100, t265Callback);
    ros::Subscriber mapLoc_sub = n.subscribe("/mapLoc/pose", 10, mapLocCallback);

    ros::Timer state_timer = n.createTimer(ros::Duration(0.005), fusionStateCallback);

    ros::spin();
    
    ros::shutdown();

    return 0;
}