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

#include <chrono>
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

std::string config_path = "/home/antusheng/research/map_based_localization/test/Multi_Sensor_Fusion/example/ROS/src/vio_localization_fusion/config/config.yaml";
multiSensorFusion::msf_core fusion(config_path);
ros::Publisher pose_pub, odom_pub, path_pub, globalPose_pub, globalOdom_pub, Tim_pub;
nav_msgs::Path path;

void publishCurrentPose()
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
    auto data = std::make_shared<multiSensorFusion::imuData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = multiSensorFusion::IMU;
    data->acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    data->gyro_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    fusion.inputIMU(data);

    publishCurrentPose();
}

void t265Callback(const nav_msgs::OdometryConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::odomData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = multiSensorFusion::Odom;
    data->name_ = "t265";
    data->pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    data->vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    Eigen::Vector3d angular_vel(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    data->q_ = Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    data->vel_ = data->q_ * data->vel_;
    Eigen::Map<const Matrix<double, 6, 6> > poseCov(msg->pose.covariance.data());
    Eigen::Map<const Matrix<double, 6, 6> > twistCov(msg->twist.covariance.data());
    data->cov_.block<3, 3>(0, 0) = 10.0 * poseCov.block<3, 3>(0, 0);
    data->cov_.block<3, 3>(3, 3) = 10.0 * twistCov.block<3, 3>(0, 0);
    data->cov_.block<3, 3>(3, 3) = data->q_.toRotationMatrix() * data->cov_.block<3, 3>(3, 3) * data->q_.toRotationMatrix().transpose();
    data->cov_.block<3, 3>(6, 6) = 10.0 * poseCov.block<3, 3>(3, 3);

    fusion.inputOdom(data);
}

void mapLocCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::poseData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = multiSensorFusion::Pose;
    data->name_ = "orb_localization";
    data->pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    data->q_ = Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    fusion.inputPose(data);
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
    Tim_pub = n.advertise<geometry_msgs::PoseStamped>("/MSF/extrinsic", 100);

    ros::Subscriber imu_sub = n.subscribe("/d400/imu", 100, imuCallback);
    ros::Subscriber t265_sub = n.subscribe("/t265/odom/sample", 100, t265Callback);
    ros::Subscriber mapLoc_sub = n.subscribe("/mapLoc/pose", 10, mapLocCallback);

    ros::spin();

    ros::shutdown();

    return 0;
}