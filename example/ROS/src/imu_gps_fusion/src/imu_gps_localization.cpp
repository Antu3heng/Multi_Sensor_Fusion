/**
 * @file vio_localization_fusion.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief fuse the pose from vio and map-based localization
 * @version 0.1
 * @date 2021-06-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "imuGPSFusion.h"

using namespace std;
using namespace Eigen;

multiSensorFusion::imuGPSFusion imu_gps_fusion;
ros::Publisher pose_pub;
ros::Publisher odom_pub;
ros::Publisher path_pub;

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    double t = msg->header.stamp.toSec();
    Vector3d acc, gyro;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu_gps_fusion.inputIMU(t, acc, gyro);
}

void GPSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    if(msg->status.status != 2)
    {
        cout << " Bad GPS message!" << endl;
        return;
    }

    double t = msg->header.stamp.toSec();
    Vector3d lla;
    lla << msg->latitude, msg->longitude, msg->altitude;
    Matrix3d cov = Eigen::Map<const Eigen::Matrix3d>(msg->position_covariance.data());

    imu_gps_fusion.inputGPS(t, lla, cov);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_gps_fusion");
    ros::NodeHandle n("~");

    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/fusion/pose", 10);
    path_pub = n.advertise<nav_msgs::Path>("/fusion/path", 10);

    ros::Subscriber imu_sub = n.subscribe("/imu/data", 10, imuCallback);
    ros::Subscriber gps_sub = n.subscribe("/fix", 10, GPSCallback);

    nav_msgs::Path path;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        if(imu_gps_fusion.isInitialized())
        {
            geometry_msgs::PoseStamped pose;

            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "world";
            pose.pose.position.x = imu_gps_fusion.getPosition()[0];
            pose.pose.position.y = imu_gps_fusion.getPosition()[1];
            pose.pose.position.z = imu_gps_fusion.getPosition()[2];
            pose.pose.orientation.w = imu_gps_fusion.getRotation().w();
            pose.pose.orientation.x = imu_gps_fusion.getRotation().x();
            pose.pose.orientation.y = imu_gps_fusion.getRotation().y();
            pose.pose.orientation.z = imu_gps_fusion.getRotation().z();

            pose_pub.publish(pose);

            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();
            path.poses.push_back(pose);
            path_pub.publish(path);
        }

        loop_rate.sleep();
    }

    ros::shutdown();

    return 0;
}