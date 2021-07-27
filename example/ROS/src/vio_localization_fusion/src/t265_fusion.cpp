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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "multiSensorsFusion.h"

using namespace std;
using namespace Eigen;

multiSensorFusion::MSF vio_map_fusion;
ros::Publisher pose_pub, odom_pub, path_pub, globalPose_pub, globalOdom_pub;

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    double t = msg->header.stamp.toSec();
    Vector3d acc, gyro;
    acc << msg->linear_acceleration.z, -msg->linear_acceleration.x, -msg->linear_acceleration.y;
    gyro << msg->angular_velocity.z, -msg->angular_velocity.x, -msg->angular_velocity.y;
    vio_map_fusion.inputIMU(t, acc, gyro);
}

void t265Callback(const nav_msgs::OdometryConstPtr &msg)
{
    double t = msg->header.stamp.toSec();
    Vector3d pos, vel;
    pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Map<const Matrix<double, 6, 6> > poseCov(msg->pose.covariance.data());
    Eigen::Map<const Matrix<double, 6, 6> > twistCov(msg->twist.covariance.data());
    Matrix<double, 9, 9> cov = Matrix<double, 9, 9>::Zero();
    cov.block<3, 3>(0, 0) = poseCov.block<3, 3>(0, 0);
    cov.block<3, 3>(3, 3) = twistCov.block<3, 3>(0, 0);
    cov.block<3, 3>(6, 6) = poseCov.block<3, 3>(3, 3);

    // get d400 pose from t265 pose
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(q);
    T.pretranslate(pos);
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    T1.pretranslate(Eigen::Vector3d(0, 0, 0.125));           
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T2.pretranslate(Eigen::Vector3d(0, 0, -0.125));
    Eigen::Isometry3d Twc_d400 = T1 * T * T2;
    pos = Twc_d400.translation();
    vel = q.toRotationMatrix() * vel;
    cov.block<3, 3>(3, 3) = q.toRotationMatrix() * cov.block<3, 3>(3, 3) * q.toRotationMatrix().transpose();

    vio_map_fusion.inputVIO(t, pos, vel, q, cov);
}

void mapLocCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    double t = msg->header.stamp.toSec();
    Vector3d pos;
    pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    Eigen::Isometry3d Tmc = Eigen::Isometry3d::Identity();
    Tmc.rotate(q);
    Tmc.pretranslate(pos);
    Eigen::Isometry3d Tci;
    Tci.linear() << 0, -1, 0,
                    0, 0, -1,
                    1, 0, 0;
    Tci.pretranslate(Eigen::Vector3d(0.0361, -0.0055, -0.0205));
    Eigen::Isometry3d Tmi = Tmc * Tci;
    pos = Tmi.translation();
    
    q = Tmi.linear();

    vio_map_fusion.inputMapPose(t, pos, q);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vio_localization_fusion");
    ros::NodeHandle n("~");

    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/MSF/pose/local", 10);
    odom_pub = n.advertise<nav_msgs::Odometry>("/MSF/odom/local", 10);
    path_pub = n.advertise<nav_msgs::Path>("/MSF/path", 10);
    globalPose_pub = n.advertise<geometry_msgs::PoseStamped>("/MSF/pose/global", 10);
    globalOdom_pub = n.advertise<nav_msgs::Odometry>("/MSF/odom/global", 10);

    ros::Subscriber imu_sub = n.subscribe("/d400/imu", 10, imuCallback);
    ros::Subscriber t265_sub = n.subscribe("/t265/odom/sample", 10, t265Callback);
    ros::Subscriber mapLoc_sub = n.subscribe("/mapLoc/pose", 10, mapLocCallback);

    nav_msgs::Path path;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        if(vio_map_fusion.isInitialized())
        {
            geometry_msgs::PoseStamped local_pose;
            nav_msgs::Odometry local_odom;

            local_odom.header.stamp = local_pose.header.stamp = ros::Time(vio_map_fusion.getCurrentTime());
            // local_odom.header.stamp = local_pose.header.stamp = ros::Time::now();
            local_odom.header.frame_id = local_pose.header.frame_id = "t265_odom_frame";
            local_odom.pose.pose.position.x = local_pose.pose.position.x = vio_map_fusion.getPosition()[0];
            local_odom.pose.pose.position.y = local_pose.pose.position.y = vio_map_fusion.getPosition()[1];
            local_odom.pose.pose.position.z = local_pose.pose.position.z = vio_map_fusion.getPosition()[2];
            local_odom.pose.pose.orientation.w = local_pose.pose.orientation.w = vio_map_fusion.getRotation().w();
            local_odom.pose.pose.orientation.x = local_pose.pose.orientation.x = vio_map_fusion.getRotation().x();
            local_odom.pose.pose.orientation.y = local_pose.pose.orientation.y = vio_map_fusion.getRotation().y();
            local_odom.pose.pose.orientation.z = local_pose.pose.orientation.z = vio_map_fusion.getRotation().z();
            local_odom.twist.twist.linear.x = vio_map_fusion.getVelocity()[0];
            local_odom.twist.twist.linear.y = vio_map_fusion.getVelocity()[1];
            local_odom.twist.twist.linear.z = vio_map_fusion.getVelocity()[2];

            pose_pub.publish(local_pose);
            odom_pub.publish(local_odom);

            path.header.stamp = ros::Time::now();
            path.header.frame_id = "t265_odom_frame";
            path.poses.push_back(local_pose);
            path_pub.publish(path);

            if(vio_map_fusion.isWithMap())
            {
                geometry_msgs::PoseStamped global_pose;
                nav_msgs::Odometry global_odom;

                global_odom.header.stamp = global_pose.header.stamp = ros::Time(vio_map_fusion.getCurrentTime());
                // global_odom.header.stamp = global_pose.header.stamp = ros::Time::now();
                global_odom.header.frame_id = global_pose.header.frame_id = "global_frame";
                Eigen::Vector3d pos_in_map = vio_map_fusion.getPositionInMapFrame();
                global_odom.pose.pose.position.x = global_pose.pose.position.x = pos_in_map[0];
                global_odom.pose.pose.position.y = global_pose.pose.position.y = pos_in_map[1];
                global_odom.pose.pose.position.z = global_pose.pose.position.z = pos_in_map[2];
                Eigen::Quaterniond rot_in_map = vio_map_fusion.getRotationInMapFrame();
                global_odom.pose.pose.orientation.w = global_pose.pose.orientation.w = rot_in_map.w();
                global_odom.pose.pose.orientation.x = global_pose.pose.orientation.x = rot_in_map.x();
                global_odom.pose.pose.orientation.y = global_pose.pose.orientation.y = rot_in_map.y();
                global_odom.pose.pose.orientation.z = global_pose.pose.orientation.z = rot_in_map.z();
                Eigen::Vector3d vel_in_map = vio_map_fusion.getVelocityInMapFrame();
                global_odom.twist.twist.linear.x = vel_in_map[0];
                global_odom.twist.twist.linear.y = vel_in_map[1];
                global_odom.twist.twist.linear.z = vel_in_map[2];   

                globalPose_pub.publish(global_pose);
                globalOdom_pub.publish(global_odom);
            }
        }

        loop_rate.sleep();
    }
    
    ros::shutdown();

    return 0;
}