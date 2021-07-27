/**
 * @file t265_localization_wrapper.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief T265 and map fusion's ROS wrapper
 * @version 0.1
 * @date 2021-07-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "vio_localization_fusion/t265_localization_wrapper.h"

t265_localization_wrapper::t265_localization_wrapper(ros::NodeHandle &nh)
{
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/MSF/pose/local", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/MSF/odom/local", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/MSF/path", 10);
    globalPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/MSF/pose/global", 10);
    globalOdom_pub = nh.advertise<nav_msgs::Odometry>("/MSF/odom/global", 10);

    imu_sub = nh.subscribe("/d400/imu", 10, &t265_localization_wrapper::imuCallback, this, ros::TransportHints().tcpNoDelay());
    t265_sub = nh.subscribe("/t265/odom/sample", 10, &t265_localization_wrapper::t265Callback, this, ros::TransportHints().tcpNoDelay());
    mapLoc_sub = nh.subscribe("/mapLoc/pose", 10, &t265_localization_wrapper::mapLocCallback, this, ros::TransportHints().tcpNoDelay());

    state_timer = nh.createTimer(ros::Duration(0.005), &t265_localization_wrapper::fusionStateCallback, this);

    sync_processing = std::thread(&t265_localization_wrapper::sync_process, this);

    ros::spin();
}

// TODO: the localization modular's process time will make the messages asynchronous!!!
// use the multi-state instead
void t265_localization_wrapper::sync_process()
{
    while(true)
    {
        sensor_msgs::ImuConstPtr imu_msg = nullptr;
        nav_msgs::OdometryConstPtr t265_msg = nullptr;
        geometry_msgs::PoseStampedConstPtr mapLoc_msg = nullptr;

        m_buf.lock();
        if(!imu_buf.empty())
        {
            imu_msg = imu_buf.front();
            imu_buf.pop();
            while(!t265_buf.empty())
            {
                if(abs(t265_buf.front()->header.stamp.toSec() - imu_msg->header.stamp.toSec()) <= 0.005)
                {
                    t265_msg = t265_buf.front();
                    t265_buf.pop();
                    break;
                }
                else if(imu_msg->header.stamp.toSec() - t265_buf.front()->header.stamp.toSec() > 0.005)
                {
                    t265_buf.pop();
                    continue;
                }
                else
                    break;
            }
            while(!mapLoc_buf.empty())
            {
// cout << fixed << setprecision(9) << mapLoc_buf.front()->header.stamp.toSec() - imu_msg->header.stamp.toSec() << endl;

                if(abs(mapLoc_buf.front()->header.stamp.toSec() - imu_msg->header.stamp.toSec()) <= 0.005)
                {
                    mapLoc_msg = mapLoc_buf.front();
                    mapLoc_buf.pop();
                    break;
                }
                else if(imu_msg->header.stamp.toSec() - mapLoc_buf.front()->header.stamp.toSec() > 0.005)
                {
                    mapLoc_buf.pop();
                    continue;
                }
                else
                    break;
            }
        }
        m_buf.unlock();

        if(imu_msg)
        {
            imuMsgProcess(imu_msg);
        }

        if(t265_msg)
        {
            t265MsgProcess(t265_msg);
        }

        if(mapLoc_msg)
        {
            mapLocMsgProcess(mapLoc_msg);
        }   
    }
}

void t265_localization_wrapper::imuMsgProcess(const sensor_msgs::ImuConstPtr &msg)
{
    double t = msg->header.stamp.toSec();
    Vector3d acc, gyro;
    acc << msg->linear_acceleration.z, -msg->linear_acceleration.x, -msg->linear_acceleration.y;
    gyro << msg->angular_velocity.z, -msg->angular_velocity.x, -msg->angular_velocity.y;
    vio_map_fusion.inputIMU(t, acc, gyro);
}

void t265_localization_wrapper::t265MsgProcess(const nav_msgs::OdometryConstPtr &msg)
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

void t265_localization_wrapper::mapLocMsgProcess(const geometry_msgs::PoseStampedConstPtr &msg)
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

void t265_localization_wrapper::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    m_buf.lock();
    imu_buf.push(msg);
    m_buf.unlock();
}

void t265_localization_wrapper::t265Callback(const nav_msgs::OdometryConstPtr &msg)
{
    m_buf.lock();
    t265_buf.push(msg);
    m_buf.unlock();
}

void t265_localization_wrapper::mapLocCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    m_buf.lock();
    mapLoc_buf.push(msg);
    m_buf.unlock();
}

void t265_localization_wrapper::fusionStateCallback(const ros::TimerEvent& event)
{
    if(vio_map_fusion.isInitialized())
    {
        geometry_msgs::PoseStamped local_pose;
        nav_msgs::Odometry local_odom;

        ros::Time t = ros::Time(vio_map_fusion.getCurrentTime());
        Eigen::Vector3d pos = vio_map_fusion.getPosition();
        Eigen::Quaterniond rot = vio_map_fusion.getRotation();
        Eigen::Vector3d vel = vio_map_fusion.getVelocity();
        if(vio_map_fusion.isWithMap())
        {
            Eigen::Vector3d pos_in_map = vio_map_fusion.getPositionInMapFrame();
            Eigen::Quaterniond rot_in_map = vio_map_fusion.getRotationInMapFrame();
            Eigen::Vector3d vel_in_map = vio_map_fusion.getVelocityInMapFrame();

            geometry_msgs::PoseStamped global_pose;
            nav_msgs::Odometry global_odom;

            global_odom.header.stamp = global_pose.header.stamp = t;
            // global_odom.header.stamp = global_pose.header.stamp = ros::Time::now();
            global_odom.header.frame_id = global_pose.header.frame_id = "global_frame";
            global_odom.pose.pose.position.x = global_pose.pose.position.x = pos_in_map[0];
            global_odom.pose.pose.position.y = global_pose.pose.position.y = pos_in_map[1];
            global_odom.pose.pose.position.z = global_pose.pose.position.z = pos_in_map[2];
            global_odom.pose.pose.orientation.w = global_pose.pose.orientation.w = rot_in_map.w();
            global_odom.pose.pose.orientation.x = global_pose.pose.orientation.x = rot_in_map.x();
            global_odom.pose.pose.orientation.y = global_pose.pose.orientation.y = rot_in_map.y();
            global_odom.pose.pose.orientation.z = global_pose.pose.orientation.z = rot_in_map.z();
            global_odom.twist.twist.linear.x = vel_in_map[0];
            global_odom.twist.twist.linear.y = vel_in_map[1];
            global_odom.twist.twist.linear.z = vel_in_map[2];   

            globalPose_pub.publish(global_pose);
            globalOdom_pub.publish(global_odom);
        }        

        local_odom.header.stamp = local_pose.header.stamp = t;
        // local_odom.header.stamp = local_pose.header.stamp = ros::Time::now();
        local_odom.header.frame_id = local_pose.header.frame_id = "t265_odom_frame";
        local_odom.pose.pose.position.x = local_pose.pose.position.x = pos[0];
        local_odom.pose.pose.position.y = local_pose.pose.position.y = pos[1];
        local_odom.pose.pose.position.z = local_pose.pose.position.z = pos[2];
        local_odom.pose.pose.orientation.w = local_pose.pose.orientation.w = rot.w();
        local_odom.pose.pose.orientation.x = local_pose.pose.orientation.x = rot.x();
        local_odom.pose.pose.orientation.y = local_pose.pose.orientation.y = rot.y();
        local_odom.pose.pose.orientation.z = local_pose.pose.orientation.z = rot.z();
        local_odom.twist.twist.linear.x = vel[0];
        local_odom.twist.twist.linear.y = vel[1];
        local_odom.twist.twist.linear.z = vel[2];

        pose_pub.publish(local_pose);
        odom_pub.publish(local_odom);

        path.header.stamp = t;
        // path.header.stamp = ros::Time::now();
        path.header.frame_id = "t265_odom_frame";
        path.poses.push_back(local_pose);
        path_pub.publish(path);
    }
}