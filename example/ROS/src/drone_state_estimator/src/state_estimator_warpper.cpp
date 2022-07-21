/**
 * @file state_estimator_warpper.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief The header of T265 and map fusion's ROS wrapper
 * @version 0.1
 * @date 2021-10-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "drone_state_estimator/state_estimator_warpper.h"

state_estimator_warpper::state_estimator_warpper(ros::NodeHandle &nh)
{
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/MSF/pose/local", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/MSF/odom/local", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/MSF/path", 10);

    imu_sub = nh.subscribe("/mavros/imu/data", 10, &state_estimator_warpper::imuCallback, this, ros::TransportHints().tcpNoDelay());
    t265_sub = nh.subscribe("/t265/odom/sample", 10, &state_estimator_warpper::t265Callback, this, ros::TransportHints().tcpNoDelay());
    mapLoc_sub = nh.subscribe("/mapLoc/pose", 10, &state_estimator_warpper::mapLocCallback, this, ros::TransportHints().tcpNoDelay());
    waypoint_sub = nh.subscribe("/waypoint/position", 10, &state_estimator_warpper::waypointCallback, this, ros::TransportHints().tcpNoDelay());

    state_timer = nh.createTimer(ros::Duration(0.005), &state_estimator_warpper::stateCallback, this);

    t265_get = false;
    T_wv = T_bc = Eigen::Isometry3d::Identity();
}

void state_estimator_warpper::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::imuData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = multiSensorFusion::IMU;
    data->acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    data->gyro_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    
    msf.inputIMU(data);
}

void state_estimator_warpper::t265Callback(const nav_msgs::OdometryConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::odomData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = multiSensorFusion::VIO;
    tf::pointMsgToEigen(msg->pose.pose.position, data->pos_);
    tf::vectorMsgToEigen(msg->twist.twist.linear, data->vel_);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, data->q_);
    Eigen::Map<const Eigen::Matrix<double, 6, 6> > poseCov(msg->pose.covariance.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 6> > twistCov(msg->twist.covariance.data());
    data->cov_.block<3, 3>(0, 0) = poseCov.block<3, 3>(0, 0);
    data->cov_.block<3, 3>(3, 3) = twistCov.block<3, 3>(0, 0);
    data->cov_.block<3, 3>(6, 6) = poseCov.block<3, 3>(3, 3);

    // get the drone's odom from t265's
    Eigen::Isometry3d T_vc = Eigen::Isometry3d::Identity();
    T_vc.rotate(data->q_);
    T_vc.pretranslate(data->pos_);

    // t265 and MAVROS's reference frame are aligned with gravity
    if (!t265_get)
    {
        T_wv.linear() << -1, 0, 0,
                        0, -1, 0,
                        0, 0, 1;
        T_bc.linear() << -0.867, 0, -0.5,
                0, -1, 0,
                -0.5, 0, 0.867;
        T_bc.pretranslate(Eigen::Vector3d(-0.185, 0, 0.03));
        T_cb = T_bc.inverse();
        Eigen::Isometry3d T_vb = T_vc * T_cb;
        Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
        T_wb.linear() = T_wv.linear() * T_vb.linear();
        T_wv = T_wb * T_vb.inverse();
        T_vw = T_wv.inverse();
        t265_get = true;
    }

    Eigen::Isometry3d T_wb = T_wv * T_vc * T_cb;
    data->pos_ = T_wb.translation();
    data->vel_ = T_wv.linear() * data->q_.toRotationMatrix() * data->vel_;
    data->q_ = Eigen::Quaterniond(T_wb.linear());
    data->cov_ *= 10;

    msf.inputVIO(data);

    // std::cout << data->q_.coeffs().transpose()
}

void state_estimator_warpper::mapLocCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::poseData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = multiSensorFusion::MapLoc;

    Eigen::Vector3d pos;
    pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Isometry3d Tmc = Eigen::Isometry3d::Identity();
    Tmc.rotate(q);
    Tmc.pretranslate(pos);
    Eigen::Isometry3d Tcb = Eigen::Isometry3d::Identity();
    Tcb.linear() << 0, -1, 0,
                    0, 0, -1,
                    1, 0, 0;
    Tcb.pretranslate(Eigen::Vector3d(0.0, 0.035, -0.03));
    Eigen::Isometry3d Tmb = Tmc * Tcb;
    Eigen::Map<const Eigen::Matrix<double, 6, 6> > poseCov(msg->pose.covariance.data());

    data->pos_ = Tmb.translation();
    data->q_ = Tmb.linear();
    data->cov_ = poseCov;

    msf.inputMapLoc(data);
}

void state_estimator_warpper::waypointCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
    auto data = std::make_shared<multiSensorFusion::posData>();

    data->timestamp_ = msg->header.stamp.toSec();
    data->type_ = multiSensorFusion::Waypoint;
    tf::pointMsgToEigen(msg->point, data->pos_);

    msf.inputWaypoint(data);
}

void state_estimator_warpper::stateCallback(const ros::TimerEvent &event)
{
    if (msf.isInitialized())
    {
        geometry_msgs::PoseStamped local_pose;
        nav_msgs::Odometry local_odom;

        multiSensorFusion::baseState currentState = msf.outputCurrentState();

        ros::Time t = ros::Time(currentState.timestamp_);

        local_odom.header.stamp = local_pose.header.stamp = t;
        local_odom.header.frame_id = local_pose.header.frame_id = "t265_odom_frame";
        tf::pointEigenToMsg(currentState.pos_, local_odom.pose.pose.position);
        tf::pointEigenToMsg(currentState.pos_, local_pose.pose.position);
        tf::quaternionEigenToMsg(currentState.q_, local_odom.pose.pose.orientation);
        tf::quaternionEigenToMsg(currentState.q_, local_pose.pose.orientation);
        tf::vectorEigenToMsg(currentState.vel_, local_odom.twist.twist.linear);

        pose_pub.publish(local_pose);
        odom_pub.publish(local_odom);

        path.header.stamp = t;
        path.header.frame_id = "t265_odom_frame";
        path.poses.push_back(local_pose);
        path_pub.publish(path);
    }
}