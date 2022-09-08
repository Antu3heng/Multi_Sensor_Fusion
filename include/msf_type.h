/**
 * @file msf_type.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_type.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_TYPE_H
#define MULTI_SENSOR_FUSION_MSF_TYPE_H

#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace MSF
{
    enum dataType
    {
        IMU,
        GPS,
        Pos,
        Pose,
        Odom
    };

    struct baseData
    {
    public:
        virtual ~baseData() = default;;

        double timestamp_;
        dataType type_;
        std::string name_;
    };
    using baseDataPtr = std::shared_ptr<baseData>;

    struct imuData : public baseData
    {
        Eigen::Vector3d acc_;
        Eigen::Vector3d gyro_;
    };
    using imuDataPtr = std::shared_ptr<imuData>;

    struct posData : public baseData
    {
        Eigen::Vector3d pos_;
        Eigen::Matrix3d cov_;
    };
    using posDataPtr = std::shared_ptr<posData>;

    struct poseData : public posData
    {
        Eigen::Quaterniond q_;
        Eigen::Matrix<double, 6, 6> cov_;
    };
    using poseDataPtr = std::shared_ptr<poseData>;

    struct odomData : public poseData
    {
        Eigen::Vector3d vel_;
        Eigen::Vector3d angler_vel_;
        Eigen::Matrix<double, 12, 12> cov_;
    };
    using odomDataPtr = std::shared_ptr<odomData>;

    struct gpsData : public baseData
    {
        Eigen::Vector3d lla_;
        Eigen::Matrix3d cov_;
    };
    using gpsDataPtr = std::shared_ptr<gpsData>;

    struct baseState
    {
        double timestamp_;

        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        Eigen::Quaterniond q_;
        Eigen::Vector3d ba_;
        Eigen::Vector3d bw_;
        Eigen::Vector3d g_;

        Eigen::Matrix<double, 18, 18> cov_;

        bool has_global_state_;
        Eigen::Vector3d pos_in_global_;
        Eigen::Vector3d vel_in_global_;
        Eigen::Quaterniond q_in_global_;
        Eigen::Vector3d local_p_global_;
        Eigen::Quaterniond local_q_global_;

        imuDataPtr imu_data_;
    };
    using baseStatePtr = std::shared_ptr<baseState>;
}

#endif //MULTI_SENSOR_FUSION_MSF_TYPE_H
