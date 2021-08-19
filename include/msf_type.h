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

namespace multiSensorFusion
{
    enum sensorType
    {
        IMU,
        VIO,
        GPS,
        MapLoc
    };

    struct imuData
    {
        double timestamp_;
        Eigen::Vector3d acc_;
        Eigen::Vector3d gyro_;
    };
    using imuDataPtr = std::shared_ptr<imuData>;

    struct vioData
    {
        double timestamp_;
        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        Eigen::Quaterniond q_;
        Eigen::Matrix<double, 9, 9> cov_;
    };
    using vioDataPtr = std::shared_ptr<vioData>;

    struct mapLocData
    {
        double timestamp_;
        Eigen::Vector3d pos_;
        Eigen::Quaterniond q_;
    };
    using mapLocDataPtr = std::shared_ptr<mapLocData>;

    struct gpsData
    {
        double timestamp_;
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

        Eigen::Matrix<double, 15, 15> cov_;

        bool isWithMap_;
        Eigen::Vector3d posInMap_;
        Eigen::Vector3d velInMap_;
        Eigen::Quaterniond qInMap_;

        imuDataPtr imuData_;
    };
    using baseStatePtr = std::shared_ptr<baseState>;
}

#endif //MULTI_SENSOR_FUSION_MSF_TYPE_H
