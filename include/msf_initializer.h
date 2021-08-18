/**
 * @file msf_initializer.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_initializer.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_INITIALIZER_H
#define MULTI_SENSOR_FUSION_MSF_INITIALIZER_H

#include <iostream>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_initializer
    {
    public:
        msf_initializer();

        msf_initializer(Eigen::Vector3d init_imu_p_vio, Eigen::Quaterniond init_imu_q_vio);

        ~msf_initializer() = default;

        void addIMU(const imuData &data);

        bool initializeUsingVIO(const vioData &data, baseState &currentState);

        // bool initializeUsingGPS(const gpsData &data, baseState &currentState);

    private:
        Eigen::Vector3d init_imu_p_vio_;
        Eigen::Quaterniond init_imu_q_vio_;

        std::map<double, imuData> imu_buffer_;

        const int nImuBuffer_ = 200;
        // const double ImuAccStdLimit_ = 3.0;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_INITIALIZER_H
