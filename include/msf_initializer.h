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

namespace MSF
{
    class msf_initializer
    {
    public:
        explicit msf_initializer(int imu_buffer_size = 100, double imu_acc_std_limit = 3.0);

        ~msf_initializer() = default;

        void setWorldFrameAsNED();

        void setGravityNorm(double gravity_norm);

        void addIMU(const imuDataPtr &data);

        bool initialize(const baseDataPtr &data, baseStatePtr &currentState);

    private:
        std::map<double, imuDataPtr> imu_buffer_;
        int imu_buffer_size_;
        double imu_acc_std_limit_;
        bool use_ned_frame_ = false;
        double gravity_norm_ = -1.0;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_INITIALIZER_H
