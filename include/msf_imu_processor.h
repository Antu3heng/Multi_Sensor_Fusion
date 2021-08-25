/**
 * @file msf_imu_processor.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_imu_processor.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_IMU_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_IMU_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_imu_processor
    {
    public:
        msf_imu_processor();

        msf_imu_processor(const double &n_a, const double &n_w, const double &n_ba, const double &n_bw,
                          Eigen::Vector3d g);

        ~msf_imu_processor() = default;

        void predict(const baseStatePtr &lastState, baseStatePtr &currentState) const;

        void predictState(const baseStatePtr &lastState, baseStatePtr &currentState) const;

        void propagateCov(const baseStatePtr &lastState, baseStatePtr &currentState) const;

    private:
        // imu noise parameters
        // unit: n_a_-m/s^2/sqrt(hz) n_w_-rad/s/sqrt(hz) n_ba_-m/s^2*sqrt(hz) n_bw_-rad/s*sqrt(hz)
        double n_a_, n_w_, n_ba_, n_bw_;
        Eigen::Vector3d g_;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_IMU_PROCESSOR_H
