/**
 * @file msf_vio_processor.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_vio_processor.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_VIO_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_VIO_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_vio_processor
    {
    public:
        msf_vio_processor();

        msf_vio_processor(const Eigen::Vector3d &imu_p_vio, const Eigen::Quaterniond &imu_q_vio,
                          bool update_transformation);

        ~msf_vio_processor() = default;

        void updateState(baseState &currentState, const vioData &data);

    private:
        // the transformation from VIO's coordinate system to imu's coordinate system
        Eigen::Vector3d imu_p_vio_;
        Eigen::Quaterniond imu_q_vio_;
        Eigen::Matrix<double, 6, 6> cov_;

        bool update_transformation_;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_VIO_PROCESSOR_H
