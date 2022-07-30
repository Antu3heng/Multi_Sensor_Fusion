/**
 * @file msf_waypoint_processor.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_imu_processor.cpp
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_POS_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_POS_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_pos_processor
    {
    public:
        msf_pos_processor() = delete;

        msf_pos_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor,
                          Eigen::Vector3d local_p_global, const Eigen::Quaterniond &local_q_global);

        ~msf_pos_processor() = default;

        void fixNoise(double n_pos);

        void updateTransformation(baseStatePtr &currentState, const posDataPtr &data);

        void updateState(baseStatePtr &currentState, const posDataPtr &data);

        void update(baseStatePtr &currentState, const posDataPtr &data);

        void transformStateToGlobal(baseStatePtr &state);

    private:
        // the transformation from position sensor's body coordinate system to MSF's body coordinate system
        Eigen::Vector3d body_p_sensor_;
        Eigen::Quaterniond body_q_sensor_;
        // the transformation from position sensor's world coordinate system to MSF's world coordinate system
        Eigen::Vector3d local_p_global_;
        Eigen::Quaterniond local_q_global_;
        Eigen::Matrix<double, 12, 12> cov_;

        bool is_use_fixed_noise_ = false;

        // position noise parameter
        // unit: n_pos_ - m
        double n_pos_{};
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_POS_PROCESSOR_H
