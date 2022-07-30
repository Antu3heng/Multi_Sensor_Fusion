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

#ifndef MULTI_SENSOR_FUSION_MSF_ODOM_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_ODOM_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_odom_processor
    {
    public:
        msf_odom_processor() = delete;

        msf_odom_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor);

        msf_odom_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor,
                           Eigen::Vector3d local_p_global, const Eigen::Quaterniond &local_q_global);

        ~msf_odom_processor() = default;

        void fixNoise(double n_pos, double n_q, double n_v);

        void setInitTransformation(const baseStatePtr &state, const odomDataPtr &data);

        void updateTransformation(baseStatePtr &currentState, const odomDataPtr &data);

        void updateState(baseStatePtr &currentState, const odomDataPtr &data);

        void update(baseStatePtr &currentState, const odomDataPtr &data);

        void transformStateToGlobal(baseStatePtr &state);

        bool has_init_transformation_{};

    private:
        // the transformation from odom sensor's body coordinate system to MSF's body coordinate system
        Eigen::Vector3d body_p_sensor_;
        Eigen::Quaterniond body_q_sensor_;
        // the transformation from odom sensor's world coordinate system to MSF's world coordinate system
        Eigen::Vector3d local_p_global_;
        Eigen::Quaterniond local_q_global_;
        Eigen::Matrix<double, 12, 12> cov_;

        bool is_use_fixed_noise_ = false;

        // odom noise parameters
        // unit: n_pos_ - m, n_q_ - degree, n_v_ - m/s
        double n_pos_{}, n_q_{}, n_v_{};
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_ODOM_PROCESSOR_H
