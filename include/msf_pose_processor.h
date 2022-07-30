/**
 * @file msf_mapLoc_processor.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_mapLoc_processor.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_POSE_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_POSE_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_pose_processor
    {
    public:
        msf_pose_processor() = delete;

        msf_pose_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor);

        msf_pose_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor,
                           Eigen::Vector3d local_p_global, const Eigen::Quaterniond &local_q_global);

        ~msf_pose_processor() = default;

        void fixNoise(double n_pos, double n_q);

        void setInitTransformation(const baseStatePtr &state, const poseDataPtr &data);

        void updateTransformation(baseStatePtr &currentState, const poseDataPtr &data);

        void updateState(baseStatePtr &currentState, const poseDataPtr &data);

        void update(baseStatePtr &currentState, const poseDataPtr &data);

        void transformStateToGlobal(baseStatePtr &state);

        bool has_init_transformation_{};

    private:
        // the transformation from pose sensor to body
        Eigen::Vector3d body_p_sensor_;
        Eigen::Quaterniond body_q_sensor_;
        // the transformation from pose sensor's world coordinate system to MSF's world coordinate system
        Eigen::Vector3d local_p_global_;
        Eigen::Quaterniond local_q_global_;
        Eigen::Matrix<double, 12, 12> cov_;

        bool is_use_fixed_noise_ = false;

        // pose noise parameters
        // unit: n_pos_ - m, n_q_ - degree
        double n_pos_{}, n_q_{};
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_POSE_PROCESSOR_H
