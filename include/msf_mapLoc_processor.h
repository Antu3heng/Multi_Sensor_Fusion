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

#ifndef MULTI_SENSOR_FUSION_MSF_MAPLOC_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_MAPLOC_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_mapLoc_processor
    {
    public:
        msf_mapLoc_processor();

        msf_mapLoc_processor(const Eigen::Vector3d &imu_p_map, const Eigen::Quaterniond &imu_q_map, double n_pos,
                             double n_q, bool update_transformation);

        ~msf_mapLoc_processor() = default;

        void getInitTransformation(const baseState &state, const mapLocData &data);

        void updateState(baseState &currentState, const mapLocData &data);

        void transformStateToMap(baseState &state);

    private:
        // the transformation from VIO to IMU
        Eigen::Vector3d imu_p_map_;
        Eigen::Quaterniond imu_q_map_;
        Eigen::Matrix<double, 6, 6> cov_;

        bool update_transformation_;

        // map-based localization noise parameters
        // unit: n_pos_-m n_q_-degree
        double n_pos_, n_q_;;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_MAPLOC_PROCESSOR_H
