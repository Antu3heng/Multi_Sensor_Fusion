/**
 * @file multiSensorsFusion.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of multiSensorsFusion.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_BASE_FUSION_H
#define MULTI_SENSOR_FUSION_BASE_FUSION_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"

namespace multiSensorFusion
{
    struct base_state
    {
        double timestamp_;
        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        Eigen::Quaterniond q_;
        Eigen::Vector3d ba_;
        Eigen::Vector3d bw_;
    };

    class base_fusion
    {
    public:
        base_fusion();

        virtual ~base_fusion() = 0;

        base_state getCurrentState();

        virtual void addState();
    };

}

#endif //MULTI_SENSOR_FUSION_BASE_FUSION_H
