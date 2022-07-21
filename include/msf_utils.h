/**
 * @file msf_utils.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief some tools
 * @version 0.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef MULTI_SENSOR_FUSION_MSF_UTILS_H
#define MULTI_SENSOR_FUSION_MSF_UTILS_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <LocalCartesian.hpp>

namespace multiSensorFusion
{

    constexpr double degreeToRadian = M_PI / 180;

    static Eigen::Quaterniond getQuaternionFromAngle(const Eigen::Vector3d &theta)
    {
        const double q_squared = theta.squaredNorm() / 4.0;

        if (q_squared < 1)
            return {sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5};
        else
        {
            const double w = 1.0 / sqrt(1 + q_squared);
            const double f = w * 0.5;
            return {w, theta[0] * f, theta[1] * f, theta[2] * f};
        }
    }

    static Eigen::Isometry3d
    getTFromRotAndTranslation(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation)
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = rotation.toRotationMatrix();
        T.pretranslate(translation);
        return T;
    }

    static Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &vec)
    {
        return (Eigen::Matrix3d() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    static Eigen::Vector3d convertLlaToENU(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &cur_lla)
    {
        Eigen::Vector3d cur_enu;
        static GeographicLib::LocalCartesian local_cartesian;
        local_cartesian.Reset(init_lla[0], init_lla[1], init_lla[2]);
        local_cartesian.Forward(cur_lla[0], cur_lla[1], cur_lla[2], cur_enu[0], cur_enu[1], cur_enu[2]);
        return cur_enu;
    }

    static Eigen::Vector3d convertENUToLla(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &cur_enu)
    {
        Eigen::Vector3d cur_lla;
        static GeographicLib::LocalCartesian local_cartesian;
        local_cartesian.Reset(init_lla[0], init_lla[1], init_lla[2]);
        local_cartesian.Reverse(cur_enu[0], cur_enu[1], cur_enu[2], cur_lla[0], cur_lla[1], cur_lla[2]);
        return cur_lla;
    }

} // namespace multiSensorFusion

#endif //MULTI_SENSOR_FUSION_MSF_UTILS_H