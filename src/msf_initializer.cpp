/**
 * @file msf_initializer.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the realization of msf_initializer.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "msf_initializer.h"

#include <utility>

namespace MSF
{
    msf_initializer::msf_initializer(int imu_buffer_size, double imu_acc_std_limit)
            : imu_buffer_size_(imu_buffer_size), imu_acc_std_limit_(imu_acc_std_limit)
    {}

    void msf_initializer::addIMU(const imuDataPtr &data)
    {
        imu_buffer_.insert(std::pair<double, imuDataPtr>(data->timestamp_, data));
        if (imu_buffer_.size() > imu_buffer_size_)
            imu_buffer_.erase(imu_buffer_.begin());
    }

    bool msf_initializer::initialize(const baseDataPtr &data, baseStatePtr &currentState)
    {
        if (imu_buffer_.size() < imu_buffer_size_)
        {
#ifdef TEST_DEBUG
            std::cerr << "[msf_initializer]: No enough IMU data!" << std::endl;
#endif
            return false;
        }

        auto it = imu_buffer_.lower_bound(data->timestamp_ - 0.003);
        if (it != imu_buffer_.end())
        {
            if (fabs(it->first - data->timestamp_) > 0.003)
            {
#ifdef TEST_DEBUG
                std::cerr << "[msf_initializer]: IMU and the input data are not synchronized! Time interval: " << fabs(it->first - data->timestamp_) << std::endl;
#endif
                return false;
            } else
            {
                // init the state with imu data
                Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
                for (auto &imu: imu_buffer_)
                    sum_acc += imu.second->acc_;
                Eigen::Vector3d mean_acc = sum_acc / (double) imu_buffer_.size();
                Eigen::Vector3d sum_err2 = Eigen::Vector3d::Zero();
                for (auto &imu: imu_buffer_)
                    sum_err2 += (imu.second->acc_ - mean_acc).cwiseAbs2();
                Eigen::Vector3d std_acc = (sum_err2 / (double) imu_buffer_.size()).cwiseSqrt();
                if (std_acc.maxCoeff() > imu_acc_std_limit_)
                {
                    std::cerr << "IMU acc std is too big: " << std_acc.transpose() << std::endl;
                    return false;
                }

                currentState->timestamp_ = it->first;
                currentState->imu_data_ = it->second;
                currentState->pos_ = currentState->vel_ = currentState->ba_ = currentState->bw_ = Eigen::Vector3d::Zero();
                currentState->g_ = Eigen::Vector3d(0., 0., -1) * mean_acc.norm();
                Eigen::Vector3d z_axis = mean_acc.normalized();
                Eigen::Vector3d x_axis = (Eigen::Vector3d::UnitX() -
                                          z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX()).normalized();
                Eigen::Vector3d y_axis = (z_axis.cross(x_axis)).normalized();
                Eigen::Matrix3d body_R_FLU;
                body_R_FLU.block<3, 1>(0, 0) = x_axis;
                body_R_FLU.block<3, 1>(0, 1) = y_axis;
                body_R_FLU.block<3, 1>(0, 2) = z_axis;
                // TODO: check the orientation
                currentState->q_ = body_R_FLU.transpose();

                currentState->cov_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
                currentState->cov_.block<3, 3>(6, 6) =
                        Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
                currentState->cov_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.1 * 0.1;
                currentState->cov_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.02 * 0.02;
                currentState->cov_.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;

                currentState->has_global_state_ = false;

                return true;
            }
        } else
        {
#ifdef TEST_DEBUG
            std::cerr << "[msf_initializer]: IMU and the input data are not synchronized!" << std::endl;
#endif
            return false;
        }
    }
}