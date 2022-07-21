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

namespace multiSensorFusion
{
    msf_initializer::msf_initializer()
    {
        init_imu_p_vio_ = Eigen::Vector3d::Zero();
        init_imu_q_vio_ = Eigen::Quaterniond::Identity();
    }

    msf_initializer::msf_initializer(Eigen::Vector3d init_imu_p_vio, const Eigen::Quaterniond &init_imu_q_vio)
            : init_imu_p_vio_(std::move(init_imu_p_vio)), init_imu_q_vio_(init_imu_q_vio)
    {}

    void msf_initializer::addIMU(const imuDataPtr &data)
    {
        imu_buffer_.insert(std::pair<double, imuDataPtr>(data->timestamp_, data));
        if (imu_buffer_.size() > nImuBuffer_)
            imu_buffer_.erase(imu_buffer_.begin());
    }

    bool msf_initializer::initializeUsingVIO(const odomDataPtr &data, baseStatePtr &currentState)
    {
        if (imu_buffer_.size() < nImuBuffer_)
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
                std::cerr << "[msf_initializer]: IMU and VIO are not synchronized!" << std::endl;
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
                if (std_acc.maxCoeff() > 3)
                {
                    std::cerr << "IMU acc std is too big: " << std_acc.transpose() << std::endl;
                    return false;
                }

                // TODO: some imu data will be abandoned? whether we can use imu processor to use these data?
                currentState->timestamp_ = it->first;
                currentState->imuData_ = it->second;

                currentState->pos_ = init_imu_q_vio_ * data->pos_ + init_imu_p_vio_;
                currentState->vel_ = init_imu_q_vio_ * data->vel_;
                currentState->q_ = init_imu_q_vio_ * data->q_;
                currentState->q_.normalized();
                currentState->ba_ = currentState->bw_ = Eigen::Vector3d::Zero();
                currentState->g_ = Eigen::Vector3d(0., 0., -1) * mean_acc.norm();

                // currentState->cov_.block<9, 9>(0, 0) = data->cov_;
                currentState->cov_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
                currentState->cov_.block<3, 3>(6, 6) =
                        Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
                currentState->cov_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.1 * 0.1;
                currentState->cov_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;
                currentState->cov_.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;

                currentState->isWithMap_ = false;

                return true;
            }
        } else
        {
            std::cerr << "[msf_initializer]: IMU and VIO are not synchronized!" << std::endl;
            return false;
        }
    }
}