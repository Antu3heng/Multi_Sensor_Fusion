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

namespace multiSensorFusion
{

    msf_initializer::msf_initializer()
    {
        init_imu_p_vio_ = Eigen::Vector3d::Zero();
        init_imu_q_vio_ = Eigen::Quaterniond::Identity();
    }

    msf_initializer::msf_initializer(Eigen::Vector3d init_imu_p_vio, Eigen::Quaterniond init_imu_q_vio)
            : init_imu_p_vio_(init_imu_p_vio), init_imu_q_vio_(init_imu_q_vio)
    {}

    void msf_initializer::addIMU(const imuData &data)
    {
        imu_buffer_.push_back(data);
        if(imu_buffer_.size() > nImuBuffer_)
                imu_buffer_.pop_front();
    }

    bool msf_initializer::initializeUsingVIO(const vioData &data, baseState &currentState)
    {
        if(imu_buffer_.size() < nImuBuffer_)
        {
            std::cerr << "[msf_initializer]: No enough IMU data!" << std::endl;
            return false;
        }
        //
        // state_.pos_ = vioPosRaw;
        // state_.vel_ = vioVelRaw;
        // state_.q_ = vioRotRaw;
        //
        // MatrixXd P = MatrixXd::Zero(nStates_, nStates_);
        // // P.block<9, 9>(0, 0) = vioCovariance;
        // P.block<6, 6>(0, 0) = Matrix<double, 6, 6>::Identity();
        // P.block<3, 3>(6, 6) = Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
        // P.block<3, 3>(9, 9) = Matrix3d::Identity() * 0.1 * 0.1;
        // P.block<3, 3>(12, 12) = Matrix3d::Identity() * 0.01 * 0.01;
        // P.block<3, 3>(15, 15) = Matrix3d::Identity() * 0.01 * 0.01;

        return false;
    }

}