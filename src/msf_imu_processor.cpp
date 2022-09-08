/**
 * @file msf_imu_processor.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the realization of msf_imu_processor.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "msf_imu_processor.h"

namespace MSF
{
    msf_imu_processor::msf_imu_processor(const double &n_a, const double &n_w, const double &n_ba, const double &n_bw)
            : n_a_(n_a), n_w_(n_w), n_ba_(n_ba), n_bw_(n_bw)
    {}

    void msf_imu_processor::predict(const baseStatePtr &lastState, baseStatePtr &currentState) const
    {
        predictState(lastState, currentState);
        propagateCov(lastState, currentState);
    }

    void msf_imu_processor::predictState(const baseStatePtr &lastState, baseStatePtr &currentState)
    {
        double dt = currentState->timestamp_ - lastState->timestamp_;
        Eigen::Vector3d acc = 0.5 * (lastState->imu_data_->acc_ + currentState->imu_data_->acc_) - lastState->ba_;
        Eigen::Vector3d gyro = 0.5 * (lastState->imu_data_->gyro_ + currentState->imu_data_->gyro_) - lastState->bw_;
        Eigen::Vector3d theta = gyro * dt;

        // attitude
        Eigen::Quaterniond dq = getQuaternionFromAngle(theta);
        currentState->q_ = lastState->q_ * dq;
        currentState->q_.normalized();
        // velocity
        Eigen::Vector3d deltaVel = (currentState->q_ * acc + lastState->g_) * dt;
        currentState->vel_ = lastState->vel_ + deltaVel;
        // position
        currentState->pos_ = lastState->pos_ + 0.5 * (lastState->vel_ + currentState->vel_) * dt;
        // ba
        currentState->ba_ = lastState->ba_;
        // bw
        currentState->bw_ = lastState->bw_;
        // g
        currentState->g_ = lastState->g_;
    }

    void msf_imu_processor::propagateCov(const baseStatePtr &lastState, baseStatePtr &currentState) const
    {
        double dt = currentState->timestamp_ - lastState->timestamp_;
        Eigen::Vector3d acc = 0.5 * (lastState->imu_data_->acc_ + currentState->imu_data_->acc_) - lastState->ba_;
        Eigen::Vector3d gyro = 0.5 * (lastState->imu_data_->gyro_ + currentState->imu_data_->gyro_) - lastState->bw_;
        Eigen::Matrix3d R = currentState->q_.toRotationMatrix();

        // propagate the covariance
        // TODO: consider use the diff to calculate the jacobi
        // <<Quaternion kinematics for the error-state Kalman filter>>
        // Matrix F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(18, 18);
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;
        F.block<3, 3>(3, 9) = -R * dt;
        F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(gyro) * dt;
        // F.block<3, 3>(6, 6) = R.transpose() * skew_symmetric(gyro) * dt; It is wrong!!!
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
        // Matrix Q
        Eigen::MatrixXd Fi = Eigen::MatrixXd::Zero(18, 12);
        Fi.block<12, 12>(3, 0) = Eigen::MatrixXd::Identity(12, 12);
        Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(12, 12);
        Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * n_a_ * n_a_ * dt * dt;
        Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * n_w_ * n_w_ * dt * dt;
        // Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * n_a_ * n_a_ * dt;
        // Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * n_w_ * n_w_ * dt;
        // TODO: still have some confusion on random walk parameters
        Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * n_ba_ * n_ba_ * dt;
        Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * n_bw_ * n_bw_ * dt;
        // Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * n_ba_ * n_ba_;
        // Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * n_bw_ * n_bw_;
        Eigen::MatrixXd Q = Fi * Qi * Fi.transpose();
        currentState->cov_ = F * lastState->cov_ * F.transpose() + Q;
        currentState->cov_ = (currentState->cov_ + currentState->cov_.transpose()) / 2.0;
    }
}