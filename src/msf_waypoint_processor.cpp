/**
 * @file msf_waypoint_processor.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_imu_processor.cpp
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "msf_waypoint_processor.h"

namespace multiSensorFusion
{
    void msf_waypoint_processor::updateState(baseStatePtr &currentState, const posDataPtr &data)
    {
        // update the Multi-sensor fusion state
        // residuals
        Eigen::Vector3d dz = data->pos_ - currentState->pos_;

        // update the covariance and the error state
        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 18);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        // Matrix R
        Eigen::Matrix3d R = data->cov_;

        // update the states and the covariance
        Eigen::MatrixXd S = H * currentState->cov_ * H.transpose() + R;
        Eigen::MatrixXd K = currentState->cov_ * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(18, 18);
        currentState->cov_ = (I - K * H) * currentState->cov_ * (I - K * H).transpose() + K * R * K.transpose();

        Eigen::VectorXd delta_states = K * dz;
        currentState->pos_ += delta_states.segment(0, 3);
        currentState->vel_ += delta_states.segment(3, 3);
        Eigen::Quaterniond delta_q = getQuaternionFromAngle(delta_states.segment(6, 3));
        currentState->q_ *= delta_q;
        currentState->q_.normalized();
        currentState->ba_ += delta_states.segment(9, 3);
        currentState->bw_ += delta_states.segment(12, 3);
        currentState->g_ += delta_states.segment(15, 3);

        // ESKF reset
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(18, 18);
        G.block<3, 3>(6, 6) -= skew_symmetric(0.5 * delta_states.segment(6, 3));
        currentState->cov_ = G * currentState->cov_ * G.transpose();
        currentState->cov_ = (currentState->cov_ + currentState->cov_.transpose()) / 2;
    }
}