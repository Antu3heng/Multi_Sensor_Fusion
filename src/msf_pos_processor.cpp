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

#include "msf_pos_processor.h"

#include <utility>

namespace multiSensorFusion
{
    msf_pos_processor::msf_pos_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor,
                                         Eigen::Vector3d local_p_global, const Eigen::Quaterniond &local_q_global)
            : body_p_sensor_(std::move(body_p_sensor)), body_q_sensor_(body_q_sensor),
              local_p_global_(std::move(local_p_global)), local_q_global_(local_q_global)
    {
        is_use_fixed_noise_ = false;
        cov_ = Eigen::Matrix<double, 12, 12>::Identity();
        cov_.block<3, 3>(0, 0) = cov_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;
        cov_.block<3, 3>(3, 3) = cov_.block<3, 3>(9, 9) =
                Eigen::Matrix3d::Identity() * 1.0 * degreeToRadian * 1.0 * degreeToRadian;
    }

    void msf_pos_processor::fixNoise(double n_pos)
    {
        is_use_fixed_noise_ = true;
        n_pos_ = n_pos;
    }

    void msf_pos_processor::updateTransformation(baseStatePtr &currentState, const posDataPtr &data)
    {
        // update the transformation state
        // residuals
        Eigen::Vector3d dz = currentState->pos_ -
                             (local_q_global_ * data->pos_ + local_p_global_ - currentState->q_ * body_p_sensor_);

        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 12);
        H.block<3, 3>(0, 0) = -currentState->q_.toRotationMatrix();
        H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 9) = -local_q_global_.toRotationMatrix() * skew_symmetric(data->pos_);

        // Matrix R
        Eigen::Matrix3d R = currentState->cov_.block<3, 3>(0, 0);
        // Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.2 * 0.2;

        // update
        Eigen::MatrixXd S = H * cov_ * H.transpose() + R;
        Eigen::MatrixXd K = cov_ * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12, 12);
        // cov_ = (I - K * H) * cov_;
        cov_ = (I - K * H) * cov_ * (I - K * H).transpose() + K * R * K.transpose();

        Eigen::VectorXd delta_states = K * dz;
        body_p_sensor_ += delta_states.segment(0, 3);
        Eigen::Quaterniond delta_q1 = getQuaternionFromAngle(delta_states.segment(3, 3));
        body_q_sensor_ *= delta_q1;
        body_q_sensor_.normalized();
        local_p_global_ += delta_states.segment(6, 3);
        Eigen::Quaterniond delta_q2 = getQuaternionFromAngle(delta_states.segment(9, 3));
        local_q_global_ *= delta_q2;
        local_q_global_.normalized();

        // ESKF reset
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(12, 12);
        G.block<3, 3>(3, 3) -= skew_symmetric(0.5 * delta_states.segment(3, 3));
        G.block<3, 3>(9, 9) -= skew_symmetric(0.5 * delta_states.segment(9, 3));
        cov_ = G * cov_ * G.transpose();
        cov_ = (cov_ + cov_.transpose()) / 2.0;
    }

    void msf_pos_processor::updateState(baseStatePtr &currentState, const posDataPtr &data)
    {
        // update the Multi-sensor fusion state
        // residuals
        Eigen::Vector3d dz = data->pos_ - local_q_global_.conjugate() *
                                          (currentState->q_ * body_p_sensor_ + currentState->pos_ - local_p_global_);

        // update the covariance and the error state
        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 18);
        H.block<3, 3>(0, 0) = local_q_global_.conjugate().toRotationMatrix();
        H.block<3, 3>(0, 6) = -local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix() *
                              skew_symmetric(body_p_sensor_);

        // Matrix R
        Eigen::Matrix3d R;
        if (is_use_fixed_noise_)
            R = Eigen::Matrix3d::Identity() * n_pos_ * n_pos_;
        else
            R = data->cov_;

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
        currentState->cov_ = (currentState->cov_ + currentState->cov_.transpose()) / 2.0;
    }

    void msf_pos_processor::update(baseStatePtr &currentState, const posDataPtr &data)
    {
        updateTransformation(currentState, data);
        updateState(currentState, data);
    }

    void msf_pos_processor::transformStateToGlobal(baseStatePtr &state)
    {
        state->pos_in_global_ = local_q_global_.conjugate() * (state->pos_ - local_p_global_);
        state->vel_in_global_ = local_q_global_.conjugate() * state->vel_;
        state->q_in_global_ = local_q_global_.conjugate() * state->q_;
        state->local_p_global_ = local_p_global_;
        state->local_q_global_ = local_q_global_;
    }
}