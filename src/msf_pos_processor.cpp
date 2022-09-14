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

namespace MSF
{
    msf_pos_processor::msf_pos_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor,
                                         Eigen::Vector3d local_p_global, const Eigen::Quaterniond &local_q_global)
            : body_p_sensor_(std::move(body_p_sensor)), body_q_sensor_(body_q_sensor),
              local_p_global_(std::move(local_p_global)), local_q_global_(local_q_global)
    {
        is_use_fixed_noise_ = false;
        cov_for_T_bs_ = cov_for_T_lg_ = Eigen::Matrix<double, 6, 6>::Identity();
        cov_for_T_bs_.block<3, 3>(0, 0) = cov_for_T_lg_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1 * 0.1;
        cov_for_T_bs_.block<3, 3>(3, 3) = cov_for_T_lg_.block<3, 3>(3, 3) =
                Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
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
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 3) = -local_q_global_.toRotationMatrix() * skew_symmetric(data->pos_);

        // Matrix R
        Eigen::Matrix3d R = currentState->cov_.block<3, 3>(0, 0);

        // update
        Eigen::MatrixXd S = H * cov_for_T_lg_ * H.transpose() + R;
        Eigen::MatrixXd K = cov_for_T_lg_ * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        // cov_for_T_lg_ = (I - K * H) * cov_for_T_lg_;
        cov_for_T_lg_ = (I - K * H) * cov_for_T_lg_ * (I - K * H).transpose() + K * R * K.transpose();

        Eigen::VectorXd delta_states = K * dz;
        local_p_global_ += delta_states.segment(0, 3);
        Eigen::Quaterniond delta_q = getQuaternionFromAngle(delta_states.segment(3, 3));
        local_q_global_ *= delta_q;
        local_q_global_.normalized();

        // ESKF reset
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(6, 6);
        G.block<3, 3>(3, 3) -= skew_symmetric(0.5 * delta_states.segment(3, 3));
        cov_for_T_lg_ = G * cov_for_T_lg_ * G.transpose();
        cov_for_T_lg_ = (cov_for_T_lg_ + cov_for_T_lg_.transpose()) / 2.0;
    }

    void msf_pos_processor::updateState(baseStatePtr &currentState, const posDataPtr &data)
    {
        // update the Multi-sensor fusion state
        // residuals
        Eigen::Vector3d dz = data->pos_ - local_q_global_.conjugate() *
                                          (currentState->q_ * body_p_sensor_ + currentState->pos_ - local_p_global_);

        // update the covariance and the error state
        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 24);
        H.block<3, 3>(0, 0) = local_q_global_.conjugate().toRotationMatrix();
        H.block<3, 3>(0, 6) = -local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix() *
                              skew_symmetric(body_p_sensor_);
        H.block<3, 3>(0, 18) = local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix();

        // Matrix R
        Eigen::Matrix3d R;
        if (is_use_fixed_noise_)
            R = Eigen::Matrix3d::Identity() * n_pos_ * n_pos_;
        else
            R = data->cov_;

        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(24, 24);
        cov.block<18, 18>(0, 0) = currentState->cov_;
        cov.block<6, 6>(18, 18) = cov_for_T_bs_;

        // update the states and the covariance
        Eigen::MatrixXd S = H * cov * H.transpose() + R;
        Eigen::MatrixXd K = cov * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(24, 24);
        cov = (I - K * H) * cov * (I - K * H).transpose() + K * R * K.transpose();

        Eigen::VectorXd delta_states = K * dz;
        currentState->pos_ += delta_states.segment(0, 3);
        currentState->vel_ += delta_states.segment(3, 3);
        Eigen::Quaterniond delta_q = getQuaternionFromAngle(delta_states.segment(6, 3));
        currentState->q_ *= delta_q;
        currentState->q_.normalized();
        currentState->ba_ += delta_states.segment(9, 3);
        currentState->bw_ += delta_states.segment(12, 3);
        currentState->g_ += delta_states.segment(15, 3);
        body_p_sensor_ += delta_states.segment(18, 3);
        Eigen::Quaterniond delta_q_for_T_bs_ = getQuaternionFromAngle(delta_states.segment(21, 3));
        body_q_sensor_ *= delta_q_for_T_bs_;
        body_q_sensor_.normalized();

        // ESKF reset
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(24, 24);
        G.block<3, 3>(6, 6) -= skew_symmetric(0.5 * delta_states.segment(6, 3));
        G.block<3, 3>(6, 6) -= skew_symmetric(0.5 * delta_states.segment(21, 3));
        cov = G * cov * G.transpose();
        cov = (cov + cov.transpose()) / 2.0;

        currentState->cov_ = cov.block<18, 18>(0, 0);
        cov_for_T_bs_ = cov.block<6, 6>(18, 18);
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