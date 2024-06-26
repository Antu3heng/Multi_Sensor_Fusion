/**
 * @file msf_mapLoc_processor.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the realization of msf_mapLoc_processor.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "msf_pose_processor.h"

#include <utility>

namespace MSF
{
    msf_pose_processor::msf_pose_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor)
            : body_p_sensor_(std::move(body_p_sensor)), body_q_sensor_(body_q_sensor)
    {
        is_use_fixed_noise_ = false;
        has_init_transformation_ = false;
        cov_for_T_bs_ = cov_for_T_lg_ = Eigen::Matrix<double, 6, 6>::Identity();
        cov_for_T_bs_.block<3, 3>(0, 0) = cov_for_T_lg_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1 * 0.1;
        cov_for_T_bs_.block<3, 3>(3, 3) = cov_for_T_lg_.block<3, 3>(3, 3) =
                Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
    }

    msf_pose_processor::msf_pose_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor,
                                           Eigen::Vector3d local_p_global, const Eigen::Quaterniond &local_q_global)
            : body_p_sensor_(std::move(body_p_sensor)), body_q_sensor_(body_q_sensor),
              local_p_global_(std::move(local_p_global)), local_q_global_(local_q_global)
    {
        is_use_fixed_noise_ = false;
        has_init_transformation_ = true;
        cov_for_T_bs_ = cov_for_T_lg_ = Eigen::Matrix<double, 6, 6>::Identity();
        cov_for_T_bs_.block<3, 3>(0, 0) = cov_for_T_lg_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1 * 0.1;
        cov_for_T_bs_.block<3, 3>(3, 3) = cov_for_T_lg_.block<3, 3>(3, 3) =
                Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
    }

    void msf_pose_processor::fixNoise(double n_pos, double n_q)
    {
        is_use_fixed_noise_ = true;
        n_pos_ = n_pos;
        n_q_ = n_q;
    }

    void msf_pose_processor::setInitTransformation(const baseStatePtr &state, const poseDataPtr &data)
    {
        has_init_transformation_ = true;
        Eigen::Isometry3d global_T_sensor = getTFromRotAndTranslation(data->pos_, data->q_);
        Eigen::Isometry3d body_T_sensor = getTFromRotAndTranslation(body_p_sensor_, body_q_sensor_);
        Eigen::Isometry3d body_T_global = body_T_sensor * global_T_sensor.inverse();
        Eigen::Isometry3d local_T_body = getTFromRotAndTranslation(state->pos_, state->q_);
        Eigen::Isometry3d local_T_global_ = local_T_body * body_T_global;
        local_p_global_ = local_T_global_.translation();
        local_q_global_ = local_T_global_.linear();
        local_q_global_.normalized();
    }

    void msf_pose_processor::updateTransformation(baseStatePtr &currentState, const poseDataPtr &data)
    {
        // update the transformation state
        // residuals
        Eigen::VectorXd dz = Eigen::VectorXd::Zero(6);
        dz.segment(0, 3) = currentState->pos_ -
                           (local_q_global_ * data->pos_ + local_p_global_ - currentState->q_ * body_p_sensor_);
        Eigen::Quaterniond dq =
                (local_q_global_ * data->q_ * body_q_sensor_.conjugate()).conjugate() * currentState->q_;
        Eigen::AngleAxisd dtheta(dq);
        dz.segment(3, 3) = dtheta.axis() * dtheta.angle();

        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 3) = -local_q_global_.toRotationMatrix() * skew_symmetric(data->pos_);
        H.block<3, 3>(3, 3) = body_q_sensor_.toRotationMatrix() * data->q_.toRotationMatrix().transpose();

        // Matrix R
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
        // R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.2 * 0.2;
        R.block<3, 3>(0, 0) = currentState->cov_.block<3, 3>(0, 0);
        // R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
        R.block<3, 3>(3, 3) = currentState->cov_.block<3, 3>(6, 6);

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

    void msf_pose_processor::updateState(baseStatePtr &currentState, const poseDataPtr &data)
    {
        // update the Multi-sensor fusion state
        // residuals
        Eigen::VectorXd dz = Eigen::VectorXd::Zero(6);
        dz.segment(0, 3) = data->pos_ - local_q_global_.conjugate() *
                                        (currentState->q_ * body_p_sensor_ + currentState->pos_ - local_p_global_);
        Eigen::Quaterniond dq =
                (local_q_global_.conjugate() * currentState->q_ * body_q_sensor_).conjugate() * data->q_;
        Eigen::AngleAxisd dtheta(dq);
        dz.segment(3, 3) = dtheta.axis() * dtheta.angle();

        // update the covariance and the error state
        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 24);
        H.block<3, 3>(0, 0) = local_q_global_.conjugate().toRotationMatrix();
        H.block<3, 3>(0, 6) = -local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix() *
                              skew_symmetric(body_p_sensor_);
        H.block<3, 3>(0, 18) = local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix();
        H.block<3, 3>(3, 6) = body_q_sensor_.toRotationMatrix().transpose();
        H.block<3, 3>(3, 21) = Eigen::Matrix3d::Identity();

        // Matrix R
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
        if (is_use_fixed_noise_)
        {
            R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * n_pos_ * n_pos_;
            R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * n_q_ * degreeToRadian * n_q_ * degreeToRadian;
        } else
            R = data->cov_;

        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(24, 24);
        cov.block<18, 18>(0, 0) = currentState->cov_;
        cov.block<6, 6>(18, 18) = cov_for_T_bs_;

        // TODO: these code is the same as msf_odom_processor's, which can use virtual and base class
        // update the states and the covariance
        Eigen::MatrixXd S = H * cov * H.transpose() + R;
        Eigen::MatrixXd K = cov * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(24, 24);
        // cov = (I - K * H) * cov;
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
        G.block<3, 3>(21, 21) -= skew_symmetric(0.5 * delta_states.segment(21, 3));
        cov = G * cov * G.transpose();
        cov = (cov + cov.transpose()) / 2.0;

        currentState->cov_ = cov.block<18, 18>(0, 0);
        cov_for_T_bs_ = cov.block<6, 6>(18, 18);
    }

    void msf_pose_processor::update(baseStatePtr &currentState, const poseDataPtr &data)
    {
        updateTransformation(currentState, data);
        updateState(currentState, data);
    }

    void msf_pose_processor::transformStateToGlobal(baseStatePtr &state)
    {
        state->pos_in_global_ = local_q_global_.conjugate() * (state->pos_ - local_p_global_);
        state->vel_in_global_ = local_q_global_.conjugate() * state->vel_;
        state->q_in_global_ = local_q_global_.conjugate() * state->q_;
        state->local_p_global_ = local_p_global_;
        state->local_q_global_ = local_q_global_;
    }
}