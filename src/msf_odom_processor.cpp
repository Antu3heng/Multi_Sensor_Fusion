/**
 * @file msf_vio_processor.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the realization of msf_vio_processor.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
*/

#include "msf_odom_processor.h"

#include <utility>

namespace multiSensorFusion
{
    msf_odom_processor::msf_odom_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor)
            : body_p_sensor_(std::move(body_p_sensor)), body_q_sensor_(body_q_sensor)
    {
        is_use_fixed_noise_ = false;
        has_init_transformation_ = false;
        cov_ = Eigen::Matrix<double, 12, 12>::Identity();
        cov_.block<3, 3>(0, 0) = cov_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;
        cov_.block<3, 3>(3, 3) = cov_.block<3, 3>(9, 9) =
                Eigen::Matrix3d::Identity() * 1.0 * degreeToRadian * 1.0 * degreeToRadian;
    }

    msf_odom_processor::msf_odom_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor,
                                           Eigen::Vector3d local_p_global, const Eigen::Quaterniond &local_q_global)
            : body_p_sensor_(std::move(body_p_sensor)), body_q_sensor_(body_q_sensor),
              local_p_global_(std::move(local_p_global)), local_q_global_(local_q_global)
    {
        is_use_fixed_noise_ = false;
        has_init_transformation_ = true;
        cov_ = Eigen::Matrix<double, 12, 12>::Identity();
        cov_.block<3, 3>(0, 0) = cov_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;
        cov_.block<3, 3>(3, 3) = cov_.block<3, 3>(9, 9) =
                Eigen::Matrix3d::Identity() * 1.0 * degreeToRadian * 1.0 * degreeToRadian;
    }

    void msf_odom_processor::fixNoise(double n_pos, double n_q, double n_v)
    {
        is_use_fixed_noise_ = true;
        n_pos_ = n_pos;
        n_q_ = n_q;
        n_v_ = n_v;
    }

    void msf_odom_processor::setInitTransformation(const baseStatePtr &state, const odomDataPtr &data)
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

    void msf_odom_processor::updateTransformation(baseStatePtr &currentState, const odomDataPtr &data)
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
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 12);
        H.block<3, 3>(0, 0) = -currentState->q_.toRotationMatrix();
        H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 9) = -local_q_global_.toRotationMatrix() * skew_symmetric(data->pos_);
        H.block<3, 3>(3, 3) = -body_q_sensor_.toRotationMatrix();
        H.block<3, 3>(3, 9) = body_q_sensor_.toRotationMatrix() * data->q_.toRotationMatrix().transpose();

        // Matrix R
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
        // R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.2 * 0.2;
        R.block<3, 3>(0, 0) = currentState->cov_.block<3, 3>(0, 0);
        // R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
        R.block<3, 3>(3, 3) = currentState->cov_.block<3, 3>(6, 6);

        // update
        Eigen::MatrixXd S = H * cov_ * H.transpose() + R;
        Eigen::MatrixXd K = cov_ * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12, 12);
        auto lastcov = cov_;
        // cov_ = (I - K * H) * cov_;
        cov_ = (I - K * H) * cov_ * (I - K * H).transpose() + K * R * K.transpose();
        // cov_.block<6, 6>(0, 0) = lastcov.block<6, 6>(0, 0);

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

    void msf_odom_processor::updateState(baseStatePtr &currentState, const odomDataPtr &data)
    {
        // residuals
        Eigen::VectorXd dz = Eigen::VectorXd::Zero(9);
        dz.segment(0, 3) = data->pos_ - local_q_global_.conjugate() *
                                        (currentState->q_ * body_p_sensor_ + currentState->pos_ - local_p_global_);
        dz.segment(3, 3) = data->vel_ - local_q_global_.conjugate() * (currentState->vel_ + skew_symmetric(
                currentState->q_ * (currentState->imu_data_->gyro_ - currentState->bw_)) * body_p_sensor_);
        Eigen::Quaterniond dq =
                (local_q_global_.conjugate() * currentState->q_ * body_q_sensor_).conjugate() * data->q_;
        Eigen::AngleAxisd dtheta(dq);
        dz.segment(6, 3) = dtheta.axis() * dtheta.angle();

        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 18);
        H.block<3, 3>(0, 0) = local_q_global_.conjugate().toRotationMatrix();
        H.block<3, 3>(0, 6) = -local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix() *
                              skew_symmetric(body_p_sensor_);
        // TODO: change vel jacobi
        H.block<3, 3>(3, 3) = local_q_global_.conjugate().toRotationMatrix();
        auto skew_angular_velocity = skew_symmetric(currentState->imu_data_->gyro_ - currentState->bw_);
        H.block<3, 3>(3, 6) = -local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix() *
                              skew_angular_velocity * skew_symmetric(
                skew_angular_velocity.transpose() * currentState->q_.toRotationMatrix().transpose() * body_p_sensor_);
        H.block<3, 3>(3, 12) = local_q_global_.conjugate().toRotationMatrix() * skew_symmetric(body_p_sensor_) *
                               currentState->q_.toRotationMatrix();
        H.block<3, 3>(6, 6) = body_q_sensor_.toRotationMatrix().transpose();

        // Matrix R
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(9, 9);
        if (is_use_fixed_noise_)
        {
            R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * n_pos_ * n_pos_;
            R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * n_v_ * n_v_;
            R.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * n_q_ * degreeToRadian * n_q_ * degreeToRadian;
        } else
            R = data->cov_;

        // TODO: these code is the same as msf_pose_processor's, which can use virtual and base class
        // update the states and the covariance
        Eigen::MatrixXd S = H * currentState->cov_ * H.transpose() + R;
        Eigen::MatrixXd K = currentState->cov_ * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(18, 18);
        // currentState->cov_ = (I - K * H) * currentState->cov_;
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

    void msf_odom_processor::update(baseStatePtr &currentState, const odomDataPtr &data)
    {
        updateTransformation(currentState, data);
        updateState(currentState, data);
    }

    void msf_odom_processor::transformStateToGlobal(baseStatePtr &state)
    {
        state->pos_in_global_ = local_q_global_.conjugate() * (state->pos_ - local_p_global_);
        state->vel_in_global_ = local_q_global_.conjugate() * state->vel_;
        state->q_in_global_ = local_q_global_.conjugate() * state->q_;
        state->local_p_global_ = local_p_global_;
        state->local_q_global_ = local_q_global_;
    }
}
