//
// Created by antusheng on 9/4/22.
//

#include "msf_gps_processor.h"

#include <utility>

namespace MSF
{
    msf_gps_processor::msf_gps_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor)
            : body_p_sensor_(std::move(body_p_sensor)), body_q_sensor_(body_q_sensor)
    {
        is_use_fixed_noise_ = false;
        has_init_transformation_ = false;
        cov_for_T_bs_ = cov_for_T_lg_ = Eigen::Matrix<double, 6, 6>::Identity();
        cov_for_T_bs_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1 * 0.1;
        cov_for_T_bs_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
        cov_for_T_lg_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 10.0 * 10.0;
        cov_for_T_lg_.block<2, 2>(3, 3) = Eigen::Matrix2d::Identity() * 10.0 * degreeToRadian * 10.0 * degreeToRadian;
        cov_for_T_lg_(5, 5) = 100.0 * degreeToRadian * 100.0 * degreeToRadian;
    }

    void msf_gps_processor::fixNoise(double n_pos)
    {
        is_use_fixed_noise_ = true;
        n_pos_ = n_pos;
    }

    void msf_gps_processor::setInitTransformation(const baseStatePtr &state, const gpsDataPtr &data)
    {
        has_init_transformation_ = true;
        init_lla_ = data->lla_;
        local_p_global_ = state->pos_;
        local_q_global_ = Eigen::Quaterniond::Identity();
    }

    void msf_gps_processor::updateTransformation(baseStatePtr &currentState, const gpsDataPtr &data)
    {
        // update the transformation state
        Eigen::Vector3d pos = convertLlaToENU(init_lla_, data->lla_);
        // residuals
        Eigen::Vector3d dz = currentState->pos_ -
                             (local_q_global_ * pos + local_p_global_ - currentState->q_ * body_p_sensor_);

        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 3) = -local_q_global_.toRotationMatrix() * skew_symmetric(pos);

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

    void msf_gps_processor::updateState(baseStatePtr &currentState, const gpsDataPtr &data)
    {
        // update the Multi-sensor fusion state
        Eigen::Vector3d pos = convertLlaToENU(init_lla_, data->lla_);
        // residuals
        Eigen::Vector3d dz = pos - local_q_global_.conjugate() *
                                          (currentState->q_ * body_p_sensor_ + currentState->pos_ - local_p_global_);

        // update the covariance and the error state
        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 24);
        H.block<3, 3>(0, 0) = local_q_global_.conjugate().toRotationMatrix();
        H.block<3, 3>(0, 6) = -local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix() *
                              skew_symmetric(body_p_sensor_);
        H.block<3, 3>(0, 18) = local_q_global_.conjugate().toRotationMatrix() * currentState->q_.toRotationMatrix();

        // Eigen::Vector3d dz = pos - currentState->pos_;
        // Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 24);
        // H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

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

    void msf_gps_processor::update(baseStatePtr &currentState, const gpsDataPtr &data)
    {
        updateTransformation(currentState, data);
        updateState(currentState, data);
    }

    void msf_gps_processor::transformStateToGlobal(baseStatePtr &state)
    {
        state->pos_in_global_ = local_q_global_.conjugate() * (state->pos_ - local_p_global_);
        state->vel_in_global_ = local_q_global_.conjugate() * state->vel_;
        state->q_in_global_ = local_q_global_.conjugate() * state->q_;
        state->local_p_global_ = local_p_global_;
        state->local_q_global_ = local_q_global_;
    }
}