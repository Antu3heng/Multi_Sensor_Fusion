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

#include "msf_mapLoc_processor.h"

namespace multiSensorFusion
{

    msf_mapLoc_processor::msf_mapLoc_processor(const Eigen::Vector3d &imu_p_map, const Eigen::Quaterniond &imu_q_map, double n_pos,
                                               double n_q, bool update_transformation = false)
            : imu_p_map_(imu_p_map), imu_q_map_(imu_q_map), update_transformation_(update_transformation),
              n_pos_(n_pos), n_q_(n_q)
    {
        if (update_transformation_)
        {
            cov_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            cov_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
        }
    }

    void msf_mapLoc_processor::updateState(baseState &currentState, const mapLocData &data)
    {
        if (update_transformation_)
        {
            //  residuals
            Eigen::VectorXd dz_transform = Eigen::VectorXd::Zero(6);
            dz_transform.segment(0, 3) = currentState.pos_ - (imu_q_map_ * data.pos_ + imu_p_map_);
            Eigen::Quaterniond dq_transform = (imu_q_map_ * data.q_).conjugate() * currentState.q_;
            Eigen::AngleAxisd dtheta_transform(dq_transform);
            dz_transform.segment(3, 3) = dtheta_transform.axis() * dtheta_transform.angle();

            // Matrix H
            Eigen::MatrixXd H_transform = Eigen::MatrixXd::Zero(6, 6);
            H_transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            H_transform.block<3, 3>(3, 3) = -data.q_.toRotationMatrix().transpose();

            // Matrix R
            Eigen::MatrixXd R_transform = Eigen::MatrixXd::Zero(6, 6);
            R_transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.2 * 0.2;
            // R_VioTMap.block<3, 3>(0, 0) = currentState.cov_.block<3, 3>(0, 0);
            R_transform.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
            // R_VioTMap.block<3, 3>(3, 3) = currentState.cov_.block<3, 3>(6, 6);

            // update the transformation from VIO to IMU and the covariance
            Eigen::MatrixXd S_transform = H_transform * cov_ * H_transform.transpose() + R_transform;
            Eigen::MatrixXd K_transform = cov_ * H_transform.transpose() * S_transform.inverse();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
            // cov_ = (I - K_transform * H_transform) * cov_;
            cov_ = (I - K_transform * H_transform) * cov_ * (I - K_transform * H_transform).transpose() +
                   K_transform * R_transform * K_transform.transpose();
            Eigen::VectorXd delta_states_transform = K_transform * dz_transform;
            imu_p_map_ += delta_states_transform.segment(0, 3);
            Eigen::Quaterniond delta_q_transform = getQuaternionFromAngle(delta_states_transform.segment(3, 3));
            imu_q_map_ *= delta_q_transform;

            // ESKF reset
            Eigen::MatrixXd G_transform = Eigen::MatrixXd::Identity(6, 6);
            G_transform.block<3, 3>(3, 3) -= skew_symmetric(0.5 * delta_states_transform.segment(3, 3));
            cov_ = G_transform * cov_ * G_transform.transpose();
        }

        // update the Multi-sensor fusion state
        // residuals
        Eigen::VectorXd dz = Eigen::VectorXd::Zero(6);
        dz.segment(0, 3) = data.pos_ - imu_q_map_.conjugate() * (currentState.pos_ - imu_p_map_);
        Eigen::Quaterniond dq = (imu_q_map_.conjugate() * currentState.q_).conjugate() * data.q_;
        Eigen::AngleAxisd dtheta(dq);
        dz.segment(3, 3) = dtheta.axis() * dtheta.angle();

        // update the covariance and the error state
        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 15);
        H.block<3, 3>(0, 0) = imu_q_map_.conjugate().toRotationMatrix();
        H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

        // Matrix R
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
        R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * n_pos_ * n_pos_;
        R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * n_q_ * degreeToRadian * n_q_ * degreeToRadian;

        // TODO: these code is the same as msf_vio_processor's, which can use virtual and base class
        // update the states and the covariance
        Eigen::MatrixXd S = H * currentState.cov_ * H.transpose() + R;
        Eigen::MatrixXd K = currentState.cov_ * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
        // currentState.cov_ = (I - K * H) * currentState.cov_;
        currentState.cov_ = (I - K * H) * currentState.cov_ * (I - K * H).transpose() + K * R * K.transpose();
        Eigen::VectorXd delta_states = K * dz;
        currentState.pos_ += delta_states.segment(0, 3);
        currentState.vel_ += delta_states.segment(3, 3);
        Eigen::Quaterniond delta_q = getQuaternionFromAngle(delta_states.segment(6, 3));
        currentState.q_ *= delta_q;
        currentState.ba_ += delta_states.segment(9, 3);
        currentState.bw_ += delta_states.segment(12, 3);

        // ESKF reset
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(15, 15);
        G.block<3, 3>(6, 6) -= skew_symmetric(0.5 * delta_states.segment(6, 3));
        currentState.cov_ = G * currentState.cov_ * G.transpose();
    }
}