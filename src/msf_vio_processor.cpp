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

#include "msf_vio_processor.h"

#include <utility>

namespace multiSensorFusion
{
    msf_vio_processor::msf_vio_processor()
    {
        imu_p_vio_ = Eigen::Vector3d::Zero();
        imu_q_vio_ = Eigen::Quaterniond::Identity();
        update_transformation_ = false;
        cov_ = Eigen::Matrix<double, 6, 6>::Identity();
        cov_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;
        cov_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0 * degreeToRadian * 1.0 * degreeToRadian;
    }

    msf_vio_processor::msf_vio_processor(Eigen::Vector3d imu_p_vio, const Eigen::Quaterniond &imu_q_vio,
                                         bool update_transformation)
            : imu_p_vio_(std::move(imu_p_vio)), imu_q_vio_(imu_q_vio), update_transformation_(update_transformation)
    {
        cov_ = Eigen::Matrix<double, 6, 6>::Identity();
        cov_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.1 * 0.1;
        cov_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0 * degreeToRadian * 1.0 * degreeToRadian;
    }

    void msf_vio_processor::updateState(baseStatePtr &currentState, const odomDataPtr &data)
    {
        // TODO: update imu and vio/SLAM's coordination transformation
        if (update_transformation_)
        {
            // residuals
            Eigen::VectorXd dz_transform = Eigen::VectorXd::Zero(6);
            dz_transform.segment(0, 3) = currentState->pos_ - (imu_q_vio_ * data->pos_ + imu_p_vio_);
            Eigen::Quaterniond dq_transform = (imu_q_vio_ * data->q_).conjugate() * currentState->q_;
            Eigen::AngleAxisd dtheta_transform(dq_transform);
            dz_transform.segment(3, 3) = dtheta_transform.axis() * dtheta_transform.angle();

            // Matrix H
            Eigen::MatrixXd H_transform = Eigen::MatrixXd::Zero(6, 6);
            H_transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            H_transform.block<3, 3>(0, 3) = -imu_q_vio_.toRotationMatrix() * skew_symmetric(data->pos_);
            H_transform.block<3, 3>(3, 3) = data->q_.toRotationMatrix().transpose();

            // Matrix R
            Eigen::MatrixXd R_transform = Eigen::MatrixXd::Zero(6, 6);
            // R_transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.2 * 0.2;
            R_transform.block<3, 3>(0, 0) = currentState->cov_.block<3, 3>(0, 0);
            // R_transform.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
            R_transform.block<3, 3>(3, 3) = currentState->cov_.block<3, 3>(6, 6);

            // update the transformation from VIO to IMU and the covariance
            Eigen::MatrixXd S_transform = H_transform * cov_ * H_transform.transpose() + R_transform;
            Eigen::MatrixXd K_transform = cov_ * H_transform.transpose() * S_transform.inverse();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
            // cov_ = (I - K_transform * H_transform) * cov_;
            cov_ = (I - K_transform * H_transform) * cov_ * (I - K_transform * H_transform).transpose() +
                   K_transform * R_transform * K_transform.transpose();
            Eigen::VectorXd delta_states_transform = K_transform * dz_transform;
            imu_p_vio_ += delta_states_transform.segment(0, 3);
            Eigen::Quaterniond delta_q_transform = getQuaternionFromAngle(delta_states_transform.segment(3, 3));
            imu_q_vio_ *= delta_q_transform;
            imu_q_vio_.normalized();

            // ESKF reset
            Eigen::MatrixXd G_transform = Eigen::MatrixXd::Identity(6, 6);
            G_transform.block<3, 3>(3, 3) -= skew_symmetric(0.5 * delta_states_transform.segment(3, 3));
            cov_ = G_transform * cov_ * G_transform.transpose();
        }

        // residuals
        Eigen::VectorXd dz = Eigen::VectorXd::Zero(9);
        dz.segment(0, 3) = data->pos_ - imu_q_vio_.conjugate() * (currentState->pos_ - imu_p_vio_);
        dz.segment(3, 3) = data->vel_ - imu_q_vio_.conjugate() * currentState->vel_;
        Eigen::Quaterniond dq = (imu_q_vio_.conjugate() * currentState->q_).conjugate() * data->q_;
        // dz.segment(6, 3) = dq.vec() / dq.w() * 2.0;
        Eigen::AngleAxisd dtheta(dq);
        dz.segment(6, 3) = dtheta.axis() * dtheta.angle();

        // Matrix H
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 18);
        H.block<3, 3>(0, 0) = imu_q_vio_.conjugate().toRotationMatrix();
        H.block<3, 3>(3, 3) = imu_q_vio_.conjugate().toRotationMatrix();
        H.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

        // Matrix R
        Eigen::MatrixXd R = data->cov_;
        // MatrixXd R = MatrixXd::Zero(9, 9);
        // R.block<3, 3>(0, 0) = Matrix3d::Identity() * 0.1 * 0.1;
        // R.block<3, 3>(3, 3) = vioCovariance.block<3, 3>(3, 3);
        // R.block<3, 3>(6, 6) = Matrix3d::Identity() * 2.0 * degreeToRadian * 2.0 * degreeToRadian;

        // TODO: these code is the same as msf_mapLoc_processor's, which can use virtual and base class
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
        currentState->cov_ = (currentState->cov_ + currentState->cov_.transpose()) / 2;
    }
}
