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
    }

    msf_vio_processor::msf_vio_processor(Eigen::Vector3d imu_p_vio, const Eigen::Quaterniond &imu_q_vio,
                                         bool update_transformation = false)
            : imu_p_vio_(std::move(imu_p_vio)), imu_q_vio_(imu_q_vio), update_transformation_(update_transformation)
    {}

    void msf_vio_processor::updateState(baseStatePtr &currentState, const vioDataPtr &data)
    {
        if (update_transformation_)
        {

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
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 15);
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
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
        // currentState->cov_ = (I - K * H) * currentState->cov_;
        currentState->cov_ = (I - K * H) * currentState->cov_ * (I - K * H).transpose() + K * R * K.transpose();
        Eigen::VectorXd delta_states = K * dz;
        currentState->pos_ += delta_states.segment(0, 3);
        currentState->vel_ += delta_states.segment(3, 3);
        Eigen::Quaterniond delta_q = getQuaternionFromAngle(delta_states.segment(6, 3));
        currentState->q_ *= delta_q;
        currentState->ba_ += delta_states.segment(9, 3);
        currentState->bw_ += delta_states.segment(12, 3);

        // ESKF reset
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(15, 15);
        G.block<3, 3>(6, 6) -= skew_symmetric(0.5 * delta_states.segment(6, 3));
        currentState->cov_ = G * currentState->cov_ * G.transpose();
    }
}